#include "ball_car.h"

#include <stddef.h>
#include <string.h>

static float ball_car_limit(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }
    if(value < -limit)
    {
        return -limit;
    }
    return value;
}

static int16_t ball_car_rpm_to_x10(float rpm)
{
    float scaled = rpm * 10.0f;

    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    if(scaled > 32767.0f)
    {
        scaled = 32767.0f;
    }
    else if(scaled < -32768.0f)
    {
        scaled = -32768.0f;
    }
    return (int16_t)scaled;
}

static float ball_car_x10_to_rpm(int16_t rpm_x10)
{
    return (float)rpm_x10 / 10.0f;
}

static void ball_car_enter_state(
    ball_car_struct *car,
    ball_car_state_enum state)
{
    car->state = state;
    car->state_ticks = 0U;
    car->target_lost_ticks = 0U;
    car->line_confirm_ticks = 0U;
}

static uint8_t ball_car_target_eligible(
    const ball_car_input_struct *input)
{
    return input->vision_link_alive
        && input->vision_valid
        && input->vision_confirmed
        && (input->vision_confidence
            >= BALL_CAR_MIN_CONTROL_CONFIDENCE)
        && (0U != input->vision_frame_width)
        && (0U != input->vision_frame_height);
}

static uint8_t ball_car_target_close(
    const ball_car_input_struct *input)
{
    uint32_t height_percent;

    if(input->vision_close)
    {
        return 1U;
    }
    if(0U == input->vision_frame_height)
    {
        return 0U;
    }

    height_percent = ((uint32_t)input->vision_height * 100U)
                   / input->vision_frame_height;
    return (height_percent >= BALL_CAR_CLOSE_PERCENT);
}

static void ball_car_clear_history(ball_car_struct *car)
{
    car->history_count = 0U;
    car->replay_index = 0U;
    car->history_sample_ticks =
        (uint8_t)(BALL_CAR_HISTORY_SAMPLE_TICKS - 1U);
    car->replay_hold_ticks = 0U;
    car->replay_left_rpm = 0.0f;
    car->replay_right_rpm = 0.0f;
}

static uint8_t ball_car_record_history(
    ball_car_struct *car,
    float left_rpm,
    float right_rpm)
{
    car->history_sample_ticks++;
    if(car->history_sample_ticks < BALL_CAR_HISTORY_SAMPLE_TICKS)
    {
        return 1U;
    }
    car->history_sample_ticks = 0U;

    if(car->history_count >= BALL_CAR_HISTORY_CAPACITY)
    {
        return 0U;
    }

    car->history[car->history_count].left_rpm_x10 =
        ball_car_rpm_to_x10(left_rpm);
    car->history[car->history_count].right_rpm_x10 =
        ball_car_rpm_to_x10(right_rpm);
    car->history_count++;
    return 1U;
}

static void ball_car_prepare_backtrack(ball_car_struct *car)
{
    car->replay_index = car->history_count;
    car->replay_hold_ticks = 0U;
    car->replay_left_rpm = 0.0f;
    car->replay_right_rpm = 0.0f;
    ball_car_enter_state(car, BALL_CAR_STATE_BACKTRACK);
}

static void ball_car_set_fault(
    ball_car_struct *car,
    ball_car_fault_enum fault)
{
    car->fault = fault;
}

static void ball_car_approach_command(
    ball_car_struct *car,
    const ball_car_input_struct *input,
    float *left_rpm,
    float *right_rpm)
{
    int16_t x_error;
    int16_t error_after_deadband;
    uint32_t height_percent;
    float progress;
    float base_rpm;
    float turn_rpm;

    x_error = (int16_t)input->vision_center_x
            - (int16_t)(input->vision_frame_width / 2U);
    car->last_target_error = x_error;

    height_percent = ((uint32_t)input->vision_height * 100U)
                   / input->vision_frame_height;
    progress = (float)height_percent / (float)BALL_CAR_CLOSE_PERCENT;
    if(progress > 1.0f)
    {
        progress = 1.0f;
    }
    base_rpm = BALL_CAR_APPROACH_FAR_RPM
             - progress
               * (BALL_CAR_APPROACH_FAR_RPM
                  - BALL_CAR_APPROACH_NEAR_RPM);

    if((x_error >= -BALL_CAR_APPROACH_X_DEADBAND)
       && (x_error <= BALL_CAR_APPROACH_X_DEADBAND))
    {
        error_after_deadband = 0;
    }
    else if(x_error > 0)
    {
        error_after_deadband =
            (int16_t)(x_error - BALL_CAR_APPROACH_X_DEADBAND);
    }
    else
    {
        error_after_deadband =
            (int16_t)(x_error + BALL_CAR_APPROACH_X_DEADBAND);
    }

    turn_rpm = BALL_CAR_VISION_STEERING_SIGN
             * BALL_CAR_APPROACH_TURN_KP
             * (float)error_after_deadband;
    *left_rpm = ball_car_limit(base_rpm + turn_rpm,
                              BALL_CAR_APPROACH_MAX_RPM);
    *right_rpm = ball_car_limit(base_rpm - turn_rpm,
                               BALL_CAR_APPROACH_MAX_RPM);
}

static void ball_car_target_search_command(
    const ball_car_struct *car,
    float *left_rpm,
    float *right_rpm)
{
    float direction = (car->last_target_error < 0) ? -1.0f : 1.0f;

    *left_rpm = direction * BALL_CAR_TARGET_SEARCH_RPM;
    *right_rpm = -direction * BALL_CAR_TARGET_SEARCH_RPM;
}

static void ball_car_reacquire_command(
    const ball_car_struct *car,
    float *left_rpm,
    float *right_rpm)
{
    uint16_t sweep_index;
    float direction;

    direction = (car->departure_line_error < 0) ? -1.0f : 1.0f;
    if(0 == car->departure_line_error)
    {
        direction = (car->last_target_error < 0) ? -1.0f : 1.0f;
    }

    sweep_index = (uint16_t)(car->state_ticks
                            / BALL_CAR_REACQUIRE_SWEEP_TICKS);
    if(0U != (sweep_index & 1U))
    {
        direction = -direction;
    }

    *left_rpm = direction * BALL_CAR_REACQUIRE_RPM;
    *right_rpm = -direction * BALL_CAR_REACQUIRE_RPM;
}

void ball_car_init(ball_car_struct *car)
{
    if(NULL == car)
    {
        return;
    }

    memset(car, 0, sizeof(*car));
    car->state = BALL_CAR_STATE_IDLE;
    car->fault = BALL_CAR_FAULT_NONE;
    ball_car_clear_history(car);
}

uint8_t ball_car_requires_line_follow(const ball_car_struct *car)
{
    if(NULL == car)
    {
        return 0U;
    }

    return (BALL_CAR_STATE_LINE_FOLLOW == car->state)
        || (BALL_CAR_STATE_TARGET_CONFIRM == car->state)
        || (BALL_CAR_STATE_LINE_FOLLOW_CARRY == car->state);
}

void ball_car_update(
    ball_car_struct *car,
    const ball_car_input_struct *input,
    ball_car_output_struct *output)
{
    uint8_t target_eligible;
    uint8_t history_ok;
    uint16_t line_detection_start;

    if((NULL == car) || (NULL == input) || (NULL == output))
    {
        return;
    }

    output->left_target_rpm = 0.0f;
    output->right_target_rpm = 0.0f;
    output->magnet_on = car->magnet_on;
    output->payload_held = car->payload_held;
    output->state = car->state;
    output->fault = car->fault;

    if(input->release_payload)
    {
        car->payload_held = 0U;
        car->magnet_on = 0U;
        if(BALL_CAR_STATE_LINE_FOLLOW_CARRY == car->state)
        {
            ball_car_enter_state(car, BALL_CAR_STATE_LINE_FOLLOW);
        }
    }

    if(!input->enabled)
    {
        ball_car_enter_state(car, BALL_CAR_STATE_IDLE);
        ball_car_clear_history(car);
        if(!car->payload_held)
        {
            car->magnet_on = 0U;
        }
        output->magnet_on = car->magnet_on;
        output->payload_held = car->payload_held;
        output->state = car->state;
        output->fault = car->fault;
        return;
    }

    if(BALL_CAR_STATE_IDLE == car->state)
    {
        car->fault = BALL_CAR_FAULT_NONE;
        ball_car_clear_history(car);
        ball_car_enter_state(
            car,
            car->payload_held
                ? BALL_CAR_STATE_LINE_FOLLOW_CARRY
                : BALL_CAR_STATE_LINE_FOLLOW);
    }

    if(car->state_ticks < 0xFFFFU)
    {
        car->state_ticks++;
    }
    target_eligible = ball_car_target_eligible(input);

    switch(car->state)
    {
        case BALL_CAR_STATE_LINE_FOLLOW:
            output->left_target_rpm = input->line_left_rpm;
            output->right_target_rpm = input->line_right_rpm;
            if(target_eligible)
            {
                car->target_confirm_ticks = 1U;
                ball_car_enter_state(car, BALL_CAR_STATE_TARGET_CONFIRM);
            }
            break;

        case BALL_CAR_STATE_TARGET_CONFIRM:
            output->left_target_rpm = input->line_left_rpm;
            output->right_target_rpm = input->line_right_rpm;
            if(target_eligible)
            {
                if(car->target_confirm_ticks < 0xFFFFU)
                {
                    car->target_confirm_ticks++;
                }
                if(car->target_confirm_ticks
                   >= BALL_CAR_TARGET_CONFIRM_TICKS)
                {
                    car->departure_line_error = input->line_error;
                    car->last_target_error =
                        (int16_t)input->vision_center_x
                        - (int16_t)(input->vision_frame_width / 2U);
                    ball_car_enter_state(car, BALL_CAR_STATE_STOP_LOCK);
                }
            }
            else
            {
                car->target_confirm_ticks = 0U;
                ball_car_enter_state(car, BALL_CAR_STATE_LINE_FOLLOW);
            }
            break;

        case BALL_CAR_STATE_STOP_LOCK:
            if(target_eligible)
            {
                car->target_lost_ticks = 0U;
            }
            else
            {
                car->target_lost_ticks++;
                if(car->target_lost_ticks
                   > BALL_CAR_TARGET_LOST_GRACE_TICKS)
                {
                    ball_car_enter_state(car, BALL_CAR_STATE_LINE_FOLLOW);
                    break;
                }
            }

            if(car->state_ticks >= BALL_CAR_STOP_LOCK_TICKS)
            {
                ball_car_clear_history(car);
                ball_car_enter_state(car, BALL_CAR_STATE_APPROACH);
            }
            break;

        case BALL_CAR_STATE_APPROACH:
            if(car->state_ticks >= BALL_CAR_APPROACH_TIMEOUT_TICKS)
            {
                ball_car_set_fault(car, BALL_CAR_FAULT_APPROACH_TIMEOUT);
                car->magnet_on = 0U;
                ball_car_prepare_backtrack(car);
                break;
            }

            if(target_eligible)
            {
                car->target_lost_ticks = 0U;
                if(ball_car_target_close(input))
                {
                    car->magnet_on = 1U;
                    ball_car_enter_state(car, BALL_CAR_STATE_FINAL_CREEP);
                    break;
                }

                ball_car_approach_command(
                    car,
                    input,
                    &output->left_target_rpm,
                    &output->right_target_rpm);
            }
            else
            {
                if(car->target_lost_ticks < 0xFFFFU)
                {
                    car->target_lost_ticks++;
                }
                if(car->target_lost_ticks
                   > BALL_CAR_TARGET_LOST_GRACE_TICKS)
                {
                    ball_car_set_fault(car, BALL_CAR_FAULT_VISION_LOST);
                    car->magnet_on = 0U;
                    ball_car_prepare_backtrack(car);
                    break;
                }
                ball_car_target_search_command(
                    car,
                    &output->left_target_rpm,
                    &output->right_target_rpm);
            }

            history_ok = ball_car_record_history(
                car,
                output->left_target_rpm,
                output->right_target_rpm);
            if(!history_ok)
            {
                ball_car_set_fault(car, BALL_CAR_FAULT_HISTORY_OVERFLOW);
                car->magnet_on = 0U;
                ball_car_prepare_backtrack(car);
            }
            break;

        case BALL_CAR_STATE_FINAL_CREEP:
            car->magnet_on = 1U;
            output->left_target_rpm = BALL_CAR_FINAL_CREEP_RPM;
            output->right_target_rpm = BALL_CAR_FINAL_CREEP_RPM;
            history_ok = ball_car_record_history(
                car,
                output->left_target_rpm,
                output->right_target_rpm);
            if(!history_ok)
            {
                ball_car_set_fault(car, BALL_CAR_FAULT_HISTORY_OVERFLOW);
                car->magnet_on = 0U;
                ball_car_prepare_backtrack(car);
            }
            else if(car->state_ticks >= BALL_CAR_FINAL_CREEP_TICKS)
            {
                ball_car_enter_state(car, BALL_CAR_STATE_PICKUP_SETTLE);
            }
            break;

        case BALL_CAR_STATE_PICKUP_SETTLE:
            car->magnet_on = 1U;
            if(car->state_ticks >= BALL_CAR_PICKUP_SETTLE_TICKS)
            {
                /*
                 * No pickup sensor is installed in the current hardware.
                 * The timed electromagnet action is therefore the initial
                 * acceptance criterion.
                 */
                car->payload_held = 1U;
                ball_car_prepare_backtrack(car);
            }
            break;

        case BALL_CAR_STATE_BACKTRACK:
            line_detection_start = (uint16_t)(car->history_count / 5U);
            if((car->replay_index <= line_detection_start)
               && input->line_valid)
            {
                if(car->line_confirm_ticks < 0xFFFFU)
                {
                    car->line_confirm_ticks++;
                }
            }
            else
            {
                car->line_confirm_ticks = 0U;
            }

            if(car->line_confirm_ticks >= BALL_CAR_LINE_CONFIRM_TICKS)
            {
                ball_car_enter_state(
                    car,
                    BALL_CAR_STATE_REACQUIRE_LINE);
                break;
            }

            if(0U == car->replay_hold_ticks)
            {
                if(0U == car->replay_index)
                {
                    ball_car_enter_state(
                        car,
                        BALL_CAR_STATE_REACQUIRE_LINE);
                    break;
                }

                car->replay_index--;
                car->replay_left_rpm =
                    -ball_car_x10_to_rpm(
                        car->history[car->replay_index].left_rpm_x10);
                car->replay_right_rpm =
                    -ball_car_x10_to_rpm(
                        car->history[car->replay_index].right_rpm_x10);
                car->replay_hold_ticks =
                    BALL_CAR_HISTORY_SAMPLE_TICKS;
            }

            output->left_target_rpm = car->replay_left_rpm;
            output->right_target_rpm = car->replay_right_rpm;
            car->replay_hold_ticks--;
            break;

        case BALL_CAR_STATE_REACQUIRE_LINE:
            if(input->line_valid)
            {
                if(car->line_confirm_ticks < 0xFFFFU)
                {
                    car->line_confirm_ticks++;
                }
            }
            else
            {
                car->line_confirm_ticks = 0U;
            }

            if(car->line_confirm_ticks >= BALL_CAR_LINE_CONFIRM_TICKS)
            {
                ball_car_enter_state(
                    car,
                    car->payload_held
                        ? BALL_CAR_STATE_LINE_FOLLOW_CARRY
                        : BALL_CAR_STATE_LINE_FOLLOW);
                break;
            }

            if(car->state_ticks >= BALL_CAR_REACQUIRE_TIMEOUT_TICKS)
            {
                ball_car_set_fault(
                    car,
                    BALL_CAR_FAULT_REACQUIRE_TIMEOUT);
                ball_car_enter_state(car, BALL_CAR_STATE_FAULT);
                break;
            }

            ball_car_reacquire_command(
                car,
                &output->left_target_rpm,
                &output->right_target_rpm);
            break;

        case BALL_CAR_STATE_LINE_FOLLOW_CARRY:
            car->magnet_on = 1U;
            output->left_target_rpm = input->line_left_rpm;
            output->right_target_rpm = input->line_right_rpm;
            break;

        case BALL_CAR_STATE_FAULT:
            break;

        case BALL_CAR_STATE_IDLE:
        default:
            break;
    }

    output->magnet_on = car->magnet_on;
    output->payload_held = car->payload_held;
    output->state = car->state;
    output->fault = car->fault;
}

const char *ball_car_state_name(ball_car_state_enum state)
{
    static const char *const names[] =
    {
        "IDLE",
        "LINE",
        "CONFIRM",
        "LOCK",
        "APPROACH",
        "CREEP",
        "PICKUP",
        "BACKTRACK",
        "REACQUIRE",
        "CARRY",
        "FAULT",
    };

    if((uint32_t)state
       >= (uint32_t)(sizeof(names) / sizeof(names[0])))
    {
        return "UNKNOWN";
    }
    return names[state];
}

const char *ball_car_fault_name(ball_car_fault_enum fault)
{
    static const char *const names[] =
    {
        "NONE",
        "VISION_LOST",
        "APPROACH_TIMEOUT",
        "HISTORY_OVERFLOW",
        "REACQUIRE_TIMEOUT",
    };

    if((uint32_t)fault
       >= (uint32_t)(sizeof(names) / sizeof(names[0])))
    {
        return "UNKNOWN";
    }
    return names[fault];
}
