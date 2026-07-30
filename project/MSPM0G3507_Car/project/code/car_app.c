#include "car_app.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "ball_car.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "angle_pid.h"
#include "car_menu.h"
#include "mpu6050_yaw.h"
#include "odometer.h"
#include "pid_test_serial.h"
#include "speed_pid.h"
#include "tb6612.h"
#include "vision_protocol.h"
#include "vision_uart.h"
#include "wheel_encoder.h"

#define CAR_START_DELAY_TICKS                   (50U)
#define CAR_STATUS_SEND_TICKS                  (20U)
#define CAR_OLED_UPDATE_TICKS                  (50U)
#define CAR_DEBUG_LOG_TICKS                    (200U)
#define CAR_HEARTBEAT_TICKS                    (50U)
#define CAR_PID_TEST_START_DELAY_TICKS          (100U)
#define CAR_RUN_LOG_DIVIDER                       (5U)
#define CAR_RUN_LOG_CAPACITY                    (600U)
#define CAR_YAW_RATE_KP_RPM_PER_DPS            (0.20f)
#define CAR_YAW_RATE_CORRECTION_MAX_RPM         (6.0f)
#define CAR_YAW_RATE_MIN_VALID_DPS               (2.0f)
#define CAR_RAD_TO_DEG                          (57.2957795f)
#define CAR_LINE_TARGET_ACCEL_RPM_PER_TICK       (4.0f)
#define CAR_LINE_TARGET_DECEL_RPM_PER_TICK      (10.0f)
#define CAR_FINISH_LINE_MIN_SENSORS               (4U)
#define CAR_FINISH_LINE_CONFIRM_TICKS             (2U)
#define CAR_FINISH_MIN_DISTANCE_CM              (550.0f)
#define CAR_FINISH_ALIGN_BRAKE_TICKS              (20U)
#define CAR_FINISH_ALIGN_CONFIRM_TICKS             (5U)
#define CAR_FINISH_ALIGN_TIMEOUT_TICKS           (300U)
#define CAR_FINISH_ALIGN_MAX_TRAVEL_CM           (15.0f)
#define CAR_FINISH_ALIGN_REVERSE_RPM             (-18.0f)
#define CAR_FINISH_ALIGN_FORWARD_RPM              (12.0f)
#define CAR_FINISH_ALIGN_YAW_MAX_RPM               (8.0f)
#define CAR_FINISH_ALIGN_YAW_TOLERANCE_DEG          (1.5f)

typedef enum
{
    CAR_LINE_STATE_STRAIGHT = 0,
    CAR_LINE_STATE_CURVE,
    CAR_LINE_STATE_SHARP,
    CAR_LINE_STATE_LOST,
} car_line_state_enum;

typedef enum
{
    CAR_FINISH_ALIGN_IDLE = 0,
    CAR_FINISH_ALIGN_BRAKE,
    CAR_FINISH_ALIGN_BACKUP_CLEAR,
    CAR_FINISH_ALIGN_BACKUP_SEEK,
    CAR_FINISH_ALIGN_FORWARD_SEEK,
    CAR_FINISH_ALIGN_YAW,
} car_finish_align_state_enum;

typedef struct
{
    uint32 timestamp_ms;
    int16 left_target_x10;
    int16 right_target_x10;
    int16 left_actual_x10;
    int16 right_actual_x10;
    int16 left_output;
    int16 right_output;
    int16 line_error;
    int16 yaw_rate_x10;
    int16 yaw_rate_target_x10;
    uint8 line_mask;
    uint8 line_state;
} car_run_log_sample_struct;

typedef struct
{
    uint32 control_tick;
    uint16 start_delay_tick;
    uint16 heartbeat_ticks;
    uint16 status_send_ticks;
    uint16 oled_update_ticks;
    uint16 debug_log_ticks;
    uint8 running_requested;
    uint8 stationary_hold_correcting;
    uint8 pid_test_active;
    uint8 encoder_test_active;
    uint8 pid_test_stall_ticks;
    car_task_t active_task;
    uint32 pid_test_start_tick;
    uint32 pid_test_stop_tick;
    float pid_test_target_rpm;
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float yaw_target;
    float yaw_angle;
    uint32 line_straight_ticks;
    uint32 line_curve_ticks;
    uint32 line_sharp_ticks;
    uint32 line_lost_ticks;
    uint16 line_state_entries[4];
    car_line_state_enum previous_line_state;
    uint8 previous_line_state_valid;
    uint8 run_log_divider;
    uint16 run_log_count;
    uint8 run_log_overflow;
    uint8 run_log_auto_dumped;
    float yaw_rate_target_dps;
    float line_command_left_rpm;
    float line_command_right_rpm;
    uint8 finish_line_armed;
    uint8 finish_line_ticks;
    car_finish_align_state_enum finish_align_state;
    uint16 finish_align_ticks;
    uint8 finish_align_confirm_ticks;

    int32 left_encoder_previous;
    int32 right_encoder_previous;
    speed_pid_struct left_speed_pid;
    speed_pid_struct right_speed_pid;
    line_sensor_data_struct line_sensor;
    line_follow_struct line_follow;
    ball_car_struct ball_car;
    ball_car_output_struct ball_output;
    vision_uart_snapshot_struct vision;
    angle_pid_struct angle_pid;
    odometer_struct odometer;
} car_app_context_struct;

static soft_iic_info_struct car_oled_iic;
static car_app_context_struct car_context;
static car_run_log_sample_struct
    car_run_log[CAR_RUN_LOG_CAPACITY];

static int16 car_rpm_to_x10(float rpm);
static void car_debug_write(const char *text);
static void car_reset_speed_control(void);

static float car_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float car_limit_float(float value, float limit)
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

static float car_slew_float(float current,
                            float target,
                            float accel_step,
                            float decel_step)
{
    if(target > (current + accel_step))
    {
        return current + accel_step;
    }
    if(target < (current - decel_step))
    {
        return current - decel_step;
    }
    return target;
}

static void car_slew_line_targets(float *left_target_rpm,
                                  float *right_target_rpm)
{
    float left_accel_step = CAR_LINE_TARGET_ACCEL_RPM_PER_TICK;
    float right_accel_step = CAR_LINE_TARGET_ACCEL_RPM_PER_TICK;

    if(LINE_FOLLOW_MODE_NORMAL == car_context.line_follow.mode)
    {
        /*
         * A confirmed lost-line search may reverse the inner wheel. Once the
         * line is visible again, leave reverse quickly instead of spending
         * about 0.25 s climbing back at the ordinary acceleration rate.
         */
        if((car_context.line_command_left_rpm < 0.0f)
           && (*left_target_rpm > 0.0f))
        {
            left_accel_step = 12.0f;
        }
        if((car_context.line_command_right_rpm < 0.0f)
           && (*right_target_rpm > 0.0f))
        {
            right_accel_step = 12.0f;
        }
        car_context.line_command_left_rpm = car_slew_float(
            car_context.line_command_left_rpm,
            *left_target_rpm,
            left_accel_step,
            CAR_LINE_TARGET_DECEL_RPM_PER_TICK);
        car_context.line_command_right_rpm = car_slew_float(
            car_context.line_command_right_rpm,
            *right_target_rpm,
            right_accel_step,
            CAR_LINE_TARGET_DECEL_RPM_PER_TICK);
        *left_target_rpm = car_context.line_command_left_rpm;
        *right_target_rpm = car_context.line_command_right_rpm;
    }
    else
    {
        /* Recovery/lost safety targets must take effect immediately. */
        car_context.line_command_left_rpm = *left_target_rpm;
        car_context.line_command_right_rpm = *right_target_rpm;
    }
}

static uint8 car_update_finish_line(void)
{
    char line[80];
    uint8 wide_line =
        (car_context.line_sensor.active_count
         >= CAR_FINISH_LINE_MIN_SENSORS)
        && (0U != (car_context.line_sensor.mask & 0x0FU))
        && (0U != (car_context.line_sensor.mask & 0xF0U));

    if(CAR_FINISH_ALIGN_IDLE != car_context.finish_align_state)
    {
        if(car_context.finish_align_ticks < 65535U)
        {
            car_context.finish_align_ticks++;
        }

        if((car_context.finish_align_ticks
            >= CAR_FINISH_ALIGN_TIMEOUT_TICKS)
           || (odometer_get_cm(&car_context.odometer)
               >= CAR_FINISH_ALIGN_MAX_TRAVEL_CM))
        {
            car_debug_write("@FINISH,2,ALIGN_TIMEOUT\r\n");
            return 1U;
        }

        switch(car_context.finish_align_state)
        {
            case CAR_FINISH_ALIGN_BRAKE:
                if(car_context.finish_align_ticks
                   >= CAR_FINISH_ALIGN_BRAKE_TICKS)
                {
                    car_context.finish_align_ticks = 0U;
                    odometer_reset(&car_context.odometer);
                    car_context.finish_align_state =
                        wide_line
                            ? CAR_FINISH_ALIGN_BACKUP_CLEAR
                            : CAR_FINISH_ALIGN_BACKUP_SEEK;
                }
                break;

            case CAR_FINISH_ALIGN_BACKUP_CLEAR:
                if(!wide_line)
                {
                    car_context.finish_align_state =
                        CAR_FINISH_ALIGN_FORWARD_SEEK;
                    car_context.finish_align_ticks = 0U;
                    odometer_reset(&car_context.odometer);
                }
                break;

            case CAR_FINISH_ALIGN_BACKUP_SEEK:
            case CAR_FINISH_ALIGN_FORWARD_SEEK:
                if(wide_line)
                {
                    if(car_context.finish_align_confirm_ticks < 255U)
                    {
                        car_context.finish_align_confirm_ticks++;
                    }
                    if(car_context.finish_align_confirm_ticks
                       >= CAR_FINISH_LINE_CONFIRM_TICKS)
                    {
                        car_context.finish_align_state =
                            CAR_FINISH_ALIGN_YAW;
                        car_context.finish_align_ticks = 0U;
                        car_context.finish_align_confirm_ticks = 0U;
                        car_reset_speed_control();
                    }
                }
                else
                {
                    car_context.finish_align_confirm_ticks = 0U;
                }
                break;

            case CAR_FINISH_ALIGN_YAW:
                if(wide_line
                   && (car_abs_float(car_context.yaw_angle)
                       <= CAR_FINISH_ALIGN_YAW_TOLERANCE_DEG))
                {
                    if(car_context.finish_align_confirm_ticks < 255U)
                    {
                        car_context.finish_align_confirm_ticks++;
                    }
                    if(car_context.finish_align_confirm_ticks
                       >= CAR_FINISH_ALIGN_CONFIRM_TICKS)
                    {
                        sprintf(line,
                                "@FINISH,2,ALIGNED,yaw=%.1f\r\n",
                                (double)car_context.yaw_angle);
                        car_debug_write(line);
                        return 1U;
                    }
                }
                else
                {
                    car_context.finish_align_confirm_ticks = 0U;
                }
                break;

            default:
                break;
        }
        return 0U;
    }

    if(!car_context.running_requested
       || !((CAR_TASK_PLAIN_LINE == car_context.active_task)
            || (CAR_TASK_LINE_FOLLOW == car_context.active_task)))
    {
        return 0U;
    }

    if(wide_line)
    {
        if(car_context.finish_line_ticks < 255U)
        {
            car_context.finish_line_ticks++;
        }
    }
    else
    {
        car_context.finish_line_ticks = 0U;
    }

    if(car_context.finish_line_ticks
       < CAR_FINISH_LINE_CONFIRM_TICKS)
    {
        return 0U;
    }

    if(!car_context.finish_line_armed)
    {
        car_context.finish_line_armed = 1U;
        car_context.finish_line_ticks = 0U;
        odometer_reset(&car_context.odometer);
        car_debug_write("@FINISH,1,ARMED,distance_cm=0\r\n");
        return 0U;
    }

    if(odometer_get_cm(&car_context.odometer)
       >= CAR_FINISH_MIN_DISTANCE_CM)
    {
        sprintf(line,
                "@FINISH,2,ALIGN_START,distance_cm=%.1f\r\n",
                (double)odometer_get_cm(&car_context.odometer));
        car_debug_write(line);
        car_context.finish_align_state = CAR_FINISH_ALIGN_BRAKE;
        car_context.finish_align_ticks = 0U;
        car_context.finish_align_confirm_ticks = 0U;
        car_context.finish_line_ticks = 0U;
        car_context.yaw_target = 0.0f;
        angle_pid_set_target(&car_context.angle_pid, 0.0f);
        angle_pid_reset(&car_context.angle_pid);
        car_reset_speed_control();
        odometer_reset(&car_context.odometer);
        return 0U;
    }
    return 0U;
}

static void car_get_finish_align_targets(float *left_target_rpm,
                                         float *right_target_rpm)
{
    float base_rpm = 0.0f;
    float yaw_diff_rpm;

    switch(car_context.finish_align_state)
    {
        case CAR_FINISH_ALIGN_BACKUP_CLEAR:
        case CAR_FINISH_ALIGN_BACKUP_SEEK:
            base_rpm = CAR_FINISH_ALIGN_REVERSE_RPM;
            break;

        case CAR_FINISH_ALIGN_FORWARD_SEEK:
            base_rpm = CAR_FINISH_ALIGN_FORWARD_RPM;
            break;

        case CAR_FINISH_ALIGN_YAW:
        case CAR_FINISH_ALIGN_BRAKE:
        default:
            base_rpm = 0.0f;
            break;
    }

    angle_pid_set_target(&car_context.angle_pid, 0.0f);
    yaw_diff_rpm = angle_pid_update(
        &car_context.angle_pid,
        car_context.yaw_angle,
        ANGLE_PID_HOLD_KP,
        ANGLE_PID_HOLD_KI,
        ANGLE_PID_HOLD_KD,
        CAR_FINISH_ALIGN_YAW_MAX_RPM,
        ANGLE_PID_HOLD_INTEGRAL_MAX,
        0.01f);

    *left_target_rpm = base_rpm - yaw_diff_rpm;
    *right_target_rpm = base_rpm + yaw_diff_rpm;
}

static void car_apply_curve_yaw_rate_control(float *left_target_rpm,
                                             float *right_target_rpm)
{
    float target_rate_dps;
    float measured_rate_abs_dps;
    float correction_rpm;

    car_context.yaw_rate_target_dps = 0.0f;
    if(!car_context.line_follow.curve_active
       || (LINE_FOLLOW_MODE_NORMAL != car_context.line_follow.mode)
       || !mpu6050_yaw_is_ready())
    {
        return;
    }

    target_rate_dps =
        (LINE_FOLLOW_CURVE_CENTER_RPM
         * ODO_WHEEL_CIRCUMFERENCE_MM / 60.0f)
        / LINE_FOLLOW_CURVE_RADIUS_MM
        * CAR_RAD_TO_DEG;
    car_context.yaw_rate_target_dps = target_rate_dps;
    measured_rate_abs_dps =
        car_abs_float(mpu6050_yaw_get_rate_dps());

    /* Let wheel-speed feedforward establish the turn before closing rate loop. */
    if(measured_rate_abs_dps < CAR_YAW_RATE_MIN_VALID_DPS)
    {
        return;
    }

    correction_rpm =
        CAR_YAW_RATE_KP_RPM_PER_DPS
        * (target_rate_dps - measured_rate_abs_dps);
    correction_rpm = car_limit_float(
        correction_rpm,
        CAR_YAW_RATE_CORRECTION_MAX_RPM);
    *left_target_rpm +=
        car_context.line_follow.curve_direction * correction_rpm;
    *right_target_rpm -=
        car_context.line_follow.curve_direction * correction_rpm;
}

static car_line_state_enum car_get_line_state(void)
{
    switch(car_context.line_follow.mode)
    {
        case LINE_FOLLOW_MODE_SHARP_LEFT:
        case LINE_FOLLOW_MODE_SHARP_RIGHT:
            return CAR_LINE_STATE_SHARP;

        case LINE_FOLLOW_MODE_LOST_SEARCH:
        case LINE_FOLLOW_MODE_LOST_STOP:
            return CAR_LINE_STATE_LOST;

        case LINE_FOLLOW_MODE_NORMAL:
        default:
            if(car_context.line_follow.curve_active)
            {
                return CAR_LINE_STATE_CURVE;
            }
            return CAR_LINE_STATE_STRAIGHT;
    }
}

static void car_update_line_state_time(uint8 motion_enabled)
{
    car_line_state_enum state;

    if(!motion_enabled
       || !((CAR_TASK_PLAIN_LINE == car_context.active_task)
            || ((CAR_TASK_LINE_FOLLOW == car_context.active_task)
                && ball_car_requires_line_follow(
                    &car_context.ball_car))))
    {
        return;
    }

    state = car_get_line_state();
    if(!car_context.previous_line_state_valid
       || (state != car_context.previous_line_state))
    {
        if(car_context.line_state_entries[state] < 65535U)
        {
            car_context.line_state_entries[state]++;
        }
        car_context.previous_line_state = state;
        car_context.previous_line_state_valid = 1U;
    }

    switch(state)
    {
        case CAR_LINE_STATE_CURVE:
            car_context.line_curve_ticks++;
            break;

        case CAR_LINE_STATE_SHARP:
            car_context.line_sharp_ticks++;
            break;

        case CAR_LINE_STATE_LOST:
            car_context.line_lost_ticks++;
            break;

        case CAR_LINE_STATE_STRAIGHT:
        default:
            car_context.line_straight_ticks++;
            break;
    }
}

static void car_debug_write(const char *text)
{
    uart_write_string(PID_TEST_UART_INDEX, text);
}

static void car_oled_write_command(uint8 command)
{
    uint8 packet[2] = {0x00U, command};
    soft_iic_write_8bit_array(&car_oled_iic, packet, 2);
}

static void car_oled_fill(uint8 pattern)
{
    uint16 index;

    car_oled_write_command(0x21U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0x22U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x07U);

    soft_iic_start(&car_oled_iic);
    soft_iic_send_data(&car_oled_iic, car_oled_iic.addr << 1);
    soft_iic_send_data(&car_oled_iic, 0x40U);
    for(index = 0U; index < 1024U; index++)
    {
        soft_iic_send_data(&car_oled_iic, pattern);
    }
    soft_iic_stop(&car_oled_iic);
}

static void car_oled_show_string(uint8 page, const char *text)
{
    uint8 column = 0U;
    uint8 font_index;
    uint8 font_column;

    car_oled_write_command(0x21U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0x22U);
    car_oled_write_command(page);
    car_oled_write_command(page);

    soft_iic_start(&car_oled_iic);
    soft_iic_send_data(&car_oled_iic, car_oled_iic.addr << 1);
    soft_iic_send_data(&car_oled_iic, 0x40U);

    while(('\0' != *text) && (column <= 121U))
    {
        if((*text < 32) || (*text > 126))
        {
            font_index = 0U;
        }
        else
        {
            font_index = (uint8)(*text - 32);
        }

        for(font_column = 0U; font_column < 6U; font_column++)
        {
            soft_iic_send_data(
                &car_oled_iic,
                ascii_font_6x8[font_index][font_column]);
            column++;
        }
        text++;
    }

    while(column < 128U)
    {
        soft_iic_send_data(&car_oled_iic, 0x00U);
        column++;
    }
    soft_iic_stop(&car_oled_iic);
}

static void car_oled_init(void)
{
    soft_iic_init(&car_oled_iic, 0x3CU, 10, A31, A28);
    system_delay_ms(100);

    car_oled_write_command(0xAEU);
    car_oled_write_command(0xD5U);
    car_oled_write_command(0x80U);
    car_oled_write_command(0xA8U);
    car_oled_write_command(0x3FU);
    car_oled_write_command(0xD3U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x40U);
    car_oled_write_command(0x8DU);
    car_oled_write_command(0x14U);
    car_oled_write_command(0x20U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0xA1U);
    car_oled_write_command(0xC8U);
    car_oled_write_command(0xDAU);
    car_oled_write_command(0x12U);
    car_oled_write_command(0x81U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0xD9U);
    car_oled_write_command(0xF1U);
    car_oled_write_command(0xDBU);
    car_oled_write_command(0x40U);
    car_oled_write_command(0xA4U);
    car_oled_write_command(0xA6U);
    car_oled_write_command(0xAFU);
    car_oled_fill(0x00U);
}

static void car_oled_update_status(uint8 motion_enabled)
{
    char line[24];

    car_oled_show_string(0U, "TI BALL CAR");
    sprintf(line,
            "MODE:%s",
            ball_car_state_name(car_context.ball_output.state));
    car_oled_show_string(2U, line);
    sprintf(line,
            "VIS:%s C:%u",
            car_context.vision.link_alive ? "ON" : "OFF",
            car_context.vision.target.confidence);
    car_oled_show_string(4U, line);
    sprintf(line,
            "%s Y:%+5.1f",
            motion_enabled ? "RUN" : "STOP",
            (double)car_context.yaw_angle);
    car_oled_show_string(6U, line);
}

static void car_reset_speed_control(void)
{
    speed_pid_set_target(&car_context.left_speed_pid, 0.0f);
    speed_pid_set_target(&car_context.right_speed_pid, 0.0f);
    car_context.left_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    car_context.right_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    tb6612_stop_all();
}

static void car_send_line_state_summary(void)
{
    char line[224];
    uint32 total_ticks =
        car_context.line_straight_ticks
        + car_context.line_curve_ticks
        + car_context.line_sharp_ticks
        + car_context.line_lost_ticks;

    if(0U == total_ticks)
    {
        return;
    }

    sprintf(line,
            "@LINESTAT,1,total=%lu,straight=%lu,curve=%lu,"
            "sharp=%lu,lost=%lu,entries=%u/%u/%u/%u,"
            "loss_events=%u,edge_events=%u,edge_max_ms=%u\r\n",
            (unsigned long)(total_ticks * SPEED_PID_BASE_PERIOD_MS),
            (unsigned long)(car_context.line_straight_ticks
                            * SPEED_PID_BASE_PERIOD_MS),
            (unsigned long)(car_context.line_curve_ticks
                            * SPEED_PID_BASE_PERIOD_MS),
            (unsigned long)(car_context.line_sharp_ticks
                            * SPEED_PID_BASE_PERIOD_MS),
            (unsigned long)(car_context.line_lost_ticks
                            * SPEED_PID_BASE_PERIOD_MS),
            car_context.line_state_entries[CAR_LINE_STATE_STRAIGHT],
            car_context.line_state_entries[CAR_LINE_STATE_CURVE],
            car_context.line_state_entries[CAR_LINE_STATE_SHARP],
            car_context.line_state_entries[CAR_LINE_STATE_LOST],
            car_context.line_follow.loss_events,
            car_context.line_follow.edge_events,
            car_context.line_follow.edge_max_ticks
                * SPEED_PID_BASE_PERIOD_MS);
    car_debug_write(line);
}

static void car_run_log_reset(void)
{
    car_context.run_log_divider = 0U;
    car_context.run_log_count = 0U;
    car_context.run_log_overflow = 0U;
    car_context.run_log_auto_dumped = 0U;
}

static void car_run_log_record(float left_target_rpm,
                               float right_target_rpm,
                               int16 left_output,
                               int16 right_output)
{
    car_run_log_sample_struct *sample;

    if(!((CAR_TASK_PLAIN_LINE == car_context.active_task)
         || (CAR_TASK_LINE_FOLLOW == car_context.active_task)))
    {
        return;
    }

    car_context.run_log_divider++;
    if(car_context.run_log_divider < CAR_RUN_LOG_DIVIDER)
    {
        return;
    }
    car_context.run_log_divider = 0U;

    if(car_context.run_log_count >= CAR_RUN_LOG_CAPACITY)
    {
        car_context.run_log_overflow = 1U;
        return;
    }

    sample = &car_run_log[car_context.run_log_count++];
    sample->timestamp_ms =
        car_context.control_tick * SPEED_PID_BASE_PERIOD_MS;
    sample->left_target_x10 = car_rpm_to_x10(left_target_rpm);
    sample->right_target_x10 = car_rpm_to_x10(right_target_rpm);
    sample->left_actual_x10 =
        car_rpm_to_x10(car_context.left_speed_pid.measured_rpm);
    sample->right_actual_x10 =
        car_rpm_to_x10(car_context.right_speed_pid.measured_rpm);
    sample->left_output = left_output;
    sample->right_output = right_output;
    sample->line_error = car_context.line_sensor.error;
    sample->yaw_rate_x10 =
        car_rpm_to_x10(mpu6050_yaw_get_rate_dps());
    sample->yaw_rate_target_x10 =
        car_rpm_to_x10(car_context.yaw_rate_target_dps);
    sample->line_mask = car_context.line_sensor.mask;
    sample->line_state = (uint8)car_get_line_state();
}

static void car_run_log_dump(void)
{
    char line[160];
    uint16 index;
    const car_run_log_sample_struct *sample;

    sprintf(line,
            "@RUNLOG,BEGIN,1,count=%u,overflow=%u,period_ms=%u\r\n",
            car_context.run_log_count,
            car_context.run_log_overflow,
            CAR_RUN_LOG_DIVIDER
                * SPEED_PID_CONTROL_DIVIDER
                * SPEED_PID_BASE_PERIOD_MS);
    car_debug_write(line);

    for(index = 0U; index < car_context.run_log_count; index++)
    {
        sample = &car_run_log[index];
        sprintf(line,
                "@RUNLOG,DATA,3,%u,%lu,%d,%d,%d,%d,%d,%d,"
                "%u,%d,%u,%d,%d\r\n",
                index,
                (unsigned long)sample->timestamp_ms,
                sample->left_target_x10,
                sample->right_target_x10,
                sample->left_actual_x10,
                sample->right_actual_x10,
                sample->left_output,
                sample->right_output,
                sample->line_mask,
                sample->line_error,
                sample->line_state,
                sample->yaw_rate_x10,
                sample->yaw_rate_target_x10);
        car_debug_write(line);
    }
    sprintf(line, "@RUNLOG,END,1,count=%u\r\n",
            car_context.run_log_count);
    car_debug_write(line);
}

static void car_stop_and_open_menu(void)
{
    uint8 had_run_log =
        (0U != car_context.run_log_count);

    /* Stop the motors before any potentially long UART dump. */
    car_context.running_requested = 0U;
    car_reset_speed_control();
    car_send_line_state_summary();
    if(had_run_log && !car_context.run_log_auto_dumped)
    {
        car_run_log_dump();
        car_context.run_log_auto_dumped = 1U;
    }
    car_context.line_straight_ticks = 0U;
    car_context.line_curve_ticks = 0U;
    car_context.line_sharp_ticks = 0U;
    car_context.line_lost_ticks = 0U;
    memset(car_context.line_state_entries,
           0,
           sizeof(car_context.line_state_entries));
    car_context.previous_line_state_valid = 0U;
    car_context.start_delay_tick = 0U;
    car_context.stationary_hold_correcting = 0U;
    car_context.pid_test_active = 0U;
    car_context.encoder_test_active = 0U;
    car_context.pid_test_stall_ticks = 0U;
    car_context.finish_align_state = CAR_FINISH_ALIGN_IDLE;
    car_context.finish_align_ticks = 0U;
    car_context.finish_align_confirm_ticks = 0U;
    car_context.active_task = CAR_TASK_NONE;
    angle_pid_reset(&car_context.angle_pid);
    car_menu_open();
}

static void car_apply_menu_task(car_task_t task)
{
    if(CAR_TASK_NONE == task)
    {
        return;
    }
    /* A local key action always takes authority over a serial PID test. */
    car_context.pid_test_active = 0U;
    car_context.encoder_test_active = 0U;
    if(CAR_TASK_STOP == task)
    {
        car_stop_and_open_menu();
        return;
    }
    if(CAR_TASK_RESET_YAW == task)
    {
        car_stop_and_open_menu();
        car_oled_show_string(0U, "KEEP CAR STILL");
        mpu6050_yaw_calibrate(MPU6500_CALIB_SAMPLES);
        mpu6050_yaw_set_angle(0.0f);
        system_delay_ms(1000U);
        car_menu_open();
        car_menu_render();
        return;
    }
    if(CAR_TASK_ODOMETER_QUERY == task)
    {
        char line[24];
        car_stop_and_open_menu();
        sprintf(line, "ODO %.1f cm",
                (double)odometer_get_cm(&car_context.odometer));
        car_oled_show_string(0U, line);
        system_delay_ms(1000U);
        car_menu_open();
        car_menu_render();
        return;
    }

    if(((CAR_TASK_PLAIN_LINE == task)
        || (CAR_TASK_LINE_FOLLOW == task))
       && !mpu6050_yaw_is_ready())
    {
        car_oled_show_string(0U, "KEEP CAR STILL");
        car_oled_show_string(2U, "CALIBRATING IMU");
        mpu6050_yaw_calibrate(MPU6500_CALIB_SAMPLES);
        mpu6050_yaw_set_angle(0.0f);
        if(!mpu6050_yaw_is_ready())
        {
            car_stop_and_open_menu();
            car_oled_show_string(0U, "IMU CAL FAILED");
            system_delay_ms(1000U);
            car_menu_render();
            return;
        }
    }

    car_context.active_task = task;
    car_context.running_requested = 1U;
    car_context.start_delay_tick = 0U;
    car_context.stationary_hold_correcting = 0U;
    car_context.line_straight_ticks = 0U;
    car_context.line_curve_ticks = 0U;
    car_context.line_sharp_ticks = 0U;
    car_context.line_lost_ticks = 0U;
    memset(car_context.line_state_entries,
           0,
           sizeof(car_context.line_state_entries));
    car_context.previous_line_state_valid = 0U;
    car_context.line_command_left_rpm = 0.0f;
    car_context.line_command_right_rpm = 0.0f;
    car_context.finish_line_armed = 0U;
    car_context.finish_line_ticks = 0U;
    car_context.finish_align_state = CAR_FINISH_ALIGN_IDLE;
    car_context.finish_align_ticks = 0U;
    car_context.finish_align_confirm_ticks = 0U;
    odometer_reset(&car_context.odometer);
    car_run_log_reset();
    car_context.yaw_target = mpu6050_yaw_get_angle();
    if(CAR_TASK_ANGLE_HOLD == task)
    {
        /*
         * Angle Hold is a diagnostic relative-heading test.  Zero the
         * accumulated display/target on entry instead of exposing an old
         * integrated angle as the starting reference.
         */
        mpu6050_yaw_set_angle(0.0f);
        car_context.yaw_target = 0.0f;
        car_context.yaw_angle = 0.0f;
    }
    angle_pid_reset(&car_context.angle_pid);
    angle_pid_set_target(&car_context.angle_pid, car_context.yaw_target);
    ball_car_init(&car_context.ball_car);
    line_follow_init(&car_context.line_follow);
    car_reset_speed_control();
    car_menu_close();
}

static int16 car_rpm_to_x10(float rpm)
{
    float value = rpm * 10.0f;

    value += (value >= 0.0f) ? 0.5f : -0.5f;
    if(value > 32767.0f)
    {
        value = 32767.0f;
    }
    else if(value < -32768.0f)
    {
        value = -32768.0f;
    }
    return (int16)value;
}

static void car_send_vision_status(
    uint8 motion_enabled,
    float left_target_rpm,
    float right_target_rpm)
{
    vision_car_status_struct status;

    status.state = (uint8_t)car_context.ball_output.state;
    status.flags = 0U;
    if(motion_enabled)
    {
        status.flags |= VISION_STATUS_FLAG_ENABLED;
    }
    if(BALL_CAR_STATE_FAULT == car_context.ball_output.state)
    {
        status.flags |= VISION_STATUS_FLAG_FAULT;
    }
    status.fault = (uint8_t)car_context.ball_output.fault;
    status.line_mask = car_context.line_sensor.mask;
    status.left_rpm_x10 = car_rpm_to_x10(left_target_rpm);
    status.right_rpm_x10 = car_rpm_to_x10(right_target_rpm);
    vision_uart_send_status(&status);
}

static void car_update_motor_control(
    float left_target_rpm,
    float right_target_rpm)
{
    int32 left_encoder_count;
    int32 right_encoder_count;
    int32 left_encoder_delta;
    int32 right_encoder_delta;
    int16 left_duty;
    int16 right_duty;
    uint8 left_stalled;
    uint8 right_stalled;

    speed_pid_set_target(
        &car_context.left_speed_pid,
        left_target_rpm);
    speed_pid_set_target(
        &car_context.right_speed_pid,
        right_target_rpm);

    if(0U != (car_context.control_tick
              % SPEED_PID_CONTROL_DIVIDER))
    {
        return;
    }

    left_encoder_count =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    right_encoder_count =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    left_encoder_delta =
        SPEED_PID_MOTOR1_ENCODER_SIGN
        * (left_encoder_count
           - car_context.left_encoder_previous);
    right_encoder_delta =
        SPEED_PID_MOTOR2_ENCODER_SIGN
        * (right_encoder_count
           - car_context.right_encoder_previous);
    car_context.left_encoder_previous = left_encoder_count;
    car_context.right_encoder_previous = right_encoder_count;
    odometer_update(&car_context.odometer,
                    left_encoder_delta,
                    right_encoder_delta);

    left_duty = speed_pid_update(
        &car_context.left_speed_pid,
        left_encoder_delta,
        car_context.speed_kp,
        car_context.speed_ki,
        car_context.speed_kd);
    right_duty = speed_pid_update(
        &car_context.right_speed_pid,
        right_encoder_delta,
        car_context.speed_kp,
        car_context.speed_ki,
        car_context.speed_kd);

    left_stalled =
        ((left_duty >= 7500) || (left_duty <= -7500))
        && (car_context.left_speed_pid.measured_rpm > -1.0f)
        && (car_context.left_speed_pid.measured_rpm < 1.0f);
    right_stalled =
        ((right_duty >= 7500) || (right_duty <= -7500))
        && (car_context.right_speed_pid.measured_rpm > -1.0f)
        && (car_context.right_speed_pid.measured_rpm < 1.0f);
    if(car_context.pid_test_active
       && (left_stalled || right_stalled))
    {
        if(car_context.pid_test_stall_ticks < 50U)
        {
            car_context.pid_test_stall_ticks++;
        }
    }
    else
    {
        car_context.pid_test_stall_ticks = 0U;
    }
    if(car_context.pid_test_stall_ticks >= 50U)
    {
        car_stop_and_open_menu();
        pid_test_serial_send_ack("STOPPED", "encoder_stall");
        return;
    }

    /* Right wheel = TB6612 A; left wheel = TB6612 B. */
    tb6612_set_motor(
        TB6612_MOTOR_A,
        SPEED_PID_TB6612_A_FORWARD_SIGN * right_duty);
    tb6612_set_motor(
        TB6612_MOTOR_B,
        SPEED_PID_TB6612_B_FORWARD_SIGN * left_duty);

    if(car_context.pid_test_active)
    {
        pid_test_serial_send_sample(
            car_context.control_tick * SPEED_PID_BASE_PERIOD_MS,
            left_target_rpm,
            right_target_rpm,
            car_context.left_speed_pid.measured_rpm,
            car_context.right_speed_pid.measured_rpm,
            left_duty,
            right_duty,
            (uint8)(0.0f != left_target_rpm
                    || 0.0f != right_target_rpm),
            car_context.line_sensor.mask,
            car_context.line_sensor.error,
            (uint8)car_get_line_state(),
            car_context.line_straight_ticks
                * SPEED_PID_BASE_PERIOD_MS,
            car_context.line_curve_ticks
                * SPEED_PID_BASE_PERIOD_MS,
            car_context.line_sharp_ticks
                * SPEED_PID_BASE_PERIOD_MS,
            car_context.line_lost_ticks
                * SPEED_PID_BASE_PERIOD_MS);
    }
    car_run_log_record(left_target_rpm,
                       right_target_rpm,
                       left_duty,
                       right_duty);
}

static void car_debug_log(
    uint8 motion_enabled,
    float left_target_rpm,
    float right_target_rpm)
{
    char log[256];

    sprintf(
        log,
        "CAR run=%u state=%s fault=%s line=%02X err=%d "
        "vision=%u target=%u conf=%u xy=%u,%u wh=%u,%u "
        "cmd[L=%d R=%d] rpm100[L=%ld R=%ld] yaw=%+.1f "
        "odo=%.1f packets=%lu crc=%lu ovf=%lu\r\n",
        motion_enabled,
        ball_car_state_name(car_context.ball_output.state),
        ball_car_fault_name(car_context.ball_output.fault),
        car_context.line_sensor.mask,
        car_context.line_sensor.error,
        car_context.vision.link_alive,
        car_context.vision.target_fresh,
        car_context.vision.target.confidence,
        car_context.vision.target.center_x,
        car_context.vision.target.center_y,
        car_context.vision.target.width,
        car_context.vision.target.height,
        (int)left_target_rpm,
        (int)right_target_rpm,
        (long)(car_context.left_speed_pid.measured_rpm * 100.0f),
        (long)(car_context.right_speed_pid.measured_rpm * 100.0f),
        (double)car_context.yaw_angle,
        (double)odometer_get_cm(&car_context.odometer),
        (unsigned long)car_context.vision.packet_count,
        (unsigned long)car_context.vision.crc_error_count,
        (unsigned long)car_context.vision.rx_overflow_count);
    car_debug_write(log);
}

void car_app_init(void)
{
    memset(&car_context, 0, sizeof(car_context));
    car_context.speed_kp = SPEED_PID_MOTOR1_KP;
    car_context.speed_ki = SPEED_PID_MOTOR1_KI;
    car_context.speed_kd = SPEED_PID_MOTOR1_KD;

    uart_init(PID_TEST_UART_INDEX,
              PID_TEST_UART_BAUDRATE,
              PID_TEST_UART_TX_PIN,
              PID_TEST_UART_RX_PIN);
    pid_test_serial_init();
    vision_uart_init();

    gpio_init(B16, GPO, 0U, GPO_PUSH_PULL);
    gpio_init(A7, GPO, 1U, GPO_PUSH_PULL);

    tb6612_init();
    wheel_encoder_init();
    speed_pid_init(&car_context.left_speed_pid);
    speed_pid_init(&car_context.right_speed_pid);
    line_sensor_init();
    line_follow_init(&car_context.line_follow);
    ball_car_init(&car_context.ball_car);
    angle_pid_init(&car_context.angle_pid);
    odometer_init(&car_context.odometer);
    mpu6050_yaw_init();
    car_oled_init();
    car_menu_init(car_oled_fill, car_oled_show_string);

    gpio_set_level(A7, 0U);
    system_delay_ms(200);
    gpio_set_level(A7, 1U);

    car_context.left_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    car_context.right_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    car_context.ball_output.state = BALL_CAR_STATE_IDLE;

    car_menu_render();
    car_debug_write("MSPM0 integrated controller ready.\r\n");
    car_debug_write(
        "Vision UART2=115200; control uses OLED four-key menu.\r\n");
}

void car_app_run(void)
{
    car_task_t requested_task;
    pid_test_command_struct pid_test_command;
    ball_car_input_struct input;
    ball_car_state_enum previous_state;
    uint8 motion_enabled;
    float line_left_rpm;
    float line_right_rpm;
    float left_target_rpm;
    float right_target_rpm;
    float angle_diff_rpm;
    float hold_error;
    char transition_log[128];

    while(1)
    {
        system_delay_ms(SPEED_PID_BASE_PERIOD_MS);
        car_context.control_tick++;

        requested_task = car_menu_update();
        car_apply_menu_task(requested_task);
        pid_test_serial_process(&pid_test_command);
        if(pid_test_command.gains_requested)
        {
            char gains_detail[80];
            if(car_context.pid_test_active)
            {
                pid_test_serial_send_ack("ERROR", "stop_before_gains");
            }
            else
            {
                car_context.speed_kp = pid_test_command.kp;
                car_context.speed_ki = pid_test_command.ki;
                car_context.speed_kd = pid_test_command.kd;
                speed_pid_reset(&car_context.left_speed_pid);
                speed_pid_reset(&car_context.right_speed_pid);
                sprintf(gains_detail,
                        "kp=%.3f,ki=%.3f,kd=%.3f",
                        (double)car_context.speed_kp,
                        (double)car_context.speed_ki,
                        (double)car_context.speed_kd);
                pid_test_serial_send_ack("GAINS", gains_detail);
            }
        }
        if(pid_test_command.stop_requested)
        {
            car_stop_and_open_menu();
            pid_test_serial_send_ack("STOPPED", "remote");
        }
        if(pid_test_command.encoder_requested)
        {
            car_stop_and_open_menu();
            car_context.encoder_test_active = 1U;
            wheel_encoder_clear_all();
            car_context.left_encoder_previous = 0;
            car_context.right_encoder_previous = 0;
            pid_test_serial_send_ack("ENCODER", "motor_output_disabled");
        }
        if(pid_test_command.dump_requested)
        {
            if(car_context.running_requested
               || car_context.pid_test_active)
            {
                pid_test_serial_send_ack("ERROR", "stop_before_dump");
            }
            else
            {
                car_run_log_dump();
            }
        }
        if(pid_test_command.start_requested)
        {
            car_stop_and_open_menu();
            car_context.pid_test_active = 1U;
            car_context.pid_test_target_rpm =
                pid_test_command.target_rpm;
            car_context.pid_test_start_tick =
                car_context.control_tick;
            car_context.pid_test_stop_tick =
                car_context.control_tick
                + (pid_test_command.duration_ms
                   / SPEED_PID_BASE_PERIOD_MS);
            car_menu_close();
            pid_test_serial_send_ack("STARTED", "safety_timer_armed");
        }
        if(car_context.pid_test_active
           && (car_context.control_tick
               >= car_context.pid_test_stop_tick))
        {
            car_stop_and_open_menu();
            pid_test_serial_send_ack("STOPPED", "board_timeout");
        }
        if(car_context.encoder_test_active
           && (0U == (car_context.control_tick % 10U)))
        {
            pid_test_serial_send_encoder(
                car_context.control_tick * SPEED_PID_BASE_PERIOD_MS,
                wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1),
                wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR1, 0U),
                wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR1, 1U),
                wheel_encoder_get_state(WHEEL_ENCODER_MOTOR1),
                wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2),
                wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR2, 0U),
                wheel_encoder_get_edge_count(WHEEL_ENCODER_MOTOR2, 1U),
                wheel_encoder_get_state(WHEEL_ENCODER_MOTOR2));
        }

        vision_uart_process(car_context.control_tick);
        vision_uart_get_snapshot(
            car_context.control_tick,
            &car_context.vision);
        line_sensor_read(&car_context.line_sensor);
        if(car_context.running_requested
           && (CAR_FINISH_ALIGN_IDLE
               == car_context.finish_align_state)
           && (CAR_TASK_ANGLE_HOLD != car_context.active_task))
        {
            mpu6050_yaw_update_fast();
        }
        else
        {
            mpu6050_yaw_update();
        }
        car_context.yaw_angle = mpu6050_yaw_get_angle();

        if(car_update_finish_line())
        {
            car_stop_and_open_menu();
            continue;
        }

        if(car_context.running_requested
           && (car_context.start_delay_tick
               < CAR_START_DELAY_TICKS))
        {
            car_context.start_delay_tick++;
        }
        motion_enabled =
            car_context.running_requested
            && (car_context.start_delay_tick
                >= CAR_START_DELAY_TICKS);
        if(car_context.pid_test_active)
        {
            motion_enabled =
                (car_context.control_tick
                 >= (car_context.pid_test_start_tick
                     + CAR_PID_TEST_START_DELAY_TICKS));
        }

        line_left_rpm = 0.0f;
        line_right_rpm = 0.0f;
        if(motion_enabled
           && (CAR_FINISH_ALIGN_IDLE
               == car_context.finish_align_state)
           && (CAR_TASK_ANGLE_HOLD != car_context.active_task)
           && ((CAR_TASK_PLAIN_LINE == car_context.active_task)
               || ball_car_requires_line_follow(
                   &car_context.ball_car)))
        {
            line_follow_update(
                &car_context.line_follow,
                &car_context.line_sensor,
                mpu6050_yaw_get_total_angle(),
                mpu6050_yaw_get_rate_dps(),
                mpu6050_yaw_is_ready(),
                odometer_get_cm(&car_context.odometer),
                &line_left_rpm,
                &line_right_rpm);
            car_apply_curve_yaw_rate_control(
                &line_left_rpm,
                &line_right_rpm);
            car_slew_line_targets(
                &line_left_rpm,
                &line_right_rpm);

            /* Permanent line loss — stop the mission. */
            if(LINE_FOLLOW_MODE_LOST_STOP
               == car_context.line_follow.mode)
            {
                car_context.running_requested = 0U;
                car_reset_speed_control();
                car_send_line_state_summary();
                if((0U != car_context.run_log_count)
                   && !car_context.run_log_auto_dumped)
                {
                    car_run_log_dump();
                    car_context.run_log_auto_dumped = 1U;
                }
                car_context.active_task = CAR_TASK_NONE;
                car_menu_open();
                continue;
            }
        }
        car_update_line_state_time(motion_enabled);

        memset(&input, 0, sizeof(input));
        input.enabled = motion_enabled;
        input.line_valid = car_context.line_sensor.line_valid;
        input.line_error = car_context.line_sensor.error;
        input.line_left_rpm = line_left_rpm;
        input.line_right_rpm = line_right_rpm;
        input.vision_link_alive = car_context.vision.link_alive;
        input.vision_valid = car_context.vision.target_fresh;
        input.vision_confirmed =
            (0U != (car_context.vision.target.flags
                    & VISION_TARGET_FLAG_CONFIRMED));
        input.vision_close =
            (0U != (car_context.vision.target.flags
                    & VISION_TARGET_FLAG_CLOSE));
        input.vision_confidence =
            car_context.vision.target.confidence;
        input.vision_center_x =
            car_context.vision.target.center_x;
        input.vision_center_y =
            car_context.vision.target.center_y;
        input.vision_width = car_context.vision.target.width;
        input.vision_height = car_context.vision.target.height;
        input.vision_frame_width =
            car_context.vision.target.frame_width;
        input.vision_frame_height =
            car_context.vision.target.frame_height;

        previous_state = car_context.ball_car.state;
        if(CAR_TASK_LINE_FOLLOW == car_context.active_task)
        {
            ball_car_update(
                &car_context.ball_car,
                &input,
                &car_context.ball_output);
        }
        else
        {
            car_context.ball_output.left_target_rpm = line_left_rpm;
            car_context.ball_output.right_target_rpm = line_right_rpm;
            car_context.ball_output.state = motion_enabled
                ? BALL_CAR_STATE_LINE_FOLLOW
                : BALL_CAR_STATE_IDLE;
            car_context.ball_output.fault = BALL_CAR_FAULT_NONE;
        }

        if((previous_state != car_context.ball_car.state)
           && (BALL_CAR_STATE_LINE_FOLLOW
               == car_context.ball_car.state))
        {
            line_follow_init(&car_context.line_follow);
        }

        left_target_rpm =
            motion_enabled
                ? car_context.ball_output.left_target_rpm
                : 0.0f;
        right_target_rpm =
            motion_enabled
                ? car_context.ball_output.right_target_rpm
                : 0.0f;
        if(car_context.pid_test_active)
        {
            left_target_rpm = motion_enabled
                ? car_context.pid_test_target_rpm : 0.0f;
            right_target_rpm = motion_enabled
                ? car_context.pid_test_target_rpm : 0.0f;
        }
        if(motion_enabled
           && (CAR_TASK_ANGLE_HOLD == car_context.active_task))
        {
            angle_pid_set_target(&car_context.angle_pid,
                                 car_context.yaw_target);
            angle_diff_rpm = angle_pid_update(
                &car_context.angle_pid,
                car_context.yaw_angle,
                ANGLE_PID_HOLD_KP,
                ANGLE_PID_HOLD_KI,
                ANGLE_PID_HOLD_KD,
                ANGLE_PID_HOLD_OUTPUT_MAX,
                ANGLE_PID_HOLD_INTEGRAL_MAX,
                0.01f);
            hold_error = car_context.angle_pid.error;
            if(!car_context.stationary_hold_correcting
               && ((hold_error <= -ANGLE_PID_HOLD_ENTER_DEG)
                   || (hold_error >= ANGLE_PID_HOLD_ENTER_DEG)))
            {
                car_context.stationary_hold_correcting = 1U;
            }
            else if(car_context.stationary_hold_correcting
                    && (hold_error > -ANGLE_PID_HOLD_EXIT_DEG)
                    && (hold_error < ANGLE_PID_HOLD_EXIT_DEG))
            {
                car_context.stationary_hold_correcting = 0U;
                angle_pid_reset(&car_context.angle_pid);
            }
            if(!car_context.stationary_hold_correcting)
            {
                angle_diff_rpm = 0.0f;
            }
            left_target_rpm = -angle_diff_rpm;
            right_target_rpm = angle_diff_rpm;
            speed_pid_set_start_duty(&car_context.left_speed_pid,
                                     SPEED_PID_HOLD_START_DUTY);
            speed_pid_set_start_duty(&car_context.right_speed_pid,
                                     SPEED_PID_HOLD_START_DUTY);
        }
        else
        {
            speed_pid_set_start_duty(&car_context.left_speed_pid,
                                     SPEED_PID_START_DUTY);
            speed_pid_set_start_duty(&car_context.right_speed_pid,
                                     SPEED_PID_START_DUTY);
        }
        if(CAR_FINISH_ALIGN_IDLE != car_context.finish_align_state)
        {
            car_get_finish_align_targets(&left_target_rpm,
                                         &right_target_rpm);
            if(CAR_FINISH_ALIGN_YAW
               == car_context.finish_align_state)
            {
                speed_pid_set_start_duty(
                    &car_context.left_speed_pid,
                    SPEED_PID_HOLD_START_DUTY);
                speed_pid_set_start_duty(
                    &car_context.right_speed_pid,
                    SPEED_PID_HOLD_START_DUTY);
            }
        }
        car_update_motor_control(
            left_target_rpm,
            right_target_rpm);

        if((CAR_TASK_LINE_FOLLOW == car_context.active_task)
           && (previous_state != car_context.ball_output.state))
        {
            sprintf(
                transition_log,
                "STATE %s -> %s, fault=%s\r\n",
                ball_car_state_name(previous_state),
                ball_car_state_name(
                    car_context.ball_output.state),
                ball_car_fault_name(
                    car_context.ball_output.fault));
            car_debug_write(transition_log);
        }

        if((CAR_TASK_LINE_FOLLOW == car_context.active_task)
           && (BALL_CAR_STATE_COMPLETE
               == car_context.ball_output.state))
        {
            car_stop_and_open_menu();
        }

        car_context.status_send_ticks++;
        if(car_context.status_send_ticks
           >= CAR_STATUS_SEND_TICKS)
        {
            car_context.status_send_ticks = 0U;
            car_send_vision_status(
                motion_enabled,
                left_target_rpm,
                right_target_rpm);
        }

        car_context.heartbeat_ticks++;
        if(car_context.heartbeat_ticks >= CAR_HEARTBEAT_TICKS)
        {
            car_context.heartbeat_ticks = 0U;
            gpio_toggle_level(B16);
        }

        car_context.oled_update_ticks++;
        if((car_context.oled_update_ticks
            >= CAR_OLED_UPDATE_TICKS)
           && !car_menu_is_open())
        {
            car_context.oled_update_ticks = 0U;
            car_oled_update_status(motion_enabled);
        }

        car_context.debug_log_ticks++;
        if(car_context.debug_log_ticks
           >= CAR_DEBUG_LOG_TICKS)
        {
            car_context.debug_log_ticks = 0U;
            car_debug_log(
                motion_enabled,
                left_target_rpm,
                right_target_rpm);
        }
    }
}
