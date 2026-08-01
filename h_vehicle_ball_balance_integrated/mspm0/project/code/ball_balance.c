#include "ball_balance.h"

#include <stddef.h>
#include <string.h>

#define TICK_S (0.010f)
#define TASK3_VISION_GRACE_TICKS (100U) /* Hold level for 1 s before a real vision fault. */

/*
 * TASK3 source: maix-h-task3-round-trip-bench-v3.4.5,
 * profile task3_completion_priority_v7.  Task 3 has no scoring-time fault:
 * the settle controllers may restart the ball as many times as necessary.
 * TASK4/5 source: maix-h_task4_center_hold_bench-v4.2.5-in-spec-latch,
 * profile task4_in_spec_latch_v11.
 */

static float absf_local(float v) { return v < 0.0f ? -v : v; }
static float clampf_local(float v, float low, float high)
{ return v < low ? low : (v > high ? high : v); }

static int16_t angle_to_pulses(float angle)
{
    float magnitude, pulses;
    angle = clampf_local(angle, -4.0f, 4.0f);
    magnitude = absf_local(angle);
    if(magnitude < 0.025f) return 0;
    if(magnitude <= 1.0f) pulses = 80.0f * magnitude;
    else if(magnitude <= 3.0f) pulses = 80.0f + 40.0f * (magnitude - 1.0f);
    else pulses = 160.0f + 40.0f * (magnitude - 3.0f);
    pulses += 0.5f;
    return (int16_t)(angle > 0.0f ? pulses : -pulses);
}

static void enter_state(ball_balance_t *c, ball_balance_state_t state,
                        uint32_t tick)
{
    c->state = state; c->state_tick = tick; c->stable_tick = 0U;
    c->stiction_tick = 0U; c->stiction_active = 0U;
    c->quiet_hold_active = 0U;
}

static float settle_angle(ball_balance_t *c, float target, float position,
                          float velocity, float kp, float kd,
                          float max_angle, float positive_min,
                          float negative_min, uint32_t trigger_ticks)
{
    float error = target - position;
    float angle = clampf_local(kp * error - kd * velocity,
                               -max_angle, max_angle);
    uint8_t eligible = absf_local(error) >= 8.0f
        && absf_local(velocity) <= 8.0f && angle * error > 0.0f;
    if(c->stiction_active)
    {
        if(!eligible || (velocity * error > 0.0f && absf_local(velocity) >= 4.0f))
        { c->stiction_active = 0U; c->stiction_tick = 0U; }
        else c->stiction_tick++;
    }
    else if(eligible)
    {
        c->stiction_tick++;
        if(c->stiction_tick >= trigger_ticks) c->stiction_active = 1U;
    }
    else c->stiction_tick = 0U;
    if(c->stiction_active)
    {
        float ramp = 2.5f * TICK_S * (float)(c->stiction_tick - trigger_ticks);
        if(error > 0.0f)
            angle = angle > positive_min + ramp ? angle
                : clampf_local(positive_min + ramp, 0.0f, max_angle);
        else
        {
            float minimum = clampf_local(negative_min + ramp, 0.0f, max_angle);
            angle = angle < -minimum ? angle : -minimum;
        }
    }
    return angle;
}

static void update_task3(ball_balance_t *c, float x, float v,
                         uint8_t valid, uint32_t tick)
{
    uint32_t phase = tick - c->state_tick;
    float error;
    if(BALL_STATE_WAIT_BALL == c->state)
    {
        if(valid && absf_local(x) <= 8.0f && absf_local(v) <= 20.0f)
        {
            if(++c->stable_tick >= 35U)
            { c->start_tick = tick; enter_state(c, BALL_STATE_HOLD_CENTER, tick); }
        }
        else c->stable_tick = 0U;
        c->requested_angle_deg = 0.0f;
        return;
    }
    /*
     * Completion has priority over the five-second scoring target.  A missed
     * scoring band is not a fault; POS_SETTLE/NEG_SETTLE keep correcting and
     * their stiction boost naturally provides second or later breakaway runs.
     * A short vision dropout levels the beam instead of aborting the attempt.
     */
    if(!valid)
    {
        if(c->invalid_tick < TASK3_VISION_GRACE_TICKS) c->invalid_tick++;
        if(c->invalid_tick >= TASK3_VISION_GRACE_TICKS)
        {
            c->fault_code = 1U;
            enter_state(c, BALL_STATE_SAFE_STOP, tick);
        }
        c->requested_angle_deg = 0.0f;
        return;
    }
    c->invalid_tick = 0U;

    /* Keep only the physical end-stop safety boundary, not a scoring timeout. */
    if(absf_local(x) >= 105.0f)
    {
        c->fault_code = 2U;
        enter_state(c, BALL_STATE_SAFE_STOP, tick);
    }
    if(BALL_STATE_SAFE_STOP == c->state) { c->requested_angle_deg = 0.0f; return; }

    switch(c->state)
    {
        case BALL_STATE_HOLD_CENTER:
            if(phase >= 25U) enter_state(c, BALL_STATE_POS_PUSH, tick);
            break;
        case BALL_STATE_POS_PUSH:
            if((phase >= 15U && ((x >= 0.5f && v >= 5.0f) || v >= 30.0f))
               || phase >= 65U) enter_state(c, BALL_STATE_POS_BRAKE, tick);
            break;
        case BALL_STATE_POS_BRAKE:
            if((phase >= 20U && x >= 42.0f && absf_local(v) <= 30.0f)
               || phase >= 28U) enter_state(c, BALL_STATE_POS_SETTLE, tick);
            break;
        case BALL_STATE_POS_SETTLE:
            if(absf_local(x - 50.0f) <= 10.0f)
                enter_state(c, BALL_STATE_NEG_PUSH, tick);
            break;
        case BALL_STATE_NEG_PUSH:
            if((phase >= 30U && (x <= 12.0f || v <= -100.0f)) || phase >= 125U)
                enter_state(c, BALL_STATE_NEG_BRAKE, tick);
            break;
        case BALL_STATE_NEG_BRAKE:
            if(x <= -40.0f && absf_local(v) <= 60.0f)
                enter_state(c, BALL_STATE_NEG_SETTLE, tick);
            break;
        case BALL_STATE_NEG_SETTLE:
            if(absf_local(x + 50.0f) <= 9.0f && absf_local(v) <= 12.0f)
            {
                if(++c->stable_tick >= 20U) enter_state(c, BALL_STATE_COMPLETE, tick);
            }
            else c->stable_tick = 0U;
            break;
        default: break;
    }

    switch(c->state)
    {
        case BALL_STATE_POS_PUSH: c->requested_angle_deg = 3.0f; break;
        case BALL_STATE_POS_BRAKE: c->requested_angle_deg = -2.2f; break;
        case BALL_STATE_POS_SETTLE:
            if(x < 40.0f && v <= -2.0f) c->requested_angle_deg = 3.0f;
            else if(v >= 90.0f || (x >= 38.0f && v >= 10.0f))
                c->requested_angle_deg = -3.0f;
            else c->requested_angle_deg = settle_angle(c, 50.0f, x, v,
                    0.045f, absf_local(v) >= 95.0f ? 0.024f : 0.018f,
                    3.0f, 2.2f, 2.6f, 12U);
            break;
        case BALL_STATE_NEG_PUSH:
            c->requested_angle_deg = v > 20.0f ? -3.0f : -1.6f;
            break;
        case BALL_STATE_NEG_BRAKE:
            c->requested_angle_deg = settle_angle(c, -50.0f, x, v,
                    0.045f, absf_local(v) >= 95.0f ? 0.024f : 0.018f,
                    3.0f, 2.2f, 2.6f, 12U);
            break;
        case BALL_STATE_NEG_SETTLE:
        case BALL_STATE_COMPLETE:
            error = -50.0f - x;
            if(c->quiet_hold_active)
            {
                if(absf_local(error) >= 10.0f || v <= -75.0f || v >= 15.0f)
                    c->quiet_hold_active = 0U;
            }
            else if(absf_local(error) <= 9.0f && v >= -60.0f && v <= 12.0f)
                c->quiet_hold_active = 1U;
            if(c->quiet_hold_active)
                c->requested_angle_deg = v <= -30.0f ? 0.30f : -0.20f;
            else c->requested_angle_deg = settle_angle(c, -50.0f, x, v,
                    absf_local(error) >= 8.0f ? 0.045f : 0.040f,
                    absf_local(error) >= 8.0f ? 0.032f : 0.035f,
                    3.0f, 2.2f, 2.6f, 12U);
            break;
        case BALL_STATE_HOLD_CENTER:
            c->requested_angle_deg = settle_angle(c, 0.0f, x, v,
                    0.045f, 0.018f, 3.0f, 2.2f, 2.6f, 12U);
            break;
        default: c->requested_angle_deg = 0.0f; break;
    }
}

static void update_center(ball_balance_t *c, float x, float v,
                          uint8_t valid, uint32_t tick)
{
    float error = -x, kp, kd, limit;
    if(BALL_STATE_WAIT_BALL == c->state)
    {
        if(valid)
        {
            if(++c->stable_tick >= 5U)
            {
                c->start_tick = tick;
                if(absf_local(x) >= 9.0f || absf_local(v) >= 30.0f)
                    enter_state(c, BALL_STATE_RECOVER_CENTER, tick);
                else enter_state(c, BALL_STATE_HOLD_CENTER, tick);
            }
        }
        else c->stable_tick = 0U;
        c->requested_angle_deg = 0.0f;
        return;
    }
    if(!valid)
    {
        enter_state(c, BALL_STATE_WAIT_BALL, tick);
        c->requested_angle_deg = 0.0f;
        return;
    }
    if(absf_local(x) >= 105.0f)
    {
        c->fault_code = 2U; enter_state(c, BALL_STATE_SAFE_STOP, tick);
        c->requested_angle_deg = 0.0f; return;
    }
    if(BALL_STATE_HOLD_CENTER == c->state
       && (absf_local(x) >= 9.0f || absf_local(v) >= 30.0f))
        enter_state(c, BALL_STATE_RECOVER_CENTER, tick);
    else if(BALL_STATE_RECOVER_CENTER == c->state)
    {
        if(absf_local(x) <= 8.0f && absf_local(v) <= 12.0f)
        {
            if(++c->stable_tick >= 40U) enter_state(c, BALL_STATE_HOLD_CENTER, tick);
        }
        else c->stable_tick = 0U;
    }

    if(BALL_STATE_HOLD_CENTER != c->state && BALL_STATE_RECOVER_CENTER != c->state)
    { c->requested_angle_deg = 0.0f; return; }
    if(BALL_STATE_HOLD_CENTER != c->state) c->quiet_hold_active = 0U;
    else if(c->quiet_hold_active)
    {
        if(absf_local(error) >= 9.0f || absf_local(v) >= 12.0f)
            c->quiet_hold_active = 0U;
    }
    else if(absf_local(error) <= 8.0f && absf_local(v) <= 5.0f)
        c->quiet_hold_active = 1U;
    if(c->quiet_hold_active)
    { c->requested_angle_deg = 0.0f; c->stiction_active = 0U; c->stiction_tick = 0U; return; }
    if(absf_local(error) >= 8.0f) { kp = 0.045f; kd = 0.032f; }
    else { kp = 0.040f; kd = 0.035f; }
    limit = BALL_STATE_RECOVER_CENTER == c->state ? 3.9f : 3.2f;
    c->requested_angle_deg = settle_angle(c, 0.0f, x, v, kp, kd,
                                           limit, 2.2f, 2.0f, 20U);
}

void ball_balance_init(ball_balance_t *c)
{ if(c) { memset(c, 0, sizeof(*c)); c->state = BALL_STATE_IDLE; } }

void ball_balance_start(ball_balance_t *c, ball_balance_mode_t mode, uint32_t tick)
{
    if(NULL == c) return;
    memset(c, 0, sizeof(*c)); c->mode = mode; c->start_tick = tick;
    c->state_tick = tick;
    c->state = (BALL_MODE_OFF == mode || BALL_MODE_ARBITRARY_RESERVED == mode)
        ? BALL_STATE_IDLE : BALL_STATE_WAIT_BALL;
}

void ball_balance_stop(ball_balance_t *c)
{
    if(NULL == c) return;
    c->mode = BALL_MODE_OFF; c->state = BALL_STATE_IDLE;
    c->requested_angle_deg = 0.0f; c->motor_target_pulses = 0;
}

void ball_balance_update(ball_balance_t *c, const vision_ball_state_struct *ball,
                         uint8_t valid, uint32_t tick)
{
    float x = 0.0f, v = 0.0f;
    if(NULL == c) return;
    if(ball) { x = (float)ball->position_x10_mm * 0.1f; v = (float)ball->velocity_mm_s; }
    if(BALL_MODE_TASK3_ROUND_TRIP == c->mode) update_task3(c, x, v, valid, tick);
    else if(BALL_MODE_CENTER_HOLD == c->mode) update_center(c, x, v, valid, tick);
    else c->requested_angle_deg = 0.0f;
    c->requested_angle_deg = clampf_local(c->requested_angle_deg, -4.0f, 4.0f);
    c->motor_target_pulses = angle_to_pulses(c->requested_angle_deg);
}

uint8_t ball_balance_is_complete(const ball_balance_t *c)
{ return c && BALL_STATE_COMPLETE == c->state; }
uint8_t ball_balance_has_fault(const ball_balance_t *c)
{ return c && BALL_STATE_SAFE_STOP == c->state; }
