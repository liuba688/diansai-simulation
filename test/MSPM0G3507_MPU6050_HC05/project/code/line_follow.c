#include "line_follow.h"

static float line_follow_limit (float value, float limit)
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

static int16 line_follow_abs_i16 (int16 value)
{
    return (value < 0) ? (int16)-value : value;
}

static void line_follow_set_sharp_targets (line_follow_mode_enum mode,
                                           float *left_target_rpm,
                                           float *right_target_rpm)
{
    if(LINE_FOLLOW_MODE_SHARP_LEFT == mode)
    {
        *left_target_rpm = LINE_FOLLOW_SHARP_INNER_RPM;
        *right_target_rpm = LINE_FOLLOW_SHARP_OUTER_RPM;
    }
    else
    {
        *left_target_rpm = LINE_FOLLOW_SHARP_OUTER_RPM;
        *right_target_rpm = LINE_FOLLOW_SHARP_INNER_RPM;
    }
}

void line_follow_init (line_follow_struct *follow)
{
    follow->previous_error = 0;
    follow->last_valid_error = 0;
    follow->sharp_left_ticks = 0;
    follow->sharp_right_ticks = 0;
    follow->sharp_release_ticks = 0;
    follow->lost_ticks = 0;
    follow->mode = LINE_FOLLOW_MODE_NORMAL;
    follow->base_rpm = LINE_FOLLOW_STRAIGHT_RPM;
    follow->correction_rpm = 0.0f;
}

void line_follow_update (line_follow_struct *follow,
                         const line_sensor_data_struct *sensor,
                         float *left_target_rpm,
                         float *right_target_rpm)
{
    int16 error_delta;
    int16 curve_strength;
    uint8 left_edge;
    uint8 right_edge;
    float requested_base_rpm;
    float correction;

    if(!sensor->line_valid)
    {
        if(follow->lost_ticks < 255)
        {
            follow->lost_ticks ++;
        }

        if(follow->lost_ticks <= LINE_FOLLOW_LOST_SEARCH_TICKS)
        {
            follow->mode = LINE_FOLLOW_MODE_LOST_SEARCH;
            if(follow->last_valid_error < 0)
            {
                *left_target_rpm = LINE_FOLLOW_LOST_INNER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_OUTER_RPM;
            }
            else
            {
                *left_target_rpm = LINE_FOLLOW_LOST_OUTER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_INNER_RPM;
            }
        }
        else
        {
            follow->mode = LINE_FOLLOW_MODE_LOST_STOP;
            *left_target_rpm = 0.0f;
            *right_target_rpm = 0.0f;
        }
        follow->correction_rpm = 0.0f;
        return;
    }

    follow->lost_ticks = 0;
    follow->last_valid_error = sensor->error;
    left_edge = (0 != (sensor->mask & 0x03))
              && (0 == (sensor->mask & 0xC0));
    right_edge = (0 != (sensor->mask & 0xC0))
               && (0 == (sensor->mask & 0x03));

    if(left_edge)
    {
        if(follow->sharp_left_ticks < 255)
        {
            follow->sharp_left_ticks ++;
        }
        follow->sharp_right_ticks = 0;
    }
    else if(right_edge)
    {
        if(follow->sharp_right_ticks < 255)
        {
            follow->sharp_right_ticks ++;
        }
        follow->sharp_left_ticks = 0;
    }
    else
    {
        follow->sharp_left_ticks = 0;
        follow->sharp_right_ticks = 0;
    }

    if(follow->sharp_left_ticks >= LINE_FOLLOW_SHARP_CONFIRM_TICKS)
    {
        follow->mode = LINE_FOLLOW_MODE_SHARP_LEFT;
        follow->sharp_release_ticks = 0;
    }
    else if(follow->sharp_right_ticks >= LINE_FOLLOW_SHARP_CONFIRM_TICKS)
    {
        follow->mode = LINE_FOLLOW_MODE_SHARP_RIGHT;
        follow->sharp_release_ticks = 0;
    }

    if((LINE_FOLLOW_MODE_SHARP_LEFT == follow->mode)
       || (LINE_FOLLOW_MODE_SHARP_RIGHT == follow->mode))
    {
        /* Return to normal only after the two center sensors are stable. */
        if(0 != (sensor->mask & 0x18))
        {
            follow->sharp_release_ticks ++;
            if(follow->sharp_release_ticks >= LINE_FOLLOW_SHARP_RELEASE_TICKS)
            {
                follow->mode = LINE_FOLLOW_MODE_NORMAL;
                follow->sharp_release_ticks = 0;
                follow->previous_error = sensor->error;
            }
        }
        else
        {
            follow->sharp_release_ticks = 0;
        }

        if(LINE_FOLLOW_MODE_NORMAL != follow->mode)
        {
            line_follow_set_sharp_targets(follow->mode,
                                          left_target_rpm,
                                          right_target_rpm);
            follow->base_rpm = LINE_FOLLOW_SHARP_OUTER_RPM;
            follow->correction_rpm =
                LINE_FOLLOW_SHARP_OUTER_RPM - LINE_FOLLOW_SHARP_INNER_RPM;
            return;
        }
    }
    else
    {
        follow->mode = LINE_FOLLOW_MODE_NORMAL;
    }

    error_delta = sensor->error - follow->previous_error;
    curve_strength = line_follow_abs_i16(sensor->error)
                   + (int16)(LINE_FOLLOW_SPEED_DERROR_GAIN
                             * line_follow_abs_i16(error_delta));
    requested_base_rpm = LINE_FOLLOW_STRAIGHT_RPM
                       - LINE_FOLLOW_SPEED_ERROR_GAIN * curve_strength;
    if(requested_base_rpm < LINE_FOLLOW_MIN_CURVE_RPM)
    {
        requested_base_rpm = LINE_FOLLOW_MIN_CURVE_RPM;
    }

    /* Brake immediately for a curve, accelerate gently after recentering. */
    if(requested_base_rpm < follow->base_rpm)
    {
        follow->base_rpm = requested_base_rpm;
    }
    else
    {
        follow->base_rpm += LINE_FOLLOW_SPEED_RECOVERY_RPM;
        if(follow->base_rpm > requested_base_rpm)
        {
            follow->base_rpm = requested_base_rpm;
        }
    }

    correction = LINE_FOLLOW_KP * sensor->error
               + LINE_FOLLOW_KD * error_delta;
    correction = line_follow_limit(correction,
                                    LINE_FOLLOW_CORRECTION_MAX_RPM);

    /*
     * Never accelerate the outside wheel above the adaptive base speed.
     * Large corrections may stop or reverse only the inside wheel.
     */
    if(correction >= 0.0f)
    {
        *left_target_rpm = follow->base_rpm;
        *right_target_rpm = follow->base_rpm - correction;
    }
    else
    {
        *left_target_rpm = follow->base_rpm + correction;
        *right_target_rpm = follow->base_rpm;
    }

    follow->previous_error = sensor->error;
    follow->correction_rpm = correction;
}
