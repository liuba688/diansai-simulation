#include "line_follow.h"

static float line_follow_limit (float value, float limit)
{
    if(value > limit) { return limit; }
    if(value < -limit) { return -limit; }
    return value;
}

static int16 line_follow_abs_i16 (int16 value)
{
    return (value < 0) ? (int16)-value : value;
}

static uint8 line_follow_mask_is_contiguous(uint8 mask)
{
    if(0U == mask) { return 0U; }
    while(0U == (mask & 0x01U)) { mask >>= 1; }
    return (uint8)(0U == (mask & (uint8)(mask + 1U)));
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

/*
 * Determine track phase from odometer distance with hysteresis.
 * Track: Straight-1 (1.5m) → Curve-1 (π×0.5m) → Straight-2 (1.5m) → Curve-2 → repeat.
 */
static track_phase_enum line_follow_detect_phase (float distance_cm,
                                                   track_phase_enum current_phase)
{
    float d = distance_cm;
    float hyst = TRACK_PHASE_HYSTERESIS_CM;

    /* Wrap into one lap. */
    while(d >= TRACK_FULL_LAP_CM) { d -= TRACK_FULL_LAP_CM; }
    while(d < 0.0f)              { d += TRACK_FULL_LAP_CM; }

    float s1_end = TRACK_STRAIGHT_LENGTH_CM;           /* 150.0 cm */
    float c1_end = TRACK_HALF_LAP_CM;                  /* 307.1 cm */
    float s2_end = TRACK_STRAIGHT_LENGTH_CM + TRACK_HALF_LAP_CM; /* 457.1 cm */

    track_phase_enum raw_phase;
    if(d < s1_end)      { raw_phase = TRACK_PHASE_STRAIGHT_1; }
    else if(d < c1_end) { raw_phase = TRACK_PHASE_CURVE_1; }
    else if(d < s2_end) { raw_phase = TRACK_PHASE_STRAIGHT_2; }
    else                { raw_phase = TRACK_PHASE_CURVE_2; }

    /* Hysteresis: only switch into a new phase after crossing well past the boundary. */
    if(raw_phase != current_phase)
    {
        switch(raw_phase)
        {
            case TRACK_PHASE_CURVE_1:
                if(d < (s1_end + hyst) && (TRACK_PHASE_STRAIGHT_1 == current_phase))
                { return current_phase; }
                break;

            case TRACK_PHASE_STRAIGHT_2:
                if(d < (c1_end + hyst) && (TRACK_PHASE_CURVE_1 == current_phase))
                { return current_phase; }
                break;

            case TRACK_PHASE_CURVE_2:
                if(d < (s2_end + hyst) && (TRACK_PHASE_STRAIGHT_2 == current_phase))
                { return current_phase; }
                break;

            case TRACK_PHASE_STRAIGHT_1:
                /* Wrap-around: only enter straight-1 after passing well past curve-2 end. */
                if((d > (TRACK_FULL_LAP_CM - hyst))
                   && (TRACK_PHASE_CURVE_2 == current_phase))
                { return current_phase; }
                break;

            default:
                break;
        }
    }
    return raw_phase;
}

void line_follow_init (line_follow_struct *follow)
{
    follow->previous_error = 0;
    follow->last_valid_error = 0;
    follow->sharp_left_ticks = 0;
    follow->sharp_right_ticks = 0;
    follow->sharp_release_ticks = 0;
    follow->lost_ticks = 0;
    follow->lost_brake_ticks = 0;
    follow->loss_events = 0U;
    follow->edge_ticks = 0U;
    follow->edge_events = 0U;
    follow->edge_max_ticks = 0U;
    follow->curve_active = 0U;
    follow->curve_aborted = 0U;
    follow->curve_abort_ticks = 0U;
    follow->curve_enter_ticks = 0U;
    follow->curve_exit_ticks = 0U;
    follow->run_ticks = 0U;
    follow->curve_start_yaw_deg = 0.0f;
    follow->curve_yaw_progress_deg = 0.0f;
    follow->mode = LINE_FOLLOW_MODE_NORMAL;
    follow->track_phase = TRACK_PHASE_STRAIGHT_1;
    follow->params.straight_rpm = LINE_FOLLOW_STRAIGHT_RPM;
    follow->params.max_curve_rpm = LINE_FOLLOW_MAX_CURVE_RPM;
    follow->params.min_curve_rpm = LINE_FOLLOW_MIN_CURVE_RPM;
    follow->params.kp = LINE_FOLLOW_KP;
    follow->params.kd = LINE_FOLLOW_KD;
    follow->params.correction_max_rpm = LINE_FOLLOW_CORRECTION_MAX_RPM;
    follow->base_rpm = follow->params.straight_rpm;
    follow->correction_rpm = 0.0f;
    follow->curve_direction = 0.0f;
}

void line_follow_set_params (line_follow_struct *follow,
                             const line_follow_params_struct *params)
{
    if((0 == follow) || (0 == params))
    {
        return;
    }
    follow->params = *params;
}

void line_follow_update (line_follow_struct *follow,
                         const line_sensor_data_struct *sensor,
                         float yaw_total_deg,
                         float yaw_rate_dps,
                         uint8 yaw_ready,
                         float distance_cm,
                         float *left_target_rpm,
                         float *right_target_rpm)
{
    int16 error_delta;
    int16 curve_strength;
    uint8 left_edge;
    uint8 right_edge;
    float requested_base_rpm;
    float correction;
    track_phase_enum new_phase;

    if(follow->run_ticks < 65535U) { follow->run_ticks++; }

    /* ================================================================
     * LOST LINE — brake pulse, emergency braking, then search
     * ================================================================ */
    if(!sensor->line_valid)
    {
        if(0U == follow->lost_ticks)
        {
            if(follow->loss_events < 65535U)
            {
                follow->loss_events++;
            }
        }
        if(follow->lost_ticks < 255) { follow->lost_ticks++; }

        /*
         * Emergency brake: yaw rate has spiked (spinning out).
         * Kill all motor output for LOST_BRAKE_TICKS, then search.
         */
        if(yaw_ready
           && ((yaw_rate_dps > LINE_FOLLOW_SPIN_YAW_DPS_THRESHOLD)
               || (yaw_rate_dps < -LINE_FOLLOW_SPIN_YAW_DPS_THRESHOLD))
           && (follow->lost_ticks <= 3))
        {
            follow->mode = LINE_FOLLOW_MODE_LOST_SEARCH;
            *left_target_rpm = 0.0f;
            *right_target_rpm = 0.0f;
            follow->lost_brake_ticks = LINE_FOLLOW_LOST_BRAKE_TICKS;
            follow->correction_rpm = 0.0f;
            return;
        }

        /* Initial brake pulse to kill angular momentum before searching. */
        if(follow->lost_brake_ticks > 0)
        {
            follow->lost_brake_ticks--;
            follow->mode = LINE_FOLLOW_MODE_LOST_SEARCH;
            *left_target_rpm = 0.0f;
            *right_target_rpm = 0.0f;
            follow->correction_rpm = 0.0f;
            return;
        }

        /*
         * Reject isolated sensor gaps. For the first 50 ms, remain in normal
         * mode and continue the last steering direction at reduced speed.
         * This avoids turning a single missed sample into an inner-wheel
         * reversal while still slowing the car promptly.
         */
        if(follow->lost_ticks <= LINE_FOLLOW_LOST_CONFIRM_TICKS)
        {
            follow->mode = LINE_FOLLOW_MODE_NORMAL;
            if(follow->last_valid_error < 0)
            {
                *left_target_rpm = LINE_FOLLOW_LOST_COAST_INNER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_COAST_OUTER_RPM;
            }
            else
            {
                *left_target_rpm = LINE_FOLLOW_LOST_COAST_OUTER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_COAST_INNER_RPM;
            }
        }
        else if(follow->lost_ticks
                <= (LINE_FOLLOW_LOST_CONFIRM_TICKS
                    + LINE_FOLLOW_LOST_FORWARD_TICKS))
        {
            follow->mode = LINE_FOLLOW_MODE_LOST_SEARCH;
            if(follow->last_valid_error < 0)
            {
                *left_target_rpm = LINE_FOLLOW_LOST_FORWARD_INNER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_FORWARD_OUTER_RPM;
            }
            else
            {
                *left_target_rpm = LINE_FOLLOW_LOST_FORWARD_OUTER_RPM;
                *right_target_rpm = LINE_FOLLOW_LOST_FORWARD_INNER_RPM;
            }
        }
        else if(follow->lost_ticks
                <= (LINE_FOLLOW_LOST_CONFIRM_TICKS
                    + LINE_FOLLOW_LOST_FORWARD_TICKS
                    + LINE_FOLLOW_LOST_SEARCH_TICKS))
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

    /* ================================================================
     * LINE VISIBLE — reset lost counters, update error tracking
     * ================================================================ */
    follow->lost_ticks = 0;
    follow->lost_brake_ticks = 0;
    follow->last_valid_error = sensor->error;

    if((0x01U == sensor->mask) || (0x80U == sensor->mask))
    {
        if(0U == follow->edge_ticks)
        {
            if(follow->edge_events < 65535U)
            {
                follow->edge_events++;
            }
        }
        if(follow->edge_ticks < 255U)
        {
            follow->edge_ticks++;
        }
        if((uint16)follow->edge_ticks > follow->edge_max_ticks)
        {
            follow->edge_max_ticks = follow->edge_ticks;
        }
    }
    else
    {
        follow->edge_ticks = 0U;
    }

    /* Odometry-based track phase detection. */
    new_phase = line_follow_detect_phase(distance_cm, follow->track_phase);
    follow->track_phase = new_phase;

    /* ================================================================
     * SHARP TURN detection (edge-based, kept for safety in tight spots)
     * ================================================================ */
    left_edge  = (0 != (sensor->mask & 0x03)) && (0 == (sensor->mask & 0xC0));
    right_edge = (0 != (sensor->mask & 0xC0)) && (0 == (sensor->mask & 0x03));

    if(left_edge)
    {
        if(follow->sharp_left_ticks < 255) { follow->sharp_left_ticks++; }
        follow->sharp_right_ticks = 0;
    }
    else if(right_edge)
    {
        if(follow->sharp_right_ticks < 255) { follow->sharp_right_ticks++; }
        follow->sharp_left_ticks = 0;
    }
    else
    {
        follow->sharp_left_ticks = 0;
        follow->sharp_right_ticks = 0;
    }

    if(LINE_FOLLOW_SHARP_ENABLE
       && (follow->sharp_left_ticks >= LINE_FOLLOW_SHARP_CONFIRM_TICKS))
    {
        follow->mode = LINE_FOLLOW_MODE_SHARP_LEFT;
        follow->sharp_release_ticks = 0;
    }
    else if(LINE_FOLLOW_SHARP_ENABLE
            && (follow->sharp_right_ticks >= LINE_FOLLOW_SHARP_CONFIRM_TICKS))
    {
        follow->mode = LINE_FOLLOW_MODE_SHARP_RIGHT;
        follow->sharp_release_ticks = 0;
    }

    if((LINE_FOLLOW_MODE_SHARP_LEFT == follow->mode)
       || (LINE_FOLLOW_MODE_SHARP_RIGHT == follow->mode))
    {
        if(0 != (sensor->mask & 0x18))
        {
            follow->sharp_release_ticks++;
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

    /* ================================================================
     * BASE SPEED — dynamic reduction based on error magnitude
     * ================================================================ */
    error_delta = sensor->error - follow->previous_error;
    curve_strength = line_follow_abs_i16(sensor->error)
                   + (int16)(LINE_FOLLOW_SPEED_DERROR_GAIN
                             * line_follow_abs_i16(error_delta));
    requested_base_rpm = follow->params.straight_rpm
                       - LINE_FOLLOW_SPEED_ERROR_GAIN * curve_strength;

    /*
     * Anticipatory braking: when approaching a curve entry, pre-reduce
     * the base speed so the car enters the curve at a safe speed instead
     * of braking abruptly at the boundary.
     */
    if(LINE_FOLLOW_ARC_FUSION_ENABLE)
    {
        float curve_entry_cm;
        float dist_to_curve;
        uint8 approaching_curve = 0U;

        if(TRACK_PHASE_STRAIGHT_1 == follow->track_phase)
        {
            curve_entry_cm = TRACK_STRAIGHT_LENGTH_CM;
            approaching_curve = 1U;
        }
        else if(TRACK_PHASE_STRAIGHT_2 == follow->track_phase)
        {
            curve_entry_cm = TRACK_STRAIGHT_LENGTH_CM + TRACK_HALF_LAP_CM;
            approaching_curve = 1U;
        }

        if(approaching_curve)
        {
            dist_to_curve = curve_entry_cm - distance_cm;
            if(dist_to_curve < TRACK_CURVE_APPROACH_ZONE_CM
               && dist_to_curve > 0.0f)
            {
                float approach_max =
                    follow->params.max_curve_rpm
                    + (follow->params.straight_rpm - follow->params.max_curve_rpm)
                      * (dist_to_curve / TRACK_CURVE_APPROACH_ZONE_CM);
                if(requested_base_rpm > approach_max)
                {
                    requested_base_rpm = approach_max;
                }
            }
            else if(dist_to_curve <= 0.0f)
            {
                /* Already past the boundary — cap to curve speed. */
                if(requested_base_rpm > follow->params.max_curve_rpm)
                {
                    requested_base_rpm = follow->params.max_curve_rpm;
                }
            }
        }
    }

    /* ================================================================
     * ARC FUSION — odometry-triggered curve feedforward + IMU damping
     * ================================================================ */
    if(LINE_FOLLOW_ARC_FUSION_ENABLE)
    {
        uint8 in_curve_phase = (TRACK_PHASE_CURVE_1 == follow->track_phase)
                            || (TRACK_PHASE_CURVE_2 == follow->track_phase);

        if(in_curve_phase
           && yaw_ready
           && !follow->curve_aborted
           && (follow->run_ticks >= LINE_FOLLOW_CURVE_START_INHIBIT_TICKS))
        {
            if(!follow->curve_active)
            {
                /* Enter curve: latch yaw origin and fixed right-turn direction. */
                follow->curve_active = 1U;
                follow->curve_direction = LINE_FOLLOW_TRACK_TURN_DIRECTION;
                follow->curve_start_yaw_deg = yaw_total_deg;
                follow->curve_yaw_progress_deg = 0.0f;
                follow->curve_exit_ticks = 0U;
            }

            /* Track yaw progress for exit detection. */
            follow->curve_yaw_progress_deg =
                line_follow_limit(
                    yaw_total_deg - follow->curve_start_yaw_deg,
                    360.0f);
            if(follow->curve_yaw_progress_deg < 0.0f)
            {
                follow->curve_yaw_progress_deg =
                    -follow->curve_yaw_progress_deg;
            }

            /*
             * A sustained large error opposite to the commanded turn means
             * the car has crossed the line or the odometry phase is late.
             * Release fixed curvature for the rest of this phase so normal
             * line following can recover instead of forcing the car onward.
             */
            if((follow->curve_direction * (float)sensor->error)
               <= -(float)LINE_FOLLOW_CURVE_ABORT_ERROR)
            {
                if(follow->curve_abort_ticks < 255U)
                {
                    follow->curve_abort_ticks++;
                }
                if(follow->curve_abort_ticks >= LINE_FOLLOW_CURVE_ABORT_TICKS)
                {
                    follow->curve_active = 0U;
                    follow->curve_aborted = 1U;
                    follow->curve_direction = 0.0f;
                    follow->curve_exit_ticks = 0U;
                }
            }
            else
            {
                follow->curve_abort_ticks = 0U;
            }

            /* Exit: enough rotation AND sensor back near center. */
            if(follow->curve_active
               && (follow->curve_yaw_progress_deg
                >= LINE_FOLLOW_CURVE_EXIT_YAW_DEG)
               && (curve_strength <= LINE_FOLLOW_CURVE_EXIT_STRENGTH))
            {
                if(follow->curve_exit_ticks < 255U)
                {
                    follow->curve_exit_ticks++;
                }
                if(follow->curve_exit_ticks
                   >= LINE_FOLLOW_CURVE_EXIT_TICKS)
                {
                    follow->curve_active = 0U;
                    follow->curve_direction = 0.0f;
                    follow->curve_enter_ticks = 0U;
                    follow->curve_exit_ticks = 0U;
                }
            }
            else
            {
                follow->curve_exit_ticks = 0U;
            }
        }
        else
        {
            /* Left curve phase — gracefully exit arc fusion. */
            if(follow->curve_active)
            {
                follow->curve_active = 0U;
                follow->curve_direction = 0.0f;
            }
            follow->curve_aborted = 0U;
            follow->curve_abort_ticks = 0U;
            follow->curve_enter_ticks = 0U;
            follow->curve_exit_ticks = 0U;
        }
    }
    /* When fusion is disabled, preserve the legacy error-based cap as a safety net. */
    if(!LINE_FOLLOW_ARC_FUSION_ENABLE
       && (curve_strength >= LINE_FOLLOW_CURVE_ENTER_STRENGTH)
       && (requested_base_rpm > follow->params.max_curve_rpm))
    {
        requested_base_rpm = follow->params.max_curve_rpm;
    }

    /* Global speed floor and curve cap. */
    if(follow->curve_active
       && (requested_base_rpm > follow->params.max_curve_rpm))
    {
        requested_base_rpm = follow->params.max_curve_rpm;
    }
    if(requested_base_rpm < follow->params.min_curve_rpm)
    {
        requested_base_rpm = follow->params.min_curve_rpm;
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

    /* PD correction. */
    correction = follow->params.kp * sensor->error
               + follow->params.kd * error_delta;
    correction = line_follow_limit(correction,
                                    follow->params.correction_max_rpm);

    /* ================================================================
     * CURVE FEEDFORWARD + FEEDBACK (when arc fusion is active)
     * ================================================================ */
    if(follow->curve_active)
    {
        float curve_feedback;
        float turn_half_diff;

        follow->base_rpm = LINE_FOLLOW_CURVE_CENTER_RPM;
        curve_feedback =
            LINE_FOLLOW_CURVE_FEEDBACK_KP * sensor->error
            + LINE_FOLLOW_CURVE_FEEDBACK_KD * error_delta;
        curve_feedback = line_follow_limit(
            curve_feedback,
            LINE_FOLLOW_CURVE_FEEDBACK_MAX_RPM);
        turn_half_diff =
            follow->curve_direction
                * LINE_FOLLOW_CURVE_HALF_DIFF_RPM
            + curve_feedback;
        *left_target_rpm = follow->base_rpm + turn_half_diff;
        *right_target_rpm = follow->base_rpm - turn_half_diff;
        follow->previous_error = sensor->error;
        follow->correction_rpm = 2.0f * turn_half_diff;
        return;
    }
    follow->curve_direction = 0.0f;

    /*
     * Standard PD output: never accelerate the outside wheel above the
     * adaptive base speed. Large corrections may stop or reverse only
     * the inside wheel.
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
