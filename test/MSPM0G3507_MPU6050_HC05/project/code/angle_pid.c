#include "angle_pid.h"

/* ========================================================================
 * Angle PID — positional form, yaw error -> differential RPM
 *
 * diff_rpm > 0 -> right wheel faster -> vehicle turns LEFT
 * ======================================================================== */

static float limit_sym (float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

static float wrap_error (float err)
{
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ======================================================================== */

void angle_pid_init (angle_pid_struct *pid)
{
    pid->target_deg = 0.0f;
    pid->error      = 0.0f;
    pid->error_prev = 0.0f;
    pid->integral   = 0.0f;
    pid->output_rpm = 0.0f;
}

void angle_pid_set_target (angle_pid_struct *pid, float target_deg)
{
    /* normalise target */
    while (target_deg >  180.0f) target_deg -= 360.0f;
    while (target_deg < -180.0f) target_deg += 360.0f;

    if (target_deg != pid->target_deg)
    {
        /* target changed => reset integral */
        pid->integral   = 0.0f;
        pid->error_prev = 0.0f;
    }

    pid->target_deg = target_deg;
}

void angle_pid_reset (angle_pid_struct *pid)
{
    float t = pid->target_deg;
    angle_pid_init(pid);
    pid->target_deg = t;
}

float angle_pid_update (angle_pid_struct *pid, float current_yaw_deg,
                        float kp, float ki, float kd,
                        float out_max, float integral_max,
                        float dt_s)
{
    float error_raw;
    float derivative;

    /* error with wrap */
    error_raw = pid->target_deg - current_yaw_deg;
    pid->error = wrap_error(error_raw);

    /* integral */
    pid->integral += pid->error * dt_s;
    pid->integral  = limit_sym(pid->integral, integral_max);

    /* derivative (on error, not measurement) */
    derivative = (pid->error - pid->error_prev) / (dt_s > 0.001f ? dt_s : 0.010f);
    pid->error_prev = pid->error;

    /* PID output */
    pid->output_rpm = kp * pid->error
                    + ki * pid->integral
                    + kd * derivative;

    pid->output_rpm = limit_sym(pid->output_rpm, out_max);

    return pid->output_rpm;
}
