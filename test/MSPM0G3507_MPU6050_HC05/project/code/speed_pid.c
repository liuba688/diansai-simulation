#include "speed_pid.h"

static float speed_pid_limit (float value, float limit)
{
    if(value > limit)
    {
        value = limit;
    }
    else if(value < -limit)
    {
        value = -limit;
    }
    return value;
}

static float speed_pid_sign (float value)
{
    if(value > 0.0f)
    {
        return 1.0f;
    }
    else if(value < 0.0f)
    {
        return -1.0f;
    }
    return 0.0f;
}

void speed_pid_init (speed_pid_struct *pid)
{
    pid->target_rpm = 0.0f;
    pid->raw_rpm = 0.0f;
    pid->measured_rpm = 0.0f;
    pid->output = 0.0f;
    pid->error_1 = 0.0f;
    pid->error_2 = 0.0f;
}

void speed_pid_reset (speed_pid_struct *pid)
{
    float target_rpm = pid->target_rpm;
    speed_pid_init(pid);
    pid->target_rpm = target_rpm;
}

void speed_pid_set_target (speed_pid_struct *pid, float target_rpm)
{
    if((0.0f == target_rpm)
       || ((pid->target_rpm > 0.0f) && (target_rpm < 0.0f))
       || ((pid->target_rpm < 0.0f) && (target_rpm > 0.0f)))
    {
        speed_pid_init(pid);
    }
    pid->target_rpm = target_rpm;
}

int16 speed_pid_update (speed_pid_struct *pid, int32 encoder_delta,
                        float kp, float ki, float kd)
{
    float error;
    float increment;
    float drive_output;

    pid->raw_rpm = ((float)encoder_delta * 60000.0f)
                 / (MOTOR_ENCODER_COUNTS_PER_REV * SPEED_PID_PERIOD_MS);
    pid->measured_rpm += SPEED_PID_FILTER_ALPHA
                       * (pid->raw_rpm - pid->measured_rpm);

    if(0.0f == pid->target_rpm)
    {
        speed_pid_reset(pid);
        return 0;
    }

    error = pid->target_rpm - pid->measured_rpm;
    increment = kp * (error - pid->error_1)
              + ki * error
              + kd * (error - 2.0f * pid->error_1 + pid->error_2);

    pid->output = speed_pid_limit(pid->output + increment,
                                  SPEED_PID_OUTPUT_LIMIT);
    pid->error_2 = pid->error_1;
    pid->error_1 = error;

    drive_output = speed_pid_sign(pid->target_rpm) * SPEED_PID_START_DUTY
                 + pid->output;
    drive_output = speed_pid_limit(drive_output, SPEED_PID_OUTPUT_LIMIT);

    return (int16)drive_output;
}
