#ifndef _speed_pid_h_
#define _speed_pid_h_

#include "zf_common_typedef.h"

// 自有电机商家给出的输出轴编码器计数：直接采用 2450 count/rev。
#define MOTOR_ENCODER_COUNTS_PER_REV       (2450.0f)

// 统一整车坐标：正目标表示小车前进。
// 实物最终映射：左轮=MOTOR1 编码器(PA25/PA14)+TB6612 B，
// 右轮=MOTOR2 编码器(PA26/PA27)+TB6612 A。以下符号按驱动通道定义。
#define SPEED_PID_TB6612_A_FORWARD_SIGN    (-1)
#define SPEED_PID_TB6612_B_FORWARD_SIGN    (1)
#define SPEED_PID_MOTOR1_ENCODER_SIGN      (-1)
#define SPEED_PID_MOTOR2_ENCODER_SIGN      (1)

#define SPEED_PID_BASE_PERIOD_MS           (10)
#define SPEED_PID_PERIOD_MS                (20)
#define SPEED_PID_CONTROL_DIVIDER          (SPEED_PID_PERIOD_MS / SPEED_PID_BASE_PERIOD_MS)

// 两轮机械特性不同，增益必须允许独立整定。
#define SPEED_PID_MOTOR1_KP                (10.0f)
#define SPEED_PID_MOTOR1_KI                (1.0f)
#define SPEED_PID_MOTOR1_KD                (0.0f)
#define SPEED_PID_MOTOR2_KP                (10.0f)
#define SPEED_PID_MOTOR2_KI                (1.0f)
#define SPEED_PID_MOTOR2_KD                (0.0f)
#define SPEED_PID_OUTPUT_LIMIT             (8000.0f)
#define SPEED_PID_START_DUTY               (1000.0f)
#define SPEED_PID_FILTER_ALPHA             (0.25f)

typedef struct
{
    float target_rpm;
    float raw_rpm;
    float measured_rpm;
    float output;
    float error_1;
    float error_2;
} speed_pid_struct;

void  speed_pid_init       (speed_pid_struct *pid);
void  speed_pid_reset      (speed_pid_struct *pid);
void  speed_pid_set_target (speed_pid_struct *pid, float target_rpm);
int16 speed_pid_update     (speed_pid_struct *pid, int32 encoder_delta,
                            float kp, float ki, float kd);

#endif
