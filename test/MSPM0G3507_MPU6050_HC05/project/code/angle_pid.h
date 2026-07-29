#ifndef _angle_pid_h_
#define _angle_pid_h_

#include "zf_common_typedef.h"

/*
 * 角度环 PID — 位置式 Yaw error -> differential RPM
 *
 * 两套参数: WEAK (巡线叠加, 小权重) / STRONG (脱线/转弯, 全权重)
 */

/* ---------- weak overlay (during line-follow) ---------- */
#define ANGLE_PID_WEAK_KP               (0.5f)
#define ANGLE_PID_WEAK_KI               (0.01f)
#define ANGLE_PID_WEAK_KD               (0.1f)
#define ANGLE_PID_WEAK_OUTPUT_MAX       (15.0f)  /* RPM */
#define ANGLE_PID_WEAK_INTEGRAL_MAX     (8.0f)   /* deg*s */

/* ---------- strong hold (off-line / turning) ---------- */
#define ANGLE_PID_STRONG_KP             (2.0f)
#define ANGLE_PID_STRONG_KI             (0.05f)
#define ANGLE_PID_STRONG_KD             (0.3f)
#define ANGLE_PID_STRONG_OUTPUT_MAX     (55.0f)  /* RPM */
#define ANGLE_PID_STRONG_INTEGRAL_MAX   (30.0f)  /* deg*s */

/* ---------- stationary hold: deliberately gentle for first real-car test ---------- */
#define ANGLE_PID_HOLD_KP               (0.8f)
#define ANGLE_PID_HOLD_KI               (0.0f)
#define ANGLE_PID_HOLD_KD               (0.05f)
#define ANGLE_PID_HOLD_OUTPUT_MAX       (18.0f)  /* RPM */
#define ANGLE_PID_HOLD_INTEGRAL_MAX     (8.0f)
#define ANGLE_PID_HOLD_DEADBAND_DEG     (1.5f)

/* ---------- generic ---------- */
#define ANGLE_PID_BASE_PERIOD_MS        (10)

/* ---------- struct ---------- */
typedef struct
{
    float target_deg;        /* desired yaw, degrees */
    float error;             /* current error, degrees (wrap-compensated) */
    float error_prev;        /* previous error */
    float integral;          /* accumulated error * dt */
    float output_rpm;        /* differential RPM: + => right faster => turn left */
} angle_pid_struct;

/* ---------- API ---------- */
void  angle_pid_init      (angle_pid_struct *pid);
void  angle_pid_set_target(angle_pid_struct *pid, float target_deg);
void  angle_pid_reset     (angle_pid_struct *pid);
float angle_pid_update    (angle_pid_struct *pid, float current_yaw_deg,
                           float kp, float ki, float kd,
                           float out_max, float integral_max,
                           float dt_s);

#endif
