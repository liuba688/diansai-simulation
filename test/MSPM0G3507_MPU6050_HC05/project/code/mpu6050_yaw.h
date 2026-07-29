#ifndef _mpu6050_yaw_h_
#define _mpu6050_yaw_h_

#include "zf_common_typedef.h"

/*
 * MPU-6500 Yaw Angle Module (精简互补滤波方案)
 *
 * 硬件: MPU-6500 (WHO_AM_I=0x70), I2C 0x68, SCL=PA1, SDA=PA0
 * Yaw 独立梯形积分。
 * 允许模块倾斜安装：上电静止校准时记录重力方向，将三轴角速度
 * 投影到该方向，得到车体绕竖直轴的角速度。
 */

/* ---------- MPU register / hw ---------- */
#define MPU6050_I2C_ADDR                (0x68)
#define MPU6500_WHO_AM_I                (0x70)

#define MPU6050_RA_PWR_MGMT_1           (0x6B)
#define MPU6050_RA_SMPLRT_DIV           (0x19)
#define MPU6050_RA_CONFIG               (0x1A)
#define MPU6050_RA_GYRO_CONFIG          (0x1B)
#define MPU6050_RA_ACCEL_CONFIG         (0x1C)
#define MPU6050_RA_ACCEL_XOUT_H         (0x3B)
#define MPU6050_RA_GYRO_XOUT_H          (0x43)
#define MPU6050_RA_WHO_AM_I             (0x75)

/* ---------- range / filter ---------- */
#define MPU6050_GYRO_FS_SEL             (0x18)   /* FS_SEL=3  ==> 2000 dps */
#define MPU6050_ACCEL_FS_SEL            (0x00)   /* AFS_SEL=0 ==>   2 g */
#define MPU6050_DLPF_CFG                (0x03)   /* DLPF=3 => ~42 Hz, 1 ms delay */
#define MPU6050_SMPLRT_DIV_VAL          (7)      /* 1 kHz / (1+7) = 125 Hz */
#define MPU6050_CLKSEL_PLL              (1)      /* use gyro-X PLL */

#define MPU6050_GYRO_SENSITIVITY        (16.4f)     /* LSB per dps  @2000 */
#define MPU6050_ACCEL_SENSITIVITY       (16384.0f)  /* LSB per g    @  2g */

/* ---------- protection thresholds ---------- */
#define MPU6500_DEAD_ZONE_DPS           (0.25f)
#define MPU6500_GYRO_LIMIT_DPS          (500.0f)
#define MPU6500_DT_MAX_MS               (50.0f)
#define MPU6500_INC_LIMIT_DEG           (5.0f)

/*
 * Installed-vehicle calibration.
 * Current firmware measured 71.3 deg for a physical 90 deg turn and
 * 283.5 deg for 360 deg.  The mean correction is approximately 1.268.
 * This includes the stable main-loop timing overhead on this target.
 */
#define MPU6500_YAW_CALIBRATION_SCALE    (1.268f)

/* ---------- drift compensation ---------- */
#define MPU6500_DRIFT_ALPHA             (0.001f)
#define MPU6500_STATIONARY_GZ_DPS       (0.5f)
#define MPU6500_STATIONARY_ACCEL_G      (0.15f)

/* ---------- calibration ---------- */
#define MPU6500_CALIB_SAMPLES           (500)
#define MPU6500_PROBE_RETRIES           (20)
#define MPU6500_PROBE_DELAY_MS          (20)

/* update period (must match main loop period) */
#define MPU6050_YAW_PERIOD_MS            (10)

/* ---------- public API ---------- */

void  mpu6050_yaw_init(void);
void  mpu6050_yaw_calibrate(uint16 samples);
void  mpu6050_yaw_update(void);          /* full: accel+gyro, ~0.5ms */
void  mpu6050_yaw_update_fast(void);     /* gyro-only yaw integral, ~0.15ms */
float mpu6050_yaw_get_angle(void);       /* deg, -180 .. +180 */
float mpu6050_yaw_get_total_angle(void); /* deg, unwrapped since reset */
void  mpu6050_yaw_set_angle(float deg);
float mpu6050_yaw_get_gz_dps(void);      /* compatibility: projected yaw rate */
float mpu6050_yaw_get_rate_dps(void);
float mpu6050_yaw_get_dt_ms(void);
uint32 mpu6050_yaw_get_timer_ticks(void);
void  mpu6050_yaw_get_mount_axis(float *x, float *y, float *z);
void  mpu6050_yaw_get_gyro_dps(float *x, float *y, float *z);
uint8 mpu6050_yaw_read_who_am_i(void);
uint8 mpu6050_yaw_is_ready(void);        /* 1 after init+calib done */

#endif
