#include "zf_common_headfile.h"
#include "mpu6050_yaw.h"

/* ========================================================================
 * MPU-6500 Yaw module — soft-I2C, trapezoidal yaw integration
 *
 * Hardware: I2C addr 0x68, SCL=PA1, SDA=PA0, soft_iic delay=10
 * Sample rate: 125 Hz (SMPLRT_DIV=7, 1 kHz internal)
 * DLPF: 42 Hz accel, 41 Hz gyro
 * Range: gyro +/-2000 dps, accel +/-2 g
 *
 * dt is measured at runtime via TIM_G12 (TIMER_US mode, 1 tick = 1 us).
 * Fallback: MPU6050_YAW_PERIOD_MS (10 ms) if timer read fails.
 * ======================================================================== */

/* free-running us timer for actual dt measurement */
#define YAW_TIMER_INDEX    (TIM_G12)

static soft_iic_info_struct mpu_iic;

/* ---- calibration / bias ---- */
static float gyro_bias_x_dps = 0.0f;
static float gyro_bias_y_dps = 0.0f;
static float gyro_bias_z_dps = 0.0f;

/* Sensor-frame unit vector parallel to the vehicle vertical axis. */
static float mount_axis_x = 0.0f;
static float mount_axis_y = 0.0f;
static float mount_axis_z = 1.0f;

static float latest_gx_dps = 0.0f;
static float latest_gy_dps = 0.0f;
static float latest_gz_dps = 0.0f;

/* ---- yaw state ---- */
static float yaw_deg       = 0.0f;
static float yaw_total_deg = 0.0f;
static float filtered_gz   = 0.0f;   /* latest gz after dead-zone & limit */
static float last_gz_valid = 0.0f;   /* previous non-zero gz for trap integr */
static uint8 is_calibrated = 0;
static uint8 is_initialized= 0;
static uint8 skip_first    = 1;       /* skip 1st update (no previous value) */

/* ---- runtime dt measurement ---- */
static uint32 last_timer_ticks   = 0;
static uint32 latest_timer_ticks = 0;
static float latest_dt_ms        = 0.0f;

/* ---- stationary detector: compare squared accel mag vs (1g)^2 ---- */
#define ACCEL_SQ_1G         (1.0f)          /* 1g^2 */
#define ACCEL_SQ_THRESH     (0.30f)         /* |a^2 - 1| < 0.30 => near 1g */

/* ========================================================================
 * internal helpers
 * ======================================================================== */

static float limit_sym (float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

static void mpu_write_reg (uint8 reg, uint8 val)
{
    soft_iic_write_8bit_register(&mpu_iic, reg, val);
}

/* read N bytes starting at reg into buf */
static void mpu_read_burst (uint8 reg, uint8 *buf, uint32 len)
{
    soft_iic_read_8bit_registers(&mpu_iic, reg, buf, len);
}

/* big-endian int16 pair -> signed raw */
static int16 be16_to_s16 (const uint8 *p)
{
    return (int16)(((uint16)p[0] << 8) | p[1]);
}

/* ---- runtime dt in seconds from free-running us counter ---- */
static float mpu_get_dt_s (uint32 timer_now)
{
    uint32 delta_ticks;
    float dt_us;

    if (0 == last_timer_ticks)
    {
        /* first reading after init/calibration: just seed */
        last_timer_ticks = timer_now;
        latest_timer_ticks = 0;
        latest_dt_ms = 0.0f;
        return 0.0f;
    }

    /* Unsigned subtraction also handles the 32-bit hardware wrap. */
    delta_ticks = timer_now - last_timer_ticks;
    last_timer_ticks = timer_now;
    latest_timer_ticks = delta_ticks;

    /*
     * TIM_G12 differs from the other timers in the SeekFree driver.  Some
     * revisions apply CPS=79 and count at 1 MHz; others still expose the
     * 80 MHz bus-clock counter and compensate inside timer_get().  Detect
     * the latter from the magnitude of a normal 10..20 ms loop interval.
     */
    if(delta_ticks > 200000U)
    {
        dt_us = (float)delta_ticks / 80.0f;
    }
    else
    {
        dt_us = (float)delta_ticks;
    }

    /* guard against wildly wrong values */
    if (dt_us < 500.0f)
    {
        latest_dt_ms = (float)MPU6050_YAW_PERIOD_MS;
        return (float)MPU6050_YAW_PERIOD_MS * 0.001f;
    }
    if (dt_us > MPU6500_DT_MAX_MS * 1000.0f)
    {
        latest_dt_ms = MPU6500_DT_MAX_MS;
        return (float)MPU6500_DT_MAX_MS * 0.001f;
    }

    latest_dt_ms = (float)dt_us * 0.001f;
    return (float)dt_us * 0.000001f;
}

/*
 * zf_driver_timer configures TIM_G12 TIMER_US with CPS=79 at 80 MHz, so the
 * hardware CTR already advances once per microsecond.  timer_get() divides
 * TIM_G12 by 80 a second time and is unsuitable for dt measurement here.
 */
static uint32 mpu_get_timer_ticks_raw (void)
{
    return timer_reg[YAW_TIMER_INDEX]->COUNTERREGS.CTR;
}

/* ========================================================================
 * init
 * ======================================================================== */

void mpu6050_yaw_init (void)
{
    uint8 who_am_i = 0;
    uint8 retry;

    soft_iic_init(&mpu_iic, MPU6050_I2C_ADDR, 10, A1, A0);
    system_delay_ms(100);

    /*
     * Do not configure or calibrate an absent device.  An unacknowledged
     * software-I2C read commonly returns 0xFF, which previously produced
     * an apparently perfect all-zero gyro after "calibration".
     */
    for(retry = 0; retry < MPU6500_PROBE_RETRIES; retry ++)
    {
        mpu_read_burst(MPU6050_RA_WHO_AM_I, &who_am_i, 1);
        if(MPU6500_WHO_AM_I == who_am_i)
        {
            break;
        }
        soft_iic_stop(&mpu_iic);
        system_delay_ms(MPU6500_PROBE_DELAY_MS);
    }
    if(MPU6500_WHO_AM_I != who_am_i)
    {
        is_initialized = 0;
        is_calibrated = 0;
        filtered_gz = 0.0f;
        return;
    }

    /* wake up & PLL clock */
    mpu_write_reg(MPU6050_RA_PWR_MGMT_1, MPU6050_CLKSEL_PLL);
    system_delay_ms(50);

    /* sample rate = 1kHz/(1+SMPLRT_DIV) */
    mpu_write_reg(MPU6050_RA_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VAL);

    /* DLPF */
    mpu_write_reg(MPU6050_RA_CONFIG, MPU6050_DLPF_CFG);

    /* gyro range */
    mpu_write_reg(MPU6050_RA_GYRO_CONFIG, MPU6050_GYRO_FS_SEL);

    /* accel range */
    mpu_write_reg(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL);

    /* start free-running us timer for runtime dt measurement */
    timer_init(YAW_TIMER_INDEX, TIMER_US);
    timer_start(YAW_TIMER_INDEX);
    last_timer_ticks = mpu_get_timer_ticks_raw();

    is_initialized = 1;
    is_calibrated  = 0;
    gyro_bias_x_dps = 0.0f;
    gyro_bias_y_dps = 0.0f;
    gyro_bias_z_dps = 0.0f;
    yaw_deg        = 0.0f;
    yaw_total_deg  = 0.0f;
    filtered_gz    = 0.0f;
    last_gz_valid  = 0.0f;
    skip_first     = 1;
}

/* ========================================================================
 * calibration — vehicle MUST be STATIONARY during this call.
 * It does not need to be level.  The mean acceleration vector records the
 * sensor orientation relative to the vehicle vertical axis.
 *
 * Reads 14 bytes from ACCEL base to capture: accel[0..5], temp[6..7],
 * gyro[8..13].  GYRO_Z is at offset 12 in the 14-byte burst.
 * ======================================================================== */

void mpu6050_yaw_calibrate (uint16 samples)
{
    int32 sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32 sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8 buf[14];   /* 14 bytes: accel 6 + temp 2 + gyro 6 */
    uint16 i;
    float mean_ax, mean_ay, mean_az;
    float accel_norm_sq;
    float accel_norm;

    if (!is_initialized) return;
    if (0 == samples) samples = MPU6500_CALIB_SAMPLES;

    for (i = 0; i < samples; i++)
    {
        mpu_read_burst(MPU6050_RA_ACCEL_XOUT_H, buf, 14);
        sum_ax += be16_to_s16(buf + 0);
        sum_ay += be16_to_s16(buf + 2);
        sum_az += be16_to_s16(buf + 4);
        sum_gx += be16_to_s16(buf + 8);
        sum_gy += be16_to_s16(buf + 10);
        sum_gz += be16_to_s16(buf + 12);  /* GYRO_Z at offset 12 in 14-byte burst */
        system_delay_ms(4);               /* ~250 Hz during calib */
    }

    mean_ax = (float)sum_ax / (float)samples;
    mean_ay = (float)sum_ay / (float)samples;
    mean_az = (float)sum_az / (float)samples;
    accel_norm_sq = mean_ax*mean_ax + mean_ay*mean_ay + mean_az*mean_az;

    /*
     * Avoid sqrtf/libm: Newton iteration is accurate enough for a unit vector.
     * Raw 1 g is near 16384, so seed with that expected magnitude.
     */
    accel_norm = MPU6050_ACCEL_SENSITIVITY;
    if(accel_norm_sq > 1.0f)
    {
        uint8 iteration;
        for(iteration = 0; iteration < 6; iteration ++)
        {
            accel_norm = 0.5f * (accel_norm + accel_norm_sq / accel_norm);
        }
        mount_axis_x = mean_ax / accel_norm;
        mount_axis_y = mean_ay / accel_norm;
        mount_axis_z = mean_az / accel_norm;
    }
    else
    {
        mount_axis_x = 0.0f;
        mount_axis_y = 0.0f;
        mount_axis_z = 1.0f;
    }

    gyro_bias_x_dps = (float)sum_gx / (float)samples / MPU6050_GYRO_SENSITIVITY;
    gyro_bias_y_dps = (float)sum_gy / (float)samples / MPU6050_GYRO_SENSITIVITY;
    gyro_bias_z_dps = (float)sum_gz / (float)samples / MPU6050_GYRO_SENSITIVITY;
    yaw_deg       = 0.0f;
    filtered_gz   = 0.0f;
    last_gz_valid = 0.0f;
    skip_first    = 1;
    is_calibrated = 1;

    /* re-seed timer so first real dt doesn't include calibration time */
    last_timer_ticks = mpu_get_timer_ticks_raw();
}

/* ========================================================================
 * full update: read accel+gyro, trapezoidal yaw integration
 *
 * Call every main-loop iteration.  dt is measured at runtime via TIM_G12.
 *
 * MPU register map from ACCEL base (0x3B):
 *   [0..1]=AX, [2..3]=AY, [4..5]=AZ, [6..7]=TEMP,
 *   [8..9]=GX, [10..11]=GY, [12..13]=GZ
 * Burst 14 bytes to capture all fields.
 * ======================================================================== */

void mpu6050_yaw_update (void)
{
    uint8 buf[14];
    float ax_g, ay_g, az_g;
    float gx_raw, gy_raw, gz_raw;
    float yaw_rate;
    float delta_yaw;
    float accel_sq;
    float dt_s;
    uint32 timer_now;

    if (!is_initialized) return;

    timer_now = mpu_get_timer_ticks_raw();

    if (skip_first)
    {
        skip_first    = 0;
        last_timer_ticks = timer_now;
        return;
    }

    dt_s = mpu_get_dt_s(timer_now);
    if (dt_s <= 0.0f) return;     /* timer not ready yet */

    /* burst read 14 bytes from ACCEL base: accel + temp + gyro */
    mpu_read_burst(MPU6050_RA_ACCEL_XOUT_H, buf, 14);

    ax_g   = (float)be16_to_s16(buf + 0)  / MPU6050_ACCEL_SENSITIVITY;
    ay_g   = (float)be16_to_s16(buf + 2)  / MPU6050_ACCEL_SENSITIVITY;
    az_g   = (float)be16_to_s16(buf + 4)  / MPU6050_ACCEL_SENSITIVITY;
    gx_raw = (float)be16_to_s16(buf + 8)  / MPU6050_GYRO_SENSITIVITY;
    gy_raw = (float)be16_to_s16(buf + 10) / MPU6050_GYRO_SENSITIVITY;
    gz_raw = (float)be16_to_s16(buf + 12) / MPU6050_GYRO_SENSITIVITY;

    latest_gx_dps = gx_raw - gyro_bias_x_dps;
    latest_gy_dps = gy_raw - gyro_bias_y_dps;
    latest_gz_dps = gz_raw - gyro_bias_z_dps;
    yaw_rate = latest_gx_dps * mount_axis_x
             + latest_gy_dps * mount_axis_y
             + latest_gz_dps * mount_axis_z;

    /* ---- dead-zone ---- */
    if ((yaw_rate > -MPU6500_DEAD_ZONE_DPS) && (yaw_rate < MPU6500_DEAD_ZONE_DPS))
    {
        yaw_rate = 0.0f;
    }

    /* ---- clip outliers ---- */
    yaw_rate   = limit_sym(yaw_rate, MPU6500_GYRO_LIMIT_DPS);
    filtered_gz = yaw_rate;

    /* ---- trapezoidal yaw integration with measured dt ---- */
    if (0.0f == last_gz_valid)
    {
        delta_yaw     = filtered_gz * dt_s;
        last_gz_valid = filtered_gz;
    }
    else
    {
        delta_yaw     = 0.5f * (last_gz_valid + filtered_gz) * dt_s;
        last_gz_valid = filtered_gz;
    }

    delta_yaw *= MPU6500_YAW_CALIBRATION_SCALE;
    delta_yaw = limit_sym(delta_yaw, MPU6500_INC_LIMIT_DEG);
    yaw_deg  += delta_yaw;
    yaw_total_deg += delta_yaw;

    /* ---- normalise to [-180, +180] ---- */
    while (yaw_deg >  180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;

    /* ---- temperature drift self-compensation (when stationary) ---- */
    /* Compare squared magnitude vs 1g^2 instead of sqrt */
    accel_sq = ax_g*ax_g + ay_g*ay_g + az_g*az_g;
    {
        float dev = accel_sq - ACCEL_SQ_1G;
        if (dev < 0.0f) dev = -dev;
        if (is_calibrated
            && (dev < ACCEL_SQ_THRESH)
            && (yaw_rate > -MPU6500_STATIONARY_GZ_DPS)
            && (yaw_rate <  MPU6500_STATIONARY_GZ_DPS))
        {
            gyro_bias_x_dps += MPU6500_DRIFT_ALPHA * (gx_raw - gyro_bias_x_dps);
            gyro_bias_y_dps += MPU6500_DRIFT_ALPHA * (gy_raw - gyro_bias_y_dps);
            gyro_bias_z_dps += MPU6500_DRIFT_ALPHA * (gz_raw - gyro_bias_z_dps);
        }
    }
}

/* ========================================================================
 * fast update: all three gyro axes, then project onto the calibrated vehicle
 * vertical axis.  A six-byte burst keeps the transaction short.
 * Call every main-loop iteration.  dt measured at runtime via TIM_G12.
 * ======================================================================== */

void mpu6050_yaw_update_fast (void)
{
    uint8 buf[6];
    float yaw_rate;
    float delta_yaw;
    float dt_s;
    uint32 timer_now;

    if (!is_initialized) return;

    timer_now = mpu_get_timer_ticks_raw();

    if (skip_first)
    {
        skip_first    = 0;
        last_timer_ticks = timer_now;
        return;
    }

    dt_s = mpu_get_dt_s(timer_now);
    if (dt_s <= 0.0f) return;     /* timer not ready yet */

    mpu_read_burst(MPU6050_RA_GYRO_XOUT_H, buf, 6);
    latest_gx_dps = (float)be16_to_s16(buf + 0) / MPU6050_GYRO_SENSITIVITY
                  - gyro_bias_x_dps;
    latest_gy_dps = (float)be16_to_s16(buf + 2) / MPU6050_GYRO_SENSITIVITY
                  - gyro_bias_y_dps;
    latest_gz_dps = (float)be16_to_s16(buf + 4) / MPU6050_GYRO_SENSITIVITY
                  - gyro_bias_z_dps;
    yaw_rate = latest_gx_dps * mount_axis_x
             + latest_gy_dps * mount_axis_y
             + latest_gz_dps * mount_axis_z;

    if ((yaw_rate > -MPU6500_DEAD_ZONE_DPS) && (yaw_rate < MPU6500_DEAD_ZONE_DPS))
    {
        yaw_rate = 0.0f;
    }

    yaw_rate    = limit_sym(yaw_rate, MPU6500_GYRO_LIMIT_DPS);
    filtered_gz = yaw_rate;

    if (0.0f == last_gz_valid)
    {
        delta_yaw     = filtered_gz * dt_s;
        last_gz_valid = filtered_gz;
    }
    else
    {
        delta_yaw     = 0.5f * (last_gz_valid + filtered_gz) * dt_s;
        last_gz_valid = filtered_gz;
    }

    delta_yaw *= MPU6500_YAW_CALIBRATION_SCALE;
    delta_yaw = limit_sym(delta_yaw, MPU6500_INC_LIMIT_DEG);
    yaw_deg  += delta_yaw;
    yaw_total_deg += delta_yaw;

    while (yaw_deg >  180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;

    /*
     * Do not adapt three independent biases in gyro-only mode.  A tilted
     * module can experience roll/pitch vibration whose yaw projection is
     * small; treating that as stationarity would corrupt the bias vector.
     */
}

/* ========================================================================
 * accessors
 * ======================================================================== */

float mpu6050_yaw_get_angle (void)
{
    return yaw_deg;
}

float mpu6050_yaw_get_total_angle (void)
{
    return yaw_total_deg;
}

void mpu6050_yaw_set_angle (float deg)
{
    yaw_deg       = deg;
    yaw_total_deg = deg;
    last_gz_valid = 0.0f;
    while (yaw_deg >  180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
}

float mpu6050_yaw_get_gz_dps (void)
{
    return filtered_gz;
}

float mpu6050_yaw_get_rate_dps (void)
{
    return filtered_gz;
}

float mpu6050_yaw_get_dt_ms (void)
{
    return latest_dt_ms;
}

uint32 mpu6050_yaw_get_timer_ticks (void)
{
    return latest_timer_ticks;
}

void mpu6050_yaw_get_mount_axis (float *x, float *y, float *z)
{
    if(x) *x = mount_axis_x;
    if(y) *y = mount_axis_y;
    if(z) *z = mount_axis_z;
}

void mpu6050_yaw_get_gyro_dps (float *x, float *y, float *z)
{
    if(x) *x = latest_gx_dps;
    if(y) *y = latest_gy_dps;
    if(z) *z = latest_gz_dps;
}

uint8 mpu6050_yaw_read_who_am_i (void)
{
    uint8 value = 0xFF;

    if(is_initialized)
    {
        mpu_read_burst(MPU6050_RA_WHO_AM_I, &value, 1);
    }
    return value;
}

uint8 mpu6050_yaw_is_ready (void)
{
    return is_initialized && is_calibrated;
}
