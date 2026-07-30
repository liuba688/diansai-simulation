#ifndef _odometer_h_
#define _odometer_h_

#include "zf_common_typedef.h"

/*
 * Encoder Odometer — cumulative distance from wheel encoder pulses.
 *
 * Loaded rolling calibration: 10 revolutions = 1950mm
 * Encoder: 2450 count/rev, AB 4x
 */

#define ODO_WHEEL_CIRCUMFERENCE_MM      (195.0f)
#define ODO_WHEEL_DIAMETER_MM           (ODO_WHEEL_CIRCUMFERENCE_MM / 3.14159265f)
#define ODO_COUNTS_PER_REV              (2450.0f)
#define ODO_MM_PER_COUNT                (ODO_WHEEL_CIRCUMFERENCE_MM / ODO_COUNTS_PER_REV)

/* Measured chassis and standard-track geometry; feedforward remains disabled. */
#define CAR_TRACK_WIDTH_MM              (209.0f)
#define CAR_LINE_SENSOR_PREVIEW_MM      (219.0f)
#define CAR_LINE_SENSOR_PITCH_MM        (10.0f)
#define CAR_STANDARD_CURVE_RADIUS_MM    (500.0f)

/* ---------- struct ---------- */
typedef struct
{
    float total_cm;          /* cumulative distance, cm */
    float left_cm;           /* left wheel alone */
    float right_cm;          /* right wheel alone */
    float target_cm;         /* one-shot trigger distance */
    uint8 target_reached;    /* set to 1 when total >= target */
} odometer_struct;

/* ---------- API ---------- */
void  odometer_init   (odometer_struct *odo);
void  odometer_update (odometer_struct *odo, int32 left_delta, int32 right_delta);
void  odometer_set_target (odometer_struct *odo, float distance_cm);
void  odometer_reset  (odometer_struct *odo);
float odometer_get_cm (const odometer_struct *odo);
uint8 odometer_is_target_reached (odometer_struct *odo);

#endif
