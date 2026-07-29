#ifndef _odometer_h_
#define _odometer_h_

#include "zf_common_typedef.h"

/*
 * Encoder Odometer — cumulative distance from wheel encoder pulses.
 *
 * Wheel: 66mm dia => 207.3mm circumference
 * Encoder: 2450 count/rev, AB 4x
 */

#define ODO_WHEEL_DIAMETER_MM           (66.0f)
#define ODO_WHEEL_CIRCUMFERENCE_MM      (3.14159265f * ODO_WHEEL_DIAMETER_MM)
#define ODO_COUNTS_PER_REV              (2450.0f)
#define ODO_MM_PER_COUNT                (ODO_WHEEL_CIRCUMFERENCE_MM / ODO_COUNTS_PER_REV)

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
