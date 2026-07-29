#include "odometer.h"

/* ========================================================================
 * Odometer — cumulative distance from encoder deltas
 *
 * Uses absolute value of each wheel delta (forward always adds distance).
 * ======================================================================== */

void odometer_init (odometer_struct *odo)
{
    odo->total_cm      = 0.0f;
    odo->left_cm       = 0.0f;
    odo->right_cm      = 0.0f;
    odo->target_cm     = 0.0f;
    odo->target_reached = 0;
}

void odometer_update (odometer_struct *odo, int32 left_delta, int32 right_delta)
{
    float left_mm, right_mm;
    float l, r;

    /* absolute deltas: only count distance, not direction */
    l = (left_delta  < 0) ? (float)(-left_delta)  : (float)left_delta;
    r = (right_delta < 0) ? (float)(-right_delta) : (float)right_delta;

    left_mm   = l * ODO_MM_PER_COUNT;
    right_mm  = r * ODO_MM_PER_COUNT;

    odo->left_cm  += left_mm  * 0.1f;
    odo->right_cm += right_mm * 0.1f;
    odo->total_cm += (left_mm + right_mm) * 0.5f * 0.1f;

    /* check target */
    if ((odo->target_cm > 0.01f) && (odo->total_cm >= odo->target_cm))
    {
        odo->target_reached = 1;
    }
}

void odometer_set_target (odometer_struct *odo, float distance_cm)
{
    odo->target_cm      = distance_cm;
    odo->target_reached = (odo->total_cm >= distance_cm) ? 1 : 0;
}

void odometer_reset (odometer_struct *odo)
{
    odometer_init(odo);
}

float odometer_get_cm (const odometer_struct *odo)
{
    return odo->total_cm;
}

uint8 odometer_is_target_reached (odometer_struct *odo)
{
    return odo->target_reached;
}
