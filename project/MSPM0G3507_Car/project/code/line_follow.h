#ifndef _line_follow_h_
#define _line_follow_h_

#include "zf_common_typedef.h"
#include "line_sensor.h"

#define LINE_FOLLOW_STRAIGHT_RPM             (70.0f)
#define LINE_FOLLOW_MIN_CURVE_RPM            (30.0f)
#define LINE_FOLLOW_SPEED_ERROR_GAIN         (0.10f)
#define LINE_FOLLOW_SPEED_DERROR_GAIN        (0.10f)
#define LINE_FOLLOW_SPEED_RECOVERY_RPM       (1.0f)

#define LINE_FOLLOW_KP                       (0.13f)
#define LINE_FOLLOW_KD                       (0.20f)
#define LINE_FOLLOW_CORRECTION_MAX_RPM       (55.0f)

#define LINE_FOLLOW_SHARP_CONFIRM_TICKS      (2)
#define LINE_FOLLOW_SHARP_RELEASE_TICKS      (3)
#define LINE_FOLLOW_SHARP_OUTER_RPM          (28.0f)
#define LINE_FOLLOW_SHARP_INNER_RPM          (-12.0f)

#define LINE_FOLLOW_LOST_SEARCH_TICKS        (20)
#define LINE_FOLLOW_LOST_OUTER_RPM           (22.0f)
#define LINE_FOLLOW_LOST_INNER_RPM           (-10.0f)

typedef enum
{
    LINE_FOLLOW_MODE_NORMAL = 0,
    LINE_FOLLOW_MODE_SHARP_LEFT,
    LINE_FOLLOW_MODE_SHARP_RIGHT,
    LINE_FOLLOW_MODE_LOST_SEARCH,
    LINE_FOLLOW_MODE_LOST_STOP,
} line_follow_mode_enum;

typedef struct
{
    int16 previous_error;
    int16 last_valid_error;
    uint8 sharp_left_ticks;
    uint8 sharp_right_ticks;
    uint8 sharp_release_ticks;
    uint8 lost_ticks;
    line_follow_mode_enum mode;
    float base_rpm;
    float correction_rpm;
} line_follow_struct;

void line_follow_init (line_follow_struct *follow);
void line_follow_update (line_follow_struct *follow,
                         const line_sensor_data_struct *sensor,
                         float *left_target_rpm,
                         float *right_target_rpm);

#endif
