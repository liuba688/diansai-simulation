#ifndef _line_follow_h_
#define _line_follow_h_

#include "zf_common_typedef.h"
#include "line_sensor.h"

#define LINE_FOLLOW_STRAIGHT_RPM             (160.0f)
#define LINE_FOLLOW_MAX_CURVE_RPM            (115.0f)
#define LINE_FOLLOW_MIN_CURVE_RPM            (92.0f)
#define LINE_FOLLOW_SPEED_ERROR_GAIN         (0.18f)
#define LINE_FOLLOW_SPEED_DERROR_GAIN        (0.10f)
#define LINE_FOLLOW_SPEED_RECOVERY_RPM       (2.0f)
#define LINE_FOLLOW_CURVE_ENTER_STRENGTH     (90)
#define LINE_FOLLOW_CURVE_EXIT_STRENGTH      (60)
#define LINE_FOLLOW_CURVE_ENTER_TICKS        (8)
#define LINE_FOLLOW_CURVE_EXIT_TICKS         (5)
#define LINE_FOLLOW_CURVE_EXIT_YAW_DEG       (160.0f)
#define LINE_FOLLOW_CURVE_START_INHIBIT_TICKS (50)
#define LINE_FOLLOW_ARC_FUSION_ENABLE         (0)
#define LINE_FOLLOW_CURVE_ABORT_ERROR          (150)
#define LINE_FOLLOW_CURVE_ABORT_TICKS          (5)

#define LINE_FOLLOW_KP                       (0.20f)
#define LINE_FOLLOW_KD                       (0.20f)
#define LINE_FOLLOW_CORRECTION_MAX_RPM       (75.0f)

/* Standard 500mm-radius curve feedforward for the measured 209mm track. */
#define LINE_FOLLOW_CURVE_RADIUS_MM           (500.0f)
#define LINE_FOLLOW_TRACK_WIDTH_MM            (209.0f)
#define LINE_FOLLOW_CURVE_CENTER_RPM          (90.0f)
/* Clockwise lap: + means left wheel faster and right wheel slower. */
#define LINE_FOLLOW_TRACK_TURN_DIRECTION      (1.0f)
#define LINE_FOLLOW_CURVE_HALF_DIFF_RPM       \
    (LINE_FOLLOW_CURVE_CENTER_RPM             \
     * LINE_FOLLOW_TRACK_WIDTH_MM             \
     / (2.0f * LINE_FOLLOW_CURVE_RADIUS_MM))
#define LINE_FOLLOW_CURVE_FEEDBACK_KP         (0.04f)
#define LINE_FOLLOW_CURVE_FEEDBACK_KD         (0.05f)
#define LINE_FOLLOW_CURVE_FEEDBACK_MAX_RPM    (8.0f)

#define LINE_FOLLOW_SHARP_ENABLE             (0)
#define LINE_FOLLOW_SHARP_CONFIRM_TICKS      (5)
#define LINE_FOLLOW_SHARP_RELEASE_TICKS      (5)
#define LINE_FOLLOW_SHARP_OUTER_RPM          (50.0f)
#define LINE_FOLLOW_SHARP_INNER_RPM          (-12.0f)

#define LINE_FOLLOW_LOST_SEARCH_TICKS        (30)
#define LINE_FOLLOW_LOST_CONFIRM_TICKS       (5)
#define LINE_FOLLOW_LOST_COAST_OUTER_RPM     (60.0f)
#define LINE_FOLLOW_LOST_COAST_INNER_RPM     (45.0f)
#define LINE_FOLLOW_LOST_FORWARD_TICKS       (30)
#define LINE_FOLLOW_LOST_FORWARD_OUTER_RPM   (55.0f)
#define LINE_FOLLOW_LOST_FORWARD_INNER_RPM   (20.0f)
#define LINE_FOLLOW_LOST_OUTER_RPM           (35.0f)
#define LINE_FOLLOW_LOST_INNER_RPM           (-20.0f)
#define LINE_FOLLOW_LOST_BRAKE_TICKS         (5)
#define LINE_FOLLOW_SPIN_YAW_DPS_THRESHOLD   (50.0f)

/* Track geometry for odometry-based phase detection (all in cm). */
#define TRACK_STRAIGHT_LENGTH_CM           (150.0f)
#define TRACK_CURVE_ARC_LENGTH_CM          (157.1f)   /* π × 50 cm */
#define TRACK_HALF_LAP_CM                  (TRACK_STRAIGHT_LENGTH_CM + TRACK_CURVE_ARC_LENGTH_CM)
#define TRACK_FULL_LAP_CM                  (2.0f * TRACK_HALF_LAP_CM)
#define TRACK_PHASE_HYSTERESIS_CM          (5.0f)
#define TRACK_CURVE_APPROACH_ZONE_CM      (20.0f)

typedef enum
{
    LINE_FOLLOW_MODE_NORMAL = 0,
    LINE_FOLLOW_MODE_SHARP_LEFT,
    LINE_FOLLOW_MODE_SHARP_RIGHT,
    LINE_FOLLOW_MODE_LOST_SEARCH,
    LINE_FOLLOW_MODE_LOST_STOP,
} line_follow_mode_enum;

typedef enum
{
    TRACK_PHASE_STRAIGHT_1 = 0,
    TRACK_PHASE_CURVE_1,
    TRACK_PHASE_STRAIGHT_2,
    TRACK_PHASE_CURVE_2,
    TRACK_PHASE_COUNT
} track_phase_enum;

/*
 * Runtime-overridable tunables.  Compile-time macros are defaults; speed-tier
 * selection copies new values here before a run starts.
 */
typedef struct
{
    float straight_rpm;
    float max_curve_rpm;
    float min_curve_rpm;
    float kp;
    float kd;
    float correction_max_rpm;
} line_follow_params_struct;

typedef struct
{
    int16 previous_error;
    int16 last_valid_error;
    uint8 sharp_left_ticks;
    uint8 sharp_right_ticks;
    uint8 sharp_release_ticks;
    uint8 lost_ticks;
    uint8 lost_brake_ticks;
    uint16 loss_events;
    uint8 edge_ticks;
    uint16 edge_events;
    uint16 edge_max_ticks;
    uint8 curve_active;
    uint8 curve_aborted;
    uint8 curve_abort_ticks;
    uint8 curve_enter_ticks;
    uint8 curve_exit_ticks;
    uint16 run_ticks;
    float curve_start_yaw_deg;
    float curve_yaw_progress_deg;
    line_follow_mode_enum mode;
    track_phase_enum track_phase;
    float base_rpm;
    float correction_rpm;
    float curve_direction;
    line_follow_params_struct params;
} line_follow_struct;

void line_follow_init (line_follow_struct *follow);
void line_follow_set_params (line_follow_struct *follow,
                             const line_follow_params_struct *params);
void line_follow_update (line_follow_struct *follow,
                         const line_sensor_data_struct *sensor,
                         float yaw_total_deg,
                         float yaw_rate_dps,
                         uint8 yaw_ready,
                         float distance_cm,
                         float *left_target_rpm,
                         float *right_target_rpm);

#endif
