#ifndef CAR_MENU_H
#define CAR_MENU_H

#include "zf_common_headfile.h"

/*
 * Four active-low keys. These pins are routed out on the carrier board, but
 * the actual switch-to-pin order must still be confirmed on the real car.
 * Change only these four macros if the measured order is different.
 */
#define CAR_MENU_KEY_UP_PIN       (B13)
#define CAR_MENU_KEY_DOWN_PIN     (B23)
#define CAR_MENU_KEY_OK_PIN       (B26)
#define CAR_MENU_KEY_BACK_PIN     (B27)

/* Use the Tianmuxing board's on-board B21 key as a single-key menu. */
#define CAR_MENU_SINGLE_B21_MODE  (0U)
#define CAR_MENU_SINGLE_B21_PIN   (B21)

#define CAR_MENU_KEY_DIAGNOSTIC_MODE (0U)

#define CAR_MENU_VISIBLE_ROWS     (4U)
#define CAR_MENU_ITEM_COUNT       (8U)

/*
 * Speed tier presets.  Select a tier from the menu and the matching
 * line-following + speed-PID parameters are loaded when a race task starts.
 */
typedef enum
{
    CAR_SPEED_TIER_CONSERVATIVE = 0,   /* 120 RPM straight, 75-90 curve */
    CAR_SPEED_TIER_NORMAL,             /* 140 RPM straight, 82-100 curve */
    CAR_SPEED_TIER_FAST,               /* 160 RPM straight, 92-115 curve */
    CAR_SPEED_TIER_SPRINT,             /* 175 RPM straight, 100-125 curve */
    CAR_SPEED_TIER_COUNT
} car_speed_tier_t;

typedef struct
{
    float straight_rpm;
    float max_curve_rpm;
    float min_curve_rpm;
    float line_kp;
    float speed_kp;
    float speed_ki;
    float speed_kd;
} car_speed_tier_params_struct;

/*
 * Task identifiers for the four-key OLED menu.
 *
 * CALIBRATE is first because the IMU must be calibrated after power-on
 * before any race task can run reliably.
 */
typedef enum
{
    CAR_TASK_NONE = 0,
    CAR_TASK_STOP,

    /* Competition tasks */
    CAR_TASK_RESET_YAW,           /* CALIBRATE: IMU yaw calibration (always first) */
    CAR_TASK_RACE_LINE,           /* basic line-follow one lap + timer + stop */
    CAR_TASK_STATIC_BALL,         /* static ball O <-> +5/-5 cm */
    CAR_TASK_AB_BALANCE,          /* A-B dynamic balance 1.5 m */
    CAR_TASK_LAP_BALANCE,         /* full-lap centre / arbitrary position balance */

    /* Diagnostics & settings */
    CAR_TASK_ANGLE_HOLD,          /* hold current heading */
    CAR_TASK_ODOMETER_QUERY,      /* show accumulated odometer */
    CAR_TASK_SPEED_TIER,          /* cycle through speed presets */

    /* Legacy / deprecated — keep for backward compat */
    CAR_TASK_LINE_FOLLOW = 20,    /* was BALL VISION; H题 no longer needs it */
    CAR_TASK_PLAIN_LINE = CAR_TASK_RACE_LINE,
    CAR_TASK_HELP
} car_task_t;

typedef void (*car_menu_clear_fn)(uint8 pattern);
typedef void (*car_menu_text_fn)(uint8 page, const char *text);

void                         car_menu_init               (car_menu_clear_fn clear_screen,
                                                          car_menu_text_fn show_text);
car_task_t                   car_menu_update             (void);
void                         car_menu_open               (void);
void                         car_menu_close              (void);
void                         car_menu_render             (void);
uint8                        car_menu_is_open            (void);
const char                  *car_menu_task_name          (car_task_t task);

/* Speed tier API */
car_speed_tier_t             car_menu_get_speed_tier     (void);
const car_speed_tier_params_struct
                            *car_menu_get_speed_tier_params(car_speed_tier_t tier);

#endif
