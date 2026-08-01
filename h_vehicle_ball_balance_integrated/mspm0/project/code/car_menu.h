#ifndef CAR_MENU_H
#define CAR_MENU_H

#include "zf_common_headfile.h"

#define CAR_MENU_KEY_UP_PIN       (B13)
#define CAR_MENU_KEY_DOWN_PIN     (B23)
#define CAR_MENU_KEY_OK_PIN       (B26)
#define CAR_MENU_KEY_BACK_PIN     (B27)
#define CAR_MENU_VISIBLE_ROWS     (4U)
#define CAR_MENU_ITEM_COUNT       (8U)

typedef enum
{
    CAR_SPEED_TIER_CONSERVATIVE = 0,
    CAR_SPEED_TIER_NORMAL,
    CAR_SPEED_TIER_FAST,
    CAR_SPEED_TIER_SPRINT,
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

typedef enum
{
    CAR_TASK_NONE = 0,
    CAR_TASK_1_VIDEO = 1,
    CAR_TASK_2_RACE,
    CAR_TASK_3_ROUND_TRIP,
    CAR_TASK_4_AB_BALANCE,
    CAR_TASK_5_LAP_BALANCE,
    CAR_TASK_6_ARBITRARY,
    CAR_TASK_RESET_YAW,
    CAR_TASK_DIAGNOSTIC,
    CAR_TASK_STOP = 250,
    CAR_TASK_START = 251,

    /* Compatibility aliases used by the proven chassis implementation. */
    CAR_TASK_RACE_LINE = CAR_TASK_2_RACE,
    CAR_TASK_STATIC_BALL = CAR_TASK_3_ROUND_TRIP,
    CAR_TASK_AB_BALANCE = CAR_TASK_4_AB_BALANCE,
    CAR_TASK_LAP_BALANCE = CAR_TASK_5_LAP_BALANCE,
    CAR_TASK_ODOMETER_QUERY = CAR_TASK_DIAGNOSTIC,
    CAR_TASK_SPEED_TIER = 30,
    CAR_TASK_ANGLE_HOLD = 31
} car_task_t;

typedef void (*car_menu_clear_fn)(uint8 pattern);
typedef void (*car_menu_text_fn)(uint8 page, const char *text);

void car_menu_init(car_menu_clear_fn clear_screen, car_menu_text_fn show_text);
car_task_t car_menu_update(void);
void car_menu_open(void);
void car_menu_close(void);
void car_menu_render(void);
uint8 car_menu_is_open(void);
const char *car_menu_task_name(car_task_t task);
car_speed_tier_t car_menu_get_speed_tier(void);
const car_speed_tier_params_struct *car_menu_get_speed_tier_params(
    car_speed_tier_t tier);
const char *car_menu_speed_tier_name(car_speed_tier_t tier);

#endif
