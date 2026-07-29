#ifndef CAR_MENU_H
#define CAR_MENU_H

#include "zf_common_headfile.h"

/*
 * Four active-low keys. These pins are routed out on the carrier board, but
 * the actual switch-to-pin order must still be confirmed on the real car.
 * Change only these four macros if the measured order is different.
 */
#define CAR_MENU_KEY_UP_PIN       (B8)
#define CAR_MENU_KEY_DOWN_PIN     (B9)
#define CAR_MENU_KEY_OK_PIN       (B10)
#define CAR_MENU_KEY_BACK_PIN     (B11)

#define CAR_MENU_VISIBLE_ROWS     (4U)
#define CAR_MENU_ITEM_COUNT       (8U)

typedef enum
{
    CAR_TASK_NONE = 0,
    CAR_TASK_STOP,
    CAR_TASK_LINE_FOLLOW,
    CAR_TASK_ANGLE_HOLD,
    CAR_TASK_PLAIN_LINE,
    CAR_TASK_RESERVED_04,
    CAR_TASK_RESERVED_05,
    CAR_TASK_RESERVED_06,
    CAR_TASK_RESERVED_07,
    CAR_TASK_RESERVED_08,
    CAR_TASK_RESET_YAW = 20,
    CAR_TASK_ODOMETER_QUERY,
    CAR_TASK_HELP
} car_task_t;

typedef void (*car_menu_clear_fn)(uint8 pattern);
typedef void (*car_menu_text_fn)(uint8 page, const char *text);

void        car_menu_init               (car_menu_clear_fn clear_screen,
                                         car_menu_text_fn show_text);
car_task_t  car_menu_update             (void);
void        car_menu_open               (void);
void        car_menu_close              (void);
void        car_menu_render             (void);
uint8       car_menu_is_open            (void);
const char *car_menu_task_name          (car_task_t task);

#endif
