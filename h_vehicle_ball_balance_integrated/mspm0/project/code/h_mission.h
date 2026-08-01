#ifndef H_MISSION_H
#define H_MISSION_H

#include <stdint.h>
#include "ball_balance.h"
#include "car_menu.h"
#include "vision_uart.h"

typedef enum
{
    H_MISSION_MENU = 0,
    H_MISSION_CONFIGURING,
    H_MISSION_READY,
    H_MISSION_STARTING,
    H_MISSION_RUNNING,
    H_MISSION_FINISHED,
    H_MISSION_FAULT
} h_mission_state_t;

typedef enum
{
    H_ROUTE_NONE = 0,
    H_ROUTE_RACE_LINE,
    H_ROUTE_AB_150_CM
} h_route_t;

typedef struct
{
    h_mission_state_t state;
    car_task_t task;
    h_route_t route;
    uint8_t run_id;
    uint8_t fault_code;
    uint8_t camera_ready;
    uint8_t camera_started;
    uint8_t last_event_sequence;
    uint8_t last_event_valid;
    uint32_t state_tick;
    uint32_t start_tick;
    uint32_t stop_tick;
    uint32_t last_command_tick;
    uint8_t command_retries;
    int16_t target_x10_mm;
    ball_balance_t balance;
} h_mission_t;

void h_mission_init(h_mission_t *mission);
uint8_t h_mission_prepare(h_mission_t *mission, car_task_t task,
                          car_speed_tier_t speed, int16_t target_x10_mm,
                          uint32_t tick);
uint8_t h_mission_start(h_mission_t *mission, car_speed_tier_t speed,
                        uint32_t tick);
void h_mission_update(h_mission_t *mission,
                      const vision_uart_snapshot_struct *vision,
                      car_speed_tier_t speed, uint32_t tick);
void h_mission_stop(h_mission_t *mission, uint8_t reason, uint32_t tick);
void h_mission_finish_chassis(h_mission_t *mission, uint32_t tick);
uint8_t h_mission_car_allowed(const h_mission_t *mission);
uint8_t h_mission_needs_balance(car_task_t task);
uint8_t h_mission_is_ready(const h_mission_t *mission);

#endif
