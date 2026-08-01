#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdint.h>
#include "vision_protocol.h"

typedef enum
{
    BALL_MODE_OFF = 0,
    BALL_MODE_TASK3_ROUND_TRIP,
    BALL_MODE_CENTER_HOLD,
    BALL_MODE_ARBITRARY_RESERVED
} ball_balance_mode_t;

typedef enum
{
    BALL_STATE_IDLE = 0,
    BALL_STATE_WAIT_BALL,
    BALL_STATE_HOLD_CENTER,
    BALL_STATE_POS_PUSH,
    BALL_STATE_POS_BRAKE,
    BALL_STATE_POS_SETTLE,
    BALL_STATE_NEG_PUSH,
    BALL_STATE_NEG_BRAKE,
    BALL_STATE_NEG_SETTLE,
    BALL_STATE_RECOVER_CENTER,
    BALL_STATE_COMPLETE,
    BALL_STATE_SAFE_STOP
} ball_balance_state_t;

typedef struct
{
    ball_balance_mode_t mode;
    ball_balance_state_t state;
    uint32_t state_tick;
    uint32_t start_tick;
    uint32_t stable_tick;
    uint32_t stiction_tick;
    uint8_t stiction_active;
    uint8_t quiet_hold_active;
    uint8_t fault_code;
    int16_t motor_target_pulses;
    float requested_angle_deg;
} ball_balance_t;

void ball_balance_init(ball_balance_t *control);
void ball_balance_start(ball_balance_t *control, ball_balance_mode_t mode,
                        uint32_t tick);
void ball_balance_stop(ball_balance_t *control);
void ball_balance_update(ball_balance_t *control,
                         const vision_ball_state_struct *ball,
                         uint8_t valid, uint32_t tick);
uint8_t ball_balance_is_complete(const ball_balance_t *control);
uint8_t ball_balance_has_fault(const ball_balance_t *control);

#endif
