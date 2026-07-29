#ifndef BALL_CAR_H
#define BALL_CAR_H

#include <stdint.h>

#define BALL_CAR_CONTROL_PERIOD_MS              (10U)
#define BALL_CAR_HISTORY_SAMPLE_TICKS           (2U)
#define BALL_CAR_HISTORY_CAPACITY               (512U)

#define BALL_CAR_MIN_CONTROL_CONFIDENCE         (60U)
#define BALL_CAR_MIN_WARNING_CONFIDENCE         (50U)
#define BALL_CAR_WARNING_SPEED_SCALE            (1.0f / 3.0f)
#define BALL_CAR_TARGET_CONFIRM_TICKS           (10U)
#define BALL_CAR_STOP_LOCK_TICKS                (20U)
#define BALL_CAR_TARGET_LOST_CONFIRM_TICKS      (10U)
#define BALL_CAR_APPROACH_TIMEOUT_TICKS         (750U)
#define BALL_CAR_FINAL_CREEP_TICKS              (200U)
#define BALL_CAR_LINE_CONFIRM_TICKS             (3U)
#define BALL_CAR_REACQUIRE_SWEEP_TICKS          (60U)
#define BALL_CAR_REACQUIRE_TIMEOUT_TICKS        (300U)

#define BALL_CAR_APPROACH_NEAR_PERCENT          (30U)
#define BALL_CAR_APPROACH_FAR_RPM               (28.0f)
#define BALL_CAR_APPROACH_NEAR_RPM              (13.0f)
#define BALL_CAR_APPROACH_MAX_RPM               (38.0f)
#define BALL_CAR_APPROACH_TURN_KP               (0.18f)
#define BALL_CAR_APPROACH_X_DEADBAND             (8)
#define BALL_CAR_VISION_STEERING_SIGN            (1.0f)
#define BALL_CAR_FINAL_CREEP_RPM                 (11.0f)
#define BALL_CAR_REACQUIRE_RPM                   (12.0f)

typedef enum
{
    BALL_CAR_STATE_IDLE = 0,
    BALL_CAR_STATE_LINE_FOLLOW,
    BALL_CAR_STATE_TARGET_CONFIRM,
    BALL_CAR_STATE_STOP_LOCK,
    BALL_CAR_STATE_APPROACH,
    BALL_CAR_STATE_FINAL_CREEP,
    BALL_CAR_STATE_RETURN_PREPARE,
    BALL_CAR_STATE_BACKTRACK,
    BALL_CAR_STATE_REACQUIRE_LINE,
    BALL_CAR_STATE_COMPLETE,
    BALL_CAR_STATE_FAULT,
} ball_car_state_enum;

typedef enum
{
    BALL_CAR_FAULT_NONE = 0,
    BALL_CAR_FAULT_VISION_LOST,
    BALL_CAR_FAULT_APPROACH_TIMEOUT,
    BALL_CAR_FAULT_HISTORY_OVERFLOW,
    BALL_CAR_FAULT_REACQUIRE_TIMEOUT,
} ball_car_fault_enum;

typedef struct
{
    int16_t left_rpm_x10;
    int16_t right_rpm_x10;
} ball_car_history_entry_struct;

typedef struct
{
    uint8_t enabled;
    uint8_t line_valid;
    int16_t line_error;
    float line_left_rpm;
    float line_right_rpm;

    uint8_t vision_link_alive;
    uint8_t vision_valid;
    uint8_t vision_confirmed;
    uint8_t vision_close;
    uint8_t vision_confidence;
    uint16_t vision_center_x;
    uint16_t vision_center_y;
    uint16_t vision_width;
    uint16_t vision_height;
    uint16_t vision_frame_width;
    uint16_t vision_frame_height;
} ball_car_input_struct;

typedef struct
{
    float left_target_rpm;
    float right_target_rpm;
    ball_car_state_enum state;
    ball_car_fault_enum fault;
} ball_car_output_struct;

typedef struct
{
    ball_car_state_enum state;
    ball_car_fault_enum fault;
    uint16_t state_ticks;
    uint16_t target_confirm_ticks;
    uint16_t target_lost_ticks;
    uint16_t line_confirm_ticks;
    int16_t departure_line_error;
    int16_t last_target_error;
    uint16_t history_count;
    uint16_t replay_index;
    uint8_t history_sample_ticks;
    uint8_t replay_hold_ticks;
    float replay_left_rpm;
    float replay_right_rpm;
    ball_car_history_entry_struct history[BALL_CAR_HISTORY_CAPACITY];
} ball_car_struct;

void ball_car_init(ball_car_struct *car);
uint8_t ball_car_requires_line_follow(const ball_car_struct *car);
void ball_car_update(
    ball_car_struct *car,
    const ball_car_input_struct *input,
    ball_car_output_struct *output);
const char *ball_car_state_name(ball_car_state_enum state);
const char *ball_car_fault_name(ball_car_fault_enum fault);

#endif
