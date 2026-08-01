#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdint.h>
#include "vision_protocol.h"

#define VISION_UART_BAUDRATE             (115200U)
#define VISION_UART_LINK_TIMEOUT_TICKS   (50U)
#define VISION_UART_BALL_TIMEOUT_TICKS   (30U)

typedef struct
{
    vision_ball_state_struct ball;
    vision_event_struct event;
    uint8_t event_type;
    uint8_t event_sequence;
    uint8_t link_alive;
    uint8_t ball_fresh;
    uint32_t packet_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t rx_overflow_count;
    uint32_t link_age_ticks;
    uint32_t ball_age_ticks;
} vision_uart_snapshot_struct;

void vision_uart_init(void);
void vision_uart_process(uint32_t current_tick);
void vision_uart_get_snapshot(uint32_t current_tick,
                              vision_uart_snapshot_struct *snapshot);
void vision_uart_send_command(uint8_t type, uint8_t task_id, uint8_t run_id,
                              int16_t target_x10_mm, uint8_t speed_tier,
                              uint8_t flags, uint32_t timestamp_ms);

#endif
