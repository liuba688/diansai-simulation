#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdint.h>

#include "vision_protocol.h"

#define VISION_UART_BAUDRATE                    (115200U)
#define VISION_UART_FRESH_TIMEOUT_TICKS         (20U)

typedef struct
{
    vision_target_struct target;
    uint8_t sequence;
    uint8_t link_alive;
    uint8_t target_fresh;
    uint32_t packet_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t rx_overflow_count;
    uint32_t age_ticks;
} vision_uart_snapshot_struct;

void vision_uart_init(void);
void vision_uart_process(uint32_t current_tick);
void vision_uart_get_snapshot(
    uint32_t current_tick,
    vision_uart_snapshot_struct *snapshot);
void vision_uart_send_status(const vision_car_status_struct *status);

#endif
