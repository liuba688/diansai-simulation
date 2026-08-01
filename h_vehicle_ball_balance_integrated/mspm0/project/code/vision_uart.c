#include "vision_uart.h"

#include <stddef.h>
#include <string.h>
#include "zf_driver_uart.h"

#define RX_SIZE (128U)
#define RX_MASK (RX_SIZE - 1U)

static volatile uint8_t rx_buffer[RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint32_t overflow_count;
static vision_protocol_parser_struct parser;
static vision_ball_state_struct latest_ball;
static vision_event_struct latest_event;
static uint8_t latest_event_type, latest_event_sequence, tx_sequence;
static uint32_t last_packet_tick, last_ball_tick, packet_count;
static uint32_t crc_error_count, format_error_count;
static uint8_t packet_seen, ball_seen;

static void rx_callback(uint32 event, void *context)
{
    uint8_t byte, next;
    (void)event; (void)context;
    while(uart_query_byte(UART_2, &byte))
    {
        next = (uint8_t)((rx_head + 1U) & RX_MASK);
        if(next == rx_tail) overflow_count++;
        else { rx_buffer[rx_head] = byte; rx_head = next; }
    }
}

void vision_uart_init(void)
{
    rx_head = rx_tail = 0U;
    overflow_count = last_packet_tick = last_ball_tick = packet_count = 0U;
    crc_error_count = format_error_count = 0U;
    latest_event_type = latest_event_sequence = tx_sequence = 0U;
    packet_seen = ball_seen = 0U;
    memset(&latest_ball, 0, sizeof(latest_ball));
    memset(&latest_event, 0, sizeof(latest_event));
    vision_protocol_parser_init(&parser);
    uart_init(UART_2, VISION_UART_BAUDRATE, UART2_TX_A23, UART2_RX_A24);
    uart_set_callback(UART_2, rx_callback, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void vision_uart_process(uint32_t current_tick)
{
    uint8_t byte;
    vision_packet_struct packet;
    vision_parse_result_enum result;
    vision_ball_state_struct ball;
    vision_event_struct event;
    while(rx_tail != rx_head)
    {
        byte = rx_buffer[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1U) & RX_MASK);
        result = vision_protocol_feed(&parser, byte, &packet);
        if(VISION_PARSE_PACKET == result)
        {
            if(VISION_PROTOCOL_VERSION != packet.version)
            { format_error_count++; continue; }
            packet_seen = 1U; packet_count++; last_packet_tick = current_tick;
            if(vision_protocol_decode_ball(&packet, &ball))
            {
                latest_ball = ball; last_ball_tick = current_tick; ball_seen = 1U;
            }
            else if(vision_protocol_decode_event(&packet, &event))
            {
                latest_event = event;
                latest_event_type = packet.type;
                latest_event_sequence = packet.sequence;
            }
            else format_error_count++;
        }
        else if(VISION_PARSE_CRC_ERROR == result) crc_error_count++;
        else if(VISION_PARSE_FORMAT_ERROR == result) format_error_count++;
    }
}

void vision_uart_get_snapshot(uint32_t tick, vision_uart_snapshot_struct *out)
{
    if(NULL == out) return;
    memset(out, 0, sizeof(*out));
    out->ball = latest_ball; out->event = latest_event;
    out->event_type = latest_event_type;
    out->event_sequence = latest_event_sequence;
    out->link_age_ticks = packet_seen ? (tick - last_packet_tick) : 0xFFFFFFFFU;
    out->ball_age_ticks = ball_seen ? (tick - last_ball_tick) : 0xFFFFFFFFU;
    out->link_alive = packet_seen && out->link_age_ticks <= VISION_UART_LINK_TIMEOUT_TICKS;
    out->ball_fresh = ball_seen && out->ball_age_ticks <= VISION_UART_BALL_TIMEOUT_TICKS
        && (latest_ball.flags & VISION_BALL_FLAG_VALID);
    out->packet_count = packet_count; out->crc_error_count = crc_error_count;
    out->format_error_count = format_error_count;
    out->rx_overflow_count = overflow_count;
}

void vision_uart_send_command(uint8_t type, uint8_t task_id, uint8_t run_id,
                              int16_t target_x10_mm, uint8_t speed_tier,
                              uint8_t flags, uint32_t timestamp_ms)
{
    uint8_t packet[VISION_PROTOCOL_MAX_PACKET_LENGTH];
    uint8_t length = vision_protocol_build_command(
        type, tx_sequence++, task_id, run_id, target_x10_mm,
        speed_tier, flags, timestamp_ms, packet, sizeof(packet));
    if(length) uart_write_buffer(UART_2, packet, length);
}
