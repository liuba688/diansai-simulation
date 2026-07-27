#include "vision_uart.h"

#include <stddef.h>
#include <string.h>

#include "zf_driver_uart.h"

#define VISION_UART_RX_BUFFER_SIZE              (128U)
#define VISION_UART_RX_BUFFER_MASK              (VISION_UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t vision_rx_buffer[VISION_UART_RX_BUFFER_SIZE];
static volatile uint8_t vision_rx_head = 0U;
static volatile uint8_t vision_rx_tail = 0U;
static volatile uint32_t vision_rx_overflow_count = 0U;

static vision_protocol_parser_struct vision_parser;
static vision_target_struct vision_latest_target;
static uint8_t vision_latest_sequence = 0U;
static uint8_t vision_packet_received = 0U;
static uint8_t vision_status_sequence = 0U;
static uint32_t vision_last_packet_tick = 0U;
static uint32_t vision_packet_count = 0U;
static uint32_t vision_crc_error_count = 0U;
static uint32_t vision_format_error_count = 0U;

static void vision_uart_rx_callback(uint32 event, void *context)
{
    uint8_t byte;
    uint8_t next_head;

    (void)event;
    (void)context;

    while(0U != uart_query_byte(UART_2, &byte))
    {
        next_head = (uint8_t)((vision_rx_head + 1U)
                            & VISION_UART_RX_BUFFER_MASK);
        if(next_head == vision_rx_tail)
        {
            vision_rx_overflow_count++;
        }
        else
        {
            vision_rx_buffer[vision_rx_head] = byte;
            vision_rx_head = next_head;
        }
    }
}

void vision_uart_init(void)
{
    vision_rx_head = 0U;
    vision_rx_tail = 0U;
    vision_rx_overflow_count = 0U;
    vision_packet_received = 0U;
    vision_status_sequence = 0U;
    vision_last_packet_tick = 0U;
    vision_packet_count = 0U;
    vision_crc_error_count = 0U;
    vision_format_error_count = 0U;
    memset(&vision_latest_target, 0, sizeof(vision_latest_target));
    vision_protocol_parser_init(&vision_parser);

    uart_init(UART_2,
              VISION_UART_BAUDRATE,
              UART2_TX_A23,
              UART2_RX_A24);
    uart_set_callback(UART_2, vision_uart_rx_callback, NULL);
    uart_set_interrupt_config(UART_2, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void vision_uart_process(uint32_t current_tick)
{
    uint8_t byte;
    vision_packet_struct packet;
    vision_target_struct target;
    vision_parse_result_enum result;

    while(vision_rx_tail != vision_rx_head)
    {
        byte = vision_rx_buffer[vision_rx_tail];
        vision_rx_tail = (uint8_t)((vision_rx_tail + 1U)
                                 & VISION_UART_RX_BUFFER_MASK);
        result = vision_protocol_feed(&vision_parser, byte, &packet);
        if(VISION_PARSE_PACKET == result)
        {
            if(0U != vision_protocol_decode_target(&packet, &target))
            {
                vision_latest_target = target;
                vision_latest_sequence = packet.sequence;
                vision_last_packet_tick = current_tick;
                vision_packet_received = 1U;
                vision_packet_count++;
            }
            else
            {
                vision_format_error_count++;
            }
        }
        else if(VISION_PARSE_CRC_ERROR == result)
        {
            vision_crc_error_count++;
        }
        else if(VISION_PARSE_FORMAT_ERROR == result)
        {
            vision_format_error_count++;
        }
    }
}

void vision_uart_get_snapshot(
    uint32_t current_tick,
    vision_uart_snapshot_struct *snapshot)
{
    uint32_t age_ticks;

    if(NULL == snapshot)
    {
        return;
    }

    age_ticks = vision_packet_received
              ? (current_tick - vision_last_packet_tick)
              : 0xFFFFFFFFU;
    snapshot->target = vision_latest_target;
    snapshot->sequence = vision_latest_sequence;
    snapshot->link_alive =
        (vision_packet_received
         && (age_ticks <= VISION_UART_FRESH_TIMEOUT_TICKS));
    snapshot->target_fresh =
        (snapshot->link_alive
         && (0U != (vision_latest_target.flags
                    & VISION_TARGET_FLAG_VALID)));
    snapshot->packet_count = vision_packet_count;
    snapshot->crc_error_count = vision_crc_error_count;
    snapshot->format_error_count = vision_format_error_count;
    snapshot->rx_overflow_count = vision_rx_overflow_count;
    snapshot->age_ticks = age_ticks;
}

void vision_uart_send_status(const vision_car_status_struct *status)
{
    uint8_t packet[VISION_PROTOCOL_MAX_PACKET_LENGTH];
    uint8_t length;

    length = vision_protocol_build_status(
        vision_status_sequence,
        status,
        packet,
        sizeof(packet));
    if(0U != length)
    {
        uart_write_buffer(UART_2, packet, length);
        vision_status_sequence++;
    }
}
