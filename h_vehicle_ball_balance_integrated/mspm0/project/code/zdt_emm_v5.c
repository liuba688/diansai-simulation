#include "zdt_emm_v5.h"

#include <stddef.h>
#include <string.h>
#include "zf_driver_uart.h"

#define ZDT_UART_BAUDRATE       (115200U)
#define ZDT_ADDRESS             (0x01U)
#define ZDT_CHECKSUM            (0x6BU)
#define ZDT_SPEED_RPM           (30U)
#define ZDT_ACCELERATION        (100U)
#define ZDT_COMMAND_PERIOD      (4U)   /* 40 ms at the 10 ms application tick. */
#define ZDT_ACK_TIMEOUT         (12U)  /* 120 ms. */
#define ZDT_MAX_ERRORS          (3U)
#define ZDT_MAX_TARGET_PULSES   (200)
#define RX_SIZE                 (64U)
#define RX_MASK                 (RX_SIZE - 1U)

static volatile uint8_t rx_buffer[RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint32_t rx_overflow;
static zdt_emm_state_t state;
static uint8_t pending_function;
static uint32_t pending_since, last_send_tick;
static int16_t target_offset;
static uint8_t drive_step_limit = 24U, brake_step_limit = 48U;
static uint8_t last_status, consecutive_errors;
static uint32_t ack_count, error_count;

static void rx_callback(uint32 event, void *context)
{
    uint8_t byte, next;
    (void)event; (void)context;
    while(uart_query_byte(UART_1, &byte))
    {
        next = (uint8_t)((rx_head + 1U) & RX_MASK);
        if(next == rx_tail) rx_overflow++;
        else { rx_buffer[rx_head] = byte; rx_head = next; }
    }
}

static void write_enable(uint8_t enabled)
{
    uint8_t frame[6] = {ZDT_ADDRESS, 0xF3U, 0xABU,
                        enabled ? 1U : 0U, 0x00U, ZDT_CHECKSUM};
    uart_write_buffer(UART_1, frame, sizeof(frame));
}

static void write_position(int16_t signed_pulses, uint8_t movement_mode)
{
    uint32_t pulses = (uint32_t)((signed_pulses < 0) ? -signed_pulses : signed_pulses);
    uint8_t frame[13] = {
        ZDT_ADDRESS, 0xFDU, (signed_pulses > 0) ? 1U : 0U,
        (uint8_t)(ZDT_SPEED_RPM >> 8), (uint8_t)ZDT_SPEED_RPM,
        ZDT_ACCELERATION,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)pulses,
        movement_mode, 0x00U, ZDT_CHECKSUM
    };
    uart_write_buffer(UART_1, frame, sizeof(frame));
}

static void send_pending(uint8_t function, uint32_t tick)
{
    pending_function = function;
    pending_since = tick;
    last_send_tick = tick;
}

static void accept_response(uint8_t function, uint8_t status, uint32_t tick)
{
    last_status = status;
    if(0x9FU == status) return;
    if(0x02U == status)
    {
        ack_count++;
        consecutive_errors = 0U;
        if(pending_function == function) pending_function = 0U;
        if(ZDT_EMM_ENABLING == state && 0xF3U == function)
        {
            write_position(0, 2U); /* Anchor current real shaft position as software zero. */
            send_pending(0xFDU, tick);
            state = ZDT_EMM_ANCHORING;
        }
        else if(ZDT_EMM_ANCHORING == state && 0xFDU == function)
        {
            target_offset = 0;
            state = ZDT_EMM_READY;
        }
        else if(ZDT_EMM_RETURNING_HOME == state && 0xFDU == function)
        {
            state = ZDT_EMM_READY;
        }
        return;
    }
    error_count++;
    if(consecutive_errors < 255U) consecutive_errors++;
    if(pending_function == function) pending_function = 0U;
    if(consecutive_errors >= ZDT_MAX_ERRORS) state = ZDT_EMM_FAULT;
}

void zdt_emm_init(void)
{
    rx_head = rx_tail = 0U; rx_overflow = 0U;
    state = ZDT_EMM_IDLE; pending_function = 0U;
    pending_since = last_send_tick = 0U; target_offset = 0;
    last_status = consecutive_errors = 0U; ack_count = error_count = 0U;
    uart_init(UART_1, ZDT_UART_BAUDRATE, UART1_TX_B6, UART1_RX_B7);
    uart_set_callback(UART_1, rx_callback, NULL);
    uart_set_interrupt_config(UART_1, UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void zdt_emm_begin(uint32_t tick)
{
    /* The power-on anchor is persistent; task selection must not redefine it. */
    if(ZDT_EMM_READY == state || ZDT_EMM_RETURNING_HOME == state
       || ZDT_EMM_ENABLING == state || ZDT_EMM_ANCHORING == state) return;
    pending_function = 0U; target_offset = 0; consecutive_errors = 0U;
    write_enable(1U); send_pending(0xF3U, tick); state = ZDT_EMM_ENABLING;
}

void zdt_emm_update(uint32_t tick)
{
    while((uint8_t)((rx_head - rx_tail) & RX_MASK) >= 4U)
    {
        uint8_t a = rx_buffer[rx_tail];
        uint8_t f = rx_buffer[(uint8_t)((rx_tail + 1U) & RX_MASK)];
        uint8_t s = rx_buffer[(uint8_t)((rx_tail + 2U) & RX_MASK)];
        uint8_t c = rx_buffer[(uint8_t)((rx_tail + 3U) & RX_MASK)];
        if(ZDT_ADDRESS == a && ZDT_CHECKSUM == c)
        {
            rx_tail = (uint8_t)((rx_tail + 4U) & RX_MASK);
            accept_response(f, s, tick);
        }
        else rx_tail = (uint8_t)((rx_tail + 1U) & RX_MASK);
    }
    if(pending_function && (tick - pending_since >= ZDT_ACK_TIMEOUT))
    {
        pending_function = 0U; error_count++;
        if(consecutive_errors < 255U) consecutive_errors++;
        if(consecutive_errors >= ZDT_MAX_ERRORS) state = ZDT_EMM_FAULT;
    }
}

uint8_t zdt_emm_request_target(int16_t requested, uint32_t tick)
{
    int16_t difference, step;
    uint8_t limit;
    if(ZDT_EMM_READY != state || pending_function
       || tick - last_send_tick < ZDT_COMMAND_PERIOD) return 0U;
    if(requested > ZDT_MAX_TARGET_PULSES) requested = ZDT_MAX_TARGET_PULSES;
    if(requested < -ZDT_MAX_TARGET_PULSES) requested = -ZDT_MAX_TARGET_PULSES;
    difference = requested - target_offset;
    limit = (target_offset != 0 && (requested * target_offset < 0
             || ((requested < 0 ? -requested : requested)
                 < (target_offset < 0 ? -target_offset : target_offset))))
        ? brake_step_limit : drive_step_limit;
    step = difference;
    if(step > (int16_t)limit) step = (int16_t)limit;
    if(step < -(int16_t)limit) step = -(int16_t)limit;
    if(0 == step) return 0U;
    write_position(step, 0U);
    target_offset += step;
    send_pending(0xFDU, tick);
    return 1U;
}

uint8_t zdt_emm_return_home(uint32_t tick)
{
    int16_t correction;
    if(ZDT_EMM_READY != state && ZDT_EMM_RETURNING_HOME != state) return 0U;
    if(ZDT_EMM_RETURNING_HOME == state) return 1U;
    if(0 == target_offset) return 1U;

    /* A new standard-position command interrupts the previous one smoothly. */
    correction = (int16_t)-target_offset;
    pending_function = 0U;
    write_position(correction, 0U);
    target_offset = 0;
    send_pending(0xFDU, tick);
    state = ZDT_EMM_RETURNING_HOME;
    return 1U;
}

void zdt_emm_set_step_limits(uint8_t drive_step, uint8_t brake_step)
{
    drive_step_limit = drive_step ? drive_step : 1U;
    brake_step_limit = brake_step ? brake_step : drive_step_limit;
}

void zdt_emm_emergency_stop(void)
{
    uint8_t frame[5] = {ZDT_ADDRESS, 0xFEU, 0x98U, 0x00U, ZDT_CHECKSUM};
    uart_write_buffer(UART_1, frame, sizeof(frame));
    pending_function = 0U; state = ZDT_EMM_IDLE; target_offset = 0;
}

void zdt_emm_get_status(zdt_emm_status_t *out)
{
    if(NULL == out) return;
    out->state = state; out->target_offset_pulses = target_offset;
    out->last_status = last_status; out->consecutive_errors = consecutive_errors;
    out->ack_count = ack_count; out->error_count = error_count;
    out->overflow_count = rx_overflow;
}

uint8_t zdt_emm_is_ready(void) { return ZDT_EMM_READY == state; }
uint8_t zdt_emm_has_fault(void) { return ZDT_EMM_FAULT == state; }
