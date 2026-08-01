#include "pid_test_serial.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"

#define PID_TEST_RX_BUFFER_SIZE        (128U)
#define PID_TEST_LINE_SIZE             (80U)
#define PID_TEST_DURATION_MIN_MS       (1000UL)
#define PID_TEST_DURATION_MAX_MS       (120000UL)

static volatile uint8 pid_test_rx_buffer[PID_TEST_RX_BUFFER_SIZE];
static volatile uint8 pid_test_rx_head;
static volatile uint8 pid_test_rx_tail;
static char pid_test_line[PID_TEST_LINE_SIZE];
static uint8 pid_test_line_length;
static uint8 pid_test_telemetry_divider;

static void pid_test_rx_callback(uint32 event, void *context)
{
    uint8 byte;
    uint8 next_head;

    (void)event;
    (void)context;
    while(0U != uart_query_byte(PID_TEST_UART_INDEX, &byte))
    {
        next_head = (uint8)((pid_test_rx_head + 1U)
                           % PID_TEST_RX_BUFFER_SIZE);
        if(next_head != pid_test_rx_tail)
        {
            pid_test_rx_buffer[pid_test_rx_head] = byte;
            pid_test_rx_head = next_head;
        }
    }
}

static void pid_test_parse_line(
    const char *line,
    pid_test_command_struct *command)
{
    float target_rpm;
    float kp;
    float ki;
    float kd;
    unsigned long duration_ms;

    if(0 == strcmp(line, "@PIDTEST,STOP"))
    {
        command->stop_requested = 1U;
        return;
    }
    if(0 == strcmp(line, "@PIDTEST,PING"))
    {
        pid_test_serial_send_ack("READY", "protocol=3");
        return;
    }
    if(0 == strcmp(line, "@PIDTEST,ENCODER"))
    {
        command->encoder_requested = 1U;
        return;
    }
    if(0 == strcmp(line, "@PIDTEST,DUMP"))
    {
        command->dump_requested = 1U;
        return;
    }
    if(3 == sscanf(line, "@PIDTEST,GAINS,%f,%f,%f", &kp, &ki, &kd))
    {
        if((kp < 0.0f) || (kp > 30.0f)
           || (ki < 0.0f) || (ki > 3.0f)
           || (kd < 0.0f) || (kd > 2.0f))
        {
            pid_test_serial_send_ack("ERROR", "gain_range");
            return;
        }
        command->kp = kp;
        command->ki = ki;
        command->kd = kd;
        command->gains_requested = 1U;
        return;
    }
    if(2 == sscanf(line,
                   "@PIDTEST,START,%f,%lu",
                   &target_rpm,
                   &duration_ms))
    {
        if((target_rpm < -100.0f) || (target_rpm > 100.0f)
           || (duration_ms < PID_TEST_DURATION_MIN_MS)
           || (duration_ms > PID_TEST_DURATION_MAX_MS))
        {
            pid_test_serial_send_ack("ERROR", "range");
            return;
        }
        command->target_rpm = target_rpm;
        command->duration_ms = (uint32)duration_ms;
        command->start_requested = 1U;
    }
}

void pid_test_serial_init(void)
{
    uart_init(PID_TEST_UART_INDEX,
              PID_TEST_UART_BAUDRATE,
              PID_TEST_UART_TX_PIN,
              PID_TEST_UART_RX_PIN);
    pid_test_rx_head = 0U;
    pid_test_rx_tail = 0U;
    pid_test_line_length = 0U;
    pid_test_telemetry_divider = 0U;
    uart_set_callback(PID_TEST_UART_INDEX, pid_test_rx_callback, NULL);
    uart_set_interrupt_config(
        PID_TEST_UART_INDEX,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
}

void pid_test_serial_process(pid_test_command_struct *command)
{
    uint8 byte;

    if(NULL == command)
    {
        return;
    }
    command->start_requested = 0U;
    command->stop_requested = 0U;
    command->encoder_requested = 0U;
    command->gains_requested = 0U;
    command->dump_requested = 0U;

    while(pid_test_rx_tail != pid_test_rx_head)
    {
        byte = pid_test_rx_buffer[pid_test_rx_tail];
        pid_test_rx_tail = (uint8)((pid_test_rx_tail + 1U)
                                  % PID_TEST_RX_BUFFER_SIZE);
        if(('\r' == byte) || ('\n' == byte))
        {
            if(0U != pid_test_line_length)
            {
                pid_test_line[pid_test_line_length] = '\0';
                pid_test_parse_line(pid_test_line, command);
                pid_test_line_length = 0U;
            }
        }
        else if((byte >= 32U) && (byte <= 126U))
        {
            if(pid_test_line_length < (PID_TEST_LINE_SIZE - 1U))
            {
                pid_test_line[pid_test_line_length++] = (char)byte;
            }
            else
            {
                pid_test_line_length = 0U;
            }
        }
        else
        {
            /* Ignore reset noise and non-ASCII startup bytes. */
            pid_test_line_length = 0U;
        }
    }
}

void pid_test_serial_send_ack(const char *status, const char *detail)
{
    char line[96];

    sprintf(line, "@PIDACK,1,%s,%s\r\n", status, detail);
    uart_write_string(PID_TEST_UART_INDEX, line);
}

void pid_test_serial_send_encoder(
    uint32 timestamp_ms,
    int32 motor1_count,
    uint32 motor1_a_edges,
    uint32 motor1_b_edges,
    uint8 motor1_state,
    int32 motor2_count,
    uint32 motor2_a_edges,
    uint32 motor2_b_edges,
    uint8 motor2_state)
{
    char line[192];

    sprintf(line,
            "@ENC,1,%lu,%ld,%lu,%lu,%u,%ld,%lu,%lu,%u\r\n",
            (unsigned long)timestamp_ms,
            (long)motor1_count,
            (unsigned long)motor1_a_edges,
            (unsigned long)motor1_b_edges,
            motor1_state,
            (long)motor2_count,
            (unsigned long)motor2_a_edges,
            (unsigned long)motor2_b_edges,
            motor2_state);
    uart_write_string(PID_TEST_UART_INDEX, line);
}

void pid_test_serial_send_sample(
    uint32 timestamp_ms,
    float left_target_rpm,
    float right_target_rpm,
    float left_actual_rpm,
    float right_actual_rpm,
    int16 left_output,
    int16 right_output,
    uint8 enabled,
    uint8 line_mask,
    int16 line_error,
    uint8 line_state,
    uint32 straight_time_ms,
    uint32 curve_time_ms,
    uint32 sharp_time_ms,
    uint32 lost_time_ms)
{
    char line[224];

    pid_test_telemetry_divider++;
    if(pid_test_telemetry_divider < PID_TEST_TELEMETRY_DIVIDER)
    {
        return;
    }
    pid_test_telemetry_divider = 0U;

    sprintf(line,
            "@PID,3,%lu,%.3f,%.3f,%.3f,%.3f,%d,%d,"
            "%.3f,%.3f,%u,%u,%d,%u,%lu,%lu,%lu,%lu\r\n",
            (unsigned long)timestamp_ms,
            (double)left_target_rpm,
            (double)right_target_rpm,
            (double)left_actual_rpm,
            (double)right_actual_rpm,
            (int)left_output,
            (int)right_output,
            (double)(left_target_rpm - left_actual_rpm),
            (double)(right_target_rpm - right_actual_rpm),
            enabled,
            line_mask,
            (int)line_error,
            line_state,
            (unsigned long)straight_time_ms,
            (unsigned long)curve_time_ms,
            (unsigned long)sharp_time_ms,
            (unsigned long)lost_time_ms);
    uart_write_string(PID_TEST_UART_INDEX, line);
}
