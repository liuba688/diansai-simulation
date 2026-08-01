#ifndef PID_TEST_SERIAL_H
#define PID_TEST_SERIAL_H

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

/*
 * Dedicated PC PID-test port:
 * MCU UART3 on PB2/PB3, exposed as carrier-board connector "UART4".
 */
#define PID_TEST_UART_INDEX         (UART_3)
#define PID_TEST_UART_TX_PIN        (UART3_TX_B2)
#define PID_TEST_UART_RX_PIN        (UART3_RX_B3)
#define PID_TEST_UART_BAUDRATE      (115200U)
#define PID_TEST_TELEMETRY_DIVIDER  (1U)   /* 50 Hz control -> 50 Hz telemetry */

typedef struct
{
    uint8 start_requested;
    uint8 stop_requested;
    uint8 encoder_requested;
    uint8 gains_requested;
    uint8 dump_requested;
    float target_rpm;
    float kp;
    float ki;
    float kd;
    uint32 duration_ms;
} pid_test_command_struct;

void pid_test_serial_init(void);
void pid_test_serial_process(pid_test_command_struct *command);
void pid_test_serial_send_ack(const char *status, const char *detail);
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
    uint32 lost_time_ms);
void pid_test_serial_send_encoder(
    uint32 timestamp_ms,
    int32 motor1_count,
    uint32 motor1_a_edges,
    uint32 motor1_b_edges,
    uint8 motor1_state,
    int32 motor2_count,
    uint32 motor2_a_edges,
    uint32 motor2_b_edges,
    uint8 motor2_state);

#endif
