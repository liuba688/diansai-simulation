#ifndef PID_TEST_SERIAL_H
#define PID_TEST_SERIAL_H

#include "zf_common_typedef.h"

typedef struct
{
    uint8 start_requested;
    uint8 stop_requested;
    float target_rpm;
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
    uint8 enabled);

#endif
