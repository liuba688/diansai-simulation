#include "car_app.h"

#include <stdio.h>
#include <string.h>

#include "zf_common_headfile.h"
#include "ball_car.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "magnet_driver.h"
#include "speed_pid.h"
#include "tb6612.h"
#include "vision_protocol.h"
#include "vision_uart.h"
#include "wheel_encoder.h"

#define CAR_START_DELAY_TICKS                  (300U)
#define CAR_STATUS_SEND_TICKS                  (20U)
#define CAR_OLED_UPDATE_TICKS                  (50U)
#define CAR_DEBUG_LOG_TICKS                    (200U)
#define CAR_HEARTBEAT_TICKS                    (50U)

typedef enum
{
    CAR_COMMAND_NONE = 0,
    CAR_COMMAND_START,
    CAR_COMMAND_STOP,
    CAR_COMMAND_RELEASE,
} car_command_enum;

typedef struct
{
    uint32 control_tick;
    uint16 start_delay_tick;
    uint16 heartbeat_ticks;
    uint16 status_send_ticks;
    uint16 oled_update_ticks;
    uint16 debug_log_ticks;
    uint8 running_requested;
    uint8 release_requested;

    int32 left_encoder_previous;
    int32 right_encoder_previous;
    speed_pid_struct left_speed_pid;
    speed_pid_struct right_speed_pid;
    line_sensor_data_struct line_sensor;
    line_follow_struct line_follow;
    ball_car_struct ball_car;
    ball_car_output_struct ball_output;
    vision_uart_snapshot_struct vision;
} car_app_context_struct;

static soft_iic_info_struct car_oled_iic;
static car_app_context_struct car_context;

static void car_ack_write(const char *text)
{
    uart_write_string(UART_1, text);
    uart_write_string(UART_3, text);
}

static void car_debug_write(const char *text)
{
    uart_write_string(UART_1, text);
}

static car_command_enum car_bluetooth_query_command(void)
{
    static char command[16];
    static uint8 command_length = 0U;
    uint8 data;

    while(0U != uart_query_byte(UART_3, &data))
    {
        if('1' == data)
        {
            command_length = 0U;
            return CAR_COMMAND_START;
        }
        if('0' == data)
        {
            command_length = 0U;
            return CAR_COMMAND_STOP;
        }
        if('2' == data)
        {
            command_length = 0U;
            return CAR_COMMAND_RELEASE;
        }

        if(('\r' == data) || ('\n' == data))
        {
            if(0U == command_length)
            {
                continue;
            }

            command[command_length] = '\0';
            command_length = 0U;
            if(0 == strcmp(command, "START"))
            {
                return CAR_COMMAND_START;
            }
            if(0 == strcmp(command, "STOP"))
            {
                return CAR_COMMAND_STOP;
            }
            if((0 == strcmp(command, "RELEASE"))
               || (0 == strcmp(command, "DROP")))
            {
                return CAR_COMMAND_RELEASE;
            }
            car_ack_write(
                "Unknown command. Use 1=START, 0=STOP, 2=RELEASE.\r\n");
        }
        else if(command_length < (sizeof(command) - 1U))
        {
            if((data >= 'a') && (data <= 'z'))
            {
                data = (uint8)(data - 'a' + 'A');
            }
            command[command_length] = (char)data;
            command_length++;
            command[command_length] = '\0';

            if(0 == strcmp(command, "START"))
            {
                command_length = 0U;
                return CAR_COMMAND_START;
            }
            if(0 == strcmp(command, "STOP"))
            {
                command_length = 0U;
                return CAR_COMMAND_STOP;
            }
            if((0 == strcmp(command, "RELEASE"))
               || (0 == strcmp(command, "DROP")))
            {
                command_length = 0U;
                return CAR_COMMAND_RELEASE;
            }
        }
        else
        {
            command_length = 0U;
        }
    }
    return CAR_COMMAND_NONE;
}

static void car_oled_write_command(uint8 command)
{
    uint8 packet[2] = {0x00U, command};
    soft_iic_write_8bit_array(&car_oled_iic, packet, 2);
}

static void car_oled_fill(uint8 pattern)
{
    uint16 index;

    car_oled_write_command(0x21U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0x22U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x07U);

    soft_iic_start(&car_oled_iic);
    soft_iic_send_data(&car_oled_iic, car_oled_iic.addr << 1);
    soft_iic_send_data(&car_oled_iic, 0x40U);
    for(index = 0U; index < 1024U; index++)
    {
        soft_iic_send_data(&car_oled_iic, pattern);
    }
    soft_iic_stop(&car_oled_iic);
}

static void car_oled_show_string(uint8 page, const char *text)
{
    uint8 column = 0U;
    uint8 font_index;
    uint8 font_column;

    car_oled_write_command(0x21U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0x22U);
    car_oled_write_command(page);
    car_oled_write_command(page);

    soft_iic_start(&car_oled_iic);
    soft_iic_send_data(&car_oled_iic, car_oled_iic.addr << 1);
    soft_iic_send_data(&car_oled_iic, 0x40U);

    while(('\0' != *text) && (column <= 121U))
    {
        if((*text < 32) || (*text > 126))
        {
            font_index = 0U;
        }
        else
        {
            font_index = (uint8)(*text - 32);
        }

        for(font_column = 0U; font_column < 6U; font_column++)
        {
            soft_iic_send_data(
                &car_oled_iic,
                ascii_font_6x8[font_index][font_column]);
            column++;
        }
        text++;
    }

    while(column < 128U)
    {
        soft_iic_send_data(&car_oled_iic, 0x00U);
        column++;
    }
    soft_iic_stop(&car_oled_iic);
}

static void car_oled_init(void)
{
    soft_iic_init(&car_oled_iic, 0x3CU, 10, A31, A28);
    system_delay_ms(100);

    car_oled_write_command(0xAEU);
    car_oled_write_command(0xD5U);
    car_oled_write_command(0x80U);
    car_oled_write_command(0xA8U);
    car_oled_write_command(0x3FU);
    car_oled_write_command(0xD3U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0x40U);
    car_oled_write_command(0x8DU);
    car_oled_write_command(0x14U);
    car_oled_write_command(0x20U);
    car_oled_write_command(0x00U);
    car_oled_write_command(0xA1U);
    car_oled_write_command(0xC8U);
    car_oled_write_command(0xDAU);
    car_oled_write_command(0x12U);
    car_oled_write_command(0x81U);
    car_oled_write_command(0x7FU);
    car_oled_write_command(0xD9U);
    car_oled_write_command(0xF1U);
    car_oled_write_command(0xDBU);
    car_oled_write_command(0x40U);
    car_oled_write_command(0xA4U);
    car_oled_write_command(0xA6U);
    car_oled_write_command(0xAFU);
    car_oled_fill(0x00U);
}

static void car_oled_update_status(uint8 motion_enabled)
{
    char line[24];

    car_oled_show_string(0U, "TI BALL CAR");
    sprintf(line,
            "MODE:%s",
            ball_car_state_name(car_context.ball_output.state));
    car_oled_show_string(2U, line);
    sprintf(line,
            "VIS:%s C:%u",
            car_context.vision.link_alive ? "ON" : "OFF",
            car_context.vision.target.confidence);
    car_oled_show_string(4U, line);
    sprintf(line,
            "%s M:%u P:%u",
            motion_enabled ? "RUN" : "STOP",
            car_context.ball_output.magnet_on,
            car_context.ball_output.payload_held);
    car_oled_show_string(6U, line);
}

static void car_reset_speed_control(void)
{
    speed_pid_set_target(&car_context.left_speed_pid, 0.0f);
    speed_pid_set_target(&car_context.right_speed_pid, 0.0f);
    car_context.left_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    car_context.right_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    tb6612_stop_all();
}

static void car_apply_command(car_command_enum command)
{
    switch(command)
    {
        case CAR_COMMAND_START:
            car_context.running_requested = 1U;
            car_context.start_delay_tick = 0U;
            car_context.release_requested = 0U;
            car_reset_speed_control();
            car_ack_write("ACK START: 3 s safety delay.\r\n");
            break;

        case CAR_COMMAND_STOP:
            car_context.running_requested = 0U;
            car_context.start_delay_tick = 0U;
            car_context.release_requested = 0U;
            car_reset_speed_control();
            car_ack_write("ACK STOP: motors stopped.\r\n");
            break;

        case CAR_COMMAND_RELEASE:
            car_context.running_requested = 0U;
            car_context.start_delay_tick = 0U;
            car_context.release_requested = 1U;
            car_reset_speed_control();
            car_ack_write("ACK RELEASE: stopped and magnet released.\r\n");
            break;

        case CAR_COMMAND_NONE:
        default:
            break;
    }
}

static int16 car_rpm_to_x10(float rpm)
{
    float value = rpm * 10.0f;

    value += (value >= 0.0f) ? 0.5f : -0.5f;
    if(value > 32767.0f)
    {
        value = 32767.0f;
    }
    else if(value < -32768.0f)
    {
        value = -32768.0f;
    }
    return (int16)value;
}

static void car_send_vision_status(
    uint8 motion_enabled,
    float left_target_rpm,
    float right_target_rpm)
{
    vision_car_status_struct status;

    status.state = (uint8_t)car_context.ball_output.state;
    status.flags = 0U;
    if(motion_enabled)
    {
        status.flags |= VISION_STATUS_FLAG_ENABLED;
    }
    if(car_context.ball_output.magnet_on)
    {
        status.flags |= VISION_STATUS_FLAG_MAGNET_ON;
    }
    if(car_context.ball_output.payload_held)
    {
        status.flags |= VISION_STATUS_FLAG_PAYLOAD_HELD;
    }
    if(BALL_CAR_STATE_FAULT == car_context.ball_output.state)
    {
        status.flags |= VISION_STATUS_FLAG_FAULT;
    }
    status.fault = (uint8_t)car_context.ball_output.fault;
    status.line_mask = car_context.line_sensor.mask;
    status.left_rpm_x10 = car_rpm_to_x10(left_target_rpm);
    status.right_rpm_x10 = car_rpm_to_x10(right_target_rpm);
    vision_uart_send_status(&status);
}

static void car_update_motor_control(
    float left_target_rpm,
    float right_target_rpm)
{
    int32 left_encoder_count;
    int32 right_encoder_count;
    int32 left_encoder_delta;
    int32 right_encoder_delta;
    int16 left_duty;
    int16 right_duty;

    speed_pid_set_target(
        &car_context.left_speed_pid,
        left_target_rpm);
    speed_pid_set_target(
        &car_context.right_speed_pid,
        right_target_rpm);

    if(0U != (car_context.control_tick
              % SPEED_PID_CONTROL_DIVIDER))
    {
        return;
    }

    left_encoder_count =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    right_encoder_count =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    left_encoder_delta =
        SPEED_PID_MOTOR1_ENCODER_SIGN
        * (left_encoder_count
           - car_context.left_encoder_previous);
    right_encoder_delta =
        SPEED_PID_MOTOR2_ENCODER_SIGN
        * (right_encoder_count
           - car_context.right_encoder_previous);
    car_context.left_encoder_previous = left_encoder_count;
    car_context.right_encoder_previous = right_encoder_count;

    left_duty = speed_pid_update(
        &car_context.left_speed_pid,
        left_encoder_delta,
        SPEED_PID_MOTOR1_KP,
        SPEED_PID_MOTOR1_KI,
        SPEED_PID_MOTOR1_KD);
    right_duty = speed_pid_update(
        &car_context.right_speed_pid,
        right_encoder_delta,
        SPEED_PID_MOTOR2_KP,
        SPEED_PID_MOTOR2_KI,
        SPEED_PID_MOTOR2_KD);

    /* Right wheel = TB6612 A; left wheel = TB6612 B. */
    tb6612_set_motor(
        TB6612_MOTOR_A,
        SPEED_PID_TB6612_A_FORWARD_SIGN * right_duty);
    tb6612_set_motor(
        TB6612_MOTOR_B,
        SPEED_PID_TB6612_B_FORWARD_SIGN * left_duty);
}

static void car_debug_log(
    uint8 motion_enabled,
    float left_target_rpm,
    float right_target_rpm)
{
    char log[256];

    sprintf(
        log,
        "CAR run=%u state=%s fault=%s line=%02X err=%d "
        "vision=%u target=%u conf=%u xy=%u,%u wh=%u,%u "
        "cmd[L=%d R=%d] rpm100[L=%ld R=%ld] "
        "mag=%u payload=%u packets=%lu crc=%lu ovf=%lu\r\n",
        motion_enabled,
        ball_car_state_name(car_context.ball_output.state),
        ball_car_fault_name(car_context.ball_output.fault),
        car_context.line_sensor.mask,
        car_context.line_sensor.error,
        car_context.vision.link_alive,
        car_context.vision.target_fresh,
        car_context.vision.target.confidence,
        car_context.vision.target.center_x,
        car_context.vision.target.center_y,
        car_context.vision.target.width,
        car_context.vision.target.height,
        (int)left_target_rpm,
        (int)right_target_rpm,
        (long)(car_context.left_speed_pid.measured_rpm * 100.0f),
        (long)(car_context.right_speed_pid.measured_rpm * 100.0f),
        car_context.ball_output.magnet_on,
        car_context.ball_output.payload_held,
        (unsigned long)car_context.vision.packet_count,
        (unsigned long)car_context.vision.crc_error_count,
        (unsigned long)car_context.vision.rx_overflow_count);
    car_debug_write(log);
}

void car_app_init(void)
{
    memset(&car_context, 0, sizeof(car_context));

    uart_init(UART_1, 115200U, UART1_TX_B6, UART1_RX_B7);
    uart_init(UART_3, 9600U, UART3_TX_B2, UART3_RX_B3);
    vision_uart_init();

    gpio_init(B16, GPO, 0U, GPO_PUSH_PULL);
    gpio_init(A7, GPO, 1U, GPO_PUSH_PULL);

    tb6612_init();
    wheel_encoder_init();
    speed_pid_init(&car_context.left_speed_pid);
    speed_pid_init(&car_context.right_speed_pid);
    line_sensor_init();
    line_follow_init(&car_context.line_follow);
    ball_car_init(&car_context.ball_car);
    magnet_driver_init();
    car_oled_init();

    gpio_set_level(A7, 0U);
    system_delay_ms(200);
    gpio_set_level(A7, 1U);

    car_context.left_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR1);
    car_context.right_encoder_previous =
        wheel_encoder_get_count(WHEEL_ENCODER_MOTOR2);
    car_context.ball_output.state = BALL_CAR_STATE_IDLE;

    car_oled_show_string(0U, "TI BALL CAR READY");
    car_oled_show_string(2U, "VISION UART2");
    car_oled_show_string(4U, "1 START 0 STOP");
    car_oled_show_string(6U, "2 RELEASE MAGNET");

    car_ack_write("MSPM0 ball-car controller ready.\r\n");
    car_debug_write(
        "Vision UART2=115200 PA23(TX)/PA24(RX); "
        "HC-05 UART3=9600 PB2/PB3.\r\n");
    car_debug_write(
        "Gray A0/A1/A2/OUT=PB25/PB18/PB21/PB22; "
        "magnet MOSFET control=PB10.\r\n");
    car_debug_write(
        "Commands: 1=START (3 s delay), 0=STOP, "
        "2=RELEASE while stopped.\r\n");
}

void car_app_run(void)
{
    car_command_enum command;
    ball_car_input_struct input;
    ball_car_state_enum previous_state;
    uint8 motion_enabled;
    float line_left_rpm;
    float line_right_rpm;
    float left_target_rpm;
    float right_target_rpm;
    char transition_log[128];

    while(1)
    {
        system_delay_ms(SPEED_PID_BASE_PERIOD_MS);
        car_context.control_tick++;

        command = car_bluetooth_query_command();
        car_apply_command(command);

        vision_uart_process(car_context.control_tick);
        vision_uart_get_snapshot(
            car_context.control_tick,
            &car_context.vision);
        line_sensor_read(&car_context.line_sensor);

        if(car_context.running_requested
           && (car_context.start_delay_tick
               < CAR_START_DELAY_TICKS))
        {
            car_context.start_delay_tick++;
        }
        motion_enabled =
            car_context.running_requested
            && (car_context.start_delay_tick
                >= CAR_START_DELAY_TICKS);

        line_left_rpm = 0.0f;
        line_right_rpm = 0.0f;
        if(motion_enabled
           && ball_car_requires_line_follow(
               &car_context.ball_car))
        {
            line_follow_update(
                &car_context.line_follow,
                &car_context.line_sensor,
                &line_left_rpm,
                &line_right_rpm);
        }

        memset(&input, 0, sizeof(input));
        input.enabled = motion_enabled;
        input.release_payload = car_context.release_requested;
        input.line_valid = car_context.line_sensor.line_valid;
        input.line_error = car_context.line_sensor.error;
        input.line_left_rpm = line_left_rpm;
        input.line_right_rpm = line_right_rpm;
        input.vision_link_alive = car_context.vision.link_alive;
        input.vision_valid = car_context.vision.target_fresh;
        input.vision_confirmed =
            (0U != (car_context.vision.target.flags
                    & VISION_TARGET_FLAG_CONFIRMED));
        input.vision_close =
            (0U != (car_context.vision.target.flags
                    & VISION_TARGET_FLAG_CLOSE));
        input.vision_confidence =
            car_context.vision.target.confidence;
        input.vision_center_x =
            car_context.vision.target.center_x;
        input.vision_center_y =
            car_context.vision.target.center_y;
        input.vision_width = car_context.vision.target.width;
        input.vision_height = car_context.vision.target.height;
        input.vision_frame_width =
            car_context.vision.target.frame_width;
        input.vision_frame_height =
            car_context.vision.target.frame_height;

        previous_state = car_context.ball_car.state;
        ball_car_update(
            &car_context.ball_car,
            &input,
            &car_context.ball_output);
        car_context.release_requested = 0U;

        if((previous_state != car_context.ball_car.state)
           && ((BALL_CAR_STATE_LINE_FOLLOW
                == car_context.ball_car.state)
               || (BALL_CAR_STATE_LINE_FOLLOW_CARRY
                   == car_context.ball_car.state)))
        {
            line_follow_init(&car_context.line_follow);
        }

        left_target_rpm =
            motion_enabled
                ? car_context.ball_output.left_target_rpm
                : 0.0f;
        right_target_rpm =
            motion_enabled
                ? car_context.ball_output.right_target_rpm
                : 0.0f;
        magnet_driver_set(car_context.ball_output.magnet_on);
        car_update_motor_control(
            left_target_rpm,
            right_target_rpm);

        if(previous_state != car_context.ball_output.state)
        {
            sprintf(
                transition_log,
                "STATE %s -> %s, fault=%s\r\n",
                ball_car_state_name(previous_state),
                ball_car_state_name(
                    car_context.ball_output.state),
                ball_car_fault_name(
                    car_context.ball_output.fault));
            car_debug_write(transition_log);
        }

        car_context.status_send_ticks++;
        if(car_context.status_send_ticks
           >= CAR_STATUS_SEND_TICKS)
        {
            car_context.status_send_ticks = 0U;
            car_send_vision_status(
                motion_enabled,
                left_target_rpm,
                right_target_rpm);
        }

        car_context.heartbeat_ticks++;
        if(car_context.heartbeat_ticks >= CAR_HEARTBEAT_TICKS)
        {
            car_context.heartbeat_ticks = 0U;
            gpio_toggle_level(B16);
        }

        car_context.oled_update_ticks++;
        if((car_context.oled_update_ticks
            >= CAR_OLED_UPDATE_TICKS)
           && !motion_enabled)
        {
            car_context.oled_update_ticks = 0U;
            car_oled_update_status(motion_enabled);
        }

        car_context.debug_log_ticks++;
        if(car_context.debug_log_ticks
           >= CAR_DEBUG_LOG_TICKS)
        {
            car_context.debug_log_ticks = 0U;
            car_debug_log(
                motion_enabled,
                left_target_rpm,
                right_target_rpm);
        }
    }
}
