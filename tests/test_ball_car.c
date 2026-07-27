#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ball_car.h"
#include "vision_protocol.h"

static void test_protocol_parser(void)
{
    uint8_t packet_bytes[23] =
    {
        0xAAU, 0x55U, 0x01U, 0x10U, 0x10U, 0x2AU,
        0x03U,
        0xA0U, 0x00U,
        0xC8U, 0x00U,
        0x28U, 0x00U,
        0x2AU, 0x00U,
        0x40U, 0x01U,
        0x40U, 0x01U,
        0x4EU, 0x02U, 0x00U,
        0x00U,
    };
    vision_protocol_parser_struct parser;
    vision_packet_struct packet;
    vision_target_struct target;
    vision_parse_result_enum result = VISION_PARSE_NONE;
    uint8_t index;

    packet_bytes[22] = vision_protocol_crc8(&packet_bytes[2], 20U);
    vision_protocol_parser_init(&parser);
    for(index = 0U; index < sizeof(packet_bytes); index++)
    {
        result = vision_protocol_feed(
            &parser,
            packet_bytes[index],
            &packet);
    }

    assert(VISION_PARSE_PACKET == result);
    assert(vision_protocol_decode_target(&packet, &target));
    assert(0x2AU == packet.sequence);
    assert(160U == target.center_x);
    assert(200U == target.center_y);
    assert(40U == target.width);
    assert(42U == target.height);
    assert(320U == target.frame_width);
    assert(320U == target.frame_height);
    assert(78U == target.confidence);
    assert(2U == target.candidate_count);

    packet_bytes[22] ^= 0x01U;
    vision_protocol_parser_init(&parser);
    for(index = 0U; index < sizeof(packet_bytes); index++)
    {
        result = vision_protocol_feed(
            &parser,
            packet_bytes[index],
            &packet);
    }
    assert(VISION_PARSE_CRC_ERROR == result);
}

static void test_status_packet_builder(void)
{
    vision_car_status_struct status;
    vision_protocol_parser_struct parser;
    vision_packet_struct packet;
    vision_parse_result_enum result = VISION_PARSE_NONE;
    uint8_t bytes[VISION_PROTOCOL_MAX_PACKET_LENGTH];
    uint8_t length;
    uint8_t index;

    memset(&status, 0, sizeof(status));
    status.state = (uint8_t)BALL_CAR_STATE_APPROACH;
    status.flags = VISION_STATUS_FLAG_ENABLED
                 | VISION_STATUS_FLAG_MAGNET_ON;
    status.fault = (uint8_t)BALL_CAR_FAULT_NONE;
    status.line_mask = 0x18U;
    status.left_rpm_x10 = 123;
    status.right_rpm_x10 = -45;

    length = vision_protocol_build_status(
        0x7CU,
        &status,
        bytes,
        sizeof(bytes));
    assert(15U == length);
    assert(VISION_PROTOCOL_HEADER_0 == bytes[0]);
    assert(VISION_PROTOCOL_HEADER_1 == bytes[1]);
    assert(VISION_PACKET_TYPE_CAR_STATUS == bytes[3]);
    assert(VISION_STATUS_PAYLOAD_LENGTH == bytes[4]);
    assert(0x7CU == bytes[5]);
    assert(bytes[14] == vision_protocol_crc8(&bytes[2], 12U));

    vision_protocol_parser_init(&parser);
    for(index = 0U; index < length; index++)
    {
        result = vision_protocol_feed(&parser, bytes[index], &packet);
    }
    assert(VISION_PARSE_PACKET == result);
    assert(VISION_PACKET_TYPE_CAR_STATUS == packet.type);
    assert((uint8_t)BALL_CAR_STATE_APPROACH == packet.payload[0]);
    assert(status.flags == packet.payload[1]);
    assert(0x18U == packet.payload[3]);
    assert(0x7BU == packet.payload[4]);
    assert(0x00U == packet.payload[5]);
    assert(0xD3U == packet.payload[6]);
    assert(0xFFU == packet.payload[7]);
}

static void set_valid_target(ball_car_input_struct *input, uint8_t close)
{
    input->vision_link_alive = 1U;
    input->vision_valid = 1U;
    input->vision_confirmed = 1U;
    input->vision_close = close;
    input->vision_confidence = 85U;
    input->vision_center_x = 170U;
    input->vision_center_y = 180U;
    input->vision_width = 40U;
    input->vision_height = close ? 100U : 45U;
    input->vision_frame_width = 320U;
    input->vision_frame_height = 320U;
}

static void set_line_follow_input(ball_car_input_struct *input)
{
    memset(input, 0, sizeof(*input));
    input->enabled = 1U;
    input->line_valid = 1U;
    input->line_left_rpm = 70.0f;
    input->line_right_rpm = 70.0f;
}

static void run_until_state(
    ball_car_struct *car,
    ball_car_input_struct *input,
    ball_car_output_struct *output,
    ball_car_state_enum expected,
    unsigned int limit)
{
    unsigned int tick;

    for(tick = 0U; (tick < limit) && (expected != car->state); tick++)
    {
        ball_car_update(car, input, output);
    }
    assert(expected == car->state);
}

static void enter_approach(
    ball_car_struct *car,
    ball_car_input_struct *input,
    ball_car_output_struct *output)
{
    ball_car_init(car);
    set_line_follow_input(input);
    memset(output, 0, sizeof(*output));

    ball_car_update(car, input, output);
    set_valid_target(input, 0U);
    run_until_state(
        car,
        input,
        output,
        BALL_CAR_STATE_STOP_LOCK,
        100U);
    run_until_state(
        car,
        input,
        output,
        BALL_CAR_STATE_APPROACH,
        100U);
}

static void test_control_gate(void)
{
    ball_car_struct car;
    ball_car_input_struct input;
    ball_car_output_struct output;
    unsigned int tick;

    ball_car_init(&car);
    set_line_follow_input(&input);
    memset(&output, 0, sizeof(output));
    ball_car_update(&car, &input, &output);

    set_valid_target(&input, 0U);
    input.vision_confidence = BALL_CAR_MIN_CONTROL_CONFIDENCE - 1U;
    for(tick = 0U; tick < 20U; tick++)
    {
        ball_car_update(&car, &input, &output);
        assert(BALL_CAR_STATE_LINE_FOLLOW == car.state);
    }

    input.vision_confidence = BALL_CAR_MIN_CONTROL_CONFIDENCE;
    input.vision_confirmed = 0U;
    for(tick = 0U; tick < 20U; tick++)
    {
        ball_car_update(&car, &input, &output);
        assert(BALL_CAR_STATE_LINE_FOLLOW == car.state);
    }

    input.vision_confirmed = 1U;
    run_until_state(
        &car,
        &input,
        &output,
        BALL_CAR_STATE_STOP_LOCK,
        100U);
}

static void test_complete_pickup_cycle(void)
{
    ball_car_struct car;
    ball_car_input_struct input;
    ball_car_output_struct output;
    unsigned int guard;

    ball_car_init(&car);
    set_line_follow_input(&input);
    memset(&output, 0, sizeof(output));
    input.line_error = 0;

    ball_car_update(&car, &input, &output);
    assert(BALL_CAR_STATE_LINE_FOLLOW == output.state);
    assert(70.0f == output.left_target_rpm);

    set_valid_target(&input, 0U);
    for(guard = 0U;
        (guard < 100U)
        && (BALL_CAR_STATE_STOP_LOCK != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_STOP_LOCK == car.state);

    for(guard = 0U;
        (guard < 100U)
        && (BALL_CAR_STATE_APPROACH != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_APPROACH == car.state);

    for(guard = 0U; guard < 10U; guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(car.history_count > 0U);

    set_valid_target(&input, 1U);
    ball_car_update(&car, &input, &output);
    assert(BALL_CAR_STATE_FINAL_CREEP == car.state);
    assert(output.magnet_on);

    for(guard = 0U;
        (guard < 200U)
        && (BALL_CAR_STATE_PICKUP_SETTLE != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_PICKUP_SETTLE == car.state);

    for(guard = 0U;
        (guard < 200U)
        && (BALL_CAR_STATE_BACKTRACK != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_BACKTRACK == car.state);
    assert(output.payload_held);
    input.vision_valid = 0U;
    input.vision_confirmed = 0U;
    input.line_valid = 0U;

    for(guard = 0U;
        (guard < 2000U)
        && (BALL_CAR_STATE_REACQUIRE_LINE != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_REACQUIRE_LINE == car.state);

    input.line_valid = 1U;
    for(guard = 0U;
        (guard < 20U)
        && (BALL_CAR_STATE_LINE_FOLLOW_CARRY != car.state);
        guard++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(BALL_CAR_STATE_LINE_FOLLOW_CARRY == car.state);
    assert(output.payload_held);
    assert(output.magnet_on);

    input.enabled = 0U;
    input.release_payload = 1U;
    ball_car_update(&car, &input, &output);
    assert(BALL_CAR_STATE_IDLE == output.state);
    assert(!output.payload_held);
    assert(!output.magnet_on);
}

static void test_vision_loss_and_reacquire_timeout(void)
{
    ball_car_struct car;
    ball_car_input_struct input;
    ball_car_output_struct output;
    unsigned int tick;

    enter_approach(&car, &input, &output);
    for(tick = 0U; tick < 12U; tick++)
    {
        ball_car_update(&car, &input, &output);
    }
    assert(car.history_count > 0U);

    input.vision_valid = 0U;
    input.vision_confirmed = 0U;
    input.line_valid = 0U;
    run_until_state(
        &car,
        &input,
        &output,
        BALL_CAR_STATE_BACKTRACK,
        BALL_CAR_TARGET_LOST_GRACE_TICKS + 5U);
    assert(BALL_CAR_FAULT_VISION_LOST == car.fault);
    assert(!output.magnet_on);

    run_until_state(
        &car,
        &input,
        &output,
        BALL_CAR_STATE_REACQUIRE_LINE,
        1000U);
    run_until_state(
        &car,
        &input,
        &output,
        BALL_CAR_STATE_FAULT,
        BALL_CAR_REACQUIRE_TIMEOUT_TICKS + 5U);
    assert(BALL_CAR_FAULT_REACQUIRE_TIMEOUT == output.fault);
    assert(0.0f == output.left_target_rpm);
    assert(0.0f == output.right_target_rpm);
}

static void test_approach_timeout_backtracks(void)
{
    ball_car_struct car;
    ball_car_input_struct input;
    ball_car_output_struct output;

    enter_approach(&car, &input, &output);
    run_until_state(
        &car,
        &input,
        &output,
        BALL_CAR_STATE_BACKTRACK,
        BALL_CAR_APPROACH_TIMEOUT_TICKS + 5U);
    assert(BALL_CAR_FAULT_APPROACH_TIMEOUT == output.fault);
    assert(!output.magnet_on);
    assert(car.history_count > 0U);
}

static void test_stop_preserves_held_payload_until_release(void)
{
    ball_car_struct car;
    ball_car_input_struct input;
    ball_car_output_struct output;

    ball_car_init(&car);
    memset(&input, 0, sizeof(input));
    memset(&output, 0, sizeof(output));
    car.state = BALL_CAR_STATE_LINE_FOLLOW_CARRY;
    car.payload_held = 1U;
    car.magnet_on = 1U;

    ball_car_update(&car, &input, &output);
    assert(BALL_CAR_STATE_IDLE == output.state);
    assert(output.payload_held);
    assert(output.magnet_on);

    input.release_payload = 1U;
    ball_car_update(&car, &input, &output);
    assert(!output.payload_held);
    assert(!output.magnet_on);
}

int main(void)
{
    test_protocol_parser();
    test_status_packet_builder();
    test_control_gate();
    test_complete_pickup_cycle();
    test_vision_loss_and_reacquire_timeout();
    test_approach_timeout_backtracks();
    test_stop_preserves_held_payload_until_release();
    puts("ball_car tests passed");
    return 0;
}
