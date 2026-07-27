#include "vision_protocol.h"

#include <stddef.h>
#include <string.h>

enum
{
    PARSER_WAIT_HEADER_0 = 0,
    PARSER_WAIT_HEADER_1,
    PARSER_READ_VERSION,
    PARSER_READ_TYPE,
    PARSER_READ_LENGTH,
    PARSER_READ_SEQUENCE,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CRC,
};

static uint8_t vision_protocol_crc8_update(uint8_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= byte;
    for(bit = 0; bit < 8U; bit++)
    {
        if(0U != (crc & 0x80U))
        {
            crc = (uint8_t)((crc << 1) ^ 0x07U);
        }
        else
        {
            crc <<= 1;
        }
    }
    return crc;
}

static uint16_t vision_protocol_read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void vision_protocol_write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void vision_protocol_write_i16(uint8_t *data, int16_t value)
{
    vision_protocol_write_u16(data, (uint16_t)value);
}

static void vision_protocol_restart(
    vision_protocol_parser_struct *parser,
    uint8_t current_byte)
{
    parser->state = (VISION_PROTOCOL_HEADER_0 == current_byte)
                  ? PARSER_WAIT_HEADER_1
                  : PARSER_WAIT_HEADER_0;
    parser->payload_index = 0U;
    parser->crc = 0U;
}

uint8_t vision_protocol_crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0U;

    if(NULL == data)
    {
        return 0U;
    }

    while(0U != length)
    {
        crc = vision_protocol_crc8_update(crc, *data);
        data++;
        length--;
    }
    return crc;
}

void vision_protocol_parser_init(vision_protocol_parser_struct *parser)
{
    if(NULL == parser)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->state = PARSER_WAIT_HEADER_0;
}

vision_parse_result_enum vision_protocol_feed(
    vision_protocol_parser_struct *parser,
    uint8_t byte,
    vision_packet_struct *packet)
{
    if((NULL == parser) || (NULL == packet))
    {
        return VISION_PARSE_FORMAT_ERROR;
    }

    switch(parser->state)
    {
        case PARSER_WAIT_HEADER_0:
            if(VISION_PROTOCOL_HEADER_0 == byte)
            {
                parser->state = PARSER_WAIT_HEADER_1;
            }
            break;

        case PARSER_WAIT_HEADER_1:
            if(VISION_PROTOCOL_HEADER_1 == byte)
            {
                parser->state = PARSER_READ_VERSION;
                parser->payload_index = 0U;
                parser->crc = 0U;
            }
            else if(VISION_PROTOCOL_HEADER_0 != byte)
            {
                parser->state = PARSER_WAIT_HEADER_0;
            }
            break;

        case PARSER_READ_VERSION:
            parser->version = byte;
            parser->crc = vision_protocol_crc8_update(0U, byte);
            parser->state = PARSER_READ_TYPE;
            break;

        case PARSER_READ_TYPE:
            parser->type = byte;
            parser->crc = vision_protocol_crc8_update(parser->crc, byte);
            parser->state = PARSER_READ_LENGTH;
            break;

        case PARSER_READ_LENGTH:
            parser->payload_length = byte;
            parser->crc = vision_protocol_crc8_update(parser->crc, byte);
            if(byte > VISION_PROTOCOL_MAX_PAYLOAD_LENGTH)
            {
                vision_protocol_restart(parser, byte);
                return VISION_PARSE_FORMAT_ERROR;
            }
            parser->state = PARSER_READ_SEQUENCE;
            break;

        case PARSER_READ_SEQUENCE:
            parser->sequence = byte;
            parser->crc = vision_protocol_crc8_update(parser->crc, byte);
            parser->payload_index = 0U;
            parser->state = (0U == parser->payload_length)
                          ? PARSER_READ_CRC
                          : PARSER_READ_PAYLOAD;
            break;

        case PARSER_READ_PAYLOAD:
            parser->payload[parser->payload_index] = byte;
            parser->payload_index++;
            parser->crc = vision_protocol_crc8_update(parser->crc, byte);
            if(parser->payload_index >= parser->payload_length)
            {
                parser->state = PARSER_READ_CRC;
            }
            break;

        case PARSER_READ_CRC:
            if(parser->crc != byte)
            {
                vision_protocol_restart(parser, byte);
                return VISION_PARSE_CRC_ERROR;
            }

            packet->version = parser->version;
            packet->type = parser->type;
            packet->payload_length = parser->payload_length;
            packet->sequence = parser->sequence;
            if(0U != parser->payload_length)
            {
                memcpy(packet->payload,
                       parser->payload,
                       parser->payload_length);
            }
            vision_protocol_restart(parser, 0U);
            return VISION_PARSE_PACKET;

        default:
            vision_protocol_parser_init(parser);
            return VISION_PARSE_FORMAT_ERROR;
    }

    return VISION_PARSE_NONE;
}

uint8_t vision_protocol_decode_target(
    const vision_packet_struct *packet,
    vision_target_struct *target)
{
    const uint8_t *payload;

    if((NULL == packet) || (NULL == target))
    {
        return 0U;
    }
    if((VISION_PROTOCOL_VERSION != packet->version)
       || (VISION_PACKET_TYPE_BALL_TARGET != packet->type)
       || (VISION_TARGET_PAYLOAD_LENGTH != packet->payload_length))
    {
        return 0U;
    }

    payload = packet->payload;
    target->flags = payload[0];
    target->center_x = vision_protocol_read_u16(&payload[1]);
    target->center_y = vision_protocol_read_u16(&payload[3]);
    target->width = vision_protocol_read_u16(&payload[5]);
    target->height = vision_protocol_read_u16(&payload[7]);
    target->frame_width = vision_protocol_read_u16(&payload[9]);
    target->frame_height = vision_protocol_read_u16(&payload[11]);
    target->confidence = payload[13];
    target->candidate_count = payload[14];
    return 1U;
}

uint8_t vision_protocol_build_status(
    uint8_t sequence,
    const vision_car_status_struct *status,
    uint8_t *output,
    uint8_t output_capacity)
{
    uint8_t index;
    uint8_t crc;
    const uint8_t packet_length =
        2U + 4U + VISION_STATUS_PAYLOAD_LENGTH + 1U;

    if((NULL == status) || (NULL == output)
       || (output_capacity < packet_length))
    {
        return 0U;
    }

    output[0] = VISION_PROTOCOL_HEADER_0;
    output[1] = VISION_PROTOCOL_HEADER_1;
    output[2] = VISION_PROTOCOL_VERSION;
    output[3] = VISION_PACKET_TYPE_CAR_STATUS;
    output[4] = VISION_STATUS_PAYLOAD_LENGTH;
    output[5] = sequence;
    output[6] = status->state;
    output[7] = status->flags;
    output[8] = status->fault;
    output[9] = status->line_mask;
    vision_protocol_write_i16(&output[10], status->left_rpm_x10);
    vision_protocol_write_i16(&output[12], status->right_rpm_x10);

    crc = 0U;
    for(index = 2U; index < (packet_length - 1U); index++)
    {
        crc = vision_protocol_crc8_update(crc, output[index]);
    }
    output[packet_length - 1U] = crc;
    return packet_length;
}
