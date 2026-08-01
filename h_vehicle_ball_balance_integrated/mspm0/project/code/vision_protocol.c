#include "vision_protocol.h"

#include <stddef.h>
#include <string.h>

enum { WAIT_H0 = 0, WAIT_H1, READ_VER, READ_TYPE, READ_LEN,
       READ_SEQ, READ_PAYLOAD, READ_CRC };

static uint8_t crc_update(uint8_t crc, uint8_t byte)
{
    uint8_t i;
    crc ^= byte;
    for(i = 0U; i < 8U; ++i)
        crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
                            : (uint8_t)(crc << 1);
    return crc;
}

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
}

static void write_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void restart(vision_protocol_parser_struct *p, uint8_t byte)
{
    p->state = (VISION_PROTOCOL_HEADER_0 == byte) ? WAIT_H1 : WAIT_H0;
    p->payload_index = 0U;
    p->crc = 0U;
}

uint8_t vision_protocol_crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0U;
    if(NULL == data) return 0U;
    while(length--) crc = crc_update(crc, *data++);
    return crc;
}

void vision_protocol_parser_init(vision_protocol_parser_struct *p)
{
    if(NULL == p) return;
    memset(p, 0, sizeof(*p));
    p->state = WAIT_H0;
}

vision_parse_result_enum vision_protocol_feed(
    vision_protocol_parser_struct *p, uint8_t byte, vision_packet_struct *out)
{
    if(NULL == p || NULL == out) return VISION_PARSE_FORMAT_ERROR;
    switch(p->state)
    {
        case WAIT_H0:
            if(VISION_PROTOCOL_HEADER_0 == byte) p->state = WAIT_H1;
            break;
        case WAIT_H1:
            if(VISION_PROTOCOL_HEADER_1 == byte) p->state = READ_VER;
            else if(VISION_PROTOCOL_HEADER_0 != byte) p->state = WAIT_H0;
            break;
        case READ_VER:
            p->version = byte; p->crc = crc_update(0U, byte); p->state = READ_TYPE;
            break;
        case READ_TYPE:
            p->type = byte; p->crc = crc_update(p->crc, byte); p->state = READ_LEN;
            break;
        case READ_LEN:
            p->payload_length = byte; p->crc = crc_update(p->crc, byte);
            if(byte > VISION_PROTOCOL_MAX_PAYLOAD_LENGTH)
            { restart(p, byte); return VISION_PARSE_FORMAT_ERROR; }
            p->state = READ_SEQ;
            break;
        case READ_SEQ:
            p->sequence = byte; p->crc = crc_update(p->crc, byte);
            p->payload_index = 0U;
            p->state = p->payload_length ? READ_PAYLOAD : READ_CRC;
            break;
        case READ_PAYLOAD:
            p->payload[p->payload_index++] = byte;
            p->crc = crc_update(p->crc, byte);
            if(p->payload_index >= p->payload_length) p->state = READ_CRC;
            break;
        case READ_CRC:
            if(p->crc != byte)
            { restart(p, byte); return VISION_PARSE_CRC_ERROR; }
            out->version = p->version; out->type = p->type;
            out->payload_length = p->payload_length; out->sequence = p->sequence;
            memcpy(out->payload, p->payload, p->payload_length);
            restart(p, 0U);
            return VISION_PARSE_PACKET;
        default:
            vision_protocol_parser_init(p);
            return VISION_PARSE_FORMAT_ERROR;
    }
    return VISION_PARSE_NONE;
}

uint8_t vision_protocol_decode_ball(const vision_packet_struct *p,
                                    vision_ball_state_struct *ball)
{
    const uint8_t *d;
    if(NULL == p || NULL == ball || VISION_PROTOCOL_VERSION != p->version
       || VISION_MSG_BALL_STATE != p->type || 14U != p->payload_length)
        return 0U;
    d = p->payload;
    ball->task_id = d[0]; ball->run_id = d[1]; ball->flags = d[2];
    ball->position_x10_mm = (int16_t)read_u16(&d[3]);
    ball->velocity_mm_s = (int16_t)read_u16(&d[5]);
    ball->confidence = d[7];
    ball->measurement_age_ms = read_u16(&d[8]);
    ball->camera_timestamp_ms = read_u32(&d[10]);
    return 1U;
}

uint8_t vision_protocol_decode_event(const vision_packet_struct *p,
                                     vision_event_struct *event)
{
    if(NULL == p || NULL == event || VISION_PROTOCOL_VERSION != p->version
       || p->payload_length < 2U) return 0U;
    if(p->type < VISION_MSG_MODE_READY || p->type > VISION_MSG_FAULT)
        return 0U;
    memset(event, 0, sizeof(*event));
    event->task_id = p->payload[0]; event->run_id = p->payload[1];
    if(p->payload_length > 2U) event->result = p->payload[2];
    if(p->payload_length > 3U) event->state = p->payload[3];
    if(VISION_MSG_FAULT == p->type && p->payload_length > 2U)
        event->fault_code = p->payload[2];
    return 1U;
}

uint8_t vision_protocol_build_command(uint8_t type, uint8_t sequence,
                                      uint8_t task_id, uint8_t run_id,
                                      int16_t target_x10_mm,
                                      uint8_t speed_tier, uint8_t flags,
                                      uint32_t timestamp_ms,
                                      uint8_t *out, uint8_t capacity)
{
    uint8_t i, crc = 0U;
    const uint8_t payload_len = 10U;
    const uint8_t total = (uint8_t)(2U + 4U + payload_len + 1U);
    if(NULL == out || capacity < total) return 0U;
    out[0] = VISION_PROTOCOL_HEADER_0; out[1] = VISION_PROTOCOL_HEADER_1;
    out[2] = VISION_PROTOCOL_VERSION; out[3] = type;
    out[4] = payload_len; out[5] = sequence;
    out[6] = task_id; out[7] = run_id;
    write_u16(&out[8], (uint16_t)target_x10_mm);
    out[10] = speed_tier; out[11] = flags;
    write_u32(&out[12], timestamp_ms);
    for(i = 2U; i < total - 1U; ++i) crc = crc_update(crc, out[i]);
    out[total - 1U] = crc;
    return total;
}
