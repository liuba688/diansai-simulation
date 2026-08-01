#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_PROTOCOL_HEADER_0            (0xAAU)
#define VISION_PROTOCOL_HEADER_1            (0x55U)
#define VISION_PROTOCOL_VERSION             (0x02U)
#define VISION_PROTOCOL_MAX_PAYLOAD_LENGTH  (20U)
#define VISION_PROTOCOL_MAX_PACKET_LENGTH   (27U)

#define VISION_MSG_BALL_STATE       (0x10U)
#define VISION_MSG_MODE_SELECT      (0x30U)
#define VISION_MSG_START            (0x31U)
#define VISION_MSG_STOP             (0x32U)
#define VISION_MSG_MCU_HEARTBEAT    (0x33U)
#define VISION_MSG_MODE_READY       (0x40U)
#define VISION_MSG_STARTED          (0x41U)
#define VISION_MSG_CAM_HEARTBEAT    (0x42U)
#define VISION_MSG_TASK_COMPLETE    (0x43U)
#define VISION_MSG_FAULT            (0x44U)

#define VISION_BALL_FLAG_VALID      (1U << 0)
#define VISION_BALL_FLAG_CONFIRMED  (1U << 1)

typedef enum
{
    VISION_PARSE_NONE = 0,
    VISION_PARSE_PACKET,
    VISION_PARSE_CRC_ERROR,
    VISION_PARSE_FORMAT_ERROR
} vision_parse_result_enum;

typedef struct
{
    uint8_t version;
    uint8_t type;
    uint8_t payload_length;
    uint8_t sequence;
    uint8_t payload[VISION_PROTOCOL_MAX_PAYLOAD_LENGTH];
} vision_packet_struct;

typedef struct
{
    uint8_t state;
    uint8_t version;
    uint8_t type;
    uint8_t payload_length;
    uint8_t sequence;
    uint8_t payload_index;
    uint8_t crc;
    uint8_t payload[VISION_PROTOCOL_MAX_PAYLOAD_LENGTH];
} vision_protocol_parser_struct;

typedef struct
{
    uint8_t task_id;
    uint8_t run_id;
    uint8_t flags;
    int16_t position_x10_mm;
    int16_t velocity_mm_s;
    uint8_t confidence;
    uint16_t measurement_age_ms;
    uint32_t camera_timestamp_ms;
} vision_ball_state_struct;

typedef struct
{
    uint8_t task_id;
    uint8_t run_id;
    uint8_t result;
    uint8_t state;
    uint8_t fault_code;
} vision_event_struct;

uint8_t vision_protocol_crc8(const uint8_t *data, uint16_t length);
void vision_protocol_parser_init(vision_protocol_parser_struct *parser);
vision_parse_result_enum vision_protocol_feed(
    vision_protocol_parser_struct *parser, uint8_t byte,
    vision_packet_struct *packet);
uint8_t vision_protocol_decode_ball(const vision_packet_struct *packet,
                                    vision_ball_state_struct *ball);
uint8_t vision_protocol_decode_event(const vision_packet_struct *packet,
                                     vision_event_struct *event);
uint8_t vision_protocol_build_command(uint8_t type, uint8_t sequence,
                                      uint8_t task_id, uint8_t run_id,
                                      int16_t target_x10_mm,
                                      uint8_t speed_tier,
                                      uint8_t flags,
                                      uint32_t timestamp_ms,
                                      uint8_t *output,
                                      uint8_t capacity);

#endif
