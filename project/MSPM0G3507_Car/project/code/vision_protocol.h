#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_PROTOCOL_HEADER_0                 (0xAAU)
#define VISION_PROTOCOL_HEADER_1                 (0x55U)
#define VISION_PROTOCOL_VERSION                  (0x01U)

#define VISION_PACKET_TYPE_BALL_TARGET           (0x10U)
#define VISION_PACKET_TYPE_CAR_STATUS            (0x20U)

#define VISION_TARGET_PAYLOAD_LENGTH             (16U)
#define VISION_STATUS_PAYLOAD_LENGTH             (8U)
#define VISION_PROTOCOL_MAX_PAYLOAD_LENGTH       (16U)
#define VISION_PROTOCOL_MAX_PACKET_LENGTH        (23U)

#define VISION_TARGET_FLAG_VALID                 (1U << 0)
#define VISION_TARGET_FLAG_CONFIRMED             (1U << 1)
#define VISION_TARGET_FLAG_CLOSE                 (1U << 2)
#define VISION_TARGET_FLAG_MULTIPLE              (1U << 3)

#define VISION_STATUS_FLAG_ENABLED               (1U << 0)
#define VISION_STATUS_FLAG_MAGNET_ON             (1U << 1)
#define VISION_STATUS_FLAG_PAYLOAD_HELD          (1U << 2)
#define VISION_STATUS_FLAG_FAULT                 (1U << 3)

typedef enum
{
    VISION_PARSE_NONE = 0,
    VISION_PARSE_PACKET,
    VISION_PARSE_CRC_ERROR,
    VISION_PARSE_FORMAT_ERROR,
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
    uint8_t flags;
    uint16_t center_x;
    uint16_t center_y;
    uint16_t width;
    uint16_t height;
    uint16_t frame_width;
    uint16_t frame_height;
    uint8_t confidence;
    uint8_t candidate_count;
} vision_target_struct;

typedef struct
{
    uint8_t state;
    uint8_t flags;
    uint8_t fault;
    uint8_t line_mask;
    int16_t left_rpm_x10;
    int16_t right_rpm_x10;
} vision_car_status_struct;

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

uint8_t vision_protocol_crc8(const uint8_t *data, uint16_t length);
void vision_protocol_parser_init(vision_protocol_parser_struct *parser);
vision_parse_result_enum vision_protocol_feed(
    vision_protocol_parser_struct *parser,
    uint8_t byte,
    vision_packet_struct *packet);
uint8_t vision_protocol_decode_target(
    const vision_packet_struct *packet,
    vision_target_struct *target);
uint8_t vision_protocol_build_status(
    uint8_t sequence,
    const vision_car_status_struct *status,
    uint8_t *output,
    uint8_t output_capacity);

#endif
