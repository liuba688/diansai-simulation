#ifndef _wheel_encoder_h_
#define _wheel_encoder_h_

#include "zf_common_typedef.h"

typedef enum
{
    WHEEL_ENCODER_MOTOR1 = 0,
    WHEEL_ENCODER_MOTOR2,
    WHEEL_ENCODER_COUNT,
} wheel_encoder_enum;

void  wheel_encoder_init      (void);
int32 wheel_encoder_get_count (wheel_encoder_enum encoder);
uint32 wheel_encoder_get_edge_count (wheel_encoder_enum encoder, uint8 channel);
uint8 wheel_encoder_get_state (wheel_encoder_enum encoder);
void  wheel_encoder_clear     (wheel_encoder_enum encoder);
void  wheel_encoder_clear_all (void);

#endif
