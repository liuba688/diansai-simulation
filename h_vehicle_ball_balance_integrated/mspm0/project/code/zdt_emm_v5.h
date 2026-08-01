#ifndef ZDT_EMM_V5_H
#define ZDT_EMM_V5_H

#include <stdint.h>

typedef enum
{
    ZDT_EMM_IDLE = 0,
    ZDT_EMM_ENABLING,
    ZDT_EMM_ANCHORING,
    ZDT_EMM_RETURNING_HOME,
    ZDT_EMM_READY,
    ZDT_EMM_FAULT
} zdt_emm_state_t;

typedef struct
{
    zdt_emm_state_t state;
    int16_t target_offset_pulses;
    uint8_t last_status;
    uint8_t consecutive_errors;
    uint32_t ack_count;
    uint32_t error_count;
    uint32_t overflow_count;
} zdt_emm_status_t;

void zdt_emm_init(void);
void zdt_emm_begin(uint32_t tick);
void zdt_emm_update(uint32_t tick);
uint8_t zdt_emm_request_target(int16_t target_pulses, uint32_t tick);
uint8_t zdt_emm_return_home(uint32_t tick);
void zdt_emm_set_step_limits(uint8_t drive_step, uint8_t brake_step);
void zdt_emm_emergency_stop(void);
void zdt_emm_get_status(zdt_emm_status_t *status);
uint8_t zdt_emm_is_ready(void);
uint8_t zdt_emm_has_fault(void);

#endif
