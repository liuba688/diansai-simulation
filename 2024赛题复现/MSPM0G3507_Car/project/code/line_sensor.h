#ifndef _line_sensor_h_
#define _line_sensor_h_

/* 2024 H3: intentionally restored to the original direct-sampling version. */

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/*
 * YB-MVX05 uses three address inputs and one digital output.
 * Default wiring uses the first four signal pins on expansion header H55:
 * X1/PB25=A0, X2/PB18=A1, X3/PB21=A2, X4/PB22=OUT.
 * Change only these four definitions if the real wiring is different.
 */
#define LINE_SENSOR_ADDRESS_0_PIN       (B25)
#define LINE_SENSOR_ADDRESS_1_PIN       (B18)
#define LINE_SENSOR_ADDRESS_2_PIN       (B21)
#define LINE_SENSOR_DATA_PIN            (B22)

/* The existing module was verified as black-line active high. */
#define LINE_SENSOR_ACTIVE_LEVEL        (1)
#define LINE_SENSOR_SETTLE_US           (5)

typedef struct
{
    uint8 mask;          /* bit0=leftmost sensor, bit7=rightmost sensor */
    uint8 active_count;
    uint8 line_valid;
    int16 error;         /* -350(left) ... 0(center) ... +350(right) */
} line_sensor_data_struct;

void line_sensor_init (void);
void line_sensor_read (line_sensor_data_struct *data);

#endif
