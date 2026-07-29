#ifndef _line_sensor_h_
#define _line_sensor_h_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/*
 * CY-DXJ8 has eight independent digital outputs.
 * Vehicle-front view: OUT1 is leftmost and OUT8 is rightmost.
 * H55-8/PB10 and H55-9/PB11 remain reserved for START and E-STOP.
 */
#define LINE_SENSOR_OUT1_PIN            (B25)  /* H55-1 */
#define LINE_SENSOR_OUT2_PIN            (B18)  /* H55-2 */
#define LINE_SENSOR_OUT3_PIN            (B21)  /* H55-3 */
#define LINE_SENSOR_OUT4_PIN            (B22)  /* H55-4 */
#define LINE_SENSOR_OUT5_PIN            (A30)  /* H55-5 */
#define LINE_SENSOR_OUT6_PIN            (B0)   /* H55-6 */
#define LINE_SENSOR_OUT7_PIN            (B1)   /* H55-7 */
#define LINE_SENSOR_OUT8_PIN            (B14)  /* H55-10 */

/*
 * Verified on the real CY-DXJ8 on 2026-07-29:
 * white floor = low, black line = 3.3 V high.
 */
#define LINE_SENSOR_ACTIVE_LEVEL        (1)

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
