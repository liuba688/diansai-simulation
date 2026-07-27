#ifndef MAGNET_DRIVER_H
#define MAGNET_DRIVER_H

#include <stdint.h>

/*
 * H55 pin 8 / PB10 drives only the logic input of an external MOSFET
 * module. Never connect an electromagnet coil directly to this GPIO.
 */
#define MAGNET_DRIVER_OUTPUT_ENABLE             (1U)
#define MAGNET_DRIVER_ACTIVE_LEVEL              (1U)

void magnet_driver_init(void);
void magnet_driver_set(uint8_t enabled);
uint8_t magnet_driver_is_on(void);

#endif
