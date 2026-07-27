#include "magnet_driver.h"

#include "zf_driver_gpio.h"

#define MAGNET_DRIVER_CONTROL_PIN               (B10)

static uint8_t magnet_driver_on = 0U;

void magnet_driver_init(void)
{
    const uint8_t inactive_level =
        (0U != MAGNET_DRIVER_ACTIVE_LEVEL) ? 0U : 1U;

    gpio_init(MAGNET_DRIVER_CONTROL_PIN,
              GPO,
              inactive_level,
              GPO_PUSH_PULL);
    magnet_driver_on = 0U;
}

void magnet_driver_set(uint8_t enabled)
{
    uint8_t output_level;

#if !MAGNET_DRIVER_OUTPUT_ENABLE
    enabled = 0U;
#endif

    magnet_driver_on = (0U != enabled);
    output_level = magnet_driver_on
                 ? MAGNET_DRIVER_ACTIVE_LEVEL
                 : (uint8_t)!MAGNET_DRIVER_ACTIVE_LEVEL;
    gpio_set_level(MAGNET_DRIVER_CONTROL_PIN, output_level);
}

uint8_t magnet_driver_is_on(void)
{
    return magnet_driver_on;
}
