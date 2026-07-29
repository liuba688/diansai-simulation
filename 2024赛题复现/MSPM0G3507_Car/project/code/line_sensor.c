#include "line_sensor.h"
#include "zf_driver_delay.h"

/* No temporal majority filter: each call returns the current eight channels. */

static void line_sensor_select (uint8 channel)
{
    gpio_set_level(LINE_SENSOR_ADDRESS_0_PIN, channel & 0x01);
    gpio_set_level(LINE_SENSOR_ADDRESS_1_PIN, (channel >> 1) & 0x01);
    gpio_set_level(LINE_SENSOR_ADDRESS_2_PIN, (channel >> 2) & 0x01);
}

void line_sensor_init (void)
{
    gpio_init(LINE_SENSOR_ADDRESS_0_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_ADDRESS_1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_ADDRESS_2_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LINE_SENSOR_DATA_PIN, GPI, 0, GPI_PULL_DOWN);
}

void line_sensor_read (line_sensor_data_struct *data)
{
    static const int16 weight[8] =
    {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    uint8 channel;
    uint8 active;
    int16 weighted_sum = 0;

    data->mask = 0;
    data->active_count = 0;

    for(channel = 0; channel < 8; channel ++)
    {
        line_sensor_select(channel);
        system_delay_us(LINE_SENSOR_SETTLE_US);
        active = (gpio_get_level(LINE_SENSOR_DATA_PIN)
                  == LINE_SENSOR_ACTIVE_LEVEL);
        if(active)
        {
            data->mask |= (uint8)(1U << channel);
            data->active_count ++;
            weighted_sum += weight[channel];
        }
    }

    data->line_valid = (0 != data->active_count);
    if(data->line_valid)
    {
        data->error = weighted_sum / data->active_count;
    }
    else
    {
        data->error = 0;
    }
}
