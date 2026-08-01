#include "line_sensor.h"

static const gpio_pin_enum line_sensor_pins[8] =
{
    /* Software order is left to right: bit0=OUT8 ... bit7=OUT1. */
    LINE_SENSOR_OUT8_PIN,
    LINE_SENSOR_OUT7_PIN,
    LINE_SENSOR_OUT6_PIN,
    LINE_SENSOR_OUT5_PIN,
    LINE_SENSOR_OUT4_PIN,
    LINE_SENSOR_OUT3_PIN,
    LINE_SENSOR_OUT2_PIN,
    LINE_SENSOR_OUT1_PIN
};

void line_sensor_init (void)
{
    uint8 channel;

    for(channel = 0; channel < 8; channel ++)
    {
        gpio_init(line_sensor_pins[channel],
                  GPI,
                  GPIO_LOW,
                  GPI_PULL_DOWN);
    }
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
        active = (gpio_get_level(line_sensor_pins[channel])
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
