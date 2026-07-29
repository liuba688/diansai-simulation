#include "zf_common_headfile.h"
#include "tb6612.h"

#define TB6612_PWMA_PIN    (PWM_TIM_G0_CH0_A12)
#define TB6612_AIN1_PIN    (B19)
#define TB6612_AIN2_PIN    (B17)

#define TB6612_PWMB_PIN    (PWM_TIM_G0_CH1_A13)
#define TB6612_BIN1_PIN    (A16)
#define TB6612_BIN2_PIN    (B24)

static int16 tb6612_limit_duty (int16 duty)
{
    if(TB6612_DUTY_MAX < duty)
    {
        duty = TB6612_DUTY_MAX;
    }
    else if(-TB6612_DUTY_MAX > duty)
    {
        duty = -TB6612_DUTY_MAX;
    }
    return duty;
}

static void tb6612_apply (gpio_pin_enum in1, gpio_pin_enum in2,
                          pwm_channel_enum pwm, int16 duty)
{
    uint16 absolute_duty;

#if !TB6612_OUTPUT_ENABLE
    duty = 0;
#endif

    duty = tb6612_limit_duty(duty);
    if(0 < duty)
    {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        absolute_duty = (uint16)duty;
    }
    else if(0 > duty)
    {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        absolute_duty = (uint16)(-duty);
    }
    else
    {
        // IN1=IN2=0、PWM=0：滑行停止，也是当前最安全的默认状态。
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        absolute_duty = 0;
    }

    pwm_set_duty(pwm, absolute_duty);
}

void tb6612_init (void)
{
    // 先锁定方向脚和 PWM 为低，再允许后续模块初始化。
    gpio_init(TB6612_AIN1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(TB6612_AIN2_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(TB6612_BIN1_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_init(TB6612_BIN2_PIN, GPO, 0, GPO_PUSH_PULL);

    pwm_init(TB6612_PWMA_PIN, TB6612_PWM_FREQUENCY, 0);
    pwm_init(TB6612_PWMB_PIN, TB6612_PWM_FREQUENCY, 0);
    tb6612_stop_all();
}

void tb6612_set_motor (tb6612_motor_enum motor, int16 duty)
{
    if(TB6612_MOTOR_A == motor)
    {
        tb6612_apply(TB6612_AIN1_PIN, TB6612_AIN2_PIN, TB6612_PWMA_PIN, duty);
    }
    else
    {
        tb6612_apply(TB6612_BIN1_PIN, TB6612_BIN2_PIN, TB6612_PWMB_PIN, duty);
    }
}

void tb6612_stop_motor (tb6612_motor_enum motor)
{
    tb6612_set_motor(motor, 0);
}

void tb6612_stop_all (void)
{
    tb6612_set_motor(TB6612_MOTOR_A, 0);
    tb6612_set_motor(TB6612_MOTOR_B, 0);
}
