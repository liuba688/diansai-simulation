#include "zf_common_headfile.h"
#include "wheel_encoder.h"

#define MOTOR1_ENCODER_A_PIN    (A25)
#define MOTOR1_ENCODER_B_PIN    (A14)
#define MOTOR2_ENCODER_A_PIN    (A26)
#define MOTOR2_ENCODER_B_PIN    (A27)

static volatile int32 wheel_encoder_count[WHEEL_ENCODER_COUNT] = {0, 0};
static volatile uint8 wheel_encoder_previous_state[WHEEL_ENCODER_COUNT] = {0, 0};
static volatile uint32 wheel_encoder_edge_count[WHEEL_ENCODER_COUNT][2] =
{
    {0, 0},
    {0, 0},
};

typedef struct
{
    uint8 encoder;
    uint8 channel;
} wheel_encoder_callback_context_struct;

static wheel_encoder_callback_context_struct wheel_encoder_callback_context[4] =
{
    {WHEEL_ENCODER_MOTOR1, 0},
    {WHEEL_ENCODER_MOTOR1, 1},
    {WHEEL_ENCODER_MOTOR2, 0},
    {WHEEL_ENCODER_MOTOR2, 1},
};

// 索引为 previous_AB << 2 | current_AB。
// 合法相邻状态得到 ±1；不变或非法跨越状态得到 0。
static const int8 quadrature_transition[16] =
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0,
};

static uint8 wheel_encoder_read_state (wheel_encoder_enum encoder)
{
    if(WHEEL_ENCODER_MOTOR1 == encoder)
    {
        return (uint8)((gpio_get_level(MOTOR1_ENCODER_A_PIN) << 1)
                     | gpio_get_level(MOTOR1_ENCODER_B_PIN));
    }
    return (uint8)((gpio_get_level(MOTOR2_ENCODER_A_PIN) << 1)
                 | gpio_get_level(MOTOR2_ENCODER_B_PIN));
}

static void wheel_encoder_edge_callback (uint32 event, void *ptr)
{
    wheel_encoder_callback_context_struct *context =
        (wheel_encoder_callback_context_struct *)ptr;
    uint8 encoder = context->encoder;
    uint8 current_state;
    uint8 transition_index;

    (void)event;
    wheel_encoder_edge_count[encoder][context->channel] ++;
    current_state = wheel_encoder_read_state((wheel_encoder_enum)encoder);
    transition_index = (uint8)((wheel_encoder_previous_state[encoder] << 2)
                             | current_state);
    wheel_encoder_count[encoder] += quadrature_transition[transition_index];
    wheel_encoder_previous_state[encoder] = current_state;
}

void wheel_encoder_init (void)
{
    // 编码器由载板 5V 供电；输入使用上拉并在 A/B 两相双边沿计数。
    exti_init(MOTOR1_ENCODER_A_PIN, EXTI_TRIGGER_BOTH,
              wheel_encoder_edge_callback, &wheel_encoder_callback_context[0]);
    exti_init(MOTOR1_ENCODER_B_PIN, EXTI_TRIGGER_BOTH,
              wheel_encoder_edge_callback, &wheel_encoder_callback_context[1]);
    exti_init(MOTOR2_ENCODER_A_PIN, EXTI_TRIGGER_BOTH,
              wheel_encoder_edge_callback, &wheel_encoder_callback_context[2]);
    exti_init(MOTOR2_ENCODER_B_PIN, EXTI_TRIGGER_BOTH,
              wheel_encoder_edge_callback, &wheel_encoder_callback_context[3]);

    wheel_encoder_previous_state[WHEEL_ENCODER_MOTOR1] =
        wheel_encoder_read_state(WHEEL_ENCODER_MOTOR1);
    wheel_encoder_previous_state[WHEEL_ENCODER_MOTOR2] =
        wheel_encoder_read_state(WHEEL_ENCODER_MOTOR2);
    wheel_encoder_clear_all();
}

int32 wheel_encoder_get_count (wheel_encoder_enum encoder)
{
    return wheel_encoder_count[encoder];
}

uint32 wheel_encoder_get_edge_count (wheel_encoder_enum encoder, uint8 channel)
{
    return wheel_encoder_edge_count[encoder][channel];
}

uint8 wheel_encoder_get_state (wheel_encoder_enum encoder)
{
    return wheel_encoder_read_state(encoder);
}

void wheel_encoder_clear (wheel_encoder_enum encoder)
{
    wheel_encoder_count[encoder] = 0;
    wheel_encoder_edge_count[encoder][0] = 0;
    wheel_encoder_edge_count[encoder][1] = 0;
}

void wheel_encoder_clear_all (void)
{
    wheel_encoder_clear(WHEEL_ENCODER_MOTOR1);
    wheel_encoder_clear(WHEEL_ENCODER_MOTOR2);
}
