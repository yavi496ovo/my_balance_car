#include "encoder.h"

#include "scheduler.h"
#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint16_t last_raw_count;
    volatile int32_t total_count;
    int16_t last_delta;
    float speed;
} encoder_channel_t;

static encoder_channel_t encoder_left;
static encoder_channel_t encoder_right;
static uint32_t encoder_last_sample_time;

static uint16_t encoder_read_raw_count(const encoder_channel_t *channel)
{
    if ((channel == 0) || (channel->timer == 0) ||
        (channel->timer->Instance == 0)) {
        return 0U;
    }

    return (uint16_t)__HAL_TIM_GET_COUNTER(channel->timer);
}

static int16_t encoder_update_channel(encoder_channel_t *channel)
{
    uint16_t raw_count;
    int16_t delta;

    raw_count = encoder_read_raw_count(channel);
    delta = (int16_t)(raw_count - channel->last_raw_count);
    channel->last_raw_count = raw_count;
    channel->last_delta = delta;
    channel->total_count += delta;

    return delta;
}

void encoder_init(void)
{
    encoder_left.timer = &htim2;
    encoder_left.last_raw_count = encoder_read_raw_count(&encoder_left);
    encoder_left.total_count = 0;
    encoder_left.last_delta = 0;
    encoder_left.speed = 0.0f;

    encoder_right.timer = &htim4;
    encoder_right.last_raw_count = encoder_read_raw_count(&encoder_right);
    encoder_right.total_count = 0;
    encoder_right.last_delta = 0;
    encoder_right.speed = 0.0f;

    encoder_last_sample_time = scheduler_get_tick();
}

void encoder_task(void)
{
    uint32_t now_time;
    uint32_t elapsed_ms;
    int16_t left_delta;
    int16_t right_delta;
    float normalize;

    now_time = scheduler_get_tick();
    elapsed_ms = now_time - encoder_last_sample_time;
    if (elapsed_ms < ENCODER_TASK_PERIOD_MS) {
        return;
    }

    left_delta = encoder_update_channel(&encoder_left);
    right_delta = encoder_update_channel(&encoder_right);
    encoder_last_sample_time = now_time;

    normalize = (float)ENCODER_TASK_PERIOD_MS / (float)elapsed_ms;
    encoder_left.speed = ENCODER_LEFT_SIGN * (float)left_delta * normalize;
    encoder_right.speed = ENCODER_RIGHT_SIGN * (float)right_delta * normalize;
}

int32_t encoder_get_left_count(void)
{
    return encoder_left.total_count;
}

int32_t encoder_get_right_count(void)
{
    return encoder_right.total_count;
}

int16_t encoder_get_left_delta(void)
{
    return encoder_left.last_delta;
}

int16_t encoder_get_right_delta(void)
{
    return encoder_right.last_delta;
}

float encoder_get_left_speed(void)
{
    return encoder_left.speed;
}

float encoder_get_right_speed(void)
{
    return encoder_right.speed;
}

uint32_t encoder_get_last_sample_time(void)
{
    return encoder_last_sample_time;
}

void encoder_get_feedback(encoder_feedback_t *feedback)
{
    if (feedback == 0) {
        return;
    }

    feedback->left_count = encoder_left.total_count;
    feedback->right_count = encoder_right.total_count;
    feedback->left_delta = encoder_left.last_delta;
    feedback->right_delta = encoder_right.last_delta;
    feedback->left_speed = encoder_left.speed;
    feedback->right_speed = encoder_right.speed;
    feedback->timestamp_ms = encoder_last_sample_time;
}
