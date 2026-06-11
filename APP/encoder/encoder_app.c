#include "encoder_app.h"

#include "motor_app.h"
#include "tim_app.h"

static uint32_t encoder_app_last_sample_time;

void encoder_app_init(void)
{
    tim_app_encoder_reset();
    encoder_init();
    encoder_app_last_sample_time = encoder_get_last_sample_time();
}

void encoder_app_task(void)
{
    uint32_t sample_time;

    encoder_task();
    sample_time = encoder_get_last_sample_time();
    if (sample_time == encoder_app_last_sample_time) {
        return;
    }

    encoder_app_last_sample_time = sample_time;
    motor_app_set_speed_feedback(encoder_get_left_speed(),
                                 encoder_get_right_speed());
}

int32_t encoder_app_get_left_count(void)
{
    return encoder_get_left_count();
}

int32_t encoder_app_get_right_count(void)
{
    return encoder_get_right_count();
}

float encoder_app_get_left_speed(void)
{
    return encoder_get_left_speed();
}

float encoder_app_get_right_speed(void)
{
    return encoder_get_right_speed();
}

uint32_t encoder_app_get_last_sample_time(void)
{
    return encoder_get_last_sample_time();
}

void encoder_app_get_feedback(encoder_feedback_t *feedback)
{
    encoder_get_feedback(feedback);
}
