#include "tim_app.h"

#include "tim.h"

static void tim_app_set_pwm_idle(void)
{
    uint32_t period_count;

    period_count = tim_app_get_pwm_period_count();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period_count);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, period_count);
}

void tim_app_init(void)
{
    MX_TIM3_Init();
    tim_app_set_pwm_idle();
    tim_app_pwm_start();

    MX_TIM2_Init();
    MX_TIM4_Init();
    tim_app_encoder_reset();
    tim_app_encoder_start();
}

void tim_app_pwm_start(void)
{
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

void tim_app_pwm_stop(void)
{
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
}

uint32_t tim_app_get_pwm_period_count(void)
{
    if (htim3.Instance == 0) {
        return 0U;
    }

    return __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
}

void tim_app_encoder_start(void)
{
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
}

void tim_app_encoder_stop(void)
{
    (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
}

void tim_app_encoder_reset(void)
{
    if (htim2.Instance != 0) {
        __HAL_TIM_SET_COUNTER(&htim2, 0U);
    }

    if (htim4.Instance != 0) {
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
    }
}
