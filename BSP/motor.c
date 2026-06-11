#include "motor.h"

#include "tim.h"

static uint8_t motor_enabled;
static uint16_t motor_deadzone;
static motor_output_t motor_output;

static int16_t motor_limit_pwm(int16_t pwm);
static uint32_t motor_pwm_to_ccr(int16_t pwm);
static void motor_write_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
static void motor_set_pwm_compare(uint32_t channel, uint32_t compare);
static void motor_set_left_dir(int16_t pwm);
static void motor_set_right_dir(int16_t pwm);

static int16_t motor_limit_pwm(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }

    if (pwm < -MOTOR_PWM_MAX) {
        return -MOTOR_PWM_MAX;
    }

    return pwm;
}

static uint32_t motor_pwm_to_ccr(int16_t pwm)
{
    uint16_t abs_pwm;
    uint32_t period_count;

    if (pwm < 0) {
        abs_pwm = (uint16_t)(-pwm);
    } else {
        abs_pwm = (uint16_t)pwm;
    }

    if (abs_pwm <= motor_deadzone) {
        abs_pwm = 0U;
    }

    if (abs_pwm > MOTOR_PWM_MAX) {
        abs_pwm = MOTOR_PWM_MAX;
    }

    if (MOTOR_PWM_TIMER.Instance != 0) {
        period_count = __HAL_TIM_GET_AUTORELOAD(&MOTOR_PWM_TIMER) + 1U;
    } else {
        period_count = MOTOR_PWM_PERIOD_COUNT;
    }

    return period_count -
           ((uint32_t)abs_pwm * period_count / MOTOR_PWM_MAX);
}

static void motor_write_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

static void motor_set_pwm_compare(uint32_t channel, uint32_t compare)
{
    if (MOTOR_PWM_TIMER.Instance != 0) {
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIMER, channel, compare);
    }
}

static void motor_set_left_dir(int16_t pwm)
{
    if (pwm > 0) {
        motor_write_pin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_SET);
        motor_write_pin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
    } else if (pwm < 0) {
        motor_write_pin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
        motor_write_pin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_SET);
    } else {
        motor_write_pin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_RESET);
        motor_write_pin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_RESET);
    }
}

static void motor_set_right_dir(int16_t pwm)
{
    if (pwm > 0) {
        motor_write_pin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_SET);
        motor_write_pin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
    } else if (pwm < 0) {
        motor_write_pin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
        motor_write_pin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_SET);
    } else {
        motor_write_pin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_RESET);
        motor_write_pin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_RESET);
    }
}

void motor_init(void)
{
    motor_enabled = 0U;
    motor_deadzone = 0U;
    motor_output.left = 0;
    motor_output.right = 0;
    motor_stop();
}

void motor_enable(uint8_t enable)
{
    motor_enabled = (enable != 0U) ? 1U : 0U;

    if (motor_enabled != 0U) {
        motor_write_pin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
    } else {
        motor_stop();
        motor_write_pin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
    }
}

uint8_t motor_is_enabled(void)
{
    return motor_enabled;
}

void motor_set_single(motor_id_t id, int16_t pwm)
{
    int16_t limited_pwm;

    limited_pwm = motor_limit_pwm(pwm);
    if (motor_enabled == 0U) {
        limited_pwm = 0;
    }

    if (id == MOTOR_ID_LEFT) {
        motor_set_left_dir(limited_pwm);
        motor_set_pwm_compare(MOTOR_PWM_LEFT_CHANNEL,
                              motor_pwm_to_ccr(limited_pwm));
        motor_output.left = limited_pwm;
    } else {
        motor_set_right_dir(limited_pwm);
        motor_set_pwm_compare(MOTOR_PWM_RIGHT_CHANNEL,
                              motor_pwm_to_ccr(limited_pwm));
        motor_output.right = limited_pwm;
    }
}

void motor_set_output(int16_t left_pwm, int16_t right_pwm)
{
    motor_set_single(MOTOR_ID_LEFT, left_pwm);
    motor_set_single(MOTOR_ID_RIGHT, right_pwm);
}

void motor_stop(void)
{
    motor_set_pwm_compare(MOTOR_PWM_LEFT_CHANNEL, motor_pwm_to_ccr(0));
    motor_set_pwm_compare(MOTOR_PWM_RIGHT_CHANNEL, motor_pwm_to_ccr(0));

    motor_set_left_dir(0);
    motor_set_right_dir(0);
    motor_output.left = 0;
    motor_output.right = 0;
}

void motor_brake(void)
{
    motor_set_pwm_compare(MOTOR_PWM_LEFT_CHANNEL, motor_pwm_to_ccr(0));
    motor_set_pwm_compare(MOTOR_PWM_RIGHT_CHANNEL, motor_pwm_to_ccr(0));

    motor_write_pin(MOTOR_LEFT_IN1_PORT, MOTOR_LEFT_IN1_PIN, GPIO_PIN_SET);
    motor_write_pin(MOTOR_LEFT_IN2_PORT, MOTOR_LEFT_IN2_PIN, GPIO_PIN_SET);
    motor_write_pin(MOTOR_RIGHT_IN1_PORT, MOTOR_RIGHT_IN1_PIN, GPIO_PIN_SET);
    motor_write_pin(MOTOR_RIGHT_IN2_PORT, MOTOR_RIGHT_IN2_PIN, GPIO_PIN_SET);
    motor_output.left = 0;
    motor_output.right = 0;
}

void motor_set_deadzone(uint16_t deadzone)
{
    if (deadzone > MOTOR_PWM_MAX) {
        motor_deadzone = MOTOR_PWM_MAX;
    } else {
        motor_deadzone = deadzone;
    }
}

motor_output_t motor_get_output(void)
{
    return motor_output;
}

void Motor_Stop(void)
{
    motor_stop();
}
