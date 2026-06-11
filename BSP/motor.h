#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/* 底层 PWM 对外统一使用 -1000 ~ 1000。 */
#ifndef MOTOR_PWM_MAX
#define MOTOR_PWM_MAX                    (1000)
#endif

/* TIM3 PWM period is ARR + 1. Current CubeMX setting: 8399 + 1. */
#ifndef MOTOR_PWM_PERIOD_COUNT
#define MOTOR_PWM_PERIOD_COUNT           (8400U)
#endif

#ifndef MOTOR_PWM_TIMER
#define MOTOR_PWM_TIMER                  htim3
#endif

#ifndef MOTOR_PWM_LEFT_CHANNEL
#define MOTOR_PWM_LEFT_CHANNEL           TIM_CHANNEL_1
#endif

#ifndef MOTOR_PWM_RIGHT_CHANNEL
#define MOTOR_PWM_RIGHT_CHANNEL          TIM_CHANNEL_2
#endif

#ifndef MOTOR_LEFT_IN1_PORT
#define MOTOR_LEFT_IN1_PORT              GPIOB
#endif

#ifndef MOTOR_LEFT_IN1_PIN
#define MOTOR_LEFT_IN1_PIN               GPIO_PIN_12
#endif

#ifndef MOTOR_LEFT_IN2_PORT
#define MOTOR_LEFT_IN2_PORT              GPIOB
#endif

#ifndef MOTOR_LEFT_IN2_PIN
#define MOTOR_LEFT_IN2_PIN               GPIO_PIN_13
#endif

#ifndef MOTOR_RIGHT_IN1_PORT
#define MOTOR_RIGHT_IN1_PORT             GPIOB
#endif

#ifndef MOTOR_RIGHT_IN1_PIN
#define MOTOR_RIGHT_IN1_PIN              GPIO_PIN_14
#endif

#ifndef MOTOR_RIGHT_IN2_PORT
#define MOTOR_RIGHT_IN2_PORT             GPIOB
#endif

#ifndef MOTOR_RIGHT_IN2_PIN
#define MOTOR_RIGHT_IN2_PIN              GPIO_PIN_15
#endif

#ifndef MOTOR_STBY_PORT
#define MOTOR_STBY_PORT                  GPIOE
#endif

#ifndef MOTOR_STBY_PIN
#define MOTOR_STBY_PIN                   GPIO_PIN_0
#endif

typedef enum {
    MOTOR_ID_LEFT = 0,
    MOTOR_ID_RIGHT
} motor_id_t;

typedef struct {
    int16_t left;
    int16_t right;
} motor_output_t;

void motor_init(void);
void motor_enable(uint8_t enable);
uint8_t motor_is_enabled(void);
void motor_set_single(motor_id_t id, int16_t pwm);
void motor_set_output(int16_t left_pwm, int16_t right_pwm);
void motor_stop(void);
void motor_brake(void);
void motor_set_deadzone(uint16_t deadzone);
motor_output_t motor_get_output(void);

/* 兼容旧代码里可能使用的命名。 */
void Motor_Stop(void);

#endif
