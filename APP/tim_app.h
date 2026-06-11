#ifndef __TIM_APP_H
#define __TIM_APP_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

void tim_app_init(void);
void tim_app_pwm_start(void);
void tim_app_pwm_stop(void);
uint32_t tim_app_get_pwm_period_count(void);
void tim_app_encoder_start(void);
void tim_app_encoder_stop(void);
void tim_app_encoder_reset(void);

#endif
