#ifndef __ENCODER_APP_H
#define __ENCODER_APP_H

#include <stdint.h>
#include "encoder.h"

#ifndef ENCODER_APP_TASK_PERIOD_MS
#define ENCODER_APP_TASK_PERIOD_MS       ENCODER_TASK_PERIOD_MS
#endif

void encoder_app_init(void);
void encoder_app_task(void);

int32_t encoder_app_get_left_count(void);
int32_t encoder_app_get_right_count(void);
float encoder_app_get_left_speed(void);
float encoder_app_get_right_speed(void);
uint32_t encoder_app_get_last_sample_time(void);
void encoder_app_get_feedback(encoder_feedback_t *feedback);

#endif
