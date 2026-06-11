#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

#ifndef ENCODER_TASK_PERIOD_MS
#define ENCODER_TASK_PERIOD_MS          (10U)
#endif

#ifndef ENCODER_LEFT_SIGN
#define ENCODER_LEFT_SIGN               (-1.0f)    /* 左轮前进时若速度为负，就改成 -1 */
#endif

#ifndef ENCODER_RIGHT_SIGN
#define ENCODER_RIGHT_SIGN              (1.0f)     /* 右轮前进时若速度为负，就改成 -1 */
#endif

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int16_t left_delta;
    int16_t right_delta;
    float left_speed;
    float right_speed;
    uint32_t timestamp_ms;
} encoder_feedback_t;

void encoder_init(void);
void encoder_task(void);

int32_t encoder_get_left_count(void);
int32_t encoder_get_right_count(void);
int16_t encoder_get_left_delta(void);
int16_t encoder_get_right_delta(void);
float encoder_get_left_speed(void);
float encoder_get_right_speed(void);
uint32_t encoder_get_last_sample_time(void);
void encoder_get_feedback(encoder_feedback_t *feedback);

#endif
