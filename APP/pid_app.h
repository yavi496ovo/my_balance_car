#ifndef __PID_APP_H
#define __PID_APP_H

#include <stdint.h>

#define PID_APP_TASK_PERIOD_MS           (5U)

typedef struct {
    float angle_deg;
    float target_angle_deg;
    float speed_angle_deg;
    float speed_target;
    float speed_feedback;
    float gyro_dps;
    float balance_pwm;
    float turn_pwm;
    float left_pwm;
    float right_pwm;
    uint8_t sensor_ready;
    uint8_t running;
    uint8_t fault;
} pid_app_debug_t;

void pid_app_init(void);
void pid_app_task(void);
void pid_app_update_sensor(float angle_deg, float gyro_dps, uint8_t ready);
void pid_app_update_encoder(float left_speed, float right_speed);
void pid_app_get_debug(pid_app_debug_t *debug);

#endif
