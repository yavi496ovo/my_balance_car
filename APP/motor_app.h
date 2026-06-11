#ifndef __MOTOR_APP_H
#define __MOTOR_APP_H

#include "pid_app.h"

#define MOTOR_APP_TASK_PERIOD_MS         PID_APP_TASK_PERIOD_MS

typedef pid_app_debug_t motor_app_debug_t;

void motor_app_init(void);
void motor_app_task(void);

void motor_app_update_sensor(float angle_deg,
                             float gyro_dps,
                             uint8_t ready,
                             uint32_t timestamp_ms);
void motor_app_set_speed_feedback(float left_speed, float right_speed);
void motor_app_get_debug(motor_app_debug_t *debug);

#endif
