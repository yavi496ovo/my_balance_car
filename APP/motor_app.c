#include "motor_app.h"

#include "motor.h"

void motor_app_init(void)
{
    motor_init();
    pid_app_init();
}

void motor_app_task(void)
{
    pid_app_task();
}

void motor_app_update_sensor(float angle_deg,
                             float gyro_dps,
                             uint8_t ready,
                             uint32_t timestamp_ms)
{
    (void)timestamp_ms;
    pid_app_update_sensor(angle_deg, gyro_dps, ready);
}

void motor_app_set_speed_feedback(float left_speed, float right_speed)
{
    pid_app_update_encoder(left_speed, right_speed);
}

void motor_app_get_debug(motor_app_debug_t *debug)
{
    pid_app_get_debug(debug);
}
