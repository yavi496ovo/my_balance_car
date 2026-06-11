#include "jy61p_app.h"

#include "motor_app.h"
#include "scheduler.h"
#include "uart_app.h"

static JY61P_Handle_t jy61p_app_handle;
static uint32_t jy61p_app_parsed_frame_count;

static void jy61p_app_update_motor_sensor(uint32_t now_ms)
{
    const JY61P_Data_t *data;
    uint8_t ready;

    data = JY61P_GetData(&jy61p_app_handle);
    if (data == 0) {
        motor_app_update_sensor(0.0f, 0.0f, 0U, now_ms);
        return;
    }

    ready = JY61P_IsControlReady(&jy61p_app_handle, now_ms);
    motor_app_update_sensor(data->control_angle_deg,
                            data->control_gyro_dps,
                            ready,
                            data->timestamp_ms);
}

void jy61p_app_init(void)
{
    jy61p_app_parsed_frame_count = 0U;
    (void)JY61P_Init(&jy61p_app_handle, scheduler_get_tick());
    JY61P_SetDataFilter(&jy61p_app_handle,
                        (uint8_t)(JY61P_FILTER_GYRO | JY61P_FILTER_ANGLE));
}

void jy61p_app_task(void)
{
    uint32_t now_ms;
    uint32_t parsed_count;

    now_ms = scheduler_get_tick();
    parsed_count = JY61P_ProcessRingBuffer(&jy61p_app_handle,
                                           uart_app_get_rx_ringbuffer(),
                                           now_ms);
    jy61p_app_parsed_frame_count += parsed_count;

    if ((parsed_count != 0U) ||
        (JY61P_IsDataReady(&jy61p_app_handle) != 0U) ||
        (JY61P_IsDataTimeout(&jy61p_app_handle, now_ms) != 0U)) {
        jy61p_app_update_motor_sensor(now_ms);
        JY61P_ClearDataReady(&jy61p_app_handle);
    }
}

const JY61P_Data_t *jy61p_app_get_data(void)
{
    return JY61P_GetData(&jy61p_app_handle);
}

uint8_t jy61p_app_is_control_ready(void)
{
    return JY61P_IsControlReady(&jy61p_app_handle, scheduler_get_tick());
}

uint32_t jy61p_app_get_parsed_frame_count(void)
{
    return jy61p_app_parsed_frame_count;
}

uint32_t jy61p_app_get_data_age(void)
{
    return JY61P_GetDataAge(&jy61p_app_handle, scheduler_get_tick());
}

void jy61p_app_start_gyro_calibration(void)
{
    JY61P_StartGyroCalibration(&jy61p_app_handle, scheduler_get_tick());
}
