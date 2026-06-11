#ifndef __JY61P_H
#define __JY61P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "ringbuffer.h"

#define JY61P_PACKET_SIZE              (11U)
#define JY61P_FRAME_SIZE               JY61P_PACKET_SIZE
#define JY61P_FRAME_HEADER             (0x55U)
#define JY61P_DATA_SIZE                (8U)

#define JY61P_TYPE_ACC                 (0x51U)
#define JY61P_TYPE_GYRO                (0x52U)
#define JY61P_TYPE_ANGLE               (0x53U)

#define JY61P_ACC_SCALE                (16.0f / 32768.0f)
#define JY61P_GYRO_SCALE               (2000.0f / 32768.0f)
#define JY61P_ANGLE_SCALE              (180.0f / 32768.0f)
#define JY61P_TEMP_SCALE               (100.0f / 32768.0f)

#define JY61P_FILTER_ACC               (0x01U)
#define JY61P_FILTER_GYRO              (0x02U)
#define JY61P_FILTER_ANGLE             (0x04U)
#define JY61P_FILTER_ALL               (JY61P_FILTER_ACC | JY61P_FILTER_GYRO | JY61P_FILTER_ANGLE)

#define JY61P_AXIS_X                   (0U)
#define JY61P_AXIS_Y                   (1U)
#define JY61P_AXIS_Z                   (2U)

#ifndef JY61P_CONTROL_ANGLE_AXIS
#define JY61P_CONTROL_ANGLE_AXIS       JY61P_AXIS_Y
#endif

#ifndef JY61P_CONTROL_GYRO_AXIS
#define JY61P_CONTROL_GYRO_AXIS        JY61P_CONTROL_ANGLE_AXIS
#endif

#ifndef JY61P_CONTROL_ANGLE_SIGN
#define JY61P_CONTROL_ANGLE_SIGN       (1.0f)
#endif

#ifndef JY61P_CONTROL_GYRO_SIGN
#define JY61P_CONTROL_GYRO_SIGN        (1.0f)
#endif

#ifndef JY61P_CONTROL_ANGLE_OFFSET_DEG
#define JY61P_CONTROL_ANGLE_OFFSET_DEG (0.0f)
#endif

#ifndef JY61P_ANGLE_LPF_ALPHA
#define JY61P_ANGLE_LPF_ALPHA          (0.6f)
#endif

#ifndef JY61P_GYRO_LPF_ALPHA
#define JY61P_GYRO_LPF_ALPHA           (0.6f)
#endif

#ifndef JY61P_ENABLE_GYRO_CALIBRATION
#define JY61P_ENABLE_GYRO_CALIBRATION  (0U)
#endif

#ifndef JY61P_GYRO_CALIB_SAMPLES
#define JY61P_GYRO_CALIB_SAMPLES       (50U)
#endif

#ifndef JY61P_GYRO_STABLE_THRESHOLD_DPS
#define JY61P_GYRO_STABLE_THRESHOLD_DPS (15.0f)
#endif

#ifndef JY61P_DATA_TIMEOUT_MS
#define JY61P_DATA_TIMEOUT_MS          (100U)
#endif

typedef enum {
    JY61P_STATE_HEADER = 0,
    JY61P_STATE_TYPE,
    JY61P_STATE_DATA,
    JY61P_STATE_CHECKSUM
} JY61P_ParseState_t;

typedef struct {
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float angle_x_deg;
    float angle_y_deg;
    float angle_z_deg;

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float control_angle_raw_deg;
    float control_angle_deg;
    float control_gyro_raw_dps;
    float control_gyro_dps;
    float control_gyro_bias_dps;

    float temperature_c;

    uint8_t acc_valid;
    uint8_t gyro_valid;
    uint8_t angle_valid;
    uint8_t frame_valid;
    uint8_t gyro_calibrated;
    uint8_t control_ready;

    uint32_t timestamp_ms;
    uint16_t gyro_calibration_samples;
} JY61P_Data_t;

typedef struct {
    uint8_t packet_buffer[JY61P_PACKET_SIZE];
    uint8_t packet_index;
    uint8_t packet_type;
    JY61P_ParseState_t parse_state;

    JY61P_Data_t data;

    float gyro_bias_sum_dps;
    float gyro_bias_dps;

    uint16_t gyro_calibration_samples;
    uint8_t gyro_calibrated;
    uint8_t control_angle_initialized;
    uint8_t control_gyro_initialized;

    uint8_t initialized;
    uint8_t data_ready;
    uint8_t data_filter_mask;

    uint32_t last_update_time;
} JY61P_Handle_t;

int8_t JY61P_Init(JY61P_Handle_t *handle, uint32_t now_ms);
int8_t JY61P_ParseByte(JY61P_Handle_t *handle, uint8_t byte, uint32_t now_ms);
uint32_t JY61P_ProcessRingBuffer(JY61P_Handle_t *handle,
                                  ringbuffer_t *rb,
                                  uint32_t now_ms);
const JY61P_Data_t *JY61P_GetData(const JY61P_Handle_t *handle);
uint8_t JY61P_IsDataReady(const JY61P_Handle_t *handle);
void JY61P_ClearDataReady(JY61P_Handle_t *handle);
uint8_t JY61P_IsDataValid(const JY61P_Handle_t *handle,
                          uint8_t check_acc,
                          uint8_t check_gyro,
                          uint8_t check_angle,
                          uint32_t now_ms);
uint8_t JY61P_IsControlReady(const JY61P_Handle_t *handle, uint32_t now_ms);
uint8_t JY61P_IsDataTimeout(const JY61P_Handle_t *handle, uint32_t now_ms);
uint32_t JY61P_GetDataTimestamp(const JY61P_Handle_t *handle);
uint32_t JY61P_GetDataAge(const JY61P_Handle_t *handle, uint32_t now_ms);
void JY61P_SetDataFilter(JY61P_Handle_t *handle, uint8_t mask);
uint8_t JY61P_IsInitialized(const JY61P_Handle_t *handle);

void JY61P_StartGyroCalibration(JY61P_Handle_t *handle, uint32_t now_ms);
uint8_t JY61P_IsGyroCalibrated(const JY61P_Handle_t *handle);
uint8_t JY61P_GetGyroCalibrationProgress(const JY61P_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif
