#include "jy61p.h"

#include <string.h>

static int16_t JY61P_MakeInt16(uint8_t low, uint8_t high);
#if (JY61P_ENABLE_GYRO_CALIBRATION != 0U)
static float JY61P_Abs(float value);
#endif
static float JY61P_LowPass(float current, float target, float alpha);
static float JY61P_SelectAxisValue(float x_value,
                                   float y_value,
                                   float z_value,
                                   uint8_t axis);
static uint8_t JY61P_CalculateChecksum(const uint8_t *frame, uint8_t len);
static void JY61P_ResetParser(JY61P_Handle_t *handle);
static void JY61P_InitDataStructure(JY61P_Handle_t *handle, uint32_t now_ms);
static int8_t JY61P_ProcessPacket(JY61P_Handle_t *handle, uint32_t now_ms);
static void JY61P_ConvertAccData(JY61P_Handle_t *handle, const uint8_t *data);
static void JY61P_ConvertGyroData(JY61P_Handle_t *handle, const uint8_t *data);
static void JY61P_ConvertAngleData(JY61P_Handle_t *handle, const uint8_t *data);
static void JY61P_UpdateControlAngle(JY61P_Handle_t *handle);
static void JY61P_ProcessGyroCalibration(JY61P_Handle_t *handle,
                                         float selected_gyro_raw_dps);
static void JY61P_UpdateControlGyro(JY61P_Handle_t *handle);
static void JY61P_UpdateControlReady(JY61P_Handle_t *handle, uint32_t now_ms);

static int16_t JY61P_MakeInt16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

#if (JY61P_ENABLE_GYRO_CALIBRATION != 0U)
static float JY61P_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}
#endif

static float JY61P_LowPass(float current, float target, float alpha)
{
    return current + alpha * (target - current);
}

static float JY61P_SelectAxisValue(float x_value,
                                   float y_value,
                                   float z_value,
                                   uint8_t axis)
{
    switch (axis) {
        case JY61P_AXIS_X:
            return x_value;

        case JY61P_AXIS_Y:
            return y_value;

        case JY61P_AXIS_Z:
        default:
            return z_value;
    }
}

static uint8_t JY61P_CalculateChecksum(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    uint8_t sum;

    sum = 0U;
    for (i = 0U; i < len; i++) {
        sum = (uint8_t)(sum + frame[i]);
    }

    return sum;
}

static void JY61P_ResetParser(JY61P_Handle_t *handle)
{
    handle->parse_state = JY61P_STATE_HEADER;
    handle->packet_index = 0U;
    handle->packet_type = 0U;
}

static void JY61P_InitDataStructure(JY61P_Handle_t *handle, uint32_t now_ms)
{
    memset(&handle->data, 0, sizeof(handle->data));

    handle->gyro_bias_sum_dps = 0.0f;
    handle->gyro_bias_dps = 0.0f;
    handle->gyro_calibration_samples = 0U;
    handle->gyro_calibrated = 0U;
    handle->control_angle_initialized = 0U;
    handle->control_gyro_initialized = 0U;

    handle->data_ready = 0U;
    handle->data_filter_mask = JY61P_FILTER_ALL;
    handle->last_update_time = now_ms;

#if (JY61P_ENABLE_GYRO_CALIBRATION == 0U)
    handle->gyro_calibrated = 1U;
    handle->data.gyro_calibrated = 1U;
#endif
}

int8_t JY61P_Init(JY61P_Handle_t *handle, uint32_t now_ms)
{
    if (handle == 0) {
        return -1;
    }

    memset(handle, 0, sizeof(JY61P_Handle_t));
    JY61P_InitDataStructure(handle, now_ms);
    JY61P_ResetParser(handle);
    handle->initialized = 1U;
    JY61P_UpdateControlReady(handle, now_ms);

    return 0;
}

void JY61P_StartGyroCalibration(JY61P_Handle_t *handle, uint32_t now_ms)
{
    if (handle == 0) {
        return;
    }

    handle->gyro_bias_sum_dps = 0.0f;
    handle->gyro_bias_dps = 0.0f;
    handle->gyro_calibration_samples = 0U;
    handle->gyro_calibrated = 0U;

#if (JY61P_ENABLE_GYRO_CALIBRATION == 0U)
    handle->gyro_calibrated = 1U;
#endif

    handle->data.control_gyro_bias_dps = 0.0f;
    handle->data.gyro_calibration_samples = 0U;
    handle->data.gyro_calibrated = handle->gyro_calibrated;
    JY61P_UpdateControlReady(handle, now_ms);
}

static void JY61P_UpdateControlAngle(JY61P_Handle_t *handle)
{
    float selected_angle_deg;

    selected_angle_deg = JY61P_SelectAxisValue(handle->data.angle_x_deg,
                                               handle->data.angle_y_deg,
                                               handle->data.angle_z_deg,
                                               JY61P_CONTROL_ANGLE_AXIS);
    selected_angle_deg = selected_angle_deg * JY61P_CONTROL_ANGLE_SIGN;
    selected_angle_deg -= JY61P_CONTROL_ANGLE_OFFSET_DEG;

    handle->data.control_angle_raw_deg = selected_angle_deg;

    if (handle->control_angle_initialized == 0U) {
        handle->data.control_angle_deg = selected_angle_deg;
        handle->control_angle_initialized = 1U;
    } else {
        handle->data.control_angle_deg =
            JY61P_LowPass(handle->data.control_angle_deg,
                          selected_angle_deg,
                          JY61P_ANGLE_LPF_ALPHA);
    }
}

static void JY61P_ProcessGyroCalibration(JY61P_Handle_t *handle,
                                         float selected_gyro_raw_dps)
{
#if (JY61P_ENABLE_GYRO_CALIBRATION == 0U)
    (void)selected_gyro_raw_dps;
    handle->gyro_calibrated = 1U;
    handle->data.gyro_calibrated = 1U;
    handle->data.gyro_calibration_samples = 0U;
    handle->data.control_gyro_bias_dps = 0.0f;
    return;
#else
    if (handle->gyro_calibrated != 0U) {
        handle->data.gyro_calibrated = 1U;
        handle->data.gyro_calibration_samples = handle->gyro_calibration_samples;
        handle->data.control_gyro_bias_dps = handle->gyro_bias_dps;
        return;
    }

    if (JY61P_Abs(selected_gyro_raw_dps) <= JY61P_GYRO_STABLE_THRESHOLD_DPS) {
        handle->gyro_bias_sum_dps += selected_gyro_raw_dps;
        handle->gyro_calibration_samples++;
    }

    if (handle->gyro_calibration_samples >= JY61P_GYRO_CALIB_SAMPLES) {
        handle->gyro_bias_dps =
            handle->gyro_bias_sum_dps / (float)handle->gyro_calibration_samples;
        handle->gyro_calibrated = 1U;
    }

    handle->data.gyro_calibrated = handle->gyro_calibrated;
    handle->data.gyro_calibration_samples = handle->gyro_calibration_samples;
    handle->data.control_gyro_bias_dps = handle->gyro_bias_dps;
#endif
}

static void JY61P_UpdateControlGyro(JY61P_Handle_t *handle)
{
    float selected_gyro_raw_dps;
    float gyro_corrected_dps;

    selected_gyro_raw_dps = JY61P_SelectAxisValue(handle->data.gyro_x_dps,
                                                  handle->data.gyro_y_dps,
                                                  handle->data.gyro_z_dps,
                                                  JY61P_CONTROL_GYRO_AXIS);
    selected_gyro_raw_dps = selected_gyro_raw_dps * JY61P_CONTROL_GYRO_SIGN;

    handle->data.control_gyro_raw_dps = selected_gyro_raw_dps;
    JY61P_ProcessGyroCalibration(handle, selected_gyro_raw_dps);

    gyro_corrected_dps = selected_gyro_raw_dps - handle->gyro_bias_dps;
    if (handle->control_gyro_initialized == 0U) {
        handle->data.control_gyro_dps = gyro_corrected_dps;
        handle->control_gyro_initialized = 1U;
    } else {
        handle->data.control_gyro_dps =
            JY61P_LowPass(handle->data.control_gyro_dps,
                          gyro_corrected_dps,
                          JY61P_GYRO_LPF_ALPHA);
    }
}

uint8_t JY61P_IsDataTimeout(const JY61P_Handle_t *handle, uint32_t now_ms)
{
    if ((handle == 0) || (handle->initialized == 0U)) {
        return 1U;
    }

    return ((uint32_t)(now_ms - handle->last_update_time) >
            JY61P_DATA_TIMEOUT_MS) ? 1U : 0U;
}

static void JY61P_UpdateControlReady(JY61P_Handle_t *handle, uint32_t now_ms)
{
    uint8_t gyro_ready;

    if (handle == 0) {
        return;
    }

#if (JY61P_ENABLE_GYRO_CALIBRATION == 0U)
    gyro_ready = 1U;
#else
    gyro_ready = handle->gyro_calibrated;
#endif

    handle->data.control_ready = (uint8_t)(handle->data.angle_valid &&
                                           handle->data.gyro_valid &&
                                           (JY61P_IsDataTimeout(handle, now_ms) == 0U) &&
                                           gyro_ready);
}

static int8_t JY61P_ProcessPacket(JY61P_Handle_t *handle, uint32_t now_ms)
{
    const uint8_t *data;
    uint8_t data_updated;

    data = &handle->packet_buffer[2];
    data_updated = 0U;

    switch (handle->packet_type) {
        case JY61P_TYPE_ACC:
            if ((handle->data_filter_mask & JY61P_FILTER_ACC) != 0U) {
                JY61P_ConvertAccData(handle, data);
                handle->data.acc_valid = 1U;
                data_updated = 1U;
            }
            break;

        case JY61P_TYPE_GYRO:
            if ((handle->data_filter_mask & JY61P_FILTER_GYRO) != 0U) {
                JY61P_ConvertGyroData(handle, data);
                handle->data.gyro_valid = 1U;
                JY61P_UpdateControlGyro(handle);
                data_updated = 1U;
            }
            break;

        case JY61P_TYPE_ANGLE:
            if ((handle->data_filter_mask & JY61P_FILTER_ANGLE) != 0U) {
                JY61P_ConvertAngleData(handle, data);
                handle->data.angle_valid = 1U;
                JY61P_UpdateControlAngle(handle);
                data_updated = 1U;
            }
            break;

        default:
            return 0;
    }

    if (data_updated != 0U) {
        handle->data.frame_valid =
            (uint8_t)(handle->data.angle_valid && handle->data.gyro_valid);
        handle->data.timestamp_ms = now_ms;
        handle->last_update_time = now_ms;
        handle->data_ready = 1U;
        JY61P_UpdateControlReady(handle, now_ms);
        return 1;
    }

    JY61P_UpdateControlReady(handle, now_ms);
    return 0;
}

static void JY61P_ConvertAccData(JY61P_Handle_t *handle, const uint8_t *data)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int16_t raw_temp;

    raw_x = JY61P_MakeInt16(data[0], data[1]);
    raw_y = JY61P_MakeInt16(data[2], data[3]);
    raw_z = JY61P_MakeInt16(data[4], data[5]);
    raw_temp = JY61P_MakeInt16(data[6], data[7]);

    handle->data.acc_x_g = (float)raw_x * JY61P_ACC_SCALE;
    handle->data.acc_y_g = (float)raw_y * JY61P_ACC_SCALE;
    handle->data.acc_z_g = (float)raw_z * JY61P_ACC_SCALE;
    handle->data.temperature_c = (float)raw_temp * JY61P_TEMP_SCALE;
}

static void JY61P_ConvertGyroData(JY61P_Handle_t *handle, const uint8_t *data)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int16_t raw_temp;

    raw_x = JY61P_MakeInt16(data[0], data[1]);
    raw_y = JY61P_MakeInt16(data[2], data[3]);
    raw_z = JY61P_MakeInt16(data[4], data[5]);
    raw_temp = JY61P_MakeInt16(data[6], data[7]);

    handle->data.gyro_x_dps = (float)raw_x * JY61P_GYRO_SCALE;
    handle->data.gyro_y_dps = (float)raw_y * JY61P_GYRO_SCALE;
    handle->data.gyro_z_dps = (float)raw_z * JY61P_GYRO_SCALE;
    handle->data.temperature_c = (float)raw_temp * JY61P_TEMP_SCALE;
}

static void JY61P_ConvertAngleData(JY61P_Handle_t *handle, const uint8_t *data)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int16_t raw_temp;

    raw_x = JY61P_MakeInt16(data[0], data[1]);
    raw_y = JY61P_MakeInt16(data[2], data[3]);
    raw_z = JY61P_MakeInt16(data[4], data[5]);
    raw_temp = JY61P_MakeInt16(data[6], data[7]);

    handle->data.angle_x_deg = (float)raw_x * JY61P_ANGLE_SCALE;
    handle->data.angle_y_deg = (float)raw_y * JY61P_ANGLE_SCALE;
    handle->data.angle_z_deg = (float)raw_z * JY61P_ANGLE_SCALE;

    handle->data.roll_deg = handle->data.angle_x_deg;
    handle->data.pitch_deg = handle->data.angle_y_deg;
    handle->data.yaw_deg = handle->data.angle_z_deg;
    handle->data.temperature_c = (float)raw_temp * JY61P_TEMP_SCALE;
}

int8_t JY61P_ParseByte(JY61P_Handle_t *handle, uint8_t byte, uint32_t now_ms)
{
    uint8_t checksum;
    int8_t result;

    if (handle == 0) {
        return -1;
    }

    if (handle->initialized == 0U) {
        (void)JY61P_Init(handle, now_ms);
    }

    switch (handle->parse_state) {
        case JY61P_STATE_HEADER:
            if (byte == JY61P_FRAME_HEADER) {
                handle->packet_buffer[0] = byte;
                handle->packet_index = 1U;
                handle->parse_state = JY61P_STATE_TYPE;
            }
            break;

        case JY61P_STATE_TYPE:
            if (byte == JY61P_FRAME_HEADER) {
                handle->packet_buffer[0] = byte;
                handle->packet_index = 1U;
                handle->parse_state = JY61P_STATE_TYPE;
                break;
            }

            handle->packet_buffer[1] = byte;
            handle->packet_type = byte;
            handle->packet_index = 2U;
            handle->parse_state = JY61P_STATE_DATA;
            break;

        case JY61P_STATE_DATA:
            handle->packet_buffer[handle->packet_index] = byte;
            handle->packet_index++;

            if (handle->packet_index >= JY61P_PACKET_SIZE) {
                handle->parse_state = JY61P_STATE_CHECKSUM;
                checksum = JY61P_CalculateChecksum(handle->packet_buffer,
                                                   JY61P_PACKET_SIZE - 1U);

                if (checksum != handle->packet_buffer[JY61P_PACKET_SIZE - 1U]) {
                    JY61P_ResetParser(handle);
                    JY61P_UpdateControlReady(handle, now_ms);
                    return -1;
                }

                result = JY61P_ProcessPacket(handle, now_ms);
                JY61P_ResetParser(handle);

                if (result < 0) {
                    JY61P_UpdateControlReady(handle, now_ms);
                    return -1;
                }

                return result;
            }
            break;

        case JY61P_STATE_CHECKSUM:
        default:
            JY61P_ResetParser(handle);
            break;
    }

    JY61P_UpdateControlReady(handle, now_ms);
    return 0;
}

uint32_t JY61P_ProcessRingBuffer(JY61P_Handle_t *handle,
                                  ringbuffer_t *rb,
                                  uint32_t now_ms)
{
    uint8_t byte;
    uint32_t parsed_count;

    if ((handle == 0) || (rb == 0)) {
        return 0U;
    }

    if (handle->initialized == 0U) {
        (void)JY61P_Init(handle, now_ms);
    }

    parsed_count = 0U;
    while (ringbuffer_read(rb, &byte) == 1U) {
        if (JY61P_ParseByte(handle, byte, now_ms) > 0) {
            parsed_count++;
        }
    }

    JY61P_UpdateControlReady(handle, now_ms);
    return parsed_count;
}

const JY61P_Data_t *JY61P_GetData(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0;
    }

    return &handle->data;
}

uint8_t JY61P_IsDataReady(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0U;
    }

    return handle->data_ready;
}

void JY61P_ClearDataReady(JY61P_Handle_t *handle)
{
    if (handle != 0) {
        handle->data_ready = 0U;
    }
}

uint8_t JY61P_IsDataValid(const JY61P_Handle_t *handle,
                          uint8_t check_acc,
                          uint8_t check_gyro,
                          uint8_t check_angle,
                          uint32_t now_ms)
{
    if (handle == 0) {
        return 0U;
    }

    if (JY61P_IsDataTimeout(handle, now_ms) != 0U) {
        return 0U;
    }

    if ((check_acc != 0U) && (handle->data.acc_valid == 0U)) {
        return 0U;
    }

    if ((check_gyro != 0U) && (handle->data.gyro_valid == 0U)) {
        return 0U;
    }

    if ((check_angle != 0U) && (handle->data.angle_valid == 0U)) {
        return 0U;
    }

    return 1U;
}

uint8_t JY61P_IsControlReady(const JY61P_Handle_t *handle, uint32_t now_ms)
{
    if (handle == 0) {
        return 0U;
    }

    if (JY61P_IsDataTimeout(handle, now_ms) != 0U) {
        return 0U;
    }

    return handle->data.control_ready;
}

uint32_t JY61P_GetDataTimestamp(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0U;
    }

    return handle->data.timestamp_ms;
}

uint32_t JY61P_GetDataAge(const JY61P_Handle_t *handle, uint32_t now_ms)
{
    if (handle == 0) {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)(now_ms - handle->last_update_time);
}

void JY61P_SetDataFilter(JY61P_Handle_t *handle, uint8_t mask)
{
    if (handle != 0) {
        handle->data_filter_mask = (uint8_t)(mask & JY61P_FILTER_ALL);
    }
}

uint8_t JY61P_IsInitialized(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0U;
    }

    return handle->initialized;
}

uint8_t JY61P_IsGyroCalibrated(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0U;
    }

    return handle->gyro_calibrated;
}

uint8_t JY61P_GetGyroCalibrationProgress(const JY61P_Handle_t *handle)
{
    if (handle == 0) {
        return 0U;
    }

    if (handle->gyro_calibrated != 0U) {
        return 100U;
    }

#if (JY61P_ENABLE_GYRO_CALIBRATION == 0U)
    return 100U;
#else
    return (uint8_t)((handle->gyro_calibration_samples * 100U) /
                     JY61P_GYRO_CALIB_SAMPLES);
#endif
}
