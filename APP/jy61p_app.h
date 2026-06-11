#ifndef __JY61P_APP_H
#define __JY61P_APP_H

#include <stdint.h>
#include "jy61p.h"

#ifndef JY61P_APP_TASK_PERIOD_MS
#define JY61P_APP_TASK_PERIOD_MS         (2U)
#endif

void jy61p_app_init(void);
void jy61p_app_task(void);

const JY61P_Data_t *jy61p_app_get_data(void);
uint8_t jy61p_app_is_control_ready(void);
uint32_t jy61p_app_get_parsed_frame_count(void);
uint32_t jy61p_app_get_data_age(void);
void jy61p_app_start_gyro_calibration(void);

#endif
