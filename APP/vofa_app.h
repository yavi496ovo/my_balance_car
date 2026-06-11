#ifndef __VOFA_APP_H
#define __VOFA_APP_H

#include "stm32f4xx_hal.h"

#ifndef VOFA_APP_TASK_PERIOD_MS
#define VOFA_APP_TASK_PERIOD_MS          (1U)
#endif

#ifndef VOFA_APP_SEND_PERIOD_MS
#define VOFA_APP_SEND_PERIOD_MS          (20U)
#endif

void vofa_app_task(void);
void vofa_app_uart_tx_cplt_callback(UART_HandleTypeDef *huart);
void vofa_app_uart_error_callback(UART_HandleTypeDef *huart);

#endif
