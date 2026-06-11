#ifndef __UART_APP_H
#define __UART_APP_H

#include <stdint.h>
#include "ringbuffer.h"
#include "stm32f4xx_hal.h"

#ifndef UART_APP_RX_BUFFER_SIZE
#define UART_APP_RX_BUFFER_SIZE          (256U)
#endif

void uart_app_init(void);
void uart_app_task(void);
void uart_app_start_receive(void);
ringbuffer_t *uart_app_get_rx_ringbuffer(void);
uint32_t uart_app_get_rx_overflow_count(void);

void uart_app_rx_cplt_callback(UART_HandleTypeDef *huart);
void uart_app_error_callback(UART_HandleTypeDef *huart);

#endif
