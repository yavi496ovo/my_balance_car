#include "vofa_app.h"

#include <stdint.h>
#include "motor_app.h"
#include "scheduler.h"
#include "usart.h"

#define VOFA_APP_CHANNEL_COUNT           (12U)
#define VOFA_APP_FRAME_TAIL_0            (0x00U)
#define VOFA_APP_FRAME_TAIL_1            (0x00U)
#define VOFA_APP_FRAME_TAIL_2            (0x80U)
#define VOFA_APP_FRAME_TAIL_3            (0x7FU)

typedef struct {
    float channel[VOFA_APP_CHANNEL_COUNT];
    uint8_t tail[4];
} vofa_app_packet_t;

static vofa_app_packet_t vofa_app_tx_packet;
static uint8_t vofa_app_tx_busy;

static void vofa_app_fill_packet(const motor_app_debug_t *debug)
{
    vofa_app_tx_packet.channel[0] = debug->angle_deg;
    vofa_app_tx_packet.channel[1] = debug->target_angle_deg;
    vofa_app_tx_packet.channel[2] = debug->speed_pwm;
    vofa_app_tx_packet.channel[3] = debug->gyro_dps;
    vofa_app_tx_packet.channel[4] = debug->balance_pwm;
    vofa_app_tx_packet.channel[5] = debug->turn_pwm;
    vofa_app_tx_packet.channel[6] = debug->left_pwm;
    vofa_app_tx_packet.channel[7] = debug->right_pwm;
    vofa_app_tx_packet.channel[8] = debug->speed_target;
    vofa_app_tx_packet.channel[9] = debug->speed_feedback;
    vofa_app_tx_packet.channel[10] = (float)debug->running;
    vofa_app_tx_packet.channel[11] = (float)debug->fault;

    vofa_app_tx_packet.tail[0] = VOFA_APP_FRAME_TAIL_0;
    vofa_app_tx_packet.tail[1] = VOFA_APP_FRAME_TAIL_1;
    vofa_app_tx_packet.tail[2] = VOFA_APP_FRAME_TAIL_2;
    vofa_app_tx_packet.tail[3] = VOFA_APP_FRAME_TAIL_3;
}

void vofa_app_task(void)
{
    static uint32_t last_send_ms = 0U;
    uint32_t now_ms;
    motor_app_debug_t debug;
    HAL_StatusTypeDef status;

    if (vofa_app_tx_busy != 0U) {
        return;
    }

    now_ms = scheduler_get_tick();
    if ((uint32_t)(now_ms - last_send_ms) < VOFA_APP_SEND_PERIOD_MS) {
        return;
    }
    last_send_ms = now_ms;

    motor_app_get_debug(&debug);
    vofa_app_fill_packet(&debug);

    vofa_app_tx_busy = 1U;
    status = HAL_UART_Transmit_IT(&huart1,
                                  (uint8_t *)&vofa_app_tx_packet,
                                  (uint16_t)sizeof(vofa_app_tx_packet));
    if (status != HAL_OK) {
        vofa_app_tx_busy = 0U;
    }
}

void vofa_app_uart_tx_cplt_callback(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART1)) {
        return;
    }

    vofa_app_tx_busy = 0U;
}

void vofa_app_uart_error_callback(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART1)) {
        return;
    }

    vofa_app_tx_busy = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    vofa_app_uart_tx_cplt_callback(huart);
}
