#include "uart_app.h"

#include "usart.h"
#include "vofa_app.h"

#define UART_APP_JY61P_AUTO_CONFIG      (1U)       /* 1=上电自动把 JY61P 恢复为 115200，0=关闭 */
#define UART_APP_JY61P_OLD_BAUD         (9600U)    /* 模块异常掉回的常见波特率 */
#define UART_APP_JY61P_TARGET_BAUD      (115200U)  /* 小车工程正常使用的 JY61P 波特率 */
#define UART_APP_JY61P_CMD_DELAY_MS     (180U)     /* JY61P 每条配置命令之间的等待时间 */
#define UART_APP_JY61P_REBOOT_MS        (1200U)    /* 保存/重启后等待模块重新输出数据 */
#define UART_APP_JY61P_TX_TIMEOUT_MS    (20U)

static uint8_t uart_app_rx_buffer[UART_APP_RX_BUFFER_SIZE];
static ringbuffer_t uart_app_rx_ringbuffer;
static uint8_t uart_app_rx_byte;
static uint32_t uart_app_rx_overflow_count;

#if (UART_APP_JY61P_AUTO_CONFIG != 0U)
static void uart_app_set_usart1_baud(uint32_t baud)
{
    huart1.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void uart_app_send_jy61p_bytes(const uint8_t *bytes, uint16_t len)
{
    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)bytes,
                            len,
                            UART_APP_JY61P_TX_TIMEOUT_MS);
    HAL_Delay(UART_APP_JY61P_CMD_DELAY_MS);
}

static void uart_app_send_jy61p_115200_config(void)
{
    static const uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};       /* 解锁配置 */
    static const uint8_t output[] = {0xFF, 0xAA, 0x02, 0x0E, 0x00};       /* 输出 ACC/GYRO/ANGLE */
    static const uint8_t rate_100hz[] = {0xFF, 0xAA, 0x03, 0x09, 0x00};   /* 回传频率 100Hz */
    static const uint8_t baud_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};  /* 波特率 115200 */
    static const uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};         /* 保存到模块内部，掉电不丢 */
    static const uint8_t reboot[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};       /* 重启模块，加载新配置 */

    uart_app_send_jy61p_bytes(unlock, (uint16_t)sizeof(unlock));
    uart_app_send_jy61p_bytes(output, (uint16_t)sizeof(output));
    uart_app_send_jy61p_bytes(rate_100hz, (uint16_t)sizeof(rate_100hz));
    uart_app_send_jy61p_bytes(baud_115200, (uint16_t)sizeof(baud_115200));
    uart_app_send_jy61p_bytes(save, (uint16_t)sizeof(save));
    uart_app_send_jy61p_bytes(reboot, (uint16_t)sizeof(reboot));
}

static void uart_app_restore_jy61p_115200(void)
{
    /* 先按 9600 发一次：处理 JY61P 异常掉回出厂波特率的情况。 */
    uart_app_set_usart1_baud(UART_APP_JY61P_OLD_BAUD);
    uart_app_send_jy61p_115200_config();
    HAL_Delay(UART_APP_JY61P_REBOOT_MS);

    /* 再按 115200 发一次：处理模块已经是目标波特率，或刚刚切换成功的情况。 */
    uart_app_set_usart1_baud(UART_APP_JY61P_TARGET_BAUD);
    uart_app_send_jy61p_115200_config();
    HAL_Delay(UART_APP_JY61P_REBOOT_MS);

    uart_app_set_usart1_baud(UART_APP_JY61P_TARGET_BAUD);
}
#endif

void uart_app_init(void)
{
    MX_USART1_UART_Init();
#if (UART_APP_JY61P_AUTO_CONFIG != 0U)
    uart_app_restore_jy61p_115200();
#endif
    ringbuffer_init(&uart_app_rx_ringbuffer,
                    uart_app_rx_buffer,
                    (uint16_t)UART_APP_RX_BUFFER_SIZE);
    uart_app_rx_overflow_count = 0U;
    uart_app_start_receive();
}

void uart_app_task(void)
{
}

void uart_app_start_receive(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &uart_app_rx_byte, 1U);
}

ringbuffer_t *uart_app_get_rx_ringbuffer(void)
{
    return &uart_app_rx_ringbuffer;
}

uint32_t uart_app_get_rx_overflow_count(void)
{
    return uart_app_rx_overflow_count;
}

void uart_app_rx_cplt_callback(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART1)) {
        return;
    }

    if (ringbuffer_write(&uart_app_rx_ringbuffer, uart_app_rx_byte) == 0U) {
        uart_app_rx_overflow_count++;
    }

    uart_app_start_receive();
}

void uart_app_error_callback(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != USART1)) {
        return;
    }

    vofa_app_uart_error_callback(huart);
    uart_app_start_receive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_app_rx_cplt_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_app_error_callback(huart);
}
