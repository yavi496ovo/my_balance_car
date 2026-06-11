#include "scheduler.h"

#include "encoder_app.h"
#include "jy61p_app.h"
#include "motor_app.h"
#include "stm32f4xx_hal.h"
#include "uart_app.h"
#include "vofa_app.h"

typedef struct {
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

static task_t scheduler_task[] = {
    {uart_app_task, 1U, 0U},
    {jy61p_app_task, JY61P_APP_TASK_PERIOD_MS, 0U},
    {encoder_app_task, ENCODER_APP_TASK_PERIOD_MS, 0U},
    {motor_app_task, MOTOR_APP_TASK_PERIOD_MS, 0U},
    {vofa_app_task, VOFA_APP_TASK_PERIOD_MS, 0U}
};

static uint8_t scheduler_task_num;

void scheduler_init(void)
{
    uint8_t i;
    uint32_t now_ms;

    scheduler_task_num = (uint8_t)(sizeof(scheduler_task) / sizeof(task_t));
    now_ms = scheduler_get_tick();

    for (i = 0U; i < scheduler_task_num; i++) {
        scheduler_task[i].last_run = now_ms;
    }
}

void scheduler_run(void)
{
    uint8_t i;
    uint32_t now_ms;

    now_ms = scheduler_get_tick();
    for (i = 0U; i < scheduler_task_num; i++) {
        if ((uint32_t)(now_ms - scheduler_task[i].last_run) >=
            scheduler_task[i].rate_ms) {
            scheduler_task[i].last_run = now_ms;
            scheduler_task[i].task_func();
        }
    }
}

uint32_t scheduler_get_tick(void)
{
    return HAL_GetTick();
}
