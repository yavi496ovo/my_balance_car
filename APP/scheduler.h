#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>

void scheduler_init(void);
void scheduler_run(void);
uint32_t scheduler_get_tick(void);

#endif
