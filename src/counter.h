#ifndef COUNTER_H_INCLUDED
#define COUNTER_H_INCLUDED

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define SYSTIMER_TICK           100000LL
#define SYSTIMER_MS_TO_TICK(x)  ((SYSTIMER_TICK * x) / 1000LL)
#define SYSTIMER_US_TO_TICK(x)  ((SYSTIMER_TICK * x) / 1000000LL)

uint32_t MainTimerGetTick();
bool Timer_Is_Expired (const uint32_t Timer);
uint32_t Main_Timer_Set(const uint32_t AddTime);
void MainTimerTickInit();
#endif /* COUNTER_H_INCLUDED */
