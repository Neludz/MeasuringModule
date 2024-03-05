#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"
#include "counter.h"
#include "clock_g431.h"

volatile uint32_t Tick = 0;

void SysTick_Handler()
{
    Tick++;
}
//-------------------------------------------------------------------------
void MainTimerTickInit()
{
    SysTick_Config(SYSCLK_FREQ/SYSTIMER_TICK);
}

//-------------------------------------------------------------------------
uint32_t MainTimerGetTick()
{
    return Tick;
}

//-------------------------------------------------------------------------
bool Timer_Is_Expired (const uint32_t Timer)
{
    uint32_t TimeTick;
    TimeTick = Tick;
    return ((TimeTick - Timer) < (1UL << 31));
}
//-------------------------------------------------------------------------
uint32_t Main_Timer_Set(const uint32_t AddTime)
{
    uint32_t TimeTick;
    TimeTick = Tick;
    return TimeTick + AddTime;
}
