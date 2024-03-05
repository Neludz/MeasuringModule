#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_ll_conf.h"
#include <stdint.h>
#include <stdbool.h>

#define UART_DELAY_US                   80
#define UART_DELAY_MS                   1

#define ADC_DIDT_DELAY_MS               1
#define ADC_MID_DELAY_MS                200
#define ADC_VOLTAGE_ITERATION_DELAY_US  200


// test value
#define R1_1        100000
#define R2_1        3300
#define R1_2        15000
#define R2_2        3300
#define R1_3        7500
#define R2_3        5100
#define R480        480000
#define R200        200
#define MV_REF      3000
#define MV_REF_HALF 1500
#define D_REF       4096
#define D_REF_HALF       (D_REF/2)

#define MIN_DISCRETE_TO_FILL_USER_VALUE     100
#define MAX_DISCRETE_TO_FILL_USER_VALUE     3950
#define MIN_FILL_USER_VALUE_V_SHUNT         -15000
#define MAX_FILL_USER_VALUE_V_SHUNT         15000
/*
Ushx = ((Dx-Dref/2)*Vref*10)/((R1*Dref)/R2) = 1 [x10 mV]
*/

#define MV_REFX   MV_REF*10
#define ADC1_1_K   ((R1_1*D_REF)/R2_1)
#define ADC1_2_K   ((R1_2*D_REF)/R2_2)

/*

*/
#define D_REF_V     4096
#define ADC2_K1     ((MV_REF*(R1_3+R2_3))/(2*R1_3))
#define ADC2_K2     ((R2_3*MV_REF)/R1_3)
#define ADC2_K_HI   ((R480+R200)/R200)


__STATIC_INLINE int32_t U_shunt_ADC1_2 (uint32_t discrete)
{
    return (long)((((int32_t)discrete-D_REF_HALF)*MV_REFX))/ADC1_2_K;
}

__STATIC_INLINE int32_t U_shunt_didt_ADC1_2 (uint32_t discrete)
{
    return (long)(((int32_t)discrete*MV_REFX))/ADC1_2_K;
}


__STATIC_INLINE int32_t U_hi_ADC2_1 (uint32_t discrete)
{
    return  (long)(((ADC2_K1-(((int32_t)discrete*ADC2_K2)/D_REF_V)-MV_REF_HALF) * ADC2_K_HI)/10000);
}
#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
