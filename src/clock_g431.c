
#include "clock_g431.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx.h"
#include "main.h"

void ClockInit(void)
{
    /* Configure Data cache, Flash prefetch,  Instruction cache */
#ifdef INSTRUCTION_CACHE_DISABLE
    LL_FLASH_DisableInstCache();
#endif /* INSTRUCTION_CACHE_DISABLE */

#ifdef DATA_CACHE_DISABLE
    LL_FLASH_DisableDataCache();
#endif /* DATA_CACHE_DISABLE */

#ifdef PREFETCH_ENABLE
    LL_FLASH_EnablePrefetch();
#endif /* PREFETCH_ENABLE */


    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* Configure voltage*/
    LL_PWR_SetRegulVoltageScaling(POWER_VOLTAGE);

#if defined USE_HSE
    LL_RCC_HSE_Enable();
    while(!LL_RCC_HSE_IsReady ())
    {
        // Wait HSE
    }
#else
#error clock not defined!
#endif

#ifdef USE_PLL
#if defined (USE_HSE)
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, PLLM_DIV, PLLN_MUL, PLLR_DIV);

#if defined (PLLP_DIV)
    LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, PLLM_DIV, PLLN_MUL, PLLP_DIV);
#endif

#if defined (PLLQ_DIV)
    LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, PLLM_DIV, PLLN_MUL, PLLQ_DIV);
#endif

#else
#error clock not defined!
#endif
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while(!LL_RCC_PLL_IsReady())
    {
        // Wait PLL
    }
    if (FLASH_LATENCY > LL_FLASH_GetLatency())
    {
        LL_FLASH_SetLatency(FLASH_LATENCY);
        while (LL_FLASH_GetLatency() != FLASH_LATENCY)
        {
            // Wait latency
        }
    }
    /*------------------------- SYSCLK Configuration ---------------------------*/

    if(SYSCLK_FREQ > 80000000U)
    {
        LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
        // Wait ClkSource
    }
    #endif //USE_PLL
    /*-------------------------- HCLK Configuration --------------------------*/

    LL_RCC_SetAHBPrescaler(AHBCLKDIV);
    if (FLASH_LATENCY < LL_FLASH_GetLatency())
    {
        LL_FLASH_SetLatency(FLASH_LATENCY);
        while (LL_FLASH_GetLatency() != FLASH_LATENCY)
        {
            // Wait latency
        }
    }
    /*-------------------------- PCLK1 Configuration ---------------------------*/
    LL_RCC_SetAPB1Prescaler(APB1CLKDIV);

    /*-------------------------- PCLK2 Configuration ---------------------------*/
    LL_RCC_SetAPB2Prescaler(APB2CLKDIV);
}
