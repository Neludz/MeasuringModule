#ifndef CLOCK_G431_H_INCLUDED
#define CLOCK_G431_H_INCLUDED


#define USE_HSE
//#define USE_HSE_BYPASS
//#define HSE_VALUE             8000000U
//#define USE_HSI
//#define USE_HSI_4
#define USE_PLL
#define SYSCLK_FREQ             140000000U


//#define  INSTRUCTION_CACHE_DISABLE    //default on
//#define  DATA_CACHE_DISABLE           //default on
//#define  PREFETCH_ENABLE              //default off

//USE_HSE->/PLLM->xPLLN->/PLLR->CLK
//                    |->/PLLP->ADC

#define PLLM_DIV            LL_RCC_PLLM_DIV_1
#define PLLN_MUL            35
#define PLLR_DIV            LL_RCC_PLLR_DIV_2
#define PLLP_DIV            LL_RCC_PLLP_DIV_3//12
//#define PLLQ_DIV            LL_RCC_PLLQ_DIV_2

#define FLASH_LATENCY       LL_FLASH_LATENCY_4
#define AHBCLKDIV           LL_RCC_SYSCLK_DIV_1
#define APB1CLKDIV          LL_RCC_APB1_DIV_1
#define APB2CLKDIV          LL_RCC_APB2_DIV_1
// LL_PWR_REGU_VOLTAGE_SCALE1
// LL_PWR_REGU_VOLTAGE_SCALE2
#define POWER_VOLTAGE   LL_PWR_REGU_VOLTAGE_SCALE1

void ClockInit(void);


#endif /* CLOCK_G431_H_INCLUDED */
