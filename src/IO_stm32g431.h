#ifndef IO_STM32G431_H_INCLUDED
#define IO_STM32G431_H_INCLUDED
#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"

//----------------------------------------------
// ALL IN ONE REG,  USE -> (OUT | PP | SP_VHI | NO_PULL)
// GPIOx_MODER -> ((X >> 0) & 0x03)
#define GET_GPIOx_MODER(x)  (x & 0x03)
#define IN          (0x00)
#define OUT         (0x01)
#define ALT         (0x02)
#define AI          (0x03)
// GPIOx_OTYPER -> ((X >> 2) & 0x01)
#define GET_GPIOx_OTYPER(x)  ((x >> 2) & 0x01)
#define PP          (0x00)
#define OD          (0x04)
//GPIOx_OSPEEDR -> ((X >> 3) & 0x03)
#define GET_GPIOx_OSPEEDR(x)  ((x >> 3) & 0x03)
#define SP_LOW      (0x00)
#define SP_MED      (0x08)
#define SP_HI       (0x10)
#define SP_VHI      (0x18)
//GPIOx_PUPDR -> ((X >> 5) & 0x03)
#define GET_GPIOx_PUPDR(x)  ((x >> 5) & 0x03)
#define NO_PULL     (0x00)
#define PULL_UP     (0x20)
#define PULL_DOWN   (0x40)
//----------------------------------------------

// GPIOx_AFRL
#define AF_0   (0x00)
#define AF_1   (0x01)
#define AF_2   (0x02)
#define AF_3   (0x03)
#define AF_4   (0x04)
#define AF_5   (0x05)
#define AF_6   (0x06)
#define AF_7   (0x07)
#define AF_8   (0x08)
#define AF_9   (0x09)
#define AF_10  (0x010)
#define AF_11  (0x011)
#define AF_12  (0x012)
#define AF_13  (0x013)
#define AF_14  (0x014)
#define AF_15  (0x015)



typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint8_t Mode;
    uint8_t AF;
    uint8_t DefState;
    uint8_t ActiveState;
} tGPIO_Line;

typedef enum
{
    OFF = 0,
    ON = 1,
    LOW = 0,
    HIGH =1,
} tIOState;

#define ADC1_CHANNELS_COUNT                 1
#define	ADC1_CH4_CURRENT_LOW_CHANNEL        4
#define	ADC1_CH3_CURRENT_HIGH_CHANNEL       3
#define ADC1_NUMBER_LIST                    {ADC1_CH3_CURRENT_HIGH_CHANNEL}
#define ADC1_1_CHANNEL_POSITION             0

#define ADC1_SAMPLE_TIME                    LL_ADC_SAMPLINGTIME_2CYCLES_5//LL_ADC_SAMPLINGTIME_2CYCLES_5//LL_ADC_SAMPLINGTIME_47CYCLES_5

#define ADC2_CHANNELS_COUNT                 1
#define	ADC2_CH3_VOLTAGE_CHANNEL            3
#define ADC2_NUMBER_LIST                    {ADC2_CH3_VOLTAGE_CHANNEL}
#define ADC2_SAMPLE_TIME                    LL_ADC_SAMPLINGTIME_2CYCLES_5//LL_ADC_SAMPLINGTIME_47CYCLES_5
#define ADC2_VOLTAGE_CHANNEL_POSITION       0

#define OPTIC_BAUDRATE              2000000


//          NAME            GPIOx   GPIO_Pin    MODE        AF      DefState    ActiveState
#define IO_TABLE\
    X_IO(io_uart_tx,        GPIOB,  3,      (ALT | PP | SP_VHI | NO_PULL),  AF_7,   0,  LOW)	\
    X_IO(io_adc_volt,       GPIOA,  6,      (AI | NO_PULL),	0,		0,  	LOW)	\
    X_IO(io_adc_cur_low,    GPIOA,  3,      (AI | NO_PULL),	0,		0,  	LOW)	\
    X_IO(io_adc_cur_high,   GPIOA,  2,      (AI | NO_PULL),	0,		0,  	LOW)	\

typedef enum
{
#define X_IO(a,b,c,d,e,f,g)	a,
    IO_TABLE
#undef X_IO
    NUM_IO		//count
} tIOLine;


void IO_Init(void);
bool IO_GetLineActive(tIOLine Line);
void IO_UARTC_Init();
void IO_SetLine(tIOLine Line, bool State);
bool IO_GetLine(tIOLine Line);
uint16_t IO_getADC_1_val(int nch);
uint16_t IO_getADC_2_val(int nch);
#endif /* IO_STM32G431_H_INCLUDED */
