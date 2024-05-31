#include "clock_g431.h"
#include "IO_stm32g431.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

//------------------------------- prototype -------------------------------------
static void IO_ConfigLine(tGPIO_Line io);
//--------------X macros---------------------------------------------------------
const tGPIO_Line IOs[NUM_IO] =
{
#define X_IO(a,b,c,d,e,f,g)	{b,c,d,e,f,g},
    IO_TABLE
#undef X_IO
};
//--------------------------------- variable -------------------------------------
const uint8_t IO_ADC1_number[ADC1_CHANNELS_COUNT] = ADC1_NUMBER_LIST;
uint16_t adc1_data[ADC1_CHANNELS_COUNT*9];
const uint8_t IO_ADC2_number[ADC2_CHANNELS_COUNT] = ADC2_NUMBER_LIST;
uint16_t adc2_data[ADC2_CHANNELS_COUNT*9];
//---------------------------------------------------------------------------------
void IO_SetLine(tIOLine Line, bool State)
{
    if (State)
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);
    else
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);
}
//---------------------------------------------------------------------------------
bool IO_GetLine(tIOLine Line)
{
    if (Line < NUM_IO)
        return (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) != 0);
    else
        return false;
}
//---------------------------------------------------------------------------------
bool IO_GetLineActive(tIOLine Line)
{
    if (Line < NUM_IO)
    {
        bool pin_set = (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) ? true : false);
        return (pin_set == ( IOs[Line].ActiveState ? true : false));
    }
    else
        return false;
}
//---------------------------------------------------------------------------------
void IO_SetLineActive(tIOLine Line, bool State)
{
    if (State ^ IOs[Line].ActiveState)
    {
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);   //reset
    }
    else
    {
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);  //set
    }
}
//---------------------------------------------------------------------------------
static void IO_ConfigLine(tGPIO_Line io)
{
    io.GPIOx->MODER &= ~(0x03 << (io.GPIO_Pin * 2));
    io.GPIOx->MODER |= (GET_GPIOx_MODER(io.Mode) << (io.GPIO_Pin * 2));

    io.GPIOx->OTYPER &= ~(0x01 << io.GPIO_Pin);
    io.GPIOx->OTYPER |= (GET_GPIOx_OTYPER(io.Mode) << io.GPIO_Pin);

    io.GPIOx->OSPEEDR &= ~(0x03 << (io.GPIO_Pin * 2));
    io.GPIOx->OSPEEDR |= (GET_GPIOx_OSPEEDR(io.Mode) << (io.GPIO_Pin * 2));

    io.GPIOx->PUPDR &= ~(0x03 << (io.GPIO_Pin * 2));
    io.GPIOx->PUPDR |= (GET_GPIOx_PUPDR(io.Mode) << (io.GPIO_Pin * 2));

    if(io.GPIO_Pin < 8)
    {
        io.GPIOx->AFR[0] &=  ~(0x0F << (io.GPIO_Pin * 4));
        io.GPIOx->AFR[0] |=  io.AF << (io.GPIO_Pin * 4);
    }
    else
    {
        io.GPIOx->AFR[1] &=  ~(0x0F << ((io.GPIO_Pin - 8) * 4));
        io.GPIOx->AFR[1] |=  io.AF << ((io.GPIO_Pin - 8) * 4);
    }

    io.GPIOx->ODR &= ~(1 << io.GPIO_Pin);
    io.GPIOx->ODR |= (io.DefState << io.GPIO_Pin);
}
//---------------------------------------------------------------------------------

void IO_Init()
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);

// Set all pins
    for (int Line = 0; Line < NUM_IO; Line++)
    {
        IO_ConfigLine(IOs[Line]);
    }
}

void IO_ADC_Init(void)
{
    uint32_t wait_loop_index = 0UL;
    int32_t i;
    LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_PLL); //LL_RCC_ADC12_CLKSOURCE_SYSCLK
    LL_RCC_PLL_EnableDomain_ADC();
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

    LL_ADC_SetCommonClock(ADC12_COMMON, LL_ADC_CLOCK_ASYNC_DIV1);
    SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_ANASWVDD);
    // ---------------- ADC1 ----------------
    LL_ADC_DisableDeepPowerDown(ADC1);
    LL_ADC_EnableInternalRegulator(ADC1);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * ((SYSCLK_FREQ / (100000UL * 2UL)) + 1UL));
    while (wait_loop_index != 0UL)
    {
        wait_loop_index--;
    }
    if (LL_ADC_IsInternalRegulatorEnabled(ADC1) == 0)
    {
        while (1){}
    }
    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS);
    LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED);//LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetSequencerLength(ADC1, (ADC1_CHANNELS_COUNT-1));
    for(i = 0; i < ADC1_CHANNELS_COUNT; i++)
    {
        //SQR
        if (i<4)
            MODIFY_REG(ADC1->SQR1, 0x1F<<(i*6+6), IO_ADC1_number[i]<<(i*6+6));
        else if(i<9)
            MODIFY_REG(ADC1->SQR2, 0x1F<<((i-4)*6), IO_ADC1_number[i]<<((i-4)*6));
        else if(i<14)
            MODIFY_REG(ADC1->SQR3, 0x1F<<((i-9)*6), IO_ADC1_number[i]<<((i-9)*6));
        //SMPR
        if (IO_ADC1_number[i] < 9)
            MODIFY_REG(ADC1->SMPR1, 0x07<<(IO_ADC1_number[i]*3), ADC1_SAMPLE_TIME<<(IO_ADC1_number[i]*3));
        else if(IO_ADC1_number[i] < 18)
            MODIFY_REG(ADC1->SMPR2, 0x07<<((IO_ADC1_number[i]-9)*3), ADC1_SAMPLE_TIME<<(IO_ADC1_number[i]*3));
    }
    //calibration
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1)) {}
    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1))  {}
   // LL_ADC_REG_StartConversion(ADC1);
    // ---------------- ADC2 ----------------
    LL_ADC_DisableDeepPowerDown(ADC2);
    LL_ADC_EnableInternalRegulator(ADC2);
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * ((SYSCLK_FREQ / (100000UL * 2UL)) + 1UL));
    while (wait_loop_index != 0UL)
    {
        wait_loop_index--;
    }
    if (LL_ADC_IsInternalRegulatorEnabled(ADC2) == 0)
    {
        while (1) {}
    }
    LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_CONTINUOUS);
    LL_ADC_REG_SetOverrun(ADC2, LL_ADC_REG_OVR_DATA_OVERWRITTEN);
    LL_ADC_SetDataAlignment(ADC2, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetResolution(ADC2, LL_ADC_RESOLUTION_12B);
    LL_ADC_REG_SetDMATransfer(ADC2, LL_ADC_REG_DMA_TRANSFER_LIMITED);
    LL_ADC_REG_SetSequencerLength(ADC2, (ADC2_CHANNELS_COUNT-1));
    for(i = 0; i < ADC2_CHANNELS_COUNT; i++)
    {
        //SQR
        if (i<4)
            MODIFY_REG(ADC2->SQR1, 0x1F<<(i*6+6), IO_ADC2_number[i]<<(i*6+6));
        else if(i<9)
            MODIFY_REG(ADC2->SQR2, 0x1F<<((i-4)*6), IO_ADC2_number[i]<<((i-4)*6));
        else if(i<14)
            MODIFY_REG(ADC2->SQR3, 0x1F<<((i-9)*6), IO_ADC2_number[i]<<((i-9)*6));
        //SMPR
        if (IO_ADC2_number[i] < 9)
            MODIFY_REG(ADC2->SMPR1, 0x07<<(IO_ADC2_number[i]*3), ADC2_SAMPLE_TIME<<(IO_ADC2_number[i]*3));
        else if(IO_ADC2_number[i] < 18)
            MODIFY_REG(ADC2->SMPR2, 0x07<<((IO_ADC2_number[i]-9)*3), ADC2_SAMPLE_TIME<<(IO_ADC2_number[i]*3));
    }
    //calibration
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC2))  {}
    LL_ADC_Enable(ADC2);
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC2))   {}
    //LL_ADC_REG_StartConversion(ADC2);
}

void IO_UART_Init(void)
{
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    //LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_8_8);
    //LL_USART_EnableFIFO(USART2);

    LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
    LL_USART_SetParity(USART2,LL_USART_PARITY_NONE);
    LL_USART_EnableDirectionTx(USART2);
    LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_16);
    LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
    LL_USART_SetPrescaler(USART2, LL_USART_PRESCALER_DIV1);

    LL_USART_SetBaudRate(USART2, SYSCLK_FREQ, LL_USART_PRESCALER_DIV1,
                         LL_USART_OVERSAMPLING_16,
                         OPTIC_BAUDRATE);
    LL_USART_EnableDMAReq_TX(USART2);
    //LL_USART_EnableIT_TC(USART2);
    LL_USART_Enable(USART2);
    //NVIC_SetPriority(USART2_IRQn, 1);
    //NVIC_EnableIRQ(USART2_IRQn);
}

void IO_DMA_Init(uint8_t *uart_buf_p)
{
     // ---------------- DMA1 ----------------
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    // ---------------- DMA2 ----------------
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    // ---------------- ADC1 ----------------
    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, (LL_DMA_DIRECTION_PERIPH_TO_MEMORY |\
                          LL_DMA_PRIORITY_VERYHIGH|\
                          LL_DMA_MODE_CIRCULAR |\
                          LL_DMA_MEMORY_INCREMENT |\
                          LL_DMA_PERIPH_NOINCREMENT |\
                          LL_DMA_MDATAALIGN_HALFWORD |\
                          LL_DMA_PDATAALIGN_HALFWORD ));


    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC1);

    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)(&(ADC1->DR)),
                           (uint32_t)adc1_data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, ADC1_CHANNELS_COUNT*9);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); /* (1) */
    NVIC_SetPriority(DMA1_Channel1_IRQn,1); /* (2) */
    // ---------------- ADC2 ----------------
    LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_1, (LL_DMA_DIRECTION_PERIPH_TO_MEMORY |\
                          LL_DMA_PRIORITY_VERYHIGH|\
                          LL_DMA_MODE_CIRCULAR |\
                          LL_DMA_MEMORY_INCREMENT |\
                          LL_DMA_PERIPH_NOINCREMENT |\
                          LL_DMA_MDATAALIGN_HALFWORD |\
                          LL_DMA_PDATAALIGN_HALFWORD ));


    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC2);

    LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_1, (uint32_t)(&(ADC2->DR)),
                           (uint32_t)adc2_data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, ADC2_CHANNELS_COUNT*9);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);
    LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);
    NVIC_EnableIRQ(DMA2_Channel1_IRQn); /* (1) */
    NVIC_SetPriority(DMA2_Channel1_IRQn,1); /* (2) */
   // ---------------- USART2 ----------------
    LL_DMA_ConfigTransfer(DMA2, LL_DMA_CHANNEL_2, (LL_DMA_DIRECTION_MEMORY_TO_PERIPH |\
                          LL_DMA_PRIORITY_HIGH|\
                          LL_DMA_MODE_NORMAL |\
                          LL_DMA_MEMORY_INCREMENT |\
                          LL_DMA_PERIPH_NOINCREMENT |\
                          LL_DMA_MDATAALIGN_BYTE |\
                          LL_DMA_PDATAALIGN_BYTE ));


    LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART2_TX);

 LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_2, (uint32_t)uart_buf_p,(uint32_t)(&(USART2->TDR)),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, 9);
    LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_2);
    LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_2);
    //LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
    NVIC_EnableIRQ(DMA2_Channel2_IRQn); /* (1) */
    NVIC_SetPriority(DMA2_Channel2_IRQn,1); /* (2) */

}
//---------------------------------------------------------------------------------
/**
 * @brief IO_getADCval - calculate median value for `nch` channel
 * @param nch - number of channel
 * @return
 */
uint16_t IO_getADC_1_val(int nch)
{
    int i, addr = nch;
#define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }
#define PIX_SWAP(a,b) {register uint16_t temp=(a);(a)=(b);(b)=temp; }
    uint16_t p[9];
    for(i = 0; i < 9; ++i, addr += ADC1_CHANNELS_COUNT) // first we should prepare array for optmed
        p[i] = adc1_data[addr];
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[1]) ;
    PIX_SORT(p[3], p[4]) ;
    PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[3]) ;
    PIX_SORT(p[5], p[8]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[3], p[6]) ;
    PIX_SORT(p[1], p[4]) ;
    PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[4], p[2]) ;
    PIX_SORT(p[6], p[4]) ;
    PIX_SORT(p[4], p[2]) ;
    return p[4];
#undef PIX_SORT
#undef PIX_SWAP
}

uint16_t IO_getADC_2_val(int nch)
{
    int i, addr = nch;
#define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }
#define PIX_SWAP(a,b) {register uint16_t temp=(a);(a)=(b);(b)=temp; }
    uint16_t p[9];
    for(i = 0; i < 9; ++i, addr += ADC2_CHANNELS_COUNT) // first we should prepare array for optmed
        p[i] = adc2_data[addr];
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[1]) ;
    PIX_SORT(p[3], p[4]) ;
    PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[3]) ;
    PIX_SORT(p[5], p[8]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[3], p[6]) ;
    PIX_SORT(p[1], p[4]) ;
    PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[4], p[2]) ;
    PIX_SORT(p[6], p[4]) ;
    PIX_SORT(p[4], p[2]) ;
    return p[4];
#undef PIX_SORT
#undef PIX_SWAP
}


