#include "main.h"
#include <stdio.h>
#include <string.h>
#include "clock_g431.h"
#include "IO_stm32g431.h"
#include "math.h"
#include "counter.h"
//-------------------------------------------------------------------------
#ifdef DEBUG_TARGET
uint32_t print_delay = 0;
#endif
uint32_t uart_busy = 0, uart_delay = 0;
int32_t discrete_adc1_2_filter=0, discrete_adc2_1_volt_filt=0;
int32_t Vshunt_adc1_2_mid=0, Vshunt_adc1_2_didt, volt_adc2_1_mid;
uint32_t adc_cur_mid_delay = 0, adc_didt_delay, adc_volt_mid_delay;
uint32_t adc_iteration_count = 0, adc_measuring_cur_iteration_count, adc_measuring_volt_iteration_count, count_measuring;
uint32_t  adc_1_busy = 0, adc_2_busy = 0;
uint8_t usart2_data[UART_BUF_SIZE];

const unsigned char crc_array[256] =
{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
//-------------------------------------------------------------------------
void DMA1_Channel1_IRQHandler()
{
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        adc_1_busy = 0;
        adc_iteration_count++;
    }
    if (LL_DMA_IsActiveFlag_TE1(DMA1))
    {
        LL_DMA_ClearFlag_TE1(DMA1);
    }
}
//-------------------------------------------------------------------------
void DMA2_Channel2_IRQHandler()
{
    if (LL_DMA_IsActiveFlag_TC2(DMA2)  )
    {
        LL_DMA_ClearFlag_TC2(DMA2);
        LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_2);
        uart_busy = 0;
    }
    if(LL_DMA_IsActiveFlag_TE2(DMA2))
    {
        LL_DMA_ClearFlag_TE2(DMA2);
        LL_DMA_DisableChannel(DMA2, LL_DMA_CHANNEL_2);
        uart_busy = 0;
    }
}
//-------------------------------------------------------------------------
void DMA2_Channel1_IRQHandler()
{
    if (LL_DMA_IsActiveFlag_TC1(DMA2))
    {
        LL_DMA_ClearFlag_TC1(DMA2);
        adc_2_busy = 0;
    }
    if (LL_DMA_IsActiveFlag_TE1(DMA2))
    {
        LL_DMA_ClearFlag_TE1(DMA2);
    }
}
//-------------------------------------------------------------------------
static unsigned char dallas_crc8(const unsigned int size, uint8_t *buf)
{
    unsigned char crc = 0;
    for ( unsigned int i = 0; i < size; ++i )
    {
        crc = crc_array[(0xFF & buf[i]) ^ crc];
    }
    return crc;
}
//-------------------------------------------------------------------------
void uart_processing(void)
{
    static uint32_t uart_state = 0;
    uint8_t crc;
    int32_t v_shunt, v_high;
    switch (uart_state)
    {
    case 0:
        if ( Timer_Is_Expired(uart_delay))
            uart_state = 1; //fall
        else
            break;
    case 1:
        uart_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(UART_DELAY_US));
        uart_state = 0;
        if(uart_busy == 0)
        {
            v_shunt = (int16_t)U_shunt_ADC1_2(discrete_adc1_2_filter);

            usart2_data[0]= Vshunt_adc1_2_mid & 0xFF;
            usart2_data[1]= (Vshunt_adc1_2_mid>>8) & 0xFF;
            usart2_data[2]= v_shunt & 0xFF;
            usart2_data[3]= (v_shunt>>8) & 0xFF;
            usart2_data[4]= Vshunt_adc1_2_didt & 0xFF;
            usart2_data[5]= (Vshunt_adc1_2_didt>>8) & 0xFF;

            v_high = U_hi_ADC2_1(discrete_adc2_1_volt_filt);
            // test_value = (Vshunt_adc1_2_didt * 200) /750;
            usart2_data[6]= v_high & 0xFF;
            usart2_data[7]= (v_high>>8) & 0xFF;
            usart2_data[8]= volt_adc2_1_mid & 0xFF;
            usart2_data[9]= (volt_adc2_1_mid>>8) & 0xFF;
            // CRC
            crc = dallas_crc8(10, usart2_data);
            usart2_data[10] = crc;
            uart_busy = 1;

            LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_2, 11);
            LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_2);
        }
        break;

    default:
        break;
    }
}
//-------------------------------------------------------------------------
void adc_process(void)
{
    static uint32_t adc_1_state = 0;
    static int32_t adc_intermediate = 0, adc_intermediate_v=0, discrete_adc1_2_mid=0, discrete_adc2_1_mid=0;
    static int32_t adc_sum_mid_cur = 0, adc_sum_mid_volt;
    static int32_t discrete_adc1_2_didt_prev=0;
    int32_t adc_im=0,  adc_im_v=0, delta_i;
    //adc_1
    switch (adc_1_state)
    {
    case 0:
        if(!adc_1_busy)
        {
            adc_1_busy = 1;
            adc_1_state = 1;  //fall
        }
        else
            break;
    case 1:
        adc_im = IO_getADC_1_val(ADC1_1_CHANNEL_POSITION);
        adc_intermediate += (adc_im - discrete_adc1_2_filter);
        discrete_adc1_2_filter = (adc_intermediate*ADC_FILTER_RATIO_A)>>8;
        adc_sum_mid_cur += discrete_adc1_2_filter;
        adc_measuring_cur_iteration_count++;

        if (Timer_Is_Expired(adc_didt_delay))
        {
            //adc_didt_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(ADC_DIDT_DELAY_US));
            if ((discrete_adc1_2_filter > (D_REF_HALF+D_REF_DIDT_DISCRET_HYSTERESIS) && discrete_adc1_2_didt_prev > (D_REF_HALF+D_REF_DIDT_DISCRET_HYSTERESIS)) || \
                    (discrete_adc1_2_filter < (D_REF_HALF-D_REF_DIDT_DISCRET_HYSTERESIS) && discrete_adc1_2_didt_prev < (D_REF_HALF-D_REF_DIDT_DISCRET_HYSTERESIS)))
            {
                delta_i = discrete_adc1_2_filter - discrete_adc1_2_didt_prev;
                adc_didt_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(ADC_DIDT_DELAY_US));
            }
            else
            {
                delta_i = 0;
                adc_didt_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(50));
            }

            if((discrete_adc1_2_filter > (D_REF_HALF+D_REF_DIDT_DISCRET_HYSTERESIS) && delta_i > 0)|| \
                    (discrete_adc1_2_filter < (D_REF_HALF-10) && delta_i < 0))
            {
                Vshunt_adc1_2_didt = U_shunt_didt_ADC1_2(delta_i);
            }
            else
            {
                Vshunt_adc1_2_didt = 0;
            }
            discrete_adc1_2_didt_prev = discrete_adc1_2_filter;
        }
        if (Timer_Is_Expired(adc_cur_mid_delay))
        {
            adc_1_state = 2; //fall
        }
        else
        {
            adc_1_state = 0;
            LL_ADC_REG_StartConversion(ADC1);
            break;
        }
    case 2:
        discrete_adc1_2_mid = adc_sum_mid_cur / adc_measuring_cur_iteration_count;
        Vshunt_adc1_2_mid = (int16_t) U_shunt_ADC1_2(discrete_adc1_2_mid);
        adc_sum_mid_cur = 0;
        count_measuring = adc_measuring_cur_iteration_count;
        adc_measuring_cur_iteration_count = 0;
        adc_cur_mid_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(ADC_MID_CURRENT_DELAY_US));
        adc_1_state = 0;
        LL_ADC_REG_StartConversion(ADC1);
        break;

    default :
        break;
    }
    //adc_2
    if(!adc_2_busy)
    {
        adc_im_v =  IO_getADC_2_val(ADC2_VOLTAGE_CHANNEL_POSITION);
        adc_intermediate_v += (adc_im_v - discrete_adc2_1_volt_filt);
        discrete_adc2_1_volt_filt = (adc_intermediate_v * ADC_FILTER_RATIO_V)>>8;
        adc_measuring_volt_iteration_count++;
        adc_sum_mid_volt += discrete_adc2_1_volt_filt;
        if (Timer_Is_Expired(adc_volt_mid_delay))
        {
            discrete_adc2_1_mid = adc_sum_mid_volt / adc_measuring_volt_iteration_count;
            volt_adc2_1_mid = (int16_t)U_hi_ADC2_1(discrete_adc2_1_mid);
            adc_sum_mid_volt = 0;
            adc_measuring_volt_iteration_count = 0;
            LL_IWDG_ReloadCounter(IWDG);
            adc_volt_mid_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(ADC_MID_VOLTAGE_DELAY_US));
        }
        adc_2_busy = 1;
        LL_ADC_REG_StartConversion(ADC2);
    }
}
//-------------------------------------------------------------------------
void print_process(void)
{
    //static uint32_t count_1=0, count_2=0;
    // uint32_t adc1_value[ADC1_CHANNELS_COUNT];
    // int32_t ADC_Val;
    if (Timer_Is_Expired(print_delay))
    {
//        count_2 = adc_iteration_count - count_1;
//        count_1 = adc_iteration_count;
        print_delay = Main_Timer_Set(SYSTIMER_MS_TO_TICK(1000));
        printf("test \n");

    }
}
//-------------------------------------------------------------------------
void iwdg_init(uint8_t x0_1s_time)
{
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);

    LL_IWDG_SetReloadCounter(IWDG, (125 * x0_1s_time)/10);
    while(!LL_IWDG_IsReady(IWDG))
    {
        // Wait reary flag
    }
    LL_IWDG_SetWindow(IWDG, 0x0FFF);
    LL_IWDG_ReloadCounter(IWDG);
}

//-------------------------------------------------------------------------
void flash_btock(void)
{
    uint8_t rdp_level = READ_BIT(FLASH->OPTR, FLASH_OPTR_RDP_Msk);
    if (rdp_level != RDP_VAL)
    {
        if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
        {
            /* Authorize the FLASH Registers access */
            WRITE_REG(FLASH->KEYR, FLASH_KEY1);
            WRITE_REG(FLASH->KEYR, FLASH_KEY2);
        }
        if (READ_BIT(FLASH->CR, FLASH_CR_OPTLOCK) != 0U)
        {
            /* Authorizes the Option Byte register programming */
            WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY1);
            WRITE_REG(FLASH->OPTKEYR, FLASH_OPTKEY2);
        }
        uint32_t tmp1;
        tmp1 = (uint32_t)(FLASH->OPTR & (~(FLASH_OPTR_RDP_Msk|FLASH_OPTR_BOR_LEV_Msk)));
        /* Calculate the option byte to write */
        tmp1 |= (uint32_t)((RDP_VAL) | (BOR_LEVEL << FLASH_OPTR_BOR_LEV_Pos));
        /* Wait for last operation to be completed */
        while((FLASH->SR) & (FLASH_SR_BSY));
        /* program read protection level */
        FLASH->OPTR = tmp1;
        /* Wait for last operation to be completed */
        while((FLASH->SR) & (FLASH_SR_BSY));
        WRITE_REG(FLASH->CR, FLASH_CR_OPTSTRT);
        while((FLASH->SR) & (FLASH_SR_BSY));
        SET_BIT(FLASH->CR, FLASH_CR_LOCK);
        while((FLASH->SR) & (FLASH_SR_BSY));
        SET_BIT(FLASH->CR, FLASH_CR_OPTLOCK);

    }
}

//-------------------------------------------------------------------------
int main(void)
{
    flash_btock();
    ClockInit();
    adc_cur_mid_delay = Main_Timer_Set(SYSTIMER_US_TO_TICK(ADC_MID_CURRENT_DELAY_US));
    iwdg_init(IWDG_TIME_X_0_1S);
    IO_Init();
    IO_ADC_Init();
    IO_UART_Init();
    IO_DMA_Init(usart2_data);
    MainTimerTickInit();
#ifdef DEBUG_TARGET
    printf ( "[ INFO ] Program start now\n" );
#endif
    uart_delay = Main_Timer_Set(SYSTIMER_MS_TO_TICK(100));
    while(1)
    {
#ifdef DEBUG_TARGET
        print_process();
#endif
        adc_process();
        uart_processing();
    }
}
