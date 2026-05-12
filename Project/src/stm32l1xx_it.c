#include "stm32l1xx_it.h"
#include "main.h"
#include "deviceset.h"
#include "key.h"
#include "sensor.h"
#include "gprs.h"
#include "stm32_mems_adapter.h"
#include "rtc.h"
#include "usart.h"
#if(USART_USE_CTR>0)
#include "rs485.h"
#endif
#if((SK60_CTR>0)||(KXYL_CTR>0))
#include "sk60.h"
#endif

#include "FreeRTOS.h"
#include "semphr.h"

void NMI_Handler(void)
{
    rstStatus = RST_STATUS_NMI;
    __asm volatile ("BKPT #0");  /* Trigger debugger if attached */
    NVIC_SystemReset();
}

void HardFault_Handler(void)
{
    rstStatus = RST_STATUS_Hard;
    __asm volatile ("BKPT #0");  /* Trigger debugger if attached */
    NVIC_SystemReset();
}

void MemManage_Handler(void)
{
    rstStatus = RST_STATUS_Mem;
    __asm volatile ("BKPT #0");  /* Trigger debugger if attached */
    NVIC_SystemReset();
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

/* SVC_Handler, PendSV_Handler, SysTick_Handler:
 * FreeRTOS provides its own implementations mapped via FreeRTOSConfig.h.
 * The default weak aliases in startup file allow override. */

void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        keyInterrupt();
    }
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI3_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line3) != RESET) {}
    EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        in2ActInterrupt();
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xADXL345Semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
#ifdef SCHYDROLOGY
        gsmIntDeal();
#endif
    }
    EXTI->PR = EXTI_Line12;
}

void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {
        cal_Num();
    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        cal_Num();
    }
    EXTI->PR = EXTI_Line7|EXTI_Line6;
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        cal_work();
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM11_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM11, TIM_IT_Update) != RESET)
    {
        gprsTimeDeal();
    }
    TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
}

void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
#if(KXYL_CTR>0)
        kxylTimeDeal();
#endif
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        keyTimeIniterrupt();
    }
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

void TIM9_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM9, TIM_IT_Update);
        sofeWareWatchDog();
    }
}

void USART1_IRQHandler(void)
{
#if((USART_USE_CTR>0)&&(rs485_USART_INIT==rs485_USART_1)&&(RS485_USART_DMA_CTR>0))
    if(rs485UsartStatus == 1)
    {
        if (USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET)
        {
            rs485Idle();
        }
    }
#endif
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
#if((SK60_CTR>0)||(KXYL_CTR>0))
#if(SK60_USART_INIT==SK60_USART_1)
        if(sk60UsartStatus>0){sk60Receive();}
        else
#endif
#endif
        {gprsReceive();}
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART1);
        usart1OverrunCount++;
    }
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {}
}

void USART2_IRQHandler(void)
{
#if((USART_USE_CTR>0)&&(rs485_USART_INIT==rs485_USART_2)&&(RS485_USART_DMA_CTR>0))
    if(rs485UsartStatus == 1)
    {
        if (USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET)
        {
            rs485Idle();
        }
    }
#endif
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
#if((SK60_CTR>0)||(KXYL_CTR>0))
#if(SK60_USART_INIT==SK60_USART_2)
        if(sk60UsartStatus>0){sk60Receive();}
#endif
#endif
    }
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART2);
        usart2OverrunCount++;
    }
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {}
}

void USART3_IRQHandler(void)
{
#if((USART_USE_CTR>0)&&(rs485_USART_INIT==rs485_USART_3)&&(RS485_USART_DMA_CTR>0))
    if(rs485UsartStatus == 1)
    {
        if (USART_GetITStatus(RS485_USART, USART_IT_IDLE) != RESET)
        {
            rs485Idle();
        }
    }
#endif
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
#if((SK60_CTR>0)||(KXYL_CTR>0))
#if(SK60_USART_INIT==SK60_USART_3)
        if(sk60UsartStatus>0){sk60Receive();}
#endif
#endif
    }
    if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        (void)USART_ReceiveData(USART3);
        usart3OverrunCount++;
    }
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {}
}

void RTC_WKUP_IRQHandler(void)
{
    if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
    {
        rtcWakeUpFlig |= RTC_WAKE_UP_FLAG;
    }
    EXTI_ClearITPendingBit(EXTI_Line20);
    RTC_ClearITPendingBit(RTC_IT_WUT);
}
