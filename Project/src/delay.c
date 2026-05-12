#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t fac_us = 32;
static uint8_t scheduler_running = 0;

void delay_ms(__IO uint32_t nTime)
{
    if(scheduler_running)
    {
        vTaskDelay(pdMS_TO_TICKS(nTime));
    }
    else
    {
        __IO uint32_t i;
        while(nTime--)
        {
            for(i = 0; i < (fac_us * 1000); i++)
            {
                __NOP();
            }
        }
    }
}

void delay_1us(__IO uint32_t nTime)
{
    if(nTime > 100 && isSchedulerRunning()) {
        vTaskDelay(pdMS_TO_TICKS((nTime + 999) / 1000));
        return;
    }
    __IO uint32_t i;
    for(i = 0; i < (fac_us * nTime); i++)
    {
        __NOP();
    }
}

void delaySchedulerReady(void)
{
    scheduler_running = 1;
}

uint8_t isSchedulerRunning(void)
{
    return scheduler_running;
}
