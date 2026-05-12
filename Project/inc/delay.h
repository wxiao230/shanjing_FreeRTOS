#ifndef __DELAY_H
#define __DELAY_H

#include "stm32l1xx.h"
#include <stdint.h>

extern void delay_ms(__IO uint32_t nTime);
extern void delay_1us(__IO uint32_t nTime);
extern void delaySchedulerReady(void);
extern uint8_t isSchedulerRunning(void);

#endif
