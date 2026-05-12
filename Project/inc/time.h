#ifndef TIME_H
#define TIME_H

#include "stm32l1xx.h"
#include <stdint.h>

#define TIME_START   0x00
#define TIME_END     0x01

extern __IO uint32_t time9Counter;

extern void Time_Init_Status(uint8_t timeStatus);
extern void NVIC_Time_Status(uint8_t timeStatus);

extern void Time4_Init_Status(uint8_t timeStatus);
extern void NVIC_Time4_Status(uint8_t timeStatus);


extern void Time3_Init_Status(uint8_t timeStatus);
extern void NVIC_Time3_Status(uint8_t timeStatus);


extern void NVIC_Time5_Status(uint8_t timeStatus);
extern void Time5_Init_Status(uint8_t timeStatus);

extern void sofeWareWatchDog(void);
extern void NVIC_Time9_Status(uint8_t timeStatus);
extern void Time9_Init_Status(uint8_t timeStatus);
extern void sofeWareWatchBegin(void);
extern void sofeWareWatchEnd(void);
extern void sofeWareWatchClear(void);
#endif

