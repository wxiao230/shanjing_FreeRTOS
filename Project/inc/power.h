#ifndef POWER_H
#define POWER_H

#include "stm32l1xx.h"



/*****************************************************************************************************
*                           3.3v power control
*
*******************************************************************************************************/
#define POWER_STATUS_OFF	0x00
#define POWER_STATUS_ON		0x01
#define RCC_GPIO_POWER3                     RCC_AHBPeriph_GPIOB        
#define GPIO_POWER3_PORT                    GPIOB

#if defined (NEW_VERSION)
#define GPIO_POWER3_PIN                     GPIO_Pin_12
#else
#define GPIO_POWER3_PIN                     GPIO_Pin_15
#endif


#define GPIO_POWER31_PIN                     GPIO_Pin_0

#define GPIO_POWER32_PIN                     GPIO_Pin_12

#define GPIO_POWER3_ALLPIN                   GPIO_POWER3_PIN         

#define POWER_33V_ON()    GPIO_SetBits(GPIO_POWER3_PORT,GPIO_POWER3_PIN)
#define POWER_33V_OFF()   GPIO_ResetBits(GPIO_POWER3_PORT,GPIO_POWER3_PIN)

#define POWER_33V1_ON()    GPIO_ResetBits(GPIO_POWER3_PORT,GPIO_POWER31_PIN)
#define POWER_33V1_OFF()   GPIO_SetBits(GPIO_POWER3_PORT,GPIO_POWER31_PIN)

#define POWER_33V2_ON()    GPIO_ResetBits(GPIO_POWER3_PORT,GPIO_POWER32_PIN)
#define POWER_33V2_OFF()   GPIO_SetBits(GPIO_POWER3_PORT,GPIO_POWER32_PIN)



/*****************************************************************************************************
*                           3.3v power control
*
*******************************************************************************************************/
#define RCC_GPIO_POWER5                     RCC_AHBPeriph_GPIOA        
#define GPIO_POWER5V_PORT                    GPIOA

#if defined (NEW_VERSION)
#define GPIO_POWER5V_PIN                     GPIO_Pin_15
#else
#define GPIO_POWER5V_PIN                     GPIO_Pin_12
#endif




#define POWER_5V_ON()    GPIO_SetBits(GPIO_POWER5V_PORT,GPIO_POWER5V_PIN)
#define POWER_5V_OFF()   GPIO_ResetBits(GPIO_POWER5V_PORT,GPIO_POWER5V_PIN)
extern __IO uint8_t	qxPowerStatus;
extern void power3PinInit(void);
extern void power5vON(void);
extern void power5vOff(void);

extern void power332Off(void);
extern void power331Off(void);
extern void power332On(void);
extern void power331On(void);
extern void qxPower33Off(void);
extern void qxPower33On(void);
#endif

