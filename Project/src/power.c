#include "power.h"


void power3PinInit(void)
{
     GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER3, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_POWER3_PORT, &GPIO_InitStructure);    

  POWER_33V_OFF(); 
 
}

void power331On(void)
{

#if defined (NEW_VERSION)

#else
     GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER3, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIO_POWER3_PORT, &GPIO_InitStructure);    


   POWER_33V_ON();
   
 #endif

}

__IO uint8_t	qxPowerStatus	=	POWER_STATUS_OFF;
void qxPower33On(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);   

   GPIO_SetBits(GPIOC,GPIO_Pin_12);
   //POWER_33V_ON();
	qxPowerStatus	=	POWER_STATUS_ON;
}

void qxPower33Off(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	qxPowerStatus	=	POWER_STATUS_OFF;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);  
  /* LEDs pins configuration */
  //GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);   

  GPIO_ResetBits(GPIOC,GPIO_Pin_12);
   //POWER_33V_ON();
}
void power331Off(void)
{

#if defined (NEW_VERSION)

#else
  GPIO_InitTypeDef GPIO_InitStructure;
  POWER_33V_OFF(); 

  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER3, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIO_POWER3_PORT, &GPIO_InitStructure);    
#endif

}

void power332On(void)
{
     GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER3, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_POWER3_PORT, &GPIO_InitStructure);    


 
  POWER_33V_ON();

}

void power332Off(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  POWER_33V_OFF(); 

  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER3, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER3_ALLPIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIO_POWER3_PORT, &GPIO_InitStructure);    

}

void power5vON(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER5, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER5V_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  GPIO_Init(GPIO_POWER5V_PORT, &GPIO_InitStructure);    

  GPIO_SetBits(GPIO_POWER5V_PORT,GPIO_POWER5V_PIN);
}
void power5vOff(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
 GPIO_ResetBits(GPIO_POWER5V_PORT,GPIO_POWER5V_PIN);
  /* Enable GPIOB and AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWER5, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_POWER5V_PIN;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AN; //¿ªÂ©Êä³ö
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_POWER5V_PORT, &GPIO_InitStructure);    

}
