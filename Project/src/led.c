/* Includes ------------------------------------------------------------------*/
#include "led.h"


#define LED_USE    1


#ifdef  LED_USE

#define RCC_GPIO_LED                                 RCC_AHBPeriph_GPIOA
#define GPIO_LED_PORT                                GPIOA    

#define WORK_LED    GPIO_Pin_1			 //working led
#define COMM_LED    GPIO_Pin_11	         //communications led

#define GPIO_LED_ALL     WORK_LED

#endif



static void Led_Turn_on_all(void);
static void Work_Led_Turn_on(void);
static void Comm_Led_Turn_on(void);

static void Led_Turn_off_all(void);
static void Work_Led_Turn_Off(void);
static void Comm_Led_Turn_Off(void);


 //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);




static void Led_Turn_on_all(void)
{
#ifdef 	LED_USE

  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_LED_ALL;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
	/* Turn On All LEDs */
	   GPIO_SetBits(GPIO_LED_PORT, GPIO_LED_ALL);
   
#endif
}
 
static void Work_Led_Turn_on(void)
{
#ifdef 	LED_USE
	/* Turn On LED1 */
	  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = WORK_LED;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_LED_PORT,WORK_LED);
   
#endif
}

static void Comm_Led_Turn_on(void)
{
#ifdef 	LED_USE
	/* Turn On LED1 */
	  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = COMM_LED;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
	GPIO_SetBits(GPIO_LED_PORT,COMM_LED);
  
#endif
}

static void Led_Turn_off_all(void)
{
#ifdef 	LED_USE
	  GPIO_InitTypeDef GPIO_InitStructure;
	/* Turn Off All LEDs */
 	 GPIO_ResetBits(GPIO_LED_PORT, GPIO_LED_ALL);

  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_LED_ALL;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);

#endif
}

static void Work_Led_Turn_Off(void)
{
#ifdef 	LED_USE
	  GPIO_InitTypeDef GPIO_InitStructure;
   	 GPIO_ResetBits(GPIO_LED_PORT, WORK_LED);

  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = WORK_LED;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
#endif
}

static void Comm_Led_Turn_Off(void)
{
#ifdef 	LED_USE
	  GPIO_InitTypeDef GPIO_InitStructure;
   	  GPIO_ResetBits(GPIO_LED_PORT, COMM_LED);


  /* Enable GPIOB, GPIOC and AFIO clock */
 // RCC_APB2PeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
   RCC_AHBPeriphClockCmd(RCC_GPIO_LED, ENABLE);  //RCC_APB2Periph_AFIO
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = COMM_LED;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_LED_PORT, &GPIO_InitStructure);
#endif
}


void led_Turn_On(uint8_t led)
{
#ifdef 	LED_USE
   switch(led)
   {
     case  LED_ALL :
	 {
	 	   Led_Turn_on_all();
		   break;
	 }
     case  LED_WORK :
	 {
	 	   Work_Led_Turn_on();
		   break;
	 }
     case  LED_COMM :
	 {
	 	  Comm_Led_Turn_on();
		  break;
	 }
	 default :
	 break;	    
   }
#endif
}

void led_Turn_Off(uint8_t led)
{
#ifdef 	LED_USE

   switch(led)
   {
     case  LED_ALL :
	 {
	 	   Led_Turn_off_all();
		   break;
	 }
     case  LED_WORK :
	 {
	 	   Work_Led_Turn_Off();
		   break;
	 }
     case  LED_COMM :
	 {
	 	  Comm_Led_Turn_Off();
		  break;
	 }
	 default :
	 break;	    
   }
#endif
}

