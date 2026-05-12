#include "led.h"
#include "key.h"
#include "time.h"
#include "sensor.h"
#include "deviceset.h"
#include "rtc.h"

#define KEY_DEBUG     0

__IO uint8_t keyPressTimes = 0x00;
__IO uint8_t keyStatus = KEY_UP_STATE;

__IO uint16_t keyCounter = 0x00;

#define CHANG_MCU_STATE_TIME        60
#define LED_BINK_TIME               33

/*


  */
void key_Config(void)
{

/* Private typedef -----------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



void keyInterrupt(void)
{
	if(keyStatus == KEY_UP_STATE)
	{
		if(batStatus > BAT_LOWER_VALULE) //BAT_LOWER_VALULE_201125 BAT_ALARM_VALULE
		{
			keyStatus = KEY_DOWN_STATE;
			keyCounter = 0x0;
			//开定时器中断
			Time4_Init_Status(TIME_START);
			//点亮LED灯
			led_Turn_On(LED_ALL);
			rtcWakeUpFlig |= KEY_WAKE_UP_FLAG;
		}
	}
}

void keyTimeIniterrupt(void)
{
 //  uint16_t eepromStatus = 0xff;

  if(keyStatus == KEY_DOWN_STATE)
	{
	   if(READ_KEY_STATUS())
	   {
			 if(keyCounter == 0)keyPressTimes++;			 
			keyCounter ++;
	   	if(keyCounter < LED_BINK_TIME)
		  {
				if(keyCounter%2==0)
				{
					led_Turn_Off(LED_ALL);  
				}
				else
				{
				 led_Turn_On(LED_ALL);  
				}
		  }
			
	   if(CHANG_MCU_STATE_TIME ==keyCounter)
	   {
			 rstStatus	=	RST_STATUS_KEY;				
			 SaveData_SWReset();	
//		 if(mcuPowerStatus == MCU_OFF_STATE)
//		 {
//#if (KEY_DEBUG>0)
//	   	printf("\n\r 设备激活，进入工作模式 \n\r");
//#endif
//		 	mcuPowerStatus = MCU_ON_STATE;
//		 }
//		 else
//		 {
//#if (KEY_DEBUG>0)
//	   	printf("\n\r 设备休眠，进入休眠模式 \n\r");
//#endif
////		 	mcuPowerStatus = MCU_OFF_STATE;
//		 
//		 }

	    		 led_Turn_Off(LED_ALL);
				 keyCounter = 0x0;
				//关闭定时器中断
		 Time4_Init_Status(TIME_END);
		 keyStatus = KEY_UP_STATE;

	   }
	   }
	   else
	   {
#if (KEY_DEBUG>0)
	   	printf("\n\r 设备正常\n\r");
#endif
	    	led_Turn_Off(LED_ALL);
				keyCounter = 0x0;
				//关闭定时器中断
				Time4_Init_Status(TIME_END);
				keyStatus = KEY_UP_STATE;

	   }
	    
	}
	else
	{
	    		 led_Turn_Off(LED_ALL);
//				 keyCounter = 0x0;
				//关闭定时器中断
		 Time4_Init_Status(TIME_END);	
	}

}


