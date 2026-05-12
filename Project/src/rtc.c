#include "rtc.h"
//#include "Printf.h"
#include <stdio.h>
#include "key.h"
#include "ds1302.h"
#include "stm32_mems_adapter.h"
#if(SK60_CTR>0)
	#include "sk60.h"
#endif
/*

*/

#define ONE_DAY_TIME()             (24*60*60)
#define JUDGE_TIEM()                (30*60)
#define ADJ_TIEM()                  (1*60) // (15*60)





__IO uint8_t rtcWakeUpFlig =   WAKE_UP_FLAG_IDLE;

static void RTC_TimeRegulate(void);


/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */



void RTC_Config(void)
{
 //  NVIC_InitTypeDef  NVIC_InitStructure;
//  EXTI_InitTypeDef  EXTI_InitStructure;
 
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_RTCAccessCmd(ENABLE);
    

/* LSE used as RTC source clock */
  /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
//    SynchPrediv = 8191;
//    AsynchPrediv = 127;
//     SynchPrediv = 0xFF;

  
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

//

//    RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
    /* EXTI configuration *******************************************************/
 // RTC_SetWakeUpCounter(0x1FFF);

//  EXTI_ClearITPendingBit(EXTI_Line20);
//  EXTI_InitStructure.EXTI_Line = EXTI_Line20;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
 // EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Wakeup Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 // NVIC_Init(&NVIC_InitStructure);


  /* Enable the Wakeup Interrupt */
//  RTC_ITConfig(RTC_IT_WUT, ENABLE); 

}




void rtcConfigurationJudge(void)
{
  /* 以下if...else.... if判断系统时间是否已经设置，判断RTC后备寄存器1的值
     是否为事先写入的0XA5A5，如果不是，则说明RTC是第一次上电，需要配置RTC，
     提示用户通过串口更改系统时间，把实际时间转化为RTC计数值写入RTC寄存器,
     并修改后备寄存器1的值为0XA5A5。
     else表示已经设置了系统时间，打印上次系统复位的原因
  */
	RTC_InitTypeDef RTC_InitStructure;
	uint32_t AsynchPrediv = 0, SynchPrediv = 0;
	/* RTC configuration  */
	RTC_Config();
	SynchPrediv = ((SLEEP_BASE_TIME*256)-1);
	AsynchPrediv = (uint32_t)0x7F;
	/* Configure the RTC data register and RTC prescaler */
	RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
	RTC_InitStructure.RTC_SynchPrediv = SynchPrediv;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;

	/* Check on RTC init */
	if (RTC_Init(&RTC_InitStructure) == ERROR)
	{
	//     printf("\n\r        /!\\***** RTC Prescaler Config failed ********/!\\ \n\r");
	}

	/* Configure the time register */
	RTC_TimeRegulate(); 
}


void rtc_NVIC_Configuration(void)
{
 
   NVIC_InitTypeDef  NVIC_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;
  /* RTC Alarm A Interrupt Configuration */
  /* EXTI configuration *********************************************************/
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Alarm Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Display the current time on the Hyperterminal.
  * @param  None
  * @retval None
  */
  
  



  				
void Time_Show(void)
{
  RTC_DateTypeDef RTC_DateStructure;
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
  printf("\n\r The mcu rtc is : 20%0.2d-%0.2d-%0.2d ", RTC_DateStructure.RTC_Year, RTC_DateStructure.RTC_Month, RTC_DateStructure.RTC_Date);
  /* Get the current Time */
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
  printf(" %0.2d:%0.2d:%0.2d \n\r", RTC_TimeStructure.RTC_Hours, RTC_TimeStructure.RTC_Minutes, RTC_TimeStructure.RTC_Seconds);
}


/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval None
  */
static void RTC_TimeRegulate(void)
{
//  uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

//  printf("\n\r==============Time Settings=====================================\n\r");
  RTC_TimeTypeDef RTC_TimeStructure;
  RTC_DateTypeDef RTC_DateStructure;

  RTC_TimeStructure.RTC_H12     = RTC_H12_PM;

  RTC_TimeStructure.RTC_Hours 	= 15;
  RTC_TimeStructure.RTC_Minutes = 14;
  RTC_TimeStructure.RTC_Seconds = 0;

  RTC_DateStructure.RTC_WeekDay = 7 ; 
  RTC_DateStructure.RTC_Month 	= 8; 
  RTC_DateStructure.RTC_Date 		= 22;  
  RTC_DateStructure.RTC_Year 		= 21;  

  if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
  {
//       printf("\n\r>> !! RTC Set data failed. !! <<\n\r");
  } 
  else
  {
//  	 printf("\n\r>> !! RTC Set date success. !! <<\n\r");
  }   
  
    RTC_WaitForSynchro();  
    
  /* Configure the RTC time register */
  if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
  {
//    printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
  } 
  else
  {
//    printf("\n\r>> !! RTC Set Time success. !! <<\n\r");
//     Time_Show();
    /* Indicator for the RTC configuration */
    RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
  }
 }


void clockConfig(void)
{
    /* After wake-up from STOP reconfigure the system clock */
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {}
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x0C)
    {}

}

void wakeUpIniterrupt(void)
{
 // NVIC_InitTypeDef  NVIC_InitStructure;
//  EXTI_InitTypeDef  EXTI_InitStructure;
  /* Enable PWR and BKP clocks */
  /* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Allow access to BKP Domain */
  /*使能RTC和后备寄存器访问 */
//  PWR_BackupAccessCmd(ENABLE);
   PWR_RTCAccessCmd(ENABLE);

       /* Enable Wakeup Counter */
    RTC_WakeUpCmd(DISABLE);


    /* RTC Wakeup Interrupt Generation: Clock Source: RTCDiv_16, Wakeup Time Base: 4s */
   RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
//  RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16);
    /* EXTI configuration *******************************************************/
//    RTC_SetWakeUpCounter(3500);
//	RTC_SetWakeUpCounter(875);
//    RTC_SetWakeUpCounter(1750);
	 RTC_SetWakeUpCounter(3600);



  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);  


}


uint32_t getCurrterTime(void)
{
	uint32_t currterTime = 0;
	readDs1302Time(&rtcTime);	
	currterTime = (queryDayToHour(rtcTime.year,rtcTime.month,rtcTime.date)+rtcTime.hour)*3600+rtcTime.min*60+rtcTime.sec;
	//currterTime = rtcTime.hour*3600+rtcTime.min*60+rtcTime.sec;//rtcTime.date*86400+ 
	curSecondTime	=	currterTime;
	return 	currterTime;
}

uint32_t getSleepTimeAndAdj(uint32_t startTime,uint32_t period)
{
	uint32_t sleepTime = 0x00;
	uint32_t currterTime = 0;
	uint32_t tempTime = 0x00;
	currterTime = getCurrterTime();
	if(currterTime >= startTime)
	{
		tempTime = currterTime - startTime; 
	}
	else
	{
		tempTime = 0; 
	}
	if(period > tempTime)
	{
		sleepTime = period - tempTime;
	}
	else
	{
		sleepTime = 0;
	}

	return 	sleepTime; 
}

uint32_t castTimeAdj(uint32_t startTime)
{
	uint32_t castTime  = 0;
	uint32_t curTime = 0;

	curTime = getCurrterTime();
	if(curTime>=startTime)
	{
		castTime = curTime - startTime;
	}
	else
	{
		castTime =  0;
	}
	return castTime;
}
void clockConfig32M(void)
{
    /* After wake-up from STOP reconfigure the system clock */
    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {}
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
    
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x0C)
    {}

}
void wakeUPNvic(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;
  EXTI_InitTypeDef  EXTI_InitStructure;
  EXTI_ClearITPendingBit(EXTI_Line20);
  EXTI_InitStructure.EXTI_Line = EXTI_Line20;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable the RTC Wakeup Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
void lowPowerEnter(uint32_t sleepTime)
{
	/* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
//	RTC_ClearITPendingBit(RTC_IT_WUT);
//	EXTI_ClearITPendingBit(EXTI_Line20);
	/* Allow access to BKP Domain */
	/*使能RTC和后备寄存器访问 */
	//  PWR_BackupAccessCmd(ENABLE);
	PWR_RTCAccessCmd(ENABLE);

	/* Enable Wakeup Counter */
	RTC_WakeUpCmd(DISABLE);
	if(sleepTime>129600)sleepTime=129600;	//最大休眠时间  36小时左右 129600=36*60*60
	
	/* RTC Wakeup Interrupt Generation: Clock Source: RTCDiv_16, Wakeup Time Base: 4s */
	if(sleepTime<=64800)//64800=18*60*60	
	{	
		if(sleepTime==0)sleepTime=1;
		RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	}
	else
	{
		sleepTime&=0x0FFFF;
		RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_17bits);
	}
	
	RTC_SetWakeUpCounter(sleepTime);
	
	/* Enable the Wakeup Interrupt */
	EXTI_ClearITPendingBit(EXTI_Line20); 
	RTC_ClearITPendingBit(RTC_IT_WUT);
	wakeUPNvic();
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	keyPressTimes = 0;
	if((keyStatus==KEY_UP_STATE)&&(adxlStatus !=ADXL345_INI2))
	{
		RTC_WakeUpCmd(ENABLE);
		PWR_UltraLowPowerCmd(ENABLE);
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);  
	}
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_RTCAccessCmd(ENABLE);
	RTC_WakeUpCmd(DISABLE);
	RTC_ITConfig(RTC_IT_WUT, DISABLE);
	RTC_WaitForSynchro(); 
	clockConfig32M();
}

void wakeUpTimeSet(uint32_t sleepTime)
{
  /* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Allow access to BKP Domain */
  /*使能RTC和后备寄存器访问 */
//  PWR_BackupAccessCmd(ENABLE);
   PWR_RTCAccessCmd(ENABLE);

       /* Enable Wakeup Counter */
    RTC_WakeUpCmd(DISABLE);


    /* RTC Wakeup Interrupt Generation: Clock Source: RTCDiv_16, Wakeup Time Base: 4s */
   RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

	 RTC_SetWakeUpCounter(sleepTime);



  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);  

}


void rtcTimeSet(uint8_t *buf)
{
  RTC_TimeTypeDef RTC_Set_TimeStructure;
  RTC_DateTypeDef RTC_Set_DateStructure;
  

   RTC_Set_TimeStructure.RTC_Seconds	= buf[0];

   RTC_Set_TimeStructure.RTC_Minutes	=  buf[1];
   RTC_Set_TimeStructure.RTC_Hours		=  buf[2];
   if(buf[2]>11)
   {
     RTC_Set_TimeStructure.RTC_H12		= RTC_H12_PM;

   }
   else
   {
      RTC_Set_TimeStructure.RTC_H12		= RTC_H12_AM;
  
   }
   RTC_Set_DateStructure.RTC_Date  		=  buf[3];
   RTC_Set_DateStructure.RTC_WeekDay 	=  buf[4];

   RTC_Set_DateStructure.RTC_Month   	=  buf[5];
   RTC_Set_DateStructure.RTC_Year  		=  buf[6];
//    GregorianDay(&devTime);  
//   debugTime1 =  mktimev(&devTime);
  /* Enable PWR and BKP clocks */
  /* PWR时钟（电源控制）与BKP时钟（RTC后备寄存器）使能 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
 // RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Allow access to BKP Domain */
  /*使能RTC和后备寄存器访问 */
//  PWR_BackupAccessCmd(ENABLE);
   PWR_RTCAccessCmd(ENABLE);
  if(RTC_SetDate(RTC_Format_BIN, &RTC_Set_DateStructure) == ERROR)
  {
//       printf("\n\r>> !! RTC Set data failed. !! <<\n\r");
  } 
  else
  {
//  	 printf("\n\r>> !! RTC Set date success. !! <<\n\r");
  }   
  
    RTC_WaitForSynchro();  
    
  /* Configure the RTC time register */
  if(RTC_SetTime(RTC_Format_BIN, &RTC_Set_TimeStructure) == ERROR)
  {
//    printf("\n\r>> !! RTC Set Time failed. !! <<\n\r");
  } 
  else
  {
//    printf("\n\r>> !! RTC Set Time success. !! <<\n\r");
        RTC_WaitForSynchro();  
//	 Time_Show();

  }

 

}
