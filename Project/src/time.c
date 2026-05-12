#include "time.h"
#include "main.h"
#include "usart.h"
#include "deviceset.h"
__IO uint32_t time9Counter = 0x00;

void Time_Init_Status(uint8_t timeStatus)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM2);
  /* TIM5 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* ---------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
  --------------------------------------------------------------- */

  /* Time base configuration */
  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Period = (10000 - 1);
  // 这个就是预分频系数，当由于为0时表示不分频所以要减1
    TIM_TimeBaseStructure.TIM_Prescaler = (3200 - 1);
 //    TIM_TimeBaseStructure.TIM_Prescaler = (800 - 1);

  // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
  // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //向上计数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器5
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  /* TIM IT enable */ //打开溢出中断
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM5 enable counter */
  

  if(timeStatus == TIME_START)
  {
     TIM_Cmd(TIM2, ENABLE);  //计数器使能，开始工作   
		 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  }
  else
  {			
			TIM_Cmd(TIM2,DISABLE);  //计数器使能，开始工作  
			TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);		
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
  }


}

void NVIC_Time_Status(uint8_t timeStatus)
{
    NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
//#if defined(USE_IAP_BOOT)
//         NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);
//#else
//		 NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
//#endif

  /* Enable the TIM5 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

  if(timeStatus == TIME_START)
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }
  NVIC_Init(&NVIC_InitStructure);
}


void Time3_Init_Status(uint8_t timeStatus)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM3);
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
  /* Time base configuration */
  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Period = (200 - 1);
  // 这个就是预分频系数，当由于为0时表示不分频所以要减1
  TIM_TimeBaseStructure.TIM_Prescaler = (3200 - 1);
 
  // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //向上计数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器3
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Clear TIM3 update pending flag[清除TIM5溢出中断标志] */
  TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

//  /* TIM IT enable */ //打开溢出中断
//  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//  /* TIM3 enable counter */
//  

//  if(timeStatus == TIME_START)
//  {
//     TIM_Cmd(TIM3, ENABLE);  //计数器使能，开始工作   
//  }
//  else
//  {
//    TIM_Cmd(TIM3,DISABLE);  //计数器使能，开始工作   
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
//  }
 /* TIMx enable counter */ 
  if(timeStatus == TIME_START)
  {   
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//打开溢出中断
		TIM_Cmd(TIM3, ENABLE);  //计数器使能，开始工作   
  }
  else
  {
		TIM_Cmd(TIM3,DISABLE);  //计数器使能，停止工作  
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);//关闭溢出中断		 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
  }	

}

void NVIC_Time3_Status(uint8_t timeStatus)
{
    NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  /* Enable the TIM5 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

  if(timeStatus == TIME_START)
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }
  NVIC_Init(&NVIC_InitStructure);
}

void Time4_Init_Status(uint8_t timeStatus)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM4);
  /* TIM5 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  /* ---------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
  --------------------------------------------------------------- */

  /* Time base configuration */
  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Period = (500 - 1);
  // 这个就是预分频系数，当由于为0时表示不分频所以要减1
  TIM_TimeBaseStructure.TIM_Prescaler = (3200 - 1);
  // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
  // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //向上计数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器5
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

  /* TIM IT enable */ //打开溢出中断
//  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  /* TIM5 enable counter */
  

  if(timeStatus == TIME_START)
  {
		 TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
     TIM_Cmd(TIM4, ENABLE);  //计数器使能，开始工作   
  }
  else
  {
    TIM_Cmd(TIM4,DISABLE);  //计数器使能，开始工作   
		 TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,DISABLE);
  }
}

void NVIC_Time4_Status(uint8_t timeStatus)
{
    NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  /* Enable the TIM5 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

  if(timeStatus == TIME_START)
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }
  NVIC_Init(&NVIC_InitStructure);
}


void Time5_Init_Status(uint8_t timeStatus)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM11);
  /* TIM5 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
  
  /* ---------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
  --------------------------------------------------------------- */

  /* Time base configuration */
  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Period = (10000 - 1);
  // 这个就是预分频系数，当由于为0时表示不分频所以要减1
    TIM_TimeBaseStructure.TIM_Prescaler = (3200 - 1);
 //    TIM_TimeBaseStructure.TIM_Prescaler = (800 - 1);

  // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
  // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //向上计数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器5
  TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

  /* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
  TIM_ClearITPendingBit(TIM11, TIM_IT_Update);

  /* TIM IT enable */ //打开溢出中断
//  TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);

  /* TIM5 enable counter */
  

  if(timeStatus == TIME_START)
  {
		TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
     TIM_Cmd(TIM11, ENABLE);  //计数器使能，开始工作   
  }
  else
  {
    TIM_Cmd(TIM11,DISABLE);  //计数器使能，开始工作   
		TIM_ITConfig(TIM11, TIM_IT_Update, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM11, DISABLE);
  }


}

void NVIC_Time5_Status(uint8_t timeStatus)
{
    NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  /* Enable the TIM5 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM11_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

  if(timeStatus == TIME_START)
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }
  NVIC_Init(&NVIC_InitStructure);

}


void sofeWareWatchBegin(void)
{
   NVIC_Time9_Status(TIME_START);

   Time9_Init_Status(TIME_START);
}
void sofeWareWatchClear(void)
{
	time9Counter = 0x00;
}

void sofeWareWatchEnd(void)
{
   NVIC_Time9_Status(TIME_END);

   Time9_Init_Status(TIME_END);

}
void Time9_Init_Status(uint8_t timeStatus)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_DeInit(TIM9);
  /* TIM5 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
  
  /* ---------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 7200, TIM2 counter clock = 7.2 MHz
  --------------------------------------------------------------- */

  /* Time base configuration */
  //这个就是自动装载的计数值，由于计数是从0开始的，计数10000次后为9999
  TIM_TimeBaseStructure.TIM_Period = (20000 - 1);
  // 这个就是预分频系数，当由于为0时表示不分频所以要减1
  TIM_TimeBaseStructure.TIM_Prescaler = (3200 - 1);
  // 高级应用本次不涉及。定义在定时器时钟(CK_INT)频率与数字滤波器(ETR,TIx)
  // 使用的采样频率之间的分频比例
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //向上计数
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器5
  TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

  /* Clear TIM5 update pending flag[清除TIM5溢出中断标志] */
  TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

  /* TIM IT enable */ //打开溢出中断
//  TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);

  /* TIM5 enable counter */
  

  if(timeStatus == TIME_START)
  {
		TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
    time9Counter = 0x00;
	  TIM_Cmd(TIM9, ENABLE);  //计数器使能，开始工作   
  }
  else
  {
    TIM_Cmd(TIM9,DISABLE);  //计数器使能，开始工作   
		TIM_ITConfig(TIM9, TIM_IT_Update, DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,DISABLE);
  }
}

void NVIC_Time9_Status(uint8_t timeStatus)
{
    NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Set the Vector Table base address at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);

  /* Enable the TIM5 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM9_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

  if(timeStatus == TIME_START)
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
     NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }
  NVIC_Init(&NVIC_InitStructure);
}

#define WATCH_DOG_TIME_OUT	1200 //1450*2/60 = 48min 1200*2/60=40

void sofeWareWatchDog(void)
{
   time9Counter ++;
   if(time9Counter==WATCH_DOG_TIME_OUT)
   {
   	  time9Counter  = 0x00;
			rstStatus = RST_STATUS_SWRST;
			SaveData_SWReset();		
			if(devUsartStatus == USART_OFF)devUSART_RUN();
			printf("\r\n swReset:4 swResetTimerOver\r\n");
	  	sysSoftReset();
   } 
}
