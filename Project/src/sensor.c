#include "sensor.h"
#include "time.h"
#include "power.h"
#include "rtc.h"
#include "ds18b20.h" 
#include "main.h"
#include "math.h"
#include "mode.h"
#include "hydrology.h"
#define SENSOR_DEBUG     1


#if   (SENSOR_DEBUG > 1)
//static __IO uint32_t calCount = 0x00;
#endif

//sendPer devPeriod;

soilfaw curFaw[DEV_MAX_NODE_NUM];
soilabc curAbc[DEV_MAX_NODE_NUM];
#include <stm32l1xx.h>
soilwat curWat[DEV_MAX_NODE_NUM];
value_t oldWat[2];
value_t	oldWatSum,curWatSum;
//soilwatT curWatT[DEV_MAX_NODE_NUM];
devDep   curDep[DEV_MAX_NODE_NUM];

/*dev sn*/
uint8_t devSnBuf[DEV_SN_SIZE+2];/*auth code*/
uint8_t authCodeBuf[AUTH_CODE_SIZE+1];

#define THE_BEGIN_CAL_TIME     4
#define THE_CAL_TIME           1
#define THE_END_CAL_TIME      (THE_BEGIN_CAL_TIME+THE_CAL_TIME) 

value_t devId = {DEBUG_DEVID};
//����������
uint8_t  devSenNum = UN_INIT_NUM;
//ÿ���豸����������
uint8_t  eachDevNum = UN_INIT_SEN_NUM;
//���ݷ�������
// value_t devSenTime = {UN_INIT_TIME};
__IO sensorValue_t sensorValue;
 //value_t devType  = {UN_INIT_TIME};

uint8_t devRegCtr = DEV_UNREG;
uint8_t devGpsCtr  = DEV_GSM_GPS;
 
/*
 �豸�������ͱ�־λ
 */ 
uint8_t sensorDataFlag = 0x00;
uint16_t 	unSendDataPacket   = 0x00;
uint16_t	unSendDataPacketUSART	=	0x00;
//__IO uint8_t  transmissionDensity = 0x01;
#define DEFAULT_DEV_INFOR        0x01010102
//�豸״̬���豸���͡��豸�ڵ㡢�豸ÿ�ڴ���������
value_t devInfor = {DEFAULT_DEV_INFOR};
#define DEFAULT_DEV_COMMUNICTION    0x01010000

 value_t devCommunictionValue = {DEFAULT_DEV_COMMUNICTION};

 value_t devSetValue = {DEFAULT_DEV_INFOR};

 uint8_t devSetStatus = DEV_UNSET_FLAG;
uint8_t devOutPutType = RS232_OUTPUT_FRE;
//__IO value_t requesData;

//�豸�궨״̬��־λ
//�豸δ�궨״̬�������С�ˮ�г�ʼֵδ�趨��

//__IO uint8_t devSetStatus = DEV_UNSET_FLAG;
//  uint8_t gprsCommunictionFlag  = 0x00;
//  uint8_t canCommunictionFlag   = 0x00;
//  uint8_t rs232CommunictionFlag = 0x01;
//  uint8_t nfcCommunictionFlag   = 0x00;
// uint8_t dataSmsSendFlag       = 0x00;
//  uint8_t ctrSmsSendFlag        = 0x00;
//
__IO uint32_t sensorDataWriteAddr = SENSOR_DATA_BASE_ADDR;
__IO uint32_t curGprsSendAddr 		 = SENSOR_DATA_BASE_ADDR;
uint32_t sensorDataReadAddr = 0;
uint32_t gpsDataWriteAddr = 0;
uint32_t xyzDataWriteAddr = 0;
__IO uint8_t sendOrder = 0x00;


void sensorPowerIoInit(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
 /******************************************************************************************************
 *
 *		portA power
 *
 ******************************************************************************************************/
 
 
 
  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERB, ENABLE);  //RCC_APB2Periph_AFIO
 
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin 	= SENSOR_PORTB_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
  GPIO_Init(GPIO_POWERB_PORT, &GPIO_InitStructure);    

//init the power off
//  GPIO_ResetBits(GPIO_POWER1_PORT,SENSOR_PORTA_ALL_POWER);
    GPIO_SetBits(GPIO_POWERB_PORT,SENSOR_PORTB_ALL_POWER);

  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERC, ENABLE);  //RCC_APB2Periph_AFIO
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTC_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_POWERC_PORT, &GPIO_InitStructure);    

//init the power off
//  GPIO_ResetBits(GPIO_POWER1_PORT,SENSOR_PORTA_ALL_POWER);
    GPIO_SetBits(GPIO_POWERC_PORT,SENSOR_PORTC_ALL_POWER);

  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERA, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTA_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_POWERA_PORT, &GPIO_InitStructure);    

//init the power off
//  GPIO_ResetBits(GPIO_POWER1_PORT,SENSOR_PORTA_ALL_POWER);
    GPIO_SetBits(GPIO_POWERA_PORT,SENSOR_PORTA_ALL_POWER);
  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERD, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTD_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_POWERD_PORT, &GPIO_InitStructure);    

//init the power off
//  GPIO_ResetBits(GPIO_POWER1_PORT,SENSOR_PORTA_ALL_POWER);
    GPIO_SetBits(GPIO_POWERD_PORT,SENSOR_PORTD_ALL_POWER);


}


void sensorPowerIoLowPower(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
 /******************************************************************************************************
 *
 *		portA power
 *
 ******************************************************************************************************/
 
       GPIO_ResetBits(GPIO_POWERB_PORT,SENSOR_PORTB_ALL_POWER);
	   GPIO_ResetBits(GPIO_POWERC_PORT,SENSOR_PORTC_ALL_POWER);
	   GPIO_ResetBits(GPIO_POWERA_PORT,SENSOR_PORTA_ALL_POWER);
	   GPIO_ResetBits(GPIO_POWERD_PORT,SENSOR_PORTD_ALL_POWER);

 
  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERB, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTB_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_POWERB_PORT, &GPIO_InitStructure);    



  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERC, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTC_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_POWERC_PORT, &GPIO_InitStructure);    


  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERA, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTA_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_POWERA_PORT, &GPIO_InitStructure);    

  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_POWERD, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_PORTD_ALL_POWER;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_POWERD_PORT, &GPIO_InitStructure);    

   RCC_AHBPeriphClockCmd(RCC_GPIO_FC, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_FC;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIO_FC_PORT, &GPIO_InitStructure); 


}




void sensorPoweCtr(uint8_t whichSensor,uint8_t status)
{
    switch(whichSensor)
    {
	    case   SENSOR1 :
		{
		    if(status == SENSOR_ON )
			{
			 GPIO_ResetBits(GPIO_POWERB_PORT,SENSOR1_POWER);
			}
			else
			{

			 GPIO_SetBits(GPIO_POWERB_PORT,SENSOR1_POWER);	
			}

		     break;
		}
	    case   SENSOR2 :
		{

		    if(status == SENSOR_ON )
			{
				  GPIO_ResetBits(GPIO_POWERB_PORT,SENSOR2_POWER);
			}
			else
			{
				  GPIO_SetBits(GPIO_POWERB_PORT,SENSOR2_POWER);	
		
			}

		     break;
		}
	    case   SENSOR3 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERC_PORT,SENSOR3_POWER);
		
			}
			else
			{
			   GPIO_SetBits(GPIO_POWERC_PORT,SENSOR3_POWER);	
			}

		     break;
		}
	    case   SENSOR4 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERB_PORT,SENSOR4_POWER);		
			}
			else
			{
			   GPIO_SetBits(GPIO_POWERB_PORT,SENSOR4_POWER);	

			}

		     break;
		}
	    case   SENSOR5 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERA_PORT,SENSOR5_POWER);
			
			}
			else
			{
			   GPIO_SetBits(GPIO_POWERA_PORT,SENSOR5_POWER);
			}


		     break;
		}
	    case   SENSOR6 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERC_PORT,SENSOR6_POWER);
			
			}
			else
			{
			   GPIO_SetBits(GPIO_POWERC_PORT,SENSOR6_POWER);
			}

		     break;
		}
	    case   SENSOR7 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERA_PORT,SENSOR7_POWER);
			
			}
			else
			{
			   GPIO_SetBits(GPIO_POWERA_PORT,SENSOR7_POWER);
			}
		     break;
		}
		case   SENSOR8 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERC_PORT,SENSOR8_POWER);		
			}
			else
			{

			  GPIO_SetBits(GPIO_POWERC_PORT,SENSOR8_POWER);	
			}
		     break;
		}
		case   SENSOR9 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERD_PORT,SENSOR9_POWER);		
			}
			else
			{

			  GPIO_SetBits(GPIO_POWERD_PORT,SENSOR9_POWER);	
			}
		     break;
		}
		case   SENSOR10 :
		{
		    if(status == SENSOR_ON )
			{
			  GPIO_ResetBits(GPIO_POWERA_PORT,SENSOR10_POWER);		
			}
			else
			{

			  GPIO_SetBits(GPIO_POWERA_PORT,SENSOR10_POWER);	
			}
		     break;
		}
	   default :
	         break;
	 
	}
}


void initSensorInputIo(void)
{
     GPIO_InitTypeDef GPIO_InitStructure;

   /* Enable GPIOA AFIO clock */
   RCC_AHBPeriphClockCmd(RCC_GPIO_SENSOR, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR_INPUT_ALL;


  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIO_SENSOR_PORT, &GPIO_InitStructure);    
   
}
#define HIGE_FREQ()        (GPIO_FC_PORT->BSRRH = GPIO_FC)
#define LOW_FREQ()         (GPIO_FC_PORT->BSRRL = GPIO_FC)

static void fcPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOB AFIO clock */
  RCC_AHBPeriphClockCmd(RCC_GPIO_FC, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_FC;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIO_FC_PORT, &GPIO_InitStructure); 
  HIGE_FREQ();
}


/*******************************************************************************************************
*
*     sensor2
*
*******************************************************************************************************/

void cal_sensor1(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;  
	
    GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_AHBPeriphClockCmd(RCC_GPIO_FC, ENABLE); //201215
		GPIO_InitStructure.GPIO_Pin 	= GPIO_FC;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_FC_PORT, &GPIO_InitStructure);  
		HIGE_FREQ();
   /* Enable GPIOA AFIO clock */
   RCC_AHBPeriphClockCmd(RCC_GPIO_SENSOR, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
		GPIO_InitStructure.GPIO_Pin = SENSOR1_INPUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_SENSOR_PORT, &GPIO_InitStructure);    	  
    /* Connect Button EXTI Line to Button GPIO Pin */
//    GPIO_EXTILineConfig(SENSOR2_INPUT_EXTI_PORT_SOURCE, SENSOR2_INPUT_EXTI_PIN_SOURCE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
   /* Connect EXTI0 Line to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,SENSOR1_INPUT_EXTI_PIN_SOURCE);
    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = SENSOR1_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);



  
  /* Set the Vector Table base address at 0x08000000 */
#if defined(USE_IAP_BOOT)
         NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);
#else
		 NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
#endif

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


  /* Enable the EXTI Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SENSOR1_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


}


void dis_sensor1(void)
{
   
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
	    GPIO_InitTypeDef GPIO_InitStructure;
       /* Clear the EXTI Line 0 */
    EXTI_ClearITPendingBit(EXTI_Line7);  
    /* Connect EXTI0 Line to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,SENSOR1_INPUT_EXTI_PIN_SOURCE);
    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = SENSOR1_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);


  
  /* Set the Vector Table base address at 0x08000000 */
#if defined(USE_IAP_BOOT)
         NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);
#else
		 NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
#endif

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


  NVIC_InitStructure.NVIC_IRQChannel = SENSOR1_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
//    GPIO_EXTILineConfig(SENSOR2_INPUT_EXTI_PORT_SOURCE, SENSOR2_INPUT_EXTI_PIN_SOURCE);  

   /* Enable GPIOA AFIO clock */
   RCC_AHBPeriphClockCmd(RCC_GPIO_SENSOR, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR1_INPUT;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
 // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_Init(GPIO_SENSOR_PORT, &GPIO_InitStructure); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,DISABLE);
  /* Enable the EXTI Interrupt */



}

/*******************************************************************************************************
*
*     sensor2
*
*******************************************************************************************************/

void cal_sensor2(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;    
    /* Connect Button EXTI Line to Button GPIO Pin */
//    GPIO_EXTILineConfig(SENSOR2_INPUT_EXTI_PORT_SOURCE, SENSOR2_INPUT_EXTI_PIN_SOURCE);  
    GPIO_InitTypeDef GPIO_InitStructure;
		RCC_AHBPeriphClockCmd(RCC_GPIO_FC, ENABLE); //201215
		GPIO_InitStructure.GPIO_Pin 	= GPIO_FC;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIO_Init(GPIO_FC_PORT, &GPIO_InitStructure);  
   LOW_FREQ();
   /* Enable GPIOA AFIO clock */
   RCC_AHBPeriphClockCmd(RCC_GPIO_SENSOR, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR2_INPUT;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIO_SENSOR_PORT, &GPIO_InitStructure); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
   /* Connect EXTI0 Line to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,SENSOR2_INPUT_EXTI_PIN_SOURCE);
    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = SENSOR2_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);



  
  /* Set the Vector Table base address at 0x08000000 */
#if defined(USE_IAP_BOOT)
         NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);
#else
		 NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
#endif

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);


  /* Enable the EXTI Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SENSOR2_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


}


void dis_sensor2(void)
{
   
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;   
    /* Connect Button EXTI Line to Button GPIO Pin */
//    GPIO_EXTILineConfig(SENSOR2_INPUT_EXTI_PORT_SOURCE, SENSOR2_INPUT_EXTI_PIN_SOURCE);  
   
  /* Enable the EXTI Interrupt */

    GPIO_InitTypeDef GPIO_InitStructure;
       /* Clear the EXTI Line 0 */
    EXTI_ClearITPendingBit(EXTI_Line9);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE);
   /* Connect EXTI0 Line to PA0 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,SENSOR2_INPUT_EXTI_PIN_SOURCE);
    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = SENSOR2_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);


  
  /* Set the Vector Table base address at 0x08000000 */
#if defined(USE_IAP_BOOT)
         NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x3000);
#else
		 NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0000);
#endif

  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  NVIC_InitStructure.NVIC_IRQChannel = SENSOR2_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
   /* Enable GPIOA AFIO clock */
   RCC_AHBPeriphClockCmd(RCC_GPIO_SENSOR, ENABLE);  //RCC_APB2Periph_AFIO
  
  /* LEDs pins configuration */
  GPIO_InitStructure.GPIO_Pin = SENSOR2_INPUT;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_Init(GPIO_SENSOR_PORT, &GPIO_InitStructure); 



}


void   sel_sensor(uint8_t whichSensor,uint8_t status)
{
	switch(whichSensor)
    {
	    case   0 :
		{
		    if(status == SENSOR_ON )
			{
			   cal_sensor1();	
//			   cal_sensor2();		
			}
			else
			{
			  dis_sensor1();
//               dis_sensor2();
			}

		     break;
		}
	    case   1 :
		{
		    if(status == SENSOR_ON )
			{
			   cal_sensor2();			
			}
			else
			{
			 
			  dis_sensor2();
			  
			}

		     break;
		}


	   default :
	         break;
	 
	}      



}


#define FREQ_H   0x01
#define FREQ_L   0x02
void fChannelSel(uint8_t whichChannel)
{
  
  
   GPIO_InitTypeDef GPIO_InitStructure;
   TIM_ICInitTypeDef  TIM_ICInitStructure;

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
     /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
     
  /* TIM4 channel 2 pin (PB.07) configuration */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  if(whichChannel ==FREQ_H)
  {

   /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH2 pin (PC.07)  
     The Rising edge is used as active edge,

  ------------------------------------------------------------ */

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,GPIO_AF_TIM3);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
  }
  else
  {
    if(whichChannel ==FREQ_L)
	{
   /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH4 pin (PC.09)  
     The Rising edge is used as active edge,

  ------------------------------------------------------------ */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM3);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_4;
	}
  }


   /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM4 CH2 pin (PB.07)  
     The Rising edge is used as active edge,
     The TIM4 CCR2 is used to compute the frequency value 
  ------------------------------------------------------------ */


  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM3, &TIM_ICInitStructure);

  TIM_TIxExternalClockConfig(TIM3,TIM_TIxExternalCLK1Source_TI1ED,TIM_ICPolarity_Rising,0x0);

  TIM_SelectInputTrigger(TIM3,TIM_TS_ETRF);
}

/*
�豸Ƶ�ʲɼ�
*/

//__IO uint32_t nowSensorNum = 0;


#define CAL_START    0x00
#define CAL_ON       0x01
#define CAL_END      0x02
#define CAL_NO       0x03

__IO uint8_t calSensorState = CAL_NO;
__IO uint32_t timeLocal = 0;
__IO uint32_t keyLocal = 0;
__IO uint8_t nowSensorId  = 0x00;

// ����������������
uint8_t sensorBuf[MAX_SENSOR_BUF_SIZE+1];
//������ֵ��״̬
xyz_t	adxlValue;
//__IO  uint16_t datalength = 0x00; 
//��������֡�������ݰ�����
uint8_t signPackNUm =0x02;
//�����ݲɼ���־
//__IO uint8_t newSensorDataFlag = SENSOR_DATA_IDEL;

//static void delay_SenMs(uint32_t delay)
//{
//    int i;
//	while(delay--)
//	for(i=0;i<8300;i++);
//}
//__IO  uint16_t sBufLength = 0;

//ϵͳ��ص�ѹ���
__IO float batStatus = 0.0;


/*�豸�ܵĲɼ��������������������ݲ���ʧ*/
value_t calAllCounter;
/*�豸���������Ĳɼ��������������������ݶ�ʧ*/
uint16_t calCounter = 0x00;

void signPacketNumSel(void)
{
	switch(devSenNum)
	{
		case 10 :
		{
			signPackNUm = 0x02;
			break;
		}				 	   	
		case 9 :
		case 8 :
		case 7 :
		{
			if(eachDevNum == 0x02 )
			{
				signPackNUm = 0x02;
			}else
			{
				signPackNUm = 0x03;
			}
			break;
		}
		case 6 :
		{
			signPackNUm = 0x03;
			break;
		}	
		case 5 :
		case 4 :
		{
			if(eachDevNum == 0x02 )
			{
				signPackNUm = 0x03;
			}else
			{
				signPackNUm = 0x04;
			}
			break;
		}	
		case 3 :
		case 2 :
		{
			if(eachDevNum == 0x02 )
			{
				signPackNUm = 0x04;
			}else
			{
				signPackNUm = 0x05;
			}
			break;
		}	
		case 1 :
		{
			if(eachDevNum == 0x02 )
			{
				signPackNUm = 0x05;
			}else
			{
				signPackNUm = 0x06;
			}
			break;
		}				   
		default :
		{
			signPackNUm = 0x01;
			break;
		}
	}
}


/* delay_powerMs() removed — use delay_ms() which automatically yields when scheduler is running */

void depPrintf(void)
{
    uint8_t i = 0;
    
    printf("\r\n�������ڵ��� :%d \r\n",devSenNum);
    for(i= 0;i <devSenNum;i++)
    {
    printf("�ڵ�%d,���%.0f\r\n",(i+1),curDep[i].dep.f);
    
    }

}
void cal_sensor(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t curNode = 0;
	value_t senTmp ; 
	uint8_t tryi=0;		//230508
	float fChange = 0;//230508
#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
	uint8_t sensorNumtemp=0;
	if((serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(devPar.stationMode == FUNCITONG_TURN_ON)&&(mqttServerNum>0))
	{
		sensorNumtemp	=	devSenNum;
		devSenNum	=	2;
	}	
#endif	 
	//	power331Off();
//	NVIC_Time9_Status(TIME_START);	  //�������Ź���ʼ��ʱ//	
//	Time9_Init_Status(TIME_START);
	SWRESET_TIMER_INIT();//230508
	readDs1302Time(&rtcTime);//
   //��ص�ѹ��Ϣװ��

   //��ʼ����������Դ����IO
	sensorPowerIoInit();
/*open sensor power*/
	power5vON();
     //������ʱ
#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
	if(sampleTestFlag == 1)
	{
			delay_ms(200);
	}
	else
#endif
	{
		delay_ms(500);
	}
	readAllTemperature();

	nowSensorId = 1;
	curNode = 0;
	fcPinInit();
	printf("�ر��¶�Ϊ:%f\r\n",result[0].rf);
	for(i=1;i< devSenNum +1;i++)
	{
	/*open the sensor power*/
	//����������Դ
		sensorPoweCtr(nowSensorId,SENSOR_ON);
		printf("\r\n��%d������,  ",nowSensorId);
		for(j = 0x00;j < eachDevNum;)//j++ 230508
		{
			timeLocal = 0x00;
			keyLocal = 0;

			calSensorState = CAL_START;
			/*enable the sensor interrupt*/
			sel_sensor(j,SENSOR_ON);
		#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
			if(sampleTestFlag == 1)
			{
				delay_ms(100);
				calSensorState = CAL_ON;
			}
		#endif
			NVIC_Time_Status(TIME_START);
			Time_Init_Status(TIME_START);
			//	while((calSensorState == CAL_ON)||(calSensorState == CAL_START));
			
			uint16_t calTimeout = 0;
		while(calTimeout++ < 5000)
			{
				if(calSensorState == CAL_END)
				{
					break;
				}
				vTaskDelay(1); /* yield 1ms per iteration to allow lower-priority tasks to run */
			}
			sel_sensor(j,SENSOR_OFF);
			if(j == 0x00)
			{
				senTmp.f = (keyLocal)*5120;
				fChange = fabs(curWat[curNode].fh.f-senTmp.f);//230508
				curWat[curNode].fh.f =  senTmp.f;
				printf("��Ƶ��%.0f  ,",senTmp.f);
				senTmp.f  = 0x00;
			}
			else
			{
				if(j == 0x01)
				{
					senTmp.f = keyLocal*320;
					fChange = fabs(curWat[curNode].fl.f-senTmp.f);//230508
					curWat[curNode].fl.f =  senTmp.f;
					printf("��Ƶ��%.0f  .",senTmp.f);
					senTmp.f  = 0x00;				 	
					//	senTmp.f = 0;	
				}
			}				
		#if   (SENSOR_DEBUG > 1)
			printf("�ɼ�%d,",keyLocal);
		#endif
	//	delay_SenMs(20);
	//�¶�У׼
	//�ɼ�Ƶ��ֵ��Χ���
	//����SFֵ
			if(devSetStatus == DEV_SET_FLAG)
			{
			}
				//�ηֱ����ֶ�
			if(eachDevNum == HIG_FRE_ONLY)
			{
				senTmp.f  = 0x00;	
				curWat[curNode].fl.f =  senTmp.f;
				senTmp.f  = 0x00;		
			}	 
			////230508
			tryi++;
			if(((keyLocal > 4000)&&(fChange<3000000))||(tryi>2))//230508
			{
				tryi =0;
				j++;
			}
			else //if(tryi<3) //230508
			{
				sensorPoweCtr(nowSensorId,SENSOR_OFF);
				power5vOff();
				delay_ms(500);
				power5vON();
				delay_ms(500);
				sensorPoweCtr(nowSensorId,SENSOR_ON);
//				delay_ms(100);
			}
	  }
			/*close the sensor power*/
		sensorPoweCtr(nowSensorId,SENSOR_OFF);
//	senTmp.i =  keyLocal;
		printf("�¶�Ϊ:%.2f\n\r",result[nowSensorId].rf);
		nowSensorId++;
		curNode ++;	  
	}
	power5vOff();
	printf("\r\n");	 
#ifdef FLASH_DEBUG
	printf("\n\r��[%d]�βɼ�\n\r",calCount);
#endif
//	printf("\n\r���ݰ�����:\n\r");
#if   (SENSOR_DEBUG > 1)
//	printf("\n\rssss\n\r");
#endif
//	led_Turn_Off(LED_ALL);
//	    POWER_33V_OFF();

	//��������Դ����IO���͹��Ŀ���
//    initPowerIoLowPower();
	sensorPowerIoLowPower();
//	Time9_Init_Status(TIME_END);		   
//	NVIC_Time9_Status(TIME_END);
//����1�͹��Ŀ���
#if   (SENSOR_DEBUG > 1)
//	printfLowPower();
#endif
#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
	if((serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(devPar.stationMode == FUNCITONG_TURN_ON)&&(mqttServerNum>0))
	{
		if(sensorNumtemp>devSenNum)
		{
			if(devSenNum>1)
			{
				for(i=devSenNum;i< sensorNumtemp +1;i++)
				{
					curWat[i].fh.f =curFaw[i].Fa.f;
					
					result[i+1].rf =(result[i].rf+result[i-1].rf+result[i-2].rf)/3;
				}
			}
			else if(devSenNum==1)
			{
				for(i=devSenNum;i< sensorNumtemp +1;i++)
				{
					curWat[i].fh.f =curFaw[i].Fa.f;
					result[i+1].rf =(result[i].rf+result[i-1].rf)/2;
				}
			}
			devSenNum	=	sensorNumtemp;
		}
	}	
#endif	 
}

void cal_work(void)
{
   timeLocal ++;
   switch(calSensorState)
   {
       case CAL_START :
	   {

	   /*wait sensor stabilization and begain cal*/	
		 if(timeLocal ==THE_BEGIN_CAL_TIME)
		 {
		   calSensorState = CAL_ON;
		 }
		  break;
	   }
     case CAL_ON :
	   {
		#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
			if(sampleTestFlag == 1)
			{
				timeLocal =THE_END_CAL_TIME;
			}
		#endif
	    if(timeLocal ==THE_END_CAL_TIME)
			{
		   calSensorState = CAL_END;
		   /*disable the interrupt*/
// 		   sel_sensor(nowSensorId,SENSOR_OFF);


		   Time_Init_Status(TIME_END);
		   NVIC_Time_Status(TIME_END);
		   timeLocal = 0x00;
//		   printf("\n\r��ǰƵ����:[%d]  ",keyLocal);
//		   delay_SenMs(20);
//		   delay_ms(20);
		   /*
		   if(nowSensorId ==0x01)
		   {
		   printf("\n\r��ǰˮ��Ƶ����:[%d]  ",keyLocal);
		   }
		   else
		   {
		   if(nowSensorId ==0x02)
		   printf("\n\r��ǰ����Ƶ����:[%d]  ",keyLocal);		   
		   }
			*/


		} 	  
		  
		  break;
	   }
	   default :
	           break;
   
   }

}

void cal_Num(void)
{
    if(calSensorState == CAL_ON)
	{
	   	keyLocal ++;
	}

}
/*
Ƶ��ֵ�¶�У׼

*/


float calFth(float tep,float fh)
{
  float temp =0;
	float fat =0;

	temp = 1-((tep -25)*(0.00044));
	if(fh >100.0)	  //Ƶ�ʱ߽���
	{
	fat = fh/temp;
	}
	else
	{
	 fat = 0;
	}

	return fat;
}

/*
����SF

*/
float calSf(float Fa,float Faw,float fh)
{
	float temp =0;
	if(Fa >fh)
	{
		temp = (Fa-fh)/Faw;
	}
	else
	{
		temp = 0.0;
	}

	return temp;
}
/*
����ˮ��ֵ
*/
float calWater(float sf,float soila,float soilb,float soilc)
{
	float temp =0;
	double tempWater =0;
	if(sf>soilc)
	{
		temp = (sf-soilc)/soila;
		tempWater =  pow(temp,soilb);
	//printf("cal water %.4f %.4f",temp,tempWater);
	}else
	{
		tempWater = 0.0;
	}

	#ifdef waterDebug
	printf("��ǰˮ��ֵΪ%.4f,",tempWater);
	#endif
	if((tempWater<0.001)||(tempWater>100.0))tempWater=0.0;//201125
	return tempWater;
}

int8_t devWaterOutput(soilwat *curWatValue,soilfaw *curFawValue,soilabc *curAbcValue)
{
	int8_t outputResult = -1;
	uint8_t wateri = 0x00;
	
	float temp1 =0.0;
	float temp2 =0.0;
	for(wateri = 0x00;wateri < devSenNum;wateri++)curWatValue[wateri].wat.f = 0.0;
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		if((curWatValue[wateri].fh.f <MIN_WATER_FR)||(curWatValue[wateri].fh.f > MAX_WATER_FR))
		{
			outputResult = -2;
			return 	outputResult ;
		}
	}
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		if((curFawValue[wateri].Fa.f<MIN_WATER_FR)||(curFawValue[wateri].Fa.f > MAX_WATER_FR))
		{
			outputResult = -3;
			return 	outputResult ;
		}
		if((curFawValue[wateri].Fw.f<MIN_WATER_FR)||(curFawValue[wateri].Fw.f > MAX_WATER_FR))
		{
			outputResult = -3;
			return 	outputResult ;
		}
	}

	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		if(curAbcValue[wateri].a.f < 0.00009 )
		{
			outputResult = -5;
			return 	outputResult ;
		}
	}
	 
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		temp1 = calFth(result[wateri+1].rf,curWatValue[wateri].fh.f); 
		temp2	= calSf(curFawValue[wateri].Fa.f,curFawValue[wateri].Faw.f,temp1);  
		curWatValue[wateri].wat.f = calWater(temp2,curAbcValue[wateri].a.f,curAbcValue[wateri].b2.f,curAbcValue[wateri].c.f);	
	// printf("��ǰˮ��ֵΪ%.4f,%.4f,%.4f",temp1,temp2,curWatValue[wateri].wat.f);
	}
	outputResult = 0;
	return 	outputResult ;
}
uint16_t  devWater(void)
{
  uint8_t i=0;
  uint8_t j =0;
  uint16_t status =0x00;
  float temp1 =0;
  float temp2 =0;

  

  for(i =0;i< devSenNum;i++)
  {

    //�ɼ�Ƶ��ֵ��Χ���
     if((curWat[i].fh.f >MIN_WATER_FR)&&(curWat[i].fh.f < MAX_WATER_FR))
	 {
	 temp1 =  calFth(result[i+1].rf,curWat[i].fh.f);
	 //�豸��ʼֵ���
	 if((curFaw[i].Fa.f > curFaw[i].Fw.f)&&(curFaw[i].Fw.f > MIN_WATER_FR))
	 {
	 temp2 =  calSf(curFaw[i].Fa.f,curFaw[i].Faw.f,temp1);
	 curWat[i].wat.f = calWater(temp2,curAbc[i].a.f,curAbc[i].b2.f,curAbc[i].c.f);
	 }
	 else
	 {
#ifdef waterDebug
    printf("�豸��ʼֵ");
#endif
	      j++;
	      curWat[i].wat.f =0;
	 }
	 //ˮ��ֵ���
	 if((curWat[i].wat.f>MIN_WATER_VALUE)&&(curWat[i].wat.f < MAX_WATER_VALUE))
	 {
	    
	 }
	 else
	 {
	     j++;
	     curWat[i].wat.f =0;	   
	 }

     }
	 else
	 {	
#ifdef waterDebug
    printf("Ƶ��ֵ����");
#endif
	     j++;
	     curWat[i].wat.f =0;
	 }
	 
 #ifdef waterDebug
    printf("��ǰˮ��ֵΪ%.4f,",curWat[i].wat.f);
#endif  
  }

  if(j ==0x00)
  {
     status =0x00;
  }
  else
  {
    status =0x02;
  }
  return status;
}
void calWaterTest(void)
{
   float a = 2.89649;
   float b = 2.475248;
   float c = 0;
   c = pow(a,b);
   printf("�ɼ�%f,",c);
}

float watSumValue(void)
{
	uint8_t nodei=0;
	float watsum=0.0;
	for(nodei=0;nodei<devSenNum;nodei++)
	{
		watsum += curWat[nodei].wat.f;
	}
	return watsum;
}

