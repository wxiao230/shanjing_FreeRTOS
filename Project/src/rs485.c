/**
  ******************************************************************************
  * @file    Project/user/rs485.c
  * @author  insentek wxh
  * @version V3.0.1
  * @date    1-December-2020
  * @brief   
  ******************************************************************************   
  */  

/* Includes ------------------------------------------------------------------*/
#include "rs485.h"						 
//#include "gpio.h"
#include "gprs.h"
#include "deviceSet.h"
#include "sensor.h"
#include "rtc.h"
#include "main.h"
#include "crc16.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t	rs485Power	=	rs485_NONE;
#if defined(STM32L1XX_MDP)
	uint8_t	rs485RxBuffer[RS485_RX_DATA_SIZE+1];
#endif
__IO uint16_t rs485RxCount	=	0;
__IO uint16_t	rx485RxStorePtr	=	0;
__IO uint16_t	rx485RxOutPtr	=	0;
#if(USART_USE_CTR>0)
__IO uint8_t 	sampleTimesContinus	=	0;
#endif
__IO uint16_t	rs485HistoryNumber	=	0;
__IO uint8_t 	rs485CmdParamId	=	0;

__IO uint8_t inputReg[MODBUS_INPUTREG_SIZE+1];
__IO uint8_t holdReg[MODBUS_HOLDREG_SIZE+1];

__IO uint32_t 	currs485SendAddr	= SENSOR_DATA_BASE_ADDR;

__IO uint8_t 	devWorkStatus	=	DEV_STATUS_OFFLINE;
__IO uint8_t 	devType				=	DEV_TYPE_INIT;

__IO uint8_t 	modbusAddress		= MODBUS_ADDRESS_Init;
__IO uint8_t	devSleepCtr			=	DEVSLEEP_INIT;
__IO uint8_t	usartTimeLimit	=	RS485_WAIT_TIME_INIT;
__IO uint16_t	rs485WaitTime		=	RS485_WAIT_TIME_INIT;
__IO uint16_t	devWorkCmd			=	DEV_WORK_CMD_NONE;
__IO uint8_t	modbusMultiLen=	0;

__IO uint8_t	rs485TimerStatus	=	rs485_NONE;
__IO uint16_t	rs485TimerCounter = 0;

uint16_t	rs485Band =	rs485BaudInit;
__IO uint8_t	rs485UsartStatus	=	0;
__IO uint32_t	rs485WaitTimeBegin	=	0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void rs485_delay_ms(uint32_t delay)
{  
	uint16_t i;
	while(delay)
	{
		delay--;
		for(i=8300;i>0;i--);
	}
}
void rs485PowerOn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RS485_EN_RCC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin 		= RS485_EN_PIN;	
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_Init(RS485_EN_Source,&GPIO_InitStructure);
	RS485_EN_ON();
	rs485_delay_ms(1);
	rs485Power	=	rs485_DONE;
}
void rs485PowerOff(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	rs485Power	=	rs485_NONE;
	RS485_EN_OFF();	
	RCC_AHBPeriphClockCmd(RS485_EN_RCC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin 		= RS485_EN_PIN;	
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_400KHz;
	GPIO_Init(RS485_EN_Source,&GPIO_InitStructure);		
}
void rs485DE_RUN(void)
{
#if(RS485_DE_USED>0)
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RS485_DE_RCC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin 		= RS485_DE_PIN;	
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_Init(RS485_EN_Source,&GPIO_InitStructure);
	RS485_DE_RX();
#endif
}
void rs485DE_SLEEP(void)
{
#if(RS485_DE_USED>0)
	GPIO_InitTypeDef GPIO_InitStructure;
	RS485_DE_RX();
	RCC_AHBPeriphClockCmd(RS485_DE_RCC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin 		= RS485_DE_PIN;	
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd 		= GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_400KHz;
	GPIO_Init(RS485_DE_Source,&GPIO_InitStructure);	
#endif
}
void rs485DmaInit(void)
{
  DMA_InitTypeDef  DMA_InitStructure;
	
/* Enable the DMA periph */
	RCC_AHBPeriphClockCmd(RS485_USART_RCC_DMA, ENABLE);
	/* Enable the USART Rx DMA request */
	USART_DMACmd(RS485_USART, USART_DMAReq_Rx, DISABLE);
	
	/* Enable the DMA channel */
	DMA_Cmd(RS485_DMA_Channel, DISABLE);		
	DMA_SetCurrDataCounter(RS485_DMA_Channel,0);
  /* DMA Configuration -------------------------------------------------------*/
  DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)(&RS485_USART->DR);
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode 								= RS485_DMA_MODE;
  DMA_InitStructure.DMA_M2M 								= DMA_M2M_Disable;	
	/* DMA channel Rx of USART Configuration */
	DMA_DeInit(RS485_DMA_Channel);
	DMA_InitStructure.DMA_BufferSize 					= (uint16_t)RS485_RX_DATA_SIZE+1;
	DMA_InitStructure.DMA_MemoryBaseAddr 			= (uint32_t)rs485RxBuffer;
	DMA_InitStructure.DMA_DIR 								= DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority 						= DMA_Priority_Low;
	DMA_Init(RS485_DMA_Channel, &DMA_InitStructure);

	/* Enable the USART Rx DMA request */
	USART_DMACmd(RS485_USART, USART_DMAReq_Rx, ENABLE);

	/* Enable the DMA channel */
	DMA_Cmd(RS485_DMA_Channel, ENABLE);			
}
void rx485RxInit(void)
{
	rs485RxCount = RS485_RX_DATA_SIZE+1-RS485_DMA_Channel->CNDTR;
}
void rs485USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RS485_USART_PIN_RCC, ENABLE);
  
  /* Enable USART clock */
  RS485_USART_RCC_Cmd(RS485_USART_RCC, ENABLE);
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(RS485_USART_Source,RS485_USART_TX_Source,RS485_USART_AF);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(RS485_USART_Source,RS485_USART_RX_Source,RS485_USART_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin 	= RS485_USART_TX_PIN|RS485_USART_RX_PIN;
  GPIO_Init(RS485_USART_Source, &GPIO_InitStructure);
  

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate 		= rs485Band;
  USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
  USART_InitStructure.USART_StopBits 		= USART_StopBits_1;
  USART_InitStructure.USART_Parity 			= USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(RS485_USART, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQHandler;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(RS485_USART, ENABLE);	
	(void)USART_GetFlagStatus(RS485_USART,USART_FLAG_IDLE);
  (void)USART_ReceiveData(RS485_USART);
 #if(RS485_USART_DMA_CTR>0)
	USART_ITConfig(RS485_USART, USART_IT_IDLE, ENABLE);
	rs485DmaInit();
 #else
	USART_ClearITPendingBit(RS485_USART,USART_FLAG_RXNE);
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE); 
 #endif
	rs485UsartStatus	=	1;
}
void rs485UsartLowPower(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	rs485UsartStatus = 0;
#if(RS485_USART_DMA_CTR>0)	
	USART_DMACmd(RS485_USART, USART_DMAReq_Rx, DISABLE);	
	DMA_Cmd(RS485_DMA_Channel, DISABLE);		
#endif	
	USART_Cmd(RS485_USART, DISABLE);
	
	USART_ITConfig(RS485_USART, USART_IT_RXNE, DISABLE);
	USART_ITConfig(RS485_USART, USART_IT_IDLE, DISABLE);

	/* Enable USART clock */
	RS485_USART_RCC_Cmd(RS485_USART_RCC, DISABLE);
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RS485_USART_PIN_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin 	= RS485_USART_TX_PIN|RS485_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;  
	GPIO_Init(RS485_USART_Source, &GPIO_InitStructure);
}
void rs485DirCtr(uint8_t status)
{
#if(RS485_DE_USED>0)
	if(status	== rs485_DIR_RX){RS485_DE_RX();}
	else{RS485_DE_TX();}
#endif
}
void rs485TimerCtr(uint8_t status)
{	
	rs485TimerCounter		=	0;
	rs485TimerStatus 		= status;
}
void rs485Clear(void)
{
	uint16_t i;
	for(i=0;i<RS485_RX_DATA_SIZE+1;i++)rs485RxBuffer[i] = 0;
	rs485RxCount		= 0;
	rx485RxStorePtr	=	0;
	rx485RxOutPtr		=	0;
	sampleTimesContinus	=	0;
	rs485HistoryNumber	=	0;
	rs485WaitTimeBegin	=	curSecondTime;//rs485TimerCounter		=	0;
}
void rs485_RUN(void)
{
	if(rs485Power	== rs485_NONE)
	{
		rs485PowerOn();
		rs485Band	=	rs485BaudInit;//rs485BaudInit; BAUD_460800 BAUD_115200 BAUD_9600
		rs485Clear();
		rs485USART_Config();
		rs485DE_RUN();
		rs485TimerCtr(rs485_DONE);
//		currs485SendAddr	=	curGprsSendAddr;	
	#if(rs485_DEBUG>0)
		if(debugStatus == DEBUG_STATUS_DONE)
		{
			printf("\r\n rs485 powerOn\r\n");
		}
	#endif
	}
}
void rs485_SLEEP(void)
{
	rs485PowerOff();
	rs485UsartLowPower();
	rs485DE_SLEEP();
	rs485TimerCtr(rs485_NONE);
#if(rs485_DEBUG>0)
	if(debugStatus == DEBUG_STATUS_DONE)
	{
		printf("\r\n rs485 powerOff\r\n");
	}
#endif
}
void rs485Receive(void)
{
	 USART_ReceiveData(RS485_USART);
}
void rs485Idle(void)
{
	uint8_t clear = clear;	
	rx485RxInit();
	clear = RS485_USART->SR;
  clear = RS485_USART->DR;
}
void rs485TimeDeal(void)
{
	if(rs485TimerStatus == rs485_DONE)
	{
		rs485TimerCounter++;
	}
#if(BEEP_ALARM_CTR>0)
	if(alarmbeepStatus > ALARM_STATUS_FREE)
	{
		alarmDelayTimes++;
		if(alarmDelayTimes> 600)
		{
			alarmbeepStatus			=	ALARM_STATUS_FREE;
			alarmDelayTimes = 0;
		}
	}
#endif
}
uint16_t findStrRxRsUsart(uint8_t *buf,uint8_t bufLength,uint8_t remainLen,uint16_t rxOutPtr)
{
	uint16_t findi 		= 0x00;
	uint16_t findj 		= 0x00;
	uint16_t ptrOut 	= 0x00;
	uint16_t counter 	= 0x00;
	uint16_t ptr    	= 0x00;
	ptrOut 						= rxOutPtr;		
	if(rx485RxOutPtr <= rx485RxStorePtr)
	{
		counter = rx485RxStorePtr - ptrOut;
	}
	else
	{
		counter = RS485_RX_DATA_SIZE+1 + rx485RxStorePtr - ptrOut;
	}
	if((bufLength+remainLen) > counter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < counter;findi++)
	{
		if(rs485RxBuffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(rs485RxBuffer[ptr++] == buf[findj])
				{
					ptr = ptr&RS485_RX_DATA_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut = ptrOut&RS485_RX_DATA_SIZE;
					break;
				}
			}
			if(findj ==bufLength)
			{
				return ptr;
			} 
		}
		else
		{
			ptrOut ++;
			ptrOut = ptrOut&RS485_RX_DATA_SIZE;
		} 
	}
	return 	UN_FIND_STATUS;
}


uint16_t modbusRegReadAck(uint8_t cmd,uint16_t address,uint16_t length,uint8_t *bufOut,__IO uint8_t *bufin)
	{
	uint16_t i;
	uint16_t dataOutLen=0;		
	uint16_t dataInLen;
	value16_t crc16;
	uint8_t	 index = 0;
	dataOutLen	=	0;
	dataInLen		=	length<<1;
	index				=	address<<1;
	bufOut[dataOutLen++]	=	modbusAddress;
	bufOut[dataOutLen++]	=	cmd;
	bufOut[dataOutLen++]	=	dataInLen;
	for(i = 0;i<dataInLen;i++)
	{
		bufOut[dataOutLen++] = bufin[i+index];
	}
	crc16.i		= CRC16_1(bufOut,dataOutLen);

	bufOut[dataOutLen++]	=	crc16.bytes.valueL;
	bufOut[dataOutLen++]	=	crc16.bytes.valueH;
#if(rs485_DEBUG>0)	
	printf("\r\n dataLen:%d,crc16:0x%0X\r\n",dataOutLen,crc16.i);
	for(dataInLen=0;dataInLen<dataOutLen;dataInLen++)printf(" %02X",bufOut[dataInLen]);	
	printf("\r\n");
#endif
	return dataOutLen;
}
uint8_t rs485Putchar(__IO uint8_t ch)
{
	uint16_t i = FAULT_TIME_MAX;	
	while ((USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET)&&(i--))
	{
	}
	USART_SendData(RS485_USART,(uint8_t)ch);
	return ch;
}

int8_t modbusSendData(uint8_t *buf,uint8_t len)
{	
	uint16_t i=0;
	if(rs485Power	==	rs485_DONE)
	{
		rs485DirCtr(rs485_DIR_TX);
		for(i=0;i<len;i++)rs485Putchar(buf[i]);	
		i=FAULT_TIME_MAX;	
		while ((USART_GetFlagStatus(RS485_USART, USART_FLAG_TC) == RESET)&&(i--))
		{
		}
		rs485DirCtr(rs485_DIR_RX);
		rs485WaitTimeBegin	=	getCurrterTime();//rs485TimerCounter = 0;
	}
	return 1;
}
void modbusOkAck(uint8_t cmd,uint16_t addr,uint16_t data)
{	
	value16_t crc16;
	uint8_t 	buf[8];	
	
	buf[0]	= modbusAddress;
	buf[1]	= cmd;
	crc16.i	=	addr;
	buf[2]	= crc16.bytes.valueH;
	buf[3]	= crc16.bytes.valueL;		
	crc16.i	=	data;
	buf[4]	= crc16.bytes.valueH;
	buf[5]	= crc16.bytes.valueL;

	crc16.i	= CRC16_1(buf,6);
	
	buf[6]	= crc16.bytes.valueL;
	buf[7]	= crc16.bytes.valueH;
	modbusSendData(buf,8);
#if(rs485_DEBUG>0)	
	printf("\r\n dataLen:8,crc16:0x%0X\r\n",crc16.i);
	for(crc16.i=0;crc16.i<8;crc16.i++)printf(" %02X",buf[crc16.i]);	
	printf("\r\n");
#endif
}
void modbusErrorAck(uint8_t cmd,uint8_t errorCode)
{	
	value16_t crc16;
	uint8_t 	buf[5];	
	
	buf[0]	= modbusAddress;
	buf[1]	= cmd|0x80;
	buf[2]	= errorCode;

	crc16.i	= CRC16_1(buf,3);
	
	buf[3]	= crc16.bytes.valueL;
	buf[4]	= crc16.bytes.valueH;
	
	modbusSendData(buf,5);	
#if(rs485_DEBUG>0)	
	printf("\r\n dataLen:5,crc16:0x%0X\r\n",crc16.i);
	for(crc16.i=0;crc16.i<5;crc16.i++)printf(" %02X",buf[crc16.i]);	
	printf("\r\n");
#endif
}
uint8_t modbusHoldRegUpdate(void)
{
	uint8_t	index = 0;
	value16_t	tempValue;	
	DS1302_TIME  rtcCurTime;
	readDs1302Time(&rtcCurTime);
	index	=	0;
	//modbus地址
	holdReg[index++]	=	0x00;
	holdReg[index++]	=	modbusAddress;
	//连续采样次数
	holdReg[index++]	=	0x00;
	holdReg[index++]	=	sampleTimesContinus;
	//读取历史条数记录
	tempValue.i	=	rs485HistoryNumber;
	holdReg[index++]	=	tempValue.bytes.valueH;
	holdReg[index++]	=	tempValue.bytes.valueL;
	//采样周期
	tempValue.i	=	(uint16_t)(curDevPeriod/60);
	
	holdReg[index++]	=	tempValue.bytes.valueH;
	holdReg[index++]	=	tempValue.bytes.valueL;
	//当前时间数据
	holdReg[index++]= rtcCurTime.year;
	holdReg[index++]= rtcCurTime.month;
	holdReg[index++]= rtcCurTime.date;
	holdReg[index++]= rtcCurTime.hour;
	holdReg[index++]= rtcCurTime.min;
	holdReg[index++]= rtcCurTime.sec;		
	//休眠功能状态
	holdReg[index++]	=	0x00;
	holdReg[index++]	=	devSleepCtr;
	//设备工作状态
	tempValue.i	=	devWorkCmd;
	holdReg[index++]	=	tempValue.bytes.valueH;
	holdReg[index++]	=	tempValue.bytes.valueL;
	return index;
}
void modbusInputRegUpdate(void)
{
	uint8_t index = 0;
	//设备工作状态
	inputReg[index++] = 0x00;
	inputReg[index++] = devWorkStatus;
	//设备Id
	inputReg[index++] = devId.bytes.value4;
	inputReg[index++] = devId.bytes.value3;
	inputReg[index++] = devId.bytes.value2;
	inputReg[index++] = devId.bytes.value1;
	//设备类型
	inputReg[index++] = 0x00;
	inputReg[index++] = devType;
}

uint16_t formatSensorFlashDataModbus(__IO uint8_t *bufOut)
{
	__IO uint16_t dataOutLength= 0x00;
	__IO uint16_t dataInLength = 0x00;
	__IO value_t tempValue;
	uint8_t tmpi = 0;
	
	//工作状态
	bufOut[dataOutLength++] = 0x00;
	bufOut[dataOutLength++] = 0x01;
	//设备Id
	bufOut[dataOutLength++] = devId.bytes.value4;
	bufOut[dataOutLength++] = devId.bytes.value3;
	bufOut[dataOutLength++] = devId.bytes.value2;
	bufOut[dataOutLength++] = devId.bytes.value1;
	//设备类型
	bufOut[dataOutLength++] = 0x00;
	bufOut[dataOutLength++] = devType;
	
	//采集时间
	bufOut[dataOutLength++] = sensorValue.time.year;//年
	bufOut[dataOutLength++] = sensorValue.time.month;//月
	bufOut[dataOutLength++] = sensorValue.time.date;//日
	bufOut[dataOutLength++] = sensorValue.time.hour;//时
	bufOut[dataOutLength++] = sensorValue.time.min;//分
	bufOut[dataOutLength++] = sensorValue.time.sec;//秒	
	//设备状态	
	//设备总的采集次数装载	
	tempValue.i	= sensorValue.countAll.i;	
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	
	//采集次数装载
	//电池电压
	tempValue.f	= sensorValue.batV.f;		
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	//充电电压数据装载
	//地表温度
	tempValue.f	= sensorValue.t.f;	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	//水分及温度
#if(DEV_MAX_NODE_NUM>10)
	for(tmpi=0;tmpi<10;tmpi++)
#else
	for(tmpi=0;tmpi<DEV_MAX_NODE_NUM;tmpi++)
#endif
	{
		//水分
		tempValue.f	=  ROUND3(sensorValue.wat[tmpi].f);		
		bufOut[dataOutLength++] = tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;
		bufOut[dataOutLength++] = tempValue.bytes.value4;
		bufOut[dataOutLength++] = tempValue.bytes.value3;		
		//温度
		tempValue.f	= sensorValue.soilt[tmpi].f;	
		bufOut[dataOutLength++] = tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;
		bufOut[dataOutLength++] = tempValue.bytes.value4;
		bufOut[dataOutLength++] = tempValue.bytes.value3;	
	}
	//倾角X
	tempValue.f	=  ROUND3(sensorValue.dipX.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	//倾角Y
	tempValue.f	=  ROUND3(sensorValue.dipX.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;	
	//倾角Z
	tempValue.f	=  ROUND3(sensorValue.dipZ.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;	
	//倾角angle
	tempValue.f	=  ROUND3(sensorValue.angle.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;		
	//倾角YAW
	tempValue.f	=  ROUND3(sensorValue.dipYaw.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
#if(SK60_CTR>0)	
	//激光位移
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	tempValue.f	=  ROUND3(sensorValue.sk60disp.f);	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
}
#endif	
#if(SENSOR_DATA_FORMAT_KEY>1)
	printf("\r\ndataLen:%d\r\n",dataOutLength);
	for(dataInLength=0;dataInLength<dataOutLength;dataInLength++)
	printf(" %02X",bufOut[dataInLength]);
	printf("\r\n");
//	devStoreBuffer(bufOut,dataOutLength);
#endif		
	return dataOutLength;			   
}


int8_t findMoubusCmdReg(modbus_t *data,uint8_t cmd)
{
	uint16_t 	findStatus = UN_FIND_STATUS;		
	value16_t crc16;		
	uint16_t 	ptrOut 	= 0x00;
	
	ptrOut	= rx485RxOutPtr;
	while(ptrOut!=rx485RxStorePtr)
	{
		data->buf[0] = modbusAddress;
		data->buf[1] = cmd;
		findStatus = findStrRxRsUsart(data->buf,2,6,ptrOut);
		if((findStatus>= FIND_ERROR)&&(cmd == WRITE_SINGLEREG))
		{
			data->buf[0] = 0;
			data->buf[1] = cmd;
			findStatus  = findStrRxRsUsart(data->buf,2,6,ptrOut);
		}
		if(findStatus >= FIND_ERROR){return ERROR_STATUS_NONE;}
		
		ptrOut	=	findStatus;
		data->buf[2] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[3] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[4] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[5] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->ask.crc.bytes.valueL	= rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->ask.crc.bytes.valueH	= rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		crc16.i		= CRC16_1(data->buf,6);
		if((crc16.i == data->ask.crc.i)||(data->ask.crc.i==MODBUS_CRC_TEST))
		{
			rs485RxBuffer[(ptrOut-1)&RS485_RX_DATA_SIZE]=0;
			rs485RxBuffer[(ptrOut-2)&RS485_RX_DATA_SIZE]=0;		
			return ERROR_STATUS_OK;
		}
	}
	return ERROR_STATUS_CRC;
}
int8_t findModbusCmdReadInputReg(void)
{
	int8_t rxResult = -1;
	uint8_t sendLen;
	modbus_t data;
	value16_t tempValue;
	
	rxResult	=	findMoubusCmdReg(&data,READ_INPUTREG);
	if(rxResult	!= ERROR_STATUS_OK)return rxResult;
			
	tempValue.i	=	data.ask.regAddr.i;
	data.ask.regAddr.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regAddr.bytes.valueL = tempValue.bytes.valueH;		
	tempValue.i	=	data.ask.regNum.i;
	data.ask.regNum.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regNum.bytes.valueL = tempValue.bytes.valueH;		

	if(data.ask.regNum.i>INPUTREG_BUFFER_LEN)
	{
		modbusErrorAck(READ_INPUTREG,ERROR_DATA);
		return ERROR_STATUS_DATA;
	}
	
	if((data.ask.regAddr.i<INPUTREG_ADDR_Start)||(data.ask.regAddr.i>INPUTREG_ADDR_End))		
	{
		rxResult = ERROR_STATUS_REG;
	}
	else if((data.ask.regNum.i+data.ask.regAddr.i)>(INPUTREG_ADDR_End+1))		
	{
		rxResult = ERROR_STATUS_REG;
	}	
	if(rxResult == ERROR_STATUS_REG)
	{
		modbusErrorAck(READ_INPUTREG,ERROR_REG_ADDR);
		return ERROR_STATUS_REG;
	}

	modbusInputRegUpdate();
	data.ask.regAddr.i	-= INPUTREG_ADDR_Start;
	sendLen = modbusRegReadAck(READ_INPUTREG,data.ask.regAddr.i,data.ask.regNum.i,sensorBuf,inputReg);
	
	modbusSendData(sensorBuf,sendLen);
//	delay_ms(25);		
	return(ERROR_STATUS_OK);
}
int8_t findModbusCmdReadHoldReg(void)
{
	int8_t rxResult = -1;
	uint8_t sendLen;
	modbus_t data;
	value16_t tempValue;
	
	rxResult	=	findMoubusCmdReg(&data,READ_HOLDREG);
	if(rxResult	!= ERROR_STATUS_OK)return rxResult;
	
	tempValue.i	=	data.ask.regAddr.i;
	data.ask.regAddr.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regAddr.bytes.valueL = tempValue.bytes.valueH;	
	tempValue.i	=	data.ask.regNum.i;
	data.ask.regNum.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regNum.bytes.valueL = tempValue.bytes.valueH;
	
	if((data.ask.regNum.i>HOLDREG_BUFFER_LEN)||(data.ask.regNum.i==0))	
	{
		modbusErrorAck(READ_HOLDREG,ERROR_DATA);
		return ERROR_STATUS_DATA;
	}
	
	if((data.ask.regAddr.i<HOLDREG_ADDR_Start)||(data.ask.regAddr.i>HOLDREG_ADDR_End))		
	{
		rxResult = ERROR_STATUS_REG;			
	}
	else if((data.ask.regNum.i+data.ask.regAddr.i)>(HOLDREG_ADDR_End+1))
	{
		rxResult = ERROR_STATUS_REG;
	}
	if(rxResult == ERROR_STATUS_REG)
	{
		modbusErrorAck(READ_HOLDREG,ERROR_REG_ADDR);
		return ERROR_STATUS_REG;
	}

	modbusHoldRegUpdate();
	data.ask.regAddr.i	-= HOLDREG_ADDR_Start;
	sendLen = modbusRegReadAck(READ_HOLDREG,data.ask.regAddr.i,data.ask.regNum.i,sensorBuf,holdReg);
	
	modbusSendData(sensorBuf,sendLen);
//	delay_ms(25);	
	return(ERROR_STATUS_OK);
}
int8_t findModbusCmdWriteSingleReg(httpConfig *usartConfig)
{
	int8_t rxResult = -1;
	modbus_t 	data;
	value16_t tempValue;

	rxResult	=	findMoubusCmdReg(&data,WRITE_SINGLEREG);
	if(rxResult	!= ERROR_STATUS_OK)return rxResult;
	
	tempValue.i	=	data.ask.regAddr.i;
	data.ask.regAddr.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regAddr.bytes.valueL = tempValue.bytes.valueH;					
	if((data.ask.regAddr.i<HOLDREG_ADDR_Start)||(data.ask.regAddr.i>HOLDREG_ADDR_End))
	{
		modbusErrorAck(WRITE_SINGLEREG,ERROR_REG_ADDR);
		return ERROR_STATUS_REG;
	}	
	tempValue.i	=	data.ask.regNum.i;
	data.ask.regNum.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regNum.bytes.valueL = tempValue.bytes.valueH;
	data.ask.regAddr.i	-= HOLDREG_ADDR_Start;
	rxResult	=	ERROR_STATUS_DATATPYE;
	switch(data.ask.regAddr.i)
	{
		case HOLDREG_ADDR_ADDRESS:
		{
			if((data.ask.regNum.i>0)&&(data.ask.regNum.i<=MODBUS_ADDRESS_MAX))
			{
				rxResult	=	ERROR_STATUS_OK;
				modbusAddress	=	data.ask.regNum.i;
				devModbusAddrSave();
			}
			break;
		}
		case HOLDREG_ADDR_SAMPLE:
		{
			if((data.ask.regNum.i>0)&&(data.ask.regNum.i<=USART_SAMPLE_MAX))
			{
				rxResult	=	ERROR_STATUS_OK;
				sampleTimesContinus	=	data.ask.regNum.i;
				sendDataCounter			= SEND_STATUS_USART;
				if(mcuPowerStatus	!=	MCU_ON_STATE)
				{
					mcuPowerStatus	=	MCU_ON_STATE;
					mcuRunStatus		= MCURUNSTATUS_Start;
				}
			}
			break;
		}
		case HOLDREG_ADDR_HISTORY:
		{
			if((data.ask.regNum.i>0)&&(data.ask.regNum.i<=USART_HISTORY_MAX))
			{
				rxResult						= ERROR_STATUS_OK;	
				rs485HistoryNumber	=	data.ask.regNum.i;
				usartConfig->requestData						=	rs485HistoryNumber;
				usartConfig->requestDataReadAddress	=	currs485SendAddr;
								
				if(!historyDataFind(usartConfig))
				{
				#if rs485_DEBUG > 0
					printf("\r\n find flash history  data  %d \r\n",usartConfig->requestData);	
				#endif									   
				}		
			}    
			break;
		}	
		case HOLDREG_ADDR_PERIOD:
		{
			if((data.ask.regNum.i>=USART_PERIOD_MIN)&&(data.ask.regNum.i<=USART_PERIOD_MAX))
			{
				rxResult	= ERROR_STATUS_OK;				
				configValue.devPeriod 		= data.ask.regNum.i*60;
				configValue.devPeriodType	= NORMAL_PERIOD_TYPE;	
				serverConfig.storeFlagConfig ++; 
				serverConfig.perSetFlag	=	CHANGE_FLAG;
			}    
			break;
		}
		case HOLDREG_ADDR_SLEEPCTR:
		{
			if(data.ask.regNum.bytes.valueH&0x80)
			{
				data.ask.regNum.bytes.valueH &= 0x7F;
				if((data.ask.regNum.bytes.valueH==DEVSLEEP_NONE)||(data.ask.regNum.bytes.valueH ==DEVSLEEP_DONE))
				{					
					if(data.ask.regNum.bytes.valueL<251)
					{
						devSleepCtr			=	data.ask.regNum.bytes.valueH;
						usartTimeLimit	=	data.ask.regNum.bytes.valueL;
						if(usartTimeLimit>240){rs485WaitTime = (usartTimeLimit-240)*3600;}
						else if(usartTimeLimit>120){rs485WaitTime = (usartTimeLimit-120)*60;}
						else if(usartTimeLimit>0){rs485WaitTime = usartTimeLimit;}
						
						devModbusAddrSave();
						rxResult	=	ERROR_STATUS_OK;	
					}					
				}										
			}
			break;
		}	
		case HOLDREG_ADDR_TIME_YM:
		case HOLDREG_ADDR_TIME_DH:
		case HOLDREG_ADDR_TIME_MS:		
		case HOLDREG_ADDR_WORKCMD:
		default:
		{
			rxResult = ERROR_STATUS_CMD;
			modbusErrorAck(WRITE_SINGLEREG,ERROR_REG_ADDR);
			break;
		}
	}	
	if(rxResult == ERROR_STATUS_OK)
	{
		modbusOkAck(WRITE_SINGLEREG,data.ask.regAddr.i+HOLDREG_ADDR_Start,data.ask.regNum.i);			
	}
	else if(rxResult == ERROR_STATUS_DATATPYE)
	{
		modbusErrorAck(WRITE_SINGLEREG,ERROR_DATA);
	}
	return(rxResult);
}	

int8_t findModbusCmdReg_Multi(modbus_multi_t *data)
{		
	uint16_t findStatus = UN_FIND_STATUS;				
	uint16_t 	remainCounter;
	uint8_t 	bytesNum;
	uint16_t  ptrOut	= 0x00;
	value16_t crc16;	
	
	ptrOut	= rx485RxOutPtr;
	
	while(ptrOut!=rx485RxStorePtr)
	{
		data->buf[0] = modbusAddress;
		data->buf[1] = WRITE_MULTIPLEREG;
		findStatus  = findStrRxRsUsart(data->buf,2,9,ptrOut);
		if(findStatus>= FIND_ERROR)
		{
			data->buf[0] = 0;
			data->buf[1] = WRITE_MULTIPLEREG;
			findStatus  = findStrRxRsUsart(data->buf,2,9,ptrOut);
		}
		if(findStatus >= FIND_ERROR){return ERROR_STATUS_NONE;}
		
		ptrOut	=	findStatus;
		data->buf[2] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[3] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[4] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[5] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->buf[6] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		
		crc16.bytes.valueH	=	data->buf[4];
		crc16.bytes.valueL	=	data->buf[5];
		if(crc16.i !=(data->ask.bytesNum>>1))continue;
	
		if(findStatus < rx485RxStorePtr)
		{
			remainCounter = rx485RxStorePtr - findStatus;
		}
		else
		{
			remainCounter =RS485_RX_DATA_SIZE+1 + rx485RxStorePtr - findStatus;
		}
		if((remainCounter<(data->ask.bytesNum+2))||(data->ask.bytesNum>240))continue;
		//{return ERROR_STATUS_TYPE;}//rx485RxOutPtr = ptrOut;
		
		for(bytesNum = 0;bytesNum<data->ask.bytesNum;bytesNum++)
		{
			data->buf[bytesNum+7] = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		}

		data->ask.crc.bytes.valueL = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->ask.crc.bytes.valueH = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		crc16.i		= CRC16_1(data->buf,(bytesNum+7));
		
		if((crc16.i == data->ask.crc.i)||(data->ask.crc.i==MODBUS_CRC_TEST))
		{
			rs485RxBuffer[(ptrOut-1)&RS485_RX_DATA_SIZE]=0;
			rs485RxBuffer[(ptrOut-2)&RS485_RX_DATA_SIZE]=0;	
			return ERROR_STATUS_OK;
		}
	}
	return ERROR_STATUS_CRC;
}

int8_t findModubsCmdWriteMultipleReg(httpConfig *usartConfig)
{
	int8_t rxResult = ERROR_STATUS_NONE;		
	modbus_multi_t data;	
	value16_t tempValue;	
	uint8_t 	bytesNum;
	uint8_t 	addr		=	0;
	uint8_t		times 	= 0;
	uint8_t		history	= 0;
	uint16_t	period	=	0;		
	DS1302_TIME setTime	=	{0,0,0,0,0,0,0};
	
	rxResult	=	findModbusCmdReg_Multi(&data);	

	if(rxResult != ERROR_STATUS_OK) return rxResult;
			
	tempValue.i	=	data.ask.regAddr.i;
	data.ask.regAddr.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regAddr.bytes.valueL = tempValue.bytes.valueH;
	tempValue.i	=	data.ask.regNum.i;
	data.ask.regNum.bytes.valueH = tempValue.bytes.valueL;
	data.ask.regNum.bytes.valueL = tempValue.bytes.valueH;
	
	if((data.ask.regNum.i>HOLDREG_BUFFER_LEN)||(data.ask.bytesNum!=(data.ask.regNum.i<<1)))
	{					
		modbusErrorAck(WRITE_MULTIPLEREG,ERROR_DATA);
		return ERROR_STATUS_DATA;
	}
	if((data.ask.regAddr.i<HOLDREG_ADDR_Start)||((data.ask.regAddr.i+data.ask.regNum.i)>HOLDREG_ADDR_End))
	{
		modbusErrorAck(READ_HOLDREG,ERROR_REG_ADDR);
		return ERROR_STATUS_REG;
	}

	data.ask.regAddr.i	-= HOLDREG_ADDR_Start;		
	rxResult	=	ERROR_STATUS_OK;
	bytesNum	=	7;
	switch(data.ask.regAddr.i)
	{
		case HOLDREG_ADDR_ADDRESS:
		{
			tempValue.bytes.valueH	=	data.buf[bytesNum++];
			tempValue.bytes.valueL	=	data.buf[bytesNum++];
			if((tempValue.i>0)&&(tempValue.i<=MODBUS_ADDRESS_MAX)){addr =	tempValue.i;}
			else
			{
				rxResult	=	ERROR_STATUS_DATATPYE;
				break;
			}
			data.ask.regNum.i --;
			if(data.ask.regNum.i==0)break;
		}
		case HOLDREG_ADDR_SAMPLE:
		{
			tempValue.bytes.valueH	=	data.buf[bytesNum++];
			tempValue.bytes.valueL	=	data.buf[bytesNum++];
			if((tempValue.i>0)&&(tempValue.i<=CMD_SAMPLE)){times	=	tempValue.i;}
			else
			{
				rxResult	=	ERROR_STATUS_DATATPYE;
				break;
			}
			data.ask.regNum.i --;
			if(data.ask.regNum.i==0)break;
		}
		case HOLDREG_ADDR_HISTORY:
		{
			tempValue.bytes.valueH	=	data.buf[bytesNum++];
			tempValue.bytes.valueL	=	data.buf[bytesNum++];
			if((tempValue.i>0)&&(tempValue.i<=USART_HISTORY_MAX)){history	=	tempValue.i;}
			else
			{
				rxResult	=	ERROR_STATUS_DATATPYE;
				break;
			}
			data.ask.regNum.i --;
			if(data.ask.regNum.i==0)break;
		}	
		case HOLDREG_ADDR_PERIOD:
		{
			tempValue.bytes.valueH	=	data.buf[bytesNum++];
			tempValue.bytes.valueL	=	data.buf[bytesNum++];
			if((tempValue.i>=USART_PERIOD_MIN)&&(tempValue.i<=USART_PERIOD_MAX))
			{
				period	= tempValue.i*60;
				serverConfig.perSetFlag	=	CHANGE_FLAG;
			}
			else
			{
				rxResult	=	ERROR_STATUS_DATATPYE;
				break;
			}
			data.ask.regNum.i --;
			if(data.ask.regNum.i==0)break;
		}
		case HOLDREG_ADDR_TIME_YM:
			if(data.ask.regNum.i<3){rxResult	=	ERROR_STATUS_DATATPYE;break;}
			setTime.year	=	data.buf[bytesNum++];
			setTime.month	=	data.buf[bytesNum++];
			data.ask.regNum.i --;
		case HOLDREG_ADDR_TIME_DH:
			if((data.ask.regNum.i<2)||(data.ask.regAddr.i == HOLDREG_ADDR_TIME_DH)){rxResult	=	ERROR_STATUS_DATATPYE;break;}
			setTime.date	=	data.buf[bytesNum++];
			setTime.hour	=	data.buf[bytesNum++];
			data.ask.regNum.i --;
		case HOLDREG_ADDR_TIME_MS:
			if(data.ask.regAddr.i == HOLDREG_ADDR_TIME_MS){rxResult	=	ERROR_STATUS_DATATPYE;break;}
			setTime.min		=	data.buf[bytesNum++];
			setTime.sec		=	data.buf[bytesNum++];				
			if(rtcCheck(setTime) == RTCERR){rxResult	=	ERROR_STATUS_DATATPYE;break;}			
			setTime.day = 0x01;	
			
			data.ask.regNum.i --;
			if(data.ask.regNum.i==0)break;							
		case HOLDREG_ADDR_SLEEPCTR:
		{
			tempValue.bytes.valueH	=	data.buf[bytesNum++];
			tempValue.bytes.valueL	=	data.buf[bytesNum++];
			if(tempValue.bytes.valueH&0x80)
			{
				tempValue.bytes.valueH &= 0x7F;				
				if((tempValue.bytes.valueH==DEVSLEEP_NONE)||(tempValue.bytes.valueH ==DEVSLEEP_DONE))
				{					
					if(tempValue.bytes.valueL<251)
					{
						devSleepCtr			=	tempValue.bytes.valueH;
						usartTimeLimit	=	tempValue.bytes.valueL;
						if(usartTimeLimit>240){rs485WaitTime = (usartTimeLimit-240)*3600;}
						else if(usartTimeLimit>120){rs485WaitTime = (usartTimeLimit-120)*60;}
						else if(usartTimeLimit>0){rs485WaitTime = usartTimeLimit;}
						devModbusAddrSave();
						rxResult	=	ERROR_STATUS_OK;	
					}					
				}										
			}
			data.ask.regNum.i --;
			//if(data.ask.regNum.i==0)
			break;
		}	
		case HOLDREG_ADDR_WORKCMD:
		default:
		{
			rxResult = ERROR_STATUS_CMD;
			modbusErrorAck(WRITE_SINGLEREG,ERROR_REG_ADDR);
			break;
		}
	}
	if(rxResult == ERROR_STATUS_OK)
	{
		modbusOkAck(WRITE_SINGLEREG,data.ask.regAddr.i+HOLDREG_ADDR_Start,data.ask.regNum.i);
		if(addr != 0)
		{
			modbusAddress	=	addr;
			devModbusAddrSave();
		}
		if(times != 0)
		{
			sampleTimesContinus	=	times;
			sendDataCounter			= SEND_STATUS_USART;
			if(mcuPowerStatus	!=	MCU_ON_STATE)
			{
				mcuPowerStatus	=	MCU_ON_STATE;
				mcuRunStatus		= MCURUNSTATUS_Start;
			}
		}
		if(history != 0)
		{
			rs485HistoryNumber									=	history;
			usartConfig->requestData						=	rs485HistoryNumber;
			usartConfig->requestDataReadAddress	=	currs485SendAddr;
							
			if(!historyDataFind(usartConfig))
			{
			#if rs485_DEBUG > 0
				printf("\n\r find flash history  data  %d \n\r",usartConfig->requestData);	
			#endif									   
			}		
		}
		if(period != 0)
		{
			configValue.devPeriod 		= period;
			configValue.devPeriodType	= NORMAL_PERIOD_TYPE;	
			serverConfig.perSetFlag		=	CHANGE_FLAG;
			serverConfig.storeFlagConfig ++; 
		}
		if((setTime.month != 0)&&(setTime.date != 0)&&(setTime.day == 1))
		{			
			setTimeTask(setTime);
			usartConfig->rtcSetFlag	= CHANGE_FLAG;
		}
	}
	else if(rxResult == ERROR_STATUS_DATATPYE)
	{
		modbusErrorAck(WRITE_MULTIPLEREG,ERROR_DATA);
	}
	return(rxResult);
}

#if(BEEP_ALARM_CTR>0)
int8_t findModbusAck_Multi(modbus_multi_t *data)
{		
	uint16_t findStatus = UN_FIND_STATUS;				
	uint16_t  ptrOut	= 0x00;
	value16_t crc16;	
	
	ptrOut	= rx485RxOutPtr;
	
	while(ptrOut!=rx485RxStorePtr)
	{
		findStatus  = findStrRxRsUsart(data->buf,2,6,ptrOut);		
		if(findStatus >= FIND_ERROR){return ERROR_STATUS_NONE;}
		
		ptrOut	=	findStatus;
		crc16.bytes.valueH = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		crc16.bytes.valueL = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;		
		if(crc16.i !=(data->ask.regAddr.i))continue;
		
		crc16.bytes.valueH = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		crc16.bytes.valueL = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;		
		if(crc16.i !=(data->ask.regNum.i))continue;	
		
		data->ask.crc.bytes.valueL = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		data->ask.crc.bytes.valueH = rs485RxBuffer[findStatus++];findStatus &= RS485_RX_DATA_SIZE;
		crc16.i		= CRC16_1(data->buf,6);
		
		if(crc16.i == data->ask.crc.i)
		{
			rs485RxBuffer[(ptrOut-1)&RS485_RX_DATA_SIZE]=0;
			rs485RxBuffer[(ptrOut-2)&RS485_RX_DATA_SIZE]=0;	
			return ERROR_STATUS_OK;
		}
	}
	return ERROR_STATUS_CRC;
}
int8_t alarmAckSendok(void)
{
	int8_t result = -1;	
	value16_t crc16;	
	uint16_t status = 0;

	uint8_t buf[8];
	buf[0]	=	0x01;
	buf[1]	=	0x10;
	crc16.i = MODBUS_ALARM_REG_ADDR;
	buf[2]	=	crc16.bytes.valueH;
	buf[3]	=	crc16.bytes.valueL;
	buf[4]	=	0x00;
	buf[5]	=	0x01;
	crc16.i	= CRC16_1(buf,6);	

	status  = findStrRxRsUsart(buf,6,2,rx485RxOutPtr);
	if(status<FIND_ERROR)
	{
		if(rs485RxBuffer[status]==crc16.bytes.valueL)
		{
			if(rs485RxBuffer[(status+1)&RS485_RX_DATA_SIZE]==crc16.bytes.valueH)
			{
				result = 0;
				alarmbeepStatus = ALARM_STATUS_ACKOK;
				alarmDelayTimes	=	0;
				for(crc16.i=0;crc16.i<8;crc16.i++)
				{
					rs485RxBuffer[(status+1-crc16.i)&RS485_RX_DATA_SIZE]=0;
				}
			#if(rs485_DEBUG>0)	
				printf(" alarm %d begin\r\n",alarmTimes);
			#endif
			}
		}		
	}
	return result;
}

int8_t alarmAckEndOk(void)
{
	int8_t result   = -1;	
	uint16_t status = 0;
	uint8_t tmpi 		= 0;
	uint8_t buf[5];
	buf[0]	=	0x01;
	buf[1]	=	0x83;
	buf[2]	=	0x05;
	buf[3]	=	0x81;
	buf[4]	=	0x33;

	status = findStrRxRsUsart(buf,5,0,rx485RxOutPtr);
	if(status<FIND_ERROR)
	{
		result	= 0;	
		alarmDelayTimes	=	0;	
		if(alarmTimes>0)alarmTimes--;
		if(alarmTimes == 0)
		{
			alarmbeepStatus = ALARM_STATUS_FREE;
		}
		else
		{
			alarmbeepStatus = ALARM_STATUS_ENDOK;
		}
		for(tmpi=0;tmpi<5;tmpi++)
		{
			rs485RxBuffer[(status+1-tmpi)&RS485_RX_DATA_SIZE]=0;
		}	
	#if(rs485_DEBUG>0)	
		printf(" alarm %d end\r\n",alarmTimes);
	#endif
	}
	return result;
}
#endif

void usartFindCmdModbus(httpConfig *usartConfig)
{
	int8_t rxResult[5] = {-1,-1,-1,-1,-1};
	int8_t rxResultSum = 1;
	uint8_t rxStatus	=	0;
	
	while(rxResultSum>0)
	{
		rxResult[0] = findModbusCmdReadHoldReg();
		rxResult[1] = findModbusCmdReadInputReg();
		rxResult[2] = findModbusCmdWriteSingleReg(usartConfig);
		rxResult[3] = findModubsCmdWriteMultipleReg(usartConfig);
	
		if((rxResult[0]==0)||(rxResult[1]==0)||(rxResult[2]==0)||(rxResult[3]==0))
		{
			rxResultSum = 2;
			rxStatus 		= 1;			
		}
		else
		{
			if(rxResultSum == 1)rxStatus	= 1;
			rxResultSum = -1;
		}
	#if(rs485_DEBUG>0)
		if((debugStatus == DEBUG_STATUS_DONE)&&(rxStatus == 1))
		{
			rxStatus	= 0;
			printf("\r\n readHoldReg:%d,",rxResult[0]);	
			printf("readInputReg:%d,",rxResult[1]);	
			printf("writeHoldReg_06:%d,",rxResult[2]);	
			printf("writeHoldReg_16:%d\r\n",rxResult[3]);
		}
	#endif
	}
		
}

void rs485RxTask(void)
{		
	int8_t	readDataResult	= -1;
	uint16_t 	counter	= 0;
__IO	uint16_t	sendLen	= 0;
	httpConfig 	usartConfig;
#if((RS485_RX_TEST>0)&&(rs485_DEBUG>0)&&(PROTOCOL_CTR==PROTOCOL_Modbus))	
	uint16_t 	tmpi	=	0;
#endif		
	usartConfig.requestData	=	0;
	
	do
	{
		if((rs485UsartStatus == 1)&&(rs485RxCount	!= rx485RxStorePtr))//if(rx485RxOutPtr	!= rx485RxStorePtr)
		{
			rx485RxStorePtr	=	rs485RxCount;
			if(rx485RxOutPtr < rx485RxStorePtr)
			{
				counter = rx485RxStorePtr - rx485RxOutPtr;
			}
			else 
			{
				counter =RS485_RX_DATA_SIZE+1 + rx485RxStorePtr - rx485RxOutPtr;
			}
			
		#if((RS485_RX_TEST>0)&&(rs485_DEBUG>0))
			if(debugStatus == DEBUG_STATUS_DONE)
			{
				printf("\r\n rxStore:%d,rxOut:%d,couner:%d\r\n",rx485RxStorePtr,rx485RxOutPtr,counter);
				if(rx485RxOutPtr < rx485RxStorePtr)
				{			
				#if(PROTOCOL_CTR==PROTOCOL_Modbus)
					for(tmpi=rx485RxOutPtr;tmpi<rx485RxStorePtr;tmpi++)printf(" %02X",rs485RxBuffer[tmpi]);
				#else
					devStoreBuffer(&rs485RxBuffer[rx485RxOutPtr],counter);
				#endif
				}
				else
				{				
				#if(PROTOCOL_CTR==PROTOCOL_Modbus)
					for(tmpi=rx485RxOutPtr;tmpi<=RS485_RX_DATA_SIZE;tmpi++)printf(" %02X",rs485RxBuffer[tmpi]);
					for(tmpi=0;tmpi<rx485RxStorePtr;tmpi++)printf(" %02X",rs485RxBuffer[tmpi]);
				#else
					devStoreBuffer(&rs485RxBuffer[rx485RxOutPtr],RS485_RX_DATA_SIZE+1-rx485RxOutPtr);
					devStoreBuffer(&rs485RxBuffer[0],rx485RxStorePtr);
				#endif
				}		
				printf("\r\n");
			}
		#endif		
		#if(PROTOCOL_CTR==PROTOCOL_Modbus)
			if(counter>7)
			{
				usartFindCmdModbus(&usartConfig);
			#if(BEEP_ALARM_CTR>0)
				if(alarmbeepStatus	== ALARM_STATUS_SEND){alarmAckSendok();}
				else if(alarmbeepStatus	== ALARM_STATUS_ACKOK){alarmAckEndOk();}
			#endif
			}
		#else
			
		#endif	
			rx485RxOutPtr	=	rx485RxStorePtr;
		}
		if(usartConfig.requestData>0)
		{
			readDataResult = sensorDataReadToBuf(SEND_TYPE_USART,usartConfig.requestDataReadAddress,sensorBuf,&sendLen);
			if(readDataResult == 1)
			{
				modbusSendData(sensorBuf,sendLen);
			#if(SWRESET_CTR>0)	
				SWRESET_TIMER_INIT();
			#endif
				//delay_ms(25);
			}
		#if(rs485_DEBUG>0)
			printf("\r\n rs_DEBUG HistoryData:%d,%d\r\n",usartConfig.requestData,readDataResult);		  
		#endif
			usartConfig.requestDataReadAddress += SENSOR_DATA_STORE_SIZE;
			if(usartConfig.requestDataReadAddress >= SENSOR_DATA_END_ADDR)
			{
				usartConfig.requestDataReadAddress = SENSOR_DATA_BASE_ADDR;
			}		
			usartConfig.requestData --;
			rs485HistoryNumber	=	usartConfig.requestData;		
		}
	}
	while(usartConfig.requestData>0);
	
}
void rs485TxTask(void)
{ 
	int8_t		readDataResult	= -1;
	DS1302_TIME sendTime;	
__IO	uint16_t 	sendLen 	= 0;
	if(rs485UsartStatus == 1)
	{
	readDs1302Time(&sendTime);
//	rainMultiAdcStatus	=	STATUS_DONE;
		
	while(unSendDataPacketUSART)
	{
		rs485RxTask();
//		rainMultiAdcStatus	=	STATUS_DONE;
	
		readDataResult = sensorDataReadToBuf(SEND_TYPE_USART,currs485SendAddr,sensorBuf,&sendLen);		
		if(readDataResult == 1)
		{	
			readDataResult = modbusSendData(sensorBuf,sendLen);
		#if(GPRS_USE_CTR==0)
			if(readDataResult == 0)gprsSendSucFlagWrite(currs485SendAddr,currs485SendAddr,sendTime);	
		#endif
			unSendDataPacketUSART	--;
//			delay_ms(25);
		}
		currs485SendAddr	+= SENSOR_DATA_STORE_SIZE;
		if(currs485SendAddr >= SENSOR_DATA_END_ADDR)
		{
			currs485SendAddr = SENSOR_DATA_BASE_ADDR;
		}		
		if(currs485SendAddr == sensorDataWriteAddr){unSendDataPacketUSART = 0;}
	#if(rs485_DEBUG>0)
		printf("\r\n unSendData rs485:%d,gprs:%d\r\n",unSendDataPacketUSART,unSendDataPacket);		  
	#endif
	}
	
	currs485SendAddr 	= sensorDataWriteAddr;
//	rainMultiAdcStatus	=	STATUS_NONE;	
}
}


void rs485RunTask(void)
{
	if(devPar.usartKey == FUNCITONG_TURN_ON)
	{
		rs485_RUN();
		rs485RxTask();
	}
}
void rs485SleepTask(uint8_t sleepctr)
{
	if((devPar.usartKey == FUNCITONG_TURN_ON)&&((rs485Power	== rs485_DONE)))
	{
		rs485RxTask();
		if((sleepctr==rs485_DONE)||((devSleepCtr == DEVSLEEP_DONE)&&((curSecondTime-rs485WaitTimeBegin)>=rs485WaitTime)))
		{
			rs485_SLEEP();		
		}
	}
}

#if(BEEP_ALARM_CTR>0)
__IO uint8_t alarmbeepStatus	= ALARM_STATUS_FREE;
__IO uint8_t alarmTimes		= 0;
__IO uint8_t alarmType		=	MODBUS_ALARM_INDEX_NORMAL;
__IO uint8_t alarmIndex		=	MODBUS_ALARM_INDEX_RAIN2;
__IO uint16_t	alarmDelayTimes	=	0;

void modbusMultiSend(modbus_multi_t *data)
{
	value16_t crc16;
	uint8_t 	buf[256];	
	uint8_t		buflength=0;
	uint8_t 	tmpi;

	buflength=0;
	buf[buflength++]	= data->ask.addr;
	buf[buflength++]	= data->ask.func;
	buf[buflength++]	= data->ask.regAddr.bytes.valueH;
	buf[buflength++]	= data->ask.regAddr.bytes.valueL;		
	buf[buflength++]	= data->ask.regNum.bytes.valueH;
	buf[buflength++]	= data->ask.regNum.bytes.valueL;
	buf[buflength++]	= data->ask.bytesNum;
	for(tmpi=0;tmpi<data->ask.regNum.i;tmpi++)
	{
		buf[buflength++]	= data->ask.reg[tmpi].bytes.valueH;
		buf[buflength++]	= data->ask.reg[tmpi].bytes.valueL;	
	}
	crc16.i	= CRC16_1(buf,buflength);
	
	buf[buflength++]	= crc16.bytes.valueL;
	buf[buflength++]	= crc16.bytes.valueH;
	modbusSendData(buf,buflength);
	
#if(rs485_DEBUG>0)	
	printf("\r\n multiSendLen:%d,crc16:0x%0X\r\n",buflength,crc16.i);
	for(tmpi=0;tmpi<buflength;tmpi++)printf(" %02X",buf[tmpi]);	
	printf("\r\n");
#endif
}

void rs485AlarmSendTask(uint16_t alarmMode)
{
	modbus_multi_t data;
	if((alarmTimes>0)&&((alarmbeepStatus == ALARM_STATUS_FREE)||(alarmbeepStatus == ALARM_STATUS_ENDOK)))
	{	
		data.ask.addr 			= MODBUS_SLAVE_ADDR_INIT;
		data.ask.func 			= WRITE_MULTIPLEREG;
		data.ask.regAddr.i 	= MODBUS_ALARM_REG_ADDR;
		data.ask.regNum.i 	= 0x01;
		data.ask.bytesNum 	= 0x02;
		data.ask.reg[0].i		= alarmMode;
		alarmDelayTimes			=	0;
		alarmbeepStatus			= ALARM_STATUS_SEND;
		rs485_RUN();rs485_delay_ms(1000);
		modbusMultiSend(&data);	
	}
	while(alarmbeepStatus	== ALARM_STATUS_SEND)
	{
		rs485RxTask();
		alarmDelayTimes++;
		if(alarmDelayTimes>500)alarmbeepStatus = ALARM_STATUS_FREE;
		if(alarmbeepStatus	!= ALARM_STATUS_SEND)break;		
		rs485_delay_ms(10);
	}
//	else if(alarmStatus	== ALARM_STATUS_SEND)
//	{
//		if(alarmDelayTimes>10) alarmStatus = ALARM_STATUS_FREE;
//	}
//	else if(alarmStatus	== ALARM_STATUS_ACKOK)
//	{
//		if(alarmDelayTimes>300)	alarmStatus = ALARM_STATUS_FREE;
//	}
//	
	if(alarmbeepStatus == ALARM_STATUS_FREE)alarmTimes = 0;
}

#endif

