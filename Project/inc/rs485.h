/**
  ******************************************************************************
  * @file    Project/user/rs485.h 
  * @author  insentek wxh
  * @version V3.0.1
  * @date    1-December-2020
  * @brief   Header for rs485.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RS485_H
#define __RS485_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "usart.h"

/* Exported define ------------------------------------------------------------*/
#define rs485_DEBUG				1
#define RS485_TEST				0
#define RS485_DE_USED			0
#define RS485_RX_TEST			1

#define RS485_POWER_CTR		1	//0485²»ÐÝÃß 1 485ÐÝÃß
#define DEVSLEEP_INIT			DEVSLEEP_DONE
#define RS485_WAIT_TIME_INIT	10

#define PROTOCOL_Private	0
#define PROTOCOL_Modbus		1
#define PROTOCOL_CTR			PROTOCOL_Modbus 	

#define rs485_NONE				0x00
#define rs485_DONE				0x01
#define rs485_WAIT				0x02

#define rs485_DIR_RX			0x00
#define rs485_DIR_TX			0x01

#define rs485BaudInit						9600
#if defined(STM32L1XX_MDP)
	#define RS485_RX_DATA_SIZE		0x1FF
#else
	#define RS485_RX_DATA_SIZE		GPRS_RX_DATA_SIZE
	#define rs485RxBuffer					GPRS_RX_Buffer
#endif
#define rs485_USART_1						1
#define rs485_USART_2						2
#define rs485_USART_3						3

#define rs485_USART_INIT				rs485_USART_3

#define RS485_USART_DMA_CTR			1
#define RS485_USART_RCC_DMA			RCC_AHBPeriph_DMA1
#define RS485_DMA_MODE					DMA_Mode_Circular//DMA_Mode_Normal DMA_Mode_Circular_220801

#if(rs485_USART_INIT==rs485_USART_1)
	#define RS485_DMA_Channel				DMA1_Channel5
	#define RS485_USART_TX_PIN			GPIO_Pin_9
	#define RS485_USART_RX_PIN			GPIO_Pin_10
	#define RS485_USART_TX_Source		GPIO_PinSource9
	#define RS485_USART_RX_Source		GPIO_PinSource10
	#define RS485_USART_Source			GPIOA
	#define RS485_USART_PIN_RCC			RCC_AHBPeriph_GPIOA	
	#define RS485_USART							USART1
	#define RS485_USART_AF					GPIO_AF_USART1	
	#define RS485_USART_RCC					RCC_APB2Periph_USART1	
	#define RS485_USART_RCC_Cmd			RCC_APB2PeriphClockCmd
	#define RS485_USART_IRQHandler	USART1_IRQn
#elif(rs485_USART_INIT==rs485_USART_3)
	#define RS485_DMA_Channel				DMA1_Channel3
	#define RS485_USART_TX_PIN			GPIO_Pin_10
	#define RS485_USART_RX_PIN			GPIO_Pin_11
	#define RS485_USART_TX_Source		GPIO_PinSource10
	#define RS485_USART_RX_Source		GPIO_PinSource11
	#define RS485_USART_Source			GPIOC
	#define RS485_USART_PIN_RCC			RCC_AHBPeriph_GPIOC	
	#define RS485_USART							USART3
	#define RS485_USART_AF					GPIO_AF_USART3	
	#define RS485_USART_RCC					RCC_APB1Periph_USART3	
	#define RS485_USART_RCC_Cmd			RCC_APB1PeriphClockCmd
	#define RS485_USART_IRQHandler	USART3_IRQn
#endif

//RS485 power control
#define RS485_EN_PIN					GPIO_Pin_4
#define RS485_EN_Source				GPIOC
#define RS485_EN_RCC					RCC_AHBPeriph_GPIOC	
#define RS485_EN_ON()					RS485_EN_Source->BSRRL = RS485_EN_PIN	//1 
#define RS485_EN_OFF()				RS485_EN_Source->BSRRH = RS485_EN_PIN	//0

#define RS485_DE_PIN					GPIO_Pin_2	
#define RS485_DE_Source				GPIOB
#define RS485_DE_RCC					RCC_AHBPeriph_GPIOB	
#define RS485_DE_TX()					RS485_DE_Source->BSRRH = RS485_DE_PIN	//0 
#define RS485_DE_RX()					RS485_DE_Source->BSRRL = RS485_DE_PIN	//1 	

#define USART_SAMPLE_MAX		100
#define USART_PERIOD_MIN		5
#define USART_PERIOD_MAX		240
#define USART_HISTORY_MAX		10000

#define CMD_SAMPLE				0x01//'ISTCFG001:'
#define CMD_SETCLOCK			0x02//'ISTCFG002:'
#define CMD_SETPERIOD			0x03//'ISTCFG003:'
#define CMD_HISTORY				0x04//'ISTCFG004:'
#define CMD_PARAM					0x05//'ISTCFG005:'

#define CMD_STATUS_OK 		0x00
#define CMD_STATUS_ERROR	0x01

/*modbus Ïà¹Ø*/

//Modbus µØÖ·

	#define MODBUS_ADDRESS_Init	1
	#define MODBUS_ADDRESS_MAX	247

//	Modbus ÃüÁîÂë
	#define MODBUS_CMD_NONE			0x00
	#define READ_HOLDREG				0x03
	#define READ_INPUTREG				0x04
	#define WRITE_SINGLEREG			0x06
	#define WRITE_MULTIPLEREG		0x10 
//Modbus ´íÎó´úÂë
	#define ERROR_NONE					0x00
	#define ERROR_FUNC					0x01
	#define ERROR_REG_ADDR			0x02
	#define ERROR_DATA					0x03
	#define ERROR_DEVICE				0x04
	#define ERROR_WAIT					0x05
	#define ERROR_BUSY					0x06
	#define ERROR_CMDERR				0x07
	#define ERROR_CRCERR				0x08
//±£´æ¼Ä´æÆ÷µØÖ·
#define HOLDREG_ADDR_Start			100	 	  
	#define HOLDREG_ADDR_ADDRESS			0
	#define HOLDREG_ADDR_SAMPLE				1
	#define HOLDREG_ADDR_HISTORY			2 
	#define HOLDREG_ADDR_PERIOD				3
	#define HOLDREG_ADDR_TIME_YM			4
	#define HOLDREG_ADDR_TIME_DH			5
	#define HOLDREG_ADDR_TIME_MS			6
	#define HOLDREG_ADDR_SLEEPCTR			7 	
	#define HOLDREG_ADDR_WORKCMD			8
	#define HOLDREG_ADDR_Last					8
#define HOLDREG_ADDR_End					(HOLDREG_ADDR_Start+HOLDREG_ADDR_Last)//108
#define HOLDREG_BUFFER_LEN				(HOLDREG_ADDR_Last+1)
#define MODBUS_HOLDREG_SIZE 			(HOLDREG_BUFFER_LEN<<1)

//ÊäÈë¼Ä´æÆ÷µØÖ·
#define INPUTREG_ADDR_Start					600	
#define INPUTREG_ADDR_WORKSTATUS		0
#define INPUTREG_ADDR_DEVID					1
#define INPUTREG_ADDR_DEVTYPE				3
#define INPUTREG_ADDR_RTC						4
#define INPUTREG_ADDR_COUNTER 			7
#define INPUTREG_ADDR_BATV					9
#define INPUTREG_ADDR_Last					64//
#define INPUTREG_ADDR_SendLast			62//
#define INPUTREG_ADDR_End						(INPUTREG_ADDR_Start+INPUTREG_ADDR_Last)
#define INPUTREG_BUFFER_LEN					(INPUTREG_ADDR_Last+1)

#define MODBUS_INPUTREG_SIZE 			(INPUTREG_BUFFER_LEN<<1)
#define INPUTREG_BUFF_SEND_START	INPUTREG_ADDR_RTC
#define INPUTREG_BUFF_SEND_LEN		(INPUTREG_ADDR_SendLast+1-INPUTREG_BUFF_SEND_START)

#define DEV_SLEEP_CTR_OFF				0x8000
#define DEV_SLEEP_CTR_ON				0x8001
#define DEV_SWRESET_CTR					0x8004
#define	DEV_RESET_DEFALUT				0xA55A
#define DEV_WORK_CMD_NONE				0x00

#define DEV_STATUS_OFFLINE			0x00
#define DEV_STATUS_ONLINE				0x01
#define DEV_STATUS_NORMAL				0x02
#define DEV_STATUS_DONE					0x03
#define DEV_STATUS_ERROR				0x04
#define DEV_STATUS_SLEEP				0x05

#define ERROR_STATUS_OK					0
#define ERROR_STATUS_NONE				-10
#define ERROR_STATUS_TYPE				-11
#define ERROR_STATUS_CRC				-12
#define ERROR_STATUS_REG				-2
#define ERROR_STATUS_DATA				-3
#define ERROR_STATUS_DATATPYE		-4
#define ERROR_STATUS_CMD				-7

#define MODBUS_CRCL		0xC1
#define MODBUS_CRCH		0xC2
#define MODBUS_CRC_TEST	(uint16_t)0xC2C1

#define MODBUS_SLAVE_ADDR_INIT					1
#define MODBUS_ALARM_REG_ADDR						0x2004		
#define MODBUS_ALARM_INDEX_NORMAL				0x01//ÓêÁ¿
#define MODBUS_ALARM_INDEX_RAIN2				0x03
#define MODBUS_ALARM_INDEX_RAIN3				0x04
#define MODBUS_ALARM_INDEX_RAIN4				0x05
#define MODBUS_ALARM_INDEX_WATERLEVEL2	0x07//Ë®Î»
#define MODBUS_ALARM_INDEX_WATERLEVEL3	0x08
#define MODBUS_ALARM_INDEX_WATERLEVEL4	0x09
#define MODBUS_ALARM_INDEX_LANDSLIDE2		0x0B//»¬ÆÂ
#define MODBUS_ALARM_INDEX_LANDSLIDE3		0x0C
#define MODBUS_ALARM_INDEX_LANDSLIDE4		0x0D
#define MODBUS_ALARM_INDEX_COLLAPSE2		0x0F//±ÀËú
#define MODBUS_ALARM_INDEX_COLLAPSE3		0x10
#define MODBUS_ALARM_INDEX_COLLAPSE4		0x11
#define MODBUS_ALARM_INDEX_FLOW2				0x13//ÄàÊ¯Á÷
#define MODBUS_ALARM_INDEX_FLOW3				0x14
#define MODBUS_ALARM_INDEX_FLOW4				0x15

#define ALARM_STATUS_FREE 	0x00
#define ALARM_STATUS_SEND 	0x01
#define ALARM_STATUS_ACKOK 	0x02
#define ALARM_STATUS_ENDOK 	0x03

/* Exported types ------------------------------------------------------------*/
typedef union
{
	uint16_t i;
	struct
	{
		uint8_t valueL;
		uint8_t valueH;
	}bytes;
	struct
	{
		uint8_t valueH;
		uint8_t valueL;	
	}modbus;
}valut16_t;
typedef union
{
	struct
	{
		uint8_t 	modbusAddr;
		uint8_t   func;
		valut16_t regAddr;
		valut16_t regNum;
		valut16_t crc; 	
	}ask;
	uint8_t buf[8];
}modbus_t;

typedef union
{
	struct
	{
		uint8_t 	addr;
		uint8_t   func;
		valut16_t regAddr;
		valut16_t regNum;
		uint8_t		bytesNum;
		valut16_t	reg[120];
		valut16_t crc;
	}ask;
	uint8_t buf[249];
}modbus_multi_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
extern __IO uint8_t		rs485Power;
extern __IO uint8_t 	modbusAddress;
extern __IO uint8_t 	inputReg[MODBUS_INPUTREG_SIZE+1];
extern __IO uint8_t		devSleepCtr;
extern __IO uint16_t	rs485TimerCounter;
extern __IO uint8_t		usartTimeLimit;
extern __IO uint16_t	rs485WaitTime;
extern __IO uint8_t 	sampleTimesContinus;
extern __IO uint8_t		rs485UsartStatus;
extern __IO uint32_t	rs485WaitTimeBegin;
extern __IO uint8_t 	devWorkStatus;
extern __IO uint32_t 	currs485SendAddr;
extern void rs485_RUN(void);
extern void rs485_SLEEP(void);
extern void rs485Receive(void);
extern void rs485Idle(void);
extern void rs485TimerCtr(uint8_t status);
extern void rs485TimeDeal(void);
extern uint16_t formatSensorFlashDataModbus(__IO uint8_t *bufOut);
extern uint16_t modbusRegReadAck(uint8_t cmd,uint16_t address,uint16_t length,uint8_t *bufOut,__IO uint8_t *bufin);
extern void rs485Task(void);
extern void rs485RxTask(void);
extern void rs485TxTask(void);
extern void rs485Test(void);
extern void rs485RunTask(void);
extern void rs485SleepTask(uint8_t sleepctr);

extern __IO uint8_t alarmbeepStatus;
extern __IO uint8_t alarmTimes;
extern __IO uint8_t alarmType;
extern __IO uint8_t	alarmIndex ;
extern __IO uint16_t	alarmDelayTimes;
extern void rs485AlarmSendTask(uint16_t alarmMode);

#endif /* __RS485232_H */

/*****************************END OF FILE*****************************/
