/**
  ******************************************************************************
  * @file    Project/user/sk60.h 
  * @author  insentek wxh
  * @version V1.0.0
  * @date    9-September-2019
  * @brief   Header for sk60.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_SK60_H
#define _SK60_H 		
/* Includes ------------------------------------------------------------------*/
	#include "stm32l1xx.h" 
	#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	float t;
	float v;
	float dis;
	uint16_t riss;
	uint16_t err;	
}sk60_t;

/* Exported define ------------------------------------------------------------*/
	#define SK60_DEBUG 	1
	#define SK60_BUFFER_SIZE		0x07//7 
	#define SK60_RX_BUFFER_SIZE	GPRS_RX_DATA_SIZE //0x7F //
	#define SK60_INIT_FLAG			9
	#define SK60_BASE_FLAG			8
	#define SK60_PRESS_FLAG			7
	#define SK60_SET_FLAG				6
	#define SK60_SET_FUNC_FLAG	5

	#define SK60_USART_Baud				19200	
	#define SK60_USART_1					1
	#define SK60_USART_2					2
	#define SK60_USART_3					3
//#if defined(STM32L1XX_MDP)
//	#define SK60_USART_INIT				SK60_USART_1
//#else
//	#define SK60_USART_INIT				SK60_USART_1
//#endif

#if(SK60_CTR>2)
	#define SK60_EN_PIN					GPIO_Pin_4
	#define SK60_EN_Source			GPIOC
	#define SK60_EN_RCC					RCC_AHBPeriph_GPIOC
	#define SK60_EN_ON()				SK60_EN_Source->BSRRL = SK60_EN_PIN	//1
	#define SK60_EN_OFF()				SK60_EN_Source->BSRRH = SK60_EN_PIN	//0
	#define SK60_USART_INIT			SK60_USART_3
#else	
	#define SK60_EN_PIN					GPIO_Pin_12
	#define SK60_EN_Source			GPIOB
	#define SK60_EN_RCC					RCC_AHBPeriph_GPIOB
	#define SK60_EN_ON()				SK60_EN_Source->BSRRH = SK60_EN_PIN	//0
	#define SK60_EN_OFF()				SK60_EN_Source->BSRRL = SK60_EN_PIN	//1
	#define SK60_USART_INIT			SK60_USART_1
#endif
	

#if(SK60_USART_INIT==SK60_USART_1)
	#define SK60_USART_TX_PIN			GPIO_Pin_9
	#define SK60_USART_RX_PIN			GPIO_Pin_10
	#define SK60_USART_TX_Source	GPIO_PinSource9
	#define SK60_USART_RX_Source	GPIO_PinSource10
	#define SK60_USART_Source			GPIOA
	#define SK60_USART_PIN_RCC		RCC_AHBPeriph_GPIOA	
	#define SK60_USART						USART1
	#define SK60_USART_AF					GPIO_AF_USART1	
	#define SK60_USART_RCC				RCC_APB2Periph_USART1	
	#define SK60_USART_RCC_Cmd		RCC_APB2PeriphClockCmd
	#define SK60_USART_IRQHandler	USART1_IRQn
	#define sk60StoreBuffer 			usart1StoreBuffer
#elif(SK60_USART_INIT==SK60_USART_3)
	#define SK60_USART_TX_PIN			GPIO_Pin_10
	#define SK60_USART_RX_PIN			GPIO_Pin_11
	#define SK60_USART_TX_Source	GPIO_PinSource10
	#define SK60_USART_RX_Source	GPIO_PinSource11
	#define SK60_USART_Source			GPIOC
	#define SK60_USART_PIN_RCC		RCC_AHBPeriph_GPIOC	
	#define SK60_USART						USART3
	#define SK60_USART_AF					GPIO_AF_USART3	
	#define SK60_USART_RCC				RCC_APB1Periph_USART3
	#define SK60_USART_RCC_Cmd		RCC_APB1PeriphClockCmd
	#define SK60_USART_IRQHandler	USART3_IRQn
	#define sk60StoreBuffer 			uart3StoreBuffer
#endif

	#define SK60_CMD_ON						'O'//0x4F 开启激光
	#define SK60_CMD_OFF					'C'//0x43 关闭激光
	#define SK60_CMD_STATUS				'S'//0x53 读取模块状态
	#define SK60_CMD_MEASURE_AUTO	'D'//0x44 启动自动测量
	#define SK60_CMD_MEASURE_SLOW	'M'//0x4D 启动慢速测量
	#define SK60_CMD_MEASURE_FAST	'F'//0x46 启动快速测
	#define SK60_CMD_VERSION			'V'//0x56 模块版本查询
	#define SK60_CMD_POWEROFF			'X'//0x58 关闭模块
	
	
	#define SK60_CMD_CTR_SIZE			6 	//'O' 'C'	7
	#define SK60_CMD_STATUS_SIZE	15 //'S' 16
	#define SK60_CMD_MEASURE_SIZE	15 //'D' 'M' 'F' 16
	#define SK60_CMD_ERR_SIZE			8 //'D' 'M' 'F' 9
	
	#define SK60_T_MIN				-50.0
	#define SK60_T_MAX				90.0
	#define SK60_V_MIN				0.0
	#define SK60_V_MAX				4.0
	#define SK60_DIS_MIN					0.0
	#define SK60_DIS_MAX					60.0
	#define SK60_DISP_MAX					0.4
	#define SK60_DIS_YEWEI_INIT		0.4
	
	#define SK60_CMD_CTR_TIME			100
	#define SK60_CMD_STATUS_TIME	100
	#define SK60_CMD_MEASURE_TIME	100
	
/* Exported variables --------------------------------------------------------*/
extern __IO	uint8_t		sk60UsartStatus;
extern __IO sk60_t 		sk60Value;
//extern __IO float 		sk60Displacement;
extern __IO uint8_t		sk60InitFlag;
extern uint8_t 				sk60TestMode;
extern __IO float 		sk60BaseDistance;
extern __IO float 		sk60OffsetDistance;
extern uint16_t	sk60DispErrTimes;
extern void sk60Receive(void);
extern void sk60Task(uint8_t sk60Measurecmd);
extern void	sk60ValueStoreWithSensorData(uint32_t address,sk60_t value);

#if(KXYL_CTR>0)
//#define KXYL_SENSOR_NUM		4
//-100~200k 0~65535
#define KXYL_ADC_MAX	65535
#define KXYL_V_MIN		(-100.0)
#define KXYL_V_MAX		200.0
#define KXYL_COF_A		(300.0/65535)
#define KXYL_COF_B		(-100.0)
extern float kxylValue[KXYL_SENSOR_NUM];
extern float kxylParam[KXYL_SENSOR_NUM][2];
extern void kxylTimeDeal(void);
extern void kxylTask(void);
#endif
#endif 
/*****************************END OF FILE*****************************/

