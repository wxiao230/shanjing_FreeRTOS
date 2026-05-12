/**
  ******************************************************************************
  * @file    Project/STM32L1xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    13-April-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
typedef union
{
	uint16_t i;
	struct
	{
		uint8_t valueL;
		uint8_t valueH;
	}bytes;	
}value16_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define GPRS_USE_CTR      	2	//GPRS���ܿ��� 0������gprs���ݣ�1����gprs���ݣ�����ʱ���ڷ���gprs��2ÿ�βɼ�������gprs
#define BOOT_CHECK_CTR			1	//boot�����ƣ�boot�汾�����0��������1�������ع鵽boot1
#define DEBUGSTATUS_CTR			1	//���Ƶ�������ֽڹ��� 0������Ϣ����򿪣�1�ϵ硢��λ��������򿪣����ߺ������ر�
#define QX_CTR							1	//��б�ɼ��������� 0����� 1 ������� 2 ֧���������
#define INTENSIVE_XL345_CTR	1	//���ܼ��ɼ����ܿ���
#define SK60_CTR						0	//�����๦�ܿ���


//QX_CTR=1&SK60_CTR=3 ����б�壬Ux3������ͨѶ���ơ�//221227
#if defined(STM32L1XX_MDP)
	#define KXYL_CTR					0	//��϶ˮѹ���ܿ���
	#define ACCE_CTR					1	//���ٶȹ��ܿ���
	#if(KXYL_CTR>0)
		#define USART_USE_CTR		0	//����ͨѶ���ܿ���  	0��CAN232���ܣ�1��ÿ�����㷢��GPRS,CAN��Ҫ��λ���У�2��ÿ�βɼ�������gprs��CAN����	
	#else
		#define USART_USE_CTR		0
	#endif
#else
	#define USART_USE_CTR			0
	#define KXYL_CTR					0
	#define ACCE_CTR					0
#endif
#define KXYL_SENSOR_NUM		4

#if(GPRS_USE_CTR>0)
	#define MQTT_USE_CTR			1	//MQTT���ܿ��� 0�ر�MQTT���� 1����MQTT����	
#else
	#define MQTT_USE_CTR			0
#endif

#define GPRS_MODE_SIM808		0
#define GPRS_MODE_SIM7100		1

#if(MQTT_USE_CTR>0)	
		#define GPRS_MODE_TYPE				GPRS_MODE_SIM7100
		#define GPRS_MULTI_LINK 			1 //��ͨ������ 0 һ��ͨ�� 1 ��ͨ��
	#if defined(STM32L1XX_MDP)		
		#define GPRS_SLEEP_CTR				1	//gprs���߿��� 0 ���� 1 ������
		#define	ALARM_CTR							1	//�Զ�Ԥ������
		#define	UPGRADE_FRIMWARE_CTR 	1	//mqtt�̼�����
		#define MQTT_TEST_SCH_CTR			0	//�Ĵ�����ģʽ ����ָ����Ӧ��0 ������Ӧ��1�Ȼظ���ִ�У�����ʱreboot�����λ���Ĵ�����Ϊ1����ʽ�̼�Ϊ0
		#define BEEP_ALARM_CTR				0	//�������ȹ���0 ��֧�ִ˹��� 1 ֧�ִ˹���  �Ĵ�����ʱΪ1����ʽ�̼�Ϊ0
		#define SECOND_MQTT_ADDR			1	//�ڶ�ƽ̨���õ�mqttd��ַ 0 ��֧�ֵڶ�ƽ̨���÷��͵�ַ��1 ֧�ֵڶ�ƽ̨�޸�һ����ַ��2 ֧�ֵڶ�ƽ̨�޸���·��ַ�����2·
	#else		
		#define GPRS_SLEEP_CTR				0	
		#define	ALARM_CTR							0	
		#define	UPGRADE_FRIMWARE_CTR 	0
		#define MQTT_TEST_SCH_CTR			0	
		#define BEEP_ALARM_CTR				0	
		#define SECOND_MQTT_ADDR			0
	#endif
#else	
	#if defined(STM32L1XX_MDP)
		#define GPRS_MODE_TYPE	GPRS_MODE_SIM7100	
	#else
		#define GPRS_MODE_TYPE	GPRS_MODE_SIM7100//GPRS_MODE_SIM808	
	#endif
	#define GPRS_MULTI_LINK				0 //��ͨ������ 0 һ��ͨ�� 1 ��ͨ��
	#define	ALARM_CTR							0	
	#define BEEP_ALARM_CTR				0 
	#define GPRS_SLEEP_CTR				0
	#define MQTT_TEST_SCH_CTR			0
	#define	UPGRADE_FRIMWARE_CTR 	0	
	#define SECOND_MQTT_ADDR			0
#endif

#define ALARM_POWERV_HIGH	6.0
#define ALARM_POWERV_LOW	3.0

#define devUSART_RUN			USART2_Config
#define devUsartStatus		usart2Status

#define DEV_TYPE_NONE					0x00
#define DEV_TYPE_SOIL					0x01
#define DEV_TYPE_WEATHER			0x02
#define DEV_TYPE_WEATHERSOIL	0x03
#define DEV_TYPE_TQGPRS				0x04
#define DEV_TYPE_TQCAN				0x05
#define DEV_TYPE_TQCAN_GPRS		0x06
#define DEV_TYPE_RT						0x07
#define DEV_TYPE_RTGPRS				0x08
#define DEV_TYPE_SJ						0x07
#define DEV_TYPE_QJGPRS				0x08
#if(GPRS_USE_CTR>0)
		#define DEV_TYPE_INIT			DEV_TYPE_QJGPRS
#else	
		#define DEV_TYPE_INIT			DEV_TYPE_SJ
#endif

#define DEBUG_STATUS_NONE			0x00
#define DEBUG_STATUS_DONE			0x01
#define DEVSLEEP_NONE					0x00
#define DEVSLEEP_DONE					0x01

#define DEVICE_OK       0x01
#define DEVICE_UNPLUG   0x03
#define DEVICE_UNALARM  0x01
#define DEVICE_ALARM    0x02
#define DEVICE_VIBRATION 0x04

//�豸����״̬
#define MCURUNSTATUS_Init				0x00
#define MCURUNSTATUS_Start			0x01     
#define MCURUNSTATUS_Sample			0x02 
#define MCURUNSTATUS_Clock			0x03
#define MCURUNSTATUS_Save				0x04 
#define MCURUNSTATUS_Send				0x05
#define MCURUNSTATUS_Sleep			0x06 
#define MCURUNSTATUS_SampleCal	0x07
#define MCURUNSTATUS_mqtt_init							0x08
#define MCURUNSTATUS_mqtt_portOpen					0x09
#define MCURUNSTATUS_mqtt_connect						0x0A
#define MCURUNSTATUS_mqtt_subscribe					0x0B
#define MCURUNSTATUS_mqtt_publishdata				0x0C
#define MCURUNSTATUS_mqtt_publishstatus			0x0D
#define MCURUNSTATUS_mqtt_receive						0x0E
#define MCURUNSTATUS_mqtt_end								0x0F
#define MCURUNSTATUS_mqtt_unsubscribe				0x10
#define MCURUNSTATUS_mqtt_ping							0x11

#define RST_STATUS_KEY		2
#define RST_STATUS_UPDATA	3
#define RST_STATUS_BOOT		4
#define RST_STATUS_RTC		5
#define RST_STATUS_SWRST	6
#define RST_STATUS_NMI		7
#define RST_STATUS_Hard		8
#define RST_STATUS_Mem		9
#define RST_STATUS_GPRSERR	10
#define RST_STATUS_MQTT		11

#define FAULT_TIME_MAX	(uint16_t)(10000)

#define SEND_STATUS_NONE		0x00
#define SEND_STATUS_NORMAL	0x01
#define SEND_STATUS_KEY			0x02
#define SEND_STATUS_USART		0x03
#define SEND_STATUS_XL345		0x04
#define SEND_STATUS_INIT		0x05
#define SEND_STATUS_PLUS		0x06
#define SEND_STATUS_ALARM		0x07

/* Exported functions ------------------------------------------------------- */
extern __IO uint8_t	rstStatus	;
extern __IO uint16_t 	gpsInterValTimer;
extern __IO uint8_t		gpsRtcSetFlag	;
extern __IO uint32_t	curDevPeriod;
extern __IO uint32_t	curDevSendPeriod;
extern __IO uint8_t		mcuPowerStatus;
extern __IO uint8_t		mcuRunStatus;
extern __IO uint8_t	sampleTestFlag	;
extern uint8_t  sendDataCounter;
extern uint8_t debugStatus ;
extern __IO uint32_t usart1OverrunCount;
extern __IO uint32_t usart2OverrunCount;
extern __IO uint32_t usart3OverrunCount;
extern __IO uint8_t 	sampleTimesContinus;
extern __IO uint8_t  tiltType;
extern uint8_t 	fireWareId;
extern uint8_t   curPeriodType;

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
extern void sysSoftReset(void);
extern void reboot1task(void);
extern void gpsIntervalInit(void);

/* FreeRTOS declarations */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern TaskHandle_t xSensorTaskHandle;
extern TaskHandle_t xCommTaskHandle;
extern TaskHandle_t xStorageTaskHandle;
extern TaskHandle_t xMonitorTaskHandle;
extern TaskHandle_t xPowerTaskHandle;
extern QueueHandle_t xSensorQueue;
extern QueueHandle_t xCommQueue;
extern SemaphoreHandle_t xSPIFlashMutex;
extern SemaphoreHandle_t xUSART1Mutex;
extern SemaphoreHandle_t xUSART3Mutex;
extern SemaphoreHandle_t xADXL345Semaphore;
extern SemaphoreHandle_t xSensorBufMutex;

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
