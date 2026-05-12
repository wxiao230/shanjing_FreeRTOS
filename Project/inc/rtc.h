#ifndef RTC_H
#define    RTC_H
#include "stm32l1xx.h"
#include <stdint.h>

#define SEND_OK_FLAG      0x01
#define SEND_FAIL_FLAG    0x02
#define MAX_SEND_TRY_NUM  3

//extern __IO uint8_t gprsSendRFlag ;
//extern __IO uint8_t gprsSendTryNum ;
/*休眠RTC最小时间单位*/
#define SLEEP_BASE_TIME		((uint32_t)1)	  /*10    10秒为单位*//*1    1秒为单位*/
#define RTC_DAY_TIME				172800	//最长发送周期为1天半
//设备未激活状态下休眠时间
#define DEVICE_SLEEP_TIEM		172800//(24*2*60*60)
#define ALARM_POWER_STATUS_SLEEP_TIME	14400//(4*60*60)	//低电压报警状态下休眠时间
#define LOWER_POWER_STATUS_SLEEP_TIME	43200//(12*1*60*60)//低电压状态下休眠时间
#define DEFAULT_SLEEP_TIME						3600//(60*60)			//设备默认发送周期

extern uint32_t getSleepTimeAndAdj(uint32_t useTime,uint32_t period);

//extern RTC_TimeTypeDef RTC_TimeStructure;
//extern RTC_DateTypeDef RTC_DateStructure;

extern void rtcConfigurationJudge(void);
extern void rtc_NVIC_Configuration(void);
extern void Time_Show(void);
extern void wakeUpIniterrupt(void);
extern void rtcTimeSet(uint8_t *buf);
extern void wakeUPNvic(void);
extern void lowPowerEnter(uint32_t sleepTime);
extern uint32_t getCurrterTime(void);
extern void wakeUpTimeSet(uint32_t sleepTime);
extern uint32_t castTimeAdj(uint32_t startTime);

/*唤醒设备中断源及方式定义*/
#define WAKE_UP_FLAG_IDLE        0X00
#define RTC_WAKE_UP_FLAG         0x01
#define KEY_WAKE_UP_FLAG         0x02
#define ADXL345_WAKE_UP_FLAG     0x04

extern __IO uint8_t rtcWakeUpFlig;
#endif


