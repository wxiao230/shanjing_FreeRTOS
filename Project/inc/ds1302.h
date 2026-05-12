#ifndef _DS1302_H
#define _DS1302_H
#include "stm32l1xx.h"

#define RTCOK						0x00
#define RTCERR					0x01

//GPS自动对时
#define RTC_CALIBRATION_DAY      8
#define RTC_CALIBRATION_HOUR    12

/*******************************************************************************************************
*
*    DS1302时钟数据结构体
*
********************************************************************************************************/
typedef struct{
//  uint32_t time;    //32压缩时间，时间戳，(B0-B5)秒、(B6-B11)分、(B12-B16)时、(B17-B21)日、(B22-B25)秒、(B26-B31)年
//  uint16_t baseYear; //基准时间年
  uint8_t  sec;       //秒（0-59）
  uint8_t  min;       //分（0-59）
  uint8_t  hour;      //时（0-23）
  uint8_t  date;      //日期（1-31）
	uint8_t  day;       //星期（1-7）
  uint8_t  month;     //月（1-12）
  uint8_t  year;      //年（0-99）
}DS1302_TIME;      //

//typedef union
//{ 
//	struct
//	{
//		uint8_t  sec;       //秒（0-59）
//		uint8_t  min;       //分（0-59）
//		uint8_t  hour;      //时（0-23）
//		uint8_t  date;      //日期（1-31）		
//		uint8_t  weakday;    //星期（1-7）		
//		uint8_t  month;     //月（1-12） 
//		uint8_t  year;      //年（0-99）
//	}rtc;
//	uint8_t buf[7];	
//}rtcTime_t;      //

extern DS1302_TIME  rtcTime;

extern DS1302_TIME  rtcTimeSample;
extern __IO uint32_t	curSecondTime ;

extern uint8_t rtcTimeBuf[7];
extern void SetDs1302TimeByGprs(uint8_t *buf);
extern void readDs1302TimeForRtc(uint8_t *buf);
extern void showDs1302Time(DS1302_TIME showTime);
extern void readDs1302Time(DS1302_TIME *readTime);
extern void SetDs1302Time(DS1302_TIME setTime);
extern void initDs1302(uint8_t initctr);
extern void rtcIoLowPower(void);
extern void ds1302BrustReadClock(DS1302_TIME *readTime);
extern void ds1302tobuf(DS1302_TIME time,uint8_t *buf);
extern uint8_t bufTods1302(DS1302_TIME *time,uint8_t *buf);
extern uint8_t rtcCheck(__IO DS1302_TIME Time);
extern uint8_t rtcCompare(__IO DS1302_TIME inTime,DS1302_TIME baseTime);
extern uint32_t queryDayToHour(uint8_t year,uint8_t mon,uint8_t date);
extern uint32_t getcurminute(void);

#endif

