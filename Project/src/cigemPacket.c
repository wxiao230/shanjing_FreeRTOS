/**
  ******************************************************************************
  * @file    Project/user/cigemPacket.c
  * @author  insentek wxh
  * @version V1.0.0
  * @date    20-July-2020
  * @brief   
  ******************************************************************************   
  */  

/* Includes ------------------------------------------------------------------*/
#include "cigemPacket.h"
#include "mqttTransport.h"
//#include "mqttClient.h"
extern __IO uint32_t curDevPeriod;
extern __IO uint32_t curDevSendPeriod;
//#include "devpacket.h"
//#include "pressure.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"
#include "mode.h"
#include "rtc.h"
#if(USART_USE_CTR>0)
	#include "rs485.h"
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UTC_TIME_CTR  1 //0 ����ʱ�� 1 ��׼UTCʱ��
#define CIGE_SENSOR_DATA_DEBUG	1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	cigemsensortime_t		cigemsensortime;
	cigemsensorrange_t	cigemsensorlimit[SENSORID_INDEX_NUM];

	uint8_t	cigemcmd	= 0;
	uint8_t cigemcmdflag[MQTT_MAX_MULTI_CONNECTION_SIZE] = {0,0,0,0};//uint8_t cigemcmdflag = CIGEM_CMD_FLAG_none;
	uint8_t cigemapikey[CIGEM_KEY_LEN];
	uint8_t cigemmsgid[MQTT_MAX_MULTI_CONNECTION_SIZE][CIGEM_MSGID_LEN];
	uint8_t cigemsettimeflag 				=	CIGEM_CMD_FLAG_none;
	uint8_t cigemrebootflag					=	CIGEM_CMD_FLAG_none;
	uint8_t cigemsampleflag					=	CIGEM_CMD_FLAG_none;
	uint8_t cigemsetsensortimeflag	=	CIGEM_CMD_FLAG_none;
	uint8_t cigemsensoridindex			=	SENSORID_INDEX_NUM;
	uint8_t cigemsetsensorrangeflag	=	CIGEM_CMD_FLAG_none;
	char 		cigemworkmode						= WORKMODE_RUN;
	uint8_t	cigemresultflag					=	CIGEM_CMD_FLAG_none;
	cigemwarning_t	cigemwarning		=	cigemwaring_initializer;
	uint8_t	cigemlinknumSave		=	0;

	__IO uint8_t	alarmSource	=	ALARM_SOURCE_NONE;
	__IO uint8_t	alarmLevel	=	WARNING_LEVEL_NORMAL;

uint32_t	cigemEarlyWarningTimes	=	0;
uint8_t cigemcmdLast =0;
#if(SECOND_MQTT_ADDR>0)
	uint8_t serverControlNo = 0;
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
	void cigemperiodrunmodeInit(void)
	{
		cigemworkmode				=	WORKMODE_RUN;
		cigemwarning.time		=	0;
		cigemwarning.level	=	1;
		cigemEarlyWarningTimes = getcurminute();
		alarmSource =	ALARM_SOURCE_NONE;
		alarmLevel	=	WARNING_LEVEL_NORMAL;
	#if((ALARM_CTR>0)&&(BEEP_ALARM_CTR>0))
		alarmTimes 	= 0;		
	#endif
	}
	void cigemperiodTypeAdj(__IO uint8_t *periodType,__IO uint32_t *period,uint16_t *density)
	{
		uint32_t curminute = 0;
		
		if((cigemwarning.level != 0)&&(cigemwarning.level != WARNING_LEVEL_NORMAL))
		{
			curminute = getcurminute();
			if(curminute>=cigemEarlyWarningTimes)
			{
				if((curminute - cigemEarlyWarningTimes)>=cigemwarning.time)cigemwarning.level = WARNING_LEVEL_NORMAL;
			}
			else
			{			
				cigemwarning.level = WARNING_LEVEL_NORMAL;
			}
			*periodType	=	NORMAL_PERIOD_TYPE;
	
			switch(cigemwarning.level)
			{
			#if 1
				case WARNING_LEVEL_RED:
					*period			=	devPar.plus_intv;				//WORKMODE_RED_PERIOD;	//20210120
					*density		= WORKMODE_PLUS_DENSITY;	//WORKMODE_RED_DENSITY;
					break;
				case WARNING_LEVEL_ORANGE:				
					*period			=	devPar.plus_intv;				//WORKMODE_ORANGE_PERIOD;	
					*density		= WORKMODE_PLUS_DENSITY*2;//WORKMODE_ORANGE_DENSITY;
					break;
				case WARNING_LEVEL_YELLOW:				
					*period			=	devPar.plus_intv;				//WORKMODE_YELLOW_PERIOD;	
					*density		= WORKMODE_PLUS_DENSITY*4;//WORKMODE_YELLOW_DENSITY;
					break;
				case WARNING_LEVEL_BLUE:
					*period			=	devPar.plus_intv;				//WORKMODE_BLUE_PERIOD;	
					*density		= WORKMODE_PLUS_DENSITY*6;//WORKMODE_BLUE_DENSITY;
					break;
			#else 	//20210119
				case WARNING_LEVEL_RED:
				case WARNING_LEVEL_ORANGE:	
				case WARNING_LEVEL_YELLOW:		
				case WARNING_LEVEL_BLUE:
					*period			=	devPar.plus_intv;
					*density		= WORKMODE_PLUS_DENSITY;
					break;
			#endif
				case WARNING_LEVEL_NORMAL:				
				default:			
					cigemwarning.time		=	0;
					cigemwarning.level	=	0;			
					*periodType		=	configValue.devPeriodType;
					*period				=	configValue.devPeriod;		
					*density			= configValue.transmissionDensity;
					break;				
			}		
		}
		else if(cigemworkmode == WORKMODE_PLUS)
		{
			*periodType	=	NORMAL_PERIOD_TYPE;
			*period			=	devPar.plus_intv;
			*density		= WORKMODE_PLUS_DENSITY;
			sendDataCounter = SEND_STATUS_PLUS;	
		}
		else if(cigemworkmode == WORKMODE_LOWPOWER)
		{
			*periodType	= NORMAL_PERIOD_TYPE;	
			*period			=	configValue.devPeriod;
			*density		= WORKMODE_LOWPOWER_DENSITY;						
		}	
	}
	void cigemworkmodeadj(void)
	{
//		if(cigemworkmode == WORKMODE_PLUS)
//		{
//			configValue.devPeriodType	=	NORMAL_PERIOD_TYPE;
//			configValue.devPeriod		=	cigemsensortime.plus;
//			serverConfig.devDensity	= WORKMODE_PLUS_DENSITY;			
//		}
//		else if(cigemworkmode == WORKMODE_LOWPOWER)
//		{
//			configValue.devPeriodType	= NORMAL_PERIOD_TYPE;	
//			configValue.devPeriod		=	cigemsensortime.upload;	
//			serverConfig.devDensity	= WORKMODE_LOWPOWER_DENSITY;						
//		}
//		else
//		{
//			configValue.devPeriod		=	cigemsensortime.sample;	
//			serverConfig.devDensity	= configValue.transmissionDensity;			
//		}
//		serverConfig.perSetFlag	=	CHANGE_FLAG;
	}

void cigemSensorRangeInit(void)
{
	cigemsensorlimit[SENSORID_INDEX_QJ].lower 		= -90.0;
	cigemsensorlimit[SENSORID_INDEX_QJ].upper 		= 90.0;
	cigemsensorlimit[SENSORID_INDEX_QJ].threshold = 60;
	
	cigemsensorlimit[SENSORID_INDEX_HS].lower 		= 0.0;
	cigemsensorlimit[SENSORID_INDEX_HS].upper 		= 100.0;
	cigemsensorlimit[SENSORID_INDEX_HS].threshold = 60;
	
	cigemsensorlimit[SENSORID_INDEX_TW].lower 		= -40.0;
	cigemsensorlimit[SENSORID_INDEX_TW].upper 		= 85.0;
	cigemsensorlimit[SENSORID_INDEX_TW].threshold = 50;
#if(ACCE_CTR>0)
	cigemsensorlimit[SENSORID_INDEX_ACCE].lower 		= -16.0;
	cigemsensorlimit[SENSORID_INDEX_ACCE].upper 		= 16.0;
	cigemsensorlimit[SENSORID_INDEX_ACCE].threshold = 2.0;
#endif
#if(SK60_CTR>1)
	cigemsensorlimit[SENSORID_INDEX_LF].lower 		= -600.0;
	cigemsensorlimit[SENSORID_INDEX_LF].upper 		= 60000.0;
	cigemsensorlimit[SENSORID_INDEX_LF].threshold = 600.0;
#endif
#if(KXYL_CTR>1)
	cigemsensorlimit[SENSORID_INDEX_KXYL].lower 		= -1000.0;
	cigemsensorlimit[SENSORID_INDEX_KXYL].upper 		= 1000.0;
	cigemsensorlimit[SENSORID_INDEX_KXYL].threshold = 200.0;
#endif
}
void cigemConfigInit(void)
{
	if(cigemworkmode < WORKMODE_RUN)cigemworkmode = WORKMODE_RUN;

	if(devPar.plus_intv>=CIGEM_TIME_PLUS_MIN)
	{
		devPar.plus_intv /= CIGEM_TIME_MIN_INIT;
		devPar.plus_intv *= CIGEM_TIME_MIN_INIT;
		cigemsensortime.plus = devPar.plus_intv;
	}
	else
	{
		cigemsensortime.plus = CIGEM_TIME_PLUS_INIT;
	}
	cigemsensortime.sample = curDevPeriod;//configValue.devPeriod;
	cigemsensortime.upload = curDevPeriod*serverConfig.devDensity;//configValue.devPeriod*configValue.transmissionDensity;
	curDevSendPeriod = cigemsensortime.upload;
}

void cigemSensorTimeChange(void)
{		
	uint32_t uploadPeriod = cigemsensortime.upload;

	if(cigemsensortime.sample > 0)
	{
		configValue.transmissionDensity = uploadPeriod / cigemsensortime.sample;
		if(uploadPeriod % cigemsensortime.sample > 0) configValue.transmissionDensity += 1;
	}
	else
	{
		configValue.transmissionDensity = 1;
	}

	cigemsensortime.sample /= CIGEM_TIME_MIN_INIT;
	cigemsensortime.sample *= CIGEM_TIME_MIN_INIT;

	configValue.devPeriod = cigemsensortime.sample;
	configValue.devPeriodType = NORMAL_PERIOD_TYPE;
	curDevPeriod = cigemsensortime.sample;

	uploadPeriod /= CIGEM_TIME_MIN_INIT;
	uploadPeriod *= CIGEM_TIME_MIN_INIT;
	curDevSendPeriod = uploadPeriod;

	cigemsensortime.upload = cigemsensortime.sample * configValue.transmissionDensity;

	cigemsensortime.plus /= CIGEM_TIME_MIN_INIT;
	cigemsensortime.plus *= CIGEM_TIME_MIN_INIT;
	devPar.plus_intv = cigemsensortime.plus;
	serverConfig.perSetFlag = CHANGE_FLAG;
}
uint16_t cigemPacketSensorData(char *bufOut,char *clientId)
{
	uint16_t datat = 0x00;	
	uint16_t dataOutLength= 0x00;
	uint8_t 	wateri;
	value16_t jsonlen;	
	char timedata[50];
	uint8_t timelen=0;
	DS1302_TIME time = sensorValue.time;
#if(UTC_TIME_CTR>0)
	utcTimeAdj(&time,0,sensorValue.time,devPar.timeZone);
#endif
	/*ʱ���T*/
	datat =	sprintf(timedata+dataOutLength, "\"%d",(time.year+2000));
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "-%02d",time.month);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "-%02dT",time.date);
	dataOutLength	+= datat;	
	datat =	sprintf(timedata+dataOutLength, "%02d:",time.hour);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "%02d:",time.min);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "%02d.000Z\":",time.sec);
	dataOutLength	+= datat;
	timelen	=	dataOutLength;
	bufOut[0]	=	DATA_TYPE_SENSOR;//dataType 2
//	bufOut[1]	=	jsonlen.bytes.valueH;
//	bufOut[2]	=	jsonlen.bytes.valueL;
	
	dataOutLength	=	3;
	datat =	sprintf(bufOut+dataOutLength,"{\"%s\":{",clientId);//json start
	dataOutLength	+= datat;
	for(wateri = 0x00;wateri < devSenNum;wateri++)//devSenNum
	{
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":",SENSORID_HS,wateri+1);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","{");
			dataOutLength	+= datat;
			memcpy(bufOut+dataOutLength,timedata,timelen);
			dataOutLength	+= timelen;
		}
		//sensorValue.wat[wateri].f	=	sensorValue.soilt[wateri].f*0.5;//201218
		datat =	sprintf(bufOut+dataOutLength,"%.2f",sensorValue.wat[wateri].f);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","}");
			dataOutLength	+= datat;
		}
		datat =	sprintf(bufOut+dataOutLength,"%s",",");
		dataOutLength	+= datat;
	}
	for(wateri = 0x00;wateri < devSenNum;wateri++)//devSenNum	
	{			
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":",SENSORID_TW,wateri+1);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","{");
			dataOutLength	+= datat;
			memcpy(bufOut+dataOutLength,timedata,timelen);
			dataOutLength	+= timelen;
		}
		datat =	sprintf(bufOut+dataOutLength,"%.2f",sensorValue.soilt[wateri].f);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","}");
			dataOutLength	+= datat;
		}	
		if(wateri<(devSenNum-1))
		{
			datat =	sprintf(bufOut+dataOutLength,"%s",",");
			dataOutLength	+= datat;
		}
	}
#if(QX_CTR>0)
	if(devPar.qxkey != FUNCITONG_TURN_OFF)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":",SENSORID_QJ);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","{");
			dataOutLength	+= datat;
			memcpy(bufOut+dataOutLength,timedata,timelen);
			dataOutLength	+= timelen;
		}
		if(devPar.mqttDataType == MQTT_DATA_TYPE_STRING)
		{
			datat =	sprintf(bufOut+dataOutLength,"\"%.3f,%.3f,%.3f,%.3f,%.3f\"",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f,sensorValue.angle.f,sensorValue.dipYaw.f);//�Ĵ�ƽֻ̨֧�ִ˸�ʽ
		}
		else if(devPar.mqttDataType == MQTT_DATA_TYPE_TREND)
		{
			//���Ժƽ̨ͬʱ֧�ִ˸�ʽ	
			datat =	sprintf(bufOut+dataOutLength,"{\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f,",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);	
			dataOutLength	+= datat;
			datat =	sprintf(bufOut+dataOutLength,"\"angle\":%.3f,\"trend\":%.3f}",sensorValue.angle.f,sensorValue.dipYaw.f);
		}
		else 
		{
			//���Ժƽ̨ͬʱ֧�ִ˸�ʽ	
			datat =	sprintf(bufOut+dataOutLength,"{\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f,",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);	
			dataOutLength	+= datat;
			datat =	sprintf(bufOut+dataOutLength,"\"angle\":%.3f,\"AZI\":%.3f}",sensorValue.angle.f,sensorValue.dipYaw.f);
		}
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","}");
			dataOutLength	+= datat;
		}
	}	
#endif
#if(ACCE_CTR>0)
	if(devPar.accelerometerkey == FUNCITONG_TURN_ON)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":",SENSORID_ACCE);
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","{");
			dataOutLength	+= datat;
			memcpy(bufOut+dataOutLength,timedata,timelen);
			dataOutLength	+= timelen;
		}
		if(devPar.mqttDataType == MQTT_DATA_TYPE_STRING)
		{
			datat =	sprintf(bufOut+dataOutLength,"\"%.3f,%.3f,%.3f\"",sensorValue.ax.f*G_UNIT,sensorValue.ay.f*G_UNIT,sensorValue.az.f*G_UNIT);//�Ĵ�ƽֻ̨֧�ִ˸�ʽ
		}	
		else 
		{
			//���Ժƽ̨ͬʱ֧�ִ˸�ʽ	
			datat =	sprintf(bufOut+dataOutLength,"{\"gX\":%.3f,\"gY\":%.3f,\"gZ\":%.3f}",sensorValue.ax.f*G_UNIT,sensorValue.ay.f*G_UNIT,sensorValue.az.f*G_UNIT);			
		}
		dataOutLength	+= datat;
		if(bufOut[0] == DATA_TYPE_2)
		{
			datat =	sprintf(bufOut+dataOutLength,"%s","}");
			dataOutLength	+= datat;
		}
	}
#endif
#if(SK60_CTR>1)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	datat =	sprintf(bufOut+dataOutLength,",\"%s\":",SENSORID_LF);
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","{");
		dataOutLength	+= datat;
		memcpy(bufOut+dataOutLength,timedata,timelen);
		dataOutLength	+= timelen;
	}
	datat =	sprintf(bufOut+dataOutLength,"%.0f",sensorValue.sk60disp.f*1000);
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","}");
		dataOutLength	+= datat;
	}
}
#endif
#if(KXYL_CTR>1)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":",SENSORID_KXYL,1);
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","{");
		dataOutLength	+= datat;
		memcpy(bufOut+dataOutLength,timedata,timelen);
		dataOutLength	+= timelen;
	}
//	datat =	sprintf(bufOut+dataOutLength,"{\"temp\":0,\"value\":%.2f}",sensorValue.kxylv1.f);	
	datat =	sprintf(bufOut+dataOutLength,"\"0,%.2f\"",sensorValue.kxylv1.f);	
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","}");
		dataOutLength	+= datat;
	}
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":",SENSORID_KXYL,2);
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","{");
		dataOutLength	+= datat;
		memcpy(bufOut+dataOutLength,timedata,timelen);
		dataOutLength	+= timelen;
	}
//	datat =	sprintf(bufOut+dataOutLength,"{\"temp\":0,\"value\":%.2f}",sensorValue.kxylv2.f);	
	datat =	sprintf(bufOut+dataOutLength,"\"0,%.2f\"",sensorValue.kxylv2.f);	
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","}");
		dataOutLength	+= datat;
	}
}
#endif
	datat =	sprintf(bufOut+dataOutLength,"%s","}}");//json end
	dataOutLength	+= datat;
	
	jsonlen.i = dataOutLength-3;
	bufOut[1]	=	jsonlen.bytes.valueH;
	bufOut[2]	=	jsonlen.bytes.valueL;
	
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nsensordatalen:%d,jsonlen:%d\r\n",dataOutLength,jsonlen.i);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;			   
}
extern	float adsexterValue;
#include "ds18b20.h"
float batdumpCaculate1(__IO float btv)
{//dump=100*(1-(3.4-V)/0.55)
	float dump=100.0;
	if(btv>3.4){dump=100.0;}
	else if(btv>2.85)
	{
		 dump=100*(1-(3.4-btv)/0.55);
	}
	else{dump = 0;}
	return dump;
}
float batdumpCaculate(__IO float btv)//20220518
{	//3.4_100 3.3_94 3.2_40 3.15_20	3.12_8	2.95_3	2.83_0
	float dump=100.0;
	if(btv>3.4){dump=100.0;}
	else if(btv>3.3)	//94+(btv-3.3)*60
	{
		 dump=94+(btv-3.3)*60;
	}
	else if(btv>3.2)	//40+(btv-3.2)*540
	{
		 dump=40+(btv-3.2)*540;
	}
	else if(btv>3.12)	//8+(btv-3.12)*400
	{
		 dump=8+(btv-3.12)*400;
	}
	else if(btv>2.83)	//(btv-2.83)*27
	{
		 dump=(btv-2.83)*27;
	}
	else{dump = 0;}
	return dump;
}	
uint16_t cigemPacketDevstatus(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;
	int valueStatus = 0;
	uint8_t 	wateri;
	float batdumpV=0.0;
	batdumpV = batdumpCaculate(batStatus);
	if(DATA_TYPE_DEVSTATUS == DATA_TYPE_1)
	{
		datat =	sprintf(bufOut+dataOutLength,"{\"ext_power_volt\":%.2f,",batStatus);
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"\"solar_volt\":%.2f,",adsexterValue);
		dataOutLength	+= datat;	
		datat =	sprintf(bufOut+dataOutLength,"\"battery_dump_energy\":%.1f,",batdumpV);
		dataOutLength	+= datat;	
		datat =	sprintf(bufOut+dataOutLength,"\"temp\":%.2f,",result[0].rf);
		dataOutLength	+= datat;
	}
	else
	{
		datat =	sprintf(bufOut+dataOutLength,"{\"ext_power_volt\":%.2f,",sensorValue.batV.f);
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"\"solar_volt\":%.2f,",sensorValue.inputV.f);
		dataOutLength	+= datat;	
		datat =	sprintf(bufOut+dataOutLength,"\"battery_dump_energy\":%.1f,",batdumpV);
		dataOutLength	+= datat;	
		datat =	sprintf(bufOut+dataOutLength,"\"temp\":%.2f,",sensorValue.t.f);
		dataOutLength	+= datat;
	}
	datat =	sprintf(bufOut+dataOutLength,"\"humidity\":%.1f,",0.0);
	dataOutLength	+= datat;	
	
	datat =	sprintf(bufOut+dataOutLength,"\"lon\":\"%lf\",",devLongitude);
	dataOutLength	+= datat;	
	datat =	sprintf(bufOut+dataOutLength,"\"lat\":\"%lf\",",devLatitude);
	dataOutLength	+= datat;	
			
	datat =	sprintf(bufOut+dataOutLength,"\"signal_4g\":%d,",rssiValue);
	dataOutLength	+= datat;	
//	datat =	sprintf(bufOut+dataOutLength,"\"signal_NB\":%d,",0);////
//	dataOutLength	+= datat;	
//	datat =	sprintf(bufOut+dataOutLength,"\"signal_bd\":%d,",0);////
//	dataOutLength	+= datat;	
	datat =	sprintf(bufOut+dataOutLength,"\"sw_version\":\"");
	dataOutLength	+= datat;
	memcpy(bufOut+dataOutLength,(char*)firwMare,strlen((char*)firwMare));
	dataOutLength	+= 8;
	datat =	sprintf(bufOut+dataOutLength,"\",\"sensor_state\":{");
	dataOutLength	+= datat;
		
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		if(sensorValue.fh[wateri].f == 0){valueStatus	=	CHANNEL_DATA_ERR;}		
		else {valueStatus	=	CHANNEL_ERROR_START;}
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":%d,",SENSORID_HS,wateri+1,valueStatus);
		dataOutLength	+= datat;	
	}
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{		
		if((sensorValue.soilt[wateri].f < -40)||(sensorValue.soilt[wateri].f > 85)){valueStatus	=	CHANNEL_DATA_ERR;}
		else{valueStatus	=	CHANNEL_ERROR_START;}
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":%d",SENSORID_TW,wateri+1,valueStatus);
		dataOutLength	+= datat;	
		if(wateri<(devSenNum-1))
		{
			datat =	sprintf(bufOut+dataOutLength,"%s",",");
			dataOutLength	+= datat;
		}
	}
#if(QX_CTR>0)
	if(devPar.qxkey != FUNCITONG_TURN_OFF)
	{
		if((sensorValue.dipX.f == 0)&&(sensorValue.dipY.f == 0)){valueStatus	=	CHANNEL_DATA_ERR;}
		else{valueStatus	=	CHANNEL_ERROR_START;}
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":%d",SENSORID_QJ,valueStatus);
		dataOutLength	+= datat;	
	}
#endif
//	datat =	sprintf(bufOut+dataOutLength,"\"%s\":{\"X\":%d,\"Y\":%d,\"Z\":%d,",SENSORID_QJ,valueStatus,valueStatus,valueStatus);
//	dataOutLength	+= datat;	
//	datat =	sprintf(bufOut+dataOutLength,"\"angle\":%d,\"%s\":%d}",valueStatus,SENSORID_QJ_DIP,valueStatus);
//	dataOutLength	+= datat;
#if(ACCE_CTR>0)
	if(devPar.accelerometerkey == FUNCITONG_TURN_ON)
	{
		if((sensorValue.ax.f == 0)&&(sensorValue.ay.f == 0)){valueStatus	=	CHANNEL_DATA_ERR;}
		else{valueStatus	=	CHANNEL_ERROR_START;}
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":%d",SENSORID_ACCE,valueStatus);
		dataOutLength	+= datat;	
	}
#endif
#if(SK60_CTR>1)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	if(sensorValue.sk60dis.f == 0){valueStatus	=	CHANNEL_DATA_ERR;}
	else{valueStatus	=	CHANNEL_ERROR_START;}
	datat =	sprintf(bufOut+dataOutLength,",\"%s\":%d",SENSORID_LF,valueStatus);
	dataOutLength	+= datat;	
}
#endif
#if(KXYL_CTR>1)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	if(sensorValue.kxylv1.f == 0){valueStatus	=	CHANNEL_DATA_ERR;}
	else{valueStatus	=	CHANNEL_ERROR_START;}
//	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":{\"temp\":%d,\"value\":%d}",SENSORID_KXYL,1,CHANNEL_NO_DATA,valueStatus);
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":%d",SENSORID_KXYL,1,valueStatus);
	dataOutLength	+= datat;	
	if(sensorValue.kxylv2.f == 0){valueStatus	=	CHANNEL_DATA_ERR;}
	else{valueStatus	=	CHANNEL_ERROR_START;}
//	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":{\"temp\":%d,\"value\":%d}",SENSORID_KXYL,2,CHANNEL_NO_DATA,valueStatus);
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":%d",SENSORID_KXYL,2,valueStatus);
	dataOutLength	+= datat;
}
#endif	
	datat =	sprintf(bufOut+dataOutLength,"%s","}}");
	dataOutLength	+= datat;
		
#if(CIGE_SENSOR_DATA_DEBUG>1)	
	printf("\r\ndevstatuslen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif		
	return dataOutLength;			   
}
uint16_t cigemPacketDevstatusPub(char *bufOut,char *clientId)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;
	value16_t jsonlen;
char timedata[50];
	uint8_t timelen=0;	
	DS1302_TIME time = sensorValue.time;
#if(UTC_TIME_CTR>0)
	utcTimeAdj(&time,0,sensorValue.time,devPar.timeZone);
#endif

	/*ʱ���T*/
	datat =	sprintf(timedata+dataOutLength, "\"%d",(time.year+2000));
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "-%02d",time.month);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "-%02dT",time.date);
	dataOutLength	+= datat;	
	datat =	sprintf(timedata+dataOutLength, "%02d:",time.hour);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "%02d:",time.min);
	dataOutLength	+= datat;
	datat =	sprintf(timedata+dataOutLength, "%02d.000Z\":",time.sec);
	dataOutLength	+= datat;
	timelen	=	dataOutLength;
	
	dataOutLength	=	0;

	bufOut[0]	=	DATA_TYPE_DEVSTATUS;//dataType 1

//	bufOut[1]	=	jsonlen.bytes.valueH;
//	bufOut[2]	=	jsonlen.bytes.valueL;

	dataOutLength	=	3;
	datat =	sprintf(bufOut+dataOutLength,"{\"%s\":{",clientId);//json start
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"\"%s\":","S1_ZT_1");//S1_ZT_1
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","{");//S1_ZT_1
		dataOutLength	+= datat;
		memcpy(bufOut+dataOutLength,timedata,timelen);
		dataOutLength	+= timelen;
	}
	datat	=	cigemPacketDevstatus(bufOut+dataOutLength);
	dataOutLength	+= datat;
	if(bufOut[0] == DATA_TYPE_2)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","}");//json end
	}
	datat =	sprintf(bufOut+dataOutLength,"%s","}}");//json end
	dataOutLength	+= datat;
	
	jsonlen.i = dataOutLength-3;
	bufOut[1]	=	jsonlen.bytes.valueH;
	bufOut[2]	=	jsonlen.bytes.valueL;
	
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\npubdevstatuslen:%d,jsonlen:%d\r\n",dataOutLength,jsonlen.i);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckapikey(char *bufOut,uint8_t *key,uint8_t *msgid)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//&apikey=6767377eb9e5ee17385e&msgid=1N9V4h_hR
	datat =	sprintf(bufOut+dataOutLength,"%s","&apikey=");	
	dataOutLength	+= datat;
	memcpy(bufOut+dataOutLength,key+1,key[0]);	
	dataOutLength	+= key[0];
	datat =	sprintf(bufOut+dataOutLength,"%s","&msgid=");	
	dataOutLength	+= datat;
	memcpy(bufOut+dataOutLength,msgid+1,msgid[0]);	
	dataOutLength	+= msgid[0];
#if(CIGE_SENSOR_DATA_DEBUG>1)	
	printf("\r\nAckapikeylen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckReqtime(char *bufOut,DS1302_TIME timein)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	DS1302_TIME time = timein;
#if(UTC_TIME_CTR>0)
	utcTimeAdj(&time,0,timein,devPar.timeZone);
#endif
	//receive:$cmd=reqtime
	//ack:$cmd=reqtime&time=YYYY-MM-DD hh:mm:ss
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=reqtime&time=");//json start
	dataOutLength	+= datat;
	
	datat =	sprintf(bufOut+dataOutLength, "%d",(time.year+2000));
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength, "-%02d",time.month);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength, "-%02d ",time.date);
	dataOutLength	+= datat;	
	datat =	sprintf(bufOut+dataOutLength, "%02d:",time.hour);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength, "%02d:",time.min);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength, "%02d",time.sec);
	dataOutLength	+= datat;
				
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckReqtimelen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}

uint16_t cigemPacketAckResult(char *bufOut,char *cmd,uint8_t flag)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//$cmd=settime&server=ntpserver 
	//$cmd=reboot
	//$cmd=setsensortime&sensor_id=value&sample_intv=value&upload_intv=value&plus_intv=value
	//$cmd=setsensorattr&sensor_id=value&threshold=value&upper_limit=value&lower_limit=value
	//$cmd=setworkmode&mode=value
	//$cmd=meteorologicalearlywarning&level=value&effective_time=value&lon_range=value&lat_range=value
	
	//ack:$cmd=settime&result=succ or $cmd=settime&result=fail
	if(flag == ERROR)
	{
		datat =	sprintf(bufOut+dataOutLength,"$cmd=%s&result=fail",cmd);
	}
	else
	{
		datat =	sprintf(bufOut+dataOutLength,"$cmd=%s&result=succ",cmd);
	}
	dataOutLength	+= datat;
		
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckResultlen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}

uint16_t cigemPacketAckGetstatus(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=getstatus	
	//ack:$cmd=getstatus&state={"ext_power_volt":24.04,"temp":42.00,"4g_signal_4g":27.0,"sw_version":"1.0.1",....}
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getstatus&state=");	
	dataOutLength	+= datat;
	datat = cigemPacketDevstatus(bufOut+dataOutLength);
	dataOutLength	+= datat;
		
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetstatuslen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}

uint16_t cigemPacketAckGetsensorID(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	uint8_t wateri;
	//receive:$cmd=getsensorID	
	//ack:$cmd=getsensorID&sensor_id=value
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getsensorID&sensor_id={");	
	dataOutLength	+= datat;
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\",",SENSORID_HS,wateri+1);
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\"",SENSORID_TW,wateri+1);
		dataOutLength	+= datat;
		if(wateri<(devSenNum-1))
		{
			datat =	sprintf(bufOut+dataOutLength,"%s",",");
			dataOutLength	+= datat;
		}
	}
#if(QX_CTR>0)
	if(devPar.qxkey != FUNCITONG_TURN_OFF)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\"",SENSORID_QJ);
		dataOutLength	+= datat;
	}
#endif
#if(ACCE_CTR>0)
	if(devPar.accelerometerkey == FUNCITONG_TURN_ON)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\"",SENSORID_ACCE);
		dataOutLength	+= datat;
	}
#endif
#if(SK60_CTR>1)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	datat =	sprintf(bufOut+dataOutLength,",\"%s\"",SENSORID_LF);
	dataOutLength	+= datat;
}
#endif
#if(KXYL_CTR>1)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	for(wateri = 0x00;wateri < 2;wateri++)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s%d\"",SENSORID_KXYL,wateri+1);
		dataOutLength	+= datat;
	}
}
#endif
	datat =	sprintf(bufOut+dataOutLength,"%s","}");
	dataOutLength	+= datat;

#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetsensorIDlen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckSample(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	uint8_t wateri;
	//receive:$cmd=sample
	//ack:$cmd=sample&datastreams={"L2_LF_1":"34.56","L2_LF_2":"67.45","L2_LF_3":"12.2"}
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=sample&datastreams={");	
	dataOutLength	+= datat;
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":%.2f,",SENSORID_HS,wateri+1,sensorValue.wat[wateri].f);
		dataOutLength	+= datat;	
	}
	for(wateri = 0x00;wateri < devSenNum;wateri++)
	{	
		datat =	sprintf(bufOut+dataOutLength,"\"%s%d\":%.2f",SENSORID_TW,wateri+1,sensorValue.soilt[wateri].f);
		dataOutLength	+= datat;
		if(wateri<(devSenNum-1))
		{
			datat =	sprintf(bufOut+dataOutLength,"%s",",");
			dataOutLength	+= datat;
		}
	}	
#if(QX_CTR>0)
	if(devPar.qxkey != FUNCITONG_TURN_OFF)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":",SENSORID_QJ);
		dataOutLength	+= datat;
		
		if(devPar.mqttDataType == MQTT_DATA_TYPE_STRING)
		{
			datat =	sprintf(bufOut+dataOutLength,"\"%.3f,%.3f,%.3f,",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);
			dataOutLength	+= datat;
			datat =	sprintf(bufOut+dataOutLength,"%.3f,%.3f\"",sensorValue.angle.f,sensorValue.dipYaw.f);
		}
		else if(devPar.mqttDataType == MQTT_DATA_TYPE_TREND)
		{
			datat =	sprintf(bufOut+dataOutLength,"{\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f,",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);
			dataOutLength	+= datat;
			datat =	sprintf(bufOut+dataOutLength,"\"angle\":%.3f,\"trend\":%.3f}",sensorValue.angle.f,sensorValue.dipYaw.f);
		}	
		else
		{
			datat =	sprintf(bufOut+dataOutLength,"{\"X\":%.3f,\"Y\":%.3f,\"Z\":%.3f,",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);
			dataOutLength	+= datat;
			datat =	sprintf(bufOut+dataOutLength,"\"angle\":%.3f,\"AZI\":%.3f}",sensorValue.angle.f,sensorValue.dipYaw.f);
		}			
		dataOutLength	+= datat;
	}
#endif
#if(ACCE_CTR>0)
	if(devPar.accelerometerkey == FUNCITONG_TURN_ON)
	{
		datat =	sprintf(bufOut+dataOutLength,",\"%s\":",SENSORID_ACCE);
		dataOutLength	+= datat;
		
		if(devPar.mqttDataType == MQTT_DATA_TYPE_STRING)
		{
			datat =	sprintf(bufOut+dataOutLength,"\"%.3f,%.3f,%.3f\"",sensorValue.ax.f*G_UNIT,sensorValue.ay.f*G_UNIT,sensorValue.az.f*G_UNIT);			
		}	
		else
		{
			datat =	sprintf(bufOut+dataOutLength,"{\"gX\":%.3f,\"gY\":%.3f,\"gZ\":%.3f}",sensorValue.ax.f*G_UNIT,sensorValue.ay.f*G_UNIT,sensorValue.az.f*G_UNIT);			
		}			
		dataOutLength	+= datat;
	}
#endif
#if(SK60_CTR>1)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	datat =	sprintf(bufOut+dataOutLength,",\"%s\":%.0f",SENSORID_LF,sensorValue.sk60disp.f*1000.0);
	dataOutLength	+= datat;
}
#endif
#if(KXYL_CTR>1)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":",SENSORID_KXYL,1);
	dataOutLength	+= datat;	
//	datat =	sprintf(bufOut+dataOutLength,"{\"temp\":0,\"value\":%.2f}",sensorValue.kxylv1.f);
	datat =	sprintf(bufOut+dataOutLength,"\"0,%.2f\"",sensorValue.kxylv1.f);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,",\"%s%d\":",SENSORID_KXYL,2);
	dataOutLength	+= datat;	
//	datat =	sprintf(bufOut+dataOutLength,"{\"temp\":0,\"value\":%.2f}",sensorValue.kxylv2.f);
	datat =	sprintf(bufOut+dataOutLength,"\"0,%.2f\"",sensorValue.kxylv2.f);
	dataOutLength	+= datat;
}
#endif
	datat =	sprintf(bufOut+dataOutLength,"%s","}");
	dataOutLength	+= datat;
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckSamplelen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckdevPeriod(char *bufOut,uint8_t sensoridindex,cigemsensortime_t period)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=reqsensortime&sensor_id=value
	//ack:$cmd=reqsensortime&sensor_id=value&sample_intv=value&upload_intv=value&plus_intv=value
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=reqsensortime&sensor_id=");	
	dataOutLength	+= datat;
#if(QX_CTR>0)
	if((devPar.qxkey != FUNCITONG_TURN_OFF)&&(sensoridindex == SENSORID_INDEX_QJ))
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_QJ);
	}
	else 
#endif
	if((sensoridindex&0x0F) == SENSORID_INDEX_HS)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_HS,(sensoridindex>>4));
	}
	else if((sensoridindex&0x0F) == SENSORID_INDEX_TW)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_TW,(sensoridindex>>4));
	}
#if(ACCE_CTR>0)
	else if(sensoridindex == SENSORID_INDEX_ACCE)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_ACCE);
	}
#endif
#if(SK60_CTR>1)
	else if(sensoridindex == SENSORID_INDEX_LF)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_LF);
	}
#endif
#if(KXYL_CTR>1)
	else if(sensoridindex == SENSORID_INDEX_KXYL)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_KXYL,(sensoridindex>>4));
	}
#endif
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"sample_intv=%d&",period.sample);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"upload_intv=%d&",period.upload);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"plus_intv=%d",period.plus);
	dataOutLength	+= datat;
	
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckreqsensortimelen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckGetsensorattr(char *bufOut,uint8_t sensoridindex,cigemsensorrange_t range)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=getsensorattr&sensor_id=value
	//ack:$cmd=getsensorattr&sensor_id=value&threshold=value&upper_limit=value&lower_limit=value
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getsensorattr&sensor_id=");	
	dataOutLength	+= datat;
#if(QX_CTR>0)
	if((devPar.qxkey != FUNCITONG_TURN_OFF)&&(sensoridindex == SENSORID_INDEX_QJ))
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_QJ);
	}
	else 
#endif
	if((sensoridindex&0x0F) == SENSORID_INDEX_HS)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_HS,(sensoridindex>>4));
	}
	else if((sensoridindex&0x0F) == SENSORID_INDEX_TW)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_TW,(sensoridindex>>4));
	}
#if(ACCE_CTR>0)
	else if(sensoridindex == SENSORID_INDEX_ACCE)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_ACCE);
	}
#endif
#if(SK60_CTR>1)
	else if(sensoridindex == SENSORID_INDEX_LF)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s&",SENSORID_LF);
	}
#endif
#if(KXYL_CTR>1)
	else if(sensoridindex == SENSORID_INDEX_KXYL)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s%d&",SENSORID_KXYL,(sensoridindex>>4));
	}
#endif	
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"threshold=%.2f&",range.threshold);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"upper_limit=%.2f&",range.upper);
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"lower_limit=%.2f",range.lower);
	dataOutLength	+= datat;
	
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetsensorattrlen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckGetworkmode(char *bufOut,char mode)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=getworkmode
	//ack:$cmd=getworkmode&mode=value
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getworkmode&mode=");	
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"%c",mode);
	dataOutLength	+= datat;
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetworkmodelen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
uint16_t cigemPacketAckGetcmdversion(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=getcmdversion
	//ack:$cmd=getcmdversion&version=value
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getcmdversion&version=");	
	dataOutLength	+= datat;

//	if(devPar.mqttDataType == MQTT_DATA_TYPE_STRING)
//	{
		datat =	sprintf(bufOut+dataOutLength,"%s",firwMare);//�Ĵ�����Ҫ��˴���̼��汾����ͬ���󶼸�Ϊ���մ˰汾���
//	}
//	else

//	{
//		datat =	sprintf(bufOut+dataOutLength,"%s",CIGEM_CMD_VERSION);
//	}
	dataOutLength	+= datat;
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetcmdversionlen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}

void clearbuffer(uint8_t* buf,int buflen)
{
	uint16_t rxi=0;
	for(rxi=0;rxi<buflen;rxi++)buf[rxi]= 0;
}

uint16_t findStrRxcigem(char *buf,uint8_t bufLength,uint8_t*data,uint16_t conuter)
{
	uint16_t findi 		= 0x00;
	uint16_t findj 		= 0x00;	
	__IO uint16_t ptr    	= 0x00;
	__IO uint16_t ptrOut 	= 0x00;
	
	if(bufLength > conuter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < conuter;findi++)
	{
		if(data[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(data[ptr++] != buf[findj])			
				{
					findj = 0x00;
					ptrOut ++;				
					break;
				}
			}
			if(findj ==bufLength)
			{
				return (ptr);
			} 
		}
		else
		{
			ptrOut ++;
		} 
	}
	return 	UN_FIND_STATUS;
}



int8_t contrastbuf(uint8_t *bufin,char*buf,uint8_t len)
{
	int8_t result = -1;
	uint8_t i;
	if(len==0)return result;
	for(i=0;i<len;i++)
	{
		if(buf[i]!=bufin[i])break;
	}
	if(i==len)result = 0;
	return result;
}
int ascii2int32(uint8_t *bufIn,uint8_t bufInlen)
{
	char databuf[12];
	int value	=	-1;
	uint16_t rxi;
	for(rxi=0;rxi<bufInlen;rxi++)
	{
		if(rxi>10)break;
		if(bufIn[rxi] == '&')
		{
			databuf[rxi] = '\0';
			value = atoi(databuf);
			break;
		}
		else if((bufIn[rxi]>='0')&&(bufIn[rxi]<='9'))
		{
			databuf[rxi] = bufIn[rxi];
		}
		else
		{
			if((bufIn[rxi]=='-')&&(rxi==0)){databuf[rxi] = bufIn[rxi];}
			else {break;}
		}
	}
	return value;
}
int16_t findsensorid(uint8_t *bufIn,uint16_t bufInlen,uint8_t *index)
{//&sensor_id=value&
	uint16_t 	findStatus 	= UN_FIND_STATUS;
	int16_t  	result = -1;	
//	uint8_t wateri;
	uint8_t waterj;
	*index	=	SENSORID_INDEX_NUM;
	findStatus	=	findStrRxcigem("&sensor_id=",11,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{
		if(contrastbuf(bufIn+findStatus,SENSORID_QJ,SENSORID_LEN_QJ) == 0)
		{
			*index	=	SENSORID_INDEX_QJ;						
		}
	#if(ACCE_CTR>0)
		else if(contrastbuf(bufIn+findStatus,SENSORID_ACCE,SENSORID_LEN_ACCE) == 0)
		{
			if(devPar.accelerometerkey == FUNCITONG_TURN_ON)*index	=	SENSORID_INDEX_ACCE;
		}
	#endif
	#if(SK60_CTR>1)
		else if(contrastbuf(bufIn+findStatus,SENSORID_LF,SENSORID_LEN_LF) == 0)
		{
			if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)*index	=	SENSORID_INDEX_LF;
		}
	#endif
	#if(KXYL_CTR>1)	
		else if(contrastbuf(bufIn+findStatus,SENSORID_KXYL,SENSORID_LEN_KXYL) == 0)
		{
			if((bufIn[findStatus+SENSORID_LEN_KXYL]>='0')&&(bufIn[findStatus+SENSORID_LEN_KXYL]<='9'))
			{
				waterj	=	bufIn[findStatus+SENSORID_LEN_KXYL]-'0';
				if((bufIn[findStatus+SENSORID_LEN_KXYL+1]>='0')&&(bufIn[findStatus+SENSORID_LEN_KXYL+1]<='9'))
				{
					if(bufIn[findStatus+SENSORID_LEN_KXYL+2]=='&')
					{
						waterj *= 10;
						waterj +=	bufIn[findStatus+SENSORID_LEN_KXYL+1]-'0';
					}
				}
				if(devPar.kxylFuction == FUNCITONG_TURN_ON)*index	=	SENSORID_INDEX_KXYL|(waterj<<4);
			}
		}
	#endif
		else if(contrastbuf(bufIn+findStatus,SENSORID_HS,SENSORID_LEN_HS) == 0)
		{
			if((bufIn[findStatus+SENSORID_LEN_HS]>='0')&&(bufIn[findStatus+SENSORID_LEN_HS]<='9'))
			{
				waterj	=	bufIn[findStatus+SENSORID_LEN_HS]-'0';
				if((bufIn[findStatus+SENSORID_LEN_HS+1]>='0')&&(bufIn[findStatus+SENSORID_LEN_HS+1]<='9'))
				{
					if(bufIn[findStatus+SENSORID_LEN_HS+2]=='&')
					{
						waterj *= 10;
						waterj +=	bufIn[findStatus+SENSORID_LEN_HS+1]-'0';
					}				
				}
				*index	=	SENSORID_INDEX_HS|(waterj<<4);
			}
		}
		else if(contrastbuf(bufIn+findStatus,SENSORID_TW,SENSORID_LEN_TW) == 0)
		{
			if((bufIn[findStatus+SENSORID_LEN_TW]>='0')&&(bufIn[findStatus+SENSORID_LEN_TW]<='9'))
			{
				waterj	=	bufIn[findStatus+SENSORID_LEN_TW]-'0';
				if((bufIn[findStatus+SENSORID_LEN_TW+1]>='0')&&(bufIn[findStatus+SENSORID_LEN_TW+1]<='9'))
				{
					if(bufIn[findStatus+SENSORID_LEN_TW+2]=='&')
					{
						waterj *= 10;
						waterj +=	bufIn[findStatus+SENSORID_LEN_TW+1]-'0';
					}
				}
				*index	=	SENSORID_INDEX_TW|(waterj<<4);
			}			
		}	
		if((*index&0x0F) <	SENSORID_INDEX_NUM)result = 0;
	}	
	return result;
}

int16_t findsensortime(uint8_t *bufIn,uint16_t bufInlen,cigemsensortime_t *time)
{//&sensor_id=value&sample_intv=value&upload_intv=value&plus_intv=value
	uint16_t 	findStatus 	= UN_FIND_STATUS;
	uint16_t 	findStatus2 = UN_FIND_STATUS;
	cigemsensortime_t	tmp = {0,0,0};
	int16_t  	result = -1;
	//$cmd=setsensortime&sensor_id=L3_TW_2&sample_intv=300&upload_intv=900&plus_intv=600
	if(findsensorid(bufIn,bufInlen,&cigemsensoridindex)==0)
	{
		findStatus	=	findStrRxcigem("&sample_intv=",13,bufIn,bufInlen);	
		findStatus2	=	findStrRxcigem("&upload_intv=",13,bufIn,bufInlen);
		if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
		{
			tmp.sample	=	ascii2int32(bufIn+findStatus,findStatus2-findStatus);		
			findStatus	=	findStrRxcigem("&plus_intv=",11,bufIn,bufInlen);
			if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
			{
				tmp.upload	=	ascii2int32(bufIn+findStatus2,findStatus-findStatus2);
				tmp.plus		=	ascii2int32(bufIn+findStatus,bufInlen-findStatus);
				if((tmp.sample>=CIGEM_TIME_SAMPLE_MIN)&&(tmp.upload>=CIGEM_TIME_UPLOAD_MIN)&&(tmp.plus>=CIGEM_TIME_PLUS_MIN))
				{
					if((tmp.sample<=CIGEM_TIME_SAMPLE_MAX)&&(tmp.upload<=CIGEM_TIME_UPLOAD_MAX)&&(tmp.plus<=CIGEM_TIME_PLUS_MAX))
					{
						*time = tmp;	
//						printf("\r\n sample:%d,up:%d,pl:%d ",tmp.sample,tmp.upload,tmp.plus);
						result = 0;
					}
				}	
			}			
		}			
	}
	return result;
}
float ascii2float(uint8_t *bufIn,uint8_t bufInlen)
{
	char 	databuf[12];
	float value	=	FLOAT_VALUE_ERR;
	uint16_t rxi = 0;
	for(rxi=0;rxi<bufInlen;rxi++)
	{
		if(rxi>10)break;
		if((bufIn[rxi] == '&')||(bufIn[rxi] == ','))
		{
			databuf[rxi] = '\0';
			value = atof(databuf);
			break;
		}
		else if(((bufIn[rxi]>='0')&&(bufIn[rxi]<='9'))||(bufIn[rxi]=='.'))
		{
			databuf[rxi] = bufIn[rxi];
		}
		else
		{
			if((bufIn[rxi]=='-')&&(rxi==0)){databuf[rxi] = bufIn[rxi];}
			else {break;}
		}
	}
	return value;
}
int16_t findsensorrange(uint8_t *bufIn,uint16_t bufInlen,cigemsensorrange_t *range)
{//&sensor_id=value&threshold=value&upper_limit=value&lower_limit=value
	uint16_t 	findStatus 	= UN_FIND_STATUS;
	uint16_t 	findStatus2 = UN_FIND_STATUS;
	cigemsensorrange_t	tmp = {0.0,0.0,0.0};
	int16_t  	result = -1;
	
	if(findsensorid(bufIn,bufInlen,&cigemsensoridindex)==0)
	{
		findStatus	=	findStrRxcigem("&threshold=",11,bufIn,bufInlen);			
		findStatus2	=	findStrRxcigem("&upper_limit=",13,bufIn,bufInlen);
		if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
		{
			tmp.threshold	=	ascii2float(bufIn+findStatus,findStatus2-findStatus);	
			findStatus	=	findStrRxcigem("&lower_limit=",13,bufIn,bufInlen);
			if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
			{
				tmp.upper	=	ascii2float(bufIn+findStatus2,findStatus-findStatus2);
				tmp.lower	=	ascii2float(bufIn+findStatus,bufInlen-findStatus);
				if((tmp.threshold >0)&&(tmp.upper>0)&&(tmp.lower>=-90.0))
				{
				//	if((tmp.threshold<=CIGEM_TIME_SAMPLE_MAX)&&(tmp.upper<=CIGEM_TIME_UPLOAD_MAX)&&(tmp.lower<=CIGEM_TIME_PLUS_MAX))
					{
						*range = tmp;
						result = 0;
					}
				}		
			}						
		}		
	}
	return result;
}

int16_t findwarning(uint8_t *bufIn,uint16_t bufInlen,cigemwarning_t *warning)
{//$cmd=meteorologicalearlywarning&level=value&effective_time=value&lon_range=value&lat_range=value
	uint16_t 	findStatus 	= UN_FIND_STATUS;
	uint16_t 	findStatus2 = UN_FIND_STATUS;
	uint16_t 	findStatus3 = UN_FIND_STATUS;
	cigemwarning_t	tmp 	= cigemwaring_initializer;

	float		longitudemin	= 0.0;
	float		longitudemax	= 0.0;
	float		latitudemin		= 0.0;
	float		latitudemax		= 0.0;
	int16_t  	result = -1;
	findStatus	=	findStrRxcigem("&level=",7,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{
		tmp.level = bufIn[findStatus];
		if((tmp.level>=WARNING_LEVEL_RED)&&(tmp.level<=WARNING_LEVEL_BLUE))
		{
	//		printf("\r\n level:%c",tmp.level);
			findStatus	=	findStrRxcigem("&effective_time=",16,bufIn,bufInlen);
			findStatus2	=	findStrRxcigem("&lon_range=",11,bufIn,bufInlen);
			if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
			{
				tmp.time	=	ascii2int32(bufIn+findStatus,findStatus2-findStatus)*60;
				//printf("\r\n time:%d",tmp.time);
				findStatus3	=	findStrRxcigem("&lat_range=",11,bufIn,bufInlen);//&effective_time=13&lon_range=33.5,34.6&lat_range=13.7,15.9
				findStatus	=	findStrRxcigem(",",1,bufIn+findStatus2,findStatus3 - findStatus2);
				if((findStatus < FIND_ERROR)&&(findStatus3 < FIND_ERROR))
				{
					longitudemin =	ascii2float(bufIn+findStatus2,findStatus-findStatus2);
					longitudemax =	ascii2float(bufIn+findStatus2+findStatus,findStatus3-findStatus-findStatus2);
					//printf("\r\n longitude:%f,%f,%d,%d",tmp.longitudemin,tmp.longitudemax,findStatus,findStatus3);
					findStatus2	=	findStrRxcigem("&",1,bufIn+findStatus3,bufInlen - findStatus3);
					findStatus	=	findStrRxcigem(",",1,bufIn+findStatus3,findStatus2 - findStatus3);					
					if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
					{	
						latitudemin =	ascii2float(bufIn+findStatus3,findStatus-findStatus3);
						latitudemax =	ascii2float(bufIn+findStatus3+findStatus,findStatus2-findStatus);
						//printf("\r\n latitude:%f,%f,%d,%d",tmp.latitudemin,tmp.latitudemax,findStatus,findStatus2);
						if((longitudemin>FLOAT_VALUE_ERR)&&(longitudemax>FLOAT_VALUE_ERR)&&(latitudemin>FLOAT_VALUE_ERR)&&(latitudemax>FLOAT_VALUE_ERR))
						{
							
							if((devLatitude>=latitudemin)&&(devLatitude<=latitudemax))
							{
								if((devLongitude>=longitudemin)&&(devLongitude<=longitudemax))
								{
									*warning = tmp;
									result	=	0;
								}
							}							
						}
					}
				}				
			}
		}
	}
	return result;
}
int16_t findalarm(uint8_t *bufIn,uint16_t bufInlen,cigemwarning_t *warning)
{//$cmd=alarm&type=L3_YL_1&level=1
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	cigemwarning_t	tmp 	= cigemwaring_initializer;

	int16_t  	result = -1;
	findStatus	=	findStrRxcigem("&type=",6,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{
		findStatus	=	findStrRxcigem(ALARM_TYPE,ALARM_TYPE_LEN,bufIn,bufInlen);
	}
	if(findStatus < FIND_ERROR)
	{
		findStatus	=	findStrRxcigem("&level=",7,bufIn,bufInlen);
	}
	if(findStatus < FIND_ERROR)
	{		
		tmp.level = bufIn[findStatus];
		if((tmp.level>=WARNING_LEVEL_RED)&&(tmp.level<=WARNING_LEVEL_BLUE))
		{
	//		printf("\r\n level:%c",tmp.level);
			tmp.time	=	ALARM_TIME_MINUTE;
			*warning 	= tmp;
			result		=	0;		
		}
	}
	return result;
}
void cigemalarmdeal(void)
{
	cigemresultflag					=	CIGEM_CMD_FLAG_ok;	
	cigemEarlyWarningTimes	=	getcurminute();						
	serverConfig.perSetFlag	=	CHANGE_FLAG;
#if((USART_USE_CTR>0)&&(BEEP_ALARM_CTR>0))					
	alarmTimes 	= 1;
	alarmbeepStatus	= ALARM_STATUS_FREE;
	alarmSource =	ALARM_SOURCE_PLATFORM;
	if(cigemwarning.level == WARNING_LEVEL_RED){alarmType = MODBUS_ALARM_INDEX_LANDSLIDE4;}
	else if(cigemwarning.level ==	WARNING_LEVEL_ORANGE){alarmType = MODBUS_ALARM_INDEX_LANDSLIDE3;}
	else{alarmType = MODBUS_ALARM_INDEX_LANDSLIDE2;}
#endif
}
int16_t findbroadcast(uint8_t *bufIn,uint16_t bufInlen,uint8_t *times,uint8_t *level)
{//$cmd=broadcast&b_num=value&b_size=value&b_content=value
 //$cmd=broadcast&b_num=1&b_size=9&b_content=L3_YL_1,2&msgid=27d0b95e-394f-4da2-a64b-8446729168fd&apikey=3A2EE5AD40015C2BBC8D53E8A0C58647
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	uint16_t 	findStatus2 = UN_FIND_STATUS;
	uint8_t   num_tmp=0,size_tmp=0,level_tmp=0;
	int16_t  	result = -1;	
	char cmbuf[50];
	
	findStatus	=	findStrRxcigem("&b_num=",7,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{
		num_tmp =	ascii2int32(bufIn+findStatus,bufInlen-findStatus);
	}
	findStatus2	=	findStrRxcigem("&b_size=",8,bufIn,bufInlen);
	if(findStatus2 < FIND_ERROR)
	{
		size_tmp =	ascii2int32(bufIn+findStatus2,bufInlen-findStatus2);
	}
	if((findStatus < FIND_ERROR)&&(findStatus2 < FIND_ERROR))
	{			
		cmbuf[0]= sprintf(cmbuf+1,"&b_content=%s,",ALARM_TYPE);
		findStatus	=	findStrRxcigem(cmbuf+1,cmbuf[0],bufIn,bufInlen);			
		if((findStatus < FIND_ERROR)&&((size_tmp+11)>=cmbuf[0]))
		{				
			level_tmp =	bufIn[findStatus];
			if((num_tmp>0)&&(level_tmp>=WARNING_LEVEL_RED)&&(level_tmp<=WARNING_LEVEL_BLUE))
			{
				*times	=	num_tmp;
				*level	= level_tmp;
				result	= 0;										
			}	
		}			
	}	
	return result;
}

#if(SECOND_MQTT_ADDR>0)
uint16_t cigemPacketAckGetplatform(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	uint16_t controlNo = 0;
#if(SECOND_MQTT_ADDR>0)
	secondServerAddr secondServerAddrBuf[SECOND_MULTI_CONNECTION_SIZE];	
#endif
	//receive:$cmd=getplatform&controlNo=value
	//ack:$cmd=getplatform&mqtt_addr=value&mqtt_port=value&client_id=value&username=value&password=value&http_addr=value&http_port=value&device_id=value&api_key=value&controlNo=value
	mqttSecondServerAddrRead(secondServerAddrBuf);
	
	controlNo = serverControlNo - '1';
	if(controlNo>=SECOND_MULTI_CONNECTION_SIZE)controlNo=0;
#if((SECOND_MULTI_CONNECTION_SIZE>1)&&(SECOND_MQTT_ADDR>1))
	if(devPar.secondAddrKey == MQTT_SECOND_ADDR_NUM_2)
	{
	}else
#endif
	{
		controlNo = 0;
		if(serverControlNo == '1')
		{
			memcpy(secondServerAddrBuf[0].endpoint,mqttServerAddrBuf[0].endpoint,SERVER_ENDPOINT_SIXE+1);
			secondServerAddrBuf[0].port = mqttServerAddrBuf[0].port;
			memcpy(secondServerAddrBuf[0].clientId,mqttServerAddrBuf[0].deviceId,SERVER_DEVICEID_SIZE+1);
			memcpy(secondServerAddrBuf[0].username,mqttServerAddrBuf[0].username,SERVER_USERNAME_SIZE+1);
			memcpy(secondServerAddrBuf[0].password,mqttServerAddrBuf[0].password,SERVER_PASSWORD_SIZE+1);
		
			secondServerAddrBuf[0].httpaddr[1]	= '0';secondServerAddrBuf[0].httpaddr[2]	= '\0';
			secondServerAddrBuf[0].httpport 		= 0;
			secondServerAddrBuf[0].deviceId[1] 	= '0';secondServerAddrBuf[0].deviceId[2] 	= '\0';
			secondServerAddrBuf[0].apikey[1] 		= '0';secondServerAddrBuf[0].apikey[2] 		= '\0';
		}
	}

	if(secondServerAddrBuf[controlNo].userFlag != TCP_SERVER_ADDRESS_USE_FLAG)
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getplatform&mqtt_addr=0,&mqtt_port=0&client_id=0&username=0&password=0&http_addr=0&http_port=0&device_id=0&api_key=0");	
		dataOutLength	+= datat;		
	}
	else
	{
		datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=getplatform&mqtt_addr=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].endpoint[1]);	
		dataOutLength	+= datat;

		datat =	sprintf(bufOut+dataOutLength,"&mqtt_port=%d",secondServerAddrBuf[controlNo].port);	
		dataOutLength	+= datat;
		
		datat =	sprintf(bufOut+dataOutLength,"%s","&client_id=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].clientId[1]);	
		dataOutLength	+= datat;

		datat =	sprintf(bufOut+dataOutLength,"%s","&username=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].username[1]);	
		dataOutLength	+= datat;
	
		datat =	sprintf(bufOut+dataOutLength,"%s","&password=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].password[1]);	
		dataOutLength	+= datat;
			
		datat =	sprintf(bufOut+dataOutLength,"%s","&http_addr=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].httpaddr[1]);	
		dataOutLength	+= datat;

		datat =	sprintf(bufOut+dataOutLength,"&http_port=%d",secondServerAddrBuf[controlNo].httpport);	
		dataOutLength	+= datat;
		
		datat =	sprintf(bufOut+dataOutLength,"%s","&device_id=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].deviceId[1]);	
		dataOutLength	+= datat;
	
		datat =	sprintf(bufOut+dataOutLength,"%s","&api_key=");	
		dataOutLength	+= datat;
		datat =	sprintf(bufOut+dataOutLength,"%s",&secondServerAddrBuf[controlNo].apikey[1]);	
		dataOutLength	+= datat;	
	}	
	datat =	sprintf(bufOut+dataOutLength,"&controlNo=%c",serverControlNo);	
	dataOutLength	+= datat;

#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckGetplatform:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}
int16_t findsetplatform(uint8_t *bufIn,uint16_t bufInlen,secondServerAddr *secondServer)
{//$cmd=setplatform&mqtt_addr=value&mqtt_port=value&client_id=value&username=value&password=value&http_addr=value&http_port=value&device_id=value&api_key=value&controlNo=xx&type=xxx
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	int16_t  	result = -1;	
	uint16_t tmpi;
	uint8_t	channel=0;
	uint8_t channelctr=0;
	secondServerAddr secondServerBuf;
//	char *bufTmp;	
//	bufTmp = secondServerBuf.endpoint;
	
	findStatus	=	findStrRxcigem("&mqtt_addr=",11,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{		
		for(tmpi=1;tmpi<=SERVER_ENDPOINT_SIXE;tmpi++)
		{			
			if(bufIn[findStatus] == '&')
			{
				if(tmpi>8)
				{
					result = 1;
					secondServerBuf.endpoint[tmpi] 	= '\0';
					secondServerBuf.endpoint[0]			= tmpi;
				}
				break;
			}
			else if(bufIn[findStatus] <=0x20){break;}
			secondServerBuf.endpoint[tmpi] = bufIn[findStatus++];
		}
	}
	if(result == 1)
	{
		findStatus	=	findStrRxcigem("&mqtt_port=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			secondServerBuf.port =	ascii2int32(bufIn+findStatus,bufInlen-findStatus);
			if(secondServerBuf.port>1)result = 2;
		}
	}
	if(result == 2)
	{
		findStatus	=	findStrRxcigem("&client_id=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_DEVICEID_SIZE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{
					if(tmpi>2)
					{
						result = 3;
						secondServerBuf.clientId[tmpi] 	= '\0';
						secondServerBuf.clientId[0]			= tmpi;
					}
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.clientId[tmpi] = bufIn[findStatus++];
			}
		}
	}
	if(result == 3)
	{
		findStatus	=	findStrRxcigem("&username=",10,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_USERNAME_SIZE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{
					if(tmpi>2)
					{
						result = 4;
						secondServerBuf.username[tmpi] 	= '\0';
						secondServerBuf.username[0]			= tmpi;
					}
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.username[tmpi] = bufIn[findStatus++];
			}
		}
	}
	if(result == 4)
	{
		findStatus	=	findStrRxcigem("&password=",10,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_PASSWORD_SIZE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{
					if(tmpi>2)
					{
						result = 5;
						secondServerBuf.password[tmpi] 	= '\0';
						secondServerBuf.password[0]			= tmpi;
					}
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.password[tmpi] = bufIn[findStatus++];
			}
		}
	}
	if(result == 5)
	{
		for(tmpi=0;tmpi<mqttServerNumFirst;tmpi++)
		{
			if(contrastbuf((uint8_t *)mqttServerAddrBuf[tmpi].endpoint,secondServerBuf.endpoint,mqttServerAddrBuf[tmpi].endpoint[0])==0)
			{
				if(mqttServerAddrBuf[tmpi].port == secondServerBuf.port)
				{
					if(contrastbuf((uint8_t *)mqttServerAddrBuf[tmpi].deviceId,secondServerBuf.clientId,mqttServerAddrBuf[tmpi].deviceId[0])==0)
					{
						result = -2;
					}
				}
			}
		}
	}
	if(result == 5)
	{
		findStatus	=	findStrRxcigem("&http_addr=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_ENDPOINT_SIXE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{					
					result = 6;
					secondServerBuf.httpaddr[tmpi] 	= '\0';
					secondServerBuf.httpaddr[0]			= tmpi;					
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.httpaddr[tmpi] = bufIn[findStatus++];
			}
		}
	}	
	if(result == 6)
	{
		findStatus	=	findStrRxcigem("&http_port=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			secondServerBuf.httpport =	ascii2int32(bufIn+findStatus,bufInlen-findStatus);
			result = 7;//if(secondServerBuf.httpport>0)
		}
	}
	if(result == 7)
	{
		findStatus	=	findStrRxcigem("&device_id=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_DEVICEID_SIZE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{
					result = 8;
					secondServerBuf.deviceId[tmpi] 	= '\0';
					secondServerBuf.deviceId[0]			= tmpi;
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.deviceId[tmpi] = bufIn[findStatus++];
			}
		}
	}
	if(result == 8)
	{
		findStatus	=	findStrRxcigem("&api_key=",9,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			for(tmpi=1;tmpi<=SERVER_PASSWORD_SIZE;tmpi++)
			{
				if(bufIn[findStatus] == '&')
				{
					result = 9;
					secondServerBuf.apikey[tmpi] 	= '\0';
					secondServerBuf.apikey[0]			= tmpi;
					break;
				}else if(bufIn[findStatus] <=0x20){break;}
				secondServerBuf.apikey[tmpi] = bufIn[findStatus++];
			}
		}
	}	
	if(result == 9)
	{
		findStatus	=	findStrRxcigem("&controlNo=",11,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			if((bufIn[findStatus] == '1')||(bufIn[findStatus]=='2'))			
			{
				channel = bufIn[findStatus] -'1';
				result  = 10;
			}
		}
	}
	if(result == 10)
	{
		findStatus	=	findStrRxcigem("&type=",6,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			if((bufIn[findStatus] == '0')||(bufIn[findStatus] == '1')||(bufIn[findStatus]=='2'))			
			{
				channelctr = bufIn[findStatus];
				result  = 11;
			}
		}
	}	
	if(result == 11)
	{		
		result = 0;
		switch(channelctr)
		{
			case '0'://����				
			case '2'://�޸�
			{				
			#if((SECOND_MULTI_CONNECTION_SIZE>1)&&(SECOND_MQTT_ADDR>1))
				if(devPar.secondAddrKey == MQTT_SECOND_ADDR_NUM_2)
				{
					for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)
					{
						if(contrastbuf((uint8_t *)secondServer[tmpi].endpoint,secondServerBuf.endpoint,secondServer[tmpi].endpoint[0])==0)
						{
							if(secondServer[tmpi].port == secondServerBuf.port)
							{
								if(contrastbuf((uint8_t *)secondServer[tmpi].clientId,secondServerBuf.clientId,secondServer[tmpi].clientId[0])==0)
								{
									if(channel != tmpi)result = -3;
								}
							}
						}
					}
				} 
			#endif
				if(result == 0)secondServerBuf.userFlag = TCP_SERVER_ADDRESS_USE_FLAG;	
				break;
			}
			case '1'://ɾ��
			{
//				for(tmpi=0;tmpi<MQTT_SECOND_ADDRESS_LENGTH-1;tmpi++)bufTmp[tmpi]=0;
//				secondServerBuf.userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;	
				clearSecondAddrBuffer(&secondServerBuf);
				break;			
			}
			default:result = -4;break;
		}
		if(result == 0)
		{
		#if((SECOND_MULTI_CONNECTION_SIZE>1)&&(SECOND_MQTT_ADDR>1))
			if(devPar.secondAddrKey == MQTT_SECOND_ADDR_NUM_2)
			{
			}else 
		#endif
			{
				if(channel == 0){result = -5;}
				else
				{
					channel = 0;
				}				
			}		
			if(result == 0)secondServer[channel]	= secondServerBuf;
		}
	}
	return result;
}

int16_t findgetplatform(uint8_t *bufIn,uint16_t bufInlen,uint8_t *controlNo)
{//$cmd=getplatform&controlNo=value
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	int16_t  	result = -1;	

	findStatus	=	findStrRxcigem("&controlNo=",11,bufIn,bufInlen);
	if(findStatus < FIND_ERROR)
	{
		if((bufIn[findStatus] == '1')||(bufIn[findStatus]=='2'))			
		{
			*controlNo = bufIn[findStatus];
			result  = 0;
		}
	}
	return result;
}
#endif

#if(UPGRADE_FRIMWARE_CTR>0)
__IO uint32_t upgradefrimwaresize = 0;//�̼���С
__IO uint8_t 	upgradelinknum			=	0;//��ǰ���յķ�������1~4
__IO uint16_t upgradepacketnum		=	0;//��ǰ���յ��İ���
__IO uint8_t	upgradeendpacketflag	=	0;
__IO uint32_t	upgradebegintime=0;
__IO uint32_t	upgradereceivetime=0;
char md5buffer[MD5_BUFFSIZE];//���յ���md5�ַ�����0wλ�ô�md5ֵ�ó���
char cigemcmdtype =0;
uint8_t	cigemupgradeflag	=	CIGEM_CMD_FLAG_none;

uint16_t cigemPacketAckUpgrade(char *bufOut)
{
	uint16_t datat = 0x00;
	uint16_t dataOutLength= 0x00;	
	//receive:$cmd=upgrade&md5=value&size=value
	//ack://ack:$cmd=supportsize&range=value		//$cmd=supportsize&range=0.05,100
	
	datat =	sprintf(bufOut+dataOutLength,"%s","$cmd=supportsize&range=");	
	dataOutLength	+= datat;
	datat =	sprintf(bufOut+dataOutLength,"%d,%d",UPGRADE_PACKET_SIZE_MIN,UPGRADE_PACKET_SIZE_MAX);
	dataOutLength	+= datat;
#if(CIGE_SENSOR_DATA_DEBUG>0)	
	printf("\r\nAckUpgradelen:%d\r\n",dataOutLength);
	devSendBuffer(bufOut,dataOutLength);
#endif
	return dataOutLength;
}

int16_t cigemdevidcheck(MQTTString topic,uint8_t channel)
{
	int16_t  result = -1;
	uint8_t *bufIn=(uint8_t *)topic.lenstring.data;
	
//	printf("\r\n devid:%d %s",mqttServerAddrBuf[channel].deviceId[0],mqttServerAddrBuf[channel].deviceId+1);
//	printf("\r\n topic:%d %s",topic.lenstring.len,topic.lenstring.data);
	if(topic.lenstring.len >= (mqttServerAddrBuf[channel].deviceId[0]+9))
	{		
		if(contrastbuf(bufIn,"$creq/",6)==0)
		{	
			cigemcmdtype = 0;
			if(contrastbuf(bufIn+6,mqttServerAddrBuf[channel].deviceId+1,mqttServerAddrBuf[channel].deviceId[0]-1)==0)
			{		
				if(contrastbuf(bufIn+6+mqttServerAddrBuf[channel].deviceId[0],"cmd",3)==0){cigemcmdtype =1;}
		
				else if(contrastbuf(bufIn+6+mqttServerAddrBuf[channel].deviceId[0],"firmware",8)==0)
				{
					if(upgradelinknum == (channel+1))
					{
						if(cigemupgradeflag	== CIGEM_CMD_FLAG_waitrx)
						{
							cigemupgradeflag	=	CIGEM_CMD_FLAG_rx;
							mqttFirewareClear();
						}
						cigemcmdtype =2;					
					}
					else{upgradelinknum = (channel+1);cigemupgradeflag	=	CIGEM_CMD_FLAG_err;}
				}
				printf("\r\n cigemcmdtype��%d,upgradelinknum:%d",cigemcmdtype,upgradelinknum);
			
				result = 0;
			}
		}
	}
	return result;
}
int16_t cigemupgradecheck(uint8_t *bufIn,uint16_t bufInlen,uint8_t linknum)
{
	uint16_t 	findStatus = UN_FIND_STATUS;
	uint32_t 	sizevalue	=	0;
	int16_t  	result = -1;
	uint16_t 	rxi=0;
	//pub:$cmd=upgrade&md5=value&size=value
	
	if((upgradelinknum == 0)||((upgradelinknum == linknum)&&(upgradepacketnum==0)))
	{
		findStatus	=	findStrRxcigem("&size=",6,bufIn,bufInlen);
		if(findStatus < FIND_ERROR)
		{
			sizevalue	=	ascii2int32(bufIn+findStatus,bufInlen-findStatus);
			if(sizevalue<=FRIMWARE_SIZE_MAX)
			{
				result = 1;
			}
		}
		if(result ==1)
		{
			clearbuffer((uint8_t *)md5buffer,MD5_BUFFSIZE);
			findStatus	=	findStrRxcigem("&md5=",5,bufIn,bufInlen);
			if(findStatus < FIND_ERROR)
			{
				for(rxi=1;rxi<(bufInlen-5);rxi++)
				{
					if(bufIn[findStatus]=='&')
					{
						result = 0;
						md5buffer[0]	 			= rxi-1;	
						upgradelinknum 			= linknum;
						upgradefrimwaresize	=	sizevalue;
						upgradepacketnum		=	0;				
						cigemupgradeflag		=	CIGEM_CMD_FLAG_req;
						upgradeendpacketflag	=	0;
						printf("\r\n upgrade channel��%d_%d",upgradelinknum,upgradefrimwaresize);
//						mqttFirewareClear();
						upgradereceivetime	=	getCurrterTime();
						upgradebegintime		= upgradereceivetime;
						serverConfig.gsmRunMode = FUNCITONG_TURN_ON;
						break;
					}
					md5buffer[rxi]=bufIn[findStatus++];
				}			
			}
		}		
	}
	return result;
}
int16_t cigemupgraderecevie(uint8_t *bufIn,uint16_t bufInlen)
{
	int16_t  	result = -1;
//	uint16_t 	rxi=0;
	uint16_t	packetindex = 0;
	uint16_t	datasize 		= 0;
	uint16_t	packetsize 	= 0;
	
	packetindex	=	((uint16_t)bufIn[0]<<8)+bufIn[1];
	datasize		=	((uint16_t)bufIn[2]<<8)+bufIn[3];
	packetsize	=	((uint16_t)bufIn[4]<<8)+bufIn[5];
	upgradereceivetime	=	getCurrterTime();
	printf("\r\n upgrade index%d_%d,%d ",packetsize,packetindex,datasize);
	if((packetindex<packetsize)&&(datasize<=UPGRADE_PACKET_SIZE_MAX))
	{
		if(((bufInlen==(datasize+6))&&(packetindex<(packetsize-1)))||((bufInlen==(datasize+7+md5buffer[0]))&&(packetindex==(packetsize-1))))
		{
			result	=	0;	
			upgradepacketnum++;
			printf(" receivenum %d ",upgradepacketnum);

			mqttFirewareSave(packetindex,bufIn+6,datasize);
			
			if((packetindex+1) == packetsize)
			{
				result = -2;
				if(bufIn[datasize+6]==md5buffer[0])
				{
					if(contrastbuf(bufIn+datasize+7,(char*)(md5buffer+1),md5buffer[0]) == 0)
					{
						result = 0;
						upgradeendpacketflag	=	1;
						printf(" ending ");
					}
				}
			}
			if(upgradepacketnum == packetsize)
			{
				if(upgradeendpacketflag==1)
				{
					printf(" upgrade full %d\r\n",upgradepacketnum);				
					cigemupgradeflag	=	CIGEM_CMD_FLAG_rxok;
				}
				else{result = -3;}
			}
		}	
	}	
	return result;
}
#endif
int16_t cigemRxdataTask(uint8_t *bufIn,uint16_t bufInlen,uint8_t* bufOut,uint8_t channel)
{
	uint16_t 	findStatus = UN_FIND_STATUS;
	uint16_t 	ptrout	=	0;
	int16_t  	result = -1;
	uint16_t 	rxi=0;
	uint8_t		b_content=0,b_times=0;
	cigemsensorrange_t	value = {0.0,0.0,0.0};
#if(SECOND_MQTT_ADDR>0)
	secondServerAddr secondServerAddrBuf[SECOND_MULTI_CONNECTION_SIZE];	
#endif	
	//findStatus	=	findStrRxcigem("$cmd=",5,bufIn,bufInlen);
	//if(findStatus < FIND_ERROR)
	if(contrastbuf(bufIn,"$cmd=",5)==0)
	{
		cigemcmd = CIGEM_CMD_none;
		ptrout	 = 5;
//		ptrout = findStatus;
		findStatus	=	findStrRxcigem("&apikey=",8,bufIn,bufInlen);//&apikey=e7eb73db9cc982caeb1f&msgid=B6ZmaDbzA
		if(findStatus < FIND_ERROR)
		{			
			if(contrastbuf(bufIn+findStatus,(char*)(cigemapikey+1),cigemapikey[0]-1) == 0)
			{
				findStatus	=	findStrRxcigem("&msgid=",7,bufIn,bufInlen);
				if(findStatus < FIND_ERROR)
				{					
					if(contrastbuf(bufIn+findStatus,(char*)(cigemmsgid[channel]+1),cigemmsgid[channel][0]-1) != 0)
					{
						cigemcmdflag[channel] = CIGEM_CMD_FLAG_req;//201227
						clearbuffer(cigemmsgid[channel],CIGEM_MSGID_LEN);
						cigemmsgid[channel][0] = 0;
						for(rxi=0;rxi<bufInlen-findStatus;rxi++)
						{
							if(((bufIn[rxi+findStatus]=='\r')&&(bufIn[rxi+1+findStatus]=='\n'))||(bufIn[rxi+findStatus]=='&'))break;
							cigemmsgid[channel][0]++;
						}						
	//cigemmsgid[0] = bufInlen-findStatus;
						for(rxi=0;rxi<cigemmsgid[channel][0];rxi++)
						{
							cigemmsgid[channel][rxi+1] = bufIn[rxi+findStatus];
						}					
						result = 1;
						cigemcmdLast =0;
					}
					else
					{
						if(cigemcmdflag[channel] == CIGEM_CMD_FLAG_req)//201227
						{
							result = 1;
							cigemcmdLast =0;
						}
						else
						{
							cigemcmdLast =1;
						}						
					}
					printf("\r\n cigemmsgid:%d,",cigemmsgid[channel][0]);
					devSendBuffer((char*)(cigemmsgid[channel]+1),cigemmsgid[channel][0]);				
				}
			}
			else
			{
				printf("\r\n dev apikey:");
				devSendBuffer((char*)(cigemapikey+1),cigemapikey[0]);
				printf("\r\n rx apikey:");
				devSendBuffer((char*)(bufIn+findStatus),cigemapikey[0]);
			}
		}
		if(result == 1)
		{
			if(contrastbuf(bufIn+ptrout,"reqtime",7) == 0)//$cmd=reqtime&apikey=e7eb73db9cc982caeb1f&msgid=B6ZmaDbzA
			{				
				cigemcmd = CIGEM_CMD_reqtime;
			}
			else if(contrastbuf(bufIn+ptrout,"settime",7) == 0)//$cmd=settime&server=ntpserver
			{			
				cigemsettimeflag = CIGEM_CMD_FLAG_req;
				cigemcmd = CIGEM_CMD_settime;			
			}
			else if(contrastbuf(bufIn+ptrout,"getstatus",9) == 0)//$cmd=getstatus
			{				
				cigemcmd = CIGEM_CMD_getstatus;			
			}
			else if(contrastbuf(bufIn+ptrout,"reboot",6) == 0)//$cmd=reboot
			{
				serverConfig.perSetFlag	=	CHANGE_FLAG;
				cigemperiodrunmodeInit();
				cigemrebootflag	=	CIGEM_CMD_FLAG_req;
				cigemcmd = CIGEM_CMD_reboot;			
			}
			else if(contrastbuf(bufIn+ptrout,"getsensorID",11) == 0)//$cmd=getsensorID
			{
				cigemcmd = CIGEM_CMD_getsensorID;			
			}
			else if(contrastbuf(bufIn+ptrout,"sample",6) == 0)//$cmd=sample
			{
				cigemsampleflag	=	CIGEM_CMD_FLAG_req;
				cigemcmd = CIGEM_CMD_sample;
			}
			else if(contrastbuf(bufIn+ptrout,"setsensortime",13) == 0)//$cmd=setsensortime&sensor_id=value&sample_intv=value&upload_intv=value&plus_intv=value
			{
				cigemsetsensortimeflag	=	CIGEM_CMD_FLAG_err;
				if(findsensortime(bufIn+ptrout,bufInlen-ptrout,&cigemsensortime)==0)
				{
//printf("\r\n intv_sample:%d,upload:%d,plus:%d\r\n",cigemsensortime.sample,cigemsensortime.upload,cigemsensortime.plus);
					cigemSensorTimeChange();				
					cigemsetsensortimeflag	=	CIGEM_CMD_FLAG_req;
					alarmSource = READ_SOURCE_STATUS;
					alarmLevel  = '4';
				}
				cigemcmd = CIGEM_CMD_setsensortime;			
			}
			else if(contrastbuf(bufIn+ptrout,"reqsensortime",13) == 0)//$cmd=reqsensortime&sensor_id=value $cmd=reqsensortime&sensor_id=L3_TW_2 //201210
			{				
				if(findsensorid(bufIn+ptrout,bufInlen-ptrout,&cigemsensoridindex)==0)
				{
					cigemConfigInit();
				}
				cigemcmd = CIGEM_CMD_reqsensortime;			
			}
			else if(contrastbuf(bufIn+ptrout,"setsensorattr",13) == 0)//$$cmd=setsensorattr&sensor_id=value&threshold=value&upper_limit=value&lower_limit=value
			{
				cigemsetsensorrangeflag	=	CIGEM_CMD_FLAG_err;
				if(findsensorrange(bufIn+ptrout,bufInlen-ptrout,&value)==0)
				{
					if((cigemsensoridindex&0x0F) < SENSORID_INDEX_NUM)//0x0F  201210
					{
						cigemsensorlimit[cigemsensoridindex&0x0F] = value;
						cigemsetsensorrangeflag	=	CIGEM_CMD_FLAG_req;
						alarmSource = READ_SOURCE_STATUS;
						alarmLevel  = '4';
//						configValue.moistureThresholdKey = FUNCITONG_TURN_ON;//20210308
//						serverConfig.storeFlagConfig++;			
					}		
					else
					{
						cigemsetsensorrangeflag	=	CIGEM_CMD_FLAG_err;
					}
				}
				cigemcmd = CIGEM_CMD_setsensorattr;			
			}
			else if(contrastbuf(bufIn+ptrout,"getsensorattr",13) == 0)//$cmd=getsensorattr&sensor_id=value  
			{
				if(findsensorid(bufIn+ptrout,bufInlen-ptrout,&cigemsensoridindex)==0)
				{
					
				}
				cigemcmd = CIGEM_CMD_getsensorattr;			
			}
			else if(contrastbuf(bufIn+ptrout,"setworkmode&mode=",17) == 0)//$cmd=setworkmode&mode=value
			{
				rxi = bufIn[ptrout+17];
				cigemresultflag	=	CIGEM_CMD_FLAG_err;
				if((rxi==WORKMODE_RUN)||(rxi==WORKMODE_LOWPOWER)||(rxi==WORKMODE_PLUS))
				{					
					cigemworkmode	=	rxi;
					cigemresultflag	=	CIGEM_CMD_FLAG_ok;
					serverConfig.perSetFlag	=	CHANGE_FLAG;//cigemworkmodeadj();
					if(rxi==WORKMODE_RUN)//210105
					{
						cigemperiodrunmodeInit();
					}
					sampleTimesContinus	= 2;
				}
				cigemcmd = CIGEM_CMD_setworkmode;			
			}
			else if(contrastbuf(bufIn+ptrout,"getworkmode",11) == 0)//$cmd=getworkmode
			{
				cigemcmd = CIGEM_CMD_getworkmode;			
			}		
			else if(contrastbuf(bufIn+ptrout,"upgrade",7) == 0)//$cmd=upgrade&md5=value&size=value
			{
			#if(UPGRADE_FRIMWARE_CTR>0)
//				cigemupgradeflag	=	CIGEM_CMD_FLAG_none;
				if(cigemupgradecheck(bufIn,bufInlen,(channel+1))==0)
				{
					cigemcmd = CIGEM_CMD_upgrade;					
				}
			#endif
			}
			else if(contrastbuf(bufIn+ptrout,"supportsize",11) == 0)//$cmd=upgrade&md5=value&size=value
			{
				cigemcmd = CIGEM_CMD_supportsize;
			}		
			else if(contrastbuf(bufIn+ptrout,"getcmdversion",13) == 0)//$cmd=getcmdversion		
			{
				cigemcmd = CIGEM_CMD_getcmdversion;
			}
			else if(contrastbuf(bufIn+ptrout,"meteorologicalearlywarning",26) == 0)
			{//$cmd=meteorologicalearlywarning&level=value&effective_time=value&lon_range=value&lat_range=value
				cigemresultflag	=	CIGEM_CMD_FLAG_err;
				if(findwarning(bufIn+ptrout,bufInlen-ptrout,&cigemwarning)==0)
				{	
				#if((USART_USE_CTR>0)&&(BEEP_ALARM_CTR>0))
					alarmTimes 	= 1;	
					alarmbeepStatus	= ALARM_STATUS_FREE;
				#endif					
					cigemalarmdeal();

				}
				cigemcmd = CIGEM_CMD_warning;			
			}
			else if(contrastbuf(bufIn+ptrout,"alarm",5) == 0)//$cmd=alarm&type=rain&level=1
			{
				cigemresultflag	=	CIGEM_CMD_FLAG_err;
				if(findalarm(bufIn+ptrout,bufInlen-ptrout,&cigemwarning)==0)
				{
				#if((USART_USE_CTR>0)&&(BEEP_ALARM_CTR>0))
					alarmTimes 	= 1;	
					alarmbeepStatus	= ALARM_STATUS_FREE;
				#endif
					cigemalarmdeal();	
					alarmLevel	=	cigemwarning.level;					
				}
				cigemcmd = CIGEM_CMD_alarm;			
			}
			else if(contrastbuf(bufIn+ptrout,"broadcast",9) == 0)//$cmd=broadcast&b_num=1&b_size=2&b_content=04
			{
				cigemresultflag	=	CIGEM_CMD_FLAG_err;
			
				if(findbroadcast(bufIn+ptrout,bufInlen-ptrout,&b_times,&b_content)==0)
				{			
				#if((USART_USE_CTR>0)&&(BEEP_ALARM_CTR>0))
					alarmTimes	=	b_times;
					alarmLevel	=	b_content;
					cigemwarning.level = alarmLevel;	
					alarmbeepStatus	= ALARM_STATUS_FREE;
				#endif
					cigemalarmdeal();					
				}			
			
				cigemcmd = CIGEM_CMD_broadcast;			
			}
		#if(SECOND_MQTT_ADDR>0)
			else if(contrastbuf(bufIn+ptrout,"setplatform",11) == 0)//$cmd=setplatform&mqtt_addr=value&mqtt_port=value&client_id=value&username=value&password=value&http_addr=value&http_port=value&device_id=value&api_key=value&controlNo=xx&type=xxx
			{
				cigemresultflag	=	CIGEM_CMD_FLAG_err;	
				mqttSecondServerAddrRead(secondServerAddrBuf);
				if(findsetplatform(bufIn+ptrout,bufInlen-ptrout,secondServerAddrBuf)==0)
				{
					cigemresultflag	=	CIGEM_CMD_FLAG_ok;					
					cigemServerAddrAdj(secondServerAddrBuf);
					alarmSource = READ_SOURCE_CHANNEL2;
					alarmLevel  = '2';
				}			
				cigemcmd = CIGEM_CMD_setplatform;
			}
			else if(contrastbuf(bufIn+ptrout,"getplatform",11) == 0)//$cmd=getplatform&controlNo=value
			{
				cigemresultflag	=	CIGEM_CMD_FLAG_err;			
				if(findgetplatform(bufIn+ptrout,bufInlen-ptrout,&serverControlNo)==0)
				{
					cigemresultflag	=	CIGEM_CMD_FLAG_ok;
					cigemcmd = CIGEM_CMD_getplatform;
					alarmSource = READ_SOURCE_CHANNEL2;
					alarmLevel  = '2';
				}				
			}
		#endif
			if(cigemcmd != CIGEM_CMD_none)
			{
			#if(UPGRADE_FRIMWARE_CTR>0)
				if(upgradelinknum == (channel+1)&&(cigemcmd == CIGEM_CMD_reboot))//(cigemcmd != CIGEM_CMD_upgrade)&&(cigemcmd != CIGEM_CMD_supportsize)
				{
					cigemupgradeflag	=	CIGEM_CMD_FLAG_none;
					upgradelinknum 		= 0;
				}
			#endif
				result = 0;		
				printf("\r\n cigemcmd:%d\r\n",cigemcmd);
			}
		}		
	}
#if(UPGRADE_FRIMWARE_CTR>0)	
	else if(cigemcmdtype == 2)
	{
		if(cigemupgradeflag	==	CIGEM_CMD_FLAG_rx)
		{
			if(cigemupgraderecevie(bufIn,bufInlen)!=0)
			{
				cigemupgradeflag	=	CIGEM_CMD_FLAG_none;
				upgradelinknum 		= 0;
			}
		}
		cigemcmdtype = 0;
	}
#endif		
	return result;
}

/*****************************END OF FILE*****************************/
