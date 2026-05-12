/**
  ******************************************************************************
  * @file    Project/user/cigemPacket.h 
  * @author  insentek wxh
  * @version V1.0.0
  * @date    20-July-2020
  * @brief   Header for cigemPacket.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CIGEMPACKET_H
#define __CIGEMPACKET_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx.h"
#include "main.h"
#include "ds1302.h"
#include "gprs.h"
#include "mqttClient.h"
/* Exported typedef -----------------------------------------------------------*/

enum sensorStatus
{
	CHANNEL_NO_DATA		= -3,
	CHANNEL_DATA_ERR	= -2,
	CHANNEL_POWER_ERR	= -1,
	CHANNEL_ERROR_START	= 0,
};
//enum workmode
//{
//	WORKMODE_RUN			= '0',
//	WORKMODE_LOWPOWER	= '1',
//	WORKMODE_PLUS			= '2',
//};
enum warninglevel
{
	WARNING_LEVEL_RED			= '0',
	WARNING_LEVEL_ORANGE	= '1',
	WARNING_LEVEL_YELLOW	= '2',
	WARNING_LEVEL_BLUE		= '3',
	WARNING_LEVEL_NORMAL	= '4',
};
typedef struct{
	int sample;
	int upload;
	int plus;	
}cigemsensortime_t;
typedef struct{
	float threshold;
	float upper;
	float lower;	
}cigemsensorrange_t;
#define SENSOR_RANGE_VALUE_LENGTH sizeof(cigemsensorrange_t)

typedef struct{
	char 		level;
	uint8_t	time;
//	float		longitudemin;
//	float		longitudemax;
//	float		latitudemin;
//	float		latitudemax;
}cigemwarning_t;
#define cigemwaring_initializer	{0,0}//{WARNING_LEVEL_NORMAL,0,0.0,0.0,0.0,0.0}
/* Exported define ------------------------------------------------------------*/

#if(KXYL_CTR>1)
	#define CIGEM_CMD_VERSION	"104"
	#define SENSORID_INDEX_NUM  6
#elif(SK60_CTR>1)
	#define CIGEM_CMD_VERSION	"103"
	#define SENSORID_INDEX_NUM  5
#elif(ACCE_CTR>0)
	#define CIGEM_CMD_VERSION	"102"
	#define SENSORID_INDEX_NUM  4
#else	
	#define CIGEM_CMD_VERSION	"101"
	#define SENSORID_INDEX_NUM  3
#endif

#define DATA_TYPE_1					1
#define DATA_TYPE_2					2
#define DATA_TYPE_DEVSTATUS	DATA_TYPE_1
#define DATA_TYPE_SENSOR		DATA_TYPE_2

#define CIGEM_KEY_LEN		49
#define CIGEM_MSGID_LEN	48

#define CIGEM_TIME_SAMPLE_MIN	1			//0				//60  
#define CIGEM_TIME_SAMPLE_MAX	86400	//86400		//24_86400 //12_43200
#define CIGEM_TIME_UPLOAD_MIN	1			//0				//60  
#define CIGEM_TIME_UPLOAD_MAX	259200//14400_4	21600_6 43200_12 //259200_72
#define CIGEM_TIME_PLUS_MIN		1			//60_1分钟 
#define CIGEM_TIME_PLUS_MAX		86400	//14400_4_240分钟
#define CIGEM_TIME_PLUS_INIT	300
#define CIGEM_TIME_MIN_INIT		1

#define G_UNIT	1.0	//1.0_g 1000.0_mg

//#define SENSORID_INDEX_NUM  4
#define SENSORID_INDEX_QJ		0
#define SENSORID_INDEX_HS		1
#define SENSORID_INDEX_TW		2
#define SENSORID_INDEX_ACCE	3
#define SENSORID_INDEX_LF		4
#define SENSORID_INDEX_KXYL	5
#define SENSORID_QJ		"L1_QJ_1"
#define SENSORID_HS		"L3_HS_"
#define SENSORID_TW		"L3_TW_"
#define SENSORID_ACCE	"L1_JS_1"
#define SENSORID_LF		"L1_LF_1"
#define SENSORID_KXYL	"L3_SY_"
#define SENSORID_LEN_QJ		7
#define SENSORID_LEN_HS		6
#define SENSORID_LEN_TW		6
#define SENSORID_LEN_ACCE	7
#define SENSORID_LEN_LF		7
#define SENSORID_LEN_KXYL	6
#define SENSORID_QJ_AZI		"AZI"			//检测院平台
#define SENSORID_QJ_TREND	"trend"		//华测平台
#define SENSORID_QJ_DIP		SENSORID_QJ_AZI

#define ALARM_TYPE				"L1_QJ_1"
#define ALARM_TYPE_LEN		7
#define ALARM_TIME_MINUTE	60

#define CIGEM_CMD_none					0
#define CIGEM_CMD_reqtime				1
#define CIGEM_CMD_settime				2
#define CIGEM_CMD_getstatus			3
#define CIGEM_CMD_reboot				4
#define CIGEM_CMD_getsensorID		5
#define CIGEM_CMD_sample				6
#define CIGEM_CMD_setsensortime	7
#define CIGEM_CMD_reqsensortime	8
#define CIGEM_CMD_setsensorattr	9
#define CIGEM_CMD_getsensorattr	10
#define CIGEM_CMD_setworkmode		11
#define CIGEM_CMD_getworkmode		12
#define CIGEM_CMD_warning				13
#define CIGEM_CMD_upgrade				14
#define CIGEM_CMD_supportsize		15
#define CIGEM_CMD_getcmdversion	16
#define CIGEM_CMD_alarm					17
#define CIGEM_CMD_broadcast			18
#define CIGEM_CMD_setplatform		19
#define CIGEM_CMD_getplatform		20

#define CIGEM_CMD_FLAG_none	0
#define CIGEM_CMD_FLAG_req	1
#define CIGEM_CMD_FLAG_ok		2
#define CIGEM_CMD_FLAG_err	3
#define CIGEM_CMD_FLAG_waitrx	4
#define CIGEM_CMD_FLAG_rx			5
#define CIGEM_CMD_FLAG_rxok		6

#define FLOAT_VALUE_MIN			-1234.0
#define FLOAT_VALUE_ERR			-4321.0

#define ALARM_SOURCE_NONE				0
#define ALARM_SOURCE_RAIN				1
#define ALARM_SOURCE_WATER			2
#define ALARM_SOURCE_QX					3
#define ALARM_SOURCE_WATER_QX		4
#define ALARM_SOURCE_PLATFORM		5
#define ALARM_SOURCE_POWERERR		6
#define ALARM_SOURCE_NETWORKERR	7

#define READ_SOURCE_CHANNEL1		8
#define READ_SOURCE_CHANNEL2		9
#define READ_SOURCE_CHANNELAll	10
#define READ_SOURCE_STATUS			11

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern cigemsensortime_t	cigemsensortime;
extern cigemsensorrange_t	cigemsensorlimit[SENSORID_INDEX_NUM];
extern uint8_t cigemcmd;
extern uint8_t cigemcmdflag[MQTT_MAX_MULTI_CONNECTION_SIZE];
extern uint8_t cigemapikey[CIGEM_KEY_LEN];
extern uint8_t cigemmsgid[MQTT_MAX_MULTI_CONNECTION_SIZE][CIGEM_MSGID_LEN];
extern uint8_t cigemsettimeflag;
extern uint8_t cigemrebootflag;
extern uint8_t cigemsampleflag;
extern uint8_t cigemsetsensortimeflag;
extern uint8_t cigemsensoridindex;
extern uint8_t cigemsetsensorrangeflag;
extern char 		cigemworkmode;
extern uint8_t	cigemresultflag;
extern cigemwarning_t	cigemwarning;
extern uint8_t	cigemlinknumSave;

extern __IO uint8_t	alarmSource;
extern __IO uint8_t	alarmLevel;
extern uint32_t	cigemEarlyWarningTimes;
extern uint8_t cigemcmdLast;
extern uint8_t serverControlNo;
/* Exported functions ------------------------------------------------------- */	
extern void cigemSensorRangeInit(void);
extern void cigemConfigInit(void);
extern void cigemperiodrunmodeInit(void);
//extern void cigemSensorTimeChange(void);
extern void clearbuffer(uint8_t* buf,int buflen);
extern uint16_t cigemPacketSensorData(char *bufOut,char *clientId);
extern uint16_t cigemPacketDevstatus(char *bufOut);
extern uint16_t cigemPacketDevstatusPub(char *bufOut,char *clientId);

extern uint16_t cigemPacketAckapikey(char *bufOut,uint8_t *key,uint8_t *msgid);
extern uint16_t cigemPacketAckReqtime(char *bufOut,DS1302_TIME time);
extern uint16_t cigemPacketAckResult(char *bufOut,char *cmd,uint8_t flag);
extern uint16_t cigemPacketAckGetstatus(char *bufOut);
extern uint16_t cigemPacketAckGetsensorID(char *bufOut);
extern uint16_t cigemPacketAckSample(char *bufOut);
extern uint16_t cigemPacketAckdevPeriod(char *bufOut,uint8_t sensoridindex,cigemsensortime_t period);
extern uint16_t cigemPacketAckGetsensorattr(char *bufOut,uint8_t sensoridindex,cigemsensorrange_t limit);
extern uint16_t cigemPacketAckGetworkmode(char *bufOut,char mode);
extern uint16_t cigemPacketAckGetcmdversion(char *bufOut);
extern uint16_t cigemPacketAckGetplatform(char *bufOut);
extern int16_t cigemRxdataTask(uint8_t *bufIn,uint16_t bufInlen,uint8_t* bufOut,uint8_t channel);
extern void cigemworkmodeadj(void);
extern void cigemperiodTypeAdj(__IO uint8_t *periodType,__IO uint32_t *period,uint16_t *density);
extern int8_t contrastbuf(uint8_t *bufin,char*buf,uint8_t len);

#if(UPGRADE_FRIMWARE_CTR>0)
	#define MD5_BUFFSIZE							41	//四川平台MD5长度为32
#if defined(STM32L1XX_MDP)
	#define FRIMWARE_SIZE_MAX					245760	//256k-16k = 240k
	#define UPGRADE_PACKET_SIZE_MIN 	1383//FIREWARE_PACKET_SIZE//0x380_896
	#define UPGRADE_PACKET_SIZE_MAX 	1383//FIREWARE_PACKET_SIZE//0x380_896  四川平台分包大小为max-39
#else
	#define FRIMWARE_SIZE_MAX					118784	//128k-12k = 116k
	#define UPGRADE_PACKET_SIZE_MIN 	1063//FIREWARE_PACKET_SIZE//0x380_896
	#define UPGRADE_PACKET_SIZE_MAX 	1063//FIREWARE_PACKET_SIZE//0x380_896  四川平台分包大小为max-39
#endif
	#define UPGRADE_STATUS_NONE				0x00
	#define UPGRADE_STATUS_START			0x01
	#define UPGRADE_STATUS_ERROR			0x02
	#define UPGRADE_STATUS_OK					0x03
	extern 	__IO uint8_t 	upgradelinknum;
	extern 	uint8_t	cigemupgradeflag	;

	extern __IO uint16_t upgradepacketnum;
	extern __IO uint32_t	upgradebegintime;
	extern __IO uint32_t	upgradereceivetime;
	extern 	int16_t cigemdevidcheck(MQTTString topic,uint8_t channel);
	extern 	uint16_t cigemPacketAckUpgrade(char *bufOut);
//$cmd=reqtime&apikey=e7eb73db9cc982caeb1f&msgid=B6ZmaDbzA
//$cmd=settime&server=ntpserver
//$cmd=getstatus
//$cmd=reboot
//$cmd=getsensorID
//$cmd=sample
//$cmd=setsensortime&sensor_id=value&sample_intv=value&upload_intv=value&plus_intv=value
//$cmd=reqsensortime&sensor_id=value
//$cmd=setsensorattr&sensor_id=value&threshold=value&upper_limit=value&lower_limit=value
//$cmd=getsensorattr&sensor_id=value
//$cmd=setworkmode&mode=value
//$cmd=getworkmode
//$cmd=meteorologicalearlywarning&level=value&effective_time=value&lon_range=value&lat_range=value
//$cmd=upgrade&md5=value&size=value
//$cmd=supportsize&range=value
//$cmd=getcmdversion

#endif		
#endif /* __CIGEMPACKET_H */

/*****************************END OF FILE*****************************/
