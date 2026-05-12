#ifndef SENSOR_H
#define SENSOR_H

#include "stm32l1xx.h"
#include "ds1302.h"
#include "main.h"
//#include "sensor.h"
/*******************************************************************************************************
*
*  传感器设置信息相关变量及数据定义
*
*******************************************************************************************************/
//传感器出厂编号
#define UN_INIT_DEVID       	0x0F000000
#define DEBUG_DEVID        	 	0x09

#define DEV_MAX_NODE_NUM			10	//设备最多节点数
#define UN_INIT_NUM         	8		//传感器出厂传感器数量
#define HIGHT_LOW_FRE_NUM_CTR	1	//1   仅采集高频水分数值  2   同时采集低频数值
//高低频率控制
#define HIG_FRE_ONLY          0x01
#define HIG_LOW_FRE_BOTH      0x02
#define UN_INIT_SEN_NUM     	HIG_FRE_ONLY

#define ALARM_NODE_1	0
#define ALARM_NODE_2	1

//默认数据发送周期
#define UN_INIT_TIME        3600//300

#define MIN_WATER_FR     (90000000.00) 	 //最小频率值
#define MAX_WATER_FR     (210000000.00)     //最大频率值

#define UN_INIT_WATER_FA	198000000
#define UN_INIT_WATER_FW	126000000

#define MIN_LOW_FR       (9000000)
#define MAX_WATER_VALUE    (99.9999)	 //水分最大值
#define MIN_WATER_VALUE    (0.00001)	 //水分最小值

typedef union
{
  uint32_t i;  /*取值范围0~~4294967295*/
  uint16_t j;
  float f;
  struct{

    uint8_t value1;	   /*取值范围0~~255*/
    uint8_t value2;	   /*取值范围0~~255*/
    uint8_t value3;	   /*取值范围0~~255*/
    uint8_t value4;	   /*取值范围0~~255*/
  }bytes;
  struct{

		uint16_t valueL;
    uint16_t valueH;  
  }words;
	uint8_t buf[4];
}value_t;

#define VALUE_TYPE_LENGTH     sizeof(value_t)

typedef union
{
  long i; 
  double f;
  struct{
    uint8_t value1;
    uint8_t value2;
    uint8_t value3;
    uint8_t value4;
		uint8_t value5;
    uint8_t value6;
    uint8_t value7;
    uint8_t value8;
  }bytes;
  uint8_t buf[8];	
}doubleV_t;
#define DOUBLE_TYPE_LENGTH     sizeof(doubleV_t)
// 设备初始值
typedef struct{
    value_t  Fa;
	value_t  Fw;
	value_t  Faw;
//     value_t  Fal;
// 	value_t  Fwl;
// 	value_t  Fawl;

}soilfaw;
#define   SOIL_FAW_LENGTH      sizeof(soilfaw)
// 设备初始值缓冲区
extern soilfaw curFaw[DEV_MAX_NODE_NUM];


//设备土壤类型参数
typedef struct{
     value_t a;
	 value_t b1;
	 value_t b2; 
	 value_t c;

}soilabc;

#define   SOIL_ABC_LENGTH      sizeof(soilabc)
//
typedef struct{
    value_t dep;
}devDep;

#define   DEP_LENGTH      sizeof(devDep)

extern devDep  curDep[DEV_MAX_NODE_NUM];
//设备土壤类型参数缓冲器
extern soilabc curAbc[DEV_MAX_NODE_NUM];

//设备水分、频率值
typedef struct{
   value_t fh;
	 value_t fl;
	 value_t wat;
}soilwat;

 
//设备水分、频率值缓冲区
extern soilwat curWat[DEV_MAX_NODE_NUM];
//前一次水分、频率值
extern value_t oldWat[2];
extern value_t oldWatSum,curWatSum;

//设备水分频率采集值、温度校准后频率值、计算后SF值
typedef struct{
     value_t fh;
	 value_t fht;
	 value_t sf;
}soilwatT;

//设备水分、频率值温度校准后缓冲区
//extern soilwatT curWatT[DEV_MAX_NODE_NUM];
//

/*
发送周期以分钟为单位进行调整，设备以十秒为最小单位进行休眠。
最小发送周期5分钟；
最大发送周期 （65535*10）/60= 10922分钟，即182小时，7天


*/

#define PERIOD_SET_BY_RS232       0x01
#define PERIOD_SET_BY_SERVER      0x02
#define DEV_SELF_SET_FLAG         0x03
//发送周期存储结构体
typedef struct{
     
	 value_t period;         //发送周期存储
	 uint8_t setTime[6];     //设置发送周期的时间
	 uint8_t setSourceFlag;  // 设置发送周期的来源（即通信方式），服务器、RS232等其他通信方式
}sendPer;

#define SEND_PERIOD_LENGTH     sizeof(sendPer)

//extern sendPer devPeriod;


typedef struct{
     //value_t x;
	// value_t y;
	// value_t z;
	 uint8_t xyzIntStatue;
	 uint8_t setSourceFlag;
	 uint8_t dataTypeFlag;
	 //uint8_t setTime[6];
}xyz_t;

#define XYZ_LENGTH            sizeof(xyz_t)
extern	xyz_t	adxlValue;

typedef union
{
  int16_t i;
  float f;

  struct{

    uint8_t valueH;
    uint8_t valueL;
	uint8_t valueH1;
	uint8_t valueH2;
  }bytes;

}iic_value_t;

typedef struct
{
	DS1302_TIME time;
	value_t		devSatus;
	value_t		countAll;
	value16_t count;
	value_t 	batV;
	value_t		inputV;
	value_t		t;
	value_t		rain;
	value_t		rainDaily;
	value_t		soilt[DEV_MAX_NODE_NUM];
	value_t		fh[DEV_MAX_NODE_NUM];
	value_t		fl[DEV_MAX_NODE_NUM];
	value_t		wat[DEV_MAX_NODE_NUM];
	value_t		ax;
	value_t		ay;
	value_t		az;
	uint8_t		sign[20];
	value_t		qxt;
	doubleV_t	dipX;
	doubleV_t	valX;
	doubleV_t	origX;
	doubleV_t	dipY;
	doubleV_t	valY;
	doubleV_t	origY;
	doubleV_t	dipYaw;
	doubleV_t	valYaw;
	doubleV_t	origYaw;
	value_t		angle;
	value_t		sk60t;
	value_t		sk60v;
	value_t		sk60dis;
	value_t		sk60disp;
	value_t		sk60status;
	value_t		kxylv1;
	value_t		kxylv2;
	doubleV_t	longitude;
	doubleV_t	latitude;
	value_t 	dipZ;
#if(KXYL_SENSOR_NUM>3)	//221128
	value_t		kxylv3;
	value_t		kxylv4;
#endif
}
sensorValue_t;
extern __IO sensorValue_t sensorValue;

//extern __IO value_t requesData;


/*****************************************************************************************************
*
*                     电池电压监测引脚定义
*
*****************************************************************************************************/



#define BATTYPE				1

#if    (BATTYPE > 2)
//#define BAT_ALARM_VALULE		1216
#define BAT_ALARM_VALULE			3.3		//1100         //3//3.3v
#define BAT_LOWER_VALULE			3.15	//1055        //3//3.15v
#define BATVALUE_LONGSLEEP		2.95
#else
//#define BAT_ALARM_VALULE		4
#define BAT_ALARM_VALULE			3.15	//1055       		//3.15v
#define BAT_LOWER_VALULE			3.05	//20210407_3.05	//2.95	//1017        //3//3.05v
#define BATVALUE_LONGSLEEP		2.8
#endif



/*******************************************************************************************************
*
*
*
*******************************************************************************************************/

#define SENSOR1        0x01
#define SENSOR2        0x02
#define SENSOR3        0x03
#define SENSOR4        0x04
#define SENSOR5        0x05
#define SENSOR6        0x06
#define SENSOR7        0x07
#define SENSOR8        0x08
#define SENSOR9        0x09
#define SENSOR10       0x0A

#define SENSOR_ON     0x00
#define SENSOR_OFF    0x01

#define SENSOR1_POWER    GPIO_Pin_0			 //SENSOR1 POWER CONTROL GPIO PIN
#define SENSOR2_POWER    GPIO_Pin_1			 //SENSOR1 POWER CONTROL GPIO PIN
//#define SENSOR2_POWER    GPIO_Pin_2			 //SENSOR1 POWER CONTROL GPIO PIN

#if defined (NEW_VERSION)
#define SENSOR4_POWER    GPIO_Pin_14			 //SENSOR1 POWER CONTROL GPIO PIN

#else
#define SENSOR4_POWER    GPIO_Pin_13			 //SENSOR1 POWER CONTROL GPIO PIN

#endif


#define SENSOR_PORTB_ALL_POWER     SENSOR1_POWER|SENSOR2_POWER|SENSOR4_POWER

#define RCC_GPIO_POWERB       RCC_AHBPeriph_GPIOB
#define GPIO_POWERB_PORT	  	GPIOB    		    //POWER CONTROL PORT

#define SENSOR3_POWER    GPIO_Pin_5			     //SENSOR1 POWER CONTROL GPIO PIN

#define SENSOR6_POWER    GPIO_Pin_8			     //SENSOR1 POWER CONTROL GPIO PIN
#define SENSOR8_POWER    GPIO_Pin_9			 //SENSOR1 POWER CONTROL GPIO PIN

#define SENSOR_PORTC_ALL_POWER     SENSOR6_POWER|SENSOR3_POWER|SENSOR8_POWER


#define RCC_GPIO_POWERC       RCC_AHBPeriph_GPIOC
#define GPIO_POWERC_PORT	  GPIOC    		    //POWER CONTROL PORT


#if defined (NEW_VERSION)
#define SENSOR10_POWER    GPIO_Pin_12			 //SENSOR1 POWER CONTROL GPIO PIN

#else
#define SENSOR10_POWER    GPIO_Pin_15			 //SENSOR1 POWER CONTROL GPIO PIN

#endif

#define SENSOR5_POWER    GPIO_Pin_11			 //SENSOR1 POWER CONTROL GPIO PIN

#define SENSOR7_POWER    GPIO_Pin_8			     //SENSOR1 POWER CONTROL GPIO PIN

#define  SENSOR_PORTA_ALL_POWER     SENSOR7_POWER|SENSOR10_POWER|SENSOR5_POWER
#define RCC_GPIO_POWERA             RCC_AHBPeriph_GPIOA
#define GPIO_POWERA_PORT	        GPIOA    		    //POWER CONTROL PORT


#define SENSOR9_POWER    					GPIO_Pin_2			     //SENSOR1 POWER CONTROL GPIO PIN

#define  SENSOR_PORTD_ALL_POWER		SENSOR9_POWER
#define RCC_GPIO_POWERD						RCC_AHBPeriph_GPIOD
#define GPIO_POWERD_PORT					GPIOD    		    //POWER CONTROL PORT


#define RCC_GPIO_SENSOR						RCC_AHBPeriph_GPIOC	//POWER CONTROL PORT
#define GPIO_SENSOR_PORT					GPIOC    		    //POWER CONTROL PORT

//the sensor2 

#define SENSOR2_INPUT                                    GPIO_Pin_6
#define SENSOR2_INPUT_EXTI_LINE                          EXTI_Line6
#define SENSOR2_INPUT_EXTI_PORT_SOURCE                   EXTI_PortSourceGPIOC
#define SENSOR2_INPUT_EXTI_PIN_SOURCE                    GPIO_PinSource6
#define SENSOR2_INPUT_EXTI_IRQn                          EXTI9_5_IRQn 

//the sensor3 

#define SENSOR1_INPUT                                   GPIO_Pin_7
#define SENSOR1_INPUT_EXTI_LINE                         EXTI_Line7
#define SENSOR1_INPUT_EXTI_PORT_SOURCE                  EXTI_PortSourceGPIOC
#define SENSOR1_INPUT_EXTI_PIN_SOURCE                   GPIO_PinSource7
#define SENSOR1_INPUT_EXTI_IRQn                         EXTI9_5_IRQn

#define SENSOR_INPUT_ALL       SENSOR1_INPUT|SENSOR2_INPUT  

/*

the high low ctr pin
*/



#if defined (NEW_VERSION)
	#define RCC_GPIO_FC                                 RCC_AHBPeriph_GPIOB
	#define GPIO_FC_PORT                                GPIOB    
	#define GPIO_FC                                     GPIO_Pin_15		
	#else
	#define RCC_GPIO_FC                                 RCC_AHBPeriph_GPIOA
	#define GPIO_FC_PORT                                GPIOA    
	#define GPIO_FC                                     GPIO_Pin_11	
	
#endif



	 //working led

//传感器id
extern  value_t devId;

//传感器数量
extern  uint8_t devSenNum;
//每节设备传感器数量
extern  uint8_t  eachDevNum;
//数据发送周期
//extern  value_t devSenTime;

extern  value_t devInfor;

//extern  value_t devType;
//设备通信方式配置参数
extern  value_t devCommunictionValue ;
//设备相关配置信息参数
extern  value_t devSetValue;

// extern  uint8_t gprsCommunictionFlag ;
// extern  uint8_t canCommunictionFlag  ;
// extern  uint8_t rs232CommunictionFlag;
// extern  uint8_t nfcCommunictionFlag  ;
// extern  uint8_t dataSmsSendFlag      ;
// extern  uint8_t ctrSmsSendFlag       ;
extern uint32_t gpsDataWriteAddr ;
extern uint32_t xyzDataWriteAddr ;

/*
数据存储地址定义
*/
extern  __IO uint32_t sensorDataWriteAddr;
extern  uint32_t sensorDataReadAddr;
extern  __IO uint32_t curGprsSendAddr;
extern __IO uint8_t sendOrder;
//

//传感器缓冲区最大字符
#if defined(STM32L1XX_MDP)
	#define MAX_SENSOR_BUF_SIZE		0x8FF//1126
#else
	#define MAX_SENSOR_BUF_SIZE		0x5FF//1126
#endif

// 传感器缓冲区定义
extern uint8_t sensorBuf[MAX_SENSOR_BUF_SIZE+1];

//单包数据帧包含数据包个数
extern uint8_t signPackNUm ;

/*设备总的采集次数计数器，掉电数据不丢失*/
extern  value_t calAllCounter;

/*设备开机以来的采集次数计数器，掉电数据丢失*/
extern  uint16_t calCounter;

//系统电池电压监测
extern __IO float batStatus;

#define AIR_TYPE        0x01
#define WATER_TYPE      0x02


extern void cal_sensor(void);
extern void cal_Num(void);
extern void cal_work(void);
extern uint16_t  devWater(void);
extern void calWaterTest(void);
extern float watSumValue(void);
extern float calFth(float tep,float fh);
extern float calSf(float Fa,float Faw,float fh);
extern float calWater(float sf,float soila,float soilb,float soilc);
extern int8_t devWaterOutput(soilwat *curWatValue,soilfaw *curFawValue,soilabc *curAbcValue);
extern void signPacketNumSel(void);
extern void depPrintf(void);
/*
串口设备相关信息
*/
#define DEV_UNSET_FLAG       0x01
//设备已标定状态（空气中、水中初始值已设定）
#define DEV_SET_FLAG         0x02

#define RS232_OUTPUT_SF            0x04
#define RS232_OUTPUT_WATER         0x08
#define RS232_OUTPUT_FRE		  0x10

extern  uint8_t devSetStatus;
extern  uint8_t devOutPutType;


/*
设备注册控制，注册后获得authcode
*/
#define DEV_UNREG              0x01
#define DEV_REG                0x02

extern  uint8_t devRegCtr;


/*dev sn*/
#define DEV_SN_SIZE                17
extern uint8_t devSnBuf[DEV_SN_SIZE+2];       
/*auth code*/
#define AUTH_CODE_SIZE             33
extern uint8_t authCodeBuf[AUTH_CODE_SIZE+1];


/*sensor id 
*传感器参数ID
*
*
*/
#define TEMPERATURE_ID                 34
#define HIG_FRE_ID                     32
#define LOW_FRE_ID                     33

#define BATTERY_ID                      50
#define TOTAL_COLLECT_COUNT_ID          51
#define COLLECT_COUNT_ID                52
#define SOLAR_VOLTAGE_ID                53
#define OUTSIDE_VOLTAGE_ID              54

#define AX_ID                          55
#define AY_ID                          56
#define AZ_ID                          57

#define GPS_LONGITUDE_ID                58
#define GPS_LATITUDE_ID                 59




#define GSM_RSSI_ID                      60
#define SIM_ERROR_COUNT_ID               61
#define GSM_ERROR__COUNT_ID              62
#define GSM_LAC_ID                       63
#define GSM_CELL_ID_ID                   64

#define SERVER_ID_ID								65
#define GPRS_REG_ERR_COUNT_ID				66
#define GPRS_BUSSINESS_ERR_COUNT_ID	67
#define GPRS_APN_ERR_ID							68
#define GPRS_WIRELESS_ERR_ID        69
#define OBTAIN_IP_ERR_ID            70
#define TCP_CONNECT_ERR_ID          71
#define TCP_SEND_ERR_ID             72
#define RECEIVE_ERR_ID              73
#define SEND_SUCCESS_COUNT          74

#define QX_DATA_SIGN					181	
#define QX_DIP_X							182	
#define QX_VAL_X							183
#define QX_ORIG_X							184
#define QX_DIP_Y							185
#define QX_VAL_Y							186
#define QX_ORIG_Y							187
#define QX_DIP_YAW						188
#define QX_VAL_YAW						189
#define QX_ORIG_YAW						190
#define QX_TEMP								194
#define QX_DIP_Z							280
#define QX_DIP_ANGLE					281
#define QX_STATUS							282

#define SIM_IMEI_ID						133//GSM模块IMEI身份码
#define SIM_ICCID_ID					134// ICCID为IC卡的唯一识别号码
#define SIM_IMSI_ID						135//国际移动用户识别码

#define SK60_T_ID							237
#define SK60_V_ID							238
#define SK60_DIS_ID						239
#define SK60_DISPLACEMENT_ID	240
#define SK60_STATUS_ID				241

#define SERVER_STATUS_ID			276	//第三方服务器连接状态
#define KXYL_VALUE1_ID				277//KXYL孔隙水压力
#define KXYL_VALUE2_ID				278//KXYL孔隙水压力
#define ALARM_STATUS_ID				279//预警信息

#define KXYL_VALUE3_ID				292//KXYL孔隙水压力 通道3
#define KXYL_VALUE4_ID				293//KXYL孔隙水压力	通道4

/*数据采集标志*/
#define SENSOR_DATA_FALG       0x01
#define SENSOR_DATA_IDEL       0x02

//extern __IO uint8_t newSensorDataFlag;

//未发送数据指示
extern uint16_t unSendDataPacket;
extern uint16_t	unSendDataPacketUSART;
//数据发送密度
//extern __IO uint8_t  transmissionDensity;


/*


*/
#define DEV_GSM_GPS           0x01
#define DEV_GSM               0x00
#define DEV_CDMA              0x02

extern  uint8_t devGpsCtr;


#define KEY_DATA_FLAG         0x01
#define RTC_DATA_FLAG         0x02
#define XYZ_DATA_FLAG         0x04
#define TEST_MODE_FLAG        0x08

extern uint8_t sensorDataFlag;

#endif
