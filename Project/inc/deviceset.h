#ifndef DEVICE_SET_H
#define DEVICE_SET_H
#include "stm32l1xx.h"
#include "sensor.h"

/*******************************************************************************************************
*
*	设备内部EEPROM存储信息位置
*
********************************************************************************************************/

//设备配置信息存储
#define DATA_EEPROM_START_ADDR           0x08080000
//设备硬件版本存储位置
#define DATA_EEPROM_HARDWARE_VER_ADDR   (DATA_EEPROM_START_ADDR+4)
//设备软件版本存储位置
#define DATA_EEPROM_SOFTWARE_VER_ADDR   (DATA_EEPROM_HARDWARE_VER_ADDR+4)
//设备固件更改标志存储位置
#define DATA_EEPROM_FIREWARE_CHG_ADDR   (DATA_EEPROM_SOFTWARE_VER_ADDR+4) 
//设备编号存储位置
#define DATA_EEPROM_ID_ADDR             (DATA_EEPROM_FIREWARE_CHG_ADDR+4)
//boot选择存储ID
#define DATA_EEPROM_BOOT_ID_ADDR           (DATA_EEPROM_ID_ADDR+4)
//设备相关信息存储位置，包括传感器数量、设备类型、
#define DATA_EEPROM_INF_ADDR            (DATA_EEPROM_BOOT_ID_ADDR+4)
//UART型设备相关信息存储位置，包括设备状态（未标定状态和标定状态）等未来可扩展信息,
#define DATA_EEPROM_SETINF_ADDR            (DATA_EEPROM_INF_ADDR+4)
//设备通信方式信息存储位置
#define DEV_COMMUNICT_SET_ADDRESS       (DATA_EEPROM_SETINF_ADDR+4)
//设备类型数据存储位置
#define DEV_TPEY_EEPROM_ADDR            (DEV_COMMUNICT_SET_ADDRESS+4) 
//水传感器温度传感器id存储位置
#define DATA_EEPROM_B20_ADDR            (DEV_TPEY_EEPROM_ADDR+4)//168

//#define DATA_EEPROM_B20_ADDR            (DEV_COMMUNICT_SET_ADDRESS+4)
#define SEND_DATA_ADDR									(DATA_EEPROM_B20_ADDR+168)	//100
#define SEND_DATA_FLAG_ADDR							(SEND_DATA_ADDR+100) 
//复位前存储数据
#define DATA_EEPROM_SAVE_BEGIN_ADDR			(SEND_DATA_FLAG_ADDR+4)
#define DATA_EEPROM_SAVE_END_ADDR				(DATA_EEPROM_SAVE_BEGIN_ADDR+400)//400
#define DATA_SAVE_SENSOR_ADDR_LEN				0//1
#define DATA_SAVE_SENSOR_LEN						((DATA_SAVE_SENSOR_ADDR_LEN+1)*4)
#define DATA_SAVE_MQTT_ADDR_LEN					50//48+2
#define DATA_SAVE_ADDR_LEN							51//1+0+48+2
#define DATA_EEPROM_SAVE_MQTT_ADDR			(DATA_EEPROM_SAVE_BEGIN_ADDR+DATA_SAVE_SENSOR_LEN)

//	flag+date+hour+rstStatus
#define DATA_EEPROM_MODBUSADDR_ADDR			(DATA_EEPROM_SAVE_END_ADDR+4)
#define DATA_EEPROM_SK60OFFSET_ADDR			(DATA_EEPROM_MODBUSADDR_ADDR+4)//210705
#define DATA_EEPROM_RESETIMES_ADDR			(DATA_EEPROM_SK60OFFSET_ADDR+4)//210708
#define SAVE_FLAG_NONE	0
#define SAVE_FLAG_DONE	0x5A
typedef union
{	
	struct
	{
		value_t cigemFlat;
		uint8_t	cigemmsgid[4][48];
		value_t	cigemcmdFlat;	
	}value;
	uint32_t buf[DATA_SAVE_ADDR_LEN-1];
}param_save_t;

#define PARAM_SAVE_LEN	sizeof(param_save_t);

#define DATA_EEPROM_KXYL_ADDR						(DATA_EEPROM_SAVE_END_ADDR+4)//8*8
#define DATA_EEPROM_FLASH_VER_ADDR			(DATA_EEPROM_KXYL_ADDR+64)//220908

#define DATA_EEPROM_DATA_END_ADDR				(DATA_EEPROM_FLASH_VER_ADDR+4)

#define DATA_EEPROM_END_ADDR       0x080803FF
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

#define REG_STATE                 0x10101018
//extern __IO uint32_t Address;
//extern __IO FLASH_Status FLASHStatus;

/*
读温度传感器相关标志位

*/
#define READ_TEMP_FLAG     0x01
#define READ_TEMP_IDEL     0x02
#define READ_TEMP_FAIL     0x04



#define EN_INT()          __enable_irq();     //系统开全局中断
#define DIS_INT()         __disable_irq();    //系统关全局中断 

/*
设备初始化参数设置
*/
#define DEVICE_UN_INIT			0x01
#define DEVICE_INIT					0x02
#define RTC_SAVE_NONE				0x00
#define RTC_SAVE_DONE				0x01
extern __IO uint8_t rtcSaveFlag;
extern void dealSetTime(void);
extern uint32_t findStrRing(uint8_t const *buf,uint8_t bufLength);

extern void uatt2Receive(void);
extern void requestSet(void);
extern void eepromTest(void);
extern uint8_t deviceInit(void);
extern void eepromValueWrite(uint32_t addressWr,uint32_t temp);
extern uint32_t eepromValueRead(uint32_t address);
extern void rtcTimeRs232Set(uint8_t *buf);
extern void devIdWrite(uint8_t *buf);

extern void devPerWrite(uint8_t *buf);
extern void rs232DevPerWrite(void);
extern void serialNumWrite(uint32_t addressWr,uint32_t num);
extern uint32_t readNum(uint32_t which);
extern void devInfWrite(uint8_t *buf);
extern void b20IdWrite(uint8_t *buf);
extern void devCommunictionSet(uint8_t *buf);
extern void devStatusSet(uint8_t *buf);
extern void uatt2UserReceive(void);

extern void devModbusAddrSave(void);

extern void devRs232BaudRateSet(uint8_t buf);

extern void devRegAuthCodeSet(void);
extern void devRegAuthCodeUnSet(void);

//extern  __IO uint16_t readIdCounter;

#define BOOT1_ID       0x00000001
#define BOOT2_ID       0x00000002

extern uint32_t readFireWareState(void);
extern void fireWareStateWrite(uint32_t state);
extern uint32_t readFireWareBootId(void);
extern void fireWareBootIdWrite(uint32_t state);
extern void eepromInint(void);
extern void fireWareVerionWrite(uint32_t state);
extern uint32_t readFireWareVerion(void);
extern void b20IdRead(uint8_t *idBuf,uint8_t idOrder);
/*
define the mcu state

*/

#define GPRS_COMMUNICTION_TYPE    0x01
#define GSM_SMS_COMMUNICTION_TYPE 0x02
#define RS232_COMMUNICTION_TYPE   0x04
#define CAN_BUS_COMMUNICTION_TYPE 0x08

#define USE_THIS_COMMUNITCTION      (0x01)
#define UN_USE_THIS_COMMUNICTION    (0x02)


#define SMS_SEND      0x01
#define SMS_UNSEND    0x02

//extern  uint8_t smsFlag;
//extern  uint8_t gprsSendFlag ;
//extern  uint8_t smsSendFlag  ;
//extern  uint8_t canSendFlag  ;

#define MCU_ON_STATE    0x01
#define MCU_OFF_STATE   0x02

//

#define REQUEST_DATA_FLAG              0x0001
#define REQUEST_FLASH_TEST             0x0002
#define REQUEST_FLASH_DATA_FLAG        0x0004
#define REQUEST_ERASE_FLASH_FLAG       0x0008
#define REQUEST_SET_TIME_FLAG          0x0010
#define REQUEST_SET_DEV_FLAG           0x0020
#define REQUEST_SET_RS232_FLAG         0x0040
#define REQUEST_SET_SLEEP_FLAG         0x0080
#define REQUEST_SET_SLEEP_TIME_FLAG	   0x0100

#define NO_REQUEST_DATA_FLAG        0x0000
extern void mqttParamSave(void);
extern void SaveData_SWReset(void);
extern void ReadData_SWReset(void);
extern void ClearData_SWReset(void);
extern void setTimeTask(DS1302_TIME setTime);
//extern __IO uint16_t  requestDataFlag;

extern void sk60OffsetValueSave(__IO float value);	
extern float sk60OffsetValueRead(void);	

//extern uint8_t timeBuf[8];

#if(KXYL_CTR>0)
	extern void kxylParamRead(void);
	extern void kxylParamSave(uint8_t index,float paramA,float paramB);
#endif
#endif
