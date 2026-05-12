#ifndef _MODE_H
#define _MODE_H
#include "stm32l1xx.h"
#include "stm32_mems_adapter.h"
#include "gprs.h"
#include "ds1302.h"
#include "ds18b20.h"

#define BOOT_SOFTWARE_EMPTEY      0x00000000
#define BOOT_SOFTWARE_WRITE       0x00000001
#define BOOT_SOFTWARE_PROGAM      0x00000002
#define BOOT_SOFTWARE_BOOT1       0x00000003
#define BOOT_SOFTWARE_OK          0x00000004

#define BOOT_SOFTWARE_VER0        0x000000E1
#define BOOT_SOFTWARE_VER2        (uint32_t)105200812//0x000000E1

#if defined (STM32L1XX_MDP)
	#define BOOT_SOFTWARE_VER1        BOOT_SOFTWARE_VER2//(uint32_t)105200812//0x000000E1
#else
	#define BOOT_SOFTWARE_VER1        BOOT_SOFTWARE_VER0//0x000000E1
#endif


#define MAX_RS232_BUF_SIZE     0x226


#define FRAME_ORDER_ADDRESS      0x01     //数据帧序号存储地址
#define FIR_COMMAND_ADDRESS      0x02
#define SEC_COMMAND_ADDRESS      0x03

#define FIREST_USER_DATA_ADDRESS  0x04	 //用户数据第一字节存储位置
#define SECOND_USER_DATA_ADDRESS  0x05	 //用户数据第二字节存储位置

#define PACKET_DATA_ADDRESS   0x0A
#define FIR_DEV_SET_COMMAND              0x01//设备设置主命令//
#define SEC_DEV_ID_SET_COMM              0x01//设置ID子命令
#define SEC_DEV_TIME_SET_COMM            0x02//设置设备RTC时钟子命令
#define SEC_DEV_COMMUNICT_SET_COMM       0x03//设置设备通信方式子命令
#define SEC_DEV_ABC_SET_COMM             0x04//设置设备土壤类型参数ABC
#define SEC_DEV_AW_SET_COMM              0x05//设置设备空气中、水中初始值子命令
#define SEC_DEV_CTR_PHONE_SET_COMM       0x06//设置设备管理电话号码子命令
#define SEC_DEV_SMS_PHONE_SET_COMM       0x07// 设置设备短信发送号码子命令
#define SEC_DEV_PERIOD_SET_COMM          0x08//设置设备数据发送周期子命令
#define SEC_DEV_GPRS_ADDR_SET_COMM       0x09//GPRS通信服务器地址
#define SEC_DEV_DEP_SET_COMM             0x0A//设备深度信息设置
#define SEC_DEV_FIRWARE_SET_COMM         0x0B//设置设备固件设置指示子命令
#define SEC_DEV_FIRWARE_SET_OK_COMM      0x0C//设置设备固件存储完毕指示子命令
#define SEC_DEV_STATUS_SET_COMM          0x0F//设备配置信息设置子命令
#define SEC_DEV_SN_SET_COMM              0x10//设备SN写入
#define SEC_DEV_AUTHCODE_SET_COMM        0x11//设备authcode写入
#define SEC_DEV_ERARE_BOOT_FLASH         0x12//擦除BOOT FLASH存储区域
#define SEC_DEV_STATUS_INIT              0x13//设备强制初始化命令

#define FIR_DEV_READ_COMM                0x02//设备相关数据读取主命令
#define SEC_DEV_DATA_READ_COMM           0x01//设备数据读取子命令
#define SEC_DEV_ABC_READ_COMM            0x02//设备土壤类型参数读取子命令
#define SEC_DEV_AW_READ_COMM             0x03//设备初始值读取子命令											
#define SEC_DEV_CTR_PHONE_READ_COMM      0x04//设备管理电话号码读取子命令
#define SEC_DEV_SMS_PHONE_READ_COMM      0x05//设备短信电话号码读取子命令
#define SEC_DEV_DEV_READ_COMM            0x06//设备相关信息读取，包括ID、节点数、数据发送周期

#define FIR_REC_EXIT_COMM                0x00//退出命令
#define FIR_ACK_COMM                     0x00//回应信息//
#define SEC_ACK_OK_COMM                  0x00
#define SEC_HEAD_ERROR_COMM							 0x01
#define SEC_LENGTH_COMM                  0x02//
#define SEC_TIMEOUT_COMM                 0x03//
#define SEC_CRC_ERROR_COMM               0x04

#define FIR_LINK_COMM                    0x04//通信命令
#define SEC_LINK_MASTER                  0x01//请求通信命令
#define SEC_UNLINK_MASTER                0x02//断开连接命令


#define DATA_UP_FLAG                     0x01//数据上行标志
#define DATA_DAOW_FLAG                   0x02//数据下行标志

//设备状态信息发送

#define  FIR_DEV_STATUS_DIS                        0x05
#define  SEC_DS18B20_DIS                           0x00
#define  SEC_SENSOR_DATA                           0x01
#define  SEC_GPRS_DIS                              0x02
#define  SEC_DEV_DIS                               0x03

#define RS232_SET_FLAG         0x01//通过RS232设置标志
#define GPRS_SET_FLAG          0x02//通过gprs设置标志
#define MQTT_SET_FLAG          0x03//通过MQTT 设置标志
//extern uint8_t rs232Buf[MAX_RS232_BUF_SIZE];

extern void recTask(uint8_t *buf);
extern void devFlashInfRead(void);
extern void serAddrTest(void);
extern void dataSent(uint8_t *buf,uint16_t bufLength,uint8_t firComm,uint8_t secComm,uint8_t *dataOrder);
extern void devStatusDataSend(uint8_t *buf,uint16_t bufLength,uint8_t *order);
extern void wakeupByRxConfig(uint8_t state);
extern void sensorWriteData(uint8_t *buf,uint32_t *address,uint16_t dataLength);
extern uint8_t flashDataRead(uint32_t dataCountern,uint32_t *readAddress);


extern void devSendPeriodSet(uint8_t* buf);
extern void devSendPeriodRead(void);
extern void devSendPeriodSetByInt(uint8_t setFlag);
extern int8_t sensorDataWrite(__IO uint32_t *address,uint8_t type);
extern void devCalCounterSetByInt(uint8_t setFlag);
extern void devCalCounterRead(DS1302_TIME *readTime);
extern void flashWriterTest(uint32_t baseAddress,uint32_t endAddress,uint16_t packSize);

extern void phoneStoreFlash(phone *setPhone,uint8_t setType);

extern uint16_t devMainFastRead(uint8_t *writeBuf);
extern void devAuthCodeWrite(uint8_t * buf,uint16_t bufLength);

extern void xyzValueStoreWithSensorData(uint32_t address,tg_ADXL345_TYPE value);
extern void gpsValueStoreWithSensorData(uint32_t address,gpsValue value);
extern int8_t  sensorDataSendFlagWrite(uint32_t address,DS1302_TIME time);
extern int8_t sensorDataSendSucJudge(uint32_t readAddress);
extern int8_t  gpsDataReadWithSensorFlash(uint32_t readAddress,gpsValue *gpsBuf);
extern int8_t  xyzDataReadWithSensorFlash(uint32_t readAddress,tg_ADXL345_TYPE *xyzBuf);
extern int8_t  sensorDataReadFromFlash(uint32_t readAddress,DS1302_TIME *time,value_t *allConuter,uint16_t *conuter,b20_value_t *b20Buf,soilwat *hlfBuf);
extern void devFlashErare(uint32_t startAddr,uint32_t endAddr);

extern int8_t devFireWareInfFlashRead(fireWareStruct *config);
extern void devFireWareInfFlashStore(fireWareStruct config);

//extern uint16_t  sensorDataReadFromFlashToBuf(uint32_t readAddress,uint8_t *readBuf);
extern int8_t  xyzDataReadFromFlashToBuf(uint32_t readAddress,uint8_t *readBuf,uint16_t *readLength);
extern int8_t  gpsDataReadFormFlashToBuf(uint32_t readAddress,uint8_t *readBuf,uint16_t *readLength);

extern uint32_t devGpsFlashWriteAddressFind(void);
extern void devGpsValueFlashStore(gpsValue loactionValue ,uint32_t *address);
//extern int8_t devGpsValueFlashRead(uint32_t readAddress,gpsValue *loactionValue);
extern uint32_t devXyzFlashWriteAddressFind(void);
extern void xyzValueStoreFlash(uint32_t *addr,tg_ADXL345_TYPE value);
extern int8_t xyzValueReadFlash(uint32_t addr,DS1302_TIME *readTime,tg_ADXL345_TYPE *value);
extern void devConfigValueFlashStore(devConfig config);
extern int8_t devConfigValueFlashRead(devConfig *config);
extern void devAbcStoreFlash(soilabc *curAbc);
extern void devfawStoreFlash(soilfaw *curFawValue);

extern int8_t flashStoreDataRead(uint32_t readAddr,uint16_t packSize,uint8_t *readBuf,uint16_t *dataLentth);
extern uint16_t FlashWrite(uint32_t baseAddr,uint32_t endAddr,__IO uint32_t *WriteAddr,uint8_t *buf,uint16_t length,uint16_t packSize);
extern int8_t firewareCheck(uint32_t readAddr);//210705

extern int8_t devParamterRead(paramter *config);
extern void devParamterStore(paramter config);
extern void firewareWrite(uint8_t *buf,uint32_t *writeAddr,uint16_t bufLength);
extern void gprsAddressRead(void);
extern void gprsAddressSet(void);
extern void devDepSetByGprs(void);
extern void little_end_to_big_end(uint8_t *data, uint8_t len);
extern void qxValuePrint(uint8_t *source);
extern void qxValueStoreWithSensorData(uint32_t address,DS1302_TIME readTime);//,uint8_t *source
extern int8_t qxValueReadFlash(uint32_t addr,DS1302_TIME *readTime,uint8_t *readBuf);
extern int8_t  qxDataReadWithSensorFlash(uint32_t readAddress,uint8_t *readBuf);

extern void sk60DataSendValueFlash(uint8_t *bufin,uint8_t num);
extern uint16_t formatSK60FlashData(uint8_t *bufOut,uint16_t index);

extern void 	gprsSendAddrSave(void);
extern uint16_t unSendPackNumAdj(uint32_t sendAddr);
extern void mqttSecondServerAddrRead(secondServerAddr * secondServerBuf);
extern void sensorRangeStore(uint8_t setFlag);
extern void mqttAddressSet(void);
extern void cigemUnsendDataClear(void);
extern void cigemParamInit(void);
extern void cigemServerAddrAdj(secondServerAddr * secondServerBuf);
extern void cigemServerLinkClose(void);

extern void sensorValueCurrent(void);
extern void sensorDataWriteAddrStore(uint8_t setFlag);
extern void sensorDataWriteAddrRead(void);

extern void mqttSendAddrSave(void);
extern void clearSecondAddrBuffer(secondServerAddr * secondServerBuf);
extern void clearSecondAddrStore(void);
extern void mqttSecondAddrSave(secondServerAddr *serverbuf);
extern int8_t mqttSecondAddrRead(secondServerAddr *serverbuf);
extern void cigemServerAddrdeal(void);
extern void gprsApnSave(uint8_t status);
extern void Config_Menu(void);

#if(UPGRADE_FRIMWARE_CTR>0)
extern void mqttFirewareClear(void);
extern void mqttFirewareSave(uint16_t index,uint8_t *buf,uint16_t buflen);
extern int8_t mqttFirewareSaveTask(uint8_t *buf,uint16_t firewarenum);
#endif


#endif
