
#ifndef _HYDROLOGY_H
#define _HYDROLOGY_H

#include "stm32l1xx.h"

/*
水文规约控制字符


*/

// #define SOH            0x7E7E     //HEX/BCD编码报文起始帧
// #define STX            0x02       //传输正文开始
// #define SYN            0x16       //多包传输正文开始
// #define ETX            0x03       //报文结束，后续无报文
// #define ETB            0x17       //报文结束，后续有报文
// #define ENQ            0x05       //询问
// #define EOT            0x04       //传输结束，退出
// #define ACK            0x06      //肯定确认，继续发送
// #define NAK            0x15      //否定应答，反馈重发
// #define ESC            0x1B     //传输结束，终端保持在线

/*
遥测站信息表
*/
typedef struct{
      uint8_t  centerAddr;     //中心地址0~255
      uint8_t  slaveAddr[5];  //遥测站地址
      uint16_t  key;               //遥测站密码
      uint8_t   slaveType;    //遥测站分类标示符
      uint8_t   waterId[8];    //土壤水分含水量标示符
      uint8_t   temperId[8]; //土壤温度表示法
      uint8_t   tempId;         //地表温度标识符
      uint8_t   workType;    //遥测站工作方式
      uint8_t   communictionId;//通信识别号
      
}slaveInf;


/*
上下行标示符
*/
#define UPLINK_IDENTIFIER     0xFFF0  //高四位0000表示上行

#define DOWNLINK_IDERTIFIER   0xFFF8   //低四位0001表示下行

/*
上行字段标示符
*/

#define EARTH_TEMPERATURT                  0x0D      //地温标示符
#define THE_10CM_SOIL_MOISTURE             0x10      // 10cm土壤水分
#define THE_20CM_SOIL_MOISTURE             0x11      // 20cm土壤水分
#define THE_30CM_SOIL_MOISTURE             0x12      // 30cm土壤水分
#define THE_40CM_SOIL_MOISTURE             0x13      // 40cm土壤水分
#define THE_50CM_SOIL_MOISTURE             0x14     // 50cm土壤水分
#define THE_60CM_SOIL_MOISTURE             0x15     // 60cm土壤水分
#define THE_80CM_SOIL_MOISTURE             0x16     // 80cm土壤水分
#define THE_100CM_SOIL_MOISTURE            0x17     // 100cm土壤水分



#define THE_10CM_TEMPERATURT        0xFF10      // 10cm土壤温度
#define THE_20CM_TEMPERATURT        0xFF11      // 20cm土壤温度
#define THE_30CM_TEMPERATURT        0xFF12      // 30cm土壤温度
#define THE_40CM_TEMPERATURT        0xFF13      // 40cm土壤温度
#define THE_50CM_TEMPERATURT        0xFF14     // 50cm土壤温度
#define THE_60CM_TEMPERATURT        0xFF15     // 60cm土壤温度
#define THE_80CM_TEMPERATURT        0xFF16     // 80cm土壤温度
#define THE_100CM_TEMPERATURT       0xFF17     // 100cm土壤温度



#define LONGITUDE                   0xff10      //设备经度
#define LATITUDE                    0xff11      //设备纬度
#define TIMES_OF_COLLECTION         0x54     //观测时间标示符

#define BATTERY_VOLAGE              0xff13      // 40cm土壤温度
#define INPUT_VOLAGE                0xff14     // 50cm土壤温度

#define X_AXIS                      0xff15     // 60cm土壤温度
#define Y_AXIS                      0xff16     // 80cm土壤温度
#define Z_AXIS                      0xff17     // 100cm土壤温度


#define RSSI_VALUE                  0xff17     // 100cm土壤温度
#define GSM_REGISTRATION_ERROR_VALUE      0xff17     // 100cm土壤温度
#define GSM_LAC_VALUE               0xff17     // 100cm土壤温度
#define GSM_CELL_ID_VALUE           0xff17     // 100cm土壤温度


#define GPRS_REGISTRATION_ERROR_VALUE      0x17     // 100cm土壤温度

#define TCP_LINK_ERROR_VALUE               0x17     // 100cm土壤温度
#define TCP_SEND_ERROR_VALUE               0x17     // 100cm土壤温度
#define TCP_RX_ERROR_VALUE                 0x17     // 100cm土壤温度
#define TCP_RX_SUC_VALUE                   0x17     // 100cm土壤温度


/*
设备基本配置标示符
*/

/*
遥测站配置标示符
*/

#define CENTER_ADDR_ID              0x01  //
#define SLAVE_ADDR_ID               0x01  //
#define KEY_ID                      0x01  //
#define CENTER_MASTE_CHANNEL_ID     0x01  //
#define CENTER_BACKUP_CHANNEL_ID    0x01  //
#define WORK_TYPE_ID                0x01  //
#define MEASURE_ELEMENTS_ID         0x01  //
#define RELAY_ID                    0x01  //
#define TIME_INTERVAL_ID            0x20  //
#define ADDER_INTERVAL_ID           0x21   //

/*
遥测站运行参数配置标示符
*/

#define STORAGE_DATA_INIT     0x01  //
#define FACTORY_RESET    0x01  //
#define SOIL_PARAMETERS     0x01  //
#define DATA_MESSAGE_SWITCH     0x01  //
#define DATA_RECEIVE_NUMBER     0x01  //
#define DATA_ACQUISTITION_CYCLE  0x01  //
#define DATA_TRANSMISSION_DENSITY     0x01  //
#define FIXED_TIME_TO_SEND    0x01                                 //
#define GPS_POSITION    0x20                           //
#define ANTI_THEFT_SWITCH  0x21   //
#define MAINTAIN_NUMBER    0x01                                 //
#define ADD_NEWS_SWITCH    0x20                           //
#define ADD_NEWS_1THRESHOLD  0x21   //
#define ADD_NEWS_2THRESHOLD  0x21   //

#define SENSORID_USE_FLAG            0x01
#define SENSORID_UNUSE_FLAG        0x02
typedef struct{
     uint8_t idUseFlag;
     uint8_t waterDataId;
     uint32_t tempDataId;
}sensorId;


typedef struct{
     uint16_t upSerialNum;
     uint16_t downSerialNum;
     uint32_t confChangeFlag;
     uint32_t requestFlag;
   
     uint32_t requestErrorFlag;    
     uint32_t confChangErrorFlag;
     uint16_t configCounter;
   
}hydrology;

extern uint16_t  bvTempValue;

//extern sensorId devSensorId[9];

extern uint8_t frReadFromFlash(uint32_t readAddr,uint8_t *readBuf,uint8_t *datType);
extern uint16_t packetCrc(uint8_t *buf,uint16_t index);
extern uint16_t frLiaisonPackage(uint8_t *buf);

extern void stationAddressTest(void);
extern void crcTestPacket(void);
extern uint16_t frHeadFromat(uint8_t *buf);
extern int8_t frMultSensorDataLoad(uint8_t *buf,uint32_t sendAddr, uint16_t *lentth,uint16_t readSendPacket,uint8_t *sendPacket,uint8_t *dataType);
extern uint16_t testModePacketEnd(uint8_t *buf,uint16_t index);


#endif
