#ifndef _GPRS_H
#define _GPRS_H
#include <stdio.h>
#include "stm32l1xx.h"
#include <stdint.h>
//#include "gprs.h"
#include "time.h"

#include "sensor.h"
//#include "ds1302.h"

#define GPRS_DEBUG    	1

#define SENSOR_DATA_FORMAT_KEY	0
#define MIN_PERIOD_CTR					1	//��С��������  Զ������51~54��Ӧ1~4����

#define GPRS_STATUS_DEBUG		1	//��ӡgprs״̬��Ϣ 	 0����ӡ 1��ӡ
#define SWRESET_CTR					1
#define GPS_TIME_CTR				0

#define GPRS_SIM_Mobile			0//�ƶ� 
#define GPRS_SIM_UNICOM_M2M	1//��ͨ��������
#define GPRS_SIM_TianYi			3//����

#define GPRS_SIM_TYPE				GPRS_SIM_Mobile
#define SEND_TYPE_GPRS			0x00
#define SEND_TYPE_USART			0x01
#define SEND_TYPE_MQTT			0x02

#define default_sever_addr 	"api.insentek.com"
#define	default_sever_port	54862

#define APN_SELECT_AUTO_CTR			1	//0 ����������APNĬ���ƶ�APN 1 ���ݿ���Ϣ�Զ�ѡ��APN
#define	GPRS_CONFIG_CTR					0	//0 ��������ʱ���������ã�1 ��������ʱ���������� 
#define GPRS_PING_CTR						0//0 ���� 1 ���������ӷ�����ǰ��ping�ٶȺͷ���������
#define SIM7600_3GPP_SET				1 //0 ������3gpp���ܣ�1 �ϵ�����3gpp	//211025 220716_1
#define DEV_APN_SET_CTR					1


#define LINK_NUM_INSENTEK				0
#define LINK_NUM_MQTT_1					1
#define LINK_NUM_MQTT_2					2
#define LINK_NUM_MQTT_3					3
#define LINK_NUM_MQTT_4					4
#define LINK_NUM_MQTT_Max				LINK_NUM_MQTT_4

#define APN_TYPE_AUTO						0
#define APN_TYPE_DEVSET					1
#define APN_TYPE_NETSET					2

#define GPRS_AT_ACKOK						1
#define APN_SAVE_STATUS_INIT		-1
#define APN_SAVE_STATUS_NONE		1
#define APN_SAVE_STATUS_DONE		2
#define APN_LEN_MAX							30

//#define GPRS_RETRY_TIMES			10
#define GPRS_RETRY_TIMES_AT			25 //210705
#define GPRS_RETRY_TIMES_NORMAL	10
#define GPRS_RETRY_TIMES_SIMOK	10
#define GPRS_RETRY_TIMES_SET		4//6
#define MQTT_RETRY_TIMES_RX			30

#define GPRS_TIMES_TRY					2
#define GPRS_TIMES_ALLTRY				3
#define GPRS_TIMES_RETRY_ALL		6

#define STATUS_NONE						0x00
#define STATUS_DONE						0x01

#define GPRS_USART						USART1
#define gprsSendBuffer				usart1StoreBuffer //usart1SendBuf //
#define gprsUSART_Config			USART1_Config
#define gprsUsartLowPower			uart1LowPower
#define devStoreBuffer				usart2StoreBuffer
#define devSendBuffer					usart2SendBuf
#define SWRESET_TIMER_INIT()	time9Counter = 0x00

#define GPRS_SLEEP_MODE_DIS    		0x01
#define GPRS_SLEEP_MODE_ENABLE   	0x02

#define GPRS_RX_DATA_SIZE      		0x7FF//20210118 0x3FF
#define FIREWARE_PACKET_SIZE			0x380//0x300//���ι̼����ݰ���󳤶�768
extern uint8_t GPRS_RX_Buffer[GPRS_RX_DATA_SIZE+1];
//#define MAX_GPRS_SERIVER_ADDERSS_SIZE      150
//��������ַ����
//extern uint8_t serAddrBuf[MAX_GPRS_SERIVER_ADDERSS_SIZE];

extern __IO uint8_t gprsPowerStatus;
extern __IO uint16_t gprsPowerOnTime;
//extern char apnBufInit[17];
extern __IO uint8_t  netlinkFlag ;
extern char  	apnBufSet[APN_LEN_MAX+2];
extern int8_t 	devApnSetStatus;
extern void gprsTask(void);
extern void gprsReceive(void);
extern void gprsTimeDeal(void);


//�绰������󳤶�
#define MAX_PHONE_NUM     15
//�绰����ṹ��
typedef struct{
   uint8_t phoneNumORG[MAX_PHONE_NUM];	//�绰����
	 uint8_t phoneNumSim[MAX_PHONE_NUM];	//���Ͷ��ź���
	 uint8_t phonelength;					//�绰���볤��
	 uint8_t phoneType;						//�绰��������
	 uint8_t phoneC;						//�绰������������
	 uint8_t phoneFlag;                     //�绰�����ʽ����־
	 uint8_t phoneUserFlag;                 //�绰�����û�ʹ�ñ�־
}phone;

#define   PHONE_NUM_LENGTH      sizeof(phone)

#define PHONE_EMPTY      0X01
#define PHONE_SET        0X02
#define PHONE_FORMAT     0X03
#define  MAX_CTR_NUM    	2	//����豸��������
#define MAX_USER_DATA_NUM	2	//����������û����ݺ������

//extern phone smsCenter;	 //�������ĺ���
//extern phone ctrNum[MAX_CTR_NUM];		 //�豸��������
//extern phone smsNum[MAX_USER_DATA_NUM];	 //�û����ݺ���


//�������ݻ���������ַ�
#define MAX_SMS_NUM          250
#define SMS_BUF_USE          0x01
#define SMS_BUF_IDLE         0X02

#define GPRS_ALATEM_DATA    0x01//GPRS��������
#define SENSOR_DATA_TYPE    0X02//����������
#define DEV_ALATEM_DATA     0X04//�豸��������
#define DEV_SIM_DATA      	0X08//�豸�������

#define EMPTEY_DATA_TYPE    0X00//
#define DEV_ALARM_DATA1     0x10//�豸�𶯱�������
#define DEV_ALARM_DATA2     0x30//�豸���Ʊ��γ���������
#define DEV_ALARM_DATA3     0x50//�豸���γ���������

/*
#define SMS_IDLE           		0x00
#define SMS_SENSOR_DATA_TYPE	0x01
#define GPRS_STATE_DATA_TYPE	0x02
*/

//�������ݻ�����
typedef struct{
   uint8_t dataLength;
   uint8_t smsLength;
   uint8_t dataFlag;
   uint8_t dataType;
   uint8_t sms[MAX_SMS_NUM];

}SMS_BUF;

#define MAX_SMS_BUF_SIZE	4	//���Ż��������������Ϊ4
extern SMS_BUF   smsBuf[MAX_SMS_BUF_SIZE];//�豸���ݶ��Ż�����

#define DEV_WATER_AIR_DATA          0x01//�豸ˮ�С������г�ʼֵ
#define DEV_FHONE_NUM_DATA          0x02//�豸�绰����
#define DEV_DEP_DATA                0x03//�豸�����Ϣ
#define DEV_ABC_DATA                0x04//�������Ͳ���A B C
#define DEV_FHONE_CTR_DATA          0x05//�豸���Ƶ绰����
#define DEV_SEND_PERIOD_DATA        0x06//��������
#define DEV_VALUE_DATA              0x07
#define DEV_CONFIG_DATA           	0x08
#define DEV_GPS_DATA                0x09
#define DEV_FIREWARE_INF_DATA   		0x0A
#define DEV_PARAMTER_DATA           0x0B
#define DEV_GPRS_ADDRESS_DATA       0x0C
#define DEV_XYZ_ADDRESS_DATA				0x0D
#define DEV_TEMPAB_DATA							0x0E
#define DEV_WINDPARAM_DATA					0x0F
#define DEV_BASELINE_DATA						0x10
#define DEV_SENSOR_RANGE_DATA				0x11
#define DEV_MQTT_ADDRESS_DATA       0x12
#define DEV_MQTT_SENDADDR_DATA      0x13
#define DEV_MQTT_ADDRESS2_DATA    	0x14

#define FLASH_ONE_PACKET_SIZE				256	 //д�ⲿFLASHÿ�����ݳ���
#define FLASH_BUF_NUM              	32	 //�洢���ñ�������������
#define FLASH_SECTOR_SIZE          	4096		 //ÿ�����ֽڳ���

//�ⲿFLASH ��ַ�����
#define EXT_FLASH_BASE_ADDR						0
//�豸�绰����洢��ʼ��ַ
#define PHONE_NUM_STORE_BASE_ADDR			EXT_FLASH_BASE_ADDR
#define PHONE_NUM_STORE_END_ADDR			(PHONE_NUM_STORE_BASE_ADDR + FLASH_SECTOR_SIZE*4)	
#define PHONE_CTR_STORE_BASE_ADDR		  (PHONE_NUM_STORE_END_ADDR + FLASH_SECTOR_SIZE*2)
#define PHONE_CTR_STORE_END_ADDR		  (PHONE_CTR_STORE_BASE_ADDR + FLASH_SECTOR_SIZE*2)
//�豸��ʼֵ�洢��ʼ��ַ
#define DEV_WATER_AIR_VALUE_BASE_ADDR	(PHONE_CTR_STORE_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_WATER_AIR_VALUE_END_ADDR	(DEV_WATER_AIR_VALUE_BASE_ADDR	+ FLASH_SECTOR_SIZE*2)
//�豸ABC�������Ͳ����洢��ʼ��ַ
#define ABC_VALUE_BASE_ADDR						(DEV_WATER_AIR_VALUE_END_ADDR + FLASH_SECTOR_SIZE*2) 
#define ABC_VALUE_END_ADDR						(ABC_VALUE_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸�����Ϣ�洢��ʼ��ַ
#define DEV_DEP_VALUE_BASE_ADDR				(ABC_VALUE_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_DEP_VALUE_END_ADDR				(DEV_DEP_VALUE_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸����״̬�洢��ַ
#define DEV_RUN_STATUS_BASE_ADDR			(DEV_DEP_VALUE_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_RUN_STATUS_END_ADDR				(DEV_RUN_STATUS_BASE_ADDR + FLASH_SECTOR_SIZE*2)
//MQTT���͵�ַ�洢��ַ
#define DEV_MQTT_SERVER_BASE_ADDR     (DEV_RUN_STATUS_END_ADDR )//+ FLASH_SECTOR_SIZE
#define DEV_MQTT_SERVER_END_ADDR      (DEV_MQTT_SERVER_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//GPRS���͵�ַ�洢��ַ
#define DEV_GPRS_SERVER_BASE_ADDR     (DEV_MQTT_SERVER_END_ADDR )//+ FLASH_SECTOR_SIZE
#define DEV_GPRS_SERVER_END_ADDR			(DEV_GPRS_SERVER_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸�̼��洢��ַ
#define DEV_FIREWARE_INF_BASE_ADDR		(DEV_GPRS_SERVER_END_ADDR + 2*FLASH_SECTOR_SIZE)
#define DEV_FIREWARE_INF_END_ADDR			(DEV_FIREWARE_INF_BASE_ADDR  + 2*FLASH_SECTOR_SIZE)
#if defined (STM32L1XX_MDP)
//�豸�̼�1�洢��ַ
	#define DEV_FIREWARE_BASE_ADDR			(DEV_FIREWARE_INF_END_ADDR + 2*FLASH_SECTOR_SIZE)
	#define DEV_FIREWARE_END_ADDR				(DEV_FIREWARE_BASE_ADDR + 150*FLASH_SECTOR_SIZE)//512bytes
//�豸�̼�2�洢��ַ
	#define DEV_FIREWARE2_BASE_ADDR			(DEV_FIREWARE_END_ADDR + 2*FLASH_SECTOR_SIZE)
	#define DEV_FIREWARE2_END_ADDR			(DEV_FIREWARE2_BASE_ADDR + 100*FLASH_SECTOR_SIZE)//896bytes
#else
//�豸�̼�1�洢��ַ
	#define DEV_FIREWARE_BASE_ADDR			(DEV_FIREWARE_INF_END_ADDR + 2*FLASH_SECTOR_SIZE)
	#define DEV_FIREWARE_END_ADDR				(DEV_FIREWARE_BASE_ADDR + 100*FLASH_SECTOR_SIZE)//512bytes
//�豸�̼�2�洢��ַ
	#define DEV_FIREWARE2_BASE_ADDR			(DEV_FIREWARE_END_ADDR + 2*FLASH_SECTOR_SIZE)
	#define DEV_FIREWARE2_END_ADDR			(DEV_FIREWARE2_BASE_ADDR + 150*FLASH_SECTOR_SIZE)//896bytes
#endif
//�豸�̼�2�洢��ַ //bootVersion=225ʱ
	#define DEV_FIREWARE0_BASE_ADDR			(DEV_FIREWARE_BASE_ADDR + 102*FLASH_SECTOR_SIZE)
	#define DEV_FIREWARE0_END_ADDR			(DEV_FIREWARE0_BASE_ADDR + 150*FLASH_SECTOR_SIZE)//896bytes
	
//�豸�����洢��ַ
#define DEV_PARAMTER_BASE_ADDR				(DEV_FIREWARE2_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_PARAMTER_END_ADDR					(DEV_PARAMTER_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸�ɼ����ݼ������洢��ַ
#define DEV_SENSOR_COUNTER_BASE_ADDR	(DEV_PARAMTER_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_SENSOR_COUNTER_END_ADDR		(DEV_SENSOR_COUNTER_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸���ݷ����ܶȴ洢��ַ
#define DEV_POWER_CTR_BASE_ADDR				(DEV_SENSOR_COUNTER_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_POWER_CTR_END_ADDR				(DEV_POWER_CTR_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//�豸���ᴫ�����洢��ַ
#define DEV_XYZ_BASE_ADDR							(DEV_POWER_CTR_END_ADDR + FLASH_SECTOR_SIZE*2)
#define DEV_XYZ_END_ADDR       				(DEV_XYZ_BASE_ADDR + FLASH_SECTOR_SIZE*2)
//�豸SN����洢��ַ
#define DEV_SN_BASE_ADDR              (DEV_XYZ_END_ADDR + 2*FLASH_SECTOR_SIZE) 
#define DEV_SN_END_ADDR               (DEV_SN_BASE_ADDR + 2*FLASH_SECTOR_SIZE)
//�豸MAINFEST�ļ��洢��ַ
#define DEV_MAINFEST_BASE_ADDR				(DEV_SN_END_ADDR + 2*FLASH_SECTOR_SIZE) 
#define DEV_MAINFEST_END_ADDR					(DEV_MAINFEST_BASE_ADDR + 2*FLASH_SECTOR_SIZE)
//�豸AUTH_CODE����洢��ַ
#define DEV_AUTH_CODE_BASE_ADDR				(DEV_MAINFEST_END_ADDR + 2*FLASH_SECTOR_SIZE) 
#define DEV_AUTH_CODE_END_ADDR				(DEV_AUTH_CODE_BASE_ADDR + 2*FLASH_SECTOR_SIZE)
//gpsλ����Ϣ�洢��ַ
#define DEV_GPS_BASE_ADDR             (DEV_AUTH_CODE_END_ADDR + 2*FLASH_SECTOR_SIZE) //
#define DEV_GPS_END_ADDR              (DEV_GPS_BASE_ADDR + 8*FLASH_SECTOR_SIZE)//32Kλ����Ϣ�洢�ռ�
//�豸������Ϣ�洢��ַ
#define DEV_CONFIG_BASE_ADDR          (DEV_GPS_END_ADDR + 2*FLASH_SECTOR_SIZE) //
#define DEV_CONFIG_END_ADDR           (DEV_CONFIG_BASE_ADDR  + 2*FLASH_SECTOR_SIZE)//8Kλ����Ϣ�洢�ռ�
//��ǰ���ݷ��͵�ַָ���ַ
#define GPRS_SENDADDR_BASE_ADDR				(DEV_CONFIG_END_ADDR		+ FLASH_SECTOR_SIZE)
#define GPRS_SENDADDR_END_ADDR				(GPRS_SENDADDR_BASE_ADDR	+ FLASH_SECTOR_SIZE*2)//*2
//GPRSAPN��ַ�洢��ַ
#define DEV_GPRS_APN_BASE_ADDR       		(GPRS_SENDADDR_END_ADDR )//+ FLASH_SECTOR_SIZE
#define DEV_GPRS_APN_END_ADDR        		(DEV_GPRS_APN_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//SENSOR_RANGE ��ַ�洢��ַ
#define SNESOR_RANGE_BASE_ADDR       		(DEV_GPRS_APN_END_ADDR )//+ FLASH_SECTOR_SIZE
#define SNESOR_RANGE_END_ADDR        		(SNESOR_RANGE_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//��ǰ���ݴ洢��ַָ���ַ
#define SENSOR_DATA_WRITE_BASE_ADDR			(SNESOR_RANGE_END_ADDR		)//+ FLASH_SECTOR_SIZE
#define SENSOR_DATA_WRITE_END_ADDR      (SENSOR_DATA_WRITE_BASE_ADDR	+ FLASH_SECTOR_SIZE*2)//*2
//��ǰmqtt���͵�ַָ���ַ
#define MQTT_SENDADDR_BASE_ADDR					(SENSOR_DATA_WRITE_END_ADDR		)//+ FLASH_SECTOR_SIZE
#define MQTT_SENDADDR_END_ADDR      		(MQTT_SENDADDR_BASE_ADDR	+ FLASH_SECTOR_SIZE*2)//*2
//MQTT�ڶ����͵�ַ�洢��ַ
#define DEV_MQTT_SERVER2_BASE_ADDR      (MQTT_SENDADDR_END_ADDR )//+ FLASH_SECTOR_SIZE
#define DEV_MQTT_SERVER2_END_ADDR       (DEV_MQTT_SERVER2_BASE_ADDR + FLASH_SECTOR_SIZE*2) 
//mqtt�̼�������ʱ����
#define MQTT_FIREWARE_BASE_ADDR					(DEV_MQTT_SERVER2_END_ADDR		)//+ FLASH_SECTOR_SIZE
#define MQTT_FIREWARE_END_ADDR      		(MQTT_FIREWARE_BASE_ADDR	+ FLASH_SECTOR_SIZE*75)//75*4=300 300*896=260k

#define FLASH_END_ADDR									SENSOR_DATA_WRITE_END_ADDR//SNESOR_RANGE_END_ADDR
//�豸���ݴ洢��ʼ��ַ
#define SENSOR_DATA_BASE_ADDR					(EXT_FLASH_BASE_ADDR + 768*FLASH_SECTOR_SIZE)
//�豸���ݴ洢������ַ
#define SENSOR_DATA_END_ADDR					8388608

#define BYTE_32_PACK_SIZE               32
#define BYTE_64_PACK_SIZE               64
#define BYTE_128_PACK_SIZE              128
#define BYTE_256_PACK_SIZE              256
#define BYTE_512_PACK_SIZE              512
#define BYTE_1024_PACK_SIZE             1024
#define BYTE_1536_PACK_SIZE							1536
#define BYTE_2048_PACK_SIZE             2048
#if defined(STM32L1XX_MDP)
	#define FIRE_WARE_STORE_SIZE2					BYTE_1536_PACK_SIZE
#else
	#define FIRE_WARE_STORE_SIZE2					BYTE_1280_PACK_SIZE
#endif
#define EXTERNAL_DATA_STORE_SIZE				BYTE_128_PACK_SIZE
#define MAIN_FAST_STORE_SIZE						BYTE_2048_PACK_SIZE
#define FIRE_WARE_STORE_SIZE						BYTE_1024_PACK_SIZE
#define SENSOR_DATA_STORE_SIZE          BYTE_512_PACK_SIZE 
#define SOIL_DATA_STORE_SIZE						BYTE_256_PACK_SIZE
#define GPRS_ADDR_STORE_SIZE						BYTE_1024_PACK_SIZE
#define MQTT_ADDR_STORE_SIZE						BYTE_1024_PACK_SIZE
#define MQTT_ADDR2_STORE_SIZE						BYTE_1024_PACK_SIZE
#define DEV_PARAMTER_STORE_SIZE					BYTE_256_PACK_SIZE
#define SOIL_DEP_STORE_SIZE							BYTE_128_PACK_SIZE
#define PHONE_NUM_STORE_SIZE						BYTE_128_PACK_SIZE
#define XYZ_DATA_STORE_SIZE							BYTE_128_PACK_SIZE
#define DEV_AUTH_CODE_STORE_SIZE				BYTE_64_PACK_SIZE
#define DEV_CONFIG_SOTRE_SIZE						BYTE_64_PACK_SIZE
#define GPS_DATA_STORE_SIZE							BYTE_64_PACK_SIZE
#define TEMPAB_STORE_SIZE								BYTE_64_PACK_SIZE
#define WINDPARAM_STORE_SIZE						BYTE_64_PACK_SIZE
#define BASELINE_STORE_SIZE							BYTE_64_PACK_SIZE

#define SENSOR_RANGE_STORE_SIZE					BYTE_256_PACK_SIZE
#define MQTT_SENDADDR_STORE_SIZE				BYTE_64_PACK_SIZE
#define CAL_COUNTER_STORE_SIZE					BYTE_32_PACK_SIZE
#define DEV_SN_STORE_SIZE								BYTE_32_PACK_SIZE
#define FIREWARE_INF_STORE_SIZE					BYTE_32_PACK_SIZE
#define SENSOR_DATA_FLAG_STORE_SIZE			BYTE_16_PACK_SIZE
#define GPRS_APN_STORE_SIZE          		BYTE_32_PACK_SIZE 
#define GPRS_SENDADDR_STORE_SIZE				BYTE_32_PACK_SIZE
#define SENSOR_WRITEADDR_STORE_SIZE		  BYTE_32_PACK_SIZE

//GPS ���ݴ洢ƫ����
#define GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST		140//
#define GPS_VALUE_SENSOR_DATA_SIZE								23//

//�������ݴ洢ƫ����
#define XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST		(GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST + 32)//
#define XYZ_VALUE_SENSOR_DATA_SIZE								18//6+12

//��б���ݴ洢ƫ����
#define QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST			(XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST + 32)//13+96
#define QX_VALUE_SENSOR_DATA_SIZE									102//

//SK60���ݴ洢ƫ����
#define SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST	(QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST + 120)//13+20
#define SK60_STORE_DATA_SIZE					20
#define KXYL_STORE_DATA_SIZE					8
#define EXTERNAL_STORE_DATA_SIZE			(6+SK60_STORE_DATA_SIZE+KXYL_STORE_DATA_SIZE)
#define KXYL_STORE_DATA2_SIZE					16
#define EXTERNAL_STORE_DATA2_SIZE			(6+SK60_STORE_DATA_SIZE+KXYL_STORE_DATA2_SIZE)


//���ͳɹ���־���ݴ洢ƫ��λ��
#define SENSOR_DATA_SEND_OK_FLAG_OFFSET				(SENSOR_DATA_STORE_SIZE - 20)//45
//#define SENSOR_DATA_SEND_OK_FLAG_OFFSET				(SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST + 45)//


#define END_CHARCCTER          0xFE

extern uint8_t idToUcs32(value_t id,uint8_t *buf);
extern uint8_t depToUcs32(float value,uint8_t *buf);
extern uint8_t numToUcs32(float value,uint8_t *buf);
extern void sensorDataBug(void);
extern void tempDataBug(void);
extern void phoneNumBug(void);
extern void depDataBug(void);
extern void fafwDebug(void);


extern uint16_t hexToAscii(uint8_t *bufIn,uint16_t inLength,uint8_t *bufOut);
extern float str2float(uint8_t *buf,uint8_t length);
extern uint32_t str2int(uint8_t *buf,uint8_t length);
extern uint16_t struct2Buf(void *data,uint8_t dataType,uint8_t *buf);
extern uint16_t buf2Struct(void *data,uint8_t dataType,uint8_t *buf,uint16_t bufLength);

//extern uint8_t phoneFormat(phone *num);

extern uint32_t findDataWriteAddr(void);
extern uint32_t findFlashWriteAddr(uint32_t baseAddress,uint32_t endAddr,uint16_t packSize,uint8_t *readBuf);
extern uint32_t findFlashReadAddr(uint32_t baseAddress,uint32_t endAddr,uint32_t writeAddress,uint16_t packSize,uint8_t *readBuf);

extern uint16_t FlashWrite(uint32_t baseAddr,uint32_t endAddr,__IO uint32_t *WriteAddr,uint8_t *buf,uint16_t length,uint16_t packSize);

extern uint16_t flashSensorDataRead(uint32_t startAddress,uint32_t endAddress,uint32_t *readAddress,uint8_t *buf,uint16_t packSize,uint16_t readCounter);
extern int8_t flashStoreDataRead(uint32_t readAddr,uint16_t packSize,uint8_t *readBuf,uint16_t *dataLentth);

//��������ַ�洢�ṹ��

#define TCP_SERVER_ADDRESS_SET_FLAG       0x01
#define TCP_SERVER_ADDRESS_UNSET_FLAG     0x02
#define TCP_SERVER_ADDRESS_USE_FLAG       0x01
#define TCP_SERVER_ADDRESS_UNUSE_FLAG     0x02

#define MAX_TCP_SERVER_SEND_ADDRESS_SINZE	0x3C
#define MAX_TCP_SERVER_ADDRESS_SIZE				0x28
#define MAX_MULTI_CONNECTION_SIZE        	0x01
#define TCP_SERVER_INSENTEK					(MAX_MULTI_CONNECTION_SIZE-1)

typedef struct{
	char setFlag;
	char userFlag;
  uint8_t packType;
  uint16_t sendPort;
  char  baseAddressBuf[MAX_TCP_SERVER_ADDRESS_SIZE];
  char  sendAddressBuf[MAX_TCP_SERVER_SEND_ADDRESS_SINZE];
}serverAddr;

#define GPRS_SEVER_ADDRESS_LENGTH     sizeof(serverAddr)
//��������ַ����
extern serverAddr tcpServerAddrBuf[MAX_MULTI_CONNECTION_SIZE];
#define STATION_MULTI_CONNECTION_SIZE        	0x01
extern serverAddr stationServerAddrBuf[STATION_MULTI_CONNECTION_SIZE];

#define SERVER_SENDADDRESS_SIZE	60
#define SERVER_PROTOCOL_SIZE		10
#define SERVER_ENDPOINT_SIXE		40
#define SERVER_PORT_SIZE				6
#define SERVER_DEVICEID_SIZE		20
#define SERVER_USERNAME_SIZE		20
#define SERVER_PASSWORD_SIZE		48//25
typedef struct{
  char 	userFlag;
	char	sendAddressBuf[SERVER_SENDADDRESS_SIZE+1];	
  char 	protocol[SERVER_PROTOCOL_SIZE+1];
  char  endpoint[SERVER_ENDPOINT_SIXE+1];
	uint16_t port;  
	char	deviceId[SERVER_DEVICEID_SIZE+1];
	char	username[SERVER_USERNAME_SIZE+1];
	char	password[SERVER_PASSWORD_SIZE+1];
}mqttServerAddr;

#define MQTT_SEVER_ADDRESS_LENGTH	sizeof(mqttServerAddr)
#if defined(STM32L1XX_MDP)
	#define MQTT_MAX_MULTI_CONNECTION_SIZE	4
#else
	#define MQTT_MAX_MULTI_CONNECTION_SIZE	4
#endif

extern mqttServerAddr mqttServerAddrBuf[MQTT_MAX_MULTI_CONNECTION_SIZE];
extern __IO uint8_t mqttServerNum;
extern __IO uint8_t mqttServerNumFirst;
extern __IO uint8_t mqttServerNumSecond;
extern __IO int8_t mqttSeverStatus[MQTT_MAX_MULTI_CONNECTION_SIZE];
extern __IO uint16_t unSendDataMqtt[MQTT_MAX_MULTI_CONNECTION_SIZE];
extern value_t	curMqttSendAddr[MQTT_MAX_MULTI_CONNECTION_SIZE];

typedef struct{
  char 	userFlag;	
  char  endpoint[SERVER_ENDPOINT_SIXE+1];
	uint16_t port;  
	char	clientId[SERVER_DEVICEID_SIZE+1];
	char	username[SERVER_USERNAME_SIZE+1];
	char	password[SERVER_PASSWORD_SIZE+1];
	char	httpaddr[SERVER_ENDPOINT_SIXE+1];	
  uint16_t 	httpport;
	char	deviceId[SERVER_DEVICEID_SIZE+1];
	char 	apikey[SERVER_PASSWORD_SIZE+1];
}secondServerAddr;//248

#define MQTT_SECOND_ADDRESS_LENGTH		sizeof(secondServerAddr)
#define SECOND_MULTI_CONNECTION_SIZE	2

#define UNSEND_FLAG           0x01
#define DATA_SEND_OK_FLAG     0x02
#define SEND_ERROR_FLAG       0x03

typedef struct{
   uint8_t linkTry;
   uint8_t sendTry;
   uint8_t rxTry;
   uint8_t rxDataTry;
   uint8_t setTry;
	 uint8_t allTry;
   uint8_t sendFlag;
	 uint8_t findIpTry;

   uint16_t gprsRegErrorCounter;
   uint16_t gprsAttachErrorCounter;
   uint16_t gprsBringUpErrorCounter;
   uint16_t gprsApnSetErrorCounter;
   uint16_t getIpErrorCounter;
   uint16_t linkErrorCounter ;
   uint16_t sendErrorCounter;
   uint16_t rxErrorCounter;
   uint16_t dataOkCounter;
   uint16_t rxDataErrorCounter;
}lsrLog;


extern lsrLog serverLSRLog[MAX_MULTI_CONNECTION_SIZE];
extern lsrLog stationServerLSRLog[STATION_MULTI_CONNECTION_SIZE];

typedef union
{
  uint32_t i;
  uint16_t j;
  uint64_t longi;
  double f;
  struct{

    uint8_t value1;	   /*ȡֵ��Χ0~~255*/
    uint8_t value2;	   /*ȡֵ��Χ0~~255*/
    uint8_t value3;	   /*ȡֵ��Χ0~~255*/
    uint8_t value4;	   /*ȡֵ��Χ0~~255*/
    uint8_t value5;	   /*ȡֵ��Χ0~~255*/
    uint8_t value6;	   /*ȡֵ��Χ0~~255*/
    uint8_t value7;	   /*ȡֵ��Χ0~~255*/
    uint8_t value8;	   /*ȡֵ��Χ0~~255*/
  }bytes;
}dou2Buf;

#define MAX_GPS_BUF_SIZE      3
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	#define FIND_LATITUDE_VALUE       1
	#define FIND_LAT_NS								2
	#define FIND_LONGITUDE_VALUE      3
	#define FIND_LONG_EW							4
	#define FIND_UTC_DATE             5
	#define FIND_UTC_TIME             6	
	#define FIND_ALTITUDE_VALUE      	7
	#define FIND_SPEED_VALUE          8  
	#define FIND_COURSE_VALUE        	9  
#else
	#define FIND_UTC_TIME             1
	#define FIND_LATITUDE_VALUE       2
	#define FIND_LONGITUDE_VALUE      3	
	#define FIND_ALTITUDE_VALUE      	4
	#define FIND_SPEED_VALUE          5  
	#define FIND_COURSE_VALUE        	6  
#endif	

//HTTP ���ش�����
#define HTTP_UNKNOW_ERROR   254
#define HTTP_TIME_OUT				253
#define HTTP_LINK_COLSE			252
#define HTTP_LINK_TIMEOUT		251
#define HTTP_OK       			0x00
#define HTTP_10001_ERROR      1
#define HTTP_10002_ERROR      2
#define HTTP_10003_ERROR      3
#define HTTP_10004_ERROR      4
#define HTTP_10005_ERROR      5
#define HTTP_10006_ERROR      6
#define HTTP_10007_ERROR      7
#define HTTP_10008_ERROR      8
#define HTTP_10009_ERROR      9

//tcp������
#define RESPONSES_OK					10000
#define TIME_ERROR						10003  //�ɼ�ʱ�����
#define SN_ERROR							10002  //�����SN
#define FILE_ERROR						10004  //�����ļ�����
#define REG_ERROR							10005  //ע�����
#define DEV_NOT_EXIT					10007
#define ATUTHCODE_ERROR				10010  //�豸������Ȩ�����
#define NODE_ERROR						10009  //�ڵ����
#define INDEX_ERROR						10008 //�ڵ���������
#define NETWORK_ERROR					10020	//�������
#define SERVER_ERROR					10011	//�������ڲ�����
#define SERVER_ENABLE_ERROR		10012	//�Ҳ������õķ���
#define SERVER_PARAM_ERROR		10013	//���������ƥ��
#define PROTOCOL_ERROR				10014	//�Ƿ���Э��
#define PROTOCOL_LEN_ERROR		10015	//�Ƿ���Э�鳤��
#define DATA_LEN_ERROR				10016	//��ȡ���ݺ�Э�鳤�Ȳ�ƥ��
#define NO_NEWFIRMWARE_ERROR	10017	//û���µĹ̼��汾
#define FIRMWARE_ERROR				10018	//�̼���ȡԽ��

#define CHANGE_FLAG              0x01   //��ֵ�ı��־
#define UNCHANGE_FLAG            0x02  //��ֵδ�ı�

typedef struct{
     int8_t   gsmStartResult;
     int8_t   gpsValueFlag;
		 uint8_t  rtcSetFlag;
		 uint8_t	perSetFlag;
		 uint8_t  simCardSmsFlag;
	   int8_t   firewareFlag;
	   uint8_t  dataSentFlag;
     uint8_t  storeFlagAbc;
     uint8_t  storeFlagConfig;
     uint8_t  storeFlagDataSmsPhone;
     uint8_t  storeFlagStationServer;
     uint8_t  storeFlagParam;
     uint8_t  storeFlagFaw;
     uint8_t  delFlagHost2;
     uint8_t  stationServerChangeFlag;
     uint8_t  storeFlagAlarmSmsPhone;
     uint8_t  storeFlagStationName;
     uint8_t  storeFlagStationPw;
     uint8_t  storeDepFlag;
     uint16_t contuineSendErrorCounter;
	 uint8_t  devApnSetFlag;//smsSensorDataFlag; //230508
	 uint8_t  storeFlagSensorRange;//smsSendDataType;
	 uint8_t  sendErrorCounter;
	 uint8_t  devState;
	 uint8_t  devAlarm;
	 uint8_t  devAlarmSmsFlag;
	 int8_t   devWaterRelValueState;
   uint8_t  rtcErrorFlag;
	 uint8_t  historyDataFlag;
	 uint16_t requestData;
	 uint8_t requestDataTimeMon;
	 uint8_t requestDataTimeDay;
	 uint8_t requestDataType;
	 uint32_t requestDataReadAddress;
	 uint8_t inItMode;
	 uint16_t inItModeCounter;
     uint8_t stationConfigChangeFlag;
     uint8_t sleepModeTry;
     uint8_t powerSavingKey;//gsmIntFlag;
     uint8_t callMode;
     uint8_t testMode;
     uint8_t staionResetFlag;
     uint8_t staionFlashResetFlag;
     uint16_t devDensity;//uint8_t devDensity;//201124
		 uint8_t  gsmRunMode;
     uint8_t simCardReadFlag;
     uint8_t simCardReadOkFlag;
		 uint8_t  getIpFlag;
		 uint8_t  linkErrorCouner;
		 uint8_t	insentekKey;
		 uint8_t 	networkMode;//210623
}httpConfig;
extern httpConfig serverConfig;
#define FIREWARE_IDLE           0x00
#define FIREWARE_QEQUEST        0x01
#define FIREWARE_UPDATA         0x02

#define REQUEST_DATA_BY_COUNTER	0x01
#define REQUEST_DATA_SEND				0x02
#define REQUEST_DATA_IDLE				0x03

#define GPRS_RUN     0x00
#define GPRS_SLEEP   0x01

extern void servStoreFlagInit(httpConfig *flagStore);
extern void flashStoreTask(httpConfig rxResult);
extern void httpConfigParameInit(httpConfig *rxResult);

#define GPS_VALUE_ERR        	-1
#define GPS_VALUE_OK        	0x01
#define GPS_VALUE_VALID				0x02
#define GPS_LOCATION_OFF			0x00
#define GPS_LOCATION_ON				0x01
#define GPS_LOCATION_INIT			GPS_LOCATION_OFF
#define GPS_ERRTIMES_RETRY		6//6	
#define GPS_ERRTIMES_LEVEL1		6//6	
#define GPS_ERRTIMES_LEVEL2		12//12	
#define GPS_INTERVAL_INIT			360//240		//���� 	6Сʱ
#define GPS_INTERVAL_LEVEL1		720//720		//����	12Сʱ
#define GPS_INTERVAL_LEVEL2		1440//1440	//����	24Сʱ
#define GPS_INTERVAL_MIN			5			//����
#define GPS_INTERVAL_MAX			4320	//72*60=4320����

#define NONE_SOURCE						0x00
#define GPS_SOURCE						0x01
#define AGPS_SOURCE						0x02
#define PLATFORM_SOURCE				0x03

#define BUF_NONE_FLAG					0x00
#define BUF_UNUSE_FLAG				0x01
#define BUF_USED_FLAG					0x02
#define BUF_USED_FLAG_LAST		0x03

#define SAVE_MODE_0						0x00
#define SAVE_MODE_1						0x01
#define GPS_SAVE_MODE					SAVE_MODE_0

#define DATA_HAVE_STORE_FLAG	0x01
#define DATA_UNSTORE_FLAG			0x00//0x02

typedef struct{
   uint8_t 	gpsSource;
	 uint8_t 	useFlag;
	 uint8_t 	usedTime;//storeFlag
	 uint8_t 	locationMode;
	 dou2Buf 	longitude;
	 dou2Buf 	latitude;
	 float   	altitude;
	 uint8_t 	saveMode;//ttff;
	 uint16_t utcmSec;//num;
	 uint16_t utcYear;
	 uint8_t  utcMon;
	 uint8_t  utcDay;	
	 uint8_t 	utcHour;
	 uint8_t 	utcMm;
	 uint8_t 	utcSec;
}gpsValue;
extern gpsValue gpsBuf[MAX_GPS_BUF_SIZE];
extern DS1302_TIME gpsRtcTime;
#define GPS_VALUE_LENGTH     sizeof(gpsValue)
extern double devLongitude ;
extern double devLatitude;

extern uint8_t findUnUseGpsBuf(void);


typedef struct{
     uint32_t fireWareLength;
 //    uint8_t  domainBuf[30];
 //    uint8_t  md5Buf[33];
 //    uint8_t  fileNameBuf[30];

}fireWareStruct;

#define FIREWARESTRUCT_LENGTH    sizeof(fireWareStruct)
extern fireWareStruct  curFireWare;
extern uint16_t gprsDataAverived(void);
extern void clearBuf(void);
extern void serverAddrDebug(void);
extern int8_t gsmTaskInit(uint8_t gprsSleepCtr);
extern int8_t gprsStart(void);
extern double gpsValueAdj(double data);
extern  int8_t gpsTask(gpsValue *curGpsValue,httpConfig *rxResult ,int8_t timeZone);
extern void serverBufInt(void);
extern int8_t gprsConfigRequest(httpConfig *rxResult);
extern void gprsSendRxTask(httpConfig * rxResult);
extern void gsmTaskEnd(uint8_t linknum);
extern void clearGpsValue(gpsValue *value);

#define NORMAL_PACKET    0x01		//�������ݽӿ�����
#define EXT_PACKET       0x02		//��չ���ݽӿ�����

#define XYZ_UNUSER_FLAG  0x01		//δʹ�����ᴫ����
#define XYZ_USER_FLAG    0x02       //ʹ�����ᴫ����

#define GSM_STATE_SEND_FLAG    0x01    //
#define GSM_STATE_UNSEND_FLAG  0x02    //

#define GPRS_STATE_SEND_FLAG   0x01	   //
#define GPRS_STATE_UNSEND_FLAG 0x02    //

#define GPS_VALUE_OK        0x01
#define GPS_VALUE_VALID			0x02

#define SENSOR_DATA           0x01
#define CONFIG_REQUEST_DATA   0x02
#define FIREWARE_REQUEST_DATA 0x04
#define DEV_REG_DATA          0x08 
#define DEV_HISTORY_DATA   		0x10
#define DEV_RTC_CAL_DATA   		0x20
#define LIAISON_DATA       		0x21

//������Ϣ��Ӧ��Ϣ
#define CONFIG_RE_DATA     		0x22
#define REQUEST_RE_DATA    		0x23
//�������
#define SENSOR_DATA_COLLECT			0x0001	//�ɼ�����֡
#define DEVICE_REGISTER					0x0002	//�豸ע��
#define FIRWARE_DOWNLOAD				0x0003	//�̼�����
#define CONFIG_REQUEST					0x0005	//�豸��������
#define SYNC_DEVICE_TIME				0x000B	//RTCʱ��У׼ 

//�������·������޸ı�־λ
#define NO_CHANGE            0x00
#define SEND_PERIOD_CHANGE   0x01
//������෢�ʹ���
#define SINGLE_SEND_MAX_PACKET_NUM1	0x02
#define SINGLE_SEND_MAX_PACKET_NUM2	0x03
#define SINGLE_SEND_MAX_PACKET_NUM3	0x04
#define SINGLE_SEND_MAX_PACKET_NUM4	0x05
#define SINGLE_SEND_MAX_PACKET_NUM5	0x06
								  
//�豸���ýṹ��
#define FUNCITONG_TURN_ON     0x31
#define FUNCITONG_TURN_OFF    0x30
#define FUNCITONG_INIT_USART	FUNCITONG_TURN_OFF
#define POWER_SAVING_KEY_INIT	FUNCITONG_TURN_OFF
#define FUNCITONG_KXYL_INIT		FUNCITONG_TURN_OFF

#define FUNCITONG_SK60_OFF			0x30
#define FUNCITONG_SK60_DISP			0x31
#define FUNCITONG_SK60_YEWEI		0x32
#define FUNCITONG_SK60_RAW			0x33
#define FUNCITONG_SK60_INIT			FUNCITONG_SK60_DISP

#define FUNCITONG_NETMODE_AUTO	0x30
#define FUNCITONG_NETMODE_2G		0x32
#define FUNCITONG_NETMODE_4G		0x34

#define INSENTEK_KEY_OFF			0x30
#define INSENTEK_KEY_CFG			0x31
#define INSENTEK_KEY_ON				0x32
#define INSENTEK_KEY_INIT			INSENTEK_KEY_ON
//#define TURN_ON_KEY					0x31
//#define TURN_OFF_KEY				0x30
#define MQTT_DATA_TYPE_CIGEM	0x30
#define MQTT_DATA_TYPE_STRING	0x31
#define MQTT_DATA_TYPE_TREND	0x32
#define MQTT_DATA_TYPE_INIT		MQTT_DATA_TYPE_STRING

#define MQTT_SECOND_ADDR_NUM_0	0x30
#define MQTT_SECOND_ADDR_NUM_1	0x31
#define MQTT_SECOND_ADDR_NUM_2	0x32
#define MQTT_SECOND_ADDR_NUM_INIT		MQTT_SECOND_ADDR_NUM_1

#define NORMAL_PERIOD_TYPE		0x31
#define FIX_TIME_SEND_TYPE		0x32

#define SEND_OK		0x01
#define SEND_ERR	0x02

typedef struct{
     uint16_t  transmissionDensity;//���ݷ����ܶ�
     uint16_t  devPeriod;//���ݷ������ڣ�ʱ��
     uint16_t  devPeriodMin;   //
     uint16_t  devPeriodHour;
     uint8_t    devPeriodType;//
		 //�˴���1λ
     uint16_t   xValue;      //����X����ֵ
     uint16_t   yValue;      //����Y����ֵ
     uint16_t   zValue;       //����Z����ֵ
     uint16_t   moistureThreshold1;//�����ˮ����ֵ1
     uint16_t   moistureThreshold2;//�����ˮ����ֵ2
     uint16_t  threshold1Period;//�����ˮ����ֵ1��������
     uint16_t  threshold2Period;//�����ˮ����ֵ2��������
     uint32_t  configVersion;//���ð汾��
     uint8_t    xyzSharkkey;//�����𶯿���
     uint8_t    gpsLocation;//GPS��λ����
     uint8_t    gpsKey;//GPS���ܿ���
     uint8_t    devWorkState;//�豸����ģʽ
     uint8_t    moistureThresholdKey;
     uint8_t    curfireWareId;
     uint16_t   moistureThreshold3;//�����ˮ����ֵ2
     uint16_t  threshold3Period;//�����ˮ����ֵ2��������
		  //�˴���2λ
}devConfig;

#define  CONFIG_DATA_LENGTH   sizeof(devConfig)
extern devConfig  configValue;

typedef struct{
   int8_t timeZone;// ����??
   char apnBuf[40];// APN����??
   uint8_t stationAddr[12];
   uint8_t stationKey[8];
   uint8_t stationPhone[15];
   uint8_t ceterAddr[6];
   uint16_t key;
   uint8_t stationMode;//ģʽ
   uint8_t workState;
   uint16_t configChangeCounter;//���������¼
   uint16_t pwChangeCounter;//�����޸Ĵ�����¼
   uint16_t bvLowCounter;//��ص͵�ѹ����
   uint16_t flashResetCounter;//��ʷ���ݳ�ʼ����¼
	
   uint32_t	plus_intv;				//���ݼӱ����� mqtt//center1Sms[18];
	 uint32_t	curAddr[MQTT_MAX_MULTI_CONNECTION_SIZE];//curMqttSendAddr[MQTT_MAX_MULTI_CONNECTION_SIZE];//
   uint16_t	gpsInterVal;	//gps��λ���
	 uint8_t	usartKey;			//���ߴ��书�ܿ���//
	 uint8_t	insentekKey;	//˽��Э�鴫�书�ܿ���// 0�رգ�1˽��Э����й������ã�2˽��Э�鷢������
   uint8_t	dipXThreshold;//���x��ֵ
	 uint8_t	dipYThreshold;//���y��ֵ
	 uint8_t	mqttDataType;
	 uint8_t  gsmRunMode;
	 uint8_t  qxkey;			//��ǹ��ܿ���
	 uint8_t  accelerometerkey;	//��ǹ��ܿ���
	 uint8_t	powerSavingKey;
	 uint8_t	secondAddrKey;
	 uint8_t	sk60Fuction;	//210705
	 uint8_t	kxylFuction;	//210705
	 uint8_t	center2Sms[2];//uint8_t	center2Sms[18];   
   uint8_t centerSmsFlag;  
   
}paramter;

#define PARAMTER_DATA_LENGTH    sizeof(paramter)
extern paramter devPar;

typedef struct
{
  uint8_t	netOperator;
	uint8_t systemMode[10];
	uint8_t operationMode[20];
	uint8_t mcc_mnc[10];
}netInfo_t;
extern netInfo_t netInformation;

#define NET_OPERATOR_NORMAL 	0
#define NET_OPERATOR_MOBILE		1
#define NET_OPERATOR_UNICOM		2
#define NET_OPERATOR_TELECOM	3


#define SEND_DENSITY_MAX					720
#define WORKMODE_LOWPOWER_DENSITY 24
#define WORKMODE_PLUS_DENSITY 		1

#define WORKMODE_RED_PERIOD 			300
#define WORKMODE_RED_DENSITY 			1
#define WORKMODE_ORANGE_PERIOD		600
#define WORKMODE_ORANGE_DENSITY		1//2
#define WORKMODE_YELLOW_PERIOD		1200
#define WORKMODE_YELLOW_DENSITY		1
#define WORKMODE_BLUE_PERIOD			1800
#define WORKMODE_BLUE_DENSITY			1

#define SK60_OFFSET_VALUE_MIN		-60.0
#define SK60_OFFSET_VALUE_MAX		60.0

enum workmode
{
	WORKMODE_RUN			= '0',
	WORKMODE_LOWPOWER	= '1',
	WORKMODE_PLUS			= '2',
};

#define RX_OK_CODE_ID       0x01
#define RX_ERROR_CODE_ID    0x02
#define RX_FRAME_OK         0x03
#define RX_FRAME_ERROR      0x04

typedef struct{
    uint16_t 	length;
    int8_t		result;
    uint8_t   counter;
}returnResult;

typedef struct{
	value_t length;
	value_t responseCode;
}rxResponse;

extern  uint8_t firwMare[8];

extern uint16_t sensorDataFormat(uint8_t curNode,uint8_t *buf,gpsValue gps,uint8_t gpsFlag,DS1302_TIME time);
extern uint16_t sensorDataLoad(uint8_t gpsFlag,gpsValue gps,DS1302_TIME time);
extern uint16_t sensorDataSmsSend(uint8_t smsType,uint8_t gpsInf,gpsValue gps,DS1302_TIME time);
extern void smsSendTask(httpConfig *smsConfig,gpsValue gps,DS1302_TIME time,uint8_t *dataBuf);
extern void gprsFireWareRxTask(httpConfig *rxResult);
extern uint32_t nextSendTime(uint8_t periodHour,devConfig sendTimeConfig);
extern void simCardReadTask(httpConfig *rxResult);

//�Ĵ���Լ�豸��غ���
typedef struct{
    uint8_t begMon;
    uint8_t begDate;
    uint8_t begHour;
    uint8_t begMin;
    uint8_t endMon;
    uint8_t endDate;
    uint8_t endHour;
    uint8_t endMin;
}queryTime;



//ֵ����
#define CONFIG_VALUE_ERROR       0x00000001
//�в���ʶ��ı�ʾ������
#define CONFIG_ID_ERROR          0x00000002
//��䲻����
#define FRAME_ERROR              0x00000004
//�޸Ĳ���ָ���в����������
#define KEY_ERROR                0x00000008
//ָ�����ݲ�����
#define REQUEST_ERROR            0x00000010
//���ݳ��޴���
#define REQUEST_OVERRUN_ERROR    0x00000020


#define CONFIG_ABC_0K             0x00000001
#define SMS_KEY_SET_OK            0x00000002
#define SMS_PHONE_OK              0x00000004
#define PERIOD_SET_OK             0x00000008
#define FIX_PERIOD_SET_OK         0x00000010
#define DENSITY_SET_OK            0x00000020
#define THRESHOLD_KEY_SET_OK      0x00000040
#define THRESHOLD_VOULUE1_SET_OK  0x00000080
#define THRESHOLD_VOULUE2_SET_OK  0x00000100
#define THRESHOLD_VOULUE3_SET_OK  0x00000200
#define STATION_RTC_SET_OK        0x00000400
#define HOST1_SET_OK              0x00000800
#define HOST2_SET_OK              0x00001000
#define HOST2_DEL_OK              0x00002000
#define STATION_SET_OK            0x00004000
#define PW_SET_OK                 0x00008000

#define SHARK_KEY_OK              0x00010000
#define SHARK_PHONE_SET_OK        0x00020000

#define SHARK_KEY_REQUEST         0x00010000
#define SHARK_PHONE_REQUEST       0x00020000
   
#define RETURN_CN_FLAG            0x00040000
#define STATION_RESET             0x00080000
#define STATION_FLASH_RESET       0x00100000
#define GPS_LOCATION_SET_OK       0x00200000

#define REQUEST_HISTORY_DATA_BY_TIME 0x01

//��ѯָ��
#define CONFIG_ABC_RQQUEST         0x00000001
#define SMS_KEY_RQQUEST            0x00000002
#define SMS_PHONE_RQQUEST          0x00000004
#define PERIOD_RQQUEST             0x00000008
#define FIX_PERIOD_RQQUEST         0x00000010
#define DENSITY_RQQUEST            0x00000020
#define THRESHOLD_KEY_RQQUEST      0x00000040
#define THRESHOLD_VOULUE1_RQQUEST  0x00000080
#define THRESHOLD_VOULUE2_RQQUEST  0x00000100
#define THRESHOLD_VOULUE3_RQQUEST  0x00000200
#define STATION_RTC_RQQUEST        0x00000400
#define HOST1_RQQUEST              0x00000800
#define HOST2_RQQUEST              0x00001000
#define HOST2_DEL_RQQUEST          0x00002000
#define STATION_RQQUEST            0x00004000
#define PW_RQQUEST                 0x00008000

#define HISTORY_DATA_REQUEST       0x10000000

//���ò����޸ķ��ͱ�־
#define HOST1_SEND_OK     	0x01
#define HOST2_SEND_OK     	0x02

#define CONFIG_UNSEND_FLAG	0x03

typedef struct{
	uint32_t confChangeFlag;
	uint32_t requestFlag;
	uint32_t requestErrorFlag;    
	uint32_t confChangErrorFlag;
	uint16_t unSendPacket;
	uint32_t curSendAddress;
	uint32_t requestDataReadAddress;
	uint32_t requestDataEndAddress;
	uint16_t  requestData;
	uint8_t historyDataFlag;
	uint8_t requestDataType;
	uint16_t upSerialNum;
	uint16_t downSerialNum;
	uint16_t configCounter;
}stationSend;

#define NULL_DATA_FLAG				0x00
#define KEY_DATA_FLAG         0x01
#define RTC_DATA_FLAG         0x02
#define CAN232_DATA_FLAG      0x04
#define TEST_MODE_FLAG        0x08

extern uint8_t sensorDataFlag;
extern stationSend curStationSend[STATION_MULTI_CONNECTION_SIZE];
extern __IO uint16_t gsmIntFlag;
extern void gsmIntDeal(void);
extern void gsmInt_Config(void);
extern void stationServerBufInt(void);
extern void stationServerAddrDebug(void);
extern void gprsStationServerSendRxTask(httpConfig *rxResult,stationSend *curStSend);
extern  void smsTextRxSendTask(httpConfig *rxResult,uint8_t *smsBuf);

//GPRS״̬��Ϣ
#define GPRS_UART_INIT			0x00
#define GPRS_OK							0x01
#define GPRS_SET						0x02
#define GPRS_LINK						0x03
#define GPRS_SEND						0x04
#define GPRS_CLOSE					0x05
#define GPRS_TURN_ON				0x06
#define GPRS_TURN_OFF				0x07
#define GPRS_UNlINK					0x08
#define GPRS_RX							0x09
#define GPRS_STATUS_QUERY   0x0B
#define GPRS_SMS_SEND				0x0C
#define GPRS_SMS_SEND_COMM	0x0D

#define GPRS_LINK_TEST          0x10
#define GPRS_GPS_STATUS_QUERY   0x13
#define GPRS_GPS_VALUE          0x14
#define GPRS_GPS_POWER_OFF      0x12
#define GPRS_SEND_TEST					0x11
#define GPRS_GPS_START					0x15
#define GPRS_GPS_POWER_ON       0x16
#define GPRS_SEND_DATA_READY    0x17
#define GPRS_RX_FIREWARE				0x18
#define GSM_MOUDLE_IDLE					0x19
#define GSM_SMS_DATA_NUM_SELECT	0x1A
#define GSM_SEND_DATA_HEARD			0x1B
#define GPRS_SINGEL_CLOSE				0x1C

#define GPRS_SENSOR_DATA_RE			0x1d
#define GPRS_REG_DATA_RE				0x20
#define GPRS_CONFIG_DATA_RE			0x21
#define GPRS_RTC_CAL_RE					0x22
#define FIND_HISTORY_DATA				0x23
#define GPRS_HISTORY_DATA				0x24

#define GPRS_CONFIG_DATA_FIND		0x25
#define GPRS_INSTRUCT_DATA_FIND	0x26

#define GPRS_CONFIG_RE_DATA			0x27
#define GPRS_REQUEST__RE_DATA		0x28

#define GPRS_SLEEP_MODE					0x29

#define GPRS_SMS_TOTAL_READ			0x30
#define GPRS_SMS_READ						0x31
#define GPRS_SMS_JUDGE					0x32
#define GPRS_SMS_DEL						0x33
#define GPRS_DNS2IP							0x34//SIM7100

//AT�����Ӧ״̬��Ϣ
#define COMM_ACK								0x07
#define COMM_SEND								0x01
#define COMM_OK									0x02
#define COMM_ERR								0x03
#define COMM_TIMEOUT						0x04
#define NO_COMM									0x05
#define COMM_BACK								0x06

#define COMM_LINK_OK						0x01
#define COMM_LINK_NONE					0x00

#define REQUEST_FIREWARE_DATA		0x01
#define OTHER_DATA							0x02
#define MQTT_DATA								0x03

#define THRESHOLD_VALUE1    		0x01
#define THRESHOLD_VALUE2    		0x02
#define THRESHOLD_VALUE3    		0x03

#define MAX_CONFIG_BUF_SIZE			12

#define NORMAL_CLOSE     0x01
#define LINK_ERR_CLOSE   0x02
#define SEND_ERR_CLOSE   0x03
#define IP_STATE_CLOSE   0x04


#define SINGLE_SEND_TYPE		0x01
#define MULTI_SEND_TYPE			0x02
#define	CONNECTION_CLOSE   	0x01
#define	CONNECTION_ALIVE_ON	0x02

#define REQUEST_CONFIG    	0x01
#define UNREQUEST						0x02

#define IP_STATUS           0x01
#define CONNECT_STATUS      0x02
#define IP_INITIAL_STATUS   0x03

enum serverStatus
{	
	serverOk = 0,				//0 ��Ӧ�����������������
	serverUnused,				//1	��Ӧ������û������
	serverLinkErr,			//2	����tcp����������δ���ӵ�tcp������
	serverConnectErr,		//3	����mqtt������ʱ����ģ��δ��ȷ������������
	serverConnectAckErr,//4 ����mqtt������ʱӦ�����
	serverSendErr,			//5	���ݷ��ʹ���ģ��δ��������
	serverSendAckErr,		//6	����Ӧ��������ݷ����ɹ���δ���յ�������Ӧ��
};

#define ROUND0(sp) decimal(sp,0)	
#define ROUND1(sp) decimal(sp,1)	
#define ROUND2(sp) decimal(sp,2)	
#define ROUND3(sp) decimal(sp,3)
#define ROUND4(sp) decimal(sp,4)	

#define PARAM_TYPE_Normal 	0x00
#define PARAM_TYPE_KXYL			0x01

extern uint16_t lacValue;
extern uint16_t cellIdValue ;
extern uint16_t rssiValue ;

extern uint16_t simCardErrorCounter;
extern uint16_t netGsmErrorCounter;




extern void gprsPowerOn(void);
extern void gprsPowerOff(void);

extern uint8_t gprsSet(lsrLog *serverLog);
extern uint8_t GPRSAttach(void);
extern uint8_t gprsLink(uint8_t *buf,uint8_t linknum);
extern uint8_t gprsSend(uint8_t *buf,__IO uint16_t length,uint8_t dataType,uint8_t linknum);
extern uint8_t closeTcp(void);
extern uint8_t closeSingleTcp(uint8_t linknum);

//extern uint16_t findStrRxGprs(char *buf,uint8_t bufLength);
extern uint16_t gprsLinkStatusAll(void);
extern uint8_t gprsLinkStatus(uint8_t linknum);
extern uint8_t setReceiveDataMode(void);

extern int8_t historyDataFind(httpConfig *http);
//extern void gprsSendSucFlagWrite(uint32_t writeAddr,uint8_t packetCounter,DS1302_TIME time);
//extern void gprsSendSucDataDeal(__IO uint32_t *sendAddress,uint8_t sendPacketCounter,__IO uint16_t *packetCounter);
extern void gprsSendSucFlagWrite(uint32_t lastSendAddr,uint32_t sendAddr,DS1302_TIME time);

extern void simCardReadTask(httpConfig *rxResult);

extern uint16_t loadSimCardInf(uint8_t *buf,uint16_t index);

extern uint8_t readReceiveMode(void);
extern uint16_t readReceivelen(uint8_t linknum);
extern uint16_t waitRxGprsData(uint8_t linknum,uint32_t *code,uint8_t waitFlag);
extern uint16_t receiveDataFlag(void);
extern uint8_t findipcloseFlag(void);
extern uint16_t findRxGprsData(uint8_t *buf,uint16_t *rxlen,uint8_t linknum);
extern int8_t findServerIpAddr(uint8_t *buf,char* addr, uint16_t port,uint8_t linknum);

extern int8_t utcTimeAdj(DS1302_TIME *timeOut,int8_t zoneOut,DS1302_TIME timeIn,int8_t zoneIn);
extern double decimal(double data,uint8_t number);
extern uint16_t sensorDataReadToBuf(uint8_t type,uint32_t readAddress,uint8_t *readBuf,__IO uint16_t *readLen);


typedef struct
{
	float		pitch;
	float 	roll;	
	float 	dipz;	
}xyzQx_t;
#define XYZ_QX_BUFFER_SIZE 0x03
extern xyzQx_t xyzQxBuffer[XYZ_QX_BUFFER_SIZE+1];
extern uint8_t xyzQxindex;
extern xyzQx_t xyzQxCalculate(float xg,float yg,float zg);
extern xyzQx_t xyzQxValueCaculate(void);
extern void xyzQxValueTask(void);

extern float qxAngleCaculate(double X,double Y);
extern float qxZCaculate(float angle);

#endif


