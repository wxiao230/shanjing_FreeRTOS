/**
  ******************************************************************************
  * @file    Project/user/mqttTransport.h 
  * @author  insentek wxh
  * @version V1.0.0
  * @date    13-July-2020
  * @brief   Header for mqttTransport.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTTTRANSPORT_H
#define __MQTTTRANSPORT_H

/* Includes ------------------------------------------------------------------*/
#include "mqttClient.h"
#include "gprs.h"
#include "stm32l1xx.h"
/* Exported typedef -----------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
#define MQTT_DEBUG					1	
#define MQTT_TEST						0 //0 正常 1不定位 2 mqtt测试
#define MQTT_KEEPALIVE_TIME	64800//18_64800 1_3600 4_14400 6_21600 12_43200

#define MQTT_RXBUFFER_LEN 	0x5FF////20210118 0x3FF
#define MQTT_TXBUFFER_LEN 	0x3FF

#define MQTT_DUP_0	0
#define MQTT_DUP_1	1
#define MQTT_QOS_0	0
#define MQTT_QOS_1	1
#define MQTT_QOS_2	2
#define MQTT_RETAIN_0	0
#define MQTT_RETAIN_1	1

#define MQTT_WAIT_ACK_NONE	0
#define MQTT_WAIT_ACK_DONE	1
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/  
extern  uint8_t 	mqttRxBuffer[MQTT_RXBUFFER_LEN+1];
extern 	uint8_t 	mqttTxBuffer[MQTT_TXBUFFER_LEN+1];
extern  uint16_t	mqttRxLen;

extern MQTTPacket_connectData 	connectData;
extern MQTTPacket_subcribeData subcribedata;
extern MQTTPacket_publishData	pubdata;

extern 	char 			mqtthost[40];
extern	uint16_t 	mqttport;
extern	__IO uint8_t	mqttlinknum;
extern	__IO uint8_t	mqttLinkChannel;
extern 	__IO uint16_t	mqttlen;
extern 	uint8_t		mqtttrytimes;

extern 	uint8_t		connack_rc;
extern 	int8_t	 	mqttmsgtype;
extern 	uint32_t	mqttcode;

/* Exported functions ------------------------------------------------------- */	


extern int transport_sendPacketBuffer(uint8_t* buf, uint16_t buflen,uint8_t rxFlag,uint8_t linknum);
extern int transport_getdata(uint8_t* buf, int count);
extern int transport_open(char* addr, int port,uint8_t linknum);
extern int transport_close(void);
extern int transport_rxTask(uint8_t* buf,uint16_t	buflen,int8_t *msgtype,uint8_t linknum);
extern void mqttcigemrxcmdTask(uint8_t *buf,uint16_t buflen,uint8_t linknum);

extern void mqttunsendpacketnum(void);
extern void mqttunsendpacketnumadj(void);
extern int8_t mqttunsendpacketCheck(httpConfig *config);
extern int8_t mqttserverinit(uint8_t linknum);
extern void mqttpublishsensordata(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char*clientid);
extern void mqttpublishdevstatus(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char*clientid);
#endif /* __MQTTTRANSPORT_H */

/*****************************END OF FILE*****************************/
