/**
  ******************************************************************************
  * @file    Project/user/mqttClient.h 
  * @author  insentek wxh
  * @version V1.0.0
  * @date    13-July-2020
  * @brief   Header for mqttClient.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MQTTCLIENT_H
#define __MQTTCLIENT_H

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32l1xx.h"
/* Exported typedef -----------------------------------------------------------*/
enum errors
{
	MQTTPACKET_BUFFER_TOO_SHORT = -2,
	MQTTPACKET_READ_ERROR = -1,
	MQTTPACKET_READ_COMPLETE
};

enum msgTypes
{
	CONNECT = 1, CONNACK, PUBLISH, PUBACK, PUBREC, PUBREL,
	PUBCOMP, SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK,
	PINGREQ, PINGRESP, DISCONNECT
};
enum connack_return_codes
{
    MQTT_CONNECTION_ACCEPTED = 0,
    MQTT_UNNACCEPTABLE_PROTOCOL = 1,
    MQTT_CLIENTID_REJECTED = 2,
    MQTT_SERVER_UNAVAILABLE = 3,
    MQTT_BAD_USERNAME_OR_PASSWORD = 4,
    MQTT_NOT_AUTHORIZED = 5,
		MQTT_NOT_CONNECT = 6,
};
enum publishstep
{
	PUB_NONE = 0,PUB_START,PUB_ACK_OK,PUB_REC,PUB_REL,PUB_COMP
};
/**
 * Bitfields for the MQTT header byte.
 */
typedef union
{
	uint8_t byte;	/**< the whole byte */
	struct
	{
		unsigned int retain : 1;	/**< retained flag bit */
		unsigned int qos : 		2;	/**< QoS value, 0, 1 or 2 */
		unsigned int dup : 		1;	/**< DUP flag bit */
		unsigned int type : 	4;	/**< message type nibble */
	} bits;
} MQTTHeader;

typedef union
{
	uint8_t all;	/**< all connect flags */
	struct
	{
		unsigned int : 							1;	/**< unused */
		unsigned int cleansession : 1;	/**< cleansession flag */
		unsigned int will : 				1;	/**< will flag */
		unsigned int willQoS : 			2;	/**< will QoS value */
		unsigned int willRetain : 	1;	/**< will retain setting */
		unsigned int password : 		1;	/**< 3.1 password */
		unsigned int username : 		1;	/**< 3.1 user name */
	} bits;
} MQTTConnectFlags;	/**< connect flags byte */

typedef union
{
	uint8_t all;	/**< all connack flags */
	struct
	{
		unsigned int sessionpresent : 1;	/**< session present flag */
    unsigned int reserved: 				7;	/**< unused */
	} bits;
} MQTTConnackFlags;	/**< connack flags byte */

typedef struct
{
	int len;
	char* data;
} MQTTLenString;

typedef struct
{
	char* cstring;
	MQTTLenString lenstring;
} MQTTString;

/** Defines the MQTT "Last Will and Testament" (LWT) settings for the connect packet.*/
typedef struct
{
	MQTTString topicName;
	MQTTString message;	
	uint8_t retained;
	char qos;
} MQTTPacket_willOptions;

typedef struct
{	
	uint8_t MQTTVersion;
	MQTTString clientID;
	uint16_t keepAliveInterval;
	uint8_t cleansession;
	uint8_t willFlag;
	MQTTPacket_willOptions will;
	MQTTString username;
	MQTTString password;
} MQTTPacket_connectData;

typedef struct
{	
	uint8_t		step;
	uint8_t 	dup;
	uint8_t 	qos;
	uint8_t 	retain;
	uint16_t		packetid;
	MQTTString 	topicName;
	uint16_t	 	payloadlen;
	char*				payload;	
} MQTTPacket_publishData;
typedef struct
{
	uint16_t packetid;
	MQTTString 	topicName;
	uint8_t			qos;
} MQTTPacket_subcribeData;
#define SUB_COUNT_MAX	5
typedef struct
{
	uint16_t 	packetid;
	uint8_t 	count;
	uint8_t		qos[SUB_COUNT_MAX];
} MQTTPacket_subackData;

/* Exported define ------------------------------------------------------------*/
#define MQTT_VER_310	3
#define MQTT_VER_311	4
#define MQTTString_initializer {NULL, {0, NULL}}
#define MQTTPacket_willOptions_initializer {MQTTString_initializer,MQTTString_initializer,0,0}
#define MQTTPacket_connectData_initializer {MQTT_VER_311,MQTTString_initializer,60,1,0,MQTTPacket_willOptions_initializer,MQTTString_initializer,MQTTString_initializer}
#define MQTTPacket_publishData_initializer {0,0,0,0,0,MQTTString_initializer,0,NULL}
#define MQTTPacket_subcribeData_initializer {0,MQTTString_initializer,0}
#define MQTTPacket_subackData_initializer {0,0,{0}}
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/ 

/* Exported functions ------------------------------------------------------- */	
extern int MQTTPacket_read(uint8_t* buf, uint16_t buflen, int (*getfn)(uint8_t*, int));
extern int MQTTSerialize_connect(uint8_t* buf, uint16_t buflen, MQTTPacket_connectData* options);
extern int MQTTSerialize_publish(uint8_t* buf, uint16_t buflen,MQTTPacket_publishData pubdata);
extern int MQTTSerialize_puback(uint8_t* buf, uint16_t buflen, uint16_t packetid);
extern int MQTTSerialize_pubrec(uint8_t* buf, uint16_t buflen, uint16_t packetid);;
extern int MQTTSerialize_pubrel(uint8_t* buf, uint16_t buflen, uint16_t packetid);
extern int MQTTSerialize_pubcomp(uint8_t* buf, uint16_t buflen, uint16_t packetid);
extern int MQTTSerialize_subscribe(uint8_t* buf, uint16_t buflen,int count,MQTTPacket_subcribeData *subdata);
extern int MQTTSerialize_unsubscribe(uint8_t* buf, uint16_t buflen,int count,MQTTPacket_subcribeData *subdata);
extern int MQTTSerialize_pingreq(uint8_t* buf, uint16_t buflen);
extern int MQTTSerialize_disconnect(uint8_t* buf, uint16_t buflen);

extern int MQTTDeserialize_connack(uint8_t* sessionPresent, uint8_t* connack_rc, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_publish(MQTTPacket_publishData *data, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_puback(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_pubrec(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_pubrel(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_pubcomp(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_suback(uint16_t maxcount,MQTTPacket_subackData* ackdata, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_unsuback(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
extern int MQTTDeserialize_pingresp(uint8_t* buf, uint16_t buflen,uint16_t *usedlen);
#endif /* __MQTTCLIENT_H */

/*****************************END OF FILE*****************************/
