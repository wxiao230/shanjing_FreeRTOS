/**
  ******************************************************************************
  * @file    Project/user/mqttClient.c
  * @author  insentek wxh
  * @version V1.0.0
  * @date    13-July-2020
  * @brief   
  ******************************************************************************   
  */  

/* Includes ------------------------------------------------------------------*/
#include "mqttClient.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_NO_OF_REMAINING_LENGTH_BYTES 4
#define min(a, b) ((a < b) ? 1 : 0)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const MQTTString MQTTStringZero = MQTTString_initializer;
const MQTTPacket_subackData MQTTSubackZero = MQTTPacket_subackData_initializer;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int MQTTPacket_encode(uint8_t* buf, int length)
{
	int	 rc = 0;
	char d	= 0;
	do
	{
		d = length % 128;
		length /= 128;
		/* if there are more digits to encode, set the top bit of this digit */
		if (length > 0)
			d |= 0x80;
		buf[rc++] = d;
	} while(length > 0);
	return rc;
}

int MQTTPacket_decode(int (*getcharfn)(uint8_t*, int), int* value)
{
	uint8_t c;
	int multiplier = 1;
	int len = 0;
	int rc = MQTTPACKET_READ_ERROR;
	*value = 0;
	do
	{		
		if (++len > MAX_NO_OF_REMAINING_LENGTH_BYTES)
		{
			rc = MQTTPACKET_READ_ERROR;	/* bad data */
			return len;
		}
		rc = (*getcharfn)(&c, 1);
		if (rc != 1)
		{	
			return len;
		}
		*value += (c & 127) * multiplier;
		multiplier *= 128;
	} while ((c & 128) != 0);
	return len;
}

static uint8_t* bufptr;

int bufchar(uint8_t* c, int count)
{
	int i;
	for (i = 0; i < count; ++i)*c = *bufptr++;
	return count;
}

int MQTTPacket_decodeBuf(uint8_t* buf, int* value)
{
	bufptr = buf;
	return MQTTPacket_decode(bufchar, value);
}

int MQTTstrlen(MQTTString mqttstring)
{
	int rc = 0;
	if(mqttstring.cstring)
	{
		rc = strlen(mqttstring.cstring);
	}
	else
	{
		rc = mqttstring.lenstring.len;
	}
	return rc;
}

int MQTTPacket_len(int rem_len)
{
	rem_len += 1; /* header byte */
	/* now remaining_length field */
	if (rem_len < 128)
	{rem_len += 1;}
	else if (rem_len < 16384)
	{	rem_len += 2;}
	else if (rem_len < 2097151)
	{	rem_len += 3;}
	else
	{	rem_len += 4;}
	return rem_len;
}

char readChar(uint8_t** pptr)
{
	char c = **pptr;
	(*pptr)++;
	return c;
}
int readInt(uint8_t** pptr)
{
	uint8_t* ptr = *pptr;
	int len = 256*(*ptr) + (*(ptr+1));
	*pptr += 2;
	return len;
}
int getLenStringLen(char* ptr)
{
	int len = 256*((uint8_t)(*ptr)) + (uint8_t)(*(ptr+1));
	return len;
}
int readMQTTLenString(MQTTString* mqttstring, uint8_t** pptr, uint8_t* enddata)
{
	int rc = 0;
	/* the first two bytes are the length of the string */
	if (enddata - (*pptr) > 1) /* enough length to read the integer? */
	{
		mqttstring->lenstring.len = readInt(pptr); /* increments pptr to point past length */
		if (&(*pptr)[mqttstring->lenstring.len] <= enddata)
		{
			mqttstring->lenstring.data = (char*)*pptr;
			*pptr += mqttstring->lenstring.len;
			rc = 1;
		}
	}
	mqttstring->cstring = NULL;
	return rc;
}

void writeChar(uint8_t** pptr, char c)
{
	**pptr = c;
	(*pptr)++;
}
void writeInt(uint8_t** pptr, int anInt)
{
	**pptr = (uint8_t)(anInt / 256);
	(*pptr)++;
	**pptr = (uint8_t)(anInt % 256);
	(*pptr)++;
}
void writeCString(uint8_t** pptr, const char* string)
{
	int len = strlen(string);
	writeInt(pptr,len);
	memcpy(*pptr,string,len);
	*pptr += len;
}
void writeMQTTString(uint8_t** pptr, MQTTString mqttstring)
{
	if (mqttstring.lenstring.len > 0)
	{
		writeInt(pptr, mqttstring.lenstring.len);
		memcpy(*pptr, mqttstring.lenstring.data, mqttstring.lenstring.len);
		*pptr += mqttstring.lenstring.len;
	}
	else if (mqttstring.cstring)
	{
		writeCString(pptr, mqttstring.cstring);
	}
	else
	{
		writeInt(pptr, 0);
	}
}

int MQTTPacket_equals(MQTTString* a, char* bptr)
{
	int alen = 0,blen = 0;
	char *aptr;
	
	if (a->cstring)
	{
		aptr = a->cstring;
		alen = strlen(a->cstring);
	}
	else
	{
		aptr = a->lenstring.data;
		alen = a->lenstring.len;
	}
	blen = strlen(bptr);
	
	return (alen == blen) && (strncmp(aptr, bptr, alen) == 0);
}
int MQTTPacket_read(uint8_t* buf, uint16_t buflen, int (*getfn)(uint8_t*, int))
{
	int rc = -1;
	MQTTHeader header = {0};
	int len = 0;
	int rem_len = 0;
	
	/* 1. read the header byte.  This has the packet type in it */
	if ((*getfn)(buf, 1) != 1)return rc;
	
	len = 1;
	/* 2. read the remaining length.  This is variable in itself */
	MQTTPacket_decode(getfn, &rem_len);
	len += MQTTPacket_encode(buf+1, rem_len); /* put the original remaining length back into the buffer */
	rc = -2;
	/* 3. read the rest of the buffer using a callback to supply the rest of the data */
	if((rem_len + len) > buflen){return rc;}
	rc = -3;
	if (rem_len && ((*getfn)(buf + len, rem_len) != rem_len)){return rc;}
	
	header.byte = buf[0];
	rc = header.bits.type;
	if((rem_len==0)&&(rc<PINGREQ))return -4;
	return rc;
}


/** MQTTSerializ */
int MQTTSerialize_connectLength(MQTTPacket_connectData* options)
{
	int len = 0;

	if (options->MQTTVersion == 3)
		len = 12; /* variable depending on MQTT or MQIsdp */
	else if (options->MQTTVersion == 4)
		len = 10;

	len += MQTTstrlen(options->clientID)+2;
	if (options->willFlag)
	{
		len += MQTTstrlen(options->will.topicName)+2 + MQTTstrlen(options->will.message)+2;
	}
	if (options->username.cstring || options->username.lenstring.data)
	{
		len += MQTTstrlen(options->username)+2;
	}
	if (options->password.cstring || options->password.lenstring.data)
	{
		len += MQTTstrlen(options->password)+2;
	}
	return len;
}

int MQTTSerialize_connect(uint8_t* buf, uint16_t buflen, MQTTPacket_connectData* options)
{
	uint8_t *ptr 	= buf;
	MQTTHeader header 	= {0};
	MQTTConnectFlags flags = {0};
	int len = 0;
	int rc = -1;

	if (MQTTPacket_len(len = MQTTSerialize_connectLength(options)) > buflen)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}

	header.byte = 0;
	header.bits.type = CONNECT;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, len); /* write remaining length */

	if (options->MQTTVersion == 4)
	{
		writeCString(&ptr, "MQTT");
		writeChar(&ptr, (char) 4);
	}
	else
	{
		writeCString(&ptr, "MQIsdp");
		writeChar(&ptr, (char) 3);
	}

	flags.all = 0;
	flags.bits.cleansession = (options->cleansession)?1:0;//options->cleansession;
	flags.bits.will = (options->willFlag)?1:0;
	if (flags.bits.will)
	{
		flags.bits.willQoS 		= options->will.qos;
		flags.bits.willRetain = options->will.retained;
	}

	if (options->username.cstring || options->username.lenstring.data)flags.bits.username = 1;
		
	if (options->password.cstring || options->password.lenstring.data)flags.bits.password = 1;
		
	writeChar(&ptr, flags.all);
	writeInt(&ptr, options->keepAliveInterval);
	writeMQTTString(&ptr, options->clientID);
	if (options->willFlag)
	{
		writeMQTTString(&ptr, options->will.topicName);
		writeMQTTString(&ptr, options->will.message);
	}
	if (flags.bits.username)writeMQTTString(&ptr, options->username);
		
	if (flags.bits.password)writeMQTTString(&ptr, options->password);
		
	rc = ptr - buf;

	return rc;
}


int MQTTSerialize_publishLength(int qos, MQTTString topicName, int payloadlen)
{
	int len = 0;

	len += 2 + MQTTstrlen(topicName) + payloadlen;
	if (qos > 0)
		len += 2; /* packetid */
	return len;
}
//int MQTTSerialize_publish(uint8_t* buf, uint16_t buflen, uint8_t dup, int qos, uint8_t retained, uint16_t packetid,
//		MQTTString topicName, uint8_t* payload, int payloadlen)
int MQTTSerialize_publish(uint8_t* buf, uint16_t buflen,MQTTPacket_publishData data)
{
	uint8_t *ptr = buf;
	MQTTHeader header = {0};
	int rem_len = 0;
	int rc = 0;

	if (MQTTPacket_len(rem_len = MQTTSerialize_publishLength(data.qos, data.topicName, data.payloadlen)) > buflen)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}

	header.bits.type 		= PUBLISH;
	header.bits.dup 		= data.dup;
	header.bits.qos 		= data.qos;
	header.bits.retain 	= data.retain;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, rem_len); /* write remaining length */;

	writeMQTTString(&ptr, data.topicName);

	if (data.qos > 0)writeInt(&ptr,data.packetid);
		
	memcpy(ptr, data.payload, data.payloadlen);
	ptr += data.payloadlen;

	rc = ptr - buf;

	return rc;
}

int MQTTSerialize_ack(uint8_t* buf, uint16_t buflen, uint8_t packettype, uint16_t packetid)
{
	MQTTHeader header = {0};
	int rc = 0;
	uint8_t *ptr = buf;

	if (buflen < 4)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}
	header.bits.type = packettype;
	header.bits.dup = 0;
	header.bits.qos = (packettype == PUBREL) ? 1 : 0;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, 2); /* write remaining length */
	writeInt(&ptr, packetid);
	rc = ptr - buf;

	return rc;
}

int MQTTSerialize_puback(uint8_t* buf, uint16_t buflen, uint16_t packetid)
{
	return MQTTSerialize_ack(buf, buflen, PUBACK, packetid);
}
int MQTTSerialize_pubrec(uint8_t* buf, uint16_t buflen, uint16_t packetid)
{
	return MQTTSerialize_ack(buf, buflen, PUBREC, packetid);
}
int MQTTSerialize_pubrel(uint8_t* buf, uint16_t buflen, uint16_t packetid)
{
	return MQTTSerialize_ack(buf, buflen, PUBREL, packetid);
}
int MQTTSerialize_pubcomp(uint8_t* buf, uint16_t buflen, uint16_t packetid)
{
	return MQTTSerialize_ack(buf, buflen, PUBCOMP, packetid);
}

int MQTTSerialize_subscribeLength(int count, MQTTString topicFilters[])
{
	int i;
	int len = 2; /* packetid */

	for (i = 0; i < count; ++i)
		len += 2 + MQTTstrlen(topicFilters[i]) + 1; /* length + topic + req_qos */
	return len;
}
//int MQTTSerialize_subscribe(uint8_t* buf, uint16_t buflen, uint16_t packetid, int count,
//		MQTTString topicFilters[], int requestedQoSs[]);
int MQTTSerialize_subscribe(uint8_t* buf, uint16_t buflen,int count,MQTTPacket_subcribeData *subdata)
{
	uint8_t *ptr = buf;
	MQTTHeader header = {0};
	int rem_len = 0;
	int rc = 0;
	int i = 0;

	if (MQTTPacket_len(rem_len = MQTTSerialize_subscribeLength(count,&subdata->topicName)) > buflen)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}
	
	header.byte = 0;
	header.bits.type 	= SUBSCRIBE;
	header.bits.dup 	= 0;
	header.bits.qos 	= 1;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, rem_len); /* write remaining length */;

	writeInt(&ptr, subdata->packetid);

	for (i = 0; i < count; ++i)
	{
		writeMQTTString(&ptr, subdata[i].topicName);
		writeChar(&ptr, subdata[i].qos);
	}

	rc = ptr - buf;
	return rc;
}
int MQTTSerialize_unsubscribeLength(int count, MQTTString topicFilters[])
{
	int i;
	int len = 2; /* packetid */

	for (i = 0; i < count; ++i)
		len += 2 + MQTTstrlen(topicFilters[i]); /* length + topic*/
	return len;
}
int MQTTSerialize_unsubscribe(uint8_t* buf, uint16_t buflen,int count,MQTTPacket_subcribeData *subdata)
{
	uint8_t *ptr = buf;
	MQTTHeader header = {0};
	int rem_len = 0;
	int rc = 0;
	int i = 0;

	if (MQTTPacket_len(rem_len = MQTTSerialize_unsubscribeLength(count,&subdata->topicName)) > buflen)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}

	header.byte = 0;
	header.bits.type 	= UNSUBSCRIBE;
	header.bits.dup 	= 0;
	header.bits.qos 	= 1;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, rem_len); /* write remaining length */;

	writeInt(&ptr, subdata->packetid);

	for (i = 0; i < count; ++i)
	{
		writeMQTTString(&ptr, subdata[i].topicName);
	}

	rc = ptr - buf;
	return rc;
}

int MQTTSerialize_zero(uint8_t* buf, uint16_t buflen, uint8_t packettype)
{
	MQTTHeader header = {0};
	int rc = -1;
	uint8_t *ptr = buf;

	if (buflen < 2)
	{
		rc = MQTTPACKET_BUFFER_TOO_SHORT;
		return rc;
	}
	header.byte = 0;
	header.bits.type = packettype;
	writeChar(&ptr, header.byte); /* write header */

	ptr += MQTTPacket_encode(ptr, 0); /* write remaining length */
	rc = ptr - buf;
	return rc;
}
int MQTTSerialize_pingreq(uint8_t* buf, uint16_t buflen)
{
	return MQTTSerialize_zero(buf, buflen, PINGREQ);
}
int MQTTSerialize_disconnect(uint8_t* buf, uint16_t buflen)
{
	return MQTTSerialize_zero(buf, buflen, DISCONNECT);
}


/** MQTTDeserialize */
int MQTTDeserialize_connack(uint8_t* sessionPresent, uint8_t* connack_rc, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	MQTTHeader header = {0};
	uint8_t* curdata = buf;
	uint8_t* enddata = NULL;
	int rc = 0;
	int mylen;
	MQTTConnackFlags flags = {0};
	
	*sessionPresent	=	0;
	*connack_rc			=	MQTT_NOT_CONNECT;
	*usedlen	=	0;
	header.byte = readChar(&curdata);
	if (header.bits.type != CONNACK)return rc;
	*usedlen	=	1;
	curdata += (MQTTPacket_decodeBuf(curdata, &mylen)); /* read remaining length rc = */
	enddata = curdata + mylen;
	if (mylen < 2)return rc;

	flags.all = readChar(&curdata);
	*sessionPresent = flags.bits.sessionpresent;
	*connack_rc 		= readChar(&curdata);
	*usedlen				=	enddata - buf;
	rc = 1;
	return rc;
}

//int MQTTDeserialize_publish(uint8_t* dup, int* qos, uint8_t* retained, uint16_t* packetid, MQTTString* topicName,
//		uint8_t** payload, int* payloadlen, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
int MQTTDeserialize_publish(MQTTPacket_publishData *data, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	MQTTHeader header = {0};
	uint8_t* curdata = buf;
	uint8_t* enddata = NULL;
	int rc 			= 0;
	int mylen 	= 0;
	
	data->dup			= 0;
	data->qos			=	0;
	data->retain	= 0;
	data->packetid=	0;
	data->topicName		=	MQTTStringZero;
	data->payloadlen	=	0;
	*usedlen			=	0;
	header.byte = readChar(&curdata);
	if (header.bits.type != PUBLISH)return rc;
	*usedlen	=	1;	
	data->dup = header.bits.dup;
	data->qos = header.bits.qos;
	data->retain = header.bits.retain;

	curdata += (MQTTPacket_decodeBuf(curdata, &mylen)); /* read remaining length rc = */
	enddata = curdata + mylen;
 /* do we have enough data to read the protocol version byte? */
	if (!readMQTTLenString(&data->topicName, &curdata, enddata) ||enddata - curdata < 0)return rc;
		
	if (data->qos > 0)data->packetid = readInt(&curdata);

	data->payloadlen = enddata - curdata;
	data->payload 	 = (char *)curdata;
	*usedlen		=	enddata - buf;
	rc = 1;

	return rc;
}

int MQTTDeserialize_ack(uint8_t* packettype, uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	MQTTHeader header = {0};
	uint8_t* curdata = buf;
	uint8_t* enddata = NULL;
	int rc = 0;
	int mylen;
	*usedlen		=	0;
	*packettype	=	0;
	*packetid		=	0;
	header.byte = readChar(&curdata);	
	if((header.bits.type == PUBREL)&&(header.bits.qos != 1))return rc;
	*packettype = header.bits.type;
	*usedlen		=	1;
	curdata += (MQTTPacket_decodeBuf(curdata, &mylen)); /* read remaining length rc = */
	
	if(header.bits.type != PINGRESP)
	{
		enddata = curdata + mylen;
		if (mylen < 2)return rc;//if (enddata - curdata < 2)return rc;
		*packetid = readInt(&curdata);
		*usedlen	=	enddata - buf;
	}
	else
	{
		if(mylen != 0)return rc;
		*usedlen	=	2;
	}
	rc = 1;
	return rc;
}
int MQTTDeserialize_puback(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	uint8_t type = 0;
	int rc = 0;
	rc = 0;
	if(MQTTDeserialize_ack(&type, packetid, buf, buflen,usedlen)==1)
	{
		if(type == PUBACK)rc = 1;
	}
	
	return rc;
}
int MQTTDeserialize_pubrec(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	uint8_t type = 0;
	int rc = 0;

	if(MQTTDeserialize_ack(&type, packetid, buf, buflen,usedlen)==1)
	{
		if(type == PUBREC)rc = 1;
	}	
	
	return rc;
}
int MQTTDeserialize_pubrel(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	uint8_t type = 0;
	int rc = 0;

	if(MQTTDeserialize_ack(&type, packetid, buf, buflen,usedlen)==1)
	{
		if (type == PUBREL)rc = 1;
	}
	return rc;
}
int MQTTDeserialize_pubcomp(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	uint8_t type = 0;
	int rc = 0;

	if(MQTTDeserialize_ack(&type, packetid, buf, buflen,usedlen)==1)
	{
		if (type == PUBCOMP)rc = 1;
	}
	return rc;
}

int MQTTDeserialize_suback(uint16_t maxcount,MQTTPacket_subackData* ackdata, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	MQTTHeader header = {0};
	uint8_t* curdata = buf;
	uint8_t* enddata = NULL;
	int rc = 0;
	int mylen;
	*usedlen	=	0;
	*ackdata 	=	MQTTSubackZero;
	header.byte = readChar(&curdata);
	if (header.bits.type != SUBACK)return rc;
	*usedlen	=	1;
	curdata += (MQTTPacket_decodeBuf(curdata, &mylen)); /* read remaining length rc = */
	enddata = curdata + mylen;
	if (mylen < 2)return rc;

	ackdata->packetid = readInt(&curdata);
	*usedlen	=	enddata - buf;
	ackdata->count = 0;
	while (curdata < enddata)
	{
		if (ackdata->count > maxcount)
		{
			rc = -1;
			return rc;
		}
		ackdata->qos[(ackdata->count)++] = readChar(&curdata);
	}
	
	rc = 1;
	return rc;
}

int MQTTDeserialize_unsuback(uint16_t* packetid, uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	uint8_t type = 0;
	int rc = 0;

	if(MQTTDeserialize_ack(&type, packetid, buf, buflen,usedlen)==1)
	{
		if (type == UNSUBACK)rc = 1;
	}
	return rc;
}
int MQTTDeserialize_pingresp(uint8_t* buf, uint16_t buflen,uint16_t *usedlen)
{
	int rc = 0;
	uint8_t type = 0;	
	uint16_t packetid = 0;
	if(MQTTDeserialize_ack(&type, &packetid, buf, buflen,usedlen)==1)
	{
		if (type == PINGRESP)rc = 1;		
	}
	return rc;
}

/*****************************END OF FILE*****************************/
