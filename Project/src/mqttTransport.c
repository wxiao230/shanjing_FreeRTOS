/**
  ******************************************************************************
  * @file    Project/user/mqttTransport.c
  * @author  insentek wxh
  * @version V1.0.0
  * @date    13-July-2020
  * @brief   
  ******************************************************************************   
  */  

/* Includes ------------------------------------------------------------------*/
#include "mqttTransport.h"
//#include "mqttClient.h"
#include "cigemPacket.h"
#include "deviceset.h"

#include <string.h>
#include <stdio.h>
#include "mode.h"
#include "usart.h"
#include "w_ads1x15.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

	uint8_t 	mqttRxBuffer[MQTT_RXBUFFER_LEN+1];
	uint8_t 	mqttTxBuffer[MQTT_TXBUFFER_LEN+1];
	uint16_t	mqttRxLen			=	0;
	uint16_t	mqttRxOutPtr	=	0;
	uint16_t	mqttRxUsedlen	=	0;
	uint16_t	mqttRxUsedPtr	=	0;

	uint8_t sessionPresent	=	1;
	uint8_t	connack_rc 			= MQTT_NOT_CONNECT;
	MQTTPacket_connectData 	connectData		= MQTTPacket_connectData_initializer;
	MQTTPacket_publishData	pubdata 			= MQTTPacket_publishData_initializer;
	MQTTPacket_publishData 	rcpubdata			=	MQTTPacket_publishData_initializer;
	MQTTPacket_subcribeData subcribedata	= MQTTPacket_subcribeData_initializer;
	MQTTPacket_subackData 	subackData		=	MQTTPacket_subackData_initializer;
	uint16_t pubmsgid			=	1;	
	uint16_t pubackmsgid	=	0;
	uint16_t unsubackmsgid	=	0;
	int8_t	 mqttmsgtype		=	0;
//	char 			mqttpubbuf[1024];
	char 			mqtthost[40] 	= {0};
	uint16_t 	mqttport 			= 0;
	__IO uint8_t 	mqttlinknum		=	LINK_NUM_MQTT_1;	
	__IO uint8_t 	mqttLinkChannel	=	0;
	__IO uint16_t	mqttlen	=	0;
	char 			subcribeString[30]={0};
	
	uint8_t		mqtttrytimes		=	0;
	
	uint32_t	mqttcode	=	0;
	
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void mqtt_delay_ms(__IO uint32_t delay)
{									//uint16_1000_3.1s 2000_6.2s 5000_15.5s
	__IO uint32_t i;//uint32_1000_2.3s 2000_4.6s 5000_11.6s
	while(delay)
	{
		delay--;
		for(i=0;i<8300;i++);
	}
}

void clearMqttBuffer(void)
{
	mqttRxLen			=	0;
	mqttRxOutPtr	=	0;
	mqttRxUsedPtr	=	0;
	clearbuffer(mqttRxBuffer,MQTT_RXBUFFER_LEN+1);
}
int transport_sendPacketBuffer(uint8_t* buf, uint16_t len,uint8_t rxFlag,uint8_t linknum)
{
	int rc = 0;
	uint32_t code;
#if((MQTT_DEBUG>1)&&(MQTT_TEST>0))
	printf(" len:%d\r\n",len);
	devStoreBuffer(buf,len);
	printf("\r\n end\r\n");
#endif
	rc = gprsSend(buf,len,MQTT_DATA,linknum);
	if(rxFlag == MQTT_WAIT_ACK_DONE)
	{
		if(readReceivelen(linknum)==0)waitRxGprsData(linknum,&code,STATUS_DONE);
	}
	return rc;
}

int transport_findRxData(uint8_t* buf, uint16_t* rxlen,uint8_t linknum)
{
	int rc = 0;
	rxlen	 = 0;
	clearMqttBuffer();
	rc = findRxGprsData(buf,rxlen,linknum);	
	return rc;
}
int transport_getdata(uint8_t* buf, int count)
{
	int rc = 0;

	if(((mqttRxLen>0)&&(mqttRxLen - mqttRxOutPtr)>=count))
	{
		memcpy(buf, (void*)&mqttRxBuffer[mqttRxOutPtr], count);
		mqttRxOutPtr += count;				
		rc = count;
	}
	
	return rc;
}
#define ISMQTTMSGTYPE(type)	((type == CONNACK)||(type == PUBLISH)||(type == PUBACK)|| \
														 (type == PUBREC)||(type == PUBREL)||(type == PUBCOMP)|| \
														 (type == SUBACK)||(type == UNSUBACK)||(type == PINGRESP))

int transport_rxTask(uint8_t* buf,uint16_t buflenIn,int8_t *msgtype,uint8_t linknum)
{
	int rc 	= 0;
	uint8_t		status = 1;	
	uint16_t	len = 0;
	uint16_t 	buflen=buflenIn;
	
	while(status)
	{
		buflen	=	buflenIn;
		if(mqttRxLen == 0)
		{
			clearMqttBuffer();
			if(readReceivelen(linknum)>0)
			{
			#if(UPGRADE_FRIMWARE_CTR>0)
				if(upgradelinknum==linknum)mqtt_delay_ms(100);
			#endif
				mqtt_delay_ms(20);
				if(findRxGprsData(mqttRxBuffer,&mqttRxLen,linknum)==COMM_OK)
				{
				#if(GPRS_MULTI_LINK>0)	
					clearBuf();
				#endif
				#if((MQTT_DEBUG>0)&&(MQTT_TEST>0))
					printf(" IPD:%d\r\n",mqttRxLen);
					devStoreBuffer(mqttRxBuffer,mqttRxLen);
					printf("\r\n end\r\n");					
				#endif
				}
			}
		}
		if(mqttRxLen == 0)
		{			
			status = 0;break;
		}
		clearbuffer(buf,buflen);
		buflen		=	mqttRxLen-mqttRxOutPtr;
		*msgtype	=	MQTTPacket_read(buf,buflen,transport_getdata);
	#if((MQTT_DEBUG>0)&&(MQTT_TEST>0))
		if(mqttRxLen>0)printf("\r\n mqttRxLen:%d, rxOutPtr:%d,msgtype:%d\r\n", mqttRxLen,mqttRxOutPtr,(int)(*msgtype));
	#endif
		switch(*msgtype)
		{
			case CONNACK:
			{
				if((MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen,&mqttRxUsedlen) == 1) && (connack_rc == 0))
				{
					printf(" connect %d ack, sessionPresent %d\n",linknum,sessionPresent);
				}
				else
				{					
					printf(" Unable to connect %d, return code %d\n",linknum,connack_rc);
				}
				break;
			}
			case PUBLISH:
			{
				if(MQTTDeserialize_publish(&rcpubdata,buf,buflen,&mqttRxUsedlen) == 1)
				{
				#if(UPGRADE_FRIMWARE_CTR>0)
					if(upgradelinknum!=linknum)
				#endif
					{
						printf("\r\n rcpublish dup:%d,qos:%d,retain:%d,msgid:%d,",rcpubdata.dup,rcpubdata.qos,rcpubdata.retain,rcpubdata.packetid);
						printf(" message topic %d:%.*s\r\n", rcpubdata.topicName.lenstring.len, rcpubdata.topicName.lenstring.len, rcpubdata.topicName.lenstring.data); //消息到达					
						printf(" message arrived:%d %.*s\n",rcpubdata.payloadlen,rcpubdata.payloadlen,rcpubdata.payload); //消息到达
					}
				#if(UPGRADE_FRIMWARE_CTR>0)
					if(cigemdevidcheck(rcpubdata.topicName, linknum-1) == 0)		
				#endif
					cigemRxdataTask((uint8_t *)rcpubdata.payload,rcpubdata.payloadlen,buf,linknum-1);
					
					rcpubdata.step	=	PUB_START;
					if(rcpubdata.qos == 2)
					{
						len = MQTTSerialize_pubrec(buf,buflen,rcpubdata.packetid);
						printf(" mqtt pubrec");
						if(transport_sendPacketBuffer(buf,len,MQTT_WAIT_ACK_DONE,linknum)==COMM_OK)
						{
							rcpubdata.step	=	PUB_REC;
						}
					}
					else if(rcpubdata.qos == 1)
					{
						len = MQTTSerialize_puback(buf,buflen,rcpubdata.packetid);
						printf(" mqtt puback");
						if(transport_sendPacketBuffer(buf,len,MQTT_WAIT_ACK_NONE,linknum)==COMM_OK)
						{
							rcpubdata.step	=	PUB_ACK_OK;
						}
					}
					else if(rcpubdata.qos == 0){rcpubdata.step	=	PUB_ACK_OK;}
				}
				else
				{
					*msgtype = 0;
				}
			
				
				break;
			}
			case PUBREL:
			{
				if(MQTTDeserialize_pubrel(&pubackmsgid,buf,buflen,&mqttRxUsedlen) == 1)
				{					
					printf("\r\n pubrel rcmsgid:%d,rcpubmsgid:%d\r\n",pubackmsgid,rcpubdata.packetid);
					if((pubackmsgid == rcpubdata.packetid)&&(rcpubdata.qos==2))
					{
						rcpubdata.step	=	PUB_REL;
						len = MQTTSerialize_pubcomp(buf,buflen,pubackmsgid);
						printf(" mqtt pubcomp");
						if(transport_sendPacketBuffer(buf,len,MQTT_WAIT_ACK_NONE,linknum)==COMM_OK)
						{
							rcpubdata.step	=	PUB_ACK_OK;							
						}
					}
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}
			case PUBACK:
			{
				if(MQTTDeserialize_puback(&pubackmsgid,buf,buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n puback rcmsgid:%d_%d,pubmsgid:%d\r\n",pubdata.qos,pubackmsgid,pubdata.packetid);
					if((pubackmsgid == pubdata.packetid)&&(pubdata.qos==1))
					{
						rcpubdata.step	=	PUB_ACK_OK;
					}					
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}
			case PUBREC:
			{
				if(MQTTDeserialize_pubrec(&pubackmsgid,buf,buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n rcmsgid:%d,pubmsgid:%d\r\n",pubackmsgid,pubdata.packetid);
					if((pubackmsgid == pubdata.packetid)&&(pubdata.qos==2))
					{
						rcpubdata.step	=	PUB_REC;
						len = MQTTSerialize_pubrel(buf,buflen,pubackmsgid);
						printf(" mqtt pubrel");
						if(transport_sendPacketBuffer(buf,len,MQTT_WAIT_ACK_DONE,linknum)==COMM_OK)
						{
							rcpubdata.step	=	PUB_REL;
						
						}
					}
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}			
			case PUBCOMP:
			{
				if(MQTTDeserialize_pubcomp(&pubackmsgid,buf,buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n rcmsgid:%d,pubmsgid:%d\r\n",pubackmsgid,pubdata.packetid);
					if((pubackmsgid == pubdata.packetid)&&(pubdata.qos==2))
					{
						rcpubdata.step =	PUB_ACK_OK;
					}
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}
			case SUBACK:
			{
				if(MQTTDeserialize_suback(1,&subackData,buf, buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n count:%d,submsgid:%d,qos:%d\r\n",subackData.count,subackData.packetid,subackData.qos[0]);
				}
				else
				{
					*msgtype = 0;
				}				
				break;
			}
			case UNSUBACK:
			{
				if(MQTTDeserialize_unsuback(&unsubackmsgid,buf, buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n unsubackmsgid:%d,submsgid:%d\r\n",unsubackmsgid,subackData.packetid);
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}
			case PINGRESP:
			{
				if(MQTTDeserialize_pingresp(buf,buflen,&mqttRxUsedlen) == 1)
				{
					printf("\r\n pingresp %d\r\n",linknum);
				}
				else
				{
					*msgtype = 0;
				}
				break;
			}
			default:
			{				
				mqttRxUsedlen  = 1;				
				break;
			}
		}

		if(ISMQTTMSGTYPE(*msgtype))
		{			
			if(mqttRxUsedlen == 0)
			{
				mqttRxUsedlen = 1;
			}
			else
			{
				status = 0;rc = 1;
			}
		}
		else
		{
			if(mqttRxUsedlen == 0)mqttRxUsedlen = 1;
		}
		
	#if((MQTT_DEBUG>0)&&(MQTT_TEST>0))
		printf("\r\n mqttRxUsedPtr:%d,rxUsedlen:%d\r\n",mqttRxUsedPtr,mqttRxUsedlen);
	#endif
		mqttRxUsedPtr	+= mqttRxUsedlen;
		if(mqttRxUsedPtr>=MQTT_RXBUFFER_LEN)
		{
			mqttRxLen = 0;
			mqttRxUsedPtr = 0;
		}
		mqttRxOutPtr	=	mqttRxUsedPtr;		
		if(mqttRxOutPtr>=mqttRxLen)mqttRxLen=0;				
	}	
	return rc;
}

int transport_open(char* addr, int port,uint8_t linknum)
{
	uint8_t  buf[200];
	int rc = 0;
	uint8_t tryTimes	=	GPRS_TIMES_ALLTRY;
	if(gprsPowerStatus	==	STATUS_NONE)
	{		
		if(gsmTaskInit(0)!=1)return -1;
		if(gprsStart()!=1)return -2;
	//	if(gprsSet(&serverLSRLog[TCP_SERVER_INSENTEK_NODE])!=COMM_OK)return -3;
	}
	while(tryTimes>0)
	{
		tryTimes--;
		rc = GPRSAttach();
		if(rc == COMM_OK)
		{
			rc = setReceiveDataMode();
			if(rc == COMM_OK)rc = findServerIpAddr(buf,addr,port,linknum);
			if(rc == COMM_OK)
			{
				if(gprsLinkStatus(linknum) == COMM_LINK_OK)
				{
//				#if(GPRS_SLEEP_CTR>0)	
					if(connack_rc ==	MQTT_CONNECTION_ACCEPTED)//if((serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(connectData.cleansession == 0))
					{
						mqttlen = MQTTSerialize_disconnect(mqttTxBuffer, MQTT_TXBUFFER_LEN+1);				
						if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_NONE,linknum)==COMM_OK)
						{
						#if(MQTT_USE_CTR>0)	
							printf(" mqtt disconnect %d_%d\r\n",mqttServerNum,linknum);
						#endif
//							mqtt_delay_ms(500);
						}
						connack_rc =	MQTT_NOT_CONNECT;
					}
					if(closeSingleTcp(linknum)==COMM_OK)
					{
						if(serverConfig.gsmRunMode != FUNCITONG_TURN_ON)mqtt_delay_ms(500);
					}
//				#endif
				}
				rc = gprsLink(buf,linknum);//连接服务器	
			}
			if(rc == COMM_OK)
			{
				tryTimes = 0;
				break;
			}
		}
		if(rc != COMM_OK)
		{
			connack_rc =	MQTT_NOT_CONNECT;
			mqtt_delay_ms(1000);
		}
	}
	return rc;
}
//关闭网络
int transport_close(void)
{
	uint8_t buf[100];
//	closeSingleTcp(LINK_NUM_MQTT_1);
//	mqtt_delay_ms(1000);
//	closeTcp();
	
	findServerIpAddr(buf,default_sever_addr,default_sever_port,LINK_NUM_INSENTEK);
	gprsLink(buf,LINK_NUM_INSENTEK);
	//gprsPowerStatus = STATUS_NONE;
//	gsmTaskEnd(LINK_NUM_INSENTEK);
	return 0;
}
void mqttconnectpacket(MQTTPacket_connectData *data,char* clientid,char* user,char* key)
{
	data->MQTTVersion				=	MQTT_VER_311;	
	data->keepAliveInterval = MQTT_KEEPALIVE_TIME;//设置心跳包间隔时间
	data->cleansession 			= 0;	//清除会话
	data->willFlag					=	0;	//无遗嘱
	data->clientID.cstring 	= clientid;//客户端ID
	data->username.cstring 	= user;//用户名
	data->password.cstring 	= key;//密码
}
void mqttsubcribepacket(char *substring,MQTTPacket_subcribeData *data,char *clinetid)
{
	uint16_t rxi=0;
	uint16_t len = 0;
	uint16_t idlen = 0;
	
	data->packetid 	=	1;
	data->qos				=	1;
	
	rxi = sprintf(substring,"%s","$creq/");
	len += rxi;
	
	idlen = strlen(clinetid);
	for(rxi=0;rxi<idlen;rxi++)
	{
		substring[len+rxi] = clinetid[rxi];
	}
	len += rxi;
	
	rxi = sprintf(substring +len,"%s","/+");
	len += rxi;		
	data->topicName.cstring = substring;//"$creq/742189/+";//"$creq/795059/+";
		
	printf("\r\n subcribetopic %d:%s",len,data->topicName.cstring);	
}
void mqttkeypacket(char *key,MQTTString *password)
{
	uint8_t rxi;
	key[0]=strlen(password->cstring);
	for(rxi=0;rxi<key[0];rxi++)
		key[rxi+1]=password->cstring[rxi];
}

void mqttpublishheadsend(MQTTPacket_publishData *data)
{
	data->step			=	PUB_NONE;	
	data->qos				= 1;
	data->retain		=	0;
	data->topicName.cstring	=	"$dp";
	
}
void mqttpublishsensordata(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char*clientid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketSensorData((char*)buf,clientid);	
	data->payload		=	(char*)buf;	
	mqttpublishheadsend(data);
}
void mqttpublishdevstatus(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char*clientid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketDevstatusPub((char*)buf,clientid);	
	data->payload		=	(char*)buf;	
	mqttpublishheadsend(data);
}

void mqttpublishheadack(MQTTPacket_publishData *data)
{
	data->step			=	PUB_NONE;	
	data->qos				= 0;
	data->retain		=	1;
	data->topicName.cstring	=	"$dr";	
}

void mqttpublishackreqtime(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckReqtime((char*)buf,rtcTime);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackresult(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char *cmd,uint8_t flag)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckResult((char*)buf,cmd,flag);	
	data->payload			=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackgetstatus(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckGetstatus((char*)buf);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackgetsensorID(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckGetsensorID((char*)buf);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishacksample(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckSample((char*)buf);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackreqsensortime(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,uint8_t sensoridindex,cigemsensortime_t period)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckdevPeriod((char*)buf,sensoridindex,period);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishacksensorrange(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,uint8_t sensoridindex,cigemsensorrange_t range)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckGetsensorattr((char*)buf,sensoridindex,range);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackworkmode(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid,char mode)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckGetworkmode((char*)buf,mode);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
void mqttpublishackcmdversion(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
{
	data->dup 				= dup;
	data->packetid		=	packetid;
	data->payloadlen	=	cigemPacketAckGetcmdversion((char*)buf);	
	data->payload		=	(char*)buf;	
	mqttpublishheadack(data);
}
#if(SECOND_MQTT_ADDR>0)
	void mqttpublishackgetplatform(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
	{
		data->dup 				= dup;
		data->packetid		=	packetid;
		data->payloadlen	=	cigemPacketAckGetplatform((char*)buf);	
		data->payload		=	(char*)buf;	
		mqttpublishheadack(data);
	}
#endif
#if(UPGRADE_FRIMWARE_CTR>0)
	void mqttpublishsupportsize(char *substring,MQTTPacket_publishData *data,char *clinetid)
	{
		uint16_t rxi = 0;
		uint16_t len = 0;
		uint16_t idlen = 0;		
		rxi = sprintf(substring,"%s","$creq/");
		len += rxi;
		
		idlen = strlen(clinetid);
		for(rxi=0;rxi<idlen;rxi++)
		{
			substring[len+rxi] = clinetid[rxi];
		}
		len += rxi;
		
		rxi = sprintf(substring +len,"%s","/supportsize");
		len += rxi;
		substring[len] = '\0';		
		data->step			=	PUB_NONE;	
		data->qos				= 1;
		data->retain		=	0;
		data->topicName.cstring	=	substring;
		printf("\r\n supportsize %d:%s",len,data->topicName.cstring);
	}
	void mqttpublishackcmdupgrade(MQTTPacket_publishData *data,uint8_t *buf,uint8_t dup,uint16_t packetid)
	{		
		data->dup 				= dup;
		data->packetid		=	packetid;
		data->payloadlen	=	cigemPacketAckUpgrade((char*)buf);	
		data->payload			=	(char*)buf;		
	}
#endif
extern void	mqttParamSave(void);	
void mqttcigemrxcmdTask(uint8_t *buf,uint16_t buflen,uint8_t linknum)
{
	uint8_t dup = 0;
	uint16_t len;
	uint8_t buffer[1024];
	uint8_t flag = 0;
#if(UPGRADE_FRIMWARE_CTR>0)
	char	supportsizeString[50]={0};
#endif
	switch(cigemcmd)
	{
		case CIGEM_CMD_reqtime:
		{
			mqttpublishackreqtime(&pubdata,buffer,dup,rcpubdata.packetid);
			break;
		}
		case CIGEM_CMD_settime:
		{
			if(cigemsettimeflag == CIGEM_CMD_FLAG_ok)
			{
//				if(serverConfig.gsmRunMode != FUNCITONG_TURN_ON)cigemsettimeflag =	CIGEM_CMD_FLAG_none;
				mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"settime",SUCCESS);
			}
			else
			{
				cigemcmd	=	CIGEM_CMD_none;
			}
			break;
		}
		case CIGEM_CMD_getstatus:
		{
			batStatus 		= Read_Value(BAT_channel)*6.144/ADS_MAXVALUE; 
			adsexterValue = Read_Value(SUN_channel)*6.144/ADS_MAXVALUE;
			mqttpublishackgetstatus(&pubdata,buffer,dup,rcpubdata.packetid);
			break;
		}
		case CIGEM_CMD_reboot:
		{
			if(cigemrebootflag == CIGEM_CMD_FLAG_ok)
			{
				cigemrebootflag	=	CIGEM_CMD_FLAG_none;
				mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"reboot",SUCCESS);
			}
			else
			{
				cigemcmd	=	CIGEM_CMD_none;
			}
			break;
		}	
		case CIGEM_CMD_getsensorID:
		{
			mqttpublishackgetsensorID(&pubdata,buffer,dup,rcpubdata.packetid);
			break;
		}	
		case CIGEM_CMD_sample:
		{
			if(cigemsampleflag == CIGEM_CMD_FLAG_ok)
			{
				sensorValueCurrent();
				sensorValue.time = rtcTime;
//				cigemsampleflag	=	CIGEM_CMD_FLAG_none;
				mqttpublishacksample(&pubdata,buffer,dup,rcpubdata.packetid);
			}
			else
			{
				cigemcmd	=	CIGEM_CMD_none;
			}
			break;
		}	
		case CIGEM_CMD_setsensortime:
		{
			//setsensortime  			
			if(cigemsetsensortimeflag == CIGEM_CMD_FLAG_ok){flag = SUCCESS;}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"setsensortime",flag);
			cigemsetsensortimeflag	=	CIGEM_CMD_FLAG_none;
			break;
		}	
		case CIGEM_CMD_reqsensortime:
		{			
			if((cigemsensoridindex&0x0F) < SENSORID_INDEX_NUM)//201210
			{
				mqttpublishackreqsensortime(&pubdata,buffer,dup,rcpubdata.packetid,cigemsensoridindex,cigemsensortime);
			}
			break;
		}	
		case CIGEM_CMD_setsensorattr:
		{
			if(cigemsetsensorrangeflag == CIGEM_CMD_FLAG_ok){flag = SUCCESS;}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"setsensorattr",flag);
			cigemsetsensorrangeflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_getsensorattr:
		{
			if((cigemsensoridindex&0x0F) < SENSORID_INDEX_NUM)//0x0F 
			{
				mqttpublishacksensorrange(&pubdata,buffer,dup,rcpubdata.packetid,cigemsensoridindex,cigemsensorlimit[cigemsensoridindex&0x0F]);
			}			
			break;
		}
		case CIGEM_CMD_setworkmode:
		{
			if(cigemresultflag == CIGEM_CMD_FLAG_ok){flag = SUCCESS;}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"setworkmode",flag);
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_getworkmode:
		{
			mqttpublishackworkmode(&pubdata,buffer,dup,rcpubdata.packetid,cigemworkmode);			
			break;
		}
		case CIGEM_CMD_getcmdversion:
		{
			mqttpublishackcmdversion(&pubdata,buffer,dup,rcpubdata.packetid);			
			break;
		}		
		case CIGEM_CMD_warning:
		{
			if(cigemresultflag == CIGEM_CMD_FLAG_ok)
			{
				flag = SUCCESS;
			}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"meteorologicalearlywarning",flag);
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_alarm:
		{
		#if(MQTT_TEST_SCH_CTR>0)
			if(serverConfig.gsmRunMode == FUNCITONG_TURN_ON)cigemresultflag = CIGEM_CMD_FLAG_ok;
		#endif
			if(cigemresultflag == CIGEM_CMD_FLAG_ok)
			{				
				flag = SUCCESS;
			}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"alarm",flag);
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_broadcast:
		{
		#if(MQTT_TEST_SCH_CTR>0)
			if(serverConfig.gsmRunMode == FUNCITONG_TURN_ON)cigemresultflag = CIGEM_CMD_FLAG_ok;
		#endif
			if(cigemresultflag == CIGEM_CMD_FLAG_ok)
			{				
				flag = SUCCESS;
			}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"broadcast",flag);
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_upgrade:
		{
		#if(UPGRADE_FRIMWARE_CTR>0)
			if(cigemupgradeflag	== CIGEM_CMD_FLAG_ok)
			{
				mqttpublishackcmdupgrade(&pubdata,buffer,dup,rcpubdata.packetid);
				mqttpublishsupportsize(supportsizeString,&pubdata,&mqttServerAddrBuf[linknum-1].deviceId[1]);
			}
			else
			{
				mqttParamSave();
				cigemcmd	=	CIGEM_CMD_none;
			}
			break;
		#endif
		}
	#if(SECOND_MQTT_ADDR>0)
		case CIGEM_CMD_setplatform:
		{
			if(cigemresultflag == CIGEM_CMD_FLAG_ok){flag = SUCCESS;}			
			else{flag = ERROR;} 
			mqttpublishackresult(&pubdata,buffer,dup,rcpubdata.packetid,"setplatform",flag);
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
		case CIGEM_CMD_getplatform:
		{
			if(cigemresultflag == CIGEM_CMD_FLAG_ok)
			{
				mqttpublishackgetplatform(&pubdata,buffer,dup,rcpubdata.packetid);
			}		
			cigemresultflag	=	CIGEM_CMD_FLAG_none;
			break;
		}
	#endif
		case CIGEM_CMD_supportsize:		
		case CIGEM_CMD_none:
		default:cigemcmd	=	CIGEM_CMD_none;break;
	}
	if(cigemcmd	!= CIGEM_CMD_none)
	{	
		len = cigemPacketAckapikey((char*)(buffer+pubdata.payloadlen),cigemapikey,&cigemmsgid[linknum-1][0]);
		pubdata.payloadlen += len;
		len = MQTTSerialize_publish(buf,buflen,pubdata);

		printf("\r\n mqtt publish cmdack:%d",cigemcmd);
		if(transport_sendPacketBuffer(buf,len,MQTT_WAIT_ACK_NONE,linknum)==COMM_OK)
		{
			cigemcmdflag[linknum-1] = CIGEM_CMD_FLAG_ok;//201227
		#if(UPGRADE_FRIMWARE_CTR>0)
			if((cigemcmd	== CIGEM_CMD_upgrade)&&(cigemupgradeflag	== CIGEM_CMD_FLAG_ok))
			{
				clearbuffer(cigemmsgid[linknum-1],CIGEM_MSGID_LEN);
				cigemupgradeflag	=	CIGEM_CMD_FLAG_waitrx;
			}
		#endif
		}
	#if(MQTT_USE_CTR>0)
		mqttParamSave();
	#endif
		cigemcmd	=	CIGEM_CMD_none;
	}
}
#if(MQTT_USE_CTR>0)
void mqttunsendpacketnum(void)
{
	uint8_t rxi=0;
	printf(" unSendPacketStation:");
	for(rxi=0;rxi<mqttServerNum;rxi++)
	{
		unSendDataMqtt[rxi]++;
		printf("%d,",unSendDataMqtt[rxi]);
	}
}
void mqttunsendpacketnumadj(void)
{
	uint8_t rxi=0;
	printf(" unSendPacketStation:");
	for(rxi=0;rxi<mqttServerNum;rxi++)
	{		
		unSendDataMqtt[rxi] = unSendPackNumAdj(curMqttSendAddr[rxi].i);//20210115
		printf("%d,",unSendDataMqtt[rxi]);
	}
}
int8_t  mqttunsendpacketCheck(httpConfig *config)
{
	uint8_t rxi=0;	
	for(rxi=0;rxi<mqttServerNum;rxi++)
	{
		if(unSendDataMqtt[rxi] >= config->devDensity)return 0;
	}
	return -1;
}
int8_t mqttserverinit(uint8_t linknum)
{	
	uint8_t rxi=0;
	if(linknum>=mqttServerNum)return -1;
	if(mqttServerAddrBuf[linknum].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)	
	{
		
		mqttlinknum = linknum+1;
		for(rxi=0;rxi<mqttServerAddrBuf[linknum].endpoint[0];rxi++)
			mqtthost[rxi] = mqttServerAddrBuf[linknum].endpoint[rxi+1];
		mqttport = mqttServerAddrBuf[linknum].port;
		
		mqttconnectpacket(&connectData,&mqttServerAddrBuf[linknum].deviceId[1],&mqttServerAddrBuf[linknum].username[1],&mqttServerAddrBuf[linknum].password[1]);
		
		mqttkeypacket((char*)cigemapikey,&connectData.password);
		/* 订阅 */
		mqttsubcribepacket(subcribeString,&subcribedata,connectData.clientID.cstring);
		return 0;
	}
	return -2;
}
#endif
/*****************************END OF FILE*****************************/
