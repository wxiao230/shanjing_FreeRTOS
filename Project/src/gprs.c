
#include "main.h"
#include "gprs.h"
#include "usart.h"
#include "delay.h"
#include "sensor.h"
#include "deviceset.h"
#include "rtc.h"
#include "ds18b20.h"

#include "spi_flash.h"
#include "mode.h"
#include "w_ads1x15.h"
#include "stm32_mems_adapter.h"
#include "power.h"
#include "hydrology.h"
#include <stdlib.h>
#include <stdarg.h>
#include "crc16.h"
#if((SK60_CTR>0)||(KXYL_CTR>0))
	#include "sk60.h"
#endif
#include <string.h>
#if(MQTT_USE_CTR>0)
	#include "cigemPacket.h"
#endif
#include <math.h>
#if(USART_USE_CTR>0)
	#include "rs485.h"
#endif

#include "qingxie.h"

#define GPS_SMS_DEBUG  	1


extern __IO uint32_t beginTime;
extern __IO uint32_t castWorkTime;


static uint16_t findStrRxGprs(char *buf,uint8_t bufLength);
//static uint8_t rxGprs(uint8_t *code);


static void stationReset(void);


static void stationPwReset(void);
static void gprsBufHeadInit(void);

extern void xl345InitSet(uint8_t intStatus);

//__IO uint8_t setPerChangeFlag = NO_CHANGE;

//��������ַ����
//uint8_t serAddrBuf[MAX_GPRS_SERIVER_ADDERSS_SIZE];


#define GPRS_SERVER_TEST 	0

//�̼��汾��Ϣ���ַ�������
//uint8_t firwMare[]="istsms04";
//uint8_t firwMare[]="ist80804";//"qx180611";
//�汾����
//qxABBCCD qx��ʾɽ����б����A�������֣�1��׼��Ʒ��2������λ�Ʋ�Ʒ,3 4G��׼��Ʒ��4 4G�������Ʒ��
//5 4G ������Ϳ�϶ˮѹ��
//BB �� CC �� D ���µڼ����޶�����
//sjABBCCD 151RC

#if defined(STM32L1XX_MDP)
	#if(SK60_CTR>0)
		uint8_t firwMare[]="sj521081";//sj320091 sj520121 sj520121 sj521071 sj521072 sj521081
	#else
		uint8_t firwMare[]="sj323051";//sj320091 sj320092 sj320121 sj320122 sj321011 sj321012 sj321021 sj321071
	#endif
#elif(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	#if(SK60_CTR>0)
		#if(KXYL_CTR>0)
			uint8_t firwMare[]="qx520081";//
		#else
			uint8_t firwMare[]="qx520081";//
		#endif
	#else
		uint8_t firwMare[]="qx321021";//qx320051 qx321021
	#endif
#else
	#if(SK60_CTR>0)
		uint8_t firwMare[]="qx220122";//qx220122 qx219122 qx219121 qx219091
	#else
		uint8_t firwMare[]="qx121021";//qx121021 qx180713 qx180702 qx180619 
	#endif
#endif

//#if(GPRS_SIM_TYPE==GPRS_SIM_TianYi)	
//char apnBufInit[17]="ctnet\r\n";//�й����� CTNET 	
////	char	apnBufSet[17] =	"ctnet\r\n";//�й����� CTNET 	
//#else
//char apnBufInit[17]="cmnet\r\n";//�й��ƶ� //CMNET
////	char	apnBufSet[17] =	"cmnet\r\n";//�й��ƶ� //CMNET;
//#endif
char 		apnBufAuto[APN_LEN_MAX+2]="cmnet\r\n";//�й��ƶ� //CMNET 17 32_211215
char  	apnBufSet[APN_LEN_MAX+2] =	{0,0,0};//"CMNET\r\n";//�й��ƶ� //CMNET;//211215
int8_t 	devApnSetStatus = APN_SAVE_STATUS_INIT;
__IO uint8_t  netlinkFlag = STATUS_NONE; //211215

__IO uint8_t	netOperator = NET_OPERATOR_NORMAL;
#define TIME_FULL            0x00
#define TIME_UN_FULL         0x01

uint8_t rxTimeFlag = TIME_UN_FULL;

/*
gprs��ͬ����ʱ�䶨��
*/
#define GPRS_RX_TIME						0x01
#define GPRS_OK_TIME						0x01
#define GPRS_LINK_TIME					0x05//0x05
#define GPRS_SEND_TIME					0x05//0x03 
#define GPRS_LINK_TIME_OUT			0x45//0x45//0x30
#define GPRS_SEND_TIME_OUT			0x45//0x45
#define GPRS_OK_TIME_OUT				0x30//0x45//0x30
#define GPRS_RX_TIME_OUT				0x45//0x45//20
#define GPRS_TX_TIME_OUT				0x10

#define GPRS_ACK_OK							0x01
#define GPRS_ACK_SET						0x02
#define GPRS_ACK_RX							0x03
#define GPRS_ACK_SEND						0x04

#define GPRS_ACK_OK_TIME_OUT		0x02
#define GPRS_ACK_SET_TIME_OUT		0x02//0x10
#define GPRS_ACK_RX_TIME_OUT		0x10
#define GPRS_ACK_SEND_TIME_OUT	0x45
#define GPRS_ACK_SEND_TIME_OUT2	0x20

//GPS����ģʽ
#define COLD_START_MODE    			0x01
#define AUTONOMY_MODE      			0x02

#define GPS_0N        					0x01//GPS POWER ON
#define GPS_OFF       					0x02//GPS POWER OFF

//GPRS���ڿ�ʼ�������ݱ�־
static __IO uint8_t uartState 	= UN_RX;
static __IO uint8_t gprsFlag 		= NO_COMM;
static __IO uint8_t gprsRxState = UN_RX;
static __IO uint8_t gprsState 	= GPRS_TURN_ON;

uint8_t  GPRS_RX_Buffer[GPRS_RX_DATA_SIZE+1];
static __IO uint16_t GPRS_RX_ptr_Out = 0;
static __IO uint16_t GPRS_RX_ptr_Store = 0;
static __IO uint16_t GPRS_RXCounter		= 0;

static __IO uint8_t gprsRxCount  = 0x00;
static __IO uint8_t commTxCount  = 0x00;

__IO uint8_t simCardType = GPRS_SIM_TYPE;//SIM������

uint16_t lacValue = 0;
uint16_t cellIdValue = 0;
uint16_t rssiValue = 0x00;

static  __IO	uint8_t cureTcpLinkChannel = 0x00;		
uint16_t simCardErrorCounter = 0x00;
uint16_t netGsmErrorCounter = 0x00;

__IO uint16_t gsmIntFlag = 0x00;
fireWareStruct  curFireWare;

serverAddr tcpServerAddrBuf[MAX_MULTI_CONNECTION_SIZE];
lsrLog serverLSRLog[MAX_MULTI_CONNECTION_SIZE];

serverAddr stationServerAddrBuf[STATION_MULTI_CONNECTION_SIZE];
lsrLog stationServerLSRLog[STATION_MULTI_CONNECTION_SIZE];
stationSend curStationSend[STATION_MULTI_CONNECTION_SIZE];

#if(MQTT_USE_CTR>0)
	mqttServerAddr mqttServerAddrBuf[MQTT_MAX_MULTI_CONNECTION_SIZE];	
	__IO uint8_t mqttServerNum	=	0;
	__IO int8_t mqttSeverStatus[MQTT_MAX_MULTI_CONNECTION_SIZE]={serverUnused,serverUnused};
	__IO uint16_t unSendDataMqtt[MQTT_MAX_MULTI_CONNECTION_SIZE]={0,0};
	value_t	curMqttSendAddr[MQTT_MAX_MULTI_CONNECTION_SIZE];
	__IO uint8_t mqttServerNumFirst	=	0;
	__IO uint8_t mqttServerNumSecond	=	0;
#if(SECOND_MQTT_ADDR>0)
//	secondServerAddr mqttSecondServerAddrBuf[SECOND_MULTI_CONNECTION_SIZE];	
	uint32_t secondSendAddr[SECOND_MULTI_CONNECTION_SIZE];
#endif
#endif
 
__IO uint8_t 	gprsPowerStatus	= STATUS_NONE;
__IO uint16_t gprsPowerOnTime	=	0;
__IO uint8_t 	gprsTryTimes	=	0;
__IO uint16_t gprsWaiteAckTimes		=	0;
__IO uint8_t 	gprsAckFlag		=	0;
__IO uint8_t 	gprsAckState	=	GPRS_ACK_SET;
__IO uint8_t 	gprsAckprintf = 1;

 __IO uint8_t serverCounter = 0x00;

#define MAX_PACKET_HEDA_SIZE 100

/* Safe sprintf: returns length or 0 on overflow */
static inline int safe_sprintf(char *buf, size_t offset, size_t max_size, const char *fmt, ...)
{
    va_list args;
    int len;
    if (offset >= max_size) return 0;
    va_start(args, fmt);
    len = vsnprintf(buf + offset, max_size - offset, fmt, args);
    va_end(args);
    if (len < 0 || (size_t)len >= max_size - offset) {
        printf("ERROR: Buffer overflow at %s:%d\r\n", __FILE__, __LINE__);
        return 0;
    }
    return len;
}
char packetHeadBuf[MAX_PACKET_HEDA_SIZE];
static __IO  uint16_t packHeadLength = 0x00; 
netInfo_t netInformation;
 
devConfig  configValue;//�豸����������ֵ
paramter devPar;//�豸����
double devLongitude = 0.0;
double devLatitude  = 0.0;
void serverAddrDebug(void)
{
	uint16_t datalength = 0x00;
	uint16_t bufLength  = 0x00;
	uint8_t testi 			= 0x00;
	for(testi = 0x00;testi < MAX_MULTI_CONNECTION_SIZE;testi ++)
	{

		tcpServerAddrBuf[testi].userFlag 			= TCP_SERVER_ADDRESS_UNUSE_FLAG;
		serverLSRLog[testi].linkErrorCounter 	= 0x00;
		serverLSRLog[testi].sendErrorCounter 	= 0x00;
		serverLSRLog[testi].rxErrorCounter 		= 0x00;
		serverLSRLog[testi].dataOkCounter			= 0x00;
		serverLSRLog[testi].allTry						= 0x00;
	}
	tcpServerAddrBuf[TCP_SERVER_INSENTEK].userFlag 	= TCP_SERVER_ADDRESS_USE_FLAG;
	tcpServerAddrBuf[TCP_SERVER_INSENTEK].packType	= EXT_PACKET;
	tcpServerAddrBuf[TCP_SERVER_INSENTEK].sendPort	= 54862;//80;
	
	datalength 	= 0x01;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	bufLength 	=	sprintf(tcpServerAddrBuf[TCP_SERVER_INSENTEK].sendAddressBuf+datalength, "%s", "AT+CIPOPEN=0,\"TCP\",\"api.insentek.com\",54862\r\n");
#else
	bufLength 	=	sprintf(tcpServerAddrBuf[TCP_SERVER_INSENTEK].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"api.insentek.com\",\"54862\"\r\n");
#endif
	tcpServerAddrBuf[TCP_SERVER_INSENTEK].sendAddressBuf[0]=bufLength;				
	datalength +=bufLength;
	datalength 	= 0x01;
	bufLength 	=	sprintf(tcpServerAddrBuf[TCP_SERVER_INSENTEK].baseAddressBuf+datalength, "%s", "api.insentek.com");
	datalength +=bufLength;
	tcpServerAddrBuf[TCP_SERVER_INSENTEK].baseAddressBuf[0] = datalength;
}

/*******************************************************************************************************
*
*			  define the gprs gpio pin
*
*******************************************************************************************************/

#define RCC_GPIO_GPRS			RCC_AHBPeriph_GPIOB
#define GPIO_GPRS_PORT		GPIOB
#define GPRS_P      			GPIO_Pin_11			 //working led

void gprsPowerOn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_GPIO_GPRS, ENABLE);//RCC_APB2Periph_AFIO

  GPIO_InitStructure.GPIO_Pin 	= GPRS_P;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     
  GPIO_Init(GPIO_GPRS_PORT, &GPIO_InitStructure);

  GPIO_SetBits(GPIO_GPRS_PORT,GPRS_P);
	gprsPowerOnTime	=	0;
	gprsPowerStatus	=	STATUS_DONE;
}

void gprsPowerOff(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_GPIO_GPRS, ENABLE);  //RCC_APB2Periph_AFIO
	gprsPowerStatus	=	STATUS_NONE;
	GPIO_ResetBits(GPIO_GPRS_PORT,GPRS_P);

	GPIO_InitStructure.GPIO_Pin 	= GPRS_P;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  
	GPIO_Init(GPIO_GPRS_PORT, &GPIO_InitStructure);
}

//__IO  uint16_t sendDataLength = 0x00;

void gprsReceive(void)
{
	if(gprsRxState == RX_STATE)
	{
		if(uartState ==UN_RX)
		{ 
			uartState = RX_STATE;	   //�ı���ձ�־λ״̬ �������ն�ʱ
			gprsRxCount = 0x00;       //  �������ն�ʱ
		}
		GPRS_RX_Buffer[GPRS_RX_ptr_Store++] = USART_ReceiveData(GPRS_USART);
		GPRS_RX_ptr_Store = GPRS_RX_ptr_Store&GPRS_RX_DATA_SIZE;
		//GPRS_RXCounter ++;
	}
	else
	{
		(void)USART_ReceiveData(GPRS_USART);
	}
}

__IO uint8_t gprsTimerCtr = TIME_END;

void gprsTimeDeal(void)
{
	gprsPowerOnTime++;
	if(gprsAckFlag == COMM_ACK)
	{
		gprsWaiteAckTimes++;		
		switch(gprsAckState)
		{
			case GPRS_ACK_OK:
			{
				if(gprsWaiteAckTimes >= GPRS_ACK_OK_TIME_OUT)
				{
					gprsWaiteAckTimes = 0x00;				
					gprsAckFlag  = COMM_TIMEOUT;	
				}
				break;
			}
			case GPRS_ACK_SET:
			{
				if(gprsWaiteAckTimes >= GPRS_ACK_SET_TIME_OUT)
				{
					gprsWaiteAckTimes = 0x00;				
					gprsAckFlag  = COMM_TIMEOUT;	
				}
				break;
			}
			case GPRS_ACK_RX:
			{
				if(gprsWaiteAckTimes >= GPRS_ACK_RX_TIME_OUT)
				{
					gprsWaiteAckTimes = 0x00;				
					gprsAckFlag  = COMM_TIMEOUT;	
				}
				break;
			}
			case GPRS_ACK_SEND:
			default:
			{
				if(gprsWaiteAckTimes >= GPRS_ACK_SEND_TIME_OUT)
				{
					gprsWaiteAckTimes = 0x00;				
					gprsAckFlag  = COMM_TIMEOUT;	
				}
			#if(GPRS_SLEEP_CTR>0)
				else if((serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(gprsWaiteAckTimes >= GPRS_ACK_SEND_TIME_OUT2))//20210105
				{
					gprsWaiteAckTimes = 0x00;				
					gprsAckFlag  = COMM_TIMEOUT;
				}
			#endif
				break;
			}
		}
	}	
#if 0
	if(uartState == RX_STATE)
	{
		gprsRxCount ++;
		switch(gprsState) 
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			case GPRS_DNS2IP :
			{
				if(gprsRxCount >= 0x0b)
				{
					commTxCount = 0x00;
					rxTimeFlag 	=	TIME_FULL;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
		#endif
			case GPRS_LINK_TEST :
			case GPRS_LINK :
			{
				if(gprsRxCount >= GPRS_LINK_TIME)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			case GPRS_SINGEL_CLOSE :			
			case GPRS_RX :
			case GPRS_SEND :
			{
				if(gprsRxCount >= GPRS_SEND_TIME)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			case GPRS_OK :
			{
				if(gprsRxCount >= GPRS_OK_TIME)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			case GPRS_UART_INIT :						
			case GPRS_SET :
			case GPRS_CLOSE :
			case GPRS_UNlINK :		
			case GPRS_STATUS_QUERY :
			case GPRS_TURN_OFF :
			case GPRS_GPS_START :
			case GPRS_GPS_STATUS_QUERY :
			case GPRS_GPS_VALUE :
			case GPRS_GPS_POWER_OFF :				
			case GPRS_SLEEP_MODE :            
			{
				if(gprsRxCount >= GPRS_RX_TIME)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			case GPRS_SMS_SEND :				
			case GPRS_SMS_SEND_COMM :
			{
				if(gprsRxCount > 0x01)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			case GPRS_SMS_TOTAL_READ  :
			case GPRS_SMS_READ     :
			case GPRS_SMS_DEL      : 
			{
				if(gprsRxCount >= 0x03)
				{
					rxTimeFlag 	=	TIME_FULL;
					commTxCount = 0x00;
					gprsFlag  	= COMM_BACK	;
					uartState 	= UN_RX;
				}
				break;
			}
			default :
			{
				rxTimeFlag 	=	TIME_FULL;
				commTxCount = 0x00;
				gprsFlag  	= COMM_BACK	;
				uartState 	= UN_RX;				
				break; 		 		  
			}
		}	
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		if(gprsRxCount >0x46)
		{
			commTxCount = 0x00;
			gprsFlag  	= COMM_BACK	;
			rxTimeFlag 	=	TIME_FULL;
			uartState 	= UN_RX;	 
		}
	#endif
	}

	if((gprsFlag  == COMM_ACK)&&((uartState != RX_STATE)))
	{
		commTxCount ++;
		switch(gprsState) 
		{
			case GPRS_LINK_TEST:
			case GPRS_LINK :
			case GPRS_SINGEL_CLOSE :
			{
				if(commTxCount >= GPRS_LINK_TIME_OUT)
				{
					commTxCount = 0x00;
				//commTimeFlage = 	TIME_FULL;
					gprsFlag  = COMM_TIMEOUT	;
				// uartState = UN_RX;
				}
				break;
			}
			case GPRS_SMS_SEND :
			case GPRS_SEND :
			case GPRS_GPS_STATUS_QUERY :
			{
				if(commTxCount >= GPRS_SEND_TIME_OUT)
				{
					commTxCount = 0x00;
				//	commTimeFlage = 	TIME_FULL;
					gprsFlag  = COMM_TIMEOUT	;	
				}
				break;
			} 
			case GPRS_RX:
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			case GPRS_DNS2IP :
		#endif
			{
				if(commTxCount >= GPRS_RX_TIME_OUT)
				{
					commTxCount = 0x00;
				//	commTimeFlage = 	TIME_FULL;
					gprsFlag  = COMM_TIMEOUT	;	
				}
				break;
			}
			case GPRS_OK :
			{
				if(commTxCount>=GPRS_OK_TIME_OUT)
				{
					commTxCount = 0x00;
				//	commTimeFlage = 	TIME_FULL;
					gprsFlag  = COMM_TIMEOUT	;			 
				}
				break;
			}			
			case GPRS_UART_INIT :
			case GPRS_SET :
			case GPRS_CLOSE :
			case GPRS_UNlINK :
			case GPRS_STATUS_QUERY :
			case GPRS_TURN_OFF :
			case GPRS_GPS_START :
			case GPRS_GPS_VALUE :
			case GPRS_GPS_POWER_OFF :			
			case GPRS_SLEEP_MODE :     
			{
				if(commTxCount >= GPRS_TX_TIME_OUT)
				{
					commTxCount = 0x00;
				//	commTimeFlage = 	TIME_FULL;
					gprsFlag  = COMM_TIMEOUT	;	
				}
				break;
			}
			case GPRS_SMS_TOTAL_READ	:
			case GPRS_SMS_READ     		:
			case GPRS_SMS_DEL      		:			
			case GPRS_SMS_SEND_COMM 	:
			{
				if(commTxCount >= 0x03)
				{
					commTxCount = 0x00;
					gprsFlag  	= COMM_TIMEOUT	;
					rxTimeFlag 	=	TIME_FULL;					
				}
				break;
			}		 			
			default :
			{
				commTxCount = 0x00;
				gprsFlag		= COMM_TIMEOUT	;
				rxTimeFlag	=	TIME_FULL;			
				break; 
			}
		}
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
		if(commTxCount > 0x46)
		{
			commTxCount = 0x00;
			gprsFlag  	= COMM_TIMEOUT;
			rxTimeFlag 	=	TIME_FULL;			
		}
	#endif
	}
	
	
#endif		
}

static void delay_GprsMs(uint32_t delay)
{
  if(isSchedulerRunning())
  {
    vTaskDelay(pdMS_TO_TICKS(delay));
    return;
  }
  uint32_t i;
	while(delay)
	{
		delay--;
		for(i=0;i<8300;i++);
	}	
}
void delay_GprsS(__IO uint16_t start,__IO uint16_t duration)
{
	while(gprsPowerOnTime<(start+duration))delay_GprsMs(500);
}

uint32_t hex2int(uint8_t *buf)
{
    uint8_t i = 0;
	uint8_t ibuf[4];

	value_t j ;
	for(i = 0;i<4;i++)
	{
	if(buf[i]<0x3A)
	{
	   ibuf[i] = buf[i]-0x30;
	}else
	{
	   if(buf[i] <0x60)
	   {
	      ibuf[i] = buf[i]-0x41+0x0A;
	   }
	   else
	   {
	      ibuf[i] = buf[i]-0x61+0x0A;
	   }
	}
	}
	j. bytes.value2 = ((ibuf[0]<<4)&0xF0)|(ibuf[1]&0x0F);
	j. bytes.value1 = ((ibuf[2]<<4)&0xF0)|(ibuf[3]&0x0F);
	j. bytes.value3 = 0x00;
	j. bytes.value4 = 0x00;

	return j.i;

}

void servStoreFlagInit(httpConfig *flagStore)
{
//    flagStore->stationServerChangeFlag 	= 0x00;   
//    flagStore->delFlagHost2 						= 0x00;
    flagStore->storeFlagAbc 						= 0x00;
    flagStore->storeFlagAlarmSmsPhone 	= 0x00;
    flagStore->storeFlagConfig 					= 0x00;
    flagStore->storeFlagDataSmsPhone 		= 0x00;
    flagStore->storeFlagParam 					= 0x00;
    flagStore->storeFlagStationServer 	= 0x00;
    flagStore->storeFlagStationName  		= 0x00;
    flagStore->storeFlagFaw							= 0x00;
    flagStore->storeFlagStationPw 			= 0x00;
    flagStore->staionResetFlag					= 0x00;
    flagStore->staionFlashResetFlag			= 0x00;
    flagStore->storeDepFlag 						= 0x00;
//		flagStore->getIpFlag 								= 0;
//		flagStore->linkErrorCouner 					= 0;
		flagStore->storeFlagSensorRange			=	0;
		flagStore->devApnSetFlag						=	0;//230508
}

void clearBufPart(uint16_t status,uint16_t len)
{
	uint16_t rxi=0;
	for(rxi = 1;rxi<= len;rxi++)
	{
		GPRS_RX_Buffer[(status-rxi)&GPRS_RX_DATA_SIZE] = 0xFE;					
	}
}
void clearGprsFlag(void)
{
	gprsRxCount	= 0x00;
	commTxCount	= 0x00;
	uartState		= UN_RX;
}
void clearBufPtr(void)
{
	GPRS_RX_ptr_Out 	= 0;
	GPRS_RX_ptr_Store	=	0;
//	GPRS_RXCounter 		= 0;
}
void clearPtrOut(uint16_t ptrOut)
{
	GPRS_RX_ptr_Out = ptrOut&GPRS_RX_DATA_SIZE;	
}
uint16_t gprsRxcounterCal(uint16_t ptrOut)
{
	uint16_t rxLen=0;
	uint16_t ptrStore = 0;
	ptrStore	=	GPRS_RX_ptr_Store;
	if(ptrStore >= ptrOut)
	{
		rxLen	=	ptrStore - ptrOut;		
	}
	else
	{
		rxLen	=	GPRS_RX_DATA_SIZE + 1 + ptrStore - ptrOut;	
	}
	return rxLen;
}
uint16_t gprsDataAverived(void)
{
	return gprsRxcounterCal(GPRS_RX_ptr_Out);	
}
void clearBuf(void)
{	
  GPRS_RX_ptr_Out = GPRS_RX_ptr_Store&GPRS_RX_DATA_SIZE;
}
void gprsBufInit(void)
{
	//������ָ���ʼ��
	clearBufPtr();	
	//��ʱ��ָ���ʼ��
	clearGprsFlag();
//  gprsState = GPRS_TURN_ON;
}
void receivePrintf(char *buf,uint8_t *bufin,uint16_t index,uint16_t num)
{
	uint16_t rxi=0;//20210201
	printf("%s",buf);
	for(rxi=0;rxi<num;rxi++)
	{
		devStoreBuffer(&bufin[(index+rxi)&GPRS_RX_DATA_SIZE],1);
		if(bufin[(index+rxi)&GPRS_RX_DATA_SIZE]=='\r')
		{
			if(bufin[(index+rxi+1)&GPRS_RX_DATA_SIZE]=='\n')break;		
		}
	}
	printf("\r\n");
}

uint16_t gprsAckWait(char *buf,uint8_t len)
{
	__IO uint16_t status = UN_FIND_STATUS;
	__IO uint16_t gprsDelayCounter = 0;
	__IO uint16_t ptrOut=0;
	uint16_t rxi=0;
	
	ptrOut = GPRS_RX_ptr_Store;	
	gprsWaiteAckTimes = 0x00;	
	gprsAckFlag 			= COMM_ACK;	

	while((gprsAckFlag >= COMM_ACK)&&(gprsDelayCounter<3750))//1500
	{
		gprsDelayCounter++;	
		
		if(findStrRxGprs(buf,len) < FIND_ERROR)
		{
			gprsAckFlag = COMM_BACK;
			status 			=	COMM_BACK;
		//	delay_GprsMs(100);
		}
		else if(gprsDelayCounter>=3750)//1500
		{
			gprsAckFlag = COMM_ERR;	
			status 			=	COMM_TIMEOUT;	
			GPRS_RXCounter	= gprsRxcounterCal(ptrOut);
		#if GPRS_DEBUG > 0
			if(gprsAckprintf>0)printf("\r\n %s waitAck timeOut!%d\r\n",buf,GPRS_RXCounter);
		#endif				
		}
		else
		{
		 #if(GPRS_MULTI_LINK==0)
			 if(gprsAckState == GPRS_ACK_SEND)
			 {
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				rxi = findStrRxGprs("+IPCLOSE: ",10);
				if(rxi < FIND_ERROR)
				{
					clearBufPart(rxi,10);
			#if GPRS_DEBUG > 0
				if(gprsAckprintf>0)receivePrintf("\r\n +IPCLOSE: ",GPRS_RX_Buffer,rxi,10);
			#endif
					gprsAckFlag = NO_COMM;	return NO_COMM;
				}				
			#else
				rxi = findStrRxGprs("CLOSED",6);
				if(rxi < FIND_ERROR)
				{
					clearBufPart(rxi,6);
			#if GPRS_DEBUG > 0
				if(gprsAckprintf>0)receivePrintf("\r\n CLOSED: ",GPRS_RX_Buffer,rxi,10);
			#endif
					gprsAckFlag = NO_COMM;	return NO_COMM;
				}							
			#endif
			}
		#endif
			rxi = findStrRxGprs("ERROR",5);
			if(rxi < FIND_ERROR)
			{		
				gprsAckFlag = COMM_BACK;	return COMM_BACK;
			}
		}
		delay_GprsMs(20);//50
	}
	
	if(gprsAckFlag  == COMM_TIMEOUT)
	{
		GPRS_RXCounter = gprsRxcounterCal(ptrOut);
	#if GPRS_DEBUG > 0
		if(gprsAckprintf>0)printf("\r\n %s waitAck timeOut.%d\r\n",buf,GPRS_RXCounter);
	#endif	  
		status = COMM_TIMEOUT;
	}	
	if(gprsAckFlag  != COMM_BACK)
	{
	#if GPRS_DEBUG > 0
		if(gprsAckprintf>0)
		{
			for(rxi=0;rxi<GPRS_RXCounter;rxi++)
			{
				devStoreBuffer(&GPRS_RX_Buffer[(ptrOut+rxi)&GPRS_RX_DATA_SIZE],1);
			}
			printf("\r\n");
		}
	#endif	
	}
	return status;
}

static uint8_t sendAtCmd_AckOK(char *buf,uint8_t cmdstate)
{
	uint16_t status = UN_FIND_STATUS;
	clearBuf();
	gprsAckState	=	cmdstate;
	gprsSendBuffer((uint8_t *)buf,strlen(buf)); 
	status = gprsAckWait("OK",2);

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,2);
			status = COMM_OK;
		}
		else
		{
			status = COMM_ERR;		
		}
		clearBuf();
	}
	return status	;	  	 
}
static uint8_t sendAt(void)
{
	return sendAtCmd_AckOK("AT\r\n",GPRS_ACK_OK); 
}

static uint8_t gprsUartSpeed(void)
{
	return sendAtCmd_AckOK("AT+IPR=115200\r\n",GPRS_ACK_SET); 
}

uint8_t gprsEchoOff(void)
{
	return sendAtCmd_AckOK("ATE0\r\n",GPRS_ACK_SET); 
//	return sendAtCmd_AckOK("ATE0&W\r\n",GPRS_ACK_SET); 
}
#if(SIM7600_3GPP_SET>0)	//211025
static uint8_t Active3GPP(void)
{
	return sendAtCmd_AckOK("AT+CMCFG=\"ACTIVE\",\"ROW_Generic_3GPP\"\r\n",GPRS_ACK_SET);	
}
static uint8_t PLMNBAND(void)
{
	return sendAtCmd_AckOK("AT+CEXTPLMNBAND=1,1,,0x000001E000000095,\r\n",GPRS_ACK_SET);	
}
#endif
static uint8_t gprsUartInit(void)
{
	uint8_t uartStatus = 0xFF;
	for(gprsTryTimes=0;gprsTryTimes<GPRS_TIMES_ALLTRY;gprsTryTimes++)
	{
		gprsAckprintf = 0;
		uartStatus = sendAt();
		gprsAckprintf = 1;
		if(uartStatus == COMM_OK){break;}	
	}
	if(uartStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r at 0k \n\r");
	#endif	 
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r at error \n\r");
	#endif	 
		return 	uartStatus; 	
	}
	for(gprsTryTimes=0;gprsTryTimes<GPRS_TIMES_ALLTRY;gprsTryTimes++)
	{
		gprsAckprintf = 0;
		uartStatus = gprsEchoOff();
		gprsAckprintf = 1;
		if(uartStatus == COMM_OK){break;}	
	} 
	if(uartStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r EchoOff 0k \n\r");
	#endif  
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r EchoOff error \n\r");
	#endif	 
		return 	uartStatus; 	
	}
	uartStatus =  gprsUartSpeed(); 
	if(uartStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r speed 0k \n\r");
	#endif		
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r speed error \n\r");
	#endif	
		return 	uartStatus; 	
	}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)
	gprsSleepModeSwitch(GPRS_SLEEP_MODE_DIS);
#else
 #if(SIM7600_3GPP_SET>0) //211025 
	Active3GPP();
	PLMNBAND();
 #endif
#endif
	uartStatus = COMM_OK;

	return 	uartStatus;
}

static uint8_t simOk(void)
{
	uint16_t status = UN_FIND_STATUS;

	gprsAckState	=	GPRS_ACK_OK;	
	gprsSendBuffer((uint8_t *)"AT+CPIN?\r\n",10);	
	status = gprsAckWait("+CPIN: READY",12);

	if(status  == COMM_BACK)
	{	
		status = findStrRxGprs("+CPIN: READY",12);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,12);
			status = COMM_OK;
		}
		else
		{
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n +CPIN:",GPRS_RX_Buffer,GPRS_RX_ptr_Out,20);
		#endif	
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;	  	 
}
uint8_t networkmode(uint8_t mode)
{
	char cmdbuf[20];
	sprintf(cmdbuf,"AT+CNMP=%d\r\n",mode);
	
	return sendAtCmd_AckOK(cmdbuf,GPRS_ACK_RX)	;	  	 
}
uint8_t readnetworkmode(void)
{	
	return sendAtCmd_AckOK("AT+CNMP?\r\n",GPRS_ACK_OK)	;	  	 
}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)	
 uint8_t cregType(void)
{
	return sendAtCmd_AckOK("AT+CREG=2\r\n",GPRS_ACK_SET); 
}
#endif

static uint8_t ccidBuf[25];
static uint8_t imeiBuf[25];
static uint8_t cimiBuf[25];

void simCardInfInit(void)
{
	ccidBuf[0] = 0x00;
	imeiBuf[0] = 0x00;
	cimiBuf[0] = 0x00;
}
uint8_t netOperatorCheck(uint8_t *buf)
{//MCC 460 MNC �ƶ� 00/02/04/07/08/13 ��ͨ 01/06/09/10 ���� 03/05/11/12 
	uint8_t mncMode = NET_OPERATOR_NORMAL;
	if((buf[1]=='4')&&(buf[2]=='6')&&(buf[3]=='0'))
	{
		if(buf[4]=='0')
		{
			if((buf[5]=='0')||(buf[5]=='2')||(buf[5]=='4')||(buf[5]=='7')||(buf[5]=='8'))
			{
				mncMode = NET_OPERATOR_MOBILE;
			}
			else if((buf[5]=='1')||(buf[5]=='6')||(buf[5]=='9'))
			{
				mncMode = NET_OPERATOR_UNICOM;
			}
			else if((buf[5]=='3')||(buf[5]=='5'))
			{
				mncMode = NET_OPERATOR_TELECOM;
			}
		}
		else if(buf[4]=='1')
		{
			if(buf[5]=='3')
			{
				mncMode = NET_OPERATOR_MOBILE;
			}
			else if(buf[5]=='0')
			{
				mncMode = NET_OPERATOR_UNICOM;
			}
			else if((buf[5]=='1')||(buf[5]=='2'))
			{
				mncMode = NET_OPERATOR_TELECOM;
			}
			
		}
	}
#if GPRS_DEBUG > 1									
	printf(" mncMode:%d\r\n",mncMode);
#endif
	return mncMode;
}
static void gprsBufHeadInit(void)
{
	uint8_t i = 0;
	GPRS_RX_ptr_Store = 0;
	clearBuf();
	for(i = 0;i <100;i++)
	{
		GPRS_RX_Buffer[i] = 0xFF;
	}
}
static uint8_t getCcid(uint8_t *buf)
{
	uint8_t geti = 0x00;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)
	uint8_t getj = 0x00;
#endif
	uint16_t status = UN_FIND_STATUS;
	uint16_t location = 0;
	gprsFlag =  COMM_SEND;
	gprsBufHeadInit();	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+CICCID\r\n",11);
#else
	gprsSendBuffer((uint8_t *)"AT+CCID\r\n",9);
#endif

	gprsAckState	=	GPRS_ACK_SET;	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = gprsAckWait("+ICCID: ",8);	
#else
	status = gprsAckWait("OK",2);
#endif

	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+ICCID: ",8);	
		if((status < FIND_ERROR))
		{	
			clearBufPart(status,8);
			location = status;		
			//printf("location start address[%d]\r\n",location); 		   
			for(geti=0;geti<21;geti++)
			{
				if((GPRS_RX_Buffer[(location+geti+1)&GPRS_RX_DATA_SIZE]== 0x0A)&&(GPRS_RX_Buffer[(location+geti)&GPRS_RX_DATA_SIZE]== 0x0D))
				{
					geti ++; //200731
					buf[0] = geti;							
					//printf("location end address[%d],length %d\r\n",location,buf[0]); 		
					status =  COMM_OK;    
					break;
				}
				else
				{
					buf[geti+1]= GPRS_RX_Buffer[(location+geti)&GPRS_RX_DATA_SIZE];
				}
			}					 
		}
	#else
		status = findStrRxGprs("OK",2);
		if((status < FIND_ERROR)&&(status >10))
		{
			clearBufPart(status,2);
			location = status-6;			
			// printf("location[%d]\r\n",location); 
			for(geti = location;geti >=1;geti --)
			{
				if((GPRS_RX_Buffer[geti]== 0x0A)&&(GPRS_RX_Buffer[geti-1]== 0x0D))
				{
					geti ++; 
					//printf("location geti[%d]\r\n",geti);  
					break;
				}
			}
			if(geti != 0x00)
			{  
				getj = 0x01;
				for(;geti < location;geti ++)
				{
					buf[getj++]= GPRS_RX_Buffer[geti];	
				}
				buf[0] = getj;
				status = COMM_OK; 
			} 
			else
			{
				status = NO_COMM; 
			}                          
		}
	#endif
		clearBuf();
	}        
	return status;
}

static uint8_t getCimi(uint8_t *buf)
{
	uint8_t geti = 0x00;
	uint8_t getj = 0x00;
	uint16_t status = UN_FIND_STATUS;
	uint16_t location = 0;
	gprsFlag =  COMM_SEND;
	gprsAckState	=	GPRS_ACK_SET;	
	gprsBufHeadInit();	
	gprsSendBuffer((uint8_t *)"AT+CIMI\r\n",9);	
	status = gprsAckWait("OK",2);	

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);

		if((status < FIND_ERROR)&&(status >10))
		{
			clearBufPart(status,2);
		//ȡ��
			location = status-6;
			//printf("location[%d]\r\n",location); 
			for(geti = location;geti >=1;geti --)
			{
				if((GPRS_RX_Buffer[geti]== 0x0A)&&(GPRS_RX_Buffer[geti-1]== 0x0D))
				{
					 geti ++; 
					 //printf("location geti[%d]\r\n",geti);  
					 break;
				}
			}
			if(geti != 0x00)
			{  
				getj = 0x01;				 
				for(;geti < location;geti ++)
				{
					buf[getj++]= GPRS_RX_Buffer[geti];	
				}
				buf[0] = getj;
				status = COMM_OK; 
			} 
			else
			{
				status = NO_COMM; 
			}                          
		}
		clearBuf();	
	}     
	return status;
}

static uint8_t getImei(uint8_t *buf)
{
	uint8_t geti = 0x00;
	uint8_t getj = 0x00;
	//char buf[15];
	uint16_t status = UN_FIND_STATUS;
	uint16_t location = 0;
	gprsFlag =  COMM_SEND;
	gprsBufHeadInit();
	gprsAckState	=	GPRS_ACK_SET;	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+CGSN\r\n",9);	
#else
	gprsSendBuffer((uint8_t *)"AT+GSN\r\n",8);	
#endif	
	status = gprsAckWait("OK",2);	

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);
		if((status < FIND_ERROR)&&(status >10))
		{
			clearBufPart(status,2);
			//ȡ��
			location = status-6;
			//printf("location[%d]\r\n",location); 
			for(geti = location;geti >=1;geti --)
			{
				if((GPRS_RX_Buffer[geti]== 0x0A)&&(GPRS_RX_Buffer[geti-1]== 0x0D))
				{
					 geti ++; 
					 //printf("location geti[%d]\r\n",geti);  
					 break;
				}
			}
			if(geti != 0x00)
			{  
				getj = 0x01;				 
				for(;geti < location;geti ++)
				{
					buf[getj++]= GPRS_RX_Buffer[geti];	
				}
				buf[0] = getj;
				status = COMM_OK; 
			} 
			else
			{
				status = NO_COMM; 
			}                          
		}
		clearBuf();	
	} 
	return status;
}

const uint8_t hexBuf[]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};

static uint8_t getLocation(void)
{	
	uint16_t status = UN_FIND_STATUS;		
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)
	uint8_t geti = 0x00;
	char buf[15];
	uint16_t location = 0;
#endif
	gprsFlag =  COMM_SEND;
	gprsAckState	=	GPRS_ACK_SET;	
	gprsSendBuffer((uint8_t *)"AT+CREG?\r\n",10);		
	status = gprsAckWait("+CREG:",6);	

	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+CREG: 2,1",10);
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 2,5",10);	
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 0,1",10);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 0,5",10);	
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 1,1",10);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 1,5",10);	
		}
	#else
		status = findStrRxGprs("+CREG: 2,1,\"",12);
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CREG: 2,5,\"",12);
		}
	#endif
		if(status < FIND_ERROR)
		{
			clearBufPart(status,10);
		#if GPRS_DEBUG > 0
			printf("\n\r network gsm registration ok \n\r");
		#endif
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)			
			location = status;
			//ȡ��
			for(geti = 0x01;geti < 5;geti ++)
			{
				if(GPRS_RX_Buffer[location]!=0x22)//0x22 '"'
				{
					buf[geti]= GPRS_RX_Buffer[location];
				}
				else
				{
					break;
				}
				location ++; 	
				location &= GPRS_RX_DATA_SIZE;
			}
			buf[0]		= geti;
			location += 3;
			location &= GPRS_RX_DATA_SIZE;
			for(geti 	= 0x01;geti < 5;geti ++)
			{
				if(GPRS_RX_Buffer[location]!=0x22)
				{
					buf[geti+6]= GPRS_RX_Buffer[location];
				}
				else
				{
					break;
				}
				location ++; 	
				location &= GPRS_RX_DATA_SIZE;				
			}
			buf[6]= geti;

			lacValue 		=	(uint16_t)hex2int((uint8_t *)buf[1]);
			cellIdValue =	(uint16_t)hex2int((uint8_t *)buf[7]);
		#if GPRS_DEBUG > 0
			printf("\n\r LAC %d\n\r: ",lacValue);
			devStoreBuffer((uint8_t *)buf[1],(buf[0]-1));
			printf("\r\n");
			printf("\n\r cell id :%d\n\r ",cellIdValue);
			devStoreBuffer((uint8_t *)buf[7],(buf[6]-1));
			printf("\n\r");
		#endif
#endif
			status = COMM_OK;
		}
		else
		{
		#if GPRS_DEBUG > 1			
			printf("\n\r network gsm registration error \n\r");
		#endif		
			status = findStrRxGprs("+CREG:",6);
			clearBufPart(status,6);
		#if GPRS_DEBUG > 0
			if((gprsTryTimes%5)==0)
			{				
				receivePrintf("\r\n +CREG:",GPRS_RX_Buffer,status,10);
			}
		#endif	
			status =  COMM_ERR;	
		}
		clearBuf();
	}
	return status	;
}
static uint8_t netWorkGprs(void )
{
	uint16_t status = UN_FIND_STATUS;

	gprsFlag =  COMM_SEND;	
	gprsSendBuffer((uint8_t *)"AT+CGREG?\r\n",11);	

	gprsAckState	=	GPRS_ACK_SET;	
	status = gprsAckWait("+CGREG:",7);	

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("+CGREG: 0,1",11);
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CGREG: 0,5",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CGREG: 1,1",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CGREG: 1,5",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CGREG: 2,1",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CGREG: 2,5",11);
		}
		if(status < FIND_ERROR)
		{
			clearBufPart(status,11);
			status = COMM_OK;
		}
		else
		{
			status = findStrRxGprs("+CGREG:",7);
			clearBufPart(status,7);
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
static uint8_t netWorkEPS(void )
{
	uint16_t status = UN_FIND_STATUS;
	
	gprsSendBuffer((uint8_t *)"AT+CEREG?\r\n",11);	

	gprsAckState	=	GPRS_ACK_SET;	
	status = gprsAckWait("+CEREG:",7);		
	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("+CEREG: 0,1",11);
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CEREG: 0,5",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CEREG: 1,1",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CEREG: 1,5",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CEREG: 2,1",11);
		}
		if(status >=FIND_ERROR )
		{
			status = findStrRxGprs("+CEREG: 2,5",11);
		}
		if(status < FIND_ERROR)
		{
			clearBufPart(status,11);
			status = COMM_OK;
		}
		else
		{
			status = findStrRxGprs("+CEREG:",7);
			clearBufPart(status,7);
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}

#endif
static uint8_t getRssi(void)
{
	char buf[5];
	uint16_t status = UN_FIND_STATUS;
	
	uint8_t rssi = 0x00;
clearBuf();	
	gprsFlag =  COMM_SEND;	
	gprsSendBuffer((uint8_t *)"AT+CSQ\r\n",8);	

	gprsAckState	=	GPRS_ACK_SET;	
	status = gprsAckWait("+CSQ: ",6);	

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("+CSQ: ",6);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,6);
			for(rssi = 0x00;rssi <5;rssi++)
			{
				buf[rssi] = 0x00;
			}
			//ȡ��
			for(rssi = 0x00;rssi < 3;rssi ++)
			{
				if(GPRS_RX_Buffer[(status+rssi)&GPRS_RX_DATA_SIZE] != 0x2c)//0x20 ','
				{
					buf[rssi]= GPRS_RX_Buffer[(status+rssi)&GPRS_RX_DATA_SIZE]; 
				}
				else
				{	 
					buf[rssi] = '\0';
					break;
				}
			}
			rssiValue = atoi(buf);
		#if GPRS_DEBUG > 1
			printf("\n\r rssi value :%d\n\r ",rssiValue);
		#endif
			status = COMM_OK;
		}
		else
		{
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}

static uint8_t setIpHead(void)
{
	return sendAtCmd_AckOK("AT+CIPHEAD=1\r\n",GPRS_ACK_SET); //Ĭ��0�������ݲ���+IPD 1�������ݴ�+IPD
}

static uint8_t linkAckType(void)
{
	sendAtCmd_AckOK("AT+CIPSRIP=0\r\n",GPRS_ACK_SET); //Ĭ��0 �������ݲ�����IP��ַ��1 ��������ʱ����IP��ַ
	sendAtCmd_AckOK("AT+CIPSPRT=1\r\n",GPRS_ACK_SET); //Ĭ��1_'>'_201114 0_������'>'
	return COMM_OK;
}

static uint8_t simCardRead(void)
{
	uint8_t checkStatus;

	checkStatus = getCcid(ccidBuf);
	if(checkStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r CCID READ 0k %s ",ccidBuf+1);
	#endif    
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r CCID READ error  ");
	#endif
		return checkStatus;    
	}    
	checkStatus = getCimi(cimiBuf);;
	if(checkStatus == COMM_OK)
	{
		netOperator = netOperatorCheck(cimiBuf);
	#if GPRS_DEBUG > 0
		printf("\n\r CIMI READ 0k %s ",cimiBuf+1);
	#endif    
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r CIMI READ error ");
	#endif
	return checkStatus;    
	}   
	checkStatus = getImei(imeiBuf);
	if(checkStatus == COMM_OK)
	{		
	#if GPRS_DEBUG > 0
		printf("\n\r IMEI READ 0k %s\n\r",imeiBuf+1);
	#endif    
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r IMEI READ error \n\r");
	#endif
		return checkStatus;    
	}   
	return checkStatus;  
}

uint8_t GPRSAttach(void )
{
	uint16_t status = UN_FIND_STATUS;
clearBuf();
	gprsAckState	=	GPRS_ACK_RX;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+NETOPEN\r\n",12);	
#else
	gprsSendBuffer((uint8_t *)"AT+CGATT?\r\n",11);	
#endif
	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = gprsAckWait("+NETOPEN: ",10);//+NETOPEN: //+IP ERROR: Network is already opened		
#else
	status = gprsAckWait("+CGATT:",7);
#endif

	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+NETOPEN: ",10);//("+ATNETOPEN: 0",13)
		if(status < FIND_ERROR)
		{
			clearBufPart(status,10);//���ٲ����ֽ�
			if(GPRS_RX_Buffer[status] == '0'){status = COMM_OK;}
			else
			{
			#if GPRS_DEBUG > 0
				receivePrintf("\r\n +NETOPEN ERR: ",GPRS_RX_Buffer,status,10);
			#endif
				status =  COMM_ERR;
			}
		}
		else
		{		
			status = findStrRxGprs("+IP ERROR: Network is already opened",36);
			if(status < FIND_ERROR) 
			{
				clearBufPart(status,36);//���ٲ����ֽ�
				status = COMM_OK;
			}
			else		
			{
				status =  COMM_ERR;	
			}		
		}
	#else
		status = findStrRxGprs("+CGATT: 1",9);		
		if(status < FIND_ERROR)
		{
			clearBufPart(status,9);//���ٲ����ֽ�
			status = COMM_OK;
		}
		else
		{		
			status =  COMM_ERR;
		}
	#endif	
		clearBuf();
	}
	return status	;
}
void autoApnRead(char *apnBuf)
{	
#if(APN_SELECT_AUTO_CTR>0)
	if(netOperator == NET_OPERATOR_MOBILE)
	{
		sprintf(apnBuf,"%s","CMNET\r\n");//CMNET CMIOT
	}
	else if(netOperator == NET_OPERATOR_UNICOM)
	{
		sprintf(apnBuf,"%s","3GNET\r\n");//3GNET 
	}
	else if(netOperator == NET_OPERATOR_TELECOM)
	{
		sprintf(apnBuf,"%s","CTNET\r\n");//CTNET
	}
	else
#endif
	{
//		if((devApnSetStatus != APN_SAVE_STATUS_INIT)&&(apnBufSet[0]!=0)&&(apnBufSet[1]!=0)){apnBuf = apnBufSet; }else
		{sprintf(apnBuf,"%s","CMNET\r\n");}//apnBuf = apnBufInit;
	}
}
static uint8_t gprsApnConfig(char *apnBuf,char *apnIn)
{
	uint8_t dataLenth = 1;
	uint8_t datat			= 0;
	uint8_t i					=0;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
	datat =  sprintf(apnBuf+dataLenth,"%s","AT+CGDCONT=1,\"IP\",\"");//20200106
#else	
	datat =  sprintf(apnBuf+dataLenth,"%s","AT+CSTT=\"");
#endif
	dataLenth += datat;
	
	datat = 0;
	for(i=0;i<APN_LEN_MAX+1;i++)//16 31_211215
	{
		if((apnIn[i]=='\r')&&(apnIn[i+1]=='\n')){break;}
		apnBuf[i+dataLenth] = apnIn[i];
		datat ++;
	}
	dataLenth +=	datat;
	datat	=  sprintf(apnBuf+dataLenth,"%s","\"\r\n");
	dataLenth +=	datat;
	dataLenth	-=	1;
	apnBuf[0]	 =  dataLenth;	
	return dataLenth;
}
static uint8_t gprsApnSet(uint8_t apnType)
{	
	uint16_t status = UN_FIND_STATUS;
	char apnBuf[50];
	char *apnIn;

 uint8_t apnFlag = 0;//211215
	
#if(DEV_APN_SET_CTR>0) //230508	
	if(apnType == APN_TYPE_AUTO)//230307
	{
		if((devApnSetStatus == APN_SAVE_STATUS_DONE)&&(apnBufSet[0]!=0)&&(apnBufSet[1]!=0))
		{
			apnIn = apnBufSet;apnFlag = 1;
		}else 

		if((devPar.apnBuf[0]!=0)&&(devPar.apnBuf[1] != 0)) //(devPar.stationMode != FUNCITONG_TURN_ON)&& 230508
		{
			apnIn = devPar.apnBuf;apnFlag = 1;
			if((devPar.apnBuf[1]=='A')&&(devPar.apnBuf[2]=='T')&&(devPar.apnBuf[3]=='+')&&(devPar.apnBuf[4]=='C'))apnFlag = 0;
		}		
	}
	else if(apnType == APN_TYPE_NETSET)
	{
		if((devPar.apnBuf[0]!=0)&&(devPar.apnBuf[1]!=0))
		{
			apnIn = devPar.apnBuf;apnFlag = 1;
			if((devPar.apnBuf[1]=='A')&&(devPar.apnBuf[2]=='T')&&(devPar.apnBuf[3]=='+')&&(devPar.apnBuf[4]=='C'))apnFlag = 0;
		}
		else if((devApnSetStatus != APN_SAVE_STATUS_INIT)&&(apnBufSet[0]!=0)&&(apnBufSet[1]!=0)){apnIn = apnBufSet;apnFlag = 1;}
	}	

	else //if(apnType == APN_TYPE_DEVSET)
	{
		if((devApnSetStatus != APN_SAVE_STATUS_INIT)&&(apnBufSet[0]!=0)&&(apnBufSet[1]!=0)){apnIn = apnBufSet;apnFlag = 1;}
	}
#endif	 	
	if(apnFlag == 0)
	{
		autoApnRead(apnBufAuto);
		apnIn = apnBufAuto;
	}	

	clearBuf();
	gprsApnConfig(apnBuf,apnIn);//apnBufSet
	gprsFlag =  COMM_SEND;
	gprsAckState	=	GPRS_ACK_SET;		
	gprsSendBuffer((uint8_t *)(apnBuf+1),apnBuf[0]);	
	
	status = gprsAckWait("OK",2);

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);
		if(status < FIND_ERROR)
		{	
			clearBufPart(status,2);//���ٲ����ֽ�
			status = COMM_OK;
		#if GPRS_DEBUG > 0
			printf("\r\napn:%s",apnBuf+1);
		#endif
		}
		else
		{
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}
//static char apnBuf[50];
 uint8_t gprsApnConfig1(char *apnBuf,char *apnIn)
{
	uint8_t dataLenth = 1;
	uint8_t datat			= 0;
	uint8_t i					=0;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
	datat =  sprintf(apnBuf+dataLenth,"%s","AT+CGDCONT=1,\"IP\",\"");//20200106
#else 
	datat =  sprintf(apnBuf+dataLenth,"%s","AT+CSTT=\"");
#endif
	dataLenth += datat;
#if(APN_SELECT_AUTO_CTR>0)	
	if(netOperator == NET_OPERATOR_MOBILE)
	{
		datat =  sprintf(apnBuf+dataLenth,"%s","CMNET");//CMNET
	}
	else if(netOperator == NET_OPERATOR_UNICOM)
	{
		datat =  sprintf(apnBuf+dataLenth,"%s","3GNET");//3GNET 
	}
	else if(netOperator == NET_OPERATOR_TELECOM)
	{
		datat =  sprintf(apnBuf+dataLenth,"%s","CTNET");//CTNET
	}
	else
#endif		
	{
		datat = 0;	
		for(i=0;i<16;i++)
		{
			if((apnIn[i]=='\r')&&(apnIn[i+1]=='\n')){break;}
			apnBuf[i+dataLenth] = apnIn[i];
			datat ++;
		}		
	}
	dataLenth +=	datat;
	datat	=  sprintf(apnBuf+dataLenth,"%s","\"\r\n");
	dataLenth +=	datat;
	dataLenth	-=	1;
	apnBuf[0]	 =  dataLenth;	
	return dataLenth;
}
 uint8_t gprsApnSet1(void)
{	
	uint16_t status = UN_FIND_STATUS;
	char apnBuf[50];
clearBuf();
	gprsApnConfig(apnBuf,apnBufSet);
	gprsFlag =  COMM_SEND;	
	gprsAckState	=	GPRS_ACK_SET;	
	gprsSendBuffer((uint8_t *)(apnBuf+1),apnBuf[0]);	
	
	status = gprsAckWait("OK",2);

	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);
		if(status < FIND_ERROR)
		{	
			clearBufPart(status,2);//���ٲ����ֽ�
			status = COMM_OK;
		#if GPRS_DEBUG > 0
			printf("\r\n apn:%s",apnBuf+1);
		#endif
		}
		else
		{
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}

static uint8_t getIpStatus(void )
{
	uint16_t status = UN_FIND_STATUS;
clearBuf();
	gprsFlag =  COMM_SEND;
	gprsAckState	= GPRS_ACK_RX;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+IPADDR\r\n",11);	
#else
	gprsSendBuffer((uint8_t *)"AT+CIFSR\r\n",10);	
#endif

#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = gprsAckWait("+IPADDR:",8);
#else
	status = gprsAckWait(".",1);	//("ERROR",5)
#endif
	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+IPADDR:",8);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,8);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n +IPADDR:",GPRS_RX_Buffer,status,20);
		#endif
			status =  COMM_OK;
		}
		else
		{
			status = findStrRxGprs("+IP ERROR: ",11);
			if(status < FIND_ERROR)
			{
				clearBufPart(status,11);//���ٲ����ֽ�
			#if GPRS_DEBUG > 0
				receivePrintf("\r\n +IP ERROR: ",GPRS_RX_Buffer,status,20);
			#endif
			}
			status = COMM_ERR;		
		}
	#else
		status = findStrRxGprs("ERROR",5);	
		if(status < FIND_ERROR)
		{
			clearBufPart(status,5);//���ٲ����ֽ�
			status =  COMM_ERR;
		}
		else
		{
			status = COMM_OK;		
		}
	#endif
		clearBuf();
	}
	return status	;
}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
static uint8_t gprsModeSet(uint8_t mode)
{
	if(mode ==0){return sendAtCmd_AckOK("AT+CIPMODE=0\r\n",GPRS_ACK_SET);}
	else{return sendAtCmd_AckOK("AT+CIPMODE=1\r\n",GPRS_ACK_SET);}	
}
static uint8_t getOperator(void)
{	
	uint16_t status = UN_FIND_STATUS;
	
	gprsAckState	=	GPRS_ACK_SET;	
	gprsSendBuffer((uint8_t *)"AT+COPS?\r\n",10);	

	
	status = gprsAckWait("+COPS:",6);
	if(status  == COMM_BACK)
	{	
		status = findStrRxGprs("+COPS:",6);
			
		if(status < FIND_ERROR)
		{
			clearBufPart(status,6);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n COPS:",GPRS_RX_Buffer,status,50);
		#endif			
			status = COMM_OK;
		}
		else
		{	
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}
static uint8_t getSysInformation(void)
{	
	uint16_t status = UN_FIND_STATUS;
	uint8_t rxi = 0x00;
	netInformation.netOperator = NET_OPERATOR_NORMAL;
	gprsAckState	=	GPRS_ACK_SET;
	gprsSendBuffer((uint8_t *)"AT+CPSI?\r\n",10);	
		
	status = gprsAckWait("+CPSI: ",7);
	if(status  == COMM_BACK)
	{//LTE,Online,460-00,0x10A7,227406025,383,EUTRAN-BAND41,40936,5,5,-126,-1184,-870,9
		status = findStrRxGprs("+CPSI: ",7);			
		if(status < FIND_ERROR)//MCC 460 MNC �ƶ� 00/02/07 ��ͨ 01/06 ���� 03/05 
		{
			clearBufPart(status,7);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n CPSI: ",GPRS_RX_Buffer,status,100);
		#endif
			for(rxi=0;rxi<8;rxi++)//systemMode
			{
				if(GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE]==',')
				{
					netInformation.systemMode[rxi+1]= '\0';
					netInformation.systemMode[0]		= rxi;
					status += (rxi+1);
				#if GPRS_DEBUG > 0
					receivePrintf(" systemMode:",netInformation.systemMode,1,rxi);
				#endif
					rxi=250;				
					break;
				}
				else
				{
					netInformation.systemMode[rxi+1]= GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE];
				}
			}
			if(rxi==250)
			{
				for(rxi=0;rxi<18;rxi++)//operationMode
				{
					if(GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE]==',')
					{
						netInformation.operationMode[rxi+1]= '\0';
						netInformation.operationMode[0]		= rxi;
						status += (rxi+1);
					#if GPRS_DEBUG > 0
						receivePrintf(" operationMode:",netInformation.operationMode,1,rxi);
					#endif
						rxi=250;
						break;
					}
					else
					{
						netInformation.operationMode[rxi+1]= GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE];
					}
				}
			}
			if(rxi==250)
			{
				for(rxi=0;rxi<8;rxi++)//MCC 460 MNC �ƶ� 00/02/04/07/08/13 ��ͨ 01/06/09/10 ���� 03/05/11/12 
				{
					if(GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE]==',')
					{
						netInformation.mcc_mnc[rxi+1]= '\0';
						netInformation.mcc_mnc[0]		= rxi;
						status += (rxi+1);					
						if((netInformation.mcc_mnc[1]=='4')&&(netInformation.mcc_mnc[2]=='6')&&(netInformation.mcc_mnc[3]=='0'))
						{
							if((netInformation.mcc_mnc[4]=='-')&&(netInformation.mcc_mnc[5]=='0'))
							{
								if((netInformation.mcc_mnc[6]=='0')||(netInformation.mcc_mnc[6]=='2')||(netInformation.mcc_mnc[6]=='4')||(netInformation.mcc_mnc[6]=='7')||(netInformation.mcc_mnc[6]=='8'))
								{
									netInformation.netOperator = NET_OPERATOR_MOBILE;
								}
								else if((netInformation.mcc_mnc[6]=='1')||(netInformation.mcc_mnc[6]=='6')||(netInformation.mcc_mnc[6]=='9'))
								{
									netInformation.netOperator = NET_OPERATOR_UNICOM;
								}
								else if((netInformation.mcc_mnc[6]=='3')||(netInformation.mcc_mnc[6]=='5'))
								{
									netInformation.netOperator = NET_OPERATOR_TELECOM;
								}
							}
							else if((netInformation.mcc_mnc[4]=='-')&&(netInformation.mcc_mnc[5]=='1'))
							{
								if(netInformation.mcc_mnc[6]=='3')
								{
									netInformation.netOperator = NET_OPERATOR_MOBILE;
								}
								else if(netInformation.mcc_mnc[6]=='0')
								{
									netInformation.netOperator = NET_OPERATOR_UNICOM;
								}
								else if((netInformation.mcc_mnc[6]=='1')||(netInformation.mcc_mnc[6]=='2'))
								{
									netInformation.netOperator = NET_OPERATOR_TELECOM;
								}
							}
						}
					#if GPRS_DEBUG > 0
						receivePrintf(" mcc_mnc:",netInformation.mcc_mnc,1,rxi);
						printf(" operator:%d\r\n",netInformation.netOperator);
					#endif
						rxi=250;
						break;
					}
					else
					{
						netInformation.mcc_mnc[rxi+1]= GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE];
					}
				}				
			}
			status = COMM_OK;
		}
		else
		{	
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}

static uint8_t getNetWorkMode(void)
{	
	uint16_t status = UN_FIND_STATUS;
//	uint8_t geti = 0x00;

	gprsAckState	=	GPRS_ACK_SET;	
	gprsSendBuffer((uint8_t *)"AT+CNSMOD?\r\n",12);	
	
	status = gprsAckWait("+CNSMOD:",8);
	if(status  == COMM_BACK)
	{	
		status = findStrRxGprs("+CNSMOD:",8);
			
		if(status < FIND_ERROR)
		{		
			clearBufPart(status,8);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n CNSMOD:",GPRS_RX_Buffer,status,10);
		#endif
			status = COMM_OK;
		}
		else
		{	
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;
}
#if(GPRS_PING_CTR>0)
static uint8_t gprsPing(uint8_t *buf,uint8_t len)
{	
	uint16_t status = UN_FIND_STATUS;
	uint8_t geti = 0x00;
	char	comBuf[100];
	uint8_t comLen=0;
	
	comLen	= sprintf(comBuf, "%s","AT+CPING=\"");
	for(geti = 0;geti<len;geti++)
	{
		comBuf[comLen++]=buf[geti];
	}
	geti	  =	comLen;
	comLen += sprintf(comBuf+geti,"%s","\",1,4,32,1000,10000,255\r\n");

	gprsAckState	=	GPRS_ACK_RX;
	gprsSendBuffer((uint8_t *)comBuf,comLen);	
for(comLen = 0;comLen<5;comLen++)
{
	status = gprsAckWait("+CPING: ",8);
	
	if(status  == COMM_BACK)
	{	
		status = findStrRxGprs("+CPING: ",8);
			
		if(status < FIND_ERROR)
		{
			clearBufPart(status,8);//���ٲ����ֽ�
			clearPtrOut(status);//20191225
		#if GPRS_DEBUG > 0
			//if(comLen>=3)
			{
				printf("\r\n PING:");
				for(geti=0;geti<len;geti++)devStoreBuffer(buf+geti,1);
				printf(",result:");
				if(GPRS_RX_Buffer[status]=='2')
				{
					printf("request timeout");
				}
				else
				{
					receivePrintf(" ",GPRS_RX_Buffer,status,100);
				}
				printf("\r\n");
			}
		#endif		
			status = COMM_OK;
		}
		else
		{	
			status =  COMM_ERR;		
		}
	}
}
	return status	;
}
#endif
#endif
uint8_t setReceiveDataMode(void)
{
#if(GPRS_MULTI_LINK>0)
	return sendAtCmd_AckOK("AT+CIPRXGET=1\r\n",GPRS_ACK_SET);//
#else
	return sendAtCmd_AckOK("AT+CIPRXGET=0\r\n",GPRS_ACK_SET);//
#endif
}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
static uint8_t setPDPNumber(void )
{
	return sendAtCmd_AckOK("AT+CSOCKSETPN=1\r\n",GPRS_ACK_SET); 
}
#else
static uint8_t bringUpWireless(void )
{
	return sendAtCmd_AckOK("AT+CIICR\r\n",GPRS_ACK_SET); 
}
#endif
static uint8_t gprsCheck(void)
{
	uint8_t checkStatus=0;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	__IO 	uint8_t	netMode = 2;
#endif
	gprsAckprintf = 0;	
	if(serverConfig.simCardReadFlag == FUNCITONG_TURN_ON)//210623
	{
		for(gprsTryTimes=0;gprsTryTimes<GPRS_RETRY_TIMES_SIMOK;gprsTryTimes++)
		{		
			checkStatus 	= simOk();	
			if(checkStatus == COMM_OK)
			{
			#if GPRS_DEBUG > 0
				printf("\n\r check ok,try:%d,powerTime:%d\n\r",gprsTryTimes,gprsPowerOnTime);
			#endif	
				break;
			}
			else{delay_GprsMs(500);}
		}
		
		if(checkStatus != COMM_OK)
		{
			simCardErrorCounter ++;
		#if GPRS_DEBUG > 0
			printf("\n\r check error %d,powerTime:%d\n\r",simCardErrorCounter,gprsPowerOnTime);
		#endif		
			gprsAckprintf = 1;
			return checkStatus;
		}
//		devRunResult.sim	= 1;
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	

//		readnetworkmode();serverConfig.networkMode
		switch(serverConfig.networkMode)
		{
			case 0x32:netMode = 13;break;
			case 0x33:netMode = 48;break;
			case 0x34:netMode = 38;break;
			default :netMode 	= 2; break;
		}
		networkmode(netMode);//2 �Զ� 13ǿ��gsm 38ǿ��LTE 48ǿ��2G+3G

		simCardReadTask(&serverConfig);
		gprsApnSet(APN_TYPE_AUTO);
		setPDPNumber();
	#endif
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)	
		checkStatus = cregType();
		if(checkStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r creg set ok \n\r");
		#endif
		}
		else
		{
		#if GPRS_DEBUG > 0
			printf("\n\r creg set error \n\r");
		#endif 
		//	return checkStatus; 
		}
		delay_GprsS(0,30);
	#endif		
	}	

	rssiValue	= 0;
	for(gprsTryTimes=0;gprsTryTimes<GPRS_RETRY_TIMES_NORMAL;gprsTryTimes++)
	{		
		checkStatus =  getRssi();		
		if(rssiValue != 0)
		{
		#if GPRS_DEBUG > 0
			if((rssiValue >10)&&(rssiValue <32))
			printf("\n\r get rssi %d,try:%d,powerTime:%d\n\r",rssiValue,gprsTryTimes,gprsPowerOnTime);
		#endif	
			break;
		}
		else{delay_GprsMs(500);}
	}
	if(checkStatus != COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r get rssi error powerTime:%d\n\r",gprsPowerOnTime);
	#endif
	}
	
//	delay_GprsS(0,30);
	for(gprsTryTimes=0;gprsTryTimes<GPRS_RETRY_TIMES_NORMAL;gprsTryTimes++)
	{
		checkStatus = getLocation();	
		if(checkStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r network gsm ok try:%d,powerTime:%d\n\r",gprsTryTimes,gprsPowerOnTime);
		#endif
			break;
		}
		else{delay_GprsMs(500);}
	}
	if(checkStatus != COMM_OK)
	{
		for(gprsTryTimes=0;gprsTryTimes<5;gprsTryTimes++)
		{
			checkStatus = netWorkGprs();
			if(checkStatus == COMM_OK)
			{
			#if GPRS_DEBUG > 0
				printf("\n\r network gsm ok2 try:%d,powerTime:%d\n\r",gprsTryTimes,gprsPowerOnTime);
			#endif
				break;
			}
			else{delay_GprsMs(500);}
		}	
		if(checkStatus != COMM_OK)
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			checkStatus = netWorkEPS();
			if(checkStatus == COMM_OK)
			{
			#if GPRS_DEBUG > 0
				printf("\n\r network gsm ok3 try:%d,powerTime:%d\n\r",gprsTryTimes,gprsPowerOnTime);
			#endif
			}
			else
		#endif
			{		
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)			
				getSysInformation();				
			#endif 
				netGsmErrorCounter ++;
			#if GPRS_DEBUG > 0
				printf("\n\r network gsm error %d,powerTime:%d\n\r",netGsmErrorCounter,gprsPowerOnTime);
			#endif 
				gprsAckprintf = 1;
				return checkStatus;	
			}
		}			
	}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	getNetWorkMode();	
	getSysInformation();	
	getOperator();
#endif 
	setIpHead();	
	linkAckType();
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
	for(gprsTryTimes=0;gprsTryTimes<GPRS_TIMES_ALLTRY;gprsTryTimes++)
	{		
		if(gprsModeSet(0) == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r mode set ok\n\r");
		#endif
			break;		
		}
		else{delay_GprsMs(500);}
	}
#endif
	gprsAckprintf = 1;
	return checkStatus;
}

uint8_t gprsSet(lsrLog *serverLog)
{
	uint8_t setStatus = 0;
 	uint8_t seti=0;

	gprsAckprintf = 0;
	for(seti=0;seti<GPRS_TIMES_ALLTRY;seti++)
	{
		setStatus =	netWorkGprs();
		if(setStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r network gprs registration ok %d\n\r",seti);
		#endif	
			break;
		}
		else
		{
			delay_GprsMs(500);
		}
	}
	if(setStatus != COMM_OK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
		setStatus =	netWorkEPS();
		if(setStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r network EPS registration ok \n\r");
		#endif	
		}
		else
	#endif
		{
			serverLog->gprsRegErrorCounter ++;
		#if GPRS_DEBUG > 0		
			printf("\n\r network gprs registration error :%d,%d\n\r",serverLog->gprsRegErrorCounter,seti);
		#endif	   
		//	return 	setStatus;
		}
	}	
	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
//	setStatus = gprsApnSet();
//	if(setStatus == COMM_OK)
//	{
//	#if GPRS_DEBUG > 0
//		printf("\n\r gprs apn set ok %d\n\r",simCardType);
//	#endif	
//	}
//	else
//	{
//		serverLog->gprsApnSetErrorCounter++;
//	#if GPRS_DEBUG > 0
//		printf("\n\r gprs apn set error :%d\n\r",serverLog->gprsApnSetErrorCounter);
//	#endif				
////		return setStatus;
//	}
//	setPDPNumber();

#endif			
	for(seti=0;seti<GPRS_TIMES_ALLTRY;seti++)
	{
		setStatus =	GPRSAttach();
		if(setStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r network attach ok\n\r");
		#endif
			break;		
		}
		else{delay_GprsMs(500);}
	}
//	if((rssiValue <15)||(rssiValue >31))//200620
	{
		getRssi();
//		if((rssiValue <10)||(rssiValue >31))getRssi();	
	}
#if GPRS_DEBUG > 0
	printf("\n\r get rssi %d,powerTime:%d\n\r",rssiValue,gprsPowerOnTime);
#endif
	if(setStatus != COMM_OK)	
	{
		serverLog->gprsAttachErrorCounter++;
	#if GPRS_DEBUG > 0
		printf("\n\r network detach :%d\n\r",serverLog->gprsAttachErrorCounter);
	#endif
		gprsAckprintf = 1;
		return setStatus;	
	}
	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	for(seti=0;seti<GPRS_TIMES_ALLTRY;seti++)
	{
		setStatus =	setReceiveDataMode();
		if(setStatus == COMM_OK){break;	}
		else{delay_GprsMs(500);}
	}	
#else
	setStatus = gprsApnSet(APN_TYPE_AUTO);
	if(setStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r gprs apn set ok %d\n\r",simCardType);
	#endif	
	}
	else
	{
		serverLog->gprsApnSetErrorCounter++;
	#if GPRS_DEBUG > 0
		printf("\n\r gprs apn set error :%d\n\r",serverLog->gprsApnSetErrorCounter);
	#endif				
//		return setStatus;
	}	
	for(seti=0;seti<GPRS_TIMES_ALLTRY;seti++)
	{
		setStatus = bringUpWireless();
		if(setStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r bring up wireless with gprs or csd ok\n\r");
		#endif	
			break;
		}
		else{delay_GprsMs(500);}
	}
	if(setStatus != COMM_OK)	
	{
		serverLog->gprsBringUpErrorCounter++;
	#if GPRS_DEBUG > 0
		printf("\n\r bring up wireless with gprs or csd error :%d\n\r",serverLog->gprsBringUpErrorCounter);
	#endif
//		return setStatus ;	
	}	
#endif
	for(seti=0;seti<GPRS_TIMES_ALLTRY;seti++)
	{
		setStatus = getIpStatus();
		if(setStatus == COMM_OK)
		{
		#if GPRS_DEBUG > 0
			printf("\n\r get local ip ok\n\r");
		#endif	
			break;
		}
		else{delay_GprsMs(500);}
	}
	gprsAckprintf = 1;
	if(setStatus != COMM_OK)	
	{
		serverLog->getIpErrorCounter++;
	#if GPRS_DEBUG > 0
		printf("\n\r get local ip error :%d\n\r",serverLog->getIpErrorCounter);
	#endif	
		return  setStatus; 
	}
	
	return setStatus;
}
uint16_t gprsLinkStatusAll(void)
{
	uint16_t linkStatus = 0;
#if(GPRS_MODE_TYPE!=GPRS_MODE_SIM7100)
	linkStatus = 1;
#else	
	uint16_t status = UN_FIND_STATUS;	
	
	uint8_t rxi = 0;	
	gprsAckState	=	GPRS_ACK_SET;
	
	gprsSendBuffer((uint8_t *)"AT+CIPCLOSE?\r\n",14);
	
	status = gprsAckWait("+CIPCLOSE: ",11);		
	if(status == COMM_BACK)
	{	
		status = findStrRxGprs("+CIPCLOSE: ",11);	
		if(status < FIND_ERROR)
		{		
			clearBufPart(status,11);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			receivePrintf("\r\n CIPCLOSE:",GPRS_RX_Buffer,status,50);
		#endif
			for(rxi=0;rxi<10;rxi++)
			{
				if(GPRS_RX_Buffer[(status + rxi*2)&GPRS_RX_DATA_SIZE]=='1')
				{
					linkStatus |= (1<<rxi);
				}				
			}
		}	
	}	
#endif
	return linkStatus;	 
}

uint8_t gprsLinkStatus(uint8_t linknum)
{
	uint8_t status = COMM_LINK_NONE;	
	uint16_t linkStatus = 0;

	linkStatus	= gprsLinkStatusAll();
	if((linkStatus&(1<<linknum))>0)
	{
		status = COMM_LINK_OK;
	}
	return status;	 
}
//extern uint16_t waitStrRxGprs(uint8_t *buf,uint8_t bufLength);
//uint8_t gprsLinkStatus1(uint8_t linknum)
//{
//	__IO uint16_t status = UN_FIND_STATUS;	
////	uint8_t rxi = 0;

//	gprsAckState	=	GPRS_ACK_SET;
//	gprsSendBuffer((uint8_t *)"AT+CIPCLOSE?\r\n",14);
//	
//	status = gprsAckWait("+CIPCLOSE: ",11);		
//	if(status == COMM_BACK)
//	{	
//		status = findStrRxGprs("+CIPCLOSE: ",11);	
//		if(status < FIND_ERROR)
//		{		
//			clearBufPart(status,11);//���ٲ����ֽ�
//		#if GPRS_DEBUG > 0
//			receivePrintf("\r\n CIPCLOSE:",GPRS_RX_Buffer,status,50);
//		#endif	
//			if(GPRS_RX_Buffer[(status + linknum*2)&GPRS_RX_DATA_SIZE]=='1'){status = COMM_LINK_OK;}	
//			else{status = COMM_LINK_NONE;}
//		}
//		else
//		{					
//			status = COMM_LINK_NONE;					
//		}
//	}
//	else
//	{
//		status = COMM_LINK_NONE;
//	}
//	return status;	 
//}

uint8_t gprsLink(uint8_t *buf,uint8_t linknum)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t rxi			= 0;
	char cmbuf[20];
clearBuf();
	gprsAckState	=	GPRS_ACK_SEND;

	gprsSendBuffer(&buf[1],buf[0]);	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100) 
	cmbuf[0] = sprintf(cmbuf+1,"+CIPOPEN: %d,",linknum);	
#else
	cmbuf[0] = sprintf(cmbuf+1,"%s","CONNECT");
#endif
	status = gprsAckWait(cmbuf+1,cmbuf[0]);
	
//#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100) 
//if(status  == COMM_TIMEOUT)
//{
//	if(gprsLinkStatus(linknum)==COMM_LINK_OK)return COMM_OK;	
//}
//#endif
	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		cmbuf[0] = sprintf(cmbuf+1,"+CIPOPEN: %d,0",linknum);		
	#else
		cmbuf[0] = sprintf(cmbuf+1,"%s","CONNECT OK");	
	#endif
		status = findStrRxGprs(cmbuf+1,cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,10);
		#if GPRS_DEBUG > 0
			printf("\n\r CONNECT ok \n\r");
		#endif	  
			status = COMM_OK;
		}
		else
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			cmbuf[0] = sprintf(cmbuf+1,"+CIPOPEN: %d,",linknum);		
		#else
			cmbuf[0] = sprintf(cmbuf+1,"%s","ALREADY CONNECT");
		#endif
			status = findStrRxGprs(cmbuf+1,cmbuf[0]);
			if(status < FIND_ERROR)
			{
				clearBufPart(status,12);//���ٲ����ֽ�
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100) 
			 #if GPRS_DEBUG > 0
				printf("\n\r CIPOPEN:0,%c%c ",GPRS_RX_Buffer[status],GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE]);								
				if((GPRS_RX_Buffer[status]=='1')&&(GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE]=='1'))
				{
					printf(" DNS parse failed \n\r");
				}	 
			 #endif				
				if((GPRS_RX_Buffer[status]=='7')||(GPRS_RX_Buffer[status]=='9')){status = COMM_OK;}
				else{status = COMM_ERR;}
			#else
				status = COMM_OK;			 
			#endif	
			#if GPRS_DEBUG > 0
				if(status == COMM_OK)printf("\n\r ALREADY CONNECT \n\r");
			 #endif				
			}
			if(status != COMM_OK)		
			{
				#if GPRS_DEBUG > 0							
				printf("\n\r CONNECT error :");	
				for(rxi=0;rxi<GPRS_RXCounter;rxi++)
					devStoreBuffer(&GPRS_RX_Buffer[(rxi+GPRS_RX_ptr_Out)&GPRS_RX_DATA_SIZE],1);
				printf("\n\r");
			#endif
				status = findStrRxGprs("ERROR",5);
				if(status < FIND_ERROR)clearBufPart(status,5);
				status =  COMM_ERR;
			}		
		}
		clearBuf();
	}	
	return status	;	  	 
}

uint8_t gprsSend(uint8_t *buf,__IO uint16_t length,uint8_t dataType,uint8_t linknum)
{
	uint16_t status = UN_FIND_STATUS;
	__IO uint16_t sendLength = 0x00;
	uint8_t commdLength = 0x00;
//	uint16_t rxj=0;
	char sendComm[30] = "AT+CIPSEND=137";

#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
//	uint16_t rxj=0;
	uint8_t rxi = 0;
	uint16_t rxStatus = 0;
#endif
	clearBuf();
	gprsAckState	=	GPRS_ACK_SEND;
	if(dataType != MQTT_DATA)
	{
		sendLength 	= length+packHeadLength;
	}
	else
	{
		sendLength 	= length;
	}

#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	commdLength = sprintf(sendComm,"AT+CIPSEND=%d,%d\r\n",linknum,sendLength);
#else
	commdLength = sprintf(sendComm,"AT+CIPSEND=%d\r\n",sendLength);
#endif
	gprsSendBuffer((uint8_t *)sendComm,commdLength);
	
//#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = gprsAckWait(">",1);
	if(status	!= COMM_BACK)
	{
		return status;//COMM_ERR;
	}
	else
	{
		status = findStrRxGprs(">",1);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,1);//���ٲ����ֽ�			
		}
	}
//#else	//'>'_201114
//	delay_GprsMs(50);
//#endif
	if(dataType != MQTT_DATA)
	{
		gprsSendBuffer((uint8_t *)packetHeadBuf,packHeadLength);
	}
	if((dataType == OTHER_DATA)||(dataType == MQTT_DATA))
	{
		gprsSendBuffer(buf,length);
	}	
//	gprsSendBuffer((uint8_t *)"\r\n",2);
	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100) //20191214
	commdLength = sprintf(sendComm,"+CIPSEND: %d,",linknum);
#else
	commdLength = sprintf(sendComm,"%s","SEND ");
#endif
	status = gprsAckWait(sendComm,commdLength);	
	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		commdLength = sprintf(sendComm,"+CIPSEND: %d,",linknum);		
	#else
		commdLength = sprintf(sendComm,"%s","SEND OK");
	#endif
		status = findStrRxGprs(sendComm,commdLength);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,commdLength);//���ٲ����ֽ�
		  clearPtrOut(status);
		 #if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			rxStatus	=	status;
			status = COMM_ERR;	
			for(rxi=0;rxi<5;rxi++)
			{
				if(GPRS_RX_Buffer[(rxStatus+rxi)&GPRS_RX_DATA_SIZE]==',')
				{
					//if(GPRS_RX_Buffer[(rxStatus+rxi+1)&GPRS_RX_DATA_SIZE]!='0')
					if(GPRS_RX_Buffer[(rxStatus)&GPRS_RX_DATA_SIZE] == GPRS_RX_Buffer[(rxStatus+rxi+1)&GPRS_RX_DATA_SIZE])
					{
						status = COMM_OK;
						break;
					}
				}
			}		
		 #if GPRS_DEBUG > 0
			if(status == COMM_OK){printf("\r\n SEND OK ");}
			else{printf("\r\n SEND fault ");}
			receivePrintf(" ",GPRS_RX_Buffer,rxStatus,10);
		#endif
		//	if(GPRS_RX_Buffer[(rxStatus+rxi+1)&GPRS_RX_DATA_SIZE] == '0')status = COMM_OK;
	#else
			status = COMM_OK;
		#if GPRS_DEBUG > 0
			printf("\r\n SEND OK \r\n");
		 #endif			
	#endif		
		}
		else
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
			status = findStrRxGprs("+CIPERROR: ",11);
		#else
			status = findStrRxGprs("+CME ERROR",10);
		#endif
			if(status < FIND_ERROR)
			{
				clearBufPart(status,10);
			#if GPRS_DEBUG > 0
				receivePrintf("\r\n SEND error: ",GPRS_RX_Buffer,status,11);
			#endif				
			}		
			status =  COMM_ERR;		
		}		
	}
	return status	;	  	 
}


uint8_t readReceiveMode(void)
{
	uint16_t status	= UN_FIND_STATUS;
	uint8_t result;
	char cmbuf[20];	
	clearBuf();
	result = 0;
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","AT+CIPRXGET?\r\n");
	gprsAckState	=	GPRS_ACK_SET;
	gprsSendBuffer((uint8_t*)(cmbuf+1),cmbuf[0]);
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","+CIPRXGET: ");
	status = gprsAckWait(cmbuf+1,cmbuf[0]);
	if(status == COMM_BACK)
	{
		status = findStrRxGprs(cmbuf+1,cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�	
			result = GPRS_RX_Buffer[status]-'0';			
		}
	}
	return result;
}
#if(GPRS_MULTI_LINK>0)
uint16_t readReceivelen(uint8_t linknum)
{
	uint16_t status	= UN_FIND_STATUS;
	uint16_t len;
	uint8_t rxi;
	char cmbuf[20];
//	clearBuf();
	len = 0;
	cmbuf[0] 	= sprintf(cmbuf+1,"AT+CIPRXGET=4,%d\r\n",linknum);
	gprsAckState	=	GPRS_ACK_SET;
	gprsSendBuffer((uint8_t*)(cmbuf+1),cmbuf[0]);
	
	cmbuf[0] 	= sprintf(cmbuf+1,"+CIPRXGET: 4,%d,",linknum);
	status = gprsAckWait(cmbuf+1,cmbuf[0]);
	if(status == COMM_BACK)
	{
		status = findStrRxGprs(cmbuf+1,cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�	
			for(rxi=0;rxi<5;rxi++)
			{
				if(GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE]== '\r')
				{
					if(GPRS_RX_Buffer[(status+rxi+1)&GPRS_RX_DATA_SIZE]== '\n')
					{
						cmbuf[rxi] = '\0';
						len= atoi(cmbuf);
					#if(GPRS_DEBUG>1)
						printf("\r\n +CIPRXGET: 4,%d,%d",linknum,len);
					#endif
						break;
					}
				}
				cmbuf[rxi] = GPRS_RX_Buffer[(status+rxi)&GPRS_RX_DATA_SIZE];
			}				
		}
		else
		{
			status = findStrRxGprs("+IP ERROR: ",11);
			if(status < FIND_ERROR)
			{
				clearBufPart(status,11);//���ٲ����ֽ�
			#if(GPRS_DEBUG>0)
				receivePrintf("\r\n +IP ERROR:",GPRS_RX_Buffer,status,50);
			#endif
			}
			status = findStrRxGprs("ERROR",5);//20210131
			if(status < FIND_ERROR)
			{
				clearBufPart(status,5);//���ٲ�����		
			}
			status = COMM_ERR;
		}
	}
	else
	{
//		status = findStrRxGprs("+IP ERROR: ",11);
//		if(status < FIND_ERROR)
//		{
//			clearBufPart(status,11);//���ٲ����ֽ�
//		#if(GPRS_DEBUG>0)
//			receivePrintf("\r\n +IP ERROR:",GPRS_RX_Buffer,status,50);
//		#endif
//		}
//		status = findStrRxGprs("ERROR",5);//20210131
//		if(status < FIND_ERROR)
//		{
//			clearBufPart(status,5);//���ٲ�����		
//		}
		status = COMM_ERR;
	}
	return len;
}
#else
uint16_t readReceivelen(uint8_t linknum)
{
	uint16_t status 		= UN_FIND_STATUS;
	uint16_t curLength 	= 0x00;
	uint16_t curRxAddress = 0x00;
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t rxi = 0x00,rxj=0;	 
	int8_t findResult = -1;

#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = findStrRxGprs("+IPD",4);
#else
	status = findStrRxGprs("+IPD,",5);
#endif
	if(status < FIND_ERROR)
	{		
		curRxAddress = status;
				
		//���ݽ�����ȫ�жϣ���ȡ���ݳ���
		for(rxi = 0x00;rxi <5;rxi ++)
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			if((GPRS_RX_Buffer[curRxAddress]== 0x0D)&&(GPRS_RX_Buffer[(curRxAddress+1)&GPRS_RX_DATA_SIZE] ==0x0A))
		#else
			if(GPRS_RX_Buffer[curRxAddress]== 0x3A)//':' 0x3A
		#endif
			{
				valueBuf[rxi] = '\0';
				curLength = atoi(valueBuf);			
				for(rxj = 0x00;rxj < 200;rxj++)
				{		
					delay_GprsMs(20);
					if(gprsRxcounterCal(curRxAddress) >= curLength)
					{				
						findResult	= 0x00;
						break;
					}							
				}
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[curRxAddress];
				curRxAddress ++;
				curRxAddress &= GPRS_RX_DATA_SIZE;
			}
		}
	}	
	if(findResult!=0)curLength = 0;
	return curLength;
}
#endif
uint16_t waitRxGprsData(uint8_t linknum,uint32_t *code,uint8_t waitFlag)
{
	uint16_t status	= UN_FIND_STATUS;
	
	char cmbuf[20];	
#if(GPRS_MULTI_LINK>0)	
	cmbuf[0] 	= sprintf(cmbuf+1,"+CIPRXGET: 1,%d",linknum);
#else
 #if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","+IPD");
 #else
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","+IPD,");
 #endif	
#endif
  if(waitFlag == STATUS_DONE)
	{
		gprsAckState	=	GPRS_ACK_SEND;	
		status = gprsAckWait(cmbuf+1,cmbuf[0]);
	}
	else
	{
		status = COMM_BACK;
	}
	if(status == COMM_BACK)
	{		
		status = findStrRxGprs(cmbuf+1,cmbuf[0]);
		if(status < FIND_ERROR)
		{
		#if(GPRS_MULTI_LINK>0)
			clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�	
		#endif
			status 		= COMM_OK;		
		}
		else
		{
			status = findStrRxGprs("ERROR",5);
			if(status < FIND_ERROR)clearBufPart(status,5);		
			status	= COMM_ERR;
		}
		*code	= status;
	}
	else
	{
		*code	= HTTP_TIME_OUT;
	}
#if(GPRS_MULTI_LINK>0)	
	clearBuf();
#endif
	return status;
}
uint16_t receiveDataFlag(void)
{
	uint16_t status	= UN_FIND_STATUS;
#if(GPRS_MULTI_LINK>0)		
	char cmbuf[20];	
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","+CIPRXGET: 1,");	
	status = findStrRxGprs(cmbuf+1,cmbuf[0]);
	if(status < FIND_ERROR)
	{
		clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�	
		status	= GPRS_RX_Buffer[(status)&GPRS_RX_DATA_SIZE]-'0';
	}
	else
	{
		status	= NO_COMM;
	}	
#endif
	return status;
}
uint8_t findipcloseFlag(void)
{
	uint16_t status	= UN_FIND_STATUS;
#if(GPRS_MULTI_LINK>0)
	char cmbuf[20];	
	cmbuf[0] 	= sprintf(cmbuf+1,"%s","+IPCLOSE: ");
	
	status = findStrRxGprs(cmbuf+1,cmbuf[0]);
	if(status < FIND_ERROR)
	{
		clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�	
		if((GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE]>='0')&&(GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE]<='9'))
		{
			status	= GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE] -'0';
		}
		else
		{
			status	= 0xF0;
		}			
	}
	else
	{
		status	= 0xF1;
	}
#endif
	return (uint8_t)status;
}

#if(GPRS_MULTI_LINK>0)
uint16_t findRxGprsData(uint8_t *buf,uint16_t *rxlen,uint8_t linknum)
{
	uint16_t status	= UN_FIND_STATUS;
	char cmbuf[20];
	uint16_t rxi = 0,rxj=0;
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t curRxAddress = 0x00;
	uint16_t curLength 	= 0x00;
	int8_t findResult = -1;
	buf[0]	=	0;
	*rxlen	=	0;
	clearBuf();
	gprsAckState	=	GPRS_ACK_RX;

	cmbuf[0] 	= sprintf(cmbuf+1,"AT+CIPRXGET=2,%d\r\n",linknum);
	gprsSendBuffer((uint8_t *)(cmbuf+1),cmbuf[0]);	
	
	cmbuf[0] 	= sprintf(cmbuf+1,"+CIPRXGET: 2,%d,",linknum);
	status = gprsAckWait(cmbuf+1,cmbuf[0]);
	if(status == COMM_BACK)
	{
		status = findStrRxGprs(cmbuf+1,cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�			
		#if(GPRS_DEBUG>1)
			printf("\r\n +CIPRXGET: 2,%d",linknum);
			receivePrintf(",",GPRS_RX_Buffer,status,10);
		#endif
			curRxAddress = status;
			for(rxi = 0x00;rxi <5;rxi ++)
			{
				if(GPRS_RX_Buffer[curRxAddress]== ',')	
				{
					valueBuf[rxi] = '\0';
					curLength = atoi(valueBuf);
					curRxAddress++;	
					if(curLength>400)delay_GprsMs(110);
					for(rxj = 0x00;rxj < 200;rxj++)
					{		
						delay_GprsMs(20);							
						if(gprsRxcounterCal(curRxAddress) >= curLength)
						{				
							findResult	= 0x00;
							break;
						}														
					}
					break;
				}
				else
				{
					valueBuf[rxi] = GPRS_RX_Buffer[curRxAddress];
					curRxAddress ++;
					curRxAddress &= GPRS_RX_DATA_SIZE;
				}
			}
			if(findResult==0)
			{				
				for(rxj=0;rxj<6;rxj++)
				{				
					if((GPRS_RX_Buffer[(curRxAddress)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(curRxAddress+1)&GPRS_RX_DATA_SIZE]==0x0A))
					{
						curRxAddress++;//0x0A
						curRxAddress++;//��ʼ�ֽ�
						for(rxi = 0x00;rxi < curLength;rxi++)
						{
							buf[rxi] = GPRS_RX_Buffer[(curRxAddress+rxi)&GPRS_RX_DATA_SIZE];
						}
						*rxlen = curLength;	
						status = COMM_OK;
						break;
					}
					curRxAddress++;
				}				
			}
			else
			{
				status	= COMM_ERR;
			}		
		}
		else
		{		
			status = findStrRxGprs("+IP ERROR: ",11);
			if(status < FIND_ERROR)
			{
				clearBufPart(status,11);//���ٲ����ֽ�
			#if(GPRS_DEBUG>0)
				receivePrintf("\r\n +IP ERROR:",GPRS_RX_Buffer,status,50);
			#endif
			}
			status = findStrRxGprs("ERROR",5);//20210131
			if(status < FIND_ERROR)
			{
				clearBufPart(status,5);//���ٲ�����		
			}
			status = COMM_ERR;		
		}				
	}
	else
	{		
//		status = findStrRxGprs("+IP ERROR: ",11);
//		if(status < FIND_ERROR)
//		{
//			clearBufPart(status,11);//���ٲ����ֽ�
//		#if(GPRS_DEBUG>0)
//			receivePrintf("\r\n +IP ERROR:",GPRS_RX_Buffer,status,50);
//		#endif
//		}
//		status = findStrRxGprs("ERROR",5);//20210131
//		if(status < FIND_ERROR)
//		{
//			clearBufPart(status,5);//���ٲ�����		
//		}
		status = COMM_ERR;		
	}
	return status;
}
#else
uint16_t findRxGprsData(uint8_t *buf,uint16_t *rxlen,uint8_t linknum)
{
	uint16_t status 		= UN_FIND_STATUS;
	uint16_t ipdStatus 	= UN_FIND_STATUS;
	uint16_t curLength 	= 0x00;
	uint16_t curRxAddress = 0x00;
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t rxi = 0x00,rxj=0;	 
	int8_t findResult = -1;
	buf[0]	=	0;
	*rxlen	=	0;

#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = findStrRxGprs("+IPD",4);
#else
	status = findStrRxGprs("+IPD,",5);
#endif
	if(status < FIND_ERROR)
	{
		ipdStatus = status;
		status = COMM_OK;
		
		curRxAddress = ipdStatus;
				
		//���ݽ�����ȫ�жϣ���ȡ���ݳ���
		for(rxi = 0x00;rxi <5;rxi ++)
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			if((GPRS_RX_Buffer[curRxAddress]== 0x0D)&&(GPRS_RX_Buffer[(curRxAddress+1)&GPRS_RX_DATA_SIZE] ==0x0A))
		#else
			if(GPRS_RX_Buffer[curRxAddress]== 0x3A)//':' 0x3A
		#endif
			{
				valueBuf[rxi] = '\0';
				curLength = atoi(valueBuf);			
				for(rxj = 0x00;rxj < 200;rxj++)
				{		
					delay_GprsMs(20);
					if(gprsRxcounterCal(curRxAddress) >= curLength)
					{				
						findResult	= 0x00;
						break;
					}										
				}
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[curRxAddress];
				curRxAddress ++;
				curRxAddress &= GPRS_RX_DATA_SIZE;
			}
		}

		if(!findResult)
		{
			clearBufPart(ipdStatus,4);//���ٲ����ֽ�		
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)					
			curRxAddress ++;
		#endif
			for(rxi = 0x00;rxi < curLength;rxi++)
			{
				buf[rxi] = GPRS_RX_Buffer[(curRxAddress+rxi+1)&GPRS_RX_DATA_SIZE];
			}
			*rxlen	=	curLength;		
		}
		else
		{
			status	= COMM_ERR;
		}
	}
	else
	{
		status = NO_COMM;
	}
#if(GPRS_DEBUG>0)
	if(status == COMM_OK)printf("\r\n gprs data received.\r\n");		
#endif
	return status;
}
#endif

int8_t findServerIpAddr(uint8_t *buf,char* addr, uint16_t port,uint8_t linknum)
{
	uint16_t 	status 			= UN_FIND_STATUS;
	uint16_t 	findStatus 	= UN_FIND_STATUS;
	uint8_t 	cmbuf[100];

	int8_t 	rxResult = -1;
	uint8_t rxi = 0x00;
	uint8_t rxj	=	0;
	uint8_t rxLength = 0;
	uint8_t	bLength =0;	 

	clearBuf();//gprsBufHeadInit();
	gprsAckState	=	GPRS_ACK_SEND;
	gprsFlag 	=	COMM_SEND;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	cmbuf[0] 	= sprintf((char*)(cmbuf+1),"AT+CDNSGIP=\"%s\"\r\n",addr);
#else
	cmbuf[0] 	= sprintf((char*)(cmbuf+1),"AT+CDNSGIP=%s\r\n",addr);
#endif
	gprsSendBuffer(cmbuf+1,cmbuf[0]);	
	
	cmbuf[0] = sprintf((char*)(cmbuf+1),"%s","+CDNSGIP:");
	status = gprsAckWait((char*)(cmbuf+1),cmbuf[0]);
	if(status  == COMM_BACK)
	{
		status = findStrRxGprs((char*)(cmbuf+1),cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,cmbuf[0]);
			cmbuf[0] = sprintf((char*)(cmbuf+1),"1,\"%s\",\"",addr);
			status = COMM_BACK;
		}
		else
		{
			status = findStrRxGprs("ERROR",5);
			if(status < FIND_ERROR)clearBufPart(status,5);		
			status	= COMM_ERR;
		}
	}
	if(status  == COMM_BACK)
	{
		status = findStrRxGprs((char*)(cmbuf+1),cmbuf[0]);
		if(status < FIND_ERROR)
		{
			clearBufPart(status,cmbuf[0]);//���ٲ����ֽ�			
			//TCP����ͷ������
			findStatus = status;
		#if GPRS_DEBUG > 1
			printf(" server 1 ip start address %d\r\n",findStatus);
		#endif
			rxLength	= 0x01;		
			bLength		=	sprintf((char*)(buf+rxLength), "AT+CIPOPEN=%d,\"TCP\",\"",linknum);
			rxj				=	bLength;
			bLength++;
		#if GPRS_DEBUG > 1
			printf(" server 1 link ip start address %d\r\n",bLength);
		#endif
			for(rxi = 0x00;rxi <16;rxi ++ )
			{
				if(GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x22)
				{
					//װ�ض˿ں�
				#if GPRS_DEBUG > 1
					printf(" server ip end address %d\r\n",(findStatus+rxi));  
				#endif
		
					rxLength 	= bLength;							
					bLength		=	sprintf((char*)(buf+rxLength),"\",%d\r\n",port);
					bLength  += rxLength;    
					buf[0] 		= bLength-1;  				
				#if GPRS_DEBUG > 0
					printf(" get ip ok");
					printf(" domain:%s,%d,ipAddr:",addr,port);
					devStoreBuffer(buf+rxj,bLength-rxj);		
					printf("\r\n");
				#endif
					rxResult = COMM_OK;
					break;
				}
				else
				{
					if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]>0x2c)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]<0x3B))//�ָ���
					{
						buf[bLength++] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
					}
					else
					{
					#if GPRS_DEBUG > 0
						printf("error character\r\n");
					#endif
						rxResult  = -5;
						break;
					} 
				}
			} 
		}
		else
		{
		#if(GPRS_DEBUG>0)
			receivePrintf("\r\n DNS GENERAL ERROR:",GPRS_RX_Buffer,status,50);
		#endif
			status =  COMM_ERR;		
		}
		clearBuf();
	}	
	return rxResult;
}

uint8_t closeTcp(void)
{
	uint16_t status = UN_FIND_STATUS;

	gprsAckState	=	GPRS_ACK_RX;//GPRS_ACK_SET 20200413
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+NETCLOSE\r\n",13);
	status = gprsAckWait("+NETCLOSE: ",11);
#else
	gprsSendBuffer((uint8_t *)"AT+CIPSHUT\r\n",12);
	status = gprsAckWait("SHUT OK",7);
#endif
	
	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+NETCLOSE: 0",12);
	#else
		status = findStrRxGprs("SHUT OK",7);
	#endif
		if(status < FIND_ERROR)
		{
			clearBufPart(status,7);//���ٲ����ֽ�
			status = COMM_OK;
		}
		else
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			status = findStrRxGprs("+NETCLOSE: ",11);			  
			if(status < FIND_ERROR)
			{
				clearBufPart(status,11);//���ٲ����ֽ�
			#if(GPRS_DEBUG>0)
				if(gprsAckprintf>0)receivePrintf("\r\n NETCLOSE ERROR:",GPRS_RX_Buffer,status,20);
			#endif					
			}
			else		
			{
				status = findStrRxGprs("+IP ERROR:",10);			  
				if(status < FIND_ERROR)
				{
					clearBufPart(status,10);//���ٲ����ֽ�
				#if(GPRS_DEBUG>0)
					if(gprsAckprintf>0)receivePrintf("\r\n +IP ERROR:",GPRS_RX_Buffer,status,20);
				#endif					
				}
			}
		#endif			
			status = COMM_ERR;
		}
		clearBuf();
	}
	return status	;	  	 
}

uint8_t closeSingleTcp(uint8_t linknum)
{
	__IO uint16_t status = UN_FIND_STATUS;		
	char cmbuf[20];
	char rcbuf[20];
	gprsAckState	=	GPRS_ACK_SEND;
clearBuf();	
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	cmbuf[0] = sprintf(cmbuf+1,"AT+CIPCLOSE=%d\r\n",linknum);
	rcbuf[0] = sprintf(rcbuf+1,"+CIPCLOSE: %d,",linknum);	
#else
	cmbuf[0] = sprintf(cmbuf+1,"%s","AT+CIPCLOSE=1\r\n");
	rcbuf[0] = sprintf(rcbuf+1,"%s","CLOSE ");
#endif
	gprsSendBuffer((uint8_t*)(cmbuf+1),cmbuf[0]);
	status = gprsAckWait(rcbuf+1,rcbuf[0]);	
	if(status == COMM_BACK)
	{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		rcbuf[0] = sprintf(rcbuf+1,"+CIPCLOSE: %d,0",linknum);			
	#else
		rcbuf[0] = sprintf(rcbuf+1,"%s","CLOSE OK");
	#endif
		status = findStrRxGprs(rcbuf+1,rcbuf[0]);
		if(status < FIND_ERROR)
		{		
			clearBufPart(status,rcbuf[0]);//���ٲ����ֽ�					
			status = COMM_OK;					
		}
		else
		{				
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100) 
			rcbuf[0] = sprintf(rcbuf+1,"+CIPCLOSE: %d,",linknum);	
			status = findStrRxGprs(rcbuf+1,rcbuf[0]);//0,14		
			if(status < FIND_ERROR)
			{
				clearBufPart(status,10);//���ٲ����ֽ�			
			#if(GPRS_DEBUG>0)
				printf("\n\r CIPCLOSE:%c%c\n\r",GPRS_RX_Buffer[status],GPRS_RX_Buffer[(status+1)&GPRS_RX_DATA_SIZE]);
			#endif
			}			
		#endif	
			status = findStrRxGprs("ERROR",5);
			if(status < FIND_ERROR)clearBufPart(status,5);
			status = COMM_ERR;			
		}
	}
	return status	;	  	 
}
uint8_t gprsReset(void)//230620
{
	return sendAtCmd_AckOK("AT+CRESET\r\n",GPRS_ACK_SET); 
}
 uint8_t enterLowPowerMode(void )
{
	uint16_t status = UN_FIND_STATUS;

	gprsAckState	=	GPRS_ACK_SET;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+CPOF\r\n",9);
//	delay_GprsMs(1000);
	status = gprsAckWait("OK",2);
#else
	gprsSendBuffer((uint8_t *)"AT+CPOWD=1\r\n",12);	
	status = gprsAckWait("NORMAL POWER DOWN",17);
#endif
	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)		
		status = findStrRxGprs("OK",2);
	#else		
		status = findStrRxGprs("NORMAL POWER DOWN",17);
	#endif
		if(status < FIND_ERROR)
		{
			clearBufPart(status,2);//���ٲ����ֽ�
		#if GPRS_DEBUG > 0
			printf("\n\r enter low power ok");
		#endif
			status = COMM_OK;
		}
		else
		{
		#if GPRS_DEBUG > 0
			printf("\n\r enter low power error");
		#endif
			status =  COMM_ERR;
		}
		clearBuf();
	}
	return status	;
}
 uint8_t gprsSleepModeSwitch(uint8_t state)
{
	uint16_t status = UN_FIND_STATUS;
	gprsFlag =  COMM_SEND;
	if(state == GPRS_SLEEP_MODE_DIS)
	{
		gprsSendBuffer((uint8_t *)"AT+CSCLK=0\r\n",12);	
	}
	else
	{
		gprsSendBuffer((uint8_t *)"AT+CSCLK=1\r\n",12);		  
	}

	gprsAckState	=	GPRS_ACK_SET;	
	status = gprsAckWait("OK",2);

	if(status  == COMM_TIMEOUT)
	{
	#if GPRS_DEBUG > 0
		printf("clk time out\r\n");
	#endif
		status = COMM_TIMEOUT;
	}
	if(status  == COMM_BACK)
	{
		status = findStrRxGprs("OK",2);
		if(status < FIND_ERROR)
		{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		if(state == GPRS_SLEEP_MODE_DIS)  
		{
			printf("control uart sleep off\r\n"); 
		}
		else
		{
			printf("control uart sleep on\r\n"); 
		}    
	#else
		#if GPRS_DEBUG > 0
			printf(" clk ok \r\n");
		#endif
	#endif
			status = COMM_OK;		
		}
		else
		{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		if(state == GPRS_SLEEP_MODE_DIS)  
		{
			printf("control uart sleep off error\r\n");
		}
		else
		{
			printf("control uart sleep on error\r\n"); 
		}    
	#endif
			status =  COMM_ERR;		
		}
		clearBuf();
	}
	return status	;

}


int8_t gsmTaskInit(uint8_t gprsSleepCtr)
{
	uint8_t gsmTry = 0x00;
	uint8_t gsmAllTry = 0x00;
	uint8_t gsmState = GPRS_RUN;	
	int8_t  gsmStatrResult = -1;

#if GPRS_DEBUG > 0
	printf("\n\r Enter GPRS task %d\n\r",GPRS_MODE_TYPE);
#endif
	NVIC_Time5_Status(TIME_START);
	delay_GprsMs(1);
	Time5_Init_Status(TIME_START);
	delay_GprsMs(1);
	delay_GprsMs(5);
#if(SWRESET_CTR>0)	
  SWRESET_TIMER_INIT();// sofeWareWatchBegin(); 230508
#endif
	//	GPRS���ڳ�ʼ�� 
	gprsUSART_Config();
	gprsBufInit();
	serverConfig.simCardReadFlag = FUNCITONG_TURN_ON;
	gprsState = GPRS_TURN_ON;
#if(GPRS_SLEEP_CTR>0)
	if((gprsPowerStatus ==	STATUS_DONE)&&(serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(devPar.stationMode == FUNCITONG_TURN_ON))
	{
		if((gprsSleepCtr >0)&&(mqttServerNum>0))return 2;
	}
#endif	
	gprsPowerOn();
#if GPRS_DEBUG > 0
	printf("\n\r GPRS powerON \n\r");
#endif
	delay_GprsMs(2500);   
	while(GPRS_RUN == gsmState)
	{
		switch(gprsState)
		{
			case GPRS_TURN_ON :
			{
				delay_GprsS(0,15);
			#if GPRS_DEBUG > 0
				printf("\n\r GPRS run powerTime:%d\n\r",gprsPowerOnTime);
			#endif
				gprsState = GPRS_UART_INIT;
				gprsRxState =RX_STATE;
				
				gprsAckprintf = 0;
				for(gsmTry=0;gsmTry<GPRS_RETRY_TIMES_AT;gsmTry++)
				{
					if(sendAt() == COMM_OK)break;
					delay_GprsMs(500);
					if(gsmTry == (GPRS_RETRY_TIMES_AT-1))gsmAllTry = GPRS_TIMES_RETRY_ALL;
				}
				gprsAckprintf = 1;	
			}
			case GPRS_UART_INIT :
			{					
				if(gprsUartInit()== COMM_OK)
				{				
					gsmState 				= GPRS_SLEEP;
					gsmStatrResult 	= 0x01;
					gprsState 			= GSM_MOUDLE_IDLE ;
					gsmAllTry				= 0x00;
				}
				else
				{
					gsmAllTry ++;
					if(gsmAllTry >=GPRS_TIMES_RETRY_ALL)
					{
						gprsState	= GPRS_TURN_OFF;
						break;						
					}	
					delay_GprsMs(2500);					
				}
				break;
			}			
			case GPRS_TURN_OFF:
			default :
			{
	//���ش���ָʾ���ر�GSMģ��
				gsmState				= GPRS_SLEEP;
				gsmStatrResult 	= -1;
				enterLowPowerMode();
				delay_GprsMs(100);
		
				NVIC_Time5_Status(TIME_END);
				Time5_Init_Status(TIME_END);
								
				gprsPowerOff();                  
				break;
			}
		}
	}
#if GPRS_DEBUG > 0
	printf("\n\r GPRS Init:%d,powerTime:%d\n\r",gsmStatrResult,gprsPowerOnTime);
#endif
	return gsmStatrResult;
}
int8_t gprsStart(void)
{
	uint8_t gprsTry	= 0x00;	
	int8_t	gprsStatrResult = -1;
	for(gprsTry=0;gprsTry<GPRS_TIMES_ALLTRY;gprsTry++)
	{
		if(gprsCheck()==COMM_OK)
		{
			gprsStatrResult = 0x01;
			break;
		}	
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
		if(gprsTry == 1)gprsReset();//230620
	#endif
		if(gprsTry<(GPRS_TIMES_ALLTRY-1))delay_GprsS(gprsPowerOnTime,40);//210623
//		delay_GprsMs(5000);	
	}
	if(gprsStatrResult == 1)
	{
		gprsAckprintf=0;
		closeTcp();
		gprsAckprintf=1;
		gprsStatrResult = -2;
		serverBufInt();//�洢��־��ʼ��	
		serverConfig.getIpFlag			 = 0;
		serverConfig.linkErrorCouner = 0;
		serverConfig.stationServerChangeFlag 	= 0x00;   
    serverConfig.delFlagHost2 						= 0x00;	
		if(xSemaphoreTake(xSPIFlashMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
        {
            flashStoreTask(serverConfig);
            xSemaphoreGive(xSPIFlashMutex);
        }
		servStoreFlagInit(&serverConfig);		
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)	
		simCardReadTask(&serverConfig);
	#endif
		for(gprsTry=0;gprsTry<GPRS_TIMES_ALLTRY;gprsTry++)
		{
			if(gprsSet(&serverLSRLog[TCP_SERVER_INSENTEK])==COMM_OK)
			{
				gprsStatrResult = 0x01;
				break;
			}
			else
			{
				if(gprsTry<(GPRS_TIMES_ALLTRY-1))delay_GprsS(gprsPowerOnTime,30);//210623
//				delay_GprsMs(5000);	
			}
		}	
	}
	return gprsStatrResult;
}

void simCardReadTask(httpConfig *rxResult)
{
	uint8_t gsmTry = 0x00;
	//uint8_t gsmAllTry = 0x00;
	uint8_t gsmState = GPRS_RUN;	
	//    int8_t   gsmStatrResult = -1;
	gprsState = GPRS_OK;
	//��ʼ��
	simCardInfInit();

	while(GPRS_RUN == gsmState)
	{  
		//��ȡ�������
		if(COMM_OK == simCardRead())
		{           
			gsmState = GPRS_SLEEP;
			rxResult->simCardReadOkFlag = FUNCITONG_TURN_ON;
			rxResult->simCardReadFlag = FUNCITONG_TURN_OFF;
			break;
		}
		else
		{
			gsmTry ++;
			if(gsmTry > 2)
			{
				gsmState = GPRS_SLEEP;
			}
		}
	}
}
void gsmTaskEnd(uint8_t linknum)
{
	gprsState = GPRS_TURN_OFF;
	if(gprsPowerStatus == STATUS_DONE)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		if(gprsLinkStatus(linknum)==COMM_LINK_OK)
		{closeSingleTcp(linknum);delay_GprsMs(500);gprsLinkStatus(linknum);}
	#else
		closeSingleTcp(linknum);
	#endif
	#if(MQTT_USE_CTR==0)
		closeTcp();
	#endif

	}
	clearBuf();
#if(GPRS_SLEEP_CTR>0)
	if(!((serverConfig.gsmRunMode == FUNCITONG_TURN_ON)&&(devPar.stationMode == FUNCITONG_TURN_ON)&&(mqttServerNum>0)))
#endif
	{
		//�ر�GPRS��Դ����IO
		enterLowPowerMode();
		gprsPowerOff();

		gprsRxState = UN_RX;
		NVIC_Time5_Status(TIME_END);
		Time5_Init_Status(TIME_END);

		gprsUsartLowPower();
	#if GPRS_DEBUG > 0
		printf("\r\n End GPRS task powerTime:%d\r\n",gprsPowerOnTime);
	#endif 
	}		
}



uint8_t gpsSwitch(uint8_t state)
{
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	delay_GprsMs(200);
	if(state == GPS_0N)
	{
		return sendAtCmd_AckOK("AT+CGPS=1\r\n",GPRS_ACK_OK);
	}
	else
	{
		return sendAtCmd_AckOK("AT+CGPS=0\r\n",GPRS_ACK_OK);	  
	}
#else
	if(state == GPS_0N)
	{
		return sendAtCmd_AckOK("AT+CGNSPWR=1\r\n",GPRS_ACK_OK);
	}
	else
	{
		return sendAtCmd_AckOK("AT+CGNSPWR=0\r\n",GPRS_ACK_OK);	  
	}
#endif
}

//����GPS���������ʽ
uint8_t gpsResetMode(void)
{
	return sendAtCmd_AckOK("AT+CGNSSEQ=\"RMC\"\r\n",GPRS_ACK_SET);	
}

uint8_t gpsStart(void)
{
	uint8_t setStatus = 0;
// #if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
// 	uint16_t ldoV;
// #endif
	setStatus =	gpsSwitch(GPS_0N);
	if(setStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r gps power on ok %d\n\r",gprsPowerOnTime);
	#endif	   
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r gps power error\n\r");
	#endif	   
		return 	setStatus;
	}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)
	setStatus =	gpsResetMode();
	if(setStatus == COMM_OK)
	{
	#if GPRS_DEBUG > 0
		printf("\n\r gps cold start  ok \n\r");
	#endif	   
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\n\r gps cold start error\n\r");
	#endif	   
		return 	setStatus;
	}
#endif
	return 	setStatus;
}

uint8_t getGpsStatus()
{
	uint16_t status = UN_FIND_STATUS;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	uint16_t status2 = UN_FIND_STATUS;
#endif
	gprsAckprintf	=	0;
	gprsFlag =  COMM_SEND;
	gprsAckState	=	GPRS_ACK_OK;//GPRS_ACK_SET
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsSendBuffer((uint8_t *)"AT+CGPSINFO\r\n",13);
	status = gprsAckWait("+CGPSINFO:",10);
#else
	gprsSendBuffer((uint8_t *)"AT+CGNSINF\r\n",12);
	status = gprsAckWait("+CGNSINF:",9);
#endif
	gprsAckprintf	=	1;

	if(status  == COMM_BACK)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status 	= findStrRxGprs("+CGPSINFO:",10);
		status2 = findStrRxGprs("OK",2);if(status2<FIND_ERROR)clearBufPart(status2,2);
		if((status < FIND_ERROR)&&(status2 < FIND_ERROR)&&((status2-status)>40))
	#else
		status = findStrRxGprs("+CGNSINF: 1,1,",14);
		if(status < FIND_ERROR)
	#endif
		{			
		#if GPRS_DEBUG > 1
			printf("\n\r Location fix ok\n\r");
		#endif
			status = COMM_OK;
		}	
		else
		{
		#if GPRS_DEBUG > 1
			printf("\n\r Location  fix error\n\r");
		#endif
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)		
			status = findStrRxGprs((uint8_t *)"+CGNSINF: ",10);	
		#endif							
		 if(status < FIND_ERROR)
		 {
			 clearBufPart(status,10);
		 }
			status =  COMM_ERR;	
			clearBuf();              
		}
	}
	return status	;
}

gpsValue gpsBuf[MAX_GPS_BUF_SIZE];
DS1302_TIME gpsRtcTime;
#define MAX_GPS_VALUE_CHAR_SIZE       30

uint8_t findUnUseGpsBuf(void)
{
   uint8_t findi = 0;
   for(findi = 0x00;findi < MAX_GPS_BUF_SIZE;findi++)
   {
   if(gpsBuf[findi].useFlag == BUF_UNUSE_FLAG)
   {
   break;
   }
   
   }
   if(findi == MAX_GPS_BUF_SIZE)
   {
      findi = 0x00;
   }    
   return findi;
}


void clearGpsValue(gpsValue *value)
{
	value->gpsSource 		= NONE_SOURCE;
	value->useFlag 			= BUF_NONE_FLAG;	
	value->usedTime 		= 0;	
	value->locationMode	=	0;
	value->longitude.f	= 0.0;
	value->latitude.f 	= 0.0;
	value->altitude 		= 0.0;
	value->saveMode			=	GPS_SAVE_MODE;
	value->utcmSec			= 0;
	value->utcYear 			= 2000;
	value->utcMon 			= 0;
	value->utcDay 			= 0;
	value->utcHour 			= 0;
	value->utcMm 				= 0;
	value->utcSec				= 0;	
}
uint8_t getGpsValue(gpsValue *value)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t valueIndex = 0x00;
	uint8_t bufIndex = 0x00;
	uint8_t gpsi = 0;
	uint8_t gpsj = 0;
	
	char gpsValueBuf[MAX_GPS_VALUE_CHAR_SIZE];
	char utcTimeBuf[5];
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = findStrRxGprs("+CGPSINFO:",10);
#else	
	status = findStrRxGprs("+CGNSINF: 1,1,",14);
#endif
	if(status < FIND_ERROR)
	{
		clearBufPart(status,10);
		valueIndex = 	status;
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		for(gpsi = 0x01;gpsi < 8;gpsi ++)
	#else
		for(gpsi = 0x01;gpsi < 5;gpsi ++)
	#endif
		{
			bufIndex = 0x00;
			for(gpsj = 0x00;gpsj < MAX_GPS_VALUE_CHAR_SIZE;gpsj ++ )
			{
				if(GPRS_RX_Buffer[valueIndex]!=0x2C)
				{
					gpsValueBuf[bufIndex]  =	GPRS_RX_Buffer[valueIndex];				
				}
				else
				{
					gpsValueBuf[bufIndex] = '\0';
					valueIndex ++;					
					break;
				}
				valueIndex ++;
				valueIndex &= GPRS_RX_DATA_SIZE;
				bufIndex ++;
			}
		//	if(bufIndex == 0x00)return status;
			
			switch(gpsi)
			{
				case FIND_LONGITUDE_VALUE :
				{
					value->longitude.f = atof(gpsValueBuf);
					/*dispaly value*/
				#if GPRS_DEBUG > 0
					printf("\r\n longitude : %lf", value->longitude.f);
				#endif
					break;
				}
				case FIND_LATITUDE_VALUE :
				{
					value->latitude.f = atof(gpsValueBuf);					 
					/*dispaly value*/
				#if GPRS_DEBUG > 0
					printf("\r\n latitude : %lf", value->latitude.f);
				#endif
					break;
				}
			case FIND_ALTITUDE_VALUE :
			{
				value->altitude = atof(gpsValueBuf);					 
	 			/*dispaly value*/
				printf("\r\n altitude : %.2f" ,value->altitude);					 
				break;
			}			
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			case FIND_LAT_NS:
			{
				bufIndex	=	0;
				//value->latitude.f	= dm2dd(value->latitude.f);
				if(gpsValueBuf[bufIndex] == 'S')value->latitude.f *= -1.0;						
			#if GPRS_DEBUG > 0
				printf("\r\n latitude2: %lf", value->latitude.f);
			#endif
				break;
			}
			case FIND_LONG_EW:
			{
				bufIndex	=	0;
				//value->longitude.f = dm2dd(value->longitude.f);
				if(gpsValueBuf[bufIndex] == 'W')value->longitude.f *= -1.0;						
			#if GPRS_DEBUG > 0
				printf("\r\n longitude2: %lf", value->longitude.f);
			#endif
				break;
			}
			case FIND_UTC_DATE :
			{
				bufIndex = 0x00;
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//day
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}				 
				utcTimeBuf[2] ='\0';
				value->utcDay = atoi(utcTimeBuf);
				
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//month
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}				 
				utcTimeBuf[2] ='\0';
				value->utcMon = atoi(utcTimeBuf);
				
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//year
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}				 
				utcTimeBuf[2] ='\0';
				value->utcYear = atoi(utcTimeBuf)+2000;
			
				break;
			}
			case FIND_UTC_TIME :
			{
				bufIndex = 0x00;
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//hour
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}				 
				utcTimeBuf[2] ='\0';
				value->utcHour = atoi(utcTimeBuf);
				
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//minute
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}				 
				utcTimeBuf[2] ='\0';
				value->utcMm = atoi(utcTimeBuf);
				
				for(gpsj = 0x00;gpsj < 2; gpsj ++)//second
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}
				utcTimeBuf[2] ='\0';
				value->utcSec = atoi(utcTimeBuf);
				bufIndex ++;//'.'
				for(gpsj = 0x00;gpsj < 1; gpsj ++)//msecond
				{
					utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
				}
				utcTimeBuf[1] ='\0';
				value->utcmSec = atoi(utcTimeBuf);
				
			#if GPRS_DEBUG > 0
				printf("\r\n yymmddHHMMSS : %02d-%02d-%02d " ,value->utcYear,value->utcMon,value->utcDay);
				printf("%02d:%02d:%02d.%d" ,value->utcHour,value->utcMm,value->utcSec,value->utcmSec);
			#endif
				break;
			}
		#endif
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM808)
				case FIND_UTC_TIME :
				{
					bufIndex = 0x00;
					for(gpsj = 0x00;gpsj < 4; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[4] ='\0';
					value->utcYear = atoi(utcTimeBuf);
					/*dispaly value*/
					//	printf("utcyear : %d\n\r" ,gpsBuf[gpsBufIndex].utcYear);
					for(gpsj = 0x00;gpsj < 2; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcMon = atoi(utcTimeBuf);
					/*dispaly value*/
					//printf("utcmonth : %d\n\r" ,gpsBuf[0].utcMon);
					for(gpsj = 0x00;gpsj < 2; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcDay = atoi(utcTimeBuf);
					/*dispaly value*/
					//	printf("utcday : %d\n\r" ,gpsBuf[gpsBufIndex].utcDay);
					for(gpsj = 0x00;gpsj < 2; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcHour = atoi(utcTimeBuf);
					/*dispaly value*/
					//	printf("utchour : %d\n\r" ,gpsBuf[gpsBufIndex].utcHour);
					for(gpsj = 0x00;gpsj < 2; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcMm = atoi(utcTimeBuf);
					/*dispaly value*/
					//	printf("utcMin : %d\n\r" ,gpsBuf[gpsBufIndex].utcMm);
					for(gpsj = 0x00;gpsj < 2; gpsj ++)
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcSec = atoi(utcTimeBuf);
					
					for(gpsj = 0x00;gpsj < 2; gpsj ++)//msecond
					{
						utcTimeBuf[gpsj] = gpsValueBuf[bufIndex ++];
					}				 
					utcTimeBuf[2] ='\0';
					value->utcmSec = atoi(utcTimeBuf);
					/*dispaly value*/
				#if GPRS_DEBUG > 0
					printf(" yymmddHHMMSS : %d-%d-%d %d:%d:%d.%d\n\r" ,value->utcYear,value->utcMon,value->utcDay,value->utcHour,value->utcMm,value->utcSec,value->utcmSec);
				#endif
					//gpsValueStoreWithSensorData(sensorDataWriteAddr,gpsBuf[gpsBufIndex]);
					//gpsDataFlag = GPS_VALUE_OK ;
					break;
				}
			#endif
				default :
				{
					break;
				}
			}
		}	  		   
		
		if((value->latitude.f < 0.01)&&(value->latitude.f > -0.01))
		{
			if((value->longitude.f < 0.01)&&(value->longitude.f > -0.01))
			{
				status = COMM_ERR;				
			}
		}
		else if((value->utcMon == 0 )||(value->utcDay == 0))
		{
			status = COMM_ERR;
		}		
	}
	else
	{
		status = COMM_ERR;
	}	
	if(status != COMM_ERR)
	{
		value->gpsSource 	= GPS_SOURCE;
		value->useFlag 		= BUF_UNUSE_FLAG;	
		value->usedTime 	= (uint8_t)gprsPowerOnTime;		
		status = COMM_OK;
	}
	else
	{			
		clearGpsValue(value);
	}
	clearBuf();	
#if GPRS_DEBUG > 0
	printf("\r\n gprsStatus:%d",status);
#endif
	return status	;
}


int8_t utcTimeAdj(DS1302_TIME *timeOut,int8_t zoneOut,DS1302_TIME timeIn,int8_t zoneIn)
{
	__IO DS1302_TIME setTime;
	__IO int8_t		timeZone;
	__IO int8_t		hour;
	__IO uint16_t	year;

	setTime		= timeIn;	
	if(rtcCheck(setTime) == RTCERR)
	{
	#if GPRS_DEBUG > 0		
			printf("\r\n timeIn check error\r\n");	
	#endif
		return -1;
	}
	//	setTime	=	timeIn;
	setTime.year	= timeIn.year;
	setTime.month	= timeIn.month;
	setTime.date	= timeIn.date;
	setTime.hour	= timeIn.hour;
	setTime.min		= timeIn.min;
	setTime.sec		= timeIn.sec;
	setTime.day		= timeIn.day;
	timeZone 	= zoneOut-zoneIn;	
	if((timeZone>12)||(timeZone<-12))
	{
	#if GPRS_DEBUG > 0
		printf("\r\n timeZone error in:%d,out:%d\r\n",zoneIn,zoneOut);	
	#endif
		return -1;
	}
	year	=	setTime.year+2000;
	if(timeZone >0)
	{
		if(timeZone == 12)
		{
			setTime.hour += 24;
		}
		else
		{
			setTime.hour += timeZone;
		}
		if(setTime.hour >= 24)
		{
			setTime.hour -= 24;
			switch(setTime.month)
			{
				case 1:
				case 3:
				case 5:
				case 7:
				case 8:
				case 10:
				case 12:
				{
					if(setTime.date == 31)
					{
						setTime.date   = 1;
						setTime.month += 1;
						if(setTime.month >12)//����һ��
						{
							setTime.month = 1;
							setTime.year += 1;
							if(setTime.year>99)setTime.year = 0;
						}
					}
					else
					{
						setTime.date += 1;					
					}
					break;
				}
				case 4 :
				case 6 :	
				case 9 :
				case 11 :	
				{
					if(setTime.date == 30)
					{
						setTime.date 	 = 1;
						setTime.month += 1;										
					}
					else
					{
						setTime.date += 1;
					}
					break;
				}
				case 2 :
				{
					if(setTime.date == 29)
					{
						setTime.date 	 = 1;
						setTime.month += 1;											
					}else
					{
						if(setTime.date==28)
						{
							if(((year%4==0x00)&&(year%100!=0x00))||(year%400==0x00))	
							{
								setTime.date 	+= 1;
							}
							else
							{
								setTime.date 	 = 1;
								setTime.month += 1;	
							}							
						}else
						{
							setTime.date  += 1;
						}
						break;
					}
				}
			}
		}
	}
	else if(timeZone <0)
	{
		if(timeZone == -12)
		{
			hour = setTime.hour -24;
		}
		else
		{
			hour = setTime.hour + timeZone;	
		}
		if(hour<0)
		{
			setTime.hour =24+ hour;		
			switch(setTime.month)
			{
				case 1:
				{
					if(setTime.date	== 1)
					{
						setTime.date 	=	31;
						setTime.month = 12;
						if(setTime.year>0){setTime.year -= 1;}
						else{setTime.year=99;}
					}
					else
					{
						setTime.date -= 1;
					}
					break;
				}								
				case 5:
				case 7:
				case 10:
				case 12:
				{
					if(setTime.date	== 1)
					{
						setTime.date 	 = 30;
						setTime.month -= 1;
					}
					else
					{
						setTime.date -=1;
					}
					break;
				}
				case 2 :				  
				case 4 :
				case 6 :
				case 8:									  	
				case 9 :
				case 11 :	
				{
					if(setTime.date	== 1)
					{
						setTime.date 	 = 31;
						setTime.month -= 1;										
					}
					else
					{
						setTime.date -= 1;
					}
					break;
				}
				case  3 :
				{
					if(setTime.date== 1)
					{
						setTime.month -= 1;	
						if(((year%4==0x00)&&(year%100!=0x00))||(year%400==0x00))	
						{
							setTime.date 	=	29;
						}
						else
						{
							setTime.date	=	28;
						}											
					}
					else
					{
						setTime.date 	-= 1;
					}
					break;
				}
			}							   
		}
		else if(hour>=0)
		{
			setTime.hour = hour;			
		}
	}
	if(rtcCheck(setTime) == RTCERR)
	{
	#if GPRS_DEBUG > 0		
			printf("\r\n timeIn check error\r\n");	
	#endif
		return -1;
	}
//	*timeOut	=	setTime;
	timeOut->year		= setTime.year;
	timeOut->month	= setTime.month;
	timeOut->date		= setTime.date;
	timeOut->hour		= setTime.hour;
	timeOut->min		= setTime.min;
	timeOut->sec		= setTime.sec;
	timeOut->day		= setTime.day;	
	return 0;
}


uint8_t gpsRtcCal(gpsValue curGpsValue,int8_t timeZone,uint8_t setTimeFalg)
{
	DS1302_TIME setTime;
	__IO DS1302_TIME timeIn;	
	__IO int8_t	zone;//
	uint8_t	rtcStatus = UNCHANGE_FLAG;
//	uint8_t buf[10];
	zone	=	timeZone;
	timeIn.year 	= curGpsValue.utcYear -2000;
	timeIn.month 	= curGpsValue.utcMon;
	timeIn.date 	= curGpsValue.utcDay;//0x01;
	timeIn.hour 	= curGpsValue.utcHour;
	timeIn.min 		=	curGpsValue.utcMm;
	timeIn.sec 		= curGpsValue.utcSec;
	
	timeIn.day 		= 0x01;
	
	if(utcTimeAdj(&setTime,zone,timeIn,0)==0)
	{
		gpsRtcTime	=	setTime;		
		if(setTimeFalg == FUNCITONG_TURN_ON)	
		{					
			castWorkTime += castTimeAdj(beginTime);
			setTimeTask(setTime);
			beginTime	= getCurrterTime();	
			rtcStatus = CHANGE_FLAG;
//			serverConfig.perSetFlag	=	CHANGE_FLAG;
		}		
	}
	return rtcStatus;
}
double gpsValueAdj(double data)
{
	double intPart;
	double decimalPart;
	double result;
	result	=	data/100;
	intPart = (int64_t)result;
	decimalPart	= (result - intPart)*100;
	result	=	intPart+decimalPart/60;
	return result;
}
int8_t gpsTask(gpsValue *curGpsValue,httpConfig *rxResult ,int8_t timeZone)
{
	int8_t 	gpsTaskResult = -1;
	uint8_t gpsTaskState 	= GPRS_RUN;
	uint8_t gpsTry 				= 0x00;
	uint8_t gpsAllTry 		= 0x00;
#if GPRS_DEBUG > 0
	printf("\n\r gpsTask %d\n\r",gprsPowerOnTime);
#endif
	
	gprsState = GPRS_GPS_START;
	clearGpsValue(curGpsValue);
	while(gpsTaskState== GPRS_RUN)
	{
		switch(gprsState)
		{
			case GPRS_GPS_START :
			{
				if(gpsStart()== COMM_OK)
				{
					delay_GprsMs(5000);
					delay_GprsMs(5000);
					gpsTry ++;				 
					gprsState = GPRS_GPS_STATUS_QUERY;
				}
				else
				{
					gpsTry ++;
					if(gpsTry > 0x03)
					{ 
						gprsState = GPRS_GPS_POWER_OFF;
						gpsTry = 0x00;
					}
					else
					{
						delay_GprsMs(500);
					}
				}
				break;
			}
			case GPRS_GPS_STATUS_QUERY :
			{				
				if(getGpsStatus()==COMM_OK)
				{
					delay_GprsMs(20);
					gprsState = GPRS_GPS_VALUE;	 									
				}
				else
				{
					gpsTry ++;
					if(gpsTry > 10)//32_1000_2.7s 10_30s 12_35s 
					{
						gpsAllTry ++;
						gpsTry = 0x00;
					#if GPRS_DEBUG > 0
						printf("\n\r Location fix error %d\n\r",gprsPowerOnTime);//if(gpsAllTry>1)
					#endif
					}
					if(gpsAllTry > 0x03)//16_0x02 
					{
						gpsTaskResult = -2;
						gprsState = GPRS_GPS_POWER_OFF;
					}
					else
					{
						delay_GprsMs(1000);								
					}					
				}
				break;
			}
			case GPRS_GPS_VALUE :
			{
			#if GPRS_DEBUG > 0
				printf("\n\r Location fix ok %d,%d\n\r",gpsTry,gpsAllTry);
			#endif
				if(getGpsValue(curGpsValue)==COMM_OK)
				{
					gpsTaskResult = 1;
					gprsState = GPRS_GPS_POWER_OFF;
				#if(GPS_TIME_CTR>0)
					if(gpsRtcCal(*curGpsValue,timeZone,gpsRtcSetFlag) == CHANGE_FLAG)
					{	
						gpsInterValTimer			=	0;						
						gpsRtcSetFlag					=	FUNCITONG_TURN_OFF;
						rxResult->rtcSetFlag 	= CHANGE_FLAG;						
					}
				#endif
					gpsIntervalInit();
					if(gpsBuf[1].gpsSource != BUF_NONE_FLAG)gpsBuf[2]	=	gpsBuf[1];
				
					gpsBuf[1] = *curGpsValue;
			
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
					devLatitude		= gpsValueAdj(gpsBuf[1].latitude.f);
					devLongitude 	= gpsValueAdj(gpsBuf[1].longitude.f);
				#else
					devLatitude		= gpsBuf[1].latitude.f;
					devLongitude 	= gpsBuf[1].longitude.f;
				#endif
		
					//gps ���ݴ洢
					power331On();
					delay_GprsMs(10);
					SPI_Flash_WAKEUP();
					delay_GprsMs(5);				
					devGpsValueFlashStore(*curGpsValue ,&gpsDataWriteAddr);
					SPI_Flash_PowerDown();
					power331Off();

					if(configValue.gpsLocation >0x00)
					{
						configValue.gpsLocation --;
					}
					gpsTry = 0;
				}
				else
				{
					gpsTry ++;
					if(gpsTry > 0x01)
					{     
						gpsAllTry ++;
						gpsTry = 0x00;
						gprsState = GPRS_GPS_STATUS_QUERY;
						if(gpsAllTry > 0x03)
						{
							gpsTaskResult = -3;
							gprsState = GPRS_GPS_POWER_OFF;
						}
					}
					else
					{
						delay_GprsMs(1000);
					}
				}
				break;
			}
			case GPRS_GPS_POWER_OFF :
			{
				gpsSwitch(GPS_OFF);
	
				gprsState 		= GSM_MOUDLE_IDLE;
				gpsTaskState 	=	GPRS_SLEEP;				  
				break;
			}
			default :
			{
				gprsState = GPRS_GPS_POWER_OFF;
			//gpsTaskState =  GPRS_SLEEP;
				break;
			}
		}
	}
#if GPRS_DEBUG > 0
	printf("\n\r gpsTask end %d\n\r",gprsPowerOnTime);
#endif
	gpsBuf[0].utcmSec  = gprsPowerOnTime;
	gpsBuf[0].usedTime = (uint8_t)gprsPowerOnTime;
	return gpsTaskResult;
}
//__IO  uint32_t ptrOut = 0x00;
static uint16_t findStrRxGprs(char *buf,uint8_t bufLength)
{
	uint16_t findi = 0x00;
	uint16_t findj = 0x00;	
	uint32_t conuter = 0x00;
	uint32_t ptr    = 0x00;
	uint32_t ptrOut = 0x00;
	
	ptr			=	GPRS_RX_ptr_Store;
	ptrOut	=	GPRS_RX_ptr_Out;
	if(ptrOut <= ptr)
	{		
		conuter = ptr - ptrOut;
	}
	else
	{	
		conuter =GPRS_RX_DATA_SIZE+1 + ptr - ptrOut;
	}
	
	if(bufLength > conuter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < conuter;findi++)
	{
		if(GPRS_RX_Buffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(GPRS_RX_Buffer[ptr++] == buf[findj])
				{
					ptr = ptr&GPRS_RX_DATA_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
					break;
				}
			}
			if(findj ==bufLength)
			{
				return (ptr&GPRS_RX_DATA_SIZE);;
			} 
		}
		else
		{
			ptrOut ++;
			ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
		} 
	}
	return 	UN_FIND_STATUS;
}



uint16_t findStrRxGprsFromHead100(uint8_t *buf,uint8_t bufLength)
{
//   uint8_t bufLength = 0x00;
	uint16_t findi = 0x00;
	uint16_t findj = 0x00;
	uint32_t ptrOut = 0x00;
	uint32_t conuter = 0x00;
	uint32_t ptr    = 0x00;
	ptrOut = GPRS_RX_ptr_Out;
//   bufLength = strlen(buf);
//    if(GPRS_RX_ptr_Out < GPRS_RX_ptr_Store)
//    {
//    		conuter = GPRS_RX_ptr_Store - GPRS_RX_ptr_Out;
//    }
//    else
//    {
//    	   	conuter =GPRS_RX_DATA_SIZE+1 - GPRS_RX_ptr_Store + GPRS_RX_ptr_Out;

//    
//    }
   
	if(bufLength > conuter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < conuter;findi++)
	{
		if(GPRS_RX_Buffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(GPRS_RX_Buffer[ptr++] == buf[findj])
				{
					ptr = ptr&GPRS_RX_DATA_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
					break;
				}
			}
			if(findj == bufLength)
			{
				return ptr;
			} 
		}
		else
		{
			ptrOut ++;
			ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
		} 
	}
	return 	UN_FIND_STATUS;
}

static uint16_t findStrRxGprsFromAddress(uint8_t *buf,uint8_t bufLength,uint16_t address)
{
//   uint8_t bufLength = 0x00;
	uint16_t findi = 0x00;
	uint16_t findj = 0x00;
	uint32_t ptrOut = 0x00;
	uint32_t conuter = 0x00;
	uint32_t ptr    = 0x00;
	ptrOut = address;
//   bufLength = strlen(buf);
	if(GPRS_RX_ptr_Out < GPRS_RX_ptr_Store)
	{
		conuter = GPRS_RX_ptr_Store - address;
	}
	else
	{
		//conuter =GPRS_RX_DATA_SIZE+1 - GPRS_RX_ptr_Store + address;
		conuter =GPRS_RX_DATA_SIZE+1 + GPRS_RX_ptr_Store - address;
	}
	if(bufLength > conuter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < conuter;findi++)
	{
		if(GPRS_RX_Buffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(GPRS_RX_Buffer[ptr++] == buf[findj])
				{
					ptr = ptr&GPRS_RX_DATA_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
					break;
				}
			}
			if(findj ==bufLength)
			{
				return ptr;
			} 
		}
		else
		{
			ptrOut ++;
			ptrOut = ptrOut&GPRS_RX_DATA_SIZE;
		} 
	}
	return 	UN_FIND_STATUS;
}


//����������Ϣ
static int8_t determineConfig(void)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	
	findStatus = findStrRxGprs("config:1",8);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,8);
		rxResult = 0;
	}
		return rxResult;
}
//���ҹ̼��汾��Ϣ
static int8_t determineFireWare(void)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = 0x00;
	
	findStatus = findStrRxGprs("firmware:1",10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = 0x01;
	}	
	return rxResult;
}
static int8_t findConfigVersion(uint32_t *versionNum)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;

	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	uint32_t tempVersion = 0x00;

	// Ѱ�����ð汾��

	findStatus = findStrRxGprs("version:", 8);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,8);
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				// rxResult = 0x00;
				tempVersion = atoi(valueBuf); 
				if( *versionNum != tempVersion)
				{
					*versionNum = tempVersion;
					rxResult =  0;				
				}              
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
				if((valueBuf[rxi] > 0x3A)||(valueBuf[rxi]< 0x2F))
				{
					break;
				}
			} 
		}
	}
	return rxResult;
}
static int8_t findRtcTime(void)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t xyzBuf[30];
	DS1302_TIME setTime;        
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t  tempx = 0;
	//	float tempy  = 0.0;
	//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxh = 0x00;
	//	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	
	findStatus = findStrRxGprs("time:",5);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,5);
		rxResult = -2;
		for(rxi = 0x00;rxi <20;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				xyzBuf[rxi] = 0x2c;
				rxResult = 0x00;                
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
			} 
		}
		//ȡֵ
		if(rxResult  == 0x00)
		{
			for(rxh = 0x00;rxh < 6;rxh++)
			{
				if(xyzBuf[rxj]!= 0x2D)
				{
					valueBuf[rxh] = xyzBuf[rxj++];
					if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
					{  
						rxResult = -3;
						break;
					}
				}
				else
				{
					rxj++;
					valueBuf[rxh] = '\0';
					tempx = atoi(valueBuf); 
					tempx -= 2000;
					if(tempx<100)
					{                             
						setTime.year = tempx ;
					}
					else
					{
						rxResult = -4;
					}
					break;
				}
			}
			if((rxh <6)&&(rxResult == 0x00))
			{
				for(rxh = 0x00;rxh < 4;rxh++)
				{
					if(xyzBuf[rxj]!= 0x2D)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						rxj++;
						valueBuf[rxh] = '\0';
						if(valueBuf[0]==0x30)
						{
							tempx = valueBuf[1] -0x30;
						//setTime.month = valueBuf[1] -0x30;
						}else
						{
							tempx = atoi(valueBuf);  
						}
						if(tempx <13)
						{
							setTime.month = tempx;
						}
						else
						{
							rxResult = -3;
							break;
						}                               
						break;
					}
				}
			}
			//showDs1302Time(setTime);
			if((rxh <4)&&(rxResult == 0x00))
			{
				for(rxh = 0x00;rxh < 4;rxh++)
				{
					if(xyzBuf[rxj]!= 0x20)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						rxj++;
						valueBuf[rxh] = '\0';
						if(valueBuf[0]==0x30)
						{
							tempx = valueBuf[1] -0x30;
						//setTime.date = valueBuf[1] -0x30;
						}
						else
						{
							tempx =  atoi(valueBuf);  
						}
						if(tempx < 32)
						{
							setTime.date = tempx;
						} 
						else
						{
							rxResult = -4;
							break; 
						}                               
						break;
					}
				}
			}	
			//showDs1302Time(setTime);
			if((rxh <4)&&(rxResult == 0x00))
			{
				for(rxh = 0x00;rxh < 4;rxh++)
				{
					if(xyzBuf[rxj]!= 0x3A)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						rxj++;
						valueBuf[rxh] = '\0';
						if(valueBuf[0]==0x30)
						{
							tempx =   valueBuf[1] -0x30;
						//setTime.hour = valueBuf[1] -0x30;
						}
						else
						{
							tempx = atoi(valueBuf);  
						}	
						if(tempx<25)
						{
							setTime.hour = tempx;
						}
						else
						{
							rxResult = -4;                                     
						}                                    
						break;
					}
				}
			}		
			//showDs1302Time(setTime);
			if((rxh <4)&&(rxResult == 0x00))
			{
				for(rxh = 0x00;rxh < 4;rxh++)
				{
					if(xyzBuf[rxj]!= 0x3A)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						rxj++;
						valueBuf[rxh] = '\0';
						if(valueBuf[0]==0x30)
						{
							tempx = valueBuf[1] -0x30;
						}else
						{
							tempx = atoi(valueBuf);  
						}	
						if(tempx < 61)
						{
							setTime.min = tempx;
						}
						else
						{
							rxResult = -4;                  
						}                                
						//tempx = atoi(valueBuf);  
						//setTime.min= atoi(valueBuf);  
						//rxResult = 0x00;
						break;
					}
				}
			}	
			//showDs1302Time(setTime);
			if((rxh <4)&&(rxResult == 0x00))
			{
				for(rxh = 0x00;rxh < 4;rxh++)
				{
					if(xyzBuf[rxj]!= 0x2C)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						rxj++;
						valueBuf[rxh] = '\0';
						if(valueBuf[0]==0x30)
						{
							tempx = valueBuf[1] -0x30;
						}else
						{
							tempx = atoi(valueBuf);  
						}
						if(tempx < 61)
						{
							setTime.sec= tempx;
						}
						else
						{
							rxResult = -3;
						}                                  
						break;
					}
				}
			}	
		}
		//showDs1302Time(setTime);					 
		if(rxResult == 0x00)
		{			
			castWorkTime += castTimeAdj(beginTime);
			setTimeTask(setTime);
			beginTime	= getCurrterTime();
			showDs1302Time(rtcTime);
			serverConfig.rtcSetFlag = CHANGE_FLAG;
//			serverConfig.perSetFlag	=	CHANGE_FLAG;
		}
	}
	return rxResult;
}
static int8_t findConfigXyzKey(devConfig *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000002:",10);
	if(findStatus < FIND_ERROR)
	{		 
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
			{
				rxResult = 0;
				key->xyzSharkkey =  GPRS_RX_Buffer[findStatus];
				if(FUNCITONG_TURN_ON ==key->xyzSharkkey )
				{
					key->devWorkState = FUNCITONG_TURN_OFF;
				}
			}
		}                              
	}
	return rxResult;
}

static int8_t findConfiggpsKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000003:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
			{
				rxResult = 0;
				if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
				{ 
					*key = FUNCITONG_TURN_ON;
				}
				if(FUNCITONG_TURN_OFF == GPRS_RX_Buffer[findStatus])
				{ 
					*key = FUNCITONG_TURN_OFF;
				}
			}
		}                              
	}
	return rxResult;
}

static int8_t findXyzMaxValue(devConfig * xyzThresholdValue)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t xyzBuf[30];
	uint16_t findStatus = UN_FIND_STATUS;
	float tempx = 0.0;
//	float tempy  = 0.0;
//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000004:",10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				xyzBuf[rxi] = 0x2c;	
				rxResult = 0x00;                  
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
			} 
		}

		if(rxResult == 0x00) 
		{
			//ȡֵ
			for(rxj = 0x00;rxj < rxi;)
			{
				for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
				{
					if(xyzBuf[rxj]!= 0x2c)
					{
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						valueBuf[rxh] = '\0';
						rxj++;
						tempx = atof(valueBuf);  
						switch(rxt)
						{
							case 0 :
							{
								xyzThresholdValue->xValue = (uint16_t)(tempx/(0.0039)); 
								rxt ++; 
								break;
							}
							case 1 :
							{
								xyzThresholdValue->yValue = (uint16_t)(tempx/(0.0039));
								rxt ++;
								break;
							}
							case 2 :
							{
								xyzThresholdValue->zValue = (uint16_t)(tempx/(0.0039)); 
								rxt ++;
								break;
							}	
							default :
								break;						  						   
						}
						break;
					}
				}
				if(rxResult != 0x00)
				{
					break;
				}
			}
		}
	}
	return rxResult;
}

static int8_t findConfigDevPeriod(devConfig *period)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;

	findStatus = findStrRxGprs("CFG000008:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = -2;
		for(rxi = 0x00;rxi < 5;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				rxResult = 0x00;
				temp  = atoi(valueBuf);
				if((temp >4)&&(temp <601))
				{
				#if(MIN_PERIOD_CTR>0)
					if(temp == 51){temp = 1;}
					else if(temp == 52){temp = 2;}
					else if(temp == 53){temp = 3;}
					else if(temp == 54){temp = 4;}
				#endif
					temp *=60;
					period->devPeriod = temp;
					rxResult = 0;
					period->devPeriodType = NORMAL_PERIOD_TYPE;
				}                   
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
				if((valueBuf[rxi]> 0x3A)||(valueBuf[rxi]< 0x2F))
				{
					break;
				}
			} 
		}			
	}
	return rxResult;
}
static int8_t findConfigDevTransmissDensity(uint16_t *density)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	
	findStatus = findStrRxGprs("CFG000012:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				temp = atoi(valueBuf);
				if((temp>0)&&(temp < 11))
				{
					*density = temp;
					rxResult = 0;
				}                     
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
				if((valueBuf[rxi] > 0x3A)||(valueBuf[rxi] < 0x2F))
				{
					 break; 
				}
			} 
		}                       
	}
	return rxResult;
}
 int8_t findConfigSmsKey(devConfig *key)
{
      uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      Ѱ�����Ὺ��
	*/
	findStatus = findStrRxGprs("CFG000013:",10);
	if(findStatus < FIND_ERROR)
		{		clearBufPart(findStatus,10); 
		  rxResult = -2;
		      if((GPRS_RX_Buffer[findStatus+1]==0x0D)&&(GPRS_RX_Buffer[findStatus+2]==0x0A))//�ָ���
		  	{
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  		{
		  			  rxResult = 0;
                      key->devWorkState =  GPRS_RX_Buffer[findStatus];
			if(FUNCITONG_TURN_ON == key->devWorkState)
				{
                                 key->xyzSharkkey = FUNCITONG_TURN_OFF;
			}					  
		  		}

		  }                              
	}
       	   
         return rxResult;
}



 int8_t findConfigSmsPhone(phone *sendPhone)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000014:", 10);
	if(findStatus < FIND_ERROR)
		{
		clearBufPart(findStatus,10);
		  rxResult = -2;
                for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                                     valueBuf[rxi] = '\0';
						rxResult = 0;			 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                      if((valueBuf[rxi]> 0x3a)||(valueBuf[rxi] < 0x2F))
                      {
							rxResult = -4;	
                            break;
                      }
					 } 

				}

				
				//ȡֵ
				if(rxResult == 0x00)
					{
                      for(rxj= 0x00;rxj < rxi;rxj++)
                      {
                         sendPhone->phoneNumORG[rxj] = valueBuf[rxj];
					   }
						sendPhone->phonelength = rxj;
						sendPhone->phoneFlag = PHONE_SET; 
			//			phoneFormat(sendPhone); 
						rxResult = 0;
	
				}			
	}
       return rxResult;      

}



 int8_t findDevSmsPhone(phone *sendPhone)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000016:", 10);
	if(findStatus < FIND_ERROR)
		{
			clearBufPart(findStatus,10);
		  rxResult = -2;
                for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                                     valueBuf[rxi] = '\0';
						rxResult = 0x00;			 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                      if((valueBuf[rxi]> 0x3a)||(valueBuf[rxi] < 0x2F))
                      {
                            break;
                      }                        
					 } 

				}

				
				//ȡֵ
				if(rxResult == 0x00)
				{
                        for(rxj= 0x00;rxj < rxi;rxj++)
                        {
                           sendPhone->phoneNumORG[rxj] = valueBuf[rxj];
						}
						sendPhone->phonelength = rxj;
						sendPhone->phoneFlag = PHONE_SET; 
//						phoneFormat(sendPhone); 
				}			
	}
       return rxResult;      

}


static int8_t findWaterabcValue(soilabc *curAbcValue)
{
    char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t abcBuf[200];
	uint16_t findStatus = UN_FIND_STATUS;
    float tempx = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxt = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxg  = 0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000017:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
	   rxResult = -2;
       for(rxi = 0x00;rxi <200;rxi ++ )
       {
		  if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
		  {
                  abcBuf[rxi] = 0x2c;
                  rxResult = 0;              
						 break;
		 }
		 else
		 {
			abcBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
		  } 
		}
		//ȡֵ
        if(rxResult == 0x00)
        {
	     	for(rxj = 0x00;rxj < rxi ;)
	     	{
                for(rxt = 0x00;rxt < 3;rxt ++)
                {
                    for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
                    {
                        if(abcBuf[rxj]==0x2c)
                        {
                            valueBuf[rxh] = '\0';
							rxj++;
							//ȡֵ

							tempx = atof(valueBuf);
							
							    break;
							}else
							{
                                valueBuf[rxh] = abcBuf[rxj++];
                                if(valueBuf[rxh]!= 0x2E)
                                {
                                if((valueBuf[rxh] < 0x2C)||(valueBuf[rxh]> 0x3A))
                                {
                                rxResult = -3;
                                    break;
                                }
                            }
							}
							
					}
				   if((rxh < MAX_CONFIG_BUF_SIZE)&&(rxResult ==0x00))
				   {
					switch(rxt)
						{
                            case 0 :
							{
                               curAbcValue[rxg].a.f = tempx;
							   break;
							}
                            case 1 :
							{
                               curAbcValue[rxg].b1.f= tempx;
				               curAbcValue[rxg].b2.f = 1/tempx;
							   break;
							}
                            case 2 :
							{
                                curAbcValue[rxg].c.f= tempx;
							    rxg ++;
							   break;
							}									   
					     }
				      }
					   else
					   	{
                                       rxResult = -2;
					   }
				}
				if(rxResult != 0x00)
					{
                                                   break;
				}
                          
			}

        }
	}
       return rxResult;
}
static int8_t findFawValue(soilfaw*curFawValue)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t abcBuf[200];
	uint16_t findStatus = UN_FIND_STATUS;
	float tempx = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxt = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxg  = 0x00;
	int8_t rxResult = -1;
	
	findStatus = findStrRxGprs("CFG000018:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = -2;
		for(rxi = 0x00;rxi <200;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				abcBuf[rxi] = 0x2c;	
				rxResult = 0x00;                         
				break;
			}
			else
			{
				abcBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
			} 
		}
		//ȡֵ
		if(rxResult==0x00)
		{
			for(rxj = 0x00;rxj < rxi ;)
			{
				for(rxt = 0x00;rxt < 2;rxt ++)
				{
					for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
					{
						if(abcBuf[rxj]==0x2c)
						{
							valueBuf[rxh] = '\0';
							rxj++;
							//ȡֵ
							tempx = atof(valueBuf);
							break;
						}else
						{
							valueBuf[rxh] = abcBuf[rxj++];
							if(valueBuf[rxh]!= 0x2E)
							{
								if((valueBuf[rxh] >0x3A)||(valueBuf[rxh]<0x2C))
								{
									rxResult = -3;
									break;
								}
							}    
						}
					}
					if((rxh < MAX_CONFIG_BUF_SIZE)&&(rxResult ==0x00))
					{
						switch(rxt)
						{
							case 0 :
							{
								curFawValue[rxg].Fa.f = tempx;
								break;
							}
							case 1 :
							{
								curFawValue[rxg].Fw.f = tempx;
								rxg ++;
								break;
							}									   
						}
					}
					else
					{
						rxResult = -2;
					}
				}
				if(rxResult != 0x00)
				{
					break;
				}
			}         
		}
	}
	return rxResult;
}
static int8_t findfireWareId(devConfig *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	
	findStatus = findStrRxGprs("CFG000019:",10);
	if(findStatus < FIND_ERROR)
	{		 
		clearBufPart(findStatus,10);
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== 0x31 )||(GPRS_RX_Buffer[findStatus]== 0x32 ))
			{
				rxResult = 0;
				key->curfireWareId =	GPRS_RX_Buffer[findStatus];					  
			}
		}                              
	}       	   
	return rxResult;
}
static int8_t findFixTimePeriod(devConfig *fixTimeperiod)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	char fixBuf[MAX_CONFIG_BUF_SIZE]; 
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint16_t temp1 = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxt = 0x00;
	int8_t rxResult = -1;

	findStatus = findStrRxGprs("CFG000020:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[rxi] = 0x2c;	
				rxResult = 0x00;                    
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
			} 
		}
			//ȡֵ
		if(rxResult == 0x00)
		{
			rxResult = -3;
			for(rxj = 0x00;rxj <3;rxj ++)
			{
				if(valueBuf[rxj]!= 0x2c)
				{
					fixBuf[rxj] = valueBuf[rxj];
					if((fixBuf[rxj] <0x2F)||(fixBuf[rxj] >0x3A))
					{
						break;
					}
				}
				else
				{
					fixBuf[rxj]  = '\0';
					temp= atoi(fixBuf);
					if(temp<9)
					{
						rxResult = 0x00;   
					}
					break;
				}
			}
		}
		if(rxResult ==0x00)
		{
			rxj++;
			rxResult = -3;
			for(rxt= 0x00;rxt<3;rxt++)
			{
				if(valueBuf[rxj+rxt]!= 0x2c)
				{
					fixBuf[rxt] = valueBuf[rxj+rxt];
					if((fixBuf[rxt] > 0x3A)||(fixBuf[rxt] < 0x2F))
					{
						break;
					}
				}
				else
				{
					fixBuf[rxt]  = '\0';
					temp1= atoi(fixBuf);                      
					if((temp1< 0x3c)) 
					{    
						if(temp!= 0x00)
						{
							fixTimeperiod->devPeriodHour = temp;
							fixTimeperiod->devPeriodMin  = temp1;
							fixTimeperiod->devPeriodType = FIX_TIME_SEND_TYPE;
//							fixTimeperiod->transmissionDensity = 0x01;
							fixTimeperiod->devPeriod 			= temp*3600;	
							rxResult  = 0x00;    
						}else
						{
							if((temp1 >4)&&(temp1 < 60))
							{
							#if(MIN_PERIOD_CTR>0)
								if(temp1 == 51){temp1 = 1;}
								else if(temp1 == 52){temp1 = 2;}
								else if(temp1 == 53){temp1 = 3;}
								else if(temp1 == 54){temp1 = 4;}
							#endif
								fixTimeperiod->devPeriodType 	= NORMAL_PERIOD_TYPE;
								fixTimeperiod->devPeriod 			= temp1*60;								
								fixTimeperiod->devPeriodHour 	= 0;							
								fixTimeperiod->devPeriodMin 	= temp1;
								rxResult  = 0x00; 
							}
						}
					}
					break;
				}
			}
		}			
	}
	return rxResult;
}
static int8_t requestDevGpsKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000021:",10);
	if(findStatus < FIND_ERROR)
	{     
		clearBufPart(findStatus,10);//���ٲ����ֽ�	
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )
			{
				rxResult = 0;
				*key = 1;
		//	if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
		//	{ 
		//		*key ++;
		//	}
			}
		}                              
	}
	return rxResult;
}
 int8_t findAlarmSmsPhone(phone *sendPhone)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000022:", 10);
	if(findStatus < FIND_ERROR)
		{
		clearBufPart(findStatus,10);
		  rxResult = -2;
                for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                         rxResult = 0;                                
                         valueBuf[rxi] = '\0';
									 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                      if((valueBuf[rxi]> 0x3a)||(valueBuf[rxi] < 0x2F))
                      {
							//rxResult = -4;	
                            break;
                      }                         
					 } 

				}
				
				//ȡֵ
				if(rxResult == 0)
					{
                      for(rxj= 0x00;rxj < rxi;rxj++)
                      {
                         sendPhone->phoneNumORG[rxj] = valueBuf[rxj];
					   }
						sendPhone->phonelength = rxj;
						sendPhone->phoneFlag = PHONE_SET; 
//						phoneFormat(sendPhone); 
						
				}			
	}
       return rxResult;      

}

static int8_t findThresholdKey(uint8_t *key)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      Ѱ�����Ὺ��
	*/
	findStatus = findStrRxGprs("CFG000025:", 10);
	if(findStatus < FIND_ERROR)
		{    clearBufPart(findStatus,10);            
		  rxResult = -2;
		     if((GPRS_RX_Buffer[findStatus+1]==0x0D)&&(GPRS_RX_Buffer[findStatus+2]==0x0A))//�ָ���
		  	{
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  		{
		  			  rxResult = 0;
					  if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
                                      { 
                                      *key = FUNCITONG_TURN_ON;
					  	}
					  
					   if(FUNCITONG_TURN_OFF == GPRS_RX_Buffer[findStatus])
                                      { 
                                      *key = FUNCITONG_TURN_OFF;
					  	}
					  
		  		}
				else
				{
                                    rxResult = -3;
				}
		  }                              
	}
       	   
         return rxResult;
}


static int8_t findThresholdValue(uint8_t thresholdType,devConfig *thresholdValue)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t xyzBuf[30];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t tempx = 0.0;
	uint16_t thresholdTemp = 0x00;
	//	float tempy  = 0.0;
	//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	if(THRESHOLD_VALUE1== thresholdType)
	{
		findStatus = findStrRxGprs("CFG000027:",10);
	}
	if(THRESHOLD_VALUE2== thresholdType)
	{
		findStatus = findStrRxGprs("CFG000040:",10);
	}
	if(THRESHOLD_VALUE3== thresholdType)
	{
		findStatus = findStrRxGprs("CFG000041:",10);
	}

	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10); 
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				xyzBuf[rxi] = 0x2c;
				rxResult =  0x00;               
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
			} 
		}
		//ȡֵ
		if(rxResult == 0x00)
		{
			for(rxj = 0x00;rxj < rxi;)
			{
				for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
				{
					if(xyzBuf[rxj]!= 0x2c)
					{						   
						valueBuf[rxh] = xyzBuf[rxj++];
						if((valueBuf[rxh]< 0x2F)||(valueBuf[rxh] > 0x3A))
						{  
							rxResult = -3;
							break;
						}
					}
					else
					{
						valueBuf[rxh] = '\0';
						rxj++;
						switch(rxt)
						{
							case 0 :
							{
								thresholdTemp = atoi(valueBuf);
								rxt ++; 
								break;
							}
							case 1 :
							{
								tempx = atoi(valueBuf);
								rxt ++;
								break;
							}
							default :
							break;						  						   
						}
						break;
					}
				}
				if(rxResult != 0x00)
				{
					break;
				}
			}
		}
		if(0x00 == rxResult)
		{   
			if(THRESHOLD_VALUE1== thresholdType)
			{
				if((thresholdTemp <3)||(thresholdTemp >20))
				{
					rxResult = -4;
				}
				if((tempx <5)||(tempx >240))
				{
					rxResult = -5;
				}	
				if(0x00 == rxResult)
				{
					thresholdValue->moistureThreshold1 =thresholdTemp;
					thresholdValue->threshold1Period = tempx*60;
				}
			}
			if(THRESHOLD_VALUE2== thresholdType)
			{
				if((thresholdTemp <30)||(thresholdTemp >60))
				{
					rxResult = -4;
				}
				if((tempx <5)||(tempx >240))
				{
					rxResult = -5;
				}	
				if(0x00 == rxResult)
				{
					thresholdValue->moistureThreshold2 =thresholdTemp;
					thresholdValue->threshold2Period = tempx*60;
				}
			}
			if(THRESHOLD_VALUE3== thresholdType)
			{
				if((thresholdTemp <10)||(thresholdTemp >30))
				{
					rxResult = -4;
				}
				if((tempx <5)||(tempx >240))
				{
					rxResult = -5;
				}	
				if(0x00 == rxResult)
				{ 
					thresholdValue->moistureThreshold3=thresholdTemp;
					thresholdValue->threshold3Period = tempx*60;
				}
			}                          
		}
	}
	return rxResult;
}
#if(MQTT_USE_CTR>0)
static int8_t configHydrologyGsmKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	findStatus = findStrRxGprs("CFG000045:",10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);
		rxResult = -2;
		if((GPRS_RX_Buffer[findStatus+1]==0x0D)&&(GPRS_RX_Buffer[findStatus+2]==0x0A))//�ָ���
		{
			if(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )
			{
				rxResult = 0;
				*key = FUNCITONG_TURN_ON;
			}
			else
			{
				rxResult = 0;
				*key = FUNCITONG_TURN_OFF;
			}
		}                              
	}
	return rxResult;
}
#endif
#define MAX_DOMAINNAME_LENGTH   20

static int8_t findClockCalibrationKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//  uint8_t rxi;
	findStatus = findStrRxGprs("CFG000029:1",11);
	if(findStatus < FIND_ERROR)
	{		 
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[findStatus]==0x0D)&&(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			*key = FUNCITONG_TURN_ON;
		}                              
	}
	return rxResult;
}
static int8_t findTimeZone(int8_t *timeZone)
{
	uint16_t findStatus = UN_FIND_STATUS;
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	int8_t rxResult = -1;
	uint8_t rxi = 0x00;
	
	findStatus = findStrRxGprs("CFG000030:",10);
	if(findStatus < FIND_ERROR)
	{		 
		clearBufPart(findStatus,10);
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				rxResult = 0x00;
				valueBuf[rxi] = '\0';
				//ȡֵ
				if((rxi >0)&&(rxi < 4))
				{
					rxResult = atoi(valueBuf);
					if((rxResult>=-12)&&(rxResult<=12))
					{
						*timeZone= rxResult;
						rxResult = 0;					
					}
				} 		 
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
				if((valueBuf[rxi] != '-')&&((valueBuf[rxi] > '9')||(valueBuf[rxi] < '0')))
				{
					break;
				}
			} 
		}                      
	}
	return rxResult;
}
#if(DEV_APN_SET_CTR>0) //230508
static int8_t findApn(paramter *par)
{
	uint16_t findStatus = UN_FIND_STATUS;
	char valueBuf[40];
	int8_t rxResult = -1;
	uint8_t rxi = 0x00;
	uint8_t apnLength = 0;
	findStatus = findStrRxGprs("CFG000031:",10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;		
		for(rxi = 0x00;rxi <APN_LEN_MAX+1;rxi ++ )	//15 31_211215
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				rxResult 		= 0x00;             
				valueBuf[0] =	rxi;
				break;
			}
			else
			{
				valueBuf[rxi+1]= GPRS_RX_Buffer[findStatus+rxi];
			}
		}
		
		if((rxi <APN_LEN_MAX+2)&&(rxResult == 0x00))//15 32_211215
		{
			if((valueBuf[0]==3)&&(valueBuf[1]=='0')&&(valueBuf[2]=='0')&&(valueBuf[3]=='0'))
			{
				for(rxi = 0;rxi <APN_LEN_MAX+2;rxi++){par->apnBuf[rxi] = 0;}//apnBufSet[rxi] = 0; 230307
				
				rxResult = 0;
			}
			else if((valueBuf[0]==3)&&(valueBuf[1]=='0')&&(valueBuf[2]=='0')&&(valueBuf[3]=='1'))
			{				
				rxResult = 1;
			}
			else if((valueBuf[0]==3)&&(valueBuf[1]=='0')&&(valueBuf[2]=='0')&&(valueBuf[3]=='2'))
			{
				for(rxi = 0;rxi <APN_LEN_MAX+2;rxi++)par->apnBuf[rxi] = 0;
				rxResult = 2;
			}
			else
			{
				apnLength	=	0;
				for(rxi = 0;rxi <APN_LEN_MAX+2;rxi++)par->apnBuf[rxi] = 0;
				for(rxi = 1;rxi <=valueBuf[0];rxi++)
				{
					par->apnBuf[apnLength++] = valueBuf[rxi] ;
				}
				rxi	= sprintf(par->apnBuf+apnLength, "%s","\r\n");

				rxResult = 2;
			}						
		}
	}
	return rxResult;

}
#endif
int8_t findSimCardKey(uint8_t *key)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      Ѱ�����Ὺ��
	*/
	findStatus = findStrRxGprs("CFG000033:",10);
	if(findStatus < FIND_ERROR)
		{		clearBufPart(findStatus,10); 
		  rxResult = -2;
		      if((GPRS_RX_Buffer[findStatus+1]==0x0D)&&(GPRS_RX_Buffer[findStatus+2]==0x0A))//�ָ���
		  	{
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  		{
		  			  rxResult = 0;
                                 *key = GPRS_RX_Buffer[findStatus];
					  
		  		}
				else
					{
                       rxResult = -3;
				}
		  }                              
	}
       	   
         return rxResult;
}

static int8_t findRequestData(httpConfig *data)
{
	char xyzBuf[MAX_CONFIG_BUF_SIZE];      
	uint16_t findStatus = UN_FIND_STATUS;
	//       uint16_t  tempx = 0;
	//	float tempy  = 0.0;
	//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	//	uint8_t rxj = 0x00;
	//	uint8_t rxh = 0x00;
	//	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000034:",10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		for(rxi = 0x00;rxi <5;rxi ++ )//20191220 4>5
		{
			if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				xyzBuf[rxi] = 0x2c;
				data->requestData = atoi(xyzBuf);  	
				if(data->requestData<=1000)
				{
					data->requestDataType = REQUEST_DATA_BY_COUNTER;
					rxResult  = 0x00;	
				}
				else
				{
					data->requestData = 0x00;
					rxResult  = -4; 				
				}
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
				if(( xyzBuf[rxi]< 0x2D)||( xyzBuf[rxi] > 0x3A))
				{  
					rxResult = -3;
					break;
				}
			} 
		}				   					   	
	}
	return rxResult;
}

int8_t findRequestDataTime(httpConfig *data)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   uint8_t xyzBuf[30];     
	uint16_t findStatus = UN_FIND_STATUS;
//       uint16_t  tempx = 0;
//	float tempy  = 0.0;
//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxh = 0x00;
//	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000035:",10);
	if(findStatus < FIND_ERROR)
		{
		  rxResult = -2;
                for(rxi = 0x00;rxi <20;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                                     xyzBuf[rxi] = 0x2c;									 
						 break;
					 }
					 else
					 {
					  xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
					 } 
				}
				//ȡֵ
				   rxResult  = 0x00;
				   if(rxResult == 0x00)
				   	{
				   for(rxh = 0x00;rxh <3;rxh++)
				   {
				   
				           if(rxh!=2)
						{
						   
						   valueBuf[rxh] = xyzBuf[rxj++];
						   if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						   {  
						      rxResult = -3;
						      break;
						   }
						}
						else{
						        valueBuf[rxh] = '\0';
							 if(valueBuf[0]==0x30)
							 {
                                       data->requestDataTimeMon= valueBuf[1] -0x30;
                                                  }else
							 {
						              data->requestDataTimeMon = atoi(valueBuf);  
						    }
												  
						   break;
							}
				   	}
				   }
				   if(rxResult == 0x00)
				   	{
				   for(rxh = 0x00;rxh <3;rxh++)
				   {
				   
				           if(rxh!=2)
						{
						   
						   valueBuf[rxh] = xyzBuf[rxj++];
						   if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
						   {  
						      rxResult = -3;
						      break;
						   }
						}
						else{
						        valueBuf[rxh] = '\0';
							 if(valueBuf[0]==0x30)
							 {
                                                        data->requestDataTimeDay= valueBuf[1] -0x30;
                                                  }else
							 {
						              data->requestDataTimeDay= atoi(valueBuf);  
						    }
						              //data->requestDataType = REQUEST_DATA_BY_TIME;						  
						   break;
							}
				   	}
				   }
	}
       return rxResult;
}
static int8_t findInitMode(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000036:",10);
	if(findStatus < FIND_ERROR)
	{		 
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
			{
				rxResult = 0;
				*key = GPRS_RX_Buffer[findStatus];
			}
			else
			{
				rxResult = -3;
			}
		}                              
	}
	return rxResult;
}
int8_t findStaionName(paramter *par)
{
	uint16_t findStatus = UN_FIND_STATUS;
	char valueBuf[15];
	int8_t rxResult = -1;
	uint8_t rxi = 0x00;

	findStatus = findStrRxGprs("CFG000037:",10);
	if(findStatus < FIND_ERROR)
	{		
		clearBufPart(findStatus,10);
		rxResult = -2;

		for(rxi = 0x00;rxi <40;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[0] =rxi+1;
				rxResult = 0;			 
				break;
			}
			else
			{
				valueBuf[rxi+1]= GPRS_RX_Buffer[findStatus+rxi];
				//��ֵ�����ж�
				if((valueBuf[rxi+1]> 0x7E)||(valueBuf[rxi+1] < 0x21) )
				{
					rxResult = -3;
					break;
				}
			} 
		}
		if(rxResult == 0)
		{
			 for(rxi = 0x00;rxi < (valueBuf[0]+1);rxi++)
			 {
					par->stationAddr[rxi] = valueBuf[rxi];
			 }
		}               	                     
	}
	return rxResult;
}
int8_t findBackUpServer(serverAddr *bAddr)
{
       uint16_t findStatus = UN_FIND_STATUS;
       char valueBuf[50];
       char portBuf[10];
	   int8_t rxResult = -1;
	   uint8_t rxi = 0x00;
       uint8_t rxj = 0x00;
	   uint8_t rxLength = 0;
	   uint8_t  bLength =0;
	findStatus = findStrRxGprs("CFG000039:",10);	 
	if(findStatus < FIND_ERROR)
	{	clearBufPart(findStatus,10);
		  rxResult = -2;
		  
                for(rxi = 0x00;rxi <50;rxi ++ )
                {
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                        valueBuf[rxi] =0x2c;
						rxResult =0x00;			 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi]= GPRS_RX_Buffer[findStatus+rxi];
					 } 

				}
		if((rxi < 50)&&(rxResult == 0x00))
		{

               printf("server 1length %d\r\n",rxi);
			  rxLength = 0x01;
			  bLength=  sprintf(bAddr-> sendAddressBuf+rxLength, "%s", "AT+CIPSTART=\"TCP\",\"");
			  bLength++;
              for(rxLength = 0;rxLength < rxi;rxLength++)
              {
                  if(valueBuf[rxLength]!= 0x2C)
                  {
                         bAddr-> sendAddressBuf[bLength++] = valueBuf[rxLength];
                         bAddr-> baseAddressBuf[rxLength+1] = valueBuf[rxLength];  
				  }else
				  {    
                      bAddr-> baseAddressBuf[0] = rxLength;
                      break;
				  }                   
			  }
                printf("server 2length %d  s%d\r\n",bLength,rxLength);
				findStatus =  sprintf(bAddr-> sendAddressBuf+bLength, "%s", "\",\"");
					   //?????
				bLength+= findStatus ;
               rxLength++;
               printf("server 3length %d  s%d\r\n",bLength,rxLength);
			   for(;rxLength < rxi;rxLength++)
			   {
                   if(valueBuf[rxLength]!= 0x2c)
                   {
                       bAddr-> sendAddressBuf[bLength++] = valueBuf[rxLength];
                       portBuf[rxj++] = valueBuf[rxLength];
                       if((valueBuf[rxLength] > 0x3A)||(valueBuf[rxLength]<0x2F))
                       {
                           rxResult = -3;                        
                           break;
                       }
                                                							       				   
					}else
					{
                        portBuf[rxj++] =  '\0';
                        break;	
					}

			   }
               if(rxResult==0x00)
               {
                  printf("server 4length %d  s%d\r\n",bLength,rxLength);
				  findStatus =  sprintf(bAddr-> sendAddressBuf+bLength, "%s", "\"\r\n");
					   //?????
					    bLength+= findStatus ;
                  printf("server 4length %d  s%d\r\n",bLength,rxLength);
                  bAddr->sendPort = atoi(portBuf);
				  bAddr-> sendAddressBuf[0] = bLength;
				  bAddr->setFlag = TCP_SERVER_ADDRESS_SET_FLAG ;
                  bAddr->userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
               } 					   
		}
	}
	return rxResult;
}


int8_t configHydrologyKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;

	findStatus = findStrRxGprs("CFG000042:",10);
	if(findStatus < FIND_ERROR)
	{  clearBufPart(findStatus,10);              
		rxResult = -2;
		if((GPRS_RX_Buffer[findStatus+1]==0x0D)&&(GPRS_RX_Buffer[findStatus+2]==0x0A))//�ָ���
		{
			if(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )
			{
				rxResult = 0;
				*key = FUNCITONG_TURN_ON;
			#if(MQTT_USE_CTR>0)
				cigemParamInit();
				cigemUnsendDataClear();
			#endif
			}
			else
			{
				rxResult = 0;
				*key = FUNCITONG_TURN_OFF;
			}
		}                              
	}
	return rxResult;
}


static int8_t findDepValue(devDep* curDepValue)
{
    char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t abcBuf[100];
	uint16_t findStatus = UN_FIND_STATUS;
    float tempx = 0.0;
    uint16_t tempCheck = 0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
//	uint8_t rxt = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxg  = 0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000043:", 10);
	if(findStatus < FIND_ERROR)
    {clearBufPart(findStatus,10);
		rxResult = -2;
        for(rxi = 0x00;rxi <200;rxi ++ )
        {
	      if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
		  {
                 abcBuf[rxi] = 0x2c;
                 rxResult = 0x00;              
						 break;
		  }
		  else
		  {
				abcBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
		  } 
	  }
		//ȡֵ
      if(rxResult==0x00)
      {
	     	for(rxj = 0x00;rxj < rxi ;)
	        {
                 for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
                 {
                     if(abcBuf[rxj]==0x2c)
                     {
                          valueBuf[rxh] = '\0';
						  rxj++;
							//ȡֵ

						tempx = atof(valueBuf);
                        if(tempCheck >150)
                         {
                             rxResult = -2;
                         }
                         else
                         {
                            curDepValue[rxg++].dep.f = tempx;
                         }   							
				      break;
					 }else
					 {
                          valueBuf[rxh] = abcBuf[rxj++];
                          if((valueBuf[rxh] > 0x3A)||(valueBuf[rxh]< 0x2F))
                          {
                              rxResult = -3;
                             break;   
                          }
					 }							
				} 

				if(rxResult != 0x00)
			    {
                     break;
				}
                          
			}
        }

	}
       return rxResult;
}



 int8_t findDevCommPhone(uint8_t *sendPhone)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
    uint8_t rxh = 0x01;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000044:", 10);
	if(findStatus < FIND_ERROR)
		{
		clearBufPart(findStatus,10);
		  rxResult = -2;
                for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
					 {
                         rxResult = 0;                                
                         valueBuf[rxi] = '\0';
									 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                      if((valueBuf[rxi]> 0x3a)||(valueBuf[rxi] < 0x2F))
                      {
							//rxResult = -4;	
                            break;
                      }                         
					 } 

				}
				
				//ȡֵ
				if(rxResult == 0)
				{
                    
                      for(rxj= 0x00;rxj < rxi;rxj++)
                      {
                         sendPhone[rxh++] = valueBuf[rxj];
					   }
						sendPhone[0] = rxh;						
				}			
	}
       return rxResult;      

}


 int8_t findBeginTimeSet(uint8_t *bTime)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000108:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);	
		rxResult = -2;
		for(rxi = 0x00;rxi < 5;rxi ++ )
		{
			if((GPRS_RX_Buffer[findStatus+rxi]==0x0D)&&(GPRS_RX_Buffer[findStatus+rxi+1]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				rxResult = 0x00;
				temp  = atoi(valueBuf);
				if(temp <24)
				{
					*bTime	= temp;
				}                   
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
				if((valueBuf[rxi]> 0x3A)||(valueBuf[rxi]< 0x2F))
				{
					break;
				}
			} 
		}			
	}
	return rxResult;
}
static int8_t findConfigSendDensity(uint16_t *density)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000112:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
		{
			if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				temp = atoi(valueBuf);
				if((temp>0)&&(temp <= SEND_DENSITY_MAX))
				{
					*density = temp;
					rxResult = 0;
				}                     
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
				if((valueBuf[rxi] > 0x3A)||(valueBuf[rxi] < 0x2F))
				{
					break; 
				}
			} 
		}                       
	}
	return rxResult;
}

static int8_t findConfigUsartKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000116:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
			{
				rxResult = 0;
				if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
				{ 
					*key = FUNCITONG_TURN_ON;
				}
				if(FUNCITONG_TURN_OFF == GPRS_RX_Buffer[findStatus])
				{ 
					*key = FUNCITONG_TURN_OFF;
				}
			}
		}                              
	}
	return rxResult;
}
static int8_t findConfigInsentekKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000120:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== INSENTEK_KEY_ON)||(GPRS_RX_Buffer[findStatus]== INSENTEK_KEY_CFG)||(GPRS_RX_Buffer[findStatus]== INSENTEK_KEY_OFF ))
			{
				rxResult = 0;				
				*key = GPRS_RX_Buffer[findStatus];							
			}
		}                              
	}
	return rxResult;
}
#if(MQTT_USE_CTR>0)
void clearserveraddr(mqttServerAddr *addr)
{	
	addr->userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;
	addr->sendAddressBuf[0] = 0;addr->sendAddressBuf[1] = '0';addr->sendAddressBuf[0] = '\0';
	addr->protocol[0] = 0;addr->protocol[1] = '0';addr->protocol[0] = '\0';
	addr->endpoint[0] = 0;addr->endpoint[1] = '0';addr->endpoint[0] = '\0';
	addr->port		= 0;
	addr->deviceId[0] = 0;addr->deviceId[1] = '0';addr->deviceId[0] = '\0';
	addr->username[0] = 0;addr->username[1] = '0';addr->username[0] = '\0';
	addr->password[0] = 0;addr->password[1] = '0';addr->password[0] = '\0';	
}
static int8_t findMqttSeverAddr(mqttServerAddr *addr)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	uint8_t rxi=0,rxj=0;
	char buf[10];
	mqttServerAddr	addrtmp;
	uint8_t serverNumTmp=0;
	
	findStatus = findStrRxGprs("CFG000117:",10);
	if(findStatus < FIND_ERROR)
	{	
		rxResult = -2;
	#if GPRS_DEBUG > 0	
		receivePrintf("\r\n rxServerAddr:",GPRS_RX_Buffer,findStatus,1024);
	#endif	
		clearBufPart(findStatus,10);
		//CFG000117:1|tcp,ghiot.cigem.cn,1885,742189,21581164,6767377eb9e5ee17385e|
		//CFG000117:1|tcp,ghiot.cigem.cn,1885,754331,202007230001,7d05975fa9cb0d575980|
		if((GPRS_RX_Buffer[findStatus] >= '0')&&(GPRS_RX_Buffer[findStatus] <= MQTT_MAX_MULTI_CONNECTION_SIZE+'0'))
		{
			mqttServerNum	=	0;
			if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE] == '0')//num
			{
				for(rxi=0;rxi<MQTT_MAX_MULTI_CONNECTION_SIZE;rxi++)clearserveraddr(&addr[rxi]);	
				rxResult	=	0;				
			}
			else if(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE] == '|')
			{
				serverNumTmp	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE] - '0';
				findStatus++;//'|'
				for(rxi=0;rxi<serverNumTmp;rxi++)
				{
					for(rxj=1;rxj<=SERVER_PROTOCOL_SIZE;rxj++)//protocol
					{
						if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
						{
							findStatus++;
							addrtmp.protocol[rxj]	= '\0';
							addrtmp.protocol[0]	= rxj;
						#if GPRS_DEBUG > 0	
							printf("\r\n %d,protocol:%s,",rxi+1,&addrtmp.protocol[1]);
						#endif
							rxj= 250;break;
						}
						addrtmp.protocol[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
					}
					if(rxj==250)
					{
						for(rxj=1;rxj<=SERVER_ENDPOINT_SIXE;rxj++)//endpoint
						{
							if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
							{
								findStatus++;
								addrtmp.endpoint[rxj]	= '\0';
								addrtmp.endpoint[0]	= rxj;
							#if GPRS_DEBUG > 0	
								printf("endpoint:%s,",&addrtmp.endpoint[1]);
							#endif	
								rxj= 250;break;
							}
							addrtmp.endpoint[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
						}						
					}
					if(rxj==250)
					{
						for(rxj=0;rxj<SERVER_PORT_SIZE;rxj++)//port
						{
							if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
							{
								findStatus++;
								buf[rxj]	= '\0';
								addrtmp.port = atoi(buf);
							#if GPRS_DEBUG > 0	
								printf("port:%d,",addrtmp.port);
							#endif
								rxj= 250;break;
							}
							buf[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];				
						}
					}
					if(rxj==250)
					{
						for(rxj=1;rxj<=SERVER_DEVICEID_SIZE;rxj++)//deviceId
						{
							if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
							{
								findStatus++;
								addrtmp.deviceId[rxj]	= '\0';
								addrtmp.deviceId[0]	= rxj;
							#if GPRS_DEBUG > 0	
								printf("deviceId:%s,",&addrtmp.deviceId[1]);
							#endif
								rxj= 250;break;
							}
							addrtmp.deviceId[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
						}						
					}
					if(rxj==250)
					{
						for(rxj=1;rxj<=SERVER_USERNAME_SIZE;rxj++)//username
						{
							if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
							{
								findStatus++;
								addrtmp.username[rxj]	= '\0';
								addrtmp.username[0]	= rxj;
							#if GPRS_DEBUG > 0	
								printf("username:%s,",&addrtmp.username[1]);
							#endif
								rxj= 250;break;
							}
							addrtmp.username[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
						}						
					}
					if(rxj==250)
					{
						for(rxj=1;rxj<=SERVER_PASSWORD_SIZE;rxj++)//password
						{
							if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='|')
							{
								findStatus++;
								addrtmp.password[rxj]	= '\0';
								addrtmp.password[0]	= rxj;
							#if GPRS_DEBUG > 0	
								printf("password:%s,",&addrtmp.password[1]);
							#endif
								rxj= 250;break;
							}
							addrtmp.password[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
						}						
					}
					if(rxj==250)//id
					{
						if(rxi==0)
						{
							for(rxj=0;rxj<MQTT_MAX_MULTI_CONNECTION_SIZE;rxj++)clearserveraddr(&addr[rxj]);	
							devPar.stationMode = FUNCITONG_TURN_ON;
						}
						addrtmp.userFlag	=	TCP_SERVER_ADDRESS_USE_FLAG;
						addr[rxi] = addrtmp;
						mqttServerNum++;
					#if GPRS_DEBUG > 0	
						printf("serverNum:%d ",mqttServerNum);
					#endif							
						rxj = 250;
						rxResult	=	0;	
					}
					if(rxj!=250)break;
				}
			}
			if(mqttServerNum>0)
			{
			#if(SECOND_MQTT_ADDR>0)
				if(mqttServerNumSecond>0)
				{
					for(rxj = 0;rxj<SECOND_MULTI_CONNECTION_SIZE;rxj++)
					{
						if((rxj+mqttServerNumFirst)<MQTT_MAX_MULTI_CONNECTION_SIZE)
						{
							secondSendAddr[rxj] = curMqttSendAddr[mqttServerNumFirst+rxj].i;
						}
					}
				}
			#endif
				cigemParamInit();
			}
		}
	}
	return rxResult;
}


static int8_t findMqttSeverAddr2(void)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	
#if(SECOND_MQTT_ADDR>0)
	char buf[10];
	uint8_t rxi=0,rxj=0;
	uint8_t serverNumTmp=0;
	secondServerAddr addr[SECOND_MULTI_CONNECTION_SIZE];
#endif
	findStatus = findStrRxGprs("CFG000128:",10);

	if(findStatus < FIND_ERROR)
	{	
		rxResult = -2;
	#if GPRS_DEBUG > 0	
		receivePrintf("\r\n rxServerAddr:",GPRS_RX_Buffer,findStatus,1024);
	#endif	
		clearBufPart(findStatus,10);
		
		if((GPRS_RX_Buffer[findStatus] == '1')&&(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE] == ','))
		{//1,1 ��ȡ��һ·��Ϣ 1,2 ��ȡ�ڶ�·��Ϣ 1,3 ��ȡ������Ϣ
			if(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE] == '1')
			{
				alarmSource = READ_SOURCE_CHANNEL1;
				alarmLevel  = '1';
				rxResult	=	0;
			}
			else if(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE] == '2')
			{
				alarmSource = READ_SOURCE_CHANNEL2;
				alarmLevel  = '2';
				rxResult	=	0;
			}
			else if(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE] == '3')
			{
				alarmSource = READ_SOURCE_CHANNELAll;
				alarmLevel  = '3';
				rxResult	=	0;
			}
			else if(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE] == '4')
			{
				alarmSource = READ_SOURCE_STATUS;
				alarmLevel  = '4';
				rxResult	=	0;
			}
		}
	#if(SECOND_MQTT_ADDR>0)	
		else if((GPRS_RX_Buffer[findStatus] == '2')&&(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE] == ','))
		{//2,0| ����ڶ�·��Ϣ 2,1| ����һ·�ڶ�·��Ϣ 2,2| ������·�ڶ�·��Ϣ			
			findStatus += 2;
			//CFG000128:2,1|tcp,ghiot.cigem.cn,1885,742189,21581164,6767377eb9e5ee17385e|
			if((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE] >= '0')&&(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE] <= SECOND_MULTI_CONNECTION_SIZE+'0'))
			{
				if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE] == '0')//num
				{
					mqttServerNumSecond	=	0;
					for(rxj=0;rxj<SECOND_MULTI_CONNECTION_SIZE;rxj++)clearSecondAddrBuffer(&addr[rxj]);;
					rxResult	=	0;
				}
				else if(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE] == '|')
				{
					serverNumTmp	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE] - '0';
					findStatus++;//'|'		
				#if((SECOND_MQTT_ADDR<2)||(SECOND_MULTI_CONNECTION_SIZE<2))
					if(serverNumTmp>0)serverNumTmp = 1;
				#endif
					for(rxi=0;rxi<serverNumTmp;rxi++)
					{
						rxj = 0;
						if((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='t')&&(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]=='c'))
						{
							if((GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]=='p')&&(GPRS_RX_Buffer[(findStatus+3)&GPRS_RX_DATA_SIZE]==','))
							{
								findStatus += 4;
								rxj= 250;
							}
						}
						if(rxj==250)
						{
							for(rxj=1;rxj<=SERVER_ENDPOINT_SIXE;rxj++)//endpoint
							{
								if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
								{
									findStatus++;
									addr[rxi].endpoint[rxj]	= '\0';
									addr[rxi].endpoint[0]	= rxj;
								#if GPRS_DEBUG > 0	
									printf("endpoint:%s,",&addr[rxi].endpoint[1]);
								#endif	
									rxj= 250;break;
								}
								addr[rxi].endpoint[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
							}						
						}
						if(rxj==250)
						{
							for(rxj=0;rxj<SERVER_PORT_SIZE;rxj++)//port
							{
								if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
								{
									findStatus++;
									buf[rxj]	= '\0';
									addr[rxi].port = atoi(buf);
								#if GPRS_DEBUG > 0	
									printf("port:%d,",addr[rxi].port);
								#endif
									rxj= 250;break;
								}
								buf[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];				
							}
						}
						if(rxj==250)
						{
							for(rxj=1;rxj<=SERVER_DEVICEID_SIZE;rxj++)//deviceId
							{
								if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
								{
									findStatus++;
									addr[rxi].deviceId[rxj]	= '\0';
									addr[rxi].deviceId[0]	= rxj;
								#if GPRS_DEBUG > 0	
									printf("deviceId:%s,",&addr[rxi].deviceId[1]);
								#endif
									rxj= 250;break;
								}
								addr[rxi].deviceId[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
							}						
						}
						if(rxj==250)
						{
							for(rxj=1;rxj<=SERVER_USERNAME_SIZE;rxj++)//username
							{
								if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
								{
									findStatus++;
									addr[rxi].username[rxj]	= '\0';
									addr[rxi].username[0]	= rxj;
								#if GPRS_DEBUG > 0	
									printf("username:%s,",&addr[rxi].username[1]);
								#endif
									rxj= 250;break;
								}
								addr[rxi].username[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
							}						
						}
						if(rxj==250)
						{
							for(rxj=1;rxj<=SERVER_PASSWORD_SIZE;rxj++)//password
							{
								if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='|')
								{
									findStatus++;
									addr[rxi].password[rxj]	= '\0';
									addr[rxi].password[0]	= rxj;//
								#if GPRS_DEBUG > 0	
									printf("password:%s,",&addr[rxi].password[1]);
								#endif
									rxj= 250;break;
								}
								addr[rxi].password[rxj]	=	GPRS_RX_Buffer[(findStatus++)&GPRS_RX_DATA_SIZE];						
							}						
						}
						if(rxj==250)//
						{
							if(rxi==0)
							{
								clearSecondAddrBuffer(&addr[1]);								
								mqttServerNumSecond	=	0;
							}
							addr[rxi].userFlag	=	TCP_SERVER_ADDRESS_USE_FLAG;
//							addr[rxi] = addrtmp;
							mqttServerNumSecond++;
						#if GPRS_DEBUG > 0	
							printf("serverNumSecond:%d ",mqttServerNumSecond);
						#endif							
							rxj = 250;
							rxResult	=	0;							
						}
						if(rxj!=250)break;
					}
				}					
			}
			if(rxResult==0)
			{
				cigemServerAddrAdj(addr);
				alarmSource = READ_SOURCE_CHANNEL2;
				alarmLevel  = '2';
			}
		}
	#endif		
	}
	return rxResult;
}

static int8_t findConfigmqttdatatype(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000124:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus] >= MQTT_DATA_TYPE_CIGEM)&&(GPRS_RX_Buffer[findStatus]<=MQTT_DATA_TYPE_TREND))
			{
				rxResult = 0;				
				*key = GPRS_RX_Buffer[findStatus];										
			}
		}                              
	}
	return rxResult;
}
#endif
static int8_t findConfigQXKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000125:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus] == FUNCITONG_TURN_ON)||(GPRS_RX_Buffer[findStatus]==FUNCITONG_TURN_OFF))
			{
				rxResult = 0;				
				*key = GPRS_RX_Buffer[findStatus];										
			}
		}                              
	}
	return rxResult;
}
#if(ACCE_CTR>0)
static int8_t findConfigAccelerometerKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000126:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus] == FUNCITONG_TURN_ON)||(GPRS_RX_Buffer[findStatus]==FUNCITONG_TURN_OFF))
			{
				rxResult = 0;				
				*key = GPRS_RX_Buffer[findStatus];										
			}
		}                              
	}
	return rxResult;
}
#endif
static int8_t findConfigPowerSavingKey(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000127:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus] == FUNCITONG_TURN_ON)||(GPRS_RX_Buffer[findStatus]==FUNCITONG_TURN_OFF))
			{
				rxResult = 0;				
				*key = GPRS_RX_Buffer[findStatus];										
			}
		}                              
	}
	return rxResult;
}
#if(SK60_CTR>0)
	static int8_t findConfigSK60TestKey(uint8_t *key)
	{
		uint16_t findStatus = UN_FIND_STATUS;
		int8_t rxResult = -1;
	//	uint8_t rxi;
		findStatus = findStrRxGprs("CFG000121:", 10);
		if(findStatus < FIND_ERROR)
		{                
			clearBufPart(findStatus,10);//���ٲ����ֽ�
			rxResult = -2;
			if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON)||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
				{
					rxResult = 0;				
					*key = GPRS_RX_Buffer[findStatus];				
				}
			}                              
		}
		return rxResult;
	}
	static int8_t findConfigSK60Function(uint8_t *key)
	{
		uint16_t findStatus = UN_FIND_STATUS;
		int8_t rxResult = -1;
	//	uint8_t rxi;
		findStatus = findStrRxGprs("CFG000132:", 10);
		if(findStatus < FIND_ERROR)
		{                
			clearBufPart(findStatus,10);//���ٲ����ֽ�
			rxResult = -2;
			if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				if((GPRS_RX_Buffer[findStatus]>= FUNCITONG_SK60_OFF)&&(GPRS_RX_Buffer[findStatus]<= FUNCITONG_SK60_RAW))
				{
					rxResult = 0;				
					*key = GPRS_RX_Buffer[findStatus];				
				}
			}                              
		}
		return rxResult;
	}
	static int8_t findConfigSK60Offset(__IO float *value)
	{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	float temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
	Ѱ�����ð汾��
	*/
	findStatus = findStrRxGprs("CFG000133:", 10);
	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++)
		{
			if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				valueBuf[rxi] = '\0';
				temp = atof(valueBuf);
				if((temp>SK60_OFFSET_VALUE_MIN)&&(temp <= SK60_OFFSET_VALUE_MAX))
				{
					*value		= temp;
					rxResult 	= 0;
				}                     
				break;
			}			
			else
			{
				if(((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]>='0')&&(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]<='9'))||(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='.')||(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='-'))
				{
					valueBuf[rxi] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
				}
				else{break;}
			} 
		}                       
	}
	return rxResult;
}
#endif
#if(KXYL_CTR>0)
	static int8_t findConfigKXYLFunction(uint8_t *key)
	{
		uint16_t findStatus = UN_FIND_STATUS;
		int8_t rxResult = -1;
	//	uint8_t rxi;
		findStatus = findStrRxGprs("CFG000134:", 10);
		if(findStatus < FIND_ERROR)
		{                
			clearBufPart(findStatus,10);//���ٲ����ֽ�
			rxResult = -2;
			if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
			{
				if((GPRS_RX_Buffer[findStatus]>= FUNCITONG_TURN_OFF)&&(GPRS_RX_Buffer[findStatus]<= FUNCITONG_TURN_ON))
				{
					rxResult = 0;				
					*key = GPRS_RX_Buffer[findStatus];				
				}
			}                              
		}
		return rxResult;
	}
static int8_t findParamConfig(uint8_t type)	//221128
{
	char valueBuf[220];
	char fixBuf[MAX_CONFIG_BUF_SIZE]; 
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t  coefID	= 0;
	float 	tempx 	= 0.0;
	float 	tempa 	= 1.0;
	float 	tempb 	= 0.0;
	uint8_t rxi 		= 0x00;
	uint8_t rxj 		= 0x00;
	uint8_t rxt 		= 0x00;
	uint8_t rxh 		= 0x00;
	uint8_t num 		= 0x00;
	int8_t rxResult = -1;

	findStatus = findStrRxGprs("CFG000147:", 10);

	if(findStatus < FIND_ERROR)
	{
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
	#if GPRS_DEBUG > 1
		printf("\r\nrx:");
	#endif
		for(rxi = 0x00;rxi <220;rxi ++ )//MAX_CONFIG_BUF_SIZE
		{
		#if GPRS_DEBUG > 1
			printf("%c",GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]);
		#endif
			rxj++;
			if((GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ��� 0x0A ����'\n' 0xD�س�'\r'
			{		
				valueBuf[rxi] = 0x2c;//0x2c ','
				valueBuf[rxi+1] = '\0';
				rxResult = 0x00;
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[(findStatus+rxi)&GPRS_RX_DATA_SIZE];
				if(((valueBuf[rxi]<'0')||(valueBuf[rxi]>'9'))&&(valueBuf[rxi]!='.')&&(valueBuf[rxi]!='-')&&(valueBuf[rxi]!=','))break;
			} 
		}

		//ȡֵ ����
		if(rxResult == 0x00)
		{
			rxj = rxi;
			num = 0;
			for(rxi=0;rxi<220;rxi++)
			{
				if(valueBuf[rxi]== 0x2c) num++;
				if(valueBuf[rxi]== '\0')break;
			}
			num /= 3;
		#if GPRS_DEBUG > 1				
			printf("\r\nrxCoef %d %d:",rxi,num);
			devSendBuffer(valueBuf,rxj);
		#endif
			rxj = 0;
			
			for(rxh = 0x00;rxh <num;rxh ++)	
			{		
				tempa 	= 1.0;
				tempb 	= 0.0;
				rxResult = -3;	
				for(rxi = 0x00;rxi <4;rxi ++)//100,
				{
					if(valueBuf[rxj]== 0x2c)					
					{
						fixBuf[rxi]  = '\0';
						rxj++;
						coefID = atoi(fixBuf);									
						rxResult = 1;								
						break;
					}
					else
					{
						fixBuf[rxi] = valueBuf[rxj++];
						if((fixBuf[rxi] <0x2F)||(fixBuf[rxi] >0x3A))//0x30~0x39 '0'~'9'
						{
							break;
						}
					}				
				}
			#if GPRS_DEBUG > 0
				printf("\r\nparamCoef %d:%d,",rxResult,coefID);	
			#endif
				//ȡֵ ����ֵ
				if(rxResult == 1)
				{		
					for(rxt = 0x00;rxt < 2;rxt ++)
					{
						rxResult = -4;
						for(rxi = 0x00;rxi < MAX_CONFIG_BUF_SIZE;rxi++)
						{
							if(valueBuf[rxj]==0x2c)
							{
								fixBuf[rxi] = '\0';
								rxj++;
								//ȡֵ
								tempx = atof(fixBuf);							
								if(rxt == 0)			{tempa = tempx;	rxResult = 0;}
								else if(rxt == 1)	{tempb = tempx;	rxResult = 1;}						
								break;
							}
							else
							{
								fixBuf[rxi] = valueBuf[rxj++];
								if((fixBuf[rxi]!= 0x2D)&&(fixBuf[rxi]!= 0x2E))//0x2E . 0x2D -
								{
									if((fixBuf[rxi] < 0x2F)||(fixBuf[rxi]> 0x3A))
									{
										rxResult = -5;
										break;
									}
								}
							}
						}					
					}			
				}
			#if GPRS_DEBUG > 0	
					printf("%.4f,%.4f:%d ",tempa,tempb,rxResult);
			#endif
				if(rxResult == 1)
				{
					rxResult = -6;				
					if((coefID>0)&&(coefID<=KXYL_SENSOR_NUM))
					{
					#if GPRS_DEBUG > 0	
						printf("\r\nch %d:",coefID);
					#endif
						coefID -= 1;
						if((tempa>0.001)||(tempa<(-0.001)))
						{
							kxylParam[coefID][0] = tempa;
							kxylParam[coefID][1] = tempb;
						}
						else if((tempa==0)&&(tempb==0))
						{
							kxylParam[coefID][0] = KXYL_COF_A;
							kxylParam[coefID][1] = KXYL_COF_B;
						}
						kxylParamSave(coefID,kxylParam[coefID][0],kxylParam[coefID][1]);
					#if GPRS_DEBUG > 0	
						printf("%.4f,%.4f",kxylParam[coefID][0],kxylParam[coefID][1]);
					#endif
						rxResult = 0;
					}
								
				}
				if(rxResult != 0)break;
			}
		}
	}
	return rxResult;
}
#endif
#if defined(STM32L1XX_MDP)
static int8_t findConfigGPSValue(double *longitude,double *latitude)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
	char databuf[15];//-179.999999
	uint8_t rxi=0;
	double longitmp = 0;
	double latitmp = 0;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000122:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		for(rxi=0;rxi<12;rxi++)
		{		
			if(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==',')
			{
				findStatus++;
				databuf[rxi] = '\0';
				longitmp = atof(databuf);
				rxResult = 1;
				break;
			}
			else if(((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]>='0')&&(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]<='9'))||(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='.'))
			{
				databuf[rxi] = GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE];
			}
			else
			{
				if((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='-')&&(rxi==0)){databuf[rxi] = GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE];}
				else {break;}
			}
			findStatus++;
		}
		if(rxResult == 1)
		{
			rxResult = -3;
			for(rxi=0;rxi<12;rxi++)
			{		
				if((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0A))
				{
					findStatus++;
					databuf[rxi] = '\0';
					latitmp = atof(databuf);
					rxResult = 2;
					break;
				}
				else if(((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]>='0')&&(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]<='9'))||(GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='.'))
				{
					databuf[rxi] = GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE];
				}
				else
				{
					if((GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE]=='-')&&(rxi==0)){databuf[rxi] = GPRS_RX_Buffer[(findStatus)&GPRS_RX_DATA_SIZE];}
					else {break;}
				}
				findStatus++;
			}
		}
		if(rxResult == 2)
		{
			if((longitmp>=-180)&&(longitmp<=180))
			{
				if((latitmp>=-90)&&(latitmp<=90))
				{
					*longitude = longitmp;
					*latitude = latitmp;
					rxResult = 0;
				}
			}			
		}
	}
	return rxResult;
}
static int8_t findConfigNetworkMode(uint8_t *key)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000130:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if((GPRS_RX_Buffer[findStatus]== FUNCITONG_NETMODE_AUTO)||(GPRS_RX_Buffer[findStatus]== FUNCITONG_NETMODE_2G)||(GPRS_RX_Buffer[findStatus]== FUNCITONG_NETMODE_4G))
			{
				rxResult 	= 0;
				*key 			= GPRS_RX_Buffer[findStatus];				
			}
		}                              
	}
	return rxResult;
}
static int8_t findConfigDevReset(void)
{
	uint16_t findStatus = UN_FIND_STATUS;
	int8_t rxResult = -1;
//	uint8_t rxi;
	findStatus = findStrRxGprs("CFG000131:", 10);
	if(findStatus < FIND_ERROR)
	{                
		clearBufPart(findStatus,10);//���ٲ����ֽ�
		rxResult = -2;
		if((GPRS_RX_Buffer[(findStatus+1)&GPRS_RX_DATA_SIZE]==0x0D)&&(GPRS_RX_Buffer[(findStatus+2)&GPRS_RX_DATA_SIZE]==0x0A))//�ָ���
		{
			if(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON)
			{
				rxResult 	= 0;
				fireWareId = 1;
			}
			else if(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF)
			{
				rxResult   = 0;
				fireWareId = 0;
			}
		}                              
	}
	return rxResult;
}
#endif
static void findAuthCode(void)
{
	uint16_t findData = UN_FIND_STATUS;
	//        uint8_t timeStatus =0x00;
	findData = UN_FIND_STATUS;
	findData = findStrRxGprs("AuthCode:",9);
	if(findData < FIND_ERROR)
	{
		clearBufPart(findData,9);
		power331On();//�򿪴洢FLASH��Դ
		SPI_Flash_WAKEUP();
		devAuthCodeWrite(&GPRS_RX_Buffer[findData],AUTH_CODE_SIZE);
		SPI_Flash_PowerDown();
		power331Off();
		devRegAuthCodeSet();	
//		devSetValue.i = (*(__IO uint32_t*)DATA_EEPROM_SETINF_ADDR);
	#if GPRS_DEBUG > 0
		printf("\n\r find auth code :");//,devSetValue.bytes.value3
		usart2StoreBuffer(&authCodeBuf[1],authCodeBuf[0]);	
		printf("\n\r");

	#endif
	}
} 
#if(GPRS_MULTI_LINK>0)
static uint8_t rxGprsTcp(rxResponse *code)
{
	uint16_t status 		= UN_FIND_STATUS;
//	uint16_t ipdStatus 	= UN_FIND_STATUS;
	uint8_t buf[1536];//1024
	uint16_t curLength 	= 0x00;
	uint16_t rxi = 0;
	
	code->length.i 				= 0;
	code->responseCode.i 	= 99;
	status = waitRxGprsData(LINK_NUM_INSENTEK,&code->responseCode.i,STATUS_DONE);
	if(status != COMM_OK)	
	{
		if(readReceivelen(LINK_NUM_INSENTEK)>0)status = COMM_OK;
	}
	if(status == COMM_OK)
	{
		code->responseCode.i = 998;
		delay_GprsMs(50);
		status = findRxGprsData(buf,&curLength,LINK_NUM_INSENTEK);
	}
	if(status == COMM_OK)
	{
		code->responseCode.i = 999;
		for(rxi=0;rxi<curLength;rxi++)
		{
			if((buf[rxi]==0x04)&&(buf[rxi+1]==0x02))					
			{
				//��Ӧ����ȡ    
				code->length.bytes.value1 = buf[rxi+5];
				code->length.bytes.value2 = buf[rxi+4];
				code->length.bytes.value3 = buf[rxi+3];
				code->length.bytes.value4 = buf[rxi+2];

				code->responseCode.bytes.value1 = buf[rxi+9];
				code->responseCode.bytes.value2 = buf[rxi+8];
				code->responseCode.bytes.value3 = buf[rxi+7];
				code->responseCode.bytes.value4 = buf[rxi+6];

				break;
			}
		}	
	}	
	
#if GPRS_DEBUG > 0
	if(code->responseCode.i != RESPONSES_OK)
	{
		printf("\r\n length:%d,code:%d ",code->length.i,code->responseCode.i);
		
		receivePrintf("\r\n responses:",buf,0,curLength);
	}
#endif
	return status;
}
#else
static uint8_t rxGprsTcp(rxResponse *code)
{
	uint16_t status 		= UN_FIND_STATUS;
	uint16_t ipdStatus 	= UN_FIND_STATUS;
	uint16_t curLength 	= 0x00;
	uint16_t curRxAddress = 0x00;
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t rxi = 0x00,rxj=0;	 
	int8_t findResult = -1;
	
	code->responseCode.i = 99;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = findStrRxGprs("+IPD",4);
#else
	status = findStrRxGprs("+IPD,",5);
#endif
	if(status < FIND_ERROR)
	{    
		ipdStatus = status;
		status = COMM_OK;
	}
	else
	{
		gprsAckState	=	GPRS_ACK_SEND;
		status = gprsAckWait("+IPD",4);
		if(status  == COMM_TIMEOUT)
		{
			code->responseCode.i = HTTP_TIME_OUT ;	  
		}
		else if(status  == NO_COMM)
		{
			code->responseCode.i = HTTP_LINK_COLSE ;	  
		}	
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+IPD",4);
	#else
		status = findStrRxGprs("+IPD,",5);
	#endif
		if(status < FIND_ERROR)
		{
			ipdStatus = status;
			status 		= COMM_OK;
		}
	}
	if(status == COMM_OK)
	{
		//delay_GprsMs(200);
		code->responseCode.i = 998;
		curRxAddress = ipdStatus;
				
		//���ݽ�����ȫ�жϣ���ȡ���ݳ���
		for(rxi = 0x00;rxi <5;rxi ++)
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			if((GPRS_RX_Buffer[curRxAddress]== 0x0D)&&(GPRS_RX_Buffer[(curRxAddress+1)&GPRS_RX_DATA_SIZE] ==0x0A))
		#else
			if(GPRS_RX_Buffer[curRxAddress]== 0x3A)//':' 0x3A
		#endif
			{
				valueBuf[rxi] = '\0';
				curLength = atoi(valueBuf);
				
				for(rxj = 0x00;rxj < 200;rxj++)
				{				
					if(gprsRxcounterCal(curRxAddress) >= curLength)
					{				
						findResult 		= 0x00;
						break;
					}	
					delay_GprsMs(20);					
				}
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[curRxAddress];
				curRxAddress ++;
				curRxAddress &= GPRS_RX_DATA_SIZE;
			}
		}

		if(!findResult)
		{		
			clearBufPart(ipdStatus,4);//���ٲ����ֽ�			
			for(rxi = 0x00;rxi < MAX_CONFIG_BUF_SIZE;rxi++)
			{
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				if((GPRS_RX_Buffer[(ipdStatus+rxi)&GPRS_RX_DATA_SIZE]== 0x0D)&&(GPRS_RX_Buffer[(ipdStatus+rxi+1)&GPRS_RX_DATA_SIZE] ==0x0A))
			#else
				if(GPRS_RX_Buffer[(ipdStatus+rxi)&GPRS_RX_DATA_SIZE]==0x3A)	
			#endif
				{	
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)					
					ipdStatus ++;
				#endif
					if((GPRS_RX_Buffer[(ipdStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x04)&&(GPRS_RX_Buffer[(ipdStatus+rxi+2)&GPRS_RX_DATA_SIZE]==0x02))					
					{
						//��Ӧ����ȡ    
						code->length.bytes.value1 = GPRS_RX_Buffer[(ipdStatus+rxi+6)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value2 = GPRS_RX_Buffer[(ipdStatus+rxi+5)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value3 = GPRS_RX_Buffer[(ipdStatus+rxi+4)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value4 = GPRS_RX_Buffer[(ipdStatus+rxi+3)&GPRS_RX_DATA_SIZE];

						code->responseCode.bytes.value1 = GPRS_RX_Buffer[(ipdStatus+rxi+10)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value2 = GPRS_RX_Buffer[(ipdStatus+rxi+9)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value3 = GPRS_RX_Buffer[(ipdStatus+rxi+8)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value4 = GPRS_RX_Buffer[(ipdStatus+rxi+7)&GPRS_RX_DATA_SIZE];
						break;
					}
				}
			}
			if(rxi >MAX_CONFIG_BUF_SIZE)
			{
				code->responseCode.i = 999;
			}
		}
	}
	else
	{		
	#if GPRS_DEBUG > 0	
		if(GPRS_RXCounter>0)
		{
			printf("\r\n ts:\r\n");
			for(rxi=0;rxi<GPRS_RXCounter;rxi++)
			{
				devStoreBuffer(&GPRS_RX_Buffer[(GPRS_RX_ptr_Out+rxi)&GPRS_RX_DATA_SIZE],1);					
			}
			printf(" \r\n te\r\n");
		}
	#endif		
	}		
	return status;
}
#endif

#if(GPRS_MULTI_LINK>0)
static int8_t rxFireWare(uint8_t *buf,uint16_t *length,rxResponse *code,uint16_t readLength,uint32_t *fileLength)
{
	uint16_t status 		= UN_FIND_STATUS;
	uint16_t ipdStatus 	= UN_FIND_STATUS;
	int8_t 	 rxStatus		= -1;	
	value_t	 tempId ;
	uint16_t curLength 	= 0x00;
	uint16_t 	rxi = 0;
//	uint8_t		rxj = 0;
	code->responseCode.i 	= 99;
	code->length.i				=	0;
	status = waitRxGprsData(LINK_NUM_INSENTEK,&code->responseCode.i,STATUS_DONE);
	if(status != COMM_OK)	
	{
		if(readReceivelen(LINK_NUM_INSENTEK)>0)status = COMM_OK;
	}
	if(status == COMM_OK)
	{
		code->responseCode.i = 100;
		delay_GprsMs(100);//130
		status = findRxGprsData(buf,&curLength,LINK_NUM_INSENTEK);
	}	
	if(status == COMM_OK)
	{
		code->responseCode.i = 101;
		for(rxi=0;rxi<curLength;rxi++)
		{
			if((buf[rxi]==0x04)&&(buf[rxi+1]==0x02))					
			{
				//��Ӧ����ȡ    
				code->length.bytes.value1 = buf[rxi+5];
				code->length.bytes.value2 = buf[rxi+4];
				code->length.bytes.value3 = buf[rxi+3];
				code->length.bytes.value4 = buf[rxi+2];

				code->responseCode.bytes.value1 = buf[rxi+9];
				code->responseCode.bytes.value2 = buf[rxi+8];
				code->responseCode.bytes.value3 = buf[rxi+7];
				code->responseCode.bytes.value4 = buf[rxi+6];
				
				tempId.bytes.value1 = buf[rxi+13];
				tempId.bytes.value2 = buf[rxi+12];
				tempId.bytes.value3 = buf[rxi+11];
				tempId.bytes.value4 = buf[rxi+10];
				*fileLength 				= tempId.i; 
				ipdStatus 					= rxi +14;
				if(code->length.i>readLength)break;
			}
		}		
	}		
	if((code->responseCode.i == RESPONSES_OK)&&(code->length.i>readLength))//����ɹ� ��Ӧ�� 10000 code->length.i=readLength+4
	{
		for(rxi= 0x00;rxi< readLength;rxi++)	
		{
			buf[rxi]	= buf[ipdStatus++];
		}
		*length 	= rxi;
		rxStatus 	= COMM_OK;
	}
	else
	{
	#if GPRS_DEBUG > 0
		printf("\r\n code:%d,length:%d",code->responseCode.i,code->length.i);
		receivePrintf("\r\n responses:",buf,0,curLength);
	#endif
		if(code->responseCode.i == RESPONSES_OK)code->responseCode.i = 102;
	}
	return rxStatus;
}
#else
static int8_t rxFireWare(uint8_t *buf,uint16_t *length,rxResponse *code,uint16_t readLength,uint32_t *fileLength)
{
	char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t status 			= UN_FIND_STATUS;
	uint16_t curLength 		= 0x00;
	uint16_t curRxAddress = 0x00;
	uint16_t ipdStatus 		= UN_FIND_STATUS;	 
	int8_t 	 rxStatus			= -1;	
	uint16_t rxi 					= 0x00,rxj=0;	 
	int8_t 	 findResult		= -1;
	value_t	 tempId ;
	uint16_t	curCounter	=	0;
	code->responseCode.i = 99;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	status = findStrRxGprs("+IPD",4);
#else
	status = findStrRxGprs("+IPD,",5);
#endif
	if(status < FIND_ERROR)
	{    
		ipdStatus = status;
		status = COMM_OK;
	}
	else
	{
		gprsAckState	=	GPRS_ACK_SEND;
		status = gprsAckWait("+IPD",4);//20191214
		
		if(status  == COMM_TIMEOUT)
		{
			code->responseCode.i = HTTP_TIME_OUT ;
		}
		else if(status  == NO_COMM)
		{
			code->responseCode.i = HTTP_LINK_COLSE ;	  
		}	
		
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		status = findStrRxGprs("+IPD",4);
	#else
		status = findStrRxGprs("+IPD,",5);
	#endif
		if(status < FIND_ERROR)
		{
			ipdStatus = status;
			status = COMM_OK;
		}		  
	}
	if(status == COMM_OK)
	{
		//delay_GprsMs(200);//���պ�������
		code->responseCode.i 	= 100;		
		curRxAddress 					= ipdStatus;	

		//���ݽ�����ȫ�жϣ���ȡ���ݳ���
		for(rxi = 0x00;rxi <5;rxi ++)
		{
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
			if((GPRS_RX_Buffer[curRxAddress]== 0x0D)&&(GPRS_RX_Buffer[(curRxAddress+1)&GPRS_RX_DATA_SIZE] ==0x0A))
		#else
			if(GPRS_RX_Buffer[curRxAddress]== 0x3A)//':' 0x3A
		#endif
			{				
				valueBuf[rxi] = '\0';
				curLength = atoi(valueBuf);
				
				for(rxj = 0x00;rxj < 200;rxj++)//20191226
				{			
					curCounter = gprsRxcounterCal(curRxAddress);
					if(curCounter >= curLength)
					{				
						findResult 		= 0x00;
						break;
					}
					delay_GprsMs(20);
				}
				break;
			}
			else
			{
				valueBuf[rxi] = GPRS_RX_Buffer[curRxAddress++];				
				curRxAddress &=GPRS_RX_DATA_SIZE;
			}
		}
		
		if(!findResult)
		{ 
			clearBufPart(ipdStatus,4);//���ٽ����ֶ� +IPD			
			for(rxi = 0x00;rxi < MAX_CONFIG_BUF_SIZE;rxi++)
			{							
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				if((GPRS_RX_Buffer[(ipdStatus+rxi)&GPRS_RX_DATA_SIZE]== 0x0D)&&(GPRS_RX_Buffer[(ipdStatus+rxi+1)&GPRS_RX_DATA_SIZE] ==0x0A))				
			#else
				if(GPRS_RX_Buffer[(ipdStatus+rxi)&GPRS_RX_DATA_SIZE]==0x3A)				
			#endif
				{
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
					ipdStatus ++;
				#endif
					
					if((GPRS_RX_Buffer[(ipdStatus+rxi+1)&GPRS_RX_DATA_SIZE]==0x04)&&(GPRS_RX_Buffer[(ipdStatus+rxi+2)&GPRS_RX_DATA_SIZE]==0x02))
					{
						//��Ӧ����ȡ    
						code->length.bytes.value1 = GPRS_RX_Buffer[(ipdStatus+rxi+6)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value2 = GPRS_RX_Buffer[(ipdStatus+rxi+5)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value3 = GPRS_RX_Buffer[(ipdStatus+rxi+4)&GPRS_RX_DATA_SIZE];
						code->length.bytes.value4 = GPRS_RX_Buffer[(ipdStatus+rxi+3)&GPRS_RX_DATA_SIZE];

						code->responseCode.bytes.value1 = GPRS_RX_Buffer[(ipdStatus+rxi+10)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value2 = GPRS_RX_Buffer[(ipdStatus+rxi+9)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value3 = GPRS_RX_Buffer[(ipdStatus+rxi+8)&GPRS_RX_DATA_SIZE];
						code->responseCode.bytes.value4 = GPRS_RX_Buffer[(ipdStatus+rxi+7)&GPRS_RX_DATA_SIZE];

						tempId.bytes.value1 = GPRS_RX_Buffer[(ipdStatus+rxi+14)&GPRS_RX_DATA_SIZE];
						tempId.bytes.value2 = GPRS_RX_Buffer[(ipdStatus+rxi+13)&GPRS_RX_DATA_SIZE];
						tempId.bytes.value3 = GPRS_RX_Buffer[(ipdStatus+rxi+12)&GPRS_RX_DATA_SIZE];
						tempId.bytes.value4 = GPRS_RX_Buffer[(ipdStatus+rxi+11)&GPRS_RX_DATA_SIZE];
						*fileLength 				= tempId.i; 
						ipdStatus 					= (ipdStatus+rxi +15)&GPRS_RX_DATA_SIZE;//GPRS_RX_DATA_SIZE_250428
						//printf(" GSM[%d] %d,%d,%d\r\n",ipdStatus,tempId.i,code->length.i,code->responseCode.i);  
						break;
					}
				}
			}			
		}
		if(code->responseCode.i ==10000)//����ɹ� ��Ӧ�� 10000
		{
			for(rxi= 0x00;rxi< readLength;rxi++)	
			{
				buf[rxi]	= GPRS_RX_Buffer[ipdStatus++];
				ipdStatus &= GPRS_RX_DATA_SIZE;
			}
			*length = rxi;
		}	
	}
	rxStatus = status;
	return rxStatus;
}
#endif
void domainBufToAtLinkBuf(uint8_t *doMainBuf,char *atLinkBuf)
{
	uint8_t i =0x00;
	uint8_t j = 0x01;
	uint8_t t = 0x00;

	i  =  sprintf(atLinkBuf+j, "%s", "AT+CIPSTART=\"TCP\",\"");
	j +=i;
	for(t = 0x01;t < doMainBuf[0];t++)
	{
		atLinkBuf[j++] = doMainBuf[t];
	}
	i  =  sprintf(atLinkBuf+j, "%s", "\",\"80\"\r\n");	 
	j +=i;
	atLinkBuf[0] =j;
}

//�豸RTC�������ݰ�
uint16_t devRtcPacket(uint8_t *buf)
{
	uint8_t packetj = 0x00;
	uint16_t dLenth = 0x00;

	for(packetj = 0x01;packetj < DEV_SN_SIZE;packetj++)
	{
		buf[dLenth++] = devSnBuf[packetj];
	}												
	for(packetj = 0x01;packetj < (AUTH_CODE_SIZE);packetj++)
	{
		buf[dLenth++] = authCodeBuf[packetj];
	}		
	return dLenth;
}

//�豸�����������ݰ�
uint16_t devConfigPacket(uint32_t configVersion,uint8_t *buf)
{
	uint8_t packetj = 0x00;
	uint16_t dLenth = 0x00;
	value_t tempId ;
	
	for(packetj = 0x01;packetj < DEV_SN_SIZE;packetj++)
	{
		buf[dLenth++] = devSnBuf[packetj];
	}												
	for(packetj = 0x01;packetj < (AUTH_CODE_SIZE);packetj++)
	{
		buf[dLenth++] = authCodeBuf[packetj];
	}	
	tempId.i= configVersion;
	buf[dLenth++] = tempId.bytes.value4;
	buf[dLenth++] = tempId.bytes.value3;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;	
	
	for(packetj = 0x00;packetj < (8);packetj++)
	{
		buf[dLenth++] = firwMare[packetj];
	}	
	return dLenth;
}

//����ͷ������װ��
uint16_t sensorDataPacketHead(char *buf,uint16_t packetLength,uint8_t datyType)
{
	uint16_t dLenth = 0x00;
	//	uint16_t sLenth = 0x00;
	//		char dataTimeBuf[10];
	value_t tempId ;
	tempId.j = 0x0503;
	//Э��ͷ
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;
	//���ݳ���		   
	tempId.i= packetLength;
	buf[dLenth++] = tempId.bytes.value4;
	buf[dLenth++] = tempId.bytes.value3;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;		

	//������Э��version

	if(SENSOR_DATA ==datyType)
	{
		tempId.j= SENSOR_DATA_COLLECT;
		buf[dLenth++] = tempId.bytes.value2;
		buf[dLenth++] = tempId.bytes.value1;
	}	
	if(DEV_REG_DATA ==datyType)
	{
		tempId.j= DEVICE_REGISTER;
		buf[dLenth++] = tempId.bytes.value2;
		buf[dLenth++] = tempId.bytes.value1;	
	}				

	if(CONFIG_REQUEST_DATA ==datyType)
	{
		tempId.j= CONFIG_REQUEST;
		buf[dLenth++] = tempId.bytes.value2;
		buf[dLenth++] = tempId.bytes.value1;								
	}		
	if(DEV_RTC_CAL_DATA ==datyType)
	{
		tempId.j= SYNC_DEVICE_TIME;
		buf[dLenth++] = tempId.bytes.value2;
		buf[dLenth++] = tempId.bytes.value1;								
	}						 

	//version
	tempId.j = 0x0001;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;		
	//type  					 
	buf[dLenth++] = 0x01;	
 		     
	return dLenth;
}
uint16_t fireWareDataPacketHead(char *buf,fireWareStruct fireWareValue,uint32_t curGetAddr,uint32_t curGetEndAddr)
{
	uint8_t packetj = 0x00;
	uint16_t dLenth = 0x00;
	uint16_t bodyLength = 0x00;
	value_t tempId ;	
	//Э��ͷ
	tempId.j = 0x0503;	
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;
	//���ݳ���		   
	tempId.i= 0x0011;
	buf[dLenth++] = tempId.bytes.value4;
	buf[dLenth++] = tempId.bytes.value3;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;
	
	//������Э��version
	tempId.j= FIRWARE_DOWNLOAD;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;
	//version
	tempId.j = 0x0001;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;		
	//type  					 
	buf[dLenth++] = 0x01;	
	bodyLength = dLenth;
	for(packetj = 0x01;packetj < DEV_SN_SIZE;packetj++)
	{
		buf[dLenth++] = devSnBuf[packetj];
	}												
	for(packetj = 0x01;packetj < AUTH_CODE_SIZE;packetj++)
	{
		buf[dLenth++] = authCodeBuf[packetj];
	}	
	tempId.i= curGetAddr;
	buf[dLenth++] = tempId.bytes.value4;
	buf[dLenth++] = tempId.bytes.value3;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;	

	tempId.i= curGetEndAddr;
	buf[dLenth++] = tempId.bytes.value4;
	buf[dLenth++] = tempId.bytes.value3;
	buf[dLenth++] = tempId.bytes.value2;
	buf[dLenth++] = tempId.bytes.value1;		

	//���ݳ���װ��	   
	tempId.i= dLenth -bodyLength ;
	buf[2] = tempId.bytes.value4;
	buf[3] = tempId.bytes.value3;
	buf[4] = tempId.bytes.value2;
	buf[5] = tempId.bytes.value1;	
	return dLenth;
}

uint16_t devRegDataPacketHead(char *buf,uint8_t channel,uint16_t index,uint16_t packLength)
{
    uint8_t packeti = 0x00;
     uint8_t packetj = 0x00;
    uint16_t sLength = 0x00;
    uint16_t dataLength = 0x00;

//������
 	dataLength +=index;
   sLength =  sprintf(buf+dataLength, "%s","POST http://api.insentek.com/v1/device/create HTTP/1.1\r\n");
				  dataLength +=sLength;	 
   /*
   host
   
   */
   	 sLength =  sprintf(buf+dataLength,"%s","Host:");
				  dataLength +=sLength;	 
	   //host�ֶ�װ��
	   for(packeti = 0x01; packeti < tcpServerAddrBuf[channel].baseAddressBuf[0];packeti++)
	   {
	   	   	       buf[dataLength++] =  tcpServerAddrBuf[channel].baseAddressBuf[packeti];
	   }
	   //�˿ں�
   	 sLength =  sprintf(buf+dataLength,":%d\r\n",tcpServerAddrBuf[channel].sendPort);
				 dataLength +=sLength;

      /*ʱ���T*/
	//
  sLength =  sprintf(buf+dataLength, "Timestamp:%d",(rtcTime.year+2000));
				 dataLength +=sLength;	 

 sLength  =  sprintf(buf+dataLength, "-%02d",rtcTime.month);
				 dataLength +=sLength;	 

  sLength  =  sprintf(buf+dataLength, "-%02d ",rtcTime.date);
				 dataLength +=sLength;	 


  sLength =  sprintf(buf+dataLength, "%02d:",rtcTime.hour);
				 dataLength +=sLength;	 
  sLength  =  sprintf(buf+dataLength, "%02d:",rtcTime.min);
				 dataLength +=sLength;	 
  sLength  =  sprintf(buf+dataLength, "%02d\r\n",rtcTime.sec);
				 dataLength +=sLength;	 
   //�豸SN
  sLength  =  sprintf(buf+dataLength,"%s","SN:");
				  dataLength +=sLength;	 
			   for(packetj = 0x00;packetj < devSnBuf[0];packetj++)
			   {
			      buf[dataLength++] = devSnBuf[packetj+1];
			   }
  sLength  =  sprintf(buf+dataLength,"%s","\r\nContent-Length:");
				  dataLength +=sLength;	 
/**/			 
   	 sLength  =  sprintf(buf+dataLength,"%d\r\n",packLength);
				 dataLength +=sLength;	 
   	sLength  =  sprintf(buf+dataLength,"%s","Connection:close\r\n\r\n");
				 dataLength +=sLength;	 		     
			return dataLength;
}


uint16_t devRegDataPackeg(char *buf,uint16_t index,uint8_t channel)
{
	uint16_t mainFastLength = 0x00;
	
	power331On();//�򿪴洢FLASH��Դ

	SPI_Flash_WAKEUP();
	mainFastLength = devMainFastRead((uint8_t *)&buf[index]);
	if(mainFastLength>0)mainFastLength = mainFastLength - 1;
	printf("\n\r mainFastLength %d\n\r",(mainFastLength));
	SPI_Flash_PowerDown();
	power331Off();
	return (mainFastLength);
}


uint16_t gpsValueLoad(double longGitude,double latitude,uint8_t *buf,uint16_t index)
{
	value_t tempId ;
	uint16_t bLength = 0x00;
	dou2Buf temp;
	bLength += index;

	buf[bLength++] = 1;
	tempId.i = GPS_LONGITUDE_ID;
	buf[bLength++] = tempId.bytes.value4;
	buf[bLength++] = tempId.bytes.value3;
	buf[bLength++] = tempId.bytes.value2;
	buf[bLength++] = tempId.bytes.value1;

	temp.f = longGitude;
	buf[bLength++] = temp.bytes.value8;      
	buf[bLength++] = temp.bytes.value7; 
	buf[bLength++] = temp.bytes.value6; 
	buf[bLength++] = temp.bytes.value5; 
	buf[bLength++] = temp.bytes.value4; 
	buf[bLength++] = temp.bytes.value3; 
	buf[bLength++] = temp.bytes.value2; 
	buf[bLength++] = temp.bytes.value1; 
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%f\n\r",1,tempId.i,longGitude);
#endif
	buf[bLength++] = 1;
	tempId.i = GPS_LATITUDE_ID;
	buf[bLength++] = tempId.bytes.value4;
	buf[bLength++] = tempId.bytes.value3;
	buf[bLength++] = tempId.bytes.value2;
	buf[bLength++] = tempId.bytes.value1;

	temp.f = latitude;
	buf[bLength++] = temp.bytes.value8;      
	buf[bLength++] = temp.bytes.value7; 
	buf[bLength++] = temp.bytes.value6; 
	buf[bLength++] = temp.bytes.value5; 
	buf[bLength++] = temp.bytes.value4; 
	buf[bLength++] = temp.bytes.value3; 
	buf[bLength++] = temp.bytes.value2; 
	buf[bLength++] = temp.bytes.value1; 
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%f\n\r",1,tempId.i,latitude);
#endif
	return bLength;
}

uint16_t gprsAddGps(uint8_t valueFlag,uint8_t *buf,uint16_t index)
{
	uint8_t gpsIndex = 0x00;
	uint16_t bLength = 0x00;

	if(valueFlag == GPS_VALUE_OK)
	{
		gpsIndex = findUnUseGpsBuf();
		if(gpsIndex == 0x00)
		{
			gpsIndex = MAX_GPS_BUF_SIZE - 1; 
		}
		else
		{
			gpsIndex = gpsIndex -1;
		}
		bLength = gpsValueLoad(gpsBuf[gpsIndex].longitude.f,gpsBuf[gpsIndex].latitude.f,buf,index);
	}
	return bLength;
}


int8_t historyDataFind(httpConfig *http)
{
	uint8_t buf[SENSOR_DATA_STORE_SIZE+1];
	uint32_t readAddr = 0x00;
	uint16_t data = 0;
	uint16_t i = 0x00;
	uint8_t dataFlag = 0x00;
	int8_t findResult = -1;
	readAddr = http->requestDataReadAddress;
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
	for(i = 0x00;i < http->requestData;i++)
	{
		 //��ȡ����	 	   	
		SPI_FLASH_BufferRead(buf,readAddr,SENSOR_DATA_STORE_SIZE);
		//���������Ƿ��ǺϷ����ݰ�
		if((buf[0]==0x2c)&&(buf[1]==0x2c))
		{
			dataFlag = 0x00;
			data ++;	   
		}else
		{
			dataFlag ++;
		}
		if(dataFlag > 0x06)
		{
			break;
		}
		readAddr -=SENSOR_DATA_STORE_SIZE;
		if(SENSOR_DATA_BASE_ADDR >readAddr)
		{
			readAddr = SENSOR_DATA_END_ADDR - SENSOR_DATA_STORE_SIZE;
		}
	}
	SPI_Flash_PowerDown();
	power331Off();
	if(data!= 0x00)
	{
		findResult = 0x00;	
		http->requestData = data;
		http->requestDataReadAddress = readAddr;
		http->requestDataType = REQUEST_DATA_SEND;
	}

	return findResult;
}
void gprsSendSucFlagWrite(uint32_t lastSendAddr,uint32_t sendAddr,DS1302_TIME time)	//221101
{
	uint8_t buf[SENSOR_DATA_STORE_SIZE];
	uint16_t length;
	
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();delay_GprsMs(5);
	do
	{
		if(flashStoreDataRead(lastSendAddr,SENSOR_DATA_STORE_SIZE,buf,&length)==1)
		{
			sensorDataSendFlagWrite(lastSendAddr,time);
		}		
		if(lastSendAddr != sendAddr)
		{
			lastSendAddr += SENSOR_DATA_STORE_SIZE;
			if(lastSendAddr>=SENSOR_DATA_END_ADDR)lastSendAddr = SENSOR_DATA_BASE_ADDR;
		}
	}
	while(lastSendAddr != sendAddr);
	SPI_Flash_PowerDown();
	power331Off();
}
void gprsSendSucDataDeal(uint32_t *sendAddress,uint32_t lastSendAddress,uint16_t *packetCounter) //221101
{
	*sendAddress = lastSendAddress + SENSOR_DATA_STORE_SIZE;
	if(*sendAddress>=SENSOR_DATA_END_ADDR)*sendAddress = SENSOR_DATA_BASE_ADDR;
	*packetCounter = unSendPackNumAdj(*sendAddress);
}
void gprsSendSucFlagWrite1(uint32_t writeAddr,uint8_t packetCounter,DS1302_TIME time)
{
	uint8_t sendi = 0x00;
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
	for(sendi = 0x00;sendi < packetCounter;sendi++)
	{
		sensorDataSendFlagWrite(writeAddr,time);
		writeAddr += SENSOR_DATA_STORE_SIZE;
	}
	SPI_Flash_PowerDown();
	power331Off();
}

void gprsSendSucDataDeal1(__IO uint32_t *sendAddress,uint8_t sendPacketCounter,__IO uint16_t *packetCounter)
{
	if(*packetCounter > sendPacketCounter)
	{
		*sendAddress += (sendPacketCounter* SENSOR_DATA_STORE_SIZE);
//		*packetCounter -= sendPacketCounter;
		if(*sendAddress  	== sensorDataWriteAddr){*packetCounter = 0;}//210705
		else{*packetCounter	-= sendPacketCounter;}
	}
	else
	{
		*packetCounter = 0x00;
		*sendAddress  = sensorDataWriteAddr; 
	}
}

void sensorDataSendValueFlash(uint8_t *bufin)
{
	uint8_t nodei = 0x00;
	uint16_t dataInLength = 0x00;	
	DS1302_TIME time;
	value_t tempValue;
	b20_value_t  b20Temp;
	
//�ɼ�ʱ��
	time.year  = bufin[dataInLength++];
	time.month = bufin[dataInLength++];
	time.date  = bufin[dataInLength++];
	time.hour  = bufin[dataInLength++];
	time.min   = bufin[dataInLength++];
	time.sec   = bufin[dataInLength++];	
	sensorValue.time = time;
	//�豸�ܵĲɼ�����װ��
	tempValue.bytes.value1	= bufin[dataInLength++];
	tempValue.bytes.value2	= bufin[dataInLength++];
	tempValue.bytes.value3	= bufin[dataInLength++];
	tempValue.bytes.value4	= bufin[dataInLength++];
	sensorValue.countAll.i  = tempValue.i;	
	//�ɼ�����װ��	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	sensorValue.count.i 	 = tempValue.i;	
	//��ص�ѹ	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];	
	sensorValue.batV.f		 =	ROUND3(tempValue.f);
	//����ѹ����װ��
	tempValue.bytes.value1	= bufin[dataInLength++];
	tempValue.bytes.value2	= bufin[dataInLength++];
	tempValue.bytes.value3	= bufin[dataInLength++];
	tempValue.bytes.value4	= bufin[dataInLength++];
	sensorValue.inputV.f		=	ROUND3(tempValue.f);	
	//�ر��¶�װ��
	if(topSensorFlag == TOP_SENSOR_USE)
	{		
		b20Temp.value_t.bytes.valueH = bufin[dataInLength++];
		b20Temp.value_t.bytes.valueL = bufin[dataInLength++];
		b20Temp.rf = b20Temp.value_t.i*(0.0625);		
		sensorValue.t.f = b20Temp.rf;		
	}

	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{	
	//�¶�����װ��
		b20Temp.value_t.bytes. valueH = bufin[dataInLength++];
		b20Temp.value_t.bytes. valueL = bufin[dataInLength++];
		b20Temp.rf = b20Temp.value_t.i*(0.0625);
		sensorValue.soilt[nodei].f = b20Temp.rf;
	//��Ƶˮ������װ��
		tempValue.bytes.value1 = bufin[dataInLength++];
		tempValue.bytes.value2 = bufin[dataInLength++];
		tempValue.bytes.value3 = bufin[dataInLength++];
		tempValue.bytes.value4 = bufin[dataInLength++];
		sensorValue.fh[nodei].f = tempValue.f;

	//��Ƶ�η�����װ��
		if(eachDevNum == 0x02)
		{
			tempValue.bytes.value1 = bufin[dataInLength++];
			tempValue.bytes.value2 = bufin[dataInLength++];
			tempValue.bytes.value3 = bufin[dataInLength++];
			tempValue.bytes.value4 = bufin[dataInLength++];
			sensorValue.fl[nodei].f = tempValue.f;
		}
	}
	devWaterOutput(curWat,curFaw,curAbc);
	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
		sensorValue.wat[nodei].f = curWat[nodei].wat.f;
	}
}

uint16_t formatFlashSensorData(uint8_t *bufOut,uint16_t index)
{
	uint8_t datai = 0x00;
	uint8_t datat= 0x00;
	uint8_t nodei = 0x00;

	uint16_t dataOutLength= 0x00;
//	uint16_t dataInLength = 0x00;
	char dataTimeBuf[10];
	DS1302_TIME time;
	value_t tempId;
	value_t tempValue;
	value_t tempIdInt;
//	b20_value_t  b20Temp;
		
	/*ʱ���T*/
	time	=	sensorValue.time;
	datai =  sprintf(dataTimeBuf+datat, "%d",(time.year+2000));
	datat +=datai; 
	datai =  sprintf(dataTimeBuf+datat, "%02d",time.month);
	datat +=datai; 
	datai =  sprintf(dataTimeBuf+datat, "%02d",time.date);
	datat +=datai; 
	dataTimeBuf[datat] = '\0';
	tempId.i = atoi(dataTimeBuf);	
#if( SENSOR_DATA_FORMAT_KEY  >0)	
	printf("\n\r�ɼ�ʱ��%d\n\r",tempId.i);
#endif	
	dataOutLength	=	index;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;

	datat = 0x00;
	datai =  sprintf(dataTimeBuf+datat, "%d",time.hour);
	datat +=datai; 
	datai =  sprintf(dataTimeBuf+datat, "%02d",time.min);
	datat +=datai; 
	datai =  sprintf(dataTimeBuf+datat, "%02d",time.sec);
	datat +=datai;
	dataTimeBuf[datat] = '\0';
	tempId.i = atoi(dataTimeBuf);
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r�ɼ�ʱ��%d\n\r",tempId.i);
#endif
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	/*
	���ӳ�������
	*/
	bufOut[dataOutLength++] = 0x0f;
	bufOut[dataOutLength++] = 0x0f;
	bufOut[dataOutLength++] = 0x0f;
	bufOut[dataOutLength++] = 0x0f;	

	bufOut[dataOutLength++] = 0x01;
	tempId.i = TOTAL_COLLECT_COUNT_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	//bufOut[dataOutLength++] = 4;
	//�豸�ܵĲɼ�����װ��
	tempValue.i	=sensorValue.countAll.i;	
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++]  = tempValue.bytes.value1;			  
	#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%d\n\r",tempId.i,tempValue.i);      
	#endif
	//�ɼ�����װ��
	bufOut[dataOutLength] = 0x01;
	dataOutLength++;
	tempId.i = COLLECT_COUNT_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;

	tempValue.i	=	sensorValue.count.i;	
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;
#if( SENSOR_DATA_FORMAT_KEY  >0)			  
	printf("\n\r����ID:%d,     ֵ%d\n\r",tempId.i,tempValue.j);    
#endif

	bufOut[dataOutLength++] = 0x01;
	tempId.i = BATTERY_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f	=	sensorValue.batV.f;
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++]  = tempValue.bytes.value1;		
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%.3f\n\r",tempId.i,tempValue.f);   
#endif
	//�豸��ص�ѹ����װ��
	bufOut[dataOutLength++] = 0x01;
	tempId.i = OUTSIDE_VOLTAGE_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f	=	sensorValue.inputV.f;
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++]  = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%.3f\n\r",tempId.i,tempValue.f);   
#endif
	//�ر��¶�װ��
	if(topSensorFlag == TOP_SENSOR_USE)
	{
		bufOut[dataOutLength++] = 0x01;
		tempId.i = TEMPERATURE_ID;
		bufOut[dataOutLength++] = tempId.bytes.value4;
		bufOut[dataOutLength++] = tempId.bytes.value3;
		bufOut[dataOutLength++] = tempId.bytes.value2;
		bufOut[dataOutLength++] = tempId.bytes.value1;
		
		tempValue.f = sensorValue.t.f;
		bufOut[dataOutLength++]= tempValue.bytes.value4;
		bufOut[dataOutLength++]= tempValue.bytes.value3;
		bufOut[dataOutLength++]= tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;		
	#if( SENSOR_DATA_FORMAT_KEY  >0)		  
		printf("\n\r����ID:%d,     ֵ%.3f\n\r",tempId.i,tempValue.f);  
	#endif
	}

	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
	//�¶�����װ��
		bufOut[dataOutLength++] = nodei +2;
		tempId.i = TEMPERATURE_ID;
		bufOut[dataOutLength++] = tempId.bytes.value4;
		bufOut[dataOutLength++] = tempId.bytes.value3;
		bufOut[dataOutLength++] = tempId.bytes.value2;
		bufOut[dataOutLength++] = tempId.bytes.value1;	
	//�¶�����װ��	
		tempValue.f = sensorValue.soilt[nodei].f;
		bufOut[dataOutLength++] = tempValue.bytes.value4;
		bufOut[dataOutLength++] = tempValue.bytes.value3;
		bufOut[dataOutLength++] = tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;
	#if( SENSOR_DATA_FORMAT_KEY  >0)		
		printf("\n\r����ID:%d,     ֵ%.3f\n\r",tempId.i,tempValue.f);     
	#endif
	//��Ƶˮ������װ��	
		bufOut[dataOutLength++] = nodei +2;
		tempId.i = HIG_FRE_ID;
		bufOut[dataOutLength++] = tempId.bytes.value4;
		bufOut[dataOutLength++] = tempId.bytes.value3;
		bufOut[dataOutLength++] = tempId.bytes.value2;
		bufOut[dataOutLength++] = tempId.bytes.value1;

		tempIdInt.i = sensorValue.fh[nodei].f;
		bufOut[dataOutLength++] = tempIdInt.bytes.value4;
		bufOut[dataOutLength++] = tempIdInt.bytes.value3;
		bufOut[dataOutLength++] = tempIdInt.bytes.value2;
		bufOut[dataOutLength++] = tempIdInt.bytes.value1;		
	#if( SENSOR_DATA_FORMAT_KEY  >0)  	
		printf("\n\r����ID:%d,     ֵ%.d\n\r",tempId.i,tempIdInt.i);
	#endif
	//��Ƶ�η�����װ��
		if(eachDevNum == 0x02)
		{		  	  	    
			bufOut[dataOutLength++]= nodei +2;
			tempId.i = LOW_FRE_ID;
			bufOut[dataOutLength++] = tempId.bytes.value4;
			bufOut[dataOutLength++] = tempId.bytes.value3;
			bufOut[dataOutLength++] = tempId.bytes.value2;
			bufOut[dataOutLength++] = tempId.bytes.value1;
			
			tempIdInt.i = sensorValue.fl[nodei].f;
			bufOut[dataOutLength++] = tempIdInt.bytes.value4;
			bufOut[dataOutLength++] = tempIdInt.bytes.value3;
			bufOut[dataOutLength++] = tempIdInt.bytes.value2;
			bufOut[dataOutLength++] = tempIdInt.bytes.value1;		
		#if( SENSOR_DATA_FORMAT_KEY  >0)
			printf("\n\r����ID:%d,     ֵ%.d\n\r",tempId.i,tempIdInt.i);    
		#endif
		}	 
	}
//	dataOutLength = dataOutLength +index;
	return dataOutLength;			   
}
#define PI	3.14159265
//(sin��)2 = (sin��1)2 + (sin��2)2��//angle = �ȣ�//Z = 90�� - angle,//AZI = yaw
float qxAngleCaculate(double X,double Y)
{
	double sinx,siny,sinx2,siny2,sinsum,sinangle,angle;
//	printf("\r\n X:%f,Y:%f",X,Y);
	if((fabs(X)>360)||(fabs(Y)>360))return 0;
//	if(fabs(X)<0.1){angle=Y;}
//	else if(fabs(Y)<0.1){angle=X;}
//	else if(fabs(X)>89.9){angle=X;}
//	else if(fabs(Y)>89.9){angle=Y;}
//	else
	{
		sinx 	= sinf(X*PI/180.0);
		siny 	= sinf(Y*PI/180.0);
		sinx2 = sinx*sinx;		
		siny2 = siny*siny;
		sinsum 		= sinx2+siny2;		
		sinangle 	= sqrt(sinsum);
		if(sinangle >1)sinangle = 1;
		angle 		= asin(sinangle)*180/PI;
//		if((X<0)||(Y<0)) angle *=(-1.0);
//		if((fabs(X)>90)||(fabs(Y)>90)){angle = 90 - angle;}	
		if(fabs(angle)>360){angle=0;}
		else
		{
			while(angle>90){angle -= 90;}
			while(angle<-90){angle += 90;}
		}
	}
//	printf(" angle:%f",angle);
	return angle;
}
float qxZCaculate(float angle)
{
	float qxz=0;
	if(fabs(angle)>360)return 0;
	if(angle>0){qxz = 90-angle;}
	else{qxz = (90+angle)*(-1.0);}
	if(fabs(angle)>360){qxz = 0;}
	else
	{
		while(qxz>90)	{qxz -= 90;}
		while(qxz<-90){qxz += 90;}
	}
	return qxz;
}

//float qxAngleCaculate1(double X,double Y)
//{
//	double cosx,cosy,cosx2,cosy2,cossum,cosangle,angle;
//	if(X==0){angle=Y;}
//	else if(Y==0){angle=X;}
//	else
//	{
//		cosx = cosf(X*PI/180.0);
//		cosy = cosf(Y*PI/180.0);
//		cosx2 = cosx*cosx;		
//		cosy2 = cosy*cosy;
//		cossum = cosx2+cosy2-cosx2*cosy2;
//		if(cossum<0) return 0.0;
//		cosangle = cosx*cosy/sqrt(cossum);
//		angle = acos(cosangle)*180/PI;
//		if((fabs(X)>90)||(fabs(Y)>90)){angle = 180 - angle;}
//		if(angle>180){angle -= 180;}
//		else if(angle<-180){angle += 180;}
//	}
//	return angle;
//}
#if((GPRS_SLEEP_CTR>0)&&(MQTT_TEST_SCH_CTR>0))
	xyzQx_t xyzQxBuffer[XYZ_QX_BUFFER_SIZE+1];
	uint8_t xyzQxindex=0;
	xyzQx_t xyzQxCalculate(float xg,float yg,float zg)
	{
		xyzQx_t result;
		float xA,yA,zA;
		
	/* ����һ
		����� ���(Roll �� ) ������(Pitch �� ) 
		Pitch(��)	= arctan(Ax/sqrt(Ay^2+Az^2)) 
		Roll(��)		= arctan(Ay/sqrt(Ax^2+Az^2))	
		dipz	=	arctan(Az/sqrt(Ax^2+Ay^2))	
		
		float pitch,roll,dipz=0.0;
		xA	=	xg;
		yA	=	zg;
		zA	= yg;
		if((fabs(xA) <0.01)&&(fabs(yA) <0.01))
		{
			pitch 	= 0.0;
			roll		=	0.0;
		}
		else if((fabs(xA) <0.01)&&(fabs(zA) <0.01))
		{
			if(yg > 0){pitch = 90.0;roll	=	0.0;}
			else {pitch = -90.0;roll	=	0.0;}
		}
		else if((fabs(yA) <0.01)&&(fabs(zA) <0.01))
		{
			if(yg > 0){pitch = 0.0;roll	=	90.0;}
			else {pitch = 0.0;roll	=	-90.0;}
		}
		else
		{
		
			pitch	=	atanf(xA/sqrt(yA*yA+zA*zA))*180.0/PI;
			roll	=	atanf(yA/sqrt(xA*xA+zA*zA))*180.0/PI;		
		}
		dipz	=	atanf(zA/sqrt(xA*xA+yA*yA))*180.0/PI;
		result.roll		= roll;	
		result.pitch	= pitch;
		result.dipz		= dipz;
		
		if(debugStatus == DEBUG_STATUS_DONE)
		{
		printf("\r\n Pitch:%.3f,Roll:%.3f,dipz:%.3f \r\n",pitch,roll,dipz);
		}

	*/
	/* ������ */
		
		float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
		float fSinZ,fCosZ,ZAng = 0.0f;

		xA	=	-zg;
		yA	=	xg;
		zA	= yg;
		
		fNormAcc 	= sqrtf((xA*xA)+(yA*yA)+(zA*zA));
		fSinPitch = xA/fNormAcc;
		fCosPitch = sqrtf(1.0-(fSinPitch * fSinPitch));	
		fSinRoll 	= yA/fNormAcc;
		fCosRoll 	= sqrtf(1.0-(fSinRoll * fSinRoll));	
		
		fSinZ 	= zA/fNormAcc;
		fCosZ 	= sqrtf(1.0-(fSinZ * fSinZ));	
		
		PitchAng 	= (float)(acosf(fCosPitch)*180/PI);
		RollAng 	= (float)(acosf(fCosRoll)*180/PI);
		ZAng 			= (float)(acosf(fCosZ)*180/PI);
		if(xA<0){PitchAng	= -PitchAng;}
		if(yA<0){RollAng	= -RollAng;}	
		if(zA<0){ZAng	= -ZAng;}
		
		result.pitch	= PitchAng;
		result.roll		= RollAng;	
		result.dipz		= ZAng;	

	 if(debugStatus == DEBUG_STATUS_DONE)
	 {
		 printf("\r\n PitchAng:%.3f,RollAng:%.3f,ZAng:%.3f\r\n",PitchAng,RollAng,ZAng);
	 }
		return result;
	}
	xyzQx_t xyzQxValueCaculate(void)
	{
		uint8_t xyzi;
		xyzQx_t result;
		result.pitch 	= 0.0;
		result.roll 	= 0.0;
		result.dipz 	= 0.0;
		for(xyzi=0;xyzi<=XYZ_QX_BUFFER_SIZE;xyzi++)
		{
			result.pitch += xyzQxBuffer[xyzi].pitch;
			result.roll += xyzQxBuffer[xyzi].roll;
			result.dipz += xyzQxBuffer[xyzi].dipz;
		}
		result.pitch /= xyzi;
		result.roll /= xyzi;
		result.dipz /= xyzi;
		return result;
	}
	void xyzQxValueTask(void)
	{
		xyzQx_t value;
		value	=	xyzQxValueCaculate();
		sensorValue.dipX.f  = value.pitch;
		sensorValue.dipY.f  = value.roll;
//		sensorValue.angle.f = qxAngleCaculate(sensorValue.dipX.f,sensorValue.dipY.f);
		sensorValue.dipZ.f  = value.dipz;//qxZCaculate(sensorValue.angle.f);
		sensorValue.angle.f =	qxZCaculate(sensorValue.dipZ.f);	
	}
#endif
void qxDataSendValueFlash(uint8_t *bufin)
{
	uint16_t dataInLength = 0x00;
	uint16_t i = 0;

	for(i = 0; i < 20; i++)sensorValue.sign[i] 			= bufin[dataInLength++];//������б����ǩ��
	
	for(i = 0; i < 4; i++)sensorValue.qxt.buf[i] 		= bufin[dataInLength++];//������б�¶�
	little_end_to_big_end((uint8_t *)sensorValue.qxt.buf,4);	
	for(i = 0; i < 8; i++)sensorValue.dipX.buf[i] 	= bufin[dataInLength++];//�������X����
	little_end_to_big_end((uint8_t *)sensorValue.dipX.buf,8);	
	for(i = 0; i < 8; i++)sensorValue.valX.buf[i] 	= bufin[dataInLength++];// ���ط���X����
	little_end_to_big_end((uint8_t *)sensorValue.valX.buf,8);
	for(i = 0; i < 8; i++)sensorValue.origX.buf[i]	= bufin[dataInLength++];// ����ԭʼ����Y����
	little_end_to_big_end((uint8_t *)sensorValue.origX.buf,8);
	for(i = 0; i < 8; i++)sensorValue.dipY.buf[i] 	= bufin[dataInLength++];//�������Y����
	little_end_to_big_end((uint8_t *)sensorValue.dipY.buf,8);
	for(i = 0; i < 8; i++)sensorValue.valY.buf[i] 	= bufin[dataInLength++];// ���ط���Y����
	little_end_to_big_end((uint8_t *)sensorValue.valY.buf,8);
	for(i = 0; i < 8; i++)sensorValue.origY.buf[i] 	= bufin[dataInLength++];// ����ԭʼ����Yaw����
	little_end_to_big_end((uint8_t *)sensorValue.origY.buf,8);
	for(i = 0; i < 8; i++)sensorValue.dipYaw.buf[i]	= bufin[dataInLength++];//�������Yaw����
	little_end_to_big_end((uint8_t *)sensorValue.dipYaw.buf,8);
	for(i = 0; i < 8; i++)sensorValue.valYaw.buf[i] = bufin[dataInLength++];// ���ط���Yaw����
	little_end_to_big_end((uint8_t *)sensorValue.valYaw.buf,8);
	for(i = 0; i < 8; i++)sensorValue.origYaw.buf[i]= bufin[dataInLength++];// ����ԭʼ����Yaw���� 
	little_end_to_big_end((uint8_t *)sensorValue.origYaw.buf,8);
	
	if((sensorValue.qxt.f>85)||(sensorValue.qxt.f<-50))sensorValue.qxt.f=0;
	if(fabs(sensorValue.dipX.f)>90)sensorValue.dipX.f=0;
	if(fabs(sensorValue.dipY.f)>90)sensorValue.dipY.f=0;
	if(fabs(sensorValue.dipYaw.f)>360)sensorValue.dipYaw.f=0;
	
	if(fabs(sensorValue.valX.f)>10)sensorValue.valX.f=0;
	if(fabs(sensorValue.origX.f)>10)sensorValue.origX.f=0;
	if(fabs(sensorValue.valY.f)>10)sensorValue.valY.f=0;
	if(fabs(sensorValue.origY.f)>10)sensorValue.origY.f=0;
	if(fabs(sensorValue.valYaw.f)>10)sensorValue.valYaw.f=0;
	if(fabs(sensorValue.origYaw.f)>10)sensorValue.origYaw.f=0;
#if(QX_CTR>1)	
	if(tiltTypeCheck(&sensorValue.sign[0])==0)
	{
		dataInLength += 4;
		for(i = 0; i < 4; i++)sensorValue.dipZ.buf[i]		= bufin[dataInLength++];
		if(fabs(sensorValue.dipZ.f)>90)sensorValue.dipZ.f=0;
		for(i = 0; i < 4; i++)sensorValue.angle.buf[i]	= bufin[dataInLength++];
		if(fabs(sensorValue.angle.f)>90)sensorValue.angle.f=0;
	}else		
#endif
	{
		sensorValue.angle.f = qxAngleCaculate(sensorValue.dipX.f,sensorValue.dipY.f);
		if(sensorValue.ay.f<0) sensorValue.angle.f *= -1.0;//20210315
//		if(fabs(sensorValue.angle.f)>90)sensorValue.angle.f=0;
		sensorValue.dipZ.f = qxZCaculate(sensorValue.angle.f);
//		if(fabs(sensorValue.dipZ.f)>90)sensorValue.dipZ.f=0;
	}
}



uint16_t formatQxFlashData(uint8_t *bufOut,uint16_t index)
{
	uint16_t dataOutLength= 0x00;
	uint16_t i = 0;
	doubleV_t	doubleValue;
	value_t	tempValue;
	value_t tempId ;
	dataOutLength	=	index;
	bufOut[dataOutLength++]= 0x01;
	tempId.i = QX_DATA_SIGN;		//������б����ǩ��
	#if( SENSOR_DATA_FORMAT_KEY  >0)
	printf("\n\r����ID:%d\n\r",tempId.i);    
	#endif    
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1; 
	bufOut[dataOutLength++] = 20;
	
	for(i = 0; i < 20; i++)
	bufOut[dataOutLength++] = sensorValue.sign[i];

	bufOut[dataOutLength++] = 0x01;									//������б�¶�
	tempId.i = QX_TEMP;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	tempValue	=	sensorValue.qxt;
	little_end_to_big_end(tempValue.buf,4);	
	for(i = 0; i < 4; i++)
	bufOut[dataOutLength++] = tempValue.buf[i];

	bufOut[dataOutLength++] = 0x01;									//�������X����
	tempId.i = QX_DIP_X;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.dipX;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;								// ���ط���X����
	tempId.i = QX_VAL_X;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.valX;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;                // ����ԭʼ����X����
	tempId.i = QX_ORIG_X;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.origX;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;                // �������Y����
	tempId.i = QX_DIP_Y;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.dipY;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;               //���ط���Y����
	tempId.i = QX_VAL_Y;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.valY;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];
	
	bufOut[dataOutLength++] = 0x01;               //����ԭʼ����Y����
	tempId.i = QX_ORIG_Y;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.origY;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;               // ��������YAW����
	tempId.i = QX_DIP_YAW;                                         
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.dipYaw;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;              	// ���ط���YAW����
	tempId.i = QX_VAL_YAW;                                         
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.valYaw;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];

	bufOut[dataOutLength++] = 0x01;    	  				// ����ԭʼ����YAW����
	tempId.i = QX_ORIG_YAW;                                         
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	doubleValue	=	sensorValue.origYaw;
	little_end_to_big_end(doubleValue.buf,8);	
	for(i = 0; i < 8; i++)
	bufOut[dataOutLength++] = doubleValue.buf[i];
	
#if(QX_CTR>1)
	bufOut[dataOutLength++] = 0x01;								//�������Z����
	tempId.i = QX_DIP_Z;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	tempValue	=	sensorValue.dipZ;
	little_end_to_big_end(tempValue.buf,4);	
	for(i = 0; i < 4; i++)
	bufOut[dataOutLength++] = tempValue.buf[i];
	
	bufOut[dataOutLength++] = 0x01;								//�������angle����
	tempId.i = QX_DIP_ANGLE;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	tempValue	=	sensorValue.angle;
	little_end_to_big_end(tempValue.buf,4);	
	for(i = 0; i < 4; i++)
	bufOut[dataOutLength++] = tempValue.buf[i];
	
	bufOut[dataOutLength++] = 0x01;									//�������״̬����
	tempId.i = QX_STATUS;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	if(tiltTypeCheck(&sensorValue.sign[0])==0)
	{		
		bufOut[dataOutLength++] = 20;
		for(i=0;i<20;i++){bufOut[dataOutLength++] = sensorValue.sign[i];}
	}
	else
	{
		if((sensorValue.sign[0]==0)&&(sensorValue.sign[1]==0)&&(sensorValue.sign[2]==0)&&(sensorValue.sign[4]==0))
		{			
			bufOut[dataOutLength++] = 2;
			bufOut[dataOutLength++] = '-';
			bufOut[dataOutLength++] = '1';			
		}
		else
		{
			bufOut[dataOutLength++] = 1;
			bufOut[dataOutLength++] = '0';
		}
	}
#endif
//	dataOutLength += index;

	return dataOutLength;    
}
void xyzDataSendValueFlash(uint8_t *bufin)
{
	uint16_t dataInLength = 0x00;
	value_t tempValue;
	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	sensorValue.ax.f	=	tempValue.f;
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	sensorValue.ay.f	=	tempValue.f;
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	sensorValue.az.f	=	tempValue.f;	
}
uint16_t  formatXyzFlashData(uint8_t *bufOut,uint16_t index)
{
	uint16_t dataOutLength= 0x00;
	value_t tempId ;
	value_t tempValue;
	dataOutLength	=	index;
	bufOut[dataOutLength++]= 0x01;
	tempId.i = AX_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;

	tempValue.f = sensorValue.ax.f;
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++]= tempValue.bytes.value2;
	bufOut[dataOutLength++]= tempValue.bytes.value1;
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = AY_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.ay.f;
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++]= tempValue.bytes.value2;
	bufOut[dataOutLength++]= tempValue.bytes.value1;
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);   
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = AZ_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.az.f;
	bufOut[dataOutLength++]= tempValue.bytes.value4;
	bufOut[dataOutLength++]= tempValue.bytes.value3;
	bufOut[dataOutLength++]= tempValue.bytes.value2;
	bufOut[dataOutLength++]= tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);   
#endif

//	dataOutLength = dataOutLength +index;
	return dataOutLength;		
}

void gpsDataSendValueFlash(uint8_t *bufin)
{
	uint16_t dataInLength = 0x00;
	doubleV_t tempValue;

	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	tempValue.bytes.value5 = bufin[dataInLength++];
	tempValue.bytes.value6 = bufin[dataInLength++];
	tempValue.bytes.value7 = bufin[dataInLength++];
	tempValue.bytes.value8 = bufin[dataInLength++];
	sensorValue.longitude.f	=	tempValue.f;
	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	tempValue.bytes.value5 = bufin[dataInLength++];
	tempValue.bytes.value6 = bufin[dataInLength++];
	tempValue.bytes.value7 = bufin[dataInLength++];
	tempValue.bytes.value8 = bufin[dataInLength++];
	sensorValue.latitude.f	=	tempValue.f;
}

uint16_t  formatGpsFlashData(uint8_t *bufOut,uint16_t index)
{
	uint16_t dataOutLength= 0x00;

	value_t   tempId ;
	doubleV_t	tempValue;
	dataOutLength	=	index;
	bufOut[dataOutLength++]= 0x01;
	tempId.i = GPS_LONGITUDE_ID ;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;

	tempValue.f = sensorValue.longitude.f;
	bufOut[dataOutLength++] = tempValue.bytes.value8;
	bufOut[dataOutLength++] = tempValue.bytes.value7;
	bufOut[dataOutLength++] = tempValue.bytes.value6;
	bufOut[dataOutLength++] = tempValue.bytes.value5;			  
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = GPS_LATITUDE_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.latitude.f;
	bufOut[dataOutLength++] = tempValue.bytes.value8;
	bufOut[dataOutLength++] = tempValue.bytes.value7;
	bufOut[dataOutLength++] = tempValue.bytes.value6;
	bufOut[dataOutLength++] = tempValue.bytes.value5;			  
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif

//	dataOutLength = dataOutLength +index;
	return dataOutLength;		
}

uint16_t sensorDataReadToBuf(uint8_t type,uint32_t readAddress,uint8_t *readBuf,__IO uint16_t *readLen)
{
	uint16_t length = 0x00;
	int8_t readResult = -1;
	uint16_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t crc16 = 0;
	uint16_t bufCrc16 = 0;
	uint16_t crc16Length = 0;
	uint16_t readBufLength = 0x00;
	uint8_t buf[SENSOR_DATA_STORE_SIZE+1];
#if((SK60_CTR>0)||(KXYL_CTR>0))
	uint8_t		kxylNum=0;
#endif
//	uint16_t i;
	if((type == SEND_TYPE_USART)||(type ==SEND_TYPE_MQTT))
	{
		power331On();//�򿪴洢FLASH��Դ
		delay_GprsMs(15);
		SPI_Flash_WAKEUP();
		delay_GprsMs(5);
	}
	//��ȡ����������
	for(nodei=0;nodei<2;nodei++)
	{
		readResult = flashStoreDataRead(readAddress,SENSOR_DATA_STORE_SIZE,buf,&length);
		if(readResult ==1)
		{
			//crc16У����֤
			readResult = -1;  
			crc16Length = length -5;
			crc16 = CRC16_1(&buf[3],crc16Length);
			bufCrc16 =(((uint16_t)(buf[length-2]<<8))&0xFF00) + buf[length-1];
			//crc16�Ƚ�
			if(crc16 ==bufCrc16)
			{
				readResult =1;
				break;
			}
		}else{delay_ms(50);}//210705
	}
	if(readResult ==1)
	{	
	//usart2StoreBuffer(buf,SENSOR_DATA_STORE_SIZE);
		sensorDataSendValueFlash(&buf[4]);
		readResult = -2;
		*readLen = 0;
	 if(rtcCheck(sensorValue.time)==RTCOK)
	 {
		readResult = 1;
		if(type == SEND_TYPE_GPRS)
		{			
			printf("readBufLength_1: %d\r\n", readBufLength);
			readBufLength = formatFlashSensorData(readBuf,readBufLength);
			printf("readBufLength_2: %d\r\n", readBufLength);
		}
		//�����������
		crc = 0x00;//210708
		if((buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST]==0x2c)&&(buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+1]==0x2c))
		{
			for(nodei = 0;nodei < XYZ_VALUE_SENSOR_DATA_SIZE;nodei++)
			{
				crc += buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+3+nodei];
			}

			if(crc == buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+XYZ_VALUE_SENSOR_DATA_SIZE+3])
			{           
				xyzDataSendValueFlash(&buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST +9]);
				if(type == SEND_TYPE_GPRS)
				{
					readBufLength = formatXyzFlashData(readBuf,readBufLength); 
					printf("readBufLength_3: %d\r\n", readBufLength);
				}
			}
		}
		crc = 0x00;
		//�����������
		if((buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST]==0x2c)&&(buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+1]==0x2c))
		{
			for(nodei = 0;nodei < GPS_VALUE_SENSOR_DATA_SIZE;nodei++)
			{
				crc += buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+3+nodei];
			} 
			if(crc == buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+GPS_VALUE_SENSOR_DATA_SIZE+3])
			{
				gpsDataSendValueFlash(&buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST +9]);
				if(type == SEND_TYPE_GPRS)readBufLength = formatGpsFlashData(readBuf,readBufLength); 	   
			}
		}
	#if(QX_CTR>0)
		//�����б����
		if(devPar.qxkey != FUNCITONG_TURN_OFF)
		{
			crc = 0x00;
			if(type == SEND_TYPE_GPRS)printf("\r\nready find qx data %d ",readBufLength); 
			if((buf[QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST]==0x2c)&&(buf[QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST+1]==0x2c))
			{	
				if(type == SEND_TYPE_GPRS)printf("\r\nfind qx data %d ",readBufLength); 
				for(nodei = 0;nodei < QX_VALUE_SENSOR_DATA_SIZE;nodei++)
				{
					crc += buf[QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST+3+nodei];
				}

				if(crc == buf[QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST+QX_VALUE_SENSOR_DATA_SIZE+3])
				{
					qxDataSendValueFlash(&buf[QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST +9]);
					if(type == SEND_TYPE_GPRS)readBufLength = formatQxFlashData(readBuf,readBufLength); 
				}			
				if(type == SEND_TYPE_GPRS)
				{
					printf("\r\nend find qx data\r\n");
					printf("readBufLength_4: %d\r\n", readBufLength);
	//				/* ����޸�*/
	//				printf("�������������Ϊ: ");
	//				for(nodei = 0;nodei<readBufLength;nodei++)printf("%02x ", readBuf[nodei]);				
	//				printf("\r\n");
				}
			}
		}
	#endif
	#if((SK60_CTR>0)||(KXYL_CTR>0))
		if((devPar.kxylFuction == FUNCITONG_TURN_ON)||(devPar.sk60Fuction != FUNCITONG_SK60_OFF))
		{
			crc = 0x00;
			//�������������
			if((buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST]==0x2c)&&(buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+1]==0x2c))
			{				
				
			#if((KXYL_CTR>0)&&(KXYL_SENSOR_NUM>3))	//221128
				for(nodei = 0;nodei < EXTERNAL_STORE_DATA2_SIZE;nodei++)//26
				{
					crc += buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+3+nodei];
				} 
				if(crc == buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+EXTERNAL_STORE_DATA2_SIZE+3])//29
				{
					kxylNum = 4;
				}
				else
			#endif
				{
					crc = 0x00;//221227
					for(nodei = 0;nodei < EXTERNAL_STORE_DATA_SIZE;nodei++)//26
					{
						crc += buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+3+nodei];
					} 
					if(crc == buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+EXTERNAL_STORE_DATA_SIZE+3])//29
					{
						kxylNum = 2;
					}
				}
				printf("\r\nkxylNum��%d\r\n",kxylNum);
				if(kxylNum>0)
				{
					sk60DataSendValueFlash(&buf[SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+9],kxylNum);
					if(type == SEND_TYPE_GPRS)
					{
						readBufLength = formatSK60FlashData(readBuf,readBufLength); 	   
						printf("readBufLength_5: %d\r\n", readBufLength);
					}
				}
			}
		}
	#endif	
	#if(USART_USE_CTR>0)
		if(type ==SEND_TYPE_USART)
		{
		#if(PROTOCOL_CTR==PROTOCOL_Modbus)
			formatSensorFlashDataModbus(inputReg);
			readBufLength	=	modbusRegReadAck(READ_INPUTREG,INPUTREG_BUFF_SEND_START,INPUTREG_BUFF_SEND_LEN,readBuf,inputReg);
		#endif
		}
		else 
	#endif
		{		
			if(type ==SEND_TYPE_MQTT){readBufLength = 1;}		
		}
		*readLen = readBufLength;	
		if((readBufLength>1)&&(type == SEND_TYPE_GPRS))
		{
			printf("�������������%d:",readBufLength);
			for(nodei = 0;nodei<readBufLength;nodei++)printf("%02x ", readBuf[nodei]);				
			printf("\r\n");
		}
	 }
	}
	else
	{
		*readLen = 0;
	}
	if((type == SEND_TYPE_USART)||(type ==SEND_TYPE_MQTT))
	{
		SPI_Flash_PowerDown();
		power331Off();		
	}
	return readResult;
}

uint16_t gprsSensorDataLoad(uint8_t *buf,uint16_t index,b20_value_t  *dsb20Buf,soilwat *sensorDataBuf)
{
    uint8_t nodei = 0x00;
//	uint8_t loadj = 0x00;
	uint16_t bLength = 0x00;
	value_t tempId ;
	value_t tempId2 ;
	value_t tempId3 ;
	value_t tempId4;
	value_t tempT;
	
	bLength +=index;
	tempId.i = TEMPERATURE_ID;
	tempId2.i = HIG_FRE_ID;
	tempId3.i = LOW_FRE_ID;
	 
		
//		  gpsBuf[gpsIndex].longitude.bytes.value8;
//�ر��¶�װ��
    if(topSensorFlag == TOP_SENSOR_USE)
	{
		  buf[bLength++] = 0;

		  buf[bLength++] = tempId.bytes. value4;
		  buf[bLength++] = tempId.bytes. value3;
		  buf[bLength++] = tempId.bytes. value2;
		  buf[bLength++] = tempId.bytes. value1;
			tempT.f	= dsb20Buf[0].rf;
		  buf[bLength++] = tempT.bytes. value4;//dsb20Buf[0].value_t.bytes.valueH2;
		  buf[bLength++] = tempT.bytes. value3;//dsb20Buf[0].value_t.bytes.valueH1;
		  buf[bLength++] = tempT.bytes. value2;//dsb20Buf[0].value_t.bytes.valueL;
		  buf[bLength++] = tempT.bytes. value1;//dsb20Buf[0].value_t.bytes.valueH;

#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%f\n\r",0,tempId.i,tempT.f);//result[0].value_t.f
#endif
	}
	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
//�¶�����װ��
		  buf[bLength++] = nodei+1;

		  buf[bLength++] = tempId.bytes. value4;
		  buf[bLength++] = tempId.bytes. value3;
		  buf[bLength++] = tempId.bytes. value2;
		  buf[bLength++] = tempId.bytes. value1;
		
			tempT.f	= dsb20Buf[nodei+1].rf;
		  buf[bLength++] = tempT.bytes. value4;
		  buf[bLength++] = tempT.bytes. value3;
		  buf[bLength++] = tempT.bytes. value2;
		  buf[bLength++] = tempT.bytes. value1;
		
// 		  buf[bLength++] = dsb20Buf[nodei+1].value_t.bytes.valueH2;
// 		  buf[bLength++] = dsb20Buf[nodei+1].value_t.bytes.valueH1;
// 		  buf[bLength++] = dsb20Buf[nodei+1].value_t.bytes.valueL;
// 		  buf[bLength++] = dsb20Buf[nodei+1].value_t.bytes.valueH;
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%f\n\r",(nodei+1),tempId.i,tempT.f);
#endif
		  
//��Ƶˮ������װ��
		  buf[bLength++] = nodei+1;

		  buf[bLength++] = tempId2.bytes.value4;
		  buf[bLength++] = tempId2.bytes.value3;
		  buf[bLength++] = tempId2.bytes.value2;
		  buf[bLength++] = tempId2.bytes.value1;
		  tempId4.i = sensorDataBuf[nodei].fh.f; 
		  buf[bLength++] = tempId4.bytes.value4;
		  buf[bLength++] = tempId4.bytes.value3;
		  buf[bLength++] = tempId4.bytes.value2;
		  buf[bLength++] = tempId4.bytes.value1;
#if(SENSOR_DATA_DEBUG > 0)
	     printf("node:%d sensorId :%d value:%d\n\r",(nodei+1),tempId2.i,tempId4.i);
#endif
//��Ƶ�η�����װ��
         if(eachDevNum == 0x02)
		 {		  	  	    
		  buf[bLength++] = nodei+1;

		  buf[bLength++] = tempId3.bytes.value4;
		  buf[bLength++] = tempId3.bytes.value3;
		  buf[bLength++] = tempId3.bytes.value2;
		  buf[bLength++] = tempId3.bytes.value1;

		  tempId4.i = sensorDataBuf[nodei].fl.f; 
		  buf[bLength++] = tempId4.bytes.value4;
		  buf[bLength++] = tempId4.bytes.value3;
		  buf[bLength++] = tempId4.bytes.value2;
		  buf[bLength++] = tempId4.bytes.value1;
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%d\n\r",(nodei+1),tempId3.i,tempId4.i);
#endif
		  }	   
	}

   	return bLength;

}

uint16_t gprsdevDataLoad(uint8_t *buf,uint16_t index,uint16_t adBatValue,uint16_t adChargeValue,uint16_t adExtValue,uint32_t calAllCount,uint16_t count)
{
     	value_t tempId ;
    	uint16_t bLength = 0x00;
		 bLength +=index;
		 buf[bLength++] = 1;
          tempId.i = BATTERY_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%f\n\r",1,tempId.i,(adBatValue*6.144/ADS_MAXVALUE));
#endif
   //
   	 tempId.f = adBatValue*6.144/ADS_MAXVALUE;
	//��ص�ѹ��Ϣװ��
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

		  buf[bLength++] = 1;
          tempId.i = TOTAL_COLLECT_COUNT_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,calAllCounter.i);
#endif
		  tempId.i = calAllCount;
	//��ص�ѹ��Ϣװ��
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

		  buf[bLength++] = 1;
      tempId.i = COLLECT_COUNT_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,calCounter);
#endif
		  tempId.j = count;
	//�ɼ�����
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

	//̫���������ѹ
		 buf[bLength++] = 1;
        	tempId.i = SOLAR_VOLTAGE_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%f\n\r",1,tempId.i,(adChargeValue*6.144/ADS_MAXVALUE));
#endif
   //
   	 tempId.f = adChargeValue*6.144/ADS_MAXVALUE;
	//
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

	//�ⲿ���������ѹ
		 buf[bLength++] = 1;
        	tempId.i = OUTSIDE_VOLTAGE_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
	  printf("node:%d sensorId :%d value:%f\n\r",1,tempId.i,(adExtValue*6.144/ADS_MAXVALUE));
#endif
   	 tempId.f = adExtValue*6.144/ADS_MAXVALUE;
	//
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

		  return   bLength;
}


uint16_t  gprsAxAyAzDataLoad(uint8_t * buf,uint16_t index,tg_ADXL345_TYPE xyzValue)
{
  //  uint8_t nodei =1;
  	value_t tempId ;
	uint16_t bLength = 0x00;
		 bLength += index;
		 buf[bLength++] = 1;
        	tempId.i = AX_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
	 // printf("node:%d sensorId :%d value:%.3f\n\r",nodei,tempId.i,xyzValue.ax);
#endif
   	 tempId.f = xyzValue.ax;
	//��ص�ѹ��Ϣװ��
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

		 buf[bLength++] = 1;
        	tempId.i = AY_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
	//  printf("node:%d sensorId :%u value:%.3f\n\r",nodei,tempId.i,xyzValue.ay);
#endif
   	 tempId.f = xyzValue.ay;
	//��ص�ѹ��Ϣװ��
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

		 buf[bLength++] = 1;
        	tempId.i = AZ_ID;
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;

   //
#if(SENSOR_DATA_DEBUG > 0)
//	  printf("node:%d sensorId :%d value:%.3f\n\r",nodei,tempId.i,xyzValue.az);
#endif	
   	      tempId.f = xyzValue.az;
	//��ص�ѹ��Ϣװ��
		  buf[bLength++] = tempId.bytes.value4;
		  buf[bLength++] = tempId.bytes.value3;
		  buf[bLength++] = tempId.bytes.value2;
		  buf[bLength++] = tempId.bytes.value1;
	   
		  		  return   bLength;
}


uint16_t gprsGsmStatusLoad(uint8_t *buf,uint16_t index)
{
	//     	uint8_t i = 0x00;
	value_t tempId ;
	uint16_t bLength = 0x00;  
  
	bLength += index;
	
	buf[bLength++] = 1;
	tempId.i = GSM_RSSI_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	tempId.j = rssiValue;
	if(rssiValue == 99)tempId.j= 1;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = SIM_ERROR_COUNT_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	tempId.j = simCardErrorCounter;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GSM_ERROR__COUNT_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	tempId.j = netGsmErrorCounter;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GSM_LAC_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	tempId.j = lacValue;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GSM_CELL_ID_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	tempId.j = cellIdValue;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	return bLength;
}

uint16_t gprsTcpStatusDataLoad(uint8_t *buf,uint8_t channel,uint16_t index,uint16_t addLength)
{
	value_t tempId ;
	uint16_t bLength = 0x00;    
#if(MQTT_USE_CTR>0)
	uint16_t dataIndex;
	uint16_t datat=0;
	uint8_t tmpi=0;
	uint8_t num=0;
#endif
#if(SECOND_MQTT_ADDR>0)
	secondServerAddr secondServerBuf[SECOND_MULTI_CONNECTION_SIZE];	
#endif
	
	bLength +=index;
	
	buf[bLength++] = 1;	
	tempId.i = SERVER_ID_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	//buf[bLength++] = 2;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,channel);
#endif
	tempId.j = channel;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GPRS_REG_ERR_COUNT_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].gprsRegErrorCounter);
#endif
	//buf[bLength++] = 2;
	tempId.j = serverLSRLog[channel].gprsRegErrorCounter;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GPRS_BUSSINESS_ERR_COUNT_ID;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].gprsAttachErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].gprsAttachErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GPRS_APN_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].gprsApnSetErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].gprsApnSetErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = GPRS_WIRELESS_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].gprsBringUpErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].gprsBringUpErrorCounter;
	//�ɼ�����
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = OBTAIN_IP_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].getIpErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].getIpErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = TCP_CONNECT_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].linkErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].linkErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = TCP_SEND_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].sendErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].sendErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = RECEIVE_ERR_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].rxErrorCounter);
#endif
	tempId.j = serverLSRLog[channel].rxErrorCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;

	buf[bLength++] = 1;
	tempId.i = SEND_SUCCESS_COUNT ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(SENSOR_DATA_DEBUG > 0)
	printf("node:%d sensorId :%d value:%d\n\r",1,tempId.i,serverLSRLog[channel].dataOkCounter);
#endif
	tempId.j = serverLSRLog[channel].dataOkCounter;
	//buf[bLength++] = 2;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
#if(MQTT_USE_CTR>0) 
	buf[bLength++] = 1;
	tempId.i 			 = SERVER_STATUS_ID ;
	buf[bLength++] = tempId.bytes. value4;
	buf[bLength++] = tempId.bytes. value3;
	buf[bLength++] = tempId.bytes. value2;
	buf[bLength++] = tempId.bytes. value1;
	
	dataIndex = bLength;
	bLength++;
	if(devPar.stationMode == FUNCITONG_TURN_ON)num = mqttServerNum;
	datat =	sprintf((char *)buf+bLength,"%d",num);
	bLength	+= datat;
	for(tmpi=0;tmpi<num;tmpi++)
	 {		
		datat =	sprintf((char *)buf+bLength,",%d",mqttSeverStatus[tmpi]);
		bLength	+= datat;		
	 }
	buf[dataIndex] = bLength-dataIndex-1;
 #if(GPRS_STATUS_DEBUG>0)		  
	printf(" paramId :%d value %d ",tempId.i,buf[index]);
	for(datat=0;datat<buf[dataIndex];datat++)printf("%c",buf[dataIndex+1+datat]);
	printf("\r\n");
 #endif
#endif
#if(ALARM_CTR>0)
	if(alarmSource>ALARM_SOURCE_NONE)//(&&(devPar.moistureThresholdKey == FUNCITONG_TURN_ON)) //210623
	{
		buf[bLength++] = 1;
		tempId.i 			 = ALARM_STATUS_ID ;
		buf[bLength++] = tempId.bytes. value4;
		buf[bLength++] = tempId.bytes. value3;
		buf[bLength++] = tempId.bytes. value2;
		buf[bLength++] = tempId.bytes. value1;

		dataIndex = bLength;
		bLength++;
		datat =	sprintf((char *)buf+bLength,"%d",alarmSource);
		bLength	+= datat;
		datat =	sprintf((char *)buf+bLength,",%c",alarmLevel);
		bLength	+= datat;
		
		if((alarmSource==READ_SOURCE_CHANNEL1)||(alarmSource==READ_SOURCE_CHANNELAll))//210623
		{
			if(alarmSource==READ_SOURCE_CHANNEL1)
			{
				datat =	sprintf((char *)buf+bLength,",%d|",mqttServerNumFirst);
				bLength	+= datat;
				num = mqttServerNumFirst;
			}
			else if(alarmSource==READ_SOURCE_CHANNELAll)
			{
				datat =	sprintf((char *)buf+bLength,",%d|",mqttServerNum);
				bLength	+= datat;	
				num = mqttServerNum;
			}
			for(tmpi=0;tmpi<num;tmpi++)
			{
				datat =	sprintf((char *)buf+bLength,"tcp,%s,",&mqttServerAddrBuf[tmpi].endpoint[1]);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%d,",mqttServerAddrBuf[tmpi].port);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%s,",&mqttServerAddrBuf[tmpi].deviceId[1]);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%s,",&mqttServerAddrBuf[tmpi].username[1]);
				bLength	+= datat;
				datat =	sprintf((char *)buf+bLength,"%s|",&mqttServerAddrBuf[tmpi].password[1]);
				bLength	+= datat;
			}
		}
		else if(alarmSource==READ_SOURCE_CHANNEL2)
		{
		#if(SECOND_MQTT_ADDR>0)
			mqttSecondServerAddrRead(secondServerBuf);
		#endif
			datat =	sprintf((char *)buf+bLength,",%d|",mqttServerNumSecond);
			bLength	+= datat;
		#if(SECOND_MQTT_ADDR>0)
			for(tmpi=0;tmpi<mqttServerNumSecond;tmpi++)
			{
				datat =	sprintf((char *)buf+bLength,"tcp,%s,",&secondServerBuf[tmpi].endpoint[1]);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%d,",secondServerBuf[tmpi].port);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%s,",&secondServerBuf[tmpi].deviceId[1]);
				bLength	+= datat;	
				datat =	sprintf((char *)buf+bLength,"%s,",&secondServerBuf[tmpi].username[1]);
				bLength	+= datat;
				datat =	sprintf((char *)buf+bLength,"%s|",&secondServerBuf[tmpi].password[1]);
				bLength	+= datat;
			}
		#endif
		}
		else if(alarmSource==READ_SOURCE_STATUS)
		{
			datat =	sprintf((char *)buf+bLength,",period %c_%d,%d",curPeriodType,serverConfig.devDensity,curDevPeriod);
			bLength	+= datat;	
			datat =	sprintf((char *)buf+bLength,",plus %d",devPar.plus_intv);
			bLength	+= datat;
			datat =	sprintf((char *)buf+bLength,",THD %.2f",cigemsensorlimit[SENSORID_INDEX_QJ].threshold);
			bLength	+= datat;	
			datat =	sprintf((char *)buf+bLength,",%.2f,%.2f",cigemsensorlimit[SENSORID_INDEX_QJ].upper,cigemsensorlimit[SENSORID_INDEX_QJ].lower);
			bLength	+= datat;
		}
		if(alarmSource>=READ_SOURCE_CHANNEL1) 
		{
			alarmSource = ALARM_SOURCE_NONE;
			alarmLevel 	= 0;
		}
	
	buf[dataIndex] = bLength-dataIndex-1;
 #if( GPRS_STATUS_DEBUG  >0)		  
	printf(" paramId :%d value %d ",tempId.i,buf[index]);
	for(datat=0;datat<buf[dataIndex];datat++)printf("%c",buf[dataIndex+1+datat]);
	printf("\r\n");
 #endif
}
#endif	 
	/*		  if(sendType == MULTI_SEND_TYPE)
	{
	buf[bLength++] = 0x00;		  
	buf[bLength++] = 0x00;	
	buf[bLength++] = 0x00;		  
	buf[bLength++] = 0x00;		  	  
	}
	*/
	tempId .i = bLength - addLength-4;
	buf[addLength++] = tempId.bytes. value4;
	buf[addLength++] = tempId.bytes. value3;
	buf[addLength++] = tempId.bytes. value2;
	buf[addLength++] = tempId.bytes. value1;
	return   bLength;
}
uint16_t loadSimCardInf(uint8_t *buf,uint16_t index)
{
	uint8_t i = 0;
	value_t tempId ;
	uint16_t bLength = 0x00;   
	//if(rxResult->simCardReadOkFlag == FUNCITONG_TURN_ON) 
	{        
	bLength += index;
	if(ccidBuf[0]>0)//220805
	{
		buf[bLength++] = 1;
		tempId.i = SIM_ICCID_ID;//134;
		buf[bLength++] = tempId.bytes. value4;
		buf[bLength++] = tempId.bytes. value3;
		buf[bLength++] = tempId.bytes. value2;
		buf[bLength++] = tempId.bytes. value1;  

		buf[bLength++] = ccidBuf[0]-1;

		for(i = 1;i < ccidBuf[0];i++)
		{
			buf[bLength++] = ccidBuf[i];
		}	
//	printf("\r\nCCID:");
	//usart2StoreBuffer(&ccidBuf[1],ccidBuf[0]);    
	}
	if(	cimiBuf[0]>0)//220805
	{
		buf[bLength++] = 1;
		tempId.i = SIM_IMSI_ID;//135;
		buf[bLength++] = tempId.bytes. value4;
		buf[bLength++] = tempId.bytes. value3;
		buf[bLength++] = tempId.bytes. value2;
		buf[bLength++] = tempId.bytes. value1; 

		buf[bLength++] = cimiBuf[0]-1;

		//printf("\r\nCIMI:%d",bLength);        
		for(i = 1;i < cimiBuf[0];i++)
		{
			buf[bLength++] = cimiBuf[i];
		}  
		//printf("\r\n2CIMI:%d",bLength); 
		// printf("\r\nCIMI:");
		//usart2StoreBuffer(&cimiBuf[1],cimiBuf[0]);
	}
	if(imeiBuf[0]>0)//220805
	{
		buf[bLength++] = 1;
		tempId.i = SIM_IMEI_ID;//133;
		buf[bLength++] = tempId.bytes. value4;
		buf[bLength++] = tempId.bytes. value3;
		buf[bLength++] = tempId.bytes. value2;
		buf[bLength++] = tempId.bytes. value1; 

		buf[bLength++] = imeiBuf[0]-1;      

		for(i = 1;i < imeiBuf[0];i++)
		{
			buf[bLength++] = imeiBuf[i];
		}   
	//	printf("\r\nIMEI:");
		//usart2StoreBuffer(&imeiBuf[1],imeiBuf[0]);  
	}  
#if (GPRS_STATUS_DEBUG > 0)
		printf(" CCID:%s\r\n",ccidBuf+1);
		//devStoreBuffer(&ccidBuf[1],ccidBuf[0]); 
		printf(" CIMI:%s\r\n",cimiBuf+1);
		//devStoreBuffer(&cimiBuf[1],cimiBuf[0]);
		printf(" IMEI:%s\r\n",imeiBuf+1);
		//devStoreBuffer(&imeiBuf[1],imeiBuf[0]); 
		//printf("\r\n");
	#endif	
	}
	return bLength;
}
#define SENSOR_DATA_DEBUG	1
int8_t gprsMultSensorDataLoad(uint8_t *buf,uint32_t sendAddr, uint16_t *lentth,uint16_t readSendPacket,uint8_t *sendPacket,uint16_t *lengthAddress)
{
	uint8_t sendi = 0x00;
	uint8_t sendj = 0x00;
	//	uint8_t sendPacket = 0x00;
__IO	uint16_t pcketLength = 0x00;
	value_t packetLoadLength;
	int8_t 	result = -1;
	uint8_t tryi=0;
	pcketLength = 0x00 ;
	*lentth = 0x00;
#if(MQTT_USE_CTR>0)
	signPackNUm	=	2;
	if(alarmSource>=READ_SOURCE_CHANNEL1)signPackNUm	=	1;//210623
#endif
	if(readSendPacket > signPackNUm)
	{
		sendj = signPackNUm;
	}  
	else
	{
		sendj = readSendPacket;
	}

	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
//װ������
//	printf("\n\r�������ݷ���4:%d\n\r",pcketLength);

	for(sendi   = 0x01;sendi  < DEV_SN_SIZE;sendi ++)
	{
		buf[(*lentth) ++] = devSnBuf[sendi];
	}
	for(sendi = 0x01;sendi< AUTH_CODE_SIZE;sendi ++)
	{
		buf[(*lentth) ++] = authCodeBuf[sendi];
	}
	sensorDataReadAddr	=	sendAddr;//221101
	for(sendi = 0x00;sendi < sendj; )
	{
	//��ȡ��װ��
		if(sendAddr ==sensorDataWriteAddr)
		{
			sendj= sendi;
			break;
		}
		pcketLength = 0;//210705
		sensorDataReadToBuf(SEND_TYPE_GPRS,sendAddr,&buf[*lentth],&pcketLength); //include QX DATA
		if(pcketLength!= 0x00)
		{
			sendi ++ ;
			//����ָ��ַ�
			//printf("\n\r�������ݷ���3:%d\n\r",pcketLength);
			//�۳�ʱ���ַ�������
			packetLoadLength.i = pcketLength -12;
			*lengthAddress = *lentth +8;	   
			buf[(*lentth)+8] 	=	packetLoadLength.bytes.value4;
			buf[(*lentth)+9] 	=	packetLoadLength.bytes.value3;
			buf[(*lentth)+10] =	packetLoadLength.bytes.value2;
			buf[(*lentth)+11] =	packetLoadLength.bytes.value1;  
			sensorDataReadAddr	=	sendAddr;//221101
			(*lentth)   = (*lentth)  +pcketLength; 			
		}
		else//221213
		{
			tryi++;
			if(tryi>10)
			{
				sendj	= sendi;	
				sensorDataReadAddr	=	sendAddr;
				break;
			}
		}
		sendAddr += SENSOR_DATA_STORE_SIZE;
		if(sendAddr >= SENSOR_DATA_END_ADDR)
		{
			sendAddr = SENSOR_DATA_BASE_ADDR;
		}
	// printf("\n\r packet:%d ,length:%d,addr:%d \n\r",sendi,*lentth,sendAddr);

	}
	SPI_Flash_PowerDown();
	power331Off();
	if(sendi != 0x00)
	{
		//*lentth   = (*lentth -4);
		*sendPacket = sendj;
		result = 1;
	}
	return result;
}



//__IO uint8_t gpsLocation = 0x00;	
//__IO uint32_t configVersion = 0;

//�������з��������ӱ�־

#define LINK_AGIN_FLAG	0x01
#define LINK_EXIT_FLG		0x02

#define DATA_FORMAT			0x02
#define DATA_UNFORMAT		0x01

#define SIGNEL_LINK_MAX_SEND_COUNTER	0x04
#define FIREWARE_REQUEST_FLAGE    		0x01

void serverBufInt(void)
{
	uint8_t tcpLinkChannel = 0x00;
	for(tcpLinkChannel= 0x00;tcpLinkChannel < MAX_MULTI_CONNECTION_SIZE;tcpLinkChannel++)
	{
		if(tcpServerAddrBuf[tcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
		{
			serverLSRLog[tcpLinkChannel].setTry = 0x00;
			serverLSRLog[tcpLinkChannel].allTry  = 0x00;
			serverLSRLog[tcpLinkChannel].linkTry = 0x00;
			serverLSRLog[tcpLinkChannel].sendTry = 0x00;
			serverLSRLog[tcpLinkChannel].rxTry = 0x00;
			serverLSRLog[tcpLinkChannel].rxDataTry= 0x00;
			serverLSRLog[tcpLinkChannel].findIpTry= 0x00;
			serverLSRLog[tcpLinkChannel].sendFlag = UNSEND_FLAG;

			if(serverLSRLog[tcpLinkChannel].gprsRegErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].gprsRegErrorCounter = 0x00;
			}                    
			if(serverLSRLog[tcpLinkChannel].gprsAttachErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].gprsAttachErrorCounter = 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].gprsBringUpErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].gprsBringUpErrorCounter= 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter= 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].getIpErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].getIpErrorCounter = 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].linkErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].linkErrorCounter = 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].sendErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].sendErrorCounter= 0x00;
			} 
			if(serverLSRLog[tcpLinkChannel].rxErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].rxErrorCounter= 0x00;
			}                  
			if(serverLSRLog[tcpLinkChannel].dataOkCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].dataOkCounter = 0x00;
			}
			if(serverLSRLog[tcpLinkChannel].rxDataErrorCounter > 50000)
			{
				serverLSRLog[tcpLinkChannel].rxDataErrorCounter = 0x00;
			}
		}
		else
		{
			serverLSRLog[tcpLinkChannel].allTry  = 0x04;	
		}   	
	}		
}
extern void sk60OffsetValueSave(__IO float value);	
int8_t gprsConfigTask(httpConfig *rxResult)
{
	__IO int8_t		configResult	= -1;
	uint16_t	periodTmp			=	0;
	uint16_t	thresholdTmp	=	0;
#if(DEV_APN_SET_CTR>0) //230508
	int8_t	resultStatus = -1;
#endif
#if(SK60_CTR>0)
	uint8_t	funcTmp = 0;
#endif	
	
	rxResult->dataSentFlag 	= 0x01;			

	configResult = determineConfig();
	//�̼��ж�
	if(rxResult->firewareFlag==FIREWARE_IDLE)
	{
		rxResult->firewareFlag =determineFireWare();	
	}	
#if GPRS_DEBUG > 0
	printf("configResult %d,cur fireWare flag[%d],config[%d]\r\n",configResult,rxResult->firewareFlag,configValue.configVersion); 
#endif
	if(!findConfigVersion(&configValue.configVersion))
	{
		rxResult->storeFlagConfig++;
	#if GPRS_DEBUG > 0
		printf("\n\r config version %d\n\r",configValue.configVersion);
	#endif
		configResult = 0;//20210308
	}
	if(configResult  == 0x00)
	{
		
		//��ȡ��������
		if(!findConfigDevPeriod(&configValue))
		{
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r ���������޸�Ϊ:%d\n\r",configValue.devPeriod);
		#endif
		}
		
		if(!findConfigSendDensity(&configValue.transmissionDensity))
		{
			rxResult->storeFlagConfig++;
			rxResult->devDensity = configValue.transmissionDensity;  
		#if GPRS_DEBUG > 0
			printf("\n\r ���ݷ����ܶ��޸�Ϊ:%d\n\r",configValue.transmissionDensity);
		#endif
		}
		
		if(!findConfigDevTransmissDensity(&configValue.transmissionDensity))
		{
			rxResult->storeFlagConfig++;
			rxResult->devDensity = configValue.transmissionDensity;  
		#if GPRS_DEBUG > 0
			printf("\n\r ʡ��ģʽ ���ݷ����ܶ��޸�Ϊ:%d\n\r",configValue.transmissionDensity);
		#endif
		}
		if(!findXyzMaxValue(&configValue))
		{
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r X�ֵᷧ:%f��Y�ֵᷧ:%f��Z�ֵᷧ:%f\n\r",(configValue.xValue*0.0039),(configValue.yValue*0.0039),(configValue.zValue*0.0039));
		#endif
		}
		if(!findConfigXyzKey(&configValue))
		{
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r xyz key state %c \n\r",configValue.xyzSharkkey);
		#endif
		}
		if(!findThresholdKey(&configValue.moistureThresholdKey))
		{
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r threshold key state %c \n\r",configValue.moistureThresholdKey);
		#endif
		}
	if(configValue.moistureThresholdKey == FUNCITONG_TURN_ON)
	{
		periodTmp			=	configValue.threshold1Period;
		thresholdTmp	=	configValue.moistureThreshold1;
		if(!findThresholdValue(THRESHOLD_VALUE1,&configValue))
		{
			if(configValue.threshold1Period<7201)//120*60=7200
			{
				devPar.plus_intv	= configValue.threshold1Period;
				rxResult->storeFlagParam++;				
			#if(MQTT_USE_CTR>0)
				cigemsensortime.plus	= devPar.plus_intv;
				cigemsensorlimit[SENSORID_INDEX_HS].threshold = configValue.moistureThreshold1;
				rxResult->storeFlagSensorRange++;
			#endif
			}
		#if(MQTT_USE_CTR>0)
			else if(configValue.threshold1Period==8400)//140
			{
				cigemsensorlimit[SENSORID_INDEX_QJ].threshold	=	configValue.moistureThreshold1-4;//1~16
				rxResult->storeFlagSensorRange++;
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;					
			}
			else if(configValue.threshold1Period==9600)//160
			{
				cigemsensorlimit[SENSORID_INDEX_QJ].threshold	=	configValue.moistureThreshold1*10;//50~200
				rxResult->storeFlagSensorRange++;
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;					
			}				
			else if(configValue.threshold1Period==10800)//180
			{
				cigemsensorlimit[SENSORID_INDEX_QJ].threshold	=	configValue.moistureThreshold1;//5~20
				rxResult->storeFlagSensorRange++;
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;					
			}				
			else if(configValue.threshold1Period==12000)//200
			{			
				cigemworkmode = WORKMODE_PLUS;			
				rxResult->perSetFlag	=	CHANGE_FLAG;
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;				
			}		
			else if(configValue.threshold1Period==13200)//220
			{		
			#if(BEEP_ALARM_CTR>0)
				alarmTimes	=	1;
				alarmbeepStatus	= ALARM_STATUS_FREE;
				alarmType	 	= MODBUS_ALARM_INDEX_LANDSLIDE2;
			#endif	
				alarmSource =	ALARM_SOURCE_PLATFORM;				
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;	
			}					
			else if(configValue.threshold1Period==14400)//240
			{		
				cigemworkmode = WORKMODE_RUN;			
				rxResult->perSetFlag	=	CHANGE_FLAG;
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;	
			}
		#endif
			else
			{
				configValue.moistureThreshold1	=	thresholdTmp;
				configValue.threshold1Period		=	periodTmp;
			}
		
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r threshold  value1:%d,period1:%d \n\r",configValue.moistureThreshold1,configValue.threshold1Period);
		#endif
		}
		periodTmp			=	configValue.threshold2Period;
		thresholdTmp	=	configValue.moistureThreshold2;
		if(!findThresholdValue(THRESHOLD_VALUE2,&configValue))
		{
			if(configValue.threshold2Period<7201)//120*60=7200
			{
				devPar.plus_intv 			= configValue.threshold2Period;	
				rxResult->storeFlagParam++;	
			#if(MQTT_USE_CTR>0)
				cigemsensortime.plus	= devPar.plus_intv;
				cigemsensorlimit[SENSORID_INDEX_HS].threshold = configValue.moistureThreshold2;
				rxResult->storeFlagSensorRange++;
			#endif				
			}
		#if(MQTT_USE_CTR>0)
			else if(configValue.threshold2Period==10800)//180
			{
				cigemsensorlimit[SENSORID_INDEX_QJ].threshold	=	configValue.moistureThreshold2;//30~60
				rxResult->storeFlagSensorRange++;
				configValue.moistureThreshold2	=	thresholdTmp;
				configValue.threshold2Period		=	periodTmp;				
			}
		#endif
			else
			{
				configValue.moistureThreshold2	=	thresholdTmp;
				configValue.threshold2Period		=	periodTmp;	
			}
		
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r threshold  value2:%d,period2:%d \n\r",configValue.moistureThreshold2,configValue.threshold2Period);
		#endif
		}
		periodTmp	=	configValue.threshold3Period;
		thresholdTmp	=	configValue.moistureThreshold3;
		if(!findThresholdValue(THRESHOLD_VALUE3,&configValue))
		{
			if(configValue.threshold3Period<7201)//120*60=7200
			{
				devPar.plus_intv	= configValue.threshold3Period;	
				rxResult->storeFlagParam++;
			#if(MQTT_USE_CTR>0)
				cigemsensortime.plus	= devPar.plus_intv;
				cigemsensorlimit[SENSORID_INDEX_HS].threshold = configValue.moistureThreshold3;
				rxResult->storeFlagSensorRange++;
			#endif
			}
		#if(MQTT_USE_CTR>0)
			else if(configValue.threshold3Period==10800)//180
			{
				cigemsensorlimit[SENSORID_INDEX_QJ].threshold	=	configValue.moistureThreshold3;//10~30
				rxResult->storeFlagSensorRange++;
				configValue.moistureThreshold2	=	thresholdTmp;
				configValue.threshold2Period		=	periodTmp;				
			}
		#endif			
			else 
			{				
				configValue.moistureThreshold3	=	thresholdTmp;
				configValue.threshold3Period		=	periodTmp;				
			}
		
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r threshold  value3:%d,period3:%d \n\r",configValue.moistureThreshold3,configValue.threshold3Period);
		#endif
		}
	}       
		if(!findfireWareId(&configValue))
		{
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r fireWare Id %c \n\r",configValue.curfireWareId);
		#endif
		}
		if(!findConfiggpsKey(&configValue.gpsKey))
		{
			if(configValue.gpsKey == FUNCITONG_TURN_OFF)
			{
				clearGpsValue(gpsBuf);
				clearGpsValue(gpsBuf+1);
			}
			rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r gps key state %c \n\r",configValue.gpsKey);
		#endif
		}
		if(!findConfigUsartKey(&devPar.usartKey))
		{						
			rxResult->storeFlagParam++;
		#if GPRS_DEBUG > 0
			printf("\n\r usart key state %c \n\r",devPar.usartKey);
		#endif
		}

		if(!configHydrologyKey(&devPar.stationMode))
		{						
			rxResult->storeFlagParam ++;
			rxResult->storeFlagSensorRange++;
		#ifdef SCHYDROLOGY 
			curStationSend[0].unSendPacket = 0x00;
			curStationSend[0].curSendAddress = curGprsSendAddr;
			curStationSend[1].unSendPacket = 0x00;
			curStationSend[1].curSendAddress = curGprsSendAddr;

		//ң��վ�ź�������
			if((devPar.stationAddr[0]!=11)||(devPar.stationKey[0]!=7))
			{//ң��վ�ų�ʼ��
				stationReset();
			}
		#endif 
		#if GPRS_DEBUG > 0
			printf("\n\r hydrology mode set ok %c\n\r",devPar.stationMode);
		#endif
		}//findConfigPowerSavingKey
		if(!findConfigQXKey(&devPar.qxkey))
		{	
			rxResult->storeFlagParam ++;
		#if GPRS_DEBUG > 0
			printf("\n\r qxkey %c\n\r",devPar.qxkey);
		#endif 
		}	
	#if(ACCE_CTR>0)
		if(!findConfigAccelerometerKey(&devPar.accelerometerkey))
		{	
			rxResult->storeFlagParam ++;
		#if GPRS_DEBUG > 0
			printf("\n\r accelerometerkey %c\n\r",devPar.accelerometerkey);
		#endif 
		}
	#endif
		if(!findConfigPowerSavingKey(&devPar.powerSavingKey))
		{	
			rxResult->storeFlagParam ++;
			serverConfig.powerSavingKey	=	devPar.powerSavingKey;
		#if GPRS_DEBUG > 0
			printf("\n\r powersavingkey %c\n\r",devPar.powerSavingKey);
		#endif 
		}
		if(!findConfigInsentekKey(&devPar.insentekKey))
		{
			rxResult->storeFlagParam ++;
			rxResult->insentekKey = devPar.insentekKey;
		#if GPRS_DEBUG > 0
			printf("\n\r insentek key state %c\n\r",devPar.insentekKey);
		#endif
		}		
	#if(MQTT_USE_CTR>0)		
		if(!configHydrologyGsmKey(&devPar.gsmRunMode))
		{
			rxResult->gsmRunMode	=	devPar.gsmRunMode;
			rxResult->storeFlagParam ++;
		#if GPRS_DEBUG > 0
			printf("\n\r gsm mode state %c\n\r",rxResult->gsmRunMode);
		#endif 
		}				
		if(!findMqttSeverAddr(mqttServerAddrBuf))
		{
			rxResult->storeFlagStationServer ++;
			rxResult->storeFlagSensorRange++;
		#if(SECOND_MQTT_ADDR>0)
			if(mqttServerNum==0){clearSecondAddrStore();}
		#endif
		#if GPRS_DEBUG > 0
			printf("\n\r station server set ok\n\r");
		#endif
		}
		if(!findMqttSeverAddr2())
		{
		#if GPRS_DEBUG > 0
//			printf("\n\r second server set ok\n\r");
		#endif
		}
		
		if(!findConfigmqttdatatype(&devPar.mqttDataType))
		{
			rxResult->storeFlagParam ++;
		#if GPRS_DEBUG > 0
			printf("\n\r mqtt data type %c\n\r",devPar.mqttDataType);
		#endif
		}
	#endif
		if(!requestDevGpsKey(&configValue.gpsLocation))
		{
		//	rxResult->storeFlagConfig++;
		#if GPRS_DEBUG > 0
			printf("\n\r gps location state %d \n\r",configValue.gpsLocation);
		#endif
		}
		if(!findFixTimePeriod(&configValue))
		{
			rxResult->storeFlagConfig++;
			rxResult->perSetFlag	=	CHANGE_FLAG;						
		#if GPRS_DEBUG > 0
			printf("\n\r fix time hour:%02d,min:%02d \n\r",configValue.devPeriodHour,configValue.devPeriodMin);
		#endif
			if(configValue.devPeriodHour<1)configValue.devPeriodHour=1;//210705
		}
	#if(SK60_CTR>0)		
		if(!findConfigSK60Function(&funcTmp))//devPar.sk60Fuction 
		{
			rxResult->storeFlagParam ++;
			
			if((funcTmp == FUNCITONG_SK60_YEWEI)&&(devPar.sk60Fuction != FUNCITONG_SK60_YEWEI))//210820 210822_funcTmp == FUNCITONG_SK60_YEWEI)
			{			
				sk60BaseDistance = 0;
				sk60OffsetValueSave(sk60BaseDistance);
				if(sk60Value.dis>0.0)
				{
					sk60OffsetDistance 	= sk60Value.dis-sk60BaseDistance;
				}				
			}
			devPar.sk60Fuction = funcTmp;
		#if GPRS_DEBUG > 0
			printf("\n\r sk60Fuction %c \n\r",devPar.sk60Fuction);
		#endif
			
			sk60InitFlag = SK60_SET_FUNC_FLAG;
		}
		if(!findConfigSK60TestKey(&sk60TestMode))
		{
		#if GPRS_DEBUG > 0
			printf("\n\r sk60TestMode %c \n\r",sk60TestMode);
		#endif
			sk60InitFlag = SK60_SET_FLAG;
			if((devPar.sk60Fuction != FUNCITONG_SK60_DISP)&&(devPar.sk60Fuction != FUNCITONG_SK60_RAW))sk60TestMode = 0;
			if(sk60TestMode != FUNCITONG_TURN_ON)sk60InitFlag	= 0;			
		}
		if(!findConfigSK60Offset(&sk60BaseDistance))//210705
		{
			sk60OffsetValueSave(sk60BaseDistance);
			if((sk60Value.dis>0.0)&&(sk60BaseDistance>0))//210820
			{
				sk60OffsetDistance 	= sk60Value.dis-sk60BaseDistance;				
			}
			sk60Value.v		= sk60BaseDistance;//210822
			sk60InitFlag = SK60_BASE_FLAG;
			sk60DispErrTimes = 0;
		#if GPRS_DEBUG > 0
			printf("\n\r sk60OffsetValue %f \n\r",sk60BaseDistance);
		#endif
		}
	#endif
	#if(KXYL_CTR>0)
		if(!findConfigKXYLFunction(&devPar.kxylFuction))//210705
		{
			rxResult->storeFlagParam ++;
		#if GPRS_DEBUG > 0
			printf("\n\r kxylFuction %c \n\r",devPar.kxylFuction);
		#endif
		}
		if(!findParamConfig(PARAM_TYPE_KXYL))
		{
		
		}
	#endif
	//					if(!findBeginTimeSet(&configValue.zeroTime))
	//					{
	//					#if GPRS_DEBUG > 0
	//						printf("\n\r rain zero time set:%d\n\r",configValue.zeroTime);
	//					#endif  
	//					}				
	//					if(!findRainPeriodSet(&configValue.rainPeriod))
	//					{
	//					#if GPRS_DEBUG > 0
	//						printf("\n\r rain period set:%d\n\r",configValue.rainPeriod);
	//					#endif
	//					}	
	//					if(!findParamConfig())
	//					{
	//				
	//					}
		if(!findWaterabcValue(curAbc))
		{
		#if GPRS_DEBUG > 0
			printf("\n\ra b c set ok ");
			printf("%f,%f,%f\n\r",curAbc[0].a.f,curAbc[0].b1.f,curAbc[0].c.f);
		#endif                                 
			rxResult->storeFlagAbc ++;
		}
		if(!findFawValue(curFaw))
		{
		#if GPRS_DEBUG > 0
			printf("\n\rFaH FwH set ok ");
			printf("%f,%f\n\r",curFaw[0].Fa.f,curFaw[0].Fw.f);
		#endif                                 
			rxResult->storeFlagFaw++;
		}	
		if(!findDepValue(curDep))
		{
			rxResult->storeDepFlag++;
		#if GPRS_DEBUG > 0
			printf("\n\rdep set ok\n\r");
		#endif 
		}
	#if(DEV_APN_SET_CTR>0) //230508
		resultStatus = findApn(&devPar);//230508
		if(resultStatus != -1)
		{
			netlinkFlag = STATUS_NONE;
			if(resultStatus == 0)
			{
				if(devApnSetStatus != APN_SAVE_STATUS_INIT)
				{
					devApnSetStatus = APN_SAVE_STATUS_NONE;
					rxResult->devApnSetFlag ++;
				}
				rxResult->storeFlagParam ++;
			}
			else if(resultStatus == 1)
			{
				if(devApnSetStatus != APN_SAVE_STATUS_INIT)
				{
					devApnSetStatus = APN_SAVE_STATUS_NONE;
					rxResult->devApnSetFlag ++;
				}
			}
			else if(resultStatus == 2)
			{
				rxResult->storeFlagParam ++;
				if(devApnSetStatus != APN_SAVE_STATUS_INIT)
				{
					if(devPar.apnBuf[0] != 0)
					{
//						if(devPar.stationMode != FUNCITONG_TURN_ON) //230508
							devApnSetStatus = APN_SAVE_STATUS_NONE;
					}
					else
					{
						devApnSetStatus = APN_SAVE_STATUS_DONE;
					}
					rxResult->devApnSetFlag ++;
				}
			}
			
		#if GPRS_DEBUG > 0
			printf("\r\napn set %d status %d ",resultStatus,devApnSetStatus);
			printf("dev:%s ",apnBufSet);
			printf("net:%s\r\n",devPar.apnBuf);											
		#endif
			gprsApnSet(0);
		}		
	#endif
	#ifdef SCHYDROLOGY                             
		if(!findStaionName(&devPar))
		{
		#if GPRS_DEBUG > 0
			printf("\n\rstation name set ok\n\r");
		#endif                      //�����ʼ��  
			stationPwReset();
			rxResult->storeFlagParam ++;
		}
		if(!findBackUpServer(&stationServerAddrBuf[0]))
		{
			rxResult->storeFlagStationServer ++;
		}
	#endif 	
		if(findTimeZone(&devPar.timeZone)==0)
		{
			rxResult->storeFlagParam ++;		
		#if GPRS_DEBUG > 0
			printf("\n\r rtc timeZone %d\n\r",devPar.timeZone);
		#endif
		}
		findInitMode(&(rxResult->inItMode));
		{
			if(rxResult->inItMode != FUNCITONG_TURN_ON)	rxResult->inItModeCounter 	= 0;//221103
		#if GPRS_DEBUG > 0
			printf("\n\r init mode %c \n\r",rxResult->inItMode);
		#endif        
		}			
		findClockCalibrationKey(&(rxResult->rtcSetFlag));									               
						
		if(rxResult->historyDataFlag != FUNCITONG_TURN_ON)
		{
		/*ȥ������ʱ���ѯ���ݵĹ���*/
		//findRequestDataTime(rxResult);
			findRequestData(rxResult);
			if(rxResult->requestDataType == REQUEST_DATA_BY_COUNTER)
			{
			//rxResult->historyDataFlag = FUNCITONG_TURN_ON;			   
				rxResult->requestDataReadAddress = curGprsSendAddr;
			#if GPRS_DEBUG > 0
				printf("\n\r find history  data  %d \n\r",rxResult->requestData);
			#endif
			}
		}
	#if defined(STM32L1XX_MDP)
		if(findConfigGPSValue(&devLongitude,&devLatitude)==0)
		{
			gpsBuf[1].longitude.f = devLongitude;
			gpsBuf[1].latitude.f 	= devLatitude;
			gpsBuf[1].gpsSource		=	PLATFORM_SOURCE;
			gpsBuf[1].useFlag			=	BUF_UNUSE_FLAG;
		//gps ���ݴ洢		
			power331On();
			delay_GprsMs(10);
			SPI_Flash_WAKEUP();
			delay_GprsMs(5);				
			devGpsValueFlashStore(gpsBuf[1] ,&gpsDataWriteAddr);
			SPI_Flash_PowerDown();
			power331Off();
		#if GPRS_DEBUG > 0
			printf("\n\r devLongitude:%f, devLatitude:%f\n\r",devLongitude,devLatitude);
		#endif
		}
		if(!findConfigNetworkMode(&(rxResult->networkMode)))//210623
		{
		#if GPRS_DEBUG > 0
			printf("\n\r networkMode:%c \n\r",rxResult->networkMode);
		#endif
		}
		if(!findConfigDevReset())//210623
		{
		#if GPRS_DEBUG > 0
			printf("\n\r find reset cmd %d\n\r",fireWareId);
		#endif
		}
	#endif		
	}
	return configResult;
}

#define FIREWARE_TASH_MAX_ERROR_COUNTER       0x10

void gprsFireWareRxTask(httpConfig *rxResult)
{
	uint8_t		txRxTaskState 	=	GPRS_RUN;
	uint16_t	rxDataLength 		= 0x00;
	uint16_t  readLength 			= 0x00;
	uint32_t	curGetFireWare 	= 0x00;
	uint32_t	curEndFireWare 	= 0x00;
	uint32_t	fireWareAddr 		= DEV_FIREWARE2_BASE_ADDR;
//	uint8_t   sendCmdAck			=	0;
	rxResponse tcpResponse;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	uint8_t 	tryTimes				=	0;
#endif
	
#if GPRS_DEBUG > 0
	printf("\n\r gprsFireWareRxTask packetSize:0x%03X,0x%03X\n\r",FIREWARE_PACKET_SIZE,GPRS_RX_DATA_SIZE);
#endif
	cureTcpLinkChannel = TCP_SERVER_INSENTEK;
	serverBufInt();
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsState	=	GPRS_SINGEL_CLOSE;//GPRS_LINK;
#else
	gprsState	=	GPRS_LINK;
#endif	
	curEndFireWare 	= curGetFireWare +FIREWARE_PACKET_SIZE;
	readLength 			= FIREWARE_PACKET_SIZE;
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
	delay_GprsMs(5);
	devFlashErare(fireWareAddr,DEV_FIREWARE2_END_ADDR);
//	delay_GprsMs(10);
	while(txRxTaskState == GPRS_RUN)
	{	
		switch(gprsState)
		{
			case GPRS_SET:
			{
				if(gprsSet(&serverLSRLog[cureTcpLinkChannel])==COMM_OK)
				{
					gprsState = GPRS_LINK ;						
				}
				else
				{
					gprsState = GPRS_SINGEL_CLOSE;
					serverLSRLog[cureTcpLinkChannel].setTry++;
					if(serverLSRLog[cureTcpLinkChannel].setTry >=GPRS_RETRY_TIMES_SET)
					{
						rxResult->contuineSendErrorCounter ++;	
						serverLSRLog[cureTcpLinkChannel].setTry = 0x00;				
						serverLSRLog[cureTcpLinkChannel].allTry ++;							
					}
				}			
				break;
			}		
			case GPRS_LINK  :
			{
				if(serverLSRLog[cureTcpLinkChannel].allTry < FIREWARE_TASH_MAX_ERROR_COUNTER)
				{			
					if(gprsLink((uint8_t *)tcpServerAddrBuf[TCP_SERVER_INSENTEK].sendAddressBuf,LINK_NUM_INSENTEK)==COMM_OK)
					{
						serverLSRLog[cureTcpLinkChannel].linkTry = 0x00;                                               
						gprsState = GPRS_SEND_DATA_READY;
					}
					else
					{
						serverLSRLog[cureTcpLinkChannel].linkTry ++;
						gprsState = GPRS_SINGEL_CLOSE;
						serverLSRLog[cureTcpLinkChannel].linkErrorCounter ++; 
					#if GPRS_DEBUG>0
						printf("\n\r link server error counter %d,allTry:%d\n\r",serverLSRLog[cureTcpLinkChannel].linkErrorCounter,serverLSRLog[cureTcpLinkChannel].allTry);
					#endif
						if(serverLSRLog[cureTcpLinkChannel].linkTry >= GPRS_TIMES_TRY)
						{	
							serverLSRLog[cureTcpLinkChannel].allTry++;						
							serverLSRLog[cureTcpLinkChannel].linkTry= 0x00;//200429
						}									
					}
				}
//				if(serverLSRLog[cureTcpLinkChannel].allTry >= FIREWARE_TASH_MAX_ERROR_COUNTER)		//220331			   
//				{ //�˳�ѭ��
//					txRxTaskState =	GPRS_SLEEP;
//					gprsState 		=	GSM_MOUDLE_IDLE;	
//				}
				break;
			}
			case GPRS_SEND_DATA_READY:
			{
				packHeadLength	= fireWareDataPacketHead(packetHeadBuf,curFireWare,curGetFireWare ,curEndFireWare);
				gprsState 			= GPRS_SEND;
			}
			case  GPRS_SEND :
			{
			#if(GPRS_MULTI_LINK>0)	
//				if(curGetFireWare==0)
				{
					while(readReceivelen(LINK_NUM_INSENTEK)>0)//20210222
					{
						findRxGprsData(sensorBuf,&rxDataLength,LINK_NUM_INSENTEK);
					}
					rxDataLength = 0;
				}
			#endif
				clearBufPtr();
//				sendCmdAck	=	gprsSend((uint8_t *)packetHeadBuf,0,REQUEST_FIREWARE_DATA,LINK_NUM_INSENTEK);
				if(gprsSend((uint8_t *)packetHeadBuf,0,REQUEST_FIREWARE_DATA,LINK_NUM_INSENTEK)==COMM_OK)  //���ʹ���������
				{
					serverLSRLog[cureTcpLinkChannel].sendTry = 0x00;
					gprsState = GPRS_RX;				
				}
				else
				{
					gprsState = GPRS_SINGEL_CLOSE;
					serverLSRLog[cureTcpLinkChannel].sendTry++;					
					serverLSRLog[cureTcpLinkChannel].sendErrorCounter ++;
				#if GPRS_DEBUG>0
					printf("\n\r send data error counter %d \n\r",serverLSRLog[cureTcpLinkChannel].sendErrorCounter);
				#endif
					if(serverLSRLog[cureTcpLinkChannel].sendTry >=GPRS_TIMES_TRY)
					{
						rxResult->contuineSendErrorCounter ++;	
						//if(serverLSRLog[cureTcpLinkChannel].sendTry > 0x07)//200429
						{                             
							serverLSRLog[cureTcpLinkChannel].sendTry  =0x00;
						}
						serverLSRLog[cureTcpLinkChannel].allTry++;						
					}				
				}
				break;
			}
			case GPRS_RX   :
			{
				if(rxFireWare((uint8_t*)sensorBuf,&rxDataLength,&tcpResponse,readLength,&curFireWare.fireWareLength)==COMM_OK)
				{
					if(tcpResponse.responseCode.i == 10000)
					{						
						//�̼����ݴ洢
					#if GPRS_DEBUG>0
						printf("\n\r read firwmare length %d\n\r",rxDataLength);
					#endif
					#if(SWRESET_CTR>0)			
						SWRESET_TIMER_INIT();
					#endif
						firewareWrite((uint8_t*)sensorBuf,&fireWareAddr,rxDataLength);
						if(firewareCheck(fireWareAddr)!=1)//210705
						{
							//��ȡ�̼�ʧ�ܣ��˳�ѭ��
							txRxTaskState =	GPRS_SLEEP;
							gprsState 		=	GSM_MOUDLE_IDLE;
							rxResult->firewareFlag = FIREWARE_IDLE;
							break;
						}
//						delay_GprsMs(10);
						//�´�ȡ�������ж�
						curGetFireWare = curEndFireWare;
						if((curFireWare.fireWareLength - curEndFireWare)> FIREWARE_PACKET_SIZE)
						{
							curEndFireWare += FIREWARE_PACKET_SIZE;
						}
						else
						{
							curEndFireWare =curFireWare.fireWareLength; 
						}
						readLength = curEndFireWare - curGetFireWare;  
					#if GPRS_DEBUG>0
						printf("\n\r get firwmare length %d\n\r",curGetFireWare);
					#endif
						if(curGetFireWare == curFireWare.fireWareLength)
						{ //��ȡ�̼��ɹ����˳�ѭ��
							txRxTaskState =	GPRS_SLEEP;
							gprsState 		=	GSM_MOUDLE_IDLE;
							rxResult->firewareFlag = FIREWARE_UPDATA;
						}
						gprsState = GPRS_SEND_DATA_READY;
						break;
					}
					else
					{       
						gprsState = GPRS_SINGEL_CLOSE;
						switch(tcpResponse.responseCode.i)
						{
							case ATUTHCODE_ERROR :
							case NETWORK_ERROR :
							case SERVER_ERROR :											
							{
							//�������������˳�����ѭ������
								txRxTaskState =	GPRS_SLEEP;
								gprsState 		=	GSM_MOUDLE_IDLE;
								break;
							}
							case 100 :
							case 101 :
							case 102 :
							{
//								gprsState = GPRS_RX ;//230110_����
								break;
							}                                 
							default :
							{
								txRxTaskState =	GPRS_SLEEP;
								gprsState 		=	GSM_MOUDLE_IDLE;				 
								break;
							}
						}
						serverLSRLog[cureTcpLinkChannel].rxDataTry ++;//200429
						if(serverLSRLog[cureTcpLinkChannel].rxDataTry >= GPRS_TIMES_TRY)//200429
						{
							serverLSRLog[cureTcpLinkChannel].allTry++;
							serverLSRLog[cureTcpLinkChannel].rxDataTry= 0x00;
						}
					}
				}
				else
				{
					gprsState = GPRS_SINGEL_CLOSE;
					serverLSRLog[cureTcpLinkChannel].rxErrorCounter ++;
				#if GPRS_DEBUG>0
					printf("\n\r rx server data error counter %d\n\r",serverLSRLog[cureTcpLinkChannel].rxErrorCounter);
				#endif
					serverLSRLog[cureTcpLinkChannel].rxTry ++;
					if(serverLSRLog[cureTcpLinkChannel].rxTry >= GPRS_TIMES_TRY)
					{
						serverLSRLog[cureTcpLinkChannel].allTry++;
					//if(serverLSRLog[cureTcpLinkChannel].rxTry > 0x06)
						serverLSRLog[cureTcpLinkChannel].rxTry= 0x00;
					}
				}
			#if GPRS_DEBUG>0
				printf("\n\r server Response code %d\n\r\n\r",tcpResponse.responseCode.i);
			#endif
				break;
			}
			case GPRS_SINGEL_CLOSE :
			{
				gprsState = GPRS_CLOSE;
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				for(tryTimes = 0;tryTimes<2;tryTimes++)
			#endif
				{
					if(closeSingleTcp(LINK_NUM_INSENTEK)==COMM_OK)
					{
						gprsState = GPRS_LINK;
						printf("\r\n socket closed.\r\n");
						break;
					}
				}
				break;
			}
			case GPRS_CLOSE :
			{
			#if(MQTT_USE_CTR==0)
				(void)closeTcp();
			#endif
				gprsState = GPRS_SET;
				break;
			}
			default :
			{
				gprsState = GPRS_SINGEL_CLOSE;
				break;
			}
		}
		if(serverLSRLog[cureTcpLinkChannel].allTry >= FIREWARE_TASH_MAX_ERROR_COUNTER)		//220331		   
		{ //�˳�ѭ��
			txRxTaskState =	GPRS_SLEEP;
			gprsState 		=	GSM_MOUDLE_IDLE;	
		}
	}
	SPI_Flash_PowerDown();
	power331Off();
}

int8_t gprsConfigRequest(httpConfig *rxResult)
{
	int8_t		configResult				= -1;
	uint8_t		txRxTaskState				=	GPRS_RUN;
	uint8_t		gprsDataTypeFlag		=	CONFIG_REQUEST_DATA;
	uint16_t	gprsSendBufLength 	= 0x00;
	uint16_t	gprsSendDataLength 	= 0x00;
	uint8_t		gprsSetTry 	= 0;
	uint8_t		gprsLinkTry = 0;
	uint8_t		gprsSendTry	= 0;
	uint8_t		gprsRxTry		= 0;
	uint8_t		gprsAllTry 	= 0;
//	uint8_t   sendCmdAck	=	0;

	rxResponse	tcpResponse;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	uint8_t		tryTimes	=	0;
#endif
#if GPRS_DEBUG > 0
	printf("\n\r request config task,powerTime:%d\n\r",gprsPowerOnTime);
#endif
	serverBufInt();
	if(devRegCtr != DEV_REG)
	{
		gprsDataTypeFlag = DEV_REG_DATA;
	}
	cureTcpLinkChannel 	= TCP_SERVER_INSENTEK;
	
	gprsState = GPRS_SINGEL_CLOSE;//GPRS_LINK;
	if(rxResult->getIpFlag < 1)
	{
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
		gprsState =GPRS_DNS2IP;
	#else
		gprsState = GPRS_LINK;
	#endif
	}
	while(txRxTaskState == GPRS_RUN) 
	{
		switch(gprsState)
		{
			case GPRS_SEND_DATA_READY:
			{
				switch(gprsDataTypeFlag)
				{
					case DEV_REG_DATA :
					{
					//MAIN�����ļ�װ��
						gprsSendBufLength 	=	devRegDataPackeg((char *)sensorBuf,0,cureTcpLinkChannel);//
					//HTTPͷ����������װ��
						packHeadLength 			= sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,DEV_REG_DATA); 
						gprsSendDataLength	= gprsSendBufLength ;				
						gprsState 					=	GPRS_SEND;	
						break;
					}		
					case CONFIG_REQUEST_DATA :
					{
						gprsSendBufLength 	= devConfigPacket(configValue.configVersion,sensorBuf);
						packHeadLength 			= sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,CONFIG_REQUEST_DATA); 
						gprsSendDataLength 	= gprsSendBufLength ;								
						gprsState 					=	GPRS_SEND;	
						break;
					}
					case DEV_RTC_CAL_DATA:
					{
						gprsSendBufLength 	= devRtcPacket(sensorBuf);
						packHeadLength 			= sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,DEV_RTC_CAL_DATA); 
						gprsSendDataLength 	= gprsSendBufLength ;						
						gprsState 					=	GPRS_SEND;											
						break;
					}
					default:break;					
				}
				break;
			}			
			case GPRS_SET:
			{
			#if GPRS_DEBUG > 0
				printf("\n\r enter server %d set task\n\r",cureTcpLinkChannel);
			#endif
				if(gprsSet(&serverLSRLog[cureTcpLinkChannel])==COMM_OK)
				{
					gprsState =GPRS_LINK;
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
						if(rxResult->getIpFlag < 1)gprsState =GPRS_DNS2IP;
				#endif
				}
				else
				{
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
					delay_GprsMs(1000);
					gprsState = GPRS_SINGEL_CLOSE;					
				#else					
					gprsState = GPRS_LINK;
				#endif
					gprsSetTry++;	
					if(gprsSetTry >=GPRS_RETRY_TIMES_SET)
					{
						txRxTaskState = GPRS_SLEEP;
						gprsState 		= GSM_MOUDLE_IDLE;
					}
				}							
				break;
			}
			case GPRS_DNS2IP :
			{
			//	clearBufPtr();
			#if GPRS_DEBUG > 0
				printf("\n\r find %s ip 1\n\r",default_sever_addr);
			#endif
			
				if(findServerIpAddr((uint8_t *)tcpServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,default_sever_addr,default_sever_port,LINK_NUM_INSENTEK)==COMM_OK)				
				{
					tcpServerAddrBuf[cureTcpLinkChannel].userFlag	= TCP_SERVER_ADDRESS_USE_FLAG;
					tcpServerAddrBuf[cureTcpLinkChannel].packType 	= EXT_PACKET;
					tcpServerAddrBuf[cureTcpLinkChannel].sendPort 	= 54862;
					gprsState = GPRS_LINK;
					rxResult->getIpFlag++;
				#if GPRS_DEBUG > 0
					printf("\n\r find %s ip ok\n\r",default_sever_addr);
				#endif	
				}
				else
				{
					gprsSetTry ++;
					gprsState = GPRS_SINGEL_CLOSE;
					if(gprsSetTry >=GPRS_RETRY_TIMES_SET)
					{			
						txRxTaskState = GPRS_SLEEP;
						gprsState 		=	GSM_MOUDLE_IDLE;
					}
				#if GPRS_DEBUG > 0
					printf("\n\r find %s ip error\n\r",default_sever_addr);
				#endif	
				}		
				break;
			}
			case GPRS_LINK:
			{			
			#if GPRS_DEBUG > 0
				printf("\n\r server %d link task try %d",cureTcpLinkChannel,gprsLinkTry);
				printf(" userFlag %d \n\r",tcpServerAddrBuf[cureTcpLinkChannel].userFlag);
			#endif
				if(tcpServerAddrBuf[cureTcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
				{
					if(gprsLinkTry < 0x04)
					{
					#if GPRS_DEBUG > 0
						printf("\n\r enter server %d link task1\n\r",cureTcpLinkChannel);	
					#endif
					#if(GPRS_MULTI_LINK>0)						
//						if(readReceiveMode()==0)setReceiveDataMode();			
					#endif
						if(gprsLink((uint8_t *)tcpServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,LINK_NUM_INSENTEK)==COMM_OK)
						{							
							gprsState =	GPRS_SEND_DATA_READY;	
							if(rxResult->getIpFlag < 1)rxResult->getIpFlag++;
						}
						else
						{
							gprsLinkTry ++;
							gprsState = GPRS_SINGEL_CLOSE;							
						#if GPRS_DEBUG > 0
							printf("\n\r link server %d error %d\n\r",cureTcpLinkChannel,gprsLinkTry);
						#endif						
						}					
					}
					else
					{
						gprsLinkTry = 0;
						cureTcpLinkChannel ++;
					}
				}
				else
				{
					cureTcpLinkChannel ++;
				}				
				break;
			}
			case  GPRS_SEND :
			{			
				//clearBufPtr();
//				sendCmdAck	=	gprsSend(sensorBuf,gprsSendDataLength,OTHER_DATA,LINK_NUM_INSENTEK);
				if(gprsSend(sensorBuf,gprsSendDataLength,OTHER_DATA,LINK_NUM_INSENTEK) == COMM_OK)  //���ʹ���������				
				{
					gprsState = GPRS_RX;			
				}
				else
				{					
					gprsSendTry++;
					gprsState =	GPRS_SINGEL_CLOSE;
					
				#if GPRS_DEBUG > 0
					printf("\n\r send data error %d \n\r",gprsSendTry);
				#endif
					if(gprsSendTry >=1)//GPRS_TIMES_TRY
					{
						gprsAllTry++;
						gprsSendTry = 0;						
					}
				}
				break;
			}
			case GPRS_RX:
			{
				if(rxGprsTcp(&tcpResponse)==COMM_OK)
				{
					if(tcpResponse.responseCode.i ==10000)
					{						
						rxResult->contuineSendErrorCounter = 0x00;
					#if GPRS_DEBUG > 0
						printf("\n\r receive server data ok\n\r");
					#endif	
						switch(gprsDataTypeFlag)
						{							
							case DEV_REG_DATA :
							{
								gprsState =  GPRS_REG_DATA_RE ;
								break;
							}							
							case CONFIG_REQUEST_DATA :
							{
								gprsState =  GPRS_CONFIG_DATA_RE ;
								break;
							}				
							case DEV_RTC_CAL_DATA :  
							{
								gprsState =  GPRS_RTC_CAL_RE ;
								break;
							}											
							default :
							{
								txRxTaskState = GPRS_SLEEP;
								gprsState 		=	GSM_MOUDLE_IDLE;                          
								break;
							}
						}
					}
					else
					{
						gprsState = GPRS_SINGEL_CLOSE;//GPRS_SEND_DATA_READY;
						gprsRxTry++;
						if(gprsRxTry >= 1)//GPRS_TIMES_TRY
						{
							gprsRxTry	=	0;
							gprsAllTry ++;							
							//gprsState = GPRS_SINGEL_CLOSE;
						}
						switch(tcpResponse.responseCode.i)
						{
							case TIME_ERROR :	
								break;
							case SN_ERROR:	
							case ATUTHCODE_ERROR :
							case DEV_NOT_EXIT :
							case SERVER_ERROR :
							{
								gprsDataTypeFlag =	DEV_REG_DATA;
								break;
							}
							case 998 :							
							case 999 :	
							{
//								gprsState = GPRS_RX;//230110_����
								break;
							}
							case NETWORK_ERROR :
								
							default :
							{
								//gprsState = GPRS_SINGEL_CLOSE;
								break;
							}
						}
					}
				#if GPRS_DEBUG > 0
					printf("\n\r data type 0x%0.2X,server Response code %d\n\r",gprsDataTypeFlag,tcpResponse.responseCode.i);
				#endif			
					break;
				}
				else
				{
					gprsState = GPRS_SINGEL_CLOSE;
					gprsRxTry++;
					if(gprsRxTry >= 1)//GPRS_TIMES_TRY
					{
//						gprsState 	= GPRS_SINGEL_CLOSE;						
						gprsRxTry = 0;	
						gprsAllTry++;						
					}
				#if GPRS_DEBUG > 0
					printf("\n\r receive server data error %d\n\r",tcpResponse.responseCode.i);				
				#endif
				}			
				break;
			}
			case GPRS_REG_DATA_RE :
			{
				if(DEV_REG_DATA == gprsDataTypeFlag)
				{		
					findAuthCode();
					gprsDataTypeFlag =	CONFIG_REQUEST_DATA;//DEV_RTC_CAL_DATA;
				}			
				gprsState = GPRS_SEND_DATA_READY;
				break;
			}
			case GPRS_CONFIG_DATA_RE :
			{
				//�ı����ݰ�����
			//	gprsDataTypeFlag =	SENSOR_DATA;		
				//���ݷ��ͳɹ���־
				gprsConfigTask(rxResult);
				//�Զ�У׼��ʱ��ÿ��12��
				if(((calCounter%6)== 0x00)||(calCounter== 0x01))
				{
					rxResult->rtcSetFlag=FUNCITONG_TURN_ON;	
				}    			
				if( rxResult->rtcSetFlag==FUNCITONG_TURN_ON)
				{
				//�ı����ݰ�����
					gprsDataTypeFlag 	=	DEV_RTC_CAL_DATA;
					gprsState 				= GPRS_SEND_DATA_READY;
				}
				else
				{
					txRxTaskState = GPRS_SLEEP;
					gprsState 		=	GSM_MOUDLE_IDLE;
				}
				break;
			}		
			case GPRS_RTC_CAL_RE :
			{
				//��ȡ����
				if(!(findRtcTime()))
				{
					rxResult->rtcSetFlag = CHANGE_FLAG;
					//�˳�
					txRxTaskState = GPRS_SLEEP;
					gprsState 		=	GSM_MOUDLE_IDLE;
				}
				else
				{				
					gprsState = GPRS_SEND_DATA_READY;
					gprsRxTry++;
					if(gprsRxTry >= 1)//GPRS_TIMES_TRY
					{
						gprsRxTry	=	0;
						gprsAllTry ++;
						gprsState = GPRS_SINGEL_CLOSE;
					}					
				}
				break;
			}
			case GPRS_SINGEL_CLOSE :
			{
				gprsState = GPRS_CLOSE;
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				for(tryTimes = 0;tryTimes<2;tryTimes++)
			#endif
				{
					if(closeSingleTcp(LINK_NUM_INSENTEK)==COMM_OK)
					{
						gprsState = GPRS_LINK;
						printf("\r\n socket closed.\r\n");
						break;
					}
				}			
				break;
			}
			case GPRS_CLOSE :
			{
			#if(MQTT_USE_CTR==0)
				(void)closeTcp();
			#endif
				gprsState = GPRS_SET;
				break;
			}			
			default:break;
		}	
		if(gprsAllTry>=GPRS_TIMES_ALLTRY)
		{
			cureTcpLinkChannel ++;
		}
		if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
		{
			txRxTaskState = GPRS_SLEEP;
			gprsState 		= GSM_MOUDLE_IDLE;			  
		}
	}
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	getRssi();
	if((rssiValue <10)||(rssiValue >31))//210705
	{
		delay_GprsS(0,30);
		getRssi();
	}
#endif
//	clearBuf();
	return configResult;
}

void gprsSendRxTask(httpConfig *rxResult)
{
	DS1302_TIME sendTime;
	/*��ǰ����TCP*/
#if(GPRS_CONFIG_CTR>0)
	int8_t	configResult	= -1;
#endif
	int8_t readFlashDataResult = -1;
	uint16_t gprsSendBufLength = 0x00;
	uint16_t gprsSendDataLength = 0x00;
	uint16_t packLengthAddr = 0x00;
	uint8_t txRxTaskState 	=	GPRS_RUN;
	uint8_t gprsDataTypeFlag =  SENSOR_DATA;
	uint8_t curSendPacket =0x00;
	uint16_t signSendCounter = 0x00;
//	uint8_t   sendCmdAck			=	0;
	rxResponse tcpResponse;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
	uint8_t	tryTimes	=	0;
#endif	
#if GPRS_DEBUG > 0
	printf("\n\r gprsSendRxTask powerTime:%d\n\r",gprsPowerOnTime);
#endif
	readDs1302Time(&sendTime);
	serverBufInt();	
	rxResult->rtcErrorFlag = UNCHANGE_FLAG;
	if(rxResult->requestData ==0x00)
	{
		rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
		rxResult->requestDataType = REQUEST_DATA_IDLE;
	}
//	if(rxResult->rtcSetFlag == FUNCITONG_TURN_ON)
//	{
//		gprsDataTypeFlag = DEV_RTC_CAL_DATA;
//	}
#if(GPRS_CONFIG_CTR>0)
	gprsDataTypeFlag	=	CONFIG_REQUEST_DATA;
#endif
	if(devRegCtr != DEV_REG)
	{
		gprsDataTypeFlag = DEV_REG_DATA;
	}  

	cureTcpLinkChannel 	= TCP_SERVER_INSENTEK;
#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
	gprsState = GPRS_SINGEL_CLOSE;//GPRS_LINK;
#else
	gprsState = GPRS_LINK;
#endif
	rxResult->sendErrorCounter++;
	while(GPRS_RUN==txRxTaskState ) 
	{
		switch(gprsState) 
		{
		  case GPRS_SEND_DATA_READY :
		  {
				switch(gprsDataTypeFlag)
				{
					case SENSOR_DATA :
					{
						if(unSendDataPacket > 0x00)	
						{				  
							/*gprsMultSensorDataLoad() send data include QX*/
							readFlashDataResult =  gprsMultSensorDataLoad((uint8_t *)sensorBuf,curGprsSendAddr,&gprsSendBufLength,unSendDataPacket,&curSendPacket,&packLengthAddr); 		      
							if(readFlashDataResult == 1)
							{
								gprsSendBufLength = gprsGsmStatusLoad((uint8_t *)sensorBuf,gprsSendBufLength);	
								if(rxResult->simCardReadOkFlag  == FUNCITONG_TURN_ON)
								{
									gprsSendBufLength = loadSimCardInf((uint8_t *)sensorBuf,gprsSendBufLength);	
								}
								gprsState = GSM_SEND_DATA_HEARD;
							}
							else
							{
								if(unSendDataPacket > 0x00)
								{
									curGprsSendAddr = sensorDataReadAddr;//221213
									curGprsSendAddr  += SENSOR_DATA_STORE_SIZE;
									if(curGprsSendAddr >= SENSOR_DATA_END_ADDR)
									{
										curGprsSendAddr = SENSOR_DATA_BASE_ADDR;
									}	
//									unSendDataPacket --;
								}
								if(curGprsSendAddr == sensorDataWriteAddr){unSendDataPacket = 0;}//221213
							}
						}
						else
						{
							curGprsSendAddr	=	sensorDataWriteAddr;//210705
							//�˳�
							if(rxResult->requestDataType == REQUEST_DATA_BY_COUNTER)
							{
								gprsState =   FIND_HISTORY_DATA;		   
								rxResult->requestDataReadAddress = curGprsSendAddr;
							}
							if (rxResult->historyDataFlag == FUNCITONG_TURN_ON)
							{
								gprsState =   GPRS_SEND_DATA_READY;
								gprsDataTypeFlag =	DEV_HISTORY_DATA;						
							}
							if(rxResult->requestDataType == REQUEST_DATA_IDLE)
							{
								rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
								cureTcpLinkChannel ++;					   
							}                     
						}																 
						if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
						{
							//�˳�
							gprsState = 	GSM_MOUDLE_IDLE;
							txRxTaskState = GPRS_SLEEP;
						}
						break;
					}
					case DEV_REG_DATA :
					{
						//MAIN�����ļ�װ��
						gprsSendBufLength =	devRegDataPackeg((char *)sensorBuf,0,cureTcpLinkChannel);
						//HTTPͷ����������װ��
						packHeadLength = sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,DEV_REG_DATA); 
						gprsSendDataLength = gprsSendBufLength ;
						// gprsState = GPRS_LINK;	
						gprsState = 	GPRS_SEND;	
						break;
					}
				#if(GPRS_CONFIG_CTR>0)	
					case CONFIG_REQUEST_DATA :
					{
						gprsSendBufLength = devConfigPacket(configValue.configVersion,(uint8_t *)sensorBuf);
						packHeadLength = sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,CONFIG_REQUEST_DATA); 
						gprsSendDataLength = gprsSendBufLength ;
						// configVersionFlag = REQUEST_CONFIG;						
						gprsState = 	GPRS_SEND;	
						break;										
					}
				#endif
					case DEV_RTC_CAL_DATA :
					{
						gprsSendBufLength = devRtcPacket((uint8_t *)sensorBuf);
						packHeadLength = sensorDataPacketHead(packetHeadBuf,gprsSendBufLength,DEV_RTC_CAL_DATA); 
						gprsSendDataLength = gprsSendBufLength ;						
						gprsState = 	GPRS_SEND;											
						break;
					}
					case DEV_HISTORY_DATA :
					{
						if(rxResult->requestData >0)
						{
							readFlashDataResult =  gprsMultSensorDataLoad((uint8_t *)sensorBuf,rxResult->requestDataReadAddress,&gprsSendBufLength,rxResult->requestData,&curSendPacket ,&packLengthAddr); 		      
							if(readFlashDataResult == 1)
							{
								gprsSendBufLength = gprsGsmStatusLoad((uint8_t *)sensorBuf,gprsSendBufLength);	                                                     	
								gprsState = GSM_SEND_DATA_HEARD;
							// gprsState = 	GPRS_LINK;	
							}
							else
							{
								if(rxResult->requestData > 0x00)
								{
									rxResult->requestDataReadAddress = sensorDataReadAddr;//221213
									rxResult->requestDataReadAddress += SENSOR_DATA_STORE_SIZE;
									if( rxResult->requestDataReadAddress>=SENSOR_DATA_END_ADDR) rxResult->requestDataReadAddress = SENSOR_DATA_BASE_ADDR;//221213
//									rxResult->requestData --;
								}
								if(rxResult->requestDataReadAddress == sensorDataWriteAddr){rxResult->requestData = 0;}//221213
							}
						}
						else
						{
							rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
							rxResult->requestDataType = REQUEST_DATA_IDLE;
							//�˳�
							txRxTaskState = GPRS_SLEEP;
							gprsState = 	GSM_MOUDLE_IDLE;
						}
						break;
					}
					default :
					{
						gprsDataTypeFlag	=	SENSOR_DATA;
						break;
					}					
				}													  
				break;
			}	
			case GPRS_SET:
			{
			#if GPRS_DEBUG > 0
				printf("\n\r enter server %d set task\n\r",cureTcpLinkChannel);
			#endif
				if(gprsSet(&serverLSRLog[cureTcpLinkChannel])==COMM_OK)
				{
					gprsState =GPRS_LINK;
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
					if(rxResult->getIpFlag < 1)
					{
						gprsState =GPRS_DNS2IP;					
					}
				#endif
				}
				else
				{
				#if(SWRESET_CTR>0)				
					//SWRESET_TIMER_INIT();
				#endif
					serverLSRLog[cureTcpLinkChannel].setTry++;					
				#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
					delay_GprsMs(1000);
					gprsState = GPRS_SINGEL_CLOSE;//GPRS_CLOSE;
					if(serverLSRLog[cureTcpLinkChannel].setTry >=GPRS_RETRY_TIMES_SET)//3
					{
						rxResult->contuineSendErrorCounter ++;	
						serverLSRLog[cureTcpLinkChannel].setTry = 0x00;
						txRxTaskState = GPRS_SLEEP;
						gprsState 		= GSM_MOUDLE_IDLE;
					}
				#else
					if(serverLSRLog[cureTcpLinkChannel].setTry >=GPRS_TIMES_TRY)
					{
						rxResult->contuineSendErrorCounter ++;	
						serverLSRLog[cureTcpLinkChannel].setTry = 0x00;					
						serverLSRLog[cureTcpLinkChannel].allTry ++;							
					}
					gprsState = GPRS_LINK;
				#endif
				}							
				break;
			}
			case GPRS_DNS2IP :
			{
				//clearBufPtr();
			#if GPRS_DEBUG > 0
				printf("\n\r find %s ip\n\r",default_sever_addr);
			#endif
				if(findServerIpAddr((uint8_t *)tcpServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,default_sever_addr,default_sever_port,LINK_NUM_INSENTEK)==COMM_OK)	
					//if(findServerIp(&tcpServerAddrBuf[TCP_SERVER_INSENTEK])==COMM_OK)
				{
					gprsState = GPRS_LINK;
					rxResult->getIpFlag ++;
//					rxResult->linkErrorCouner = 0;				
				#if GPRS_DEBUG > 0
					printf("\n\r find %s ip ok\n\r",default_sever_addr);
				#endif						
				}
				else
				{
					serverLSRLog[cureTcpLinkChannel].setTry ++;
					gprsState = GPRS_SINGEL_CLOSE;//GPRS_CLOSE;
					if(serverLSRLog[cureTcpLinkChannel].setTry >=GPRS_RETRY_TIMES_SET)//3
					{
						rxResult->contuineSendErrorCounter ++;
						serverLSRLog[cureTcpLinkChannel].setTry = 0;	
						txRxTaskState = GPRS_SLEEP;
						gprsState 		=	GSM_MOUDLE_IDLE;
					}
				#if GPRS_DEBUG > 0
					printf("\n\r find %s ip error\n\r",default_sever_addr);
				#endif
				}
		#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
			#if(GPRS_PING_CTR>0)
				gprsPing("www.baidu.com",13);
				gprsPing("%s",strlen(default_sever_addr));
			#endif
		#endif
				break;
			}						
			case GPRS_LINK:
			{
			#if GPRS_DEBUG > 0
				printf("\n\r server %d link task try %d",cureTcpLinkChannel,serverLSRLog[cureTcpLinkChannel].allTry);
				printf(" userFlag %d \n\r",tcpServerAddrBuf[cureTcpLinkChannel].userFlag);
			#endif
				if(tcpServerAddrBuf[cureTcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
				{
					if(serverLSRLog[cureTcpLinkChannel].allTry < 0x04)
					{
					#if GPRS_DEBUG > 0
						printf("\n\r enter server %d link task\n\r",cureTcpLinkChannel);	
					#endif	
					#if(GPRS_MULTI_LINK>0)						
//						if(readReceiveMode()==0)setReceiveDataMode();			
					#endif
						if(gprsLink((uint8_t *)tcpServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,LINK_NUM_INSENTEK)==COMM_OK)
						{
							serverLSRLog[cureTcpLinkChannel].linkTry = 0x00;
							gprsState = GPRS_SEND_DATA_READY;
						}
						else
						{							                                        
							serverLSRLog[cureTcpLinkChannel].linkTry ++;
							gprsState = GPRS_SINGEL_CLOSE;
							serverLSRLog[cureTcpLinkChannel].linkErrorCounter ++; 
							printf("\n\r link server error counter %d\n\r",serverLSRLog[cureTcpLinkChannel].linkErrorCounter);
							if(serverLSRLog[cureTcpLinkChannel].linkTry >= 1)//GPRS_TIMES_TRY
							{
								rxResult->contuineSendErrorCounter ++;	
								serverLSRLog[cureTcpLinkChannel].allTry++;
								serverLSRLog[cureTcpLinkChannel].linkTry= 0x00;
							}					
						}
					}  //end if(serverLSRLog[cureTcpLinkChannel].allTry <=4)
					else
					{
						cureTcpLinkChannel ++;					
					}
				}
				else
				{
					cureTcpLinkChannel ++;
				}
								  
				if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
				{
				//�˳�
					txRxTaskState = GPRS_SLEEP;
					gprsState = 	GSM_MOUDLE_IDLE;			  
				}
				break;
			}

			case GSM_SEND_DATA_HEARD :
			{					   
				gprsSendDataLength = gprsTcpStatusDataLoad((uint8_t *)sensorBuf,cureTcpLinkChannel,gprsSendBufLength,packLengthAddr);
			
				packHeadLength = sensorDataPacketHead(packetHeadBuf,gprsSendDataLength,SENSOR_DATA);                                      
				printf("\n\r send data length1 %d \n\r",gprsSendBufLength);
				printf("\n\r send data length2 %d \n\r",gprsSendDataLength);
				gprsState = GPRS_SEND; 		                     									  
				break;
			}
			case  GPRS_SEND :
			{
				if(gprsLinkStatus(LINK_NUM_INSENTEK) !=COMM_LINK_OK) //230110
				{
					if(gprsLink((uint8_t *)tcpServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,LINK_NUM_INSENTEK)!=COMM_OK)
					{
						gprsState = GPRS_CLOSE;
					#if GPRS_DEBUG > 0		
						printf("\r\ngprs link closed.\r\n");
					#endif
						break;
					}
				}
		//		clearBufPtr();
//				sendCmdAck	=	gprsSend((uint8_t *)sensorBuf,gprsSendDataLength,OTHER_DATA,LINK_NUM_INSENTEK);
				if(gprsSend((uint8_t *)sensorBuf,gprsSendDataLength,OTHER_DATA,LINK_NUM_INSENTEK) == COMM_OK)  //���ʹ���������
				{
					serverLSRLog[cureTcpLinkChannel].sendTry = 0x00;
					gprsState = GPRS_RX;
					signSendCounter ++;				
				}
				else
				{        					
					gprsState =GPRS_SINGEL_CLOSE;
					serverLSRLog[cureTcpLinkChannel].sendTry++;
					serverLSRLog[cureTcpLinkChannel].sendErrorCounter ++;
					printf("\n\r send data error counter %d \n\r",serverLSRLog[cureTcpLinkChannel].sendErrorCounter);
					if(serverLSRLog[cureTcpLinkChannel].sendTry >=1)//GPRS_TIMES_TRY
					{					
						rxResult->contuineSendErrorCounter ++;		
						serverLSRLog[cureTcpLinkChannel].sendTry  =0x00;
						serverLSRLog[cureTcpLinkChannel].allTry++;	
					}					
				}
				break;
			}
			case GPRS_RX   :
			{
				//delay_GprsMs(100);
				if(rxGprsTcp(&tcpResponse)==COMM_OK)
				{
					if(tcpResponse.responseCode.i ==10000)
					{
					#if(SWRESET_CTR>0)
						SWRESET_TIMER_INIT();
					#endif
						simCardErrorCounter	= 0;
						netGsmErrorCounter	=	0;
					// curDataErrorCounter  = 0x00;
						serverLSRLog[cureTcpLinkChannel].dataOkCounter ++;
						rxResult->contuineSendErrorCounter = 0x00;
					#if GPRS_DEBUG > 0
						printf("\n\r receive server data ok\n\r");
					#endif	
						switch(gprsDataTypeFlag)
						{
							case SENSOR_DATA :
							{
								gprsState =  GPRS_SENSOR_DATA_RE ;
								break;
							}
							case DEV_REG_DATA :
							{
								gprsState =  GPRS_REG_DATA_RE ;
								break;
							}
						#if(GPRS_CONFIG_CTR>0)
							case CONFIG_REQUEST_DATA :
							{
								gprsState =  GPRS_CONFIG_DATA_RE ;
								break;
							}
						#endif
							case DEV_RTC_CAL_DATA :  
							{
								gprsState =  GPRS_RTC_CAL_RE ;
								break;
							}
							case DEV_HISTORY_DATA :  
							{
								gprsState =  GPRS_HISTORY_DATA ;
								break;
							}								
							default :
							{
								txRxTaskState = GPRS_SLEEP;
								gprsState = 	GSM_MOUDLE_IDLE;                          
								break;
							}
						}
					}
					else
					{
						gprsState = GPRS_SINGEL_CLOSE;//GPRS_SEND_DATA_READY;
						serverLSRLog[cureTcpLinkChannel].rxDataTry++;
						if(serverLSRLog[cureTcpLinkChannel].rxDataTry >=1)//0x02
						{
							serverLSRLog[cureTcpLinkChannel].rxDataTry = 0;
							serverLSRLog[cureTcpLinkChannel].allTry++;
							serverLSRLog[cureTcpLinkChannel].rxDataErrorCounter ++;
							
//							cureTcpLinkChannel ++;
//							gprsState = GPRS_SINGEL_CLOSE;
						}						          	
						switch(tcpResponse.responseCode.i)
						{
							case TIME_ERROR :
							{
								/* Phase 3: don't skip failed records — leave for retry on next connection */
								if(gprsDataTypeFlag == SENSOR_DATA )
								{
									if(unSendDataPacket > 0x00)
									{
										/* record is NOT advanced — it stays in cache for retry */
									}
								}
								//����RTCʱ�ӱ�־λ������RTCʱ��У׼		 
								if(rxResult->rtcErrorFlag != CHANGE_FLAG)
								{
									rxResult->rtcErrorFlag = CHANGE_FLAG;
									gprsDataTypeFlag =	DEV_RTC_CAL_DATA;	
								}
								if(gprsDataTypeFlag == DEV_HISTORY_DATA )	
								{
									if(rxResult->requestData  >0x00)
									{
										rxResult->requestDataReadAddress += SENSOR_DATA_STORE_SIZE;
										rxResult->requestData --;
									}
								}
								break;
							}
							case SN_ERROR :
							case ATUTHCODE_ERROR :
							case DEV_NOT_EXIT :
							{
								gprsDataTypeFlag =	DEV_REG_DATA;
								break;
							}
							case NETWORK_ERROR :
							case SERVER_ERROR :											
							{
								//�������������˳�����ѭ������
								if (gprsDataTypeFlag !=CONFIG_REQUEST_DATA)
								{
									cureTcpLinkChannel++;
								}
								else  
								{
									gprsDataTypeFlag =	SENSOR_DATA;
								}
								break;
							}
							case 998 :			
							case 999 :
							{
								gprsState = GPRS_SEND;//GPRS_SEND_DATA_READY GPRS_SEND_230110
								break;
							}
							default :
							{
								gprsState = GPRS_SINGEL_CLOSE;
								break;
							}
						}
					}
					printf("\n\r data type %d,server Response code %d\n\r",gprsDataTypeFlag,tcpResponse.responseCode.i);
					if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
					{
					//�˳�
						txRxTaskState = GPRS_SLEEP;
						gprsState = 	GSM_MOUDLE_IDLE;			  
					}
					break;
				}
				else
				{		
				//������ֵ������					
					rxResult->contuineSendErrorCounter ++;
					serverLSRLog[cureTcpLinkChannel].rxErrorCounter ++;
					serverLSRLog[cureTcpLinkChannel].rxTry ++;
					gprsState = GPRS_SINGEL_CLOSE;

					if(serverLSRLog[cureTcpLinkChannel].rxTry >= 1)//GPRS_TIMES_TRY
					{
						serverLSRLog[cureTcpLinkChannel].allTry++;
						serverLSRLog[cureTcpLinkChannel].rxTry= 0x00;
						rxResult->contuineSendErrorCounter ++;
					}	
				#if GPRS_DEBUG > 0
					printf("\n\r receive server data error %d\n\r",tcpResponse.responseCode.i);
					printf("\n\r server insentek data error\n\r");
					printf("\n\r rx server data error counter %d\n\r",serverLSRLog[cureTcpLinkChannel].rxErrorCounter);
				#endif					
				}
				break;
			}
			case GPRS_REG_DATA_RE :
			{
				//rxResult->contuineSendErrorCounter = 0x00;
				if(DEV_REG_DATA == gprsDataTypeFlag)
				{		
					findAuthCode();
				#if(GPRS_CONFIG_CTR>0)
					gprsDataTypeFlag =	CONFIG_REQUEST_DATA;	
				#else
					gprsDataTypeFlag =	SENSOR_DATA;	
				#endif
				}
				// gprsState = GPRS_SINGEL_CLOSE;	
				gprsState = GPRS_SEND_DATA_READY;
				break;
			}
		#if(GPRS_CONFIG_CTR>0)
			case GPRS_CONFIG_DATA_RE :
			{				
				gprsConfigTask(rxResult);		
				if( rxResult->rtcSetFlag==FUNCITONG_TURN_ON)
				{			
					gprsDataTypeFlag =	DEV_RTC_CAL_DATA;
				}
				else
				{
					gprsDataTypeFlag =	SENSOR_DATA;
				}
				gprsState = GPRS_SEND_DATA_READY;				
				break;
			}					   
		#endif
			case GPRS_RTC_CAL_RE :
			{
				//��ȡ����
				if(!(findRtcTime()))
				{
					rxResult->rtcSetFlag = CHANGE_FLAG;
				}
				//�ı����ݰ�����
				gprsDataTypeFlag =	SENSOR_DATA;
				gprsState = GPRS_SEND_DATA_READY;
				break;
			}
			case GPRS_SENSOR_DATA_RE :
			{
				gprsSendSucFlagWrite(curGprsSendAddr,sensorDataReadAddr,sendTime);
				gprsSendSucDataDeal((uint32_t *)&curGprsSendAddr,sensorDataReadAddr,(uint16_t *)&unSendDataPacket);
				gprsSendAddrSave();
				printf("\n\r send  data  %d   ,%d \n\r",curSendPacket,unSendDataPacket);		  
				rxResult->simCardReadOkFlag  = FUNCITONG_TURN_OFF;        
				curSendPacket = 0x00;
				gprsState = GPRS_SEND_DATA_READY;					
				if(unSendDataPacket==0x00)
				{				
					/*�豸����*/					                              
					rxResult->sendErrorCounter = SEND_OK; 
					if(rxResult->requestDataType == REQUEST_DATA_BY_COUNTER)
					{
						gprsState =   FIND_HISTORY_DATA;		   
						rxResult->requestDataReadAddress = curGprsSendAddr;
					}
					if (rxResult->historyDataFlag == FUNCITONG_TURN_ON)
					{
						gprsDataTypeFlag =	DEV_HISTORY_DATA;						
					}
					if(rxResult->requestDataType == REQUEST_DATA_IDLE)
					{
						rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
						cureTcpLinkChannel ++;					   
					}		                                       
				}
		               
				if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
				{
				//�˳�
					txRxTaskState = GPRS_SLEEP;
				}
				break;
			}
					   
			case GPRS_HISTORY_DATA :
			{     
				gprsSendSucDataDeal(&rxResult->requestDataReadAddress,sensorDataReadAddr,&rxResult->requestData);;//221101  				
//				rxResult->requestDataReadAddress += SENSOR_DATA_STORE_SIZE*curSendPacket;
//				if(rxResult->requestData > curSendPacket)
//				{
//					rxResult->requestData -= curSendPacket;
//				}else
//				{
//					rxResult->requestData = 0x00;
//				}
				printf("\n\r send history  data  %d   ,%d \n\r",curSendPacket,rxResult->requestData);		  
				curSendPacket = 0x00;

				rxResult->sendErrorCounter = SEND_OK; 	
				gprsState = GPRS_SEND_DATA_READY;			
				if(0x00 >=rxResult->requestData)
				{
					rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
					rxResult->requestDataType = REQUEST_DATA_IDLE;	
					//�˳�
					txRxTaskState = GPRS_SLEEP;
				}
				break;
			}
			
			case GPRS_SINGEL_CLOSE :
			{
				gprsState = GPRS_CLOSE;
			#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)
				for(tryTimes = 0;tryTimes<2;tryTimes++)
			#endif
				{
					if(closeSingleTcp(LINK_NUM_INSENTEK)==COMM_OK)
					{
						gprsState = GPRS_LINK;
						printf("\r\n socket closed.\r\n");
						break;
					}
				}

				if((unSendDataPacket== 0x00)&&(rxResult->requestData ==0x00))
				{
				//�˳�
					txRxTaskState = GPRS_SLEEP;
					gprsState 		=	GSM_MOUDLE_IDLE;		  
				}
		
				break;
			}			
			case GPRS_CLOSE :
			{
			#if(MQTT_USE_CTR==0)
				(void)closeTcp();
			#endif
				gprsState = GPRS_SET;
				break;
			}	
			case FIND_HISTORY_DATA :
			{
				printf("\n\rEnter find flash history  task   \n\r");			
				if(!historyDataFind(rxResult))
				{
					printf("\n\r find flash history  data  %d \n\r",rxResult->requestData);	
					rxResult->historyDataFlag = FUNCITONG_TURN_ON;
					gprsState = GPRS_LINK;
					gprsDataTypeFlag =	DEV_HISTORY_DATA;			   
				}else
				{
				//�˳�
					txRxTaskState = GPRS_SLEEP;
					gprsState = 	GSM_MOUDLE_IDLE;
					rxResult->requestDataType = REQUEST_DATA_IDLE;		 
				}
				printf("\n\rfind flash history  task  End \n\r");
				break;
			}

			default :
			{
				txRxTaskState = GPRS_SLEEP;
				gprsState = 	GSM_MOUDLE_IDLE;
				break;
			}
		}
		//������෢��300������
		if(signSendCounter > 300)
		{
		//�˳�
			txRxTaskState = GPRS_SLEEP;
			gprsState = 	GSM_MOUDLE_IDLE;		
		}
		if(cureTcpLinkChannel >=MAX_MULTI_CONNECTION_SIZE)
		{
			txRxTaskState = GPRS_SLEEP;
			gprsState 		= GSM_MOUDLE_IDLE;			  
		}
	}
	gprsState = GSM_MOUDLE_IDLE;
	if(rxResult->sendErrorCounter>  0x04)
	{
		rxResult->sendErrorCounter = SEND_OK;
	}		 
}

#define MAX_UINT16_T           65500

void gprsTxRxResultPrintf(void)
{
     uint8_t tcpLinkChannel = 0x00;
	 for(tcpLinkChannel = 0x00 ;tcpLinkChannel < MAX_MULTI_CONNECTION_SIZE;tcpLinkChannel ++)
	 	{
	 	if(tcpServerAddrBuf[tcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
	 		{
	 	printf("\n\r server ID %d\n\r",tcpLinkChannel);
              printf("\n\r server receive %d packet data\n\r",serverLSRLog[tcpLinkChannel].dataOkCounter);
	       if(serverLSRLog[tcpLinkChannel].dataOkCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].dataOkCounter = 0x00;
		   }
	       printf("\n\r network gprs registration error :%d\n\r",serverLSRLog[tcpLinkChannel].gprsRegErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].gprsRegErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].gprsRegErrorCounter = 0x00;
		   }
		printf("\n\r network gprs deattach :%d\n\r",serverLSRLog[tcpLinkChannel].gprsAttachErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].gprsAttachErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].gprsAttachErrorCounter = 0x00;
		   }		
	       printf("\n\r gprs apn set error :%d\n\r",serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter = 0x00;
		   }			   
	       printf("\n\r bring up wireless with gprs or csd error :%d\n\r",serverLSRLog[tcpLinkChannel].gprsBringUpErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].gprsApnSetErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].gprsBringUpErrorCounter = 0x00;
		   }	
	       printf("\n\r get local ip error :%d\n\r",serverLSRLog[tcpLinkChannel].getIpErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].getIpErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].getIpErrorCounter = 0x00;
		   }			   
 	       printf("\n\r link server error counter %d\n\r",serverLSRLog[tcpLinkChannel].linkErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].linkErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].linkErrorCounter = 0x00;
		   }				   
 	       printf("\n\r send data error counter %d \n\r",serverLSRLog[tcpLinkChannel].sendErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].sendErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].sendErrorCounter = 0x00;
		   }			   
	      printf("\n\r rx server data error counter %d\n\r",serverLSRLog[tcpLinkChannel].rxErrorCounter);
	       if(serverLSRLog[tcpLinkChannel].rxErrorCounter >MAX_UINT16_T )
	       	{
                      serverLSRLog[tcpLinkChannel].rxErrorCounter = 0x00;
		   }			  
		}
		}

}



char hex2asc[]="0123456789ABCDEF";
uint16_t hexToAscii(uint8_t *bufIn,uint16_t inLength,uint8_t *bufOut)
{
	uint16_t hexi = 0;
	uint16_t hexj = 0;

	for(hexi = 0;hexi < inLength;hexi++)
	{
		bufOut[hexj++]= hex2asc[((bufIn[hexi]&0xF0)>>4)];//��λ
		bufOut[hexj++]= hex2asc[(bufIn[hexi]&0x0F)];//��λ
	}
	return hexj++;
}

float str2float(uint8_t *buf,uint8_t length)
{
   char cavBuf[20];
   uint8_t cavi = 0;
   float reault = 0;
 
   if(length<20)
   {
   	  for(cavi = 0;cavi < length;cavi++)
	  {
	      cavBuf[cavi] = buf[cavi];
	  }
	  cavBuf[cavi]  = '\0';
	  reault =atof(cavBuf);
   }
   else
   {
      reault = 0;
   }
   return reault;
}

uint32_t str2int(uint8_t *buf,uint8_t length)
{
   char cavBuf[12];
   uint8_t cavi = 0;
   uint32_t reault = 0;
 
   if(length<10)
   {
   	  for(cavi = 0;cavi < length;cavi++)
	  {
	      cavBuf[cavi] = buf[cavi];
	  }
	  cavBuf[cavi]  = '\0';
	  reault =atol(cavBuf);
   }
   else
   {
      reault = 0;
   }
   return reault;
}
void httpConfigParameInit(httpConfig *rxResult)
{
	rxResult->gsmStartResult = -1;
	rxResult->gpsValueFlag = GPS_VALUE_VALID;
	rxResult->rtcSetFlag = UNCHANGE_FLAG;
	rxResult->firewareFlag= FIREWARE_IDLE;	   
	rxResult->dataSentFlag = 0x00;
	rxResult->contuineSendErrorCounter = 0x00;
	rxResult->devApnSetFlag	=	0;////	rxResult->smsSensorDataFlag = EMPTEY_DATA_TYPE;

	rxResult->sendErrorCounter = SEND_OK;
	rxResult->rtcSetFlag = FUNCITONG_TURN_ON;
	rxResult->historyDataFlag = FUNCITONG_TURN_OFF;
	rxResult->requestDataType = REQUEST_DATA_IDLE;
	rxResult->requestDataReadAddress = SENSOR_DATA_BASE_ADDR ;
	rxResult->inItMode  = FUNCITONG_TURN_OFF;
	rxResult->inItModeCounter = 0x00;
//	rxResult->gsmIntFlag =0x00;
	rxResult->callMode = 0x00;
	rxResult->storeDepFlag = 0x00;
	rxResult->testMode = FUNCITONG_TURN_OFF;
	rxResult->gsmRunMode 				= FUNCITONG_TURN_OFF;
	rxResult->simCardReadFlag 	= FUNCITONG_TURN_ON;
	rxResult->simCardReadOkFlag  = FUNCITONG_TURN_OFF;
	
	rxResult->getIpFlag 				= 0;
	rxResult->linkErrorCouner 	= 0;
}


void stationFlashReset(void)
{
	sensorDataWriteAddr = SENSOR_DATA_BASE_ADDR;
	curGprsSendAddr = SENSOR_DATA_BASE_ADDR; 
	curStationSend[0].curSendAddress = curGprsSendAddr;
	curStationSend[1].curSendAddress = curGprsSendAddr;
	curStationSend[0].historyDataFlag = FUNCITONG_TURN_OFF;
	curStationSend[1].historyDataFlag = FUNCITONG_TURN_OFF;
	curStationSend[0].requestData = 0x00;
	curStationSend[1].requestData = 0x00;	
	curStationSend[0].unSendPacket = 0 ;
	curStationSend[1].unSendPacket = 0;
	unSendDataPacket = 0;
	calCounter = 0x00;

}
void flashStoreTask(httpConfig rxResult)
{
	uint8_t storeFlag = 0;
	
	if((rxResult.staionFlashResetFlag >0x00)||(rxResult.staionResetFlag >0x00)||(rxResult.storeFlagConfig >0x00)){storeFlag = 1;}
	else if((rxResult.storeFlagAbc >0x00)||(rxResult.storeFlagFaw >0x00)||(rxResult.storeDepFlag >0x00)){storeFlag = 1;}
	else if((rxResult.storeFlagParam >0x00)||(rxResult.storeFlagStationServer >0x00)){storeFlag = 1;}
#if(MQTT_USE_CTR>0)
	else if(rxResult.storeFlagSensorRange>0x00)storeFlag = 1;
#endif
	if(storeFlag == 1)
	{
	//�������ݴ洢
		power331On();//�򿪴洢FLASH��Դ
		SPI_Flash_WAKEUP();
		if(rxResult.staionFlashResetFlag >0x00)
		{
			stationFlashReset();
		}
		if(rxResult.staionResetFlag >0x00)
		{
			stationReset();
		}     
		if(rxResult.storeFlagConfig >0x00)
		{
			devConfigValueFlashStore(configValue);
		}
		if(rxResult.storeFlagDataSmsPhone >0x00)
		{
		//	phoneStoreFlash(smsNum,SEC_DEV_SMS_PHONE_SET_COMM);
		} 
		if(rxResult.storeFlagAlarmSmsPhone > 0x00)
		{
	//		phoneStoreFlash(ctrNum,SEC_DEV_CTR_PHONE_SET_COMM);
		}							
		if(rxResult.storeFlagAbc >0x00)
		{
			devAbcStoreFlash(curAbc);
		}
		if(rxResult.storeFlagFaw >0x00)
		{
			devfawStoreFlash(curFaw);
		}
		if(rxResult.storeDepFlag>0x00)
		{
			devDepSetByGprs();
		}
		if(rxResult.storeFlagParam >0x00)
		{
			devParamterStore(devPar);
		}
		if(rxResult.storeFlagStationServer >0x00)
		{
		#if(MQTT_USE_CTR>0)
			mqttAddressSet();
		#if(SECOND_MQTT_ADDR>0)
			if(mqttServerNumSecond>0)
			{
				cigemServerAddrdeal();			
				for(storeFlag = 0;storeFlag<SECOND_MULTI_CONNECTION_SIZE;storeFlag++)
				{
					if((storeFlag+mqttServerNumFirst)<MQTT_MAX_MULTI_CONNECTION_SIZE)
					{
						curMqttSendAddr[mqttServerNumFirst+storeFlag].i = secondSendAddr[storeFlag];
					}
				}
//				mqttunsendpacketnumadj();
			}
		#endif
		#endif
		//	gprsAddressSet();
		}
	#if(MQTT_USE_CTR>0)
		if(rxResult.storeFlagSensorRange>0x00)
		{
			sensorRangeStore(GPRS_SET_FLAG);
		}
	#endif
	#if(DEV_APN_SET_CTR>0) //230508
		if(rxResult.devApnSetFlag>0x00)
		{
			gprsApnSave(devApnSetStatus);
		}
	#endif
		SPI_Flash_PowerDown();
		power331Off();
//		servStoreFlagInit(&serverConfig);
	}
}

uint32_t findFlashWriteAddr(uint32_t baseAddress,uint32_t endAddr,uint16_t packSize,uint8_t *readBuf)
{
	uint32_t readAddress =0;
	uint32_t address =0;
	//	   uint8_t readBuf[11];
	uint16_t i =0;
	uint8_t findAddFiag =0x00;
	//	   uint8_t findi = 0;
	//	   uint8_t findStatus = 0x00;
	//	   uint32_t j =0;
	//	   uint32_t bufReadAddress = 0x00;
	for(readAddress = baseAddress;readAddress <endAddr;readAddress += packSize)
	{
		SPI_FLASH_BufferRead(readBuf,readAddress,packSize);
		if(((readBuf[0]==0xFF)&&(readBuf[1]==0xFF))&&(readBuf[3]==0xFF))
		{
			if(findAddFiag ==0x00)
			{
				address =  readAddress;
			}

			for(i = 0;i< packSize;i++)
			{
				if(readBuf[i]!=0xFF)
				{
					findAddFiag =0;
					break;
				}
			}
			if(i >= packSize)
			{
				findAddFiag = 0x02;
			}
			if(findAddFiag ==0x02)
			{
			#ifdef FLASH_DEBUG
				printf("\n\r FIND 1FLASH[%d]��ַ\n\r",readAddress);
			#endif
			break;
			} 
		}
		else
		{
			findAddFiag =0;
		}	
	}
	if(findAddFiag ==0x00)
	{
		address =  baseAddress;
	}
#ifdef FLASH_DEBUG
	printf("\n\r FIND 2FLASH[%d]��ַ\n\r",address);
#endif
	return address;
}


uint32_t findDataWriteAddr(void)
{
	uint8_t dataBuf[SENSOR_DATA_STORE_SIZE+1];
	uint32_t readAddress =0;
	readAddress = findFlashWriteAddr(SENSOR_DATA_BASE_ADDR,SENSOR_DATA_END_ADDR,SENSOR_DATA_STORE_SIZE,dataBuf);
	return readAddress ;

}

uint32_t findFlashReadAddr(uint32_t baseAddress,uint32_t endAddr,uint32_t writeAddress,uint16_t packSize,uint8_t *readBuf)
{
     uint32_t readAddress =0;
	   uint32_t address =0;
	   uint16_t i =0;
	   uint16_t crcj = 0;
	   uint8_t crcData = 0x00;

	   uint8_t findAddFiag = 0x00;

	   for(readAddress =baseAddress;readAddress < endAddr;readAddress +=packSize)
	   {
	        crcData = 0x00;
	      	SPI_FLASH_BufferRead(readBuf,writeAddress,packSize);
			if(writeAddress ==baseAddress )
			{
			  writeAddress = endAddr;
			}    
		 if((readBuf[0]==0x2C)&&(readBuf[1]==0x2C))
		 {
		 	 for(i = (packSize-1);i>2;i--)
			 {
	            if(readBuf[i] ==END_CHARCCTER )
		         {
		           if((readBuf[i-1] ==END_CHARCCTER )&&(readBuf[i-2] ==END_CHARCCTER ))
			        {
					 
					  /*У����*/
					  for(crcj = 0x00;crcj < i -3;crcj ++)
					  {
					   crcData += readBuf[crcj];   
					  }
					  if(crcData == readBuf[i-3])
					  {
					   findAddFiag = 0x02 ;
					  address =  writeAddress;
			          break;
					  }
			        }		 
		        }			    
			 }		 
		 }
		 	if(findAddFiag ==0x02)
			 {
			    break;
			 }	
		 	writeAddress -=  packSize;   
	   }
			 return address;
}





uint16_t flashSensorDataRead(uint32_t startAddress,uint32_t endAddress,uint32_t *readAddress,uint8_t *buf,uint16_t packSize,uint16_t readCounter)
{
    uint32_t address = 0x00;
	uint16_t addri = 0x00;
//	uint16_t addrj = 0x00;
	uint16_t dataCounter = 0x00;

	address = startAddress;
	for(;address < endAddress;address +=packSize)
	{
	   	SPI_FLASH_BufferRead(buf,*readAddress,packSize);
		if(*readAddress  == startAddress)
		{
		   *readAddress = endAddress; 
		}

		   *readAddress -= packSize; 
		usart2StoreBuffer(buf,packSize);
		if((buf[0]==0x2c)&&(buf[1]==0x2c))
		{
		      for(addri = (packSize -1);addri > 2;addri--)
			  {
			      if(buf[addri]==END_CHARCCTER)
				  {
				     if((buf[addri-1]==END_CHARCCTER)&&(buf[addri-2]==END_CHARCCTER))
					 {					 
					        usart2StoreBuffer(&buf[3],(addri-2));
							dataCounter ++;
					 }
				  
				  }
			  
			  }
		
		}
	  if(dataCounter >= readCounter)
	  {
	    break;
	  }
	}
	return dataCounter;
}

uint16_t buf2Struct(void *data,uint8_t dataType,uint8_t *buf,uint16_t bufLength)
{
	uint16_t dataLength = 0;
	uint16_t datai = 0;
	uint16_t dataj =0;
	uint16_t status =0x00;
	uint8_t *ptr = &dataType;

	switch(dataType)
	{
		case DEV_WATER_AIR_DATA :
		{
			datai =	SOIL_FAW_LENGTH;
			ptr = (uint8_t *)((soilfaw *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_DEP_DATA :
		{
			datai =	DEP_LENGTH;
			ptr = (uint8_t *)((devDep *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_ABC_DATA :
		{
			datai =	SOIL_ABC_LENGTH;
			ptr = (uint8_t *)((soilabc *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_FHONE_NUM_DATA :
		{
			datai =	PHONE_NUM_LENGTH;
			ptr = (uint8_t *)((phone *)data);
			dataLength = datai*MAX_USER_DATA_NUM;
			break;
		}
		case DEV_FHONE_CTR_DATA :
		{
			datai =	PHONE_NUM_LENGTH;
			ptr = (uint8_t *)((phone *)data);
			dataLength = datai*2;
			break;
		}
		case DEV_SEND_PERIOD_DATA :
		{
			datai =   SEND_PERIOD_LENGTH;
			ptr = (uint8_t *)((sendPer *)data);
			dataLength = datai;
			break;
		}
		case DEV_VALUE_DATA  :
		{
			datai =   VALUE_TYPE_LENGTH;
			ptr = (uint8_t *)((value_t *)data);
			dataLength = datai;	   
			break;
		}
		case DEV_CONFIG_DATA :
		{
			datai =   CONFIG_DATA_LENGTH;
			ptr = (uint8_t *)((devConfig*)data);
			dataLength = datai;
			break;                
		}	 
		case DEV_GPS_DATA    :
		{
			datai =  GPS_VALUE_LENGTH;
			ptr = (uint8_t *)((gpsValue*)data);
			dataLength = datai;					
			break;
		}	 
		case DEV_FIREWARE_INF_DATA   :
		{
			datai =  FIREWARESTRUCT_LENGTH ;
			ptr = (uint8_t *)((fireWareStruct*)data);
			dataLength = datai;					
			break;	   
		}
		case DEV_PARAMTER_DATA   :
		{
			datai =  PARAMTER_DATA_LENGTH;
			ptr = (uint8_t *)((paramter*)data);
			dataLength = datai;					
			break;
		}		
		case DEV_GPRS_ADDRESS_DATA :
		{
			datai =  GPRS_SEVER_ADDRESS_LENGTH;
			ptr = (uint8_t *)((serverAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<STATION_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}					
			break;
		}
	#if(MQTT_USE_CTR>0)
		case DEV_MQTT_ADDRESS_DATA :
		{
			datai =  MQTT_SEVER_ADDRESS_LENGTH;
			ptr = (uint8_t *)((mqttServerAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<MQTT_MAX_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}
			break;
		}
		case DEV_SENSOR_RANGE_DATA:
		{
			datai =  SENSOR_RANGE_VALUE_LENGTH;
			ptr = (uint8_t *)((cigemsensorrange_t*)data);
			dataLength = 0;
			for(dataj=0;dataj<SENSORID_INDEX_NUM;dataj++)
			{
				dataLength += datai;
			}			
			break;
		}
		case DEV_MQTT_SENDADDR_DATA:
		{
			datai =  VALUE_TYPE_LENGTH;
			ptr = (uint8_t *)((value_t*)data);
			dataLength = 0;
			for(dataj=0;dataj<MQTT_MAX_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}					
			break;
		}
	#if(SECOND_MQTT_ADDR>0)
		case DEV_MQTT_ADDRESS2_DATA :
		{
			datai =  MQTT_SECOND_ADDRESS_LENGTH;			
			ptr = (uint8_t *)((secondServerAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<SECOND_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}			
			break;
		}	
	 #endif
	#endif
		default :
		break;
	}
	if(datai ==0x00)
	{
	// return 0;
		status  = 0x01;
	}
	else
	{
		for(dataj = 0x00;dataj < dataLength;dataj++)
		{
			ptr[dataj] = buf[dataj];//*ptr++ = buf[dataj];//201115
		}  
	}
	return status;
}
   
uint16_t struct2Buf(void *data,uint8_t dataType,uint8_t *buf)
{
	uint16_t dataLength = 0;
	uint16_t bufLength  = 0;
	uint16_t datai = 0;
	uint16_t dataj = 0;
	uint8_t *ptr	= &dataType;

	switch(dataType)
	{
		case DEV_WATER_AIR_DATA :
		{
			datai =	SOIL_FAW_LENGTH;
			ptr = (uint8_t *)((soilfaw *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_DEP_DATA :
		{
			datai =	DEP_LENGTH;
			ptr = (uint8_t *)((devDep *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_ABC_DATA :
		{
			datai =	SOIL_ABC_LENGTH;
			ptr = (uint8_t *)((soilabc *)data);
			dataLength = datai*DEV_MAX_NODE_NUM;
			break;
		}
		case DEV_FHONE_NUM_DATA :
		{
			datai =	PHONE_NUM_LENGTH;
			ptr = (uint8_t *)((phone *)data);
			dataLength = datai*MAX_USER_DATA_NUM;
			break;
		}
		case DEV_FHONE_CTR_DATA :
		{
			datai =	PHONE_NUM_LENGTH;
			ptr = (uint8_t *)((phone *)data);
			dataLength = datai*2;
			break;
		}
		case DEV_SEND_PERIOD_DATA :
		{
			datai =   SEND_PERIOD_LENGTH;
			ptr = (uint8_t *)((sendPer *)data);
			dataLength = datai;
			break;
		}
		case DEV_VALUE_DATA  :
		{
			datai =   VALUE_TYPE_LENGTH;
			ptr = (uint8_t *)((value_t *)data);
			dataLength = datai;	   
			break;
		}
		case DEV_CONFIG_DATA :
		{
			datai =   CONFIG_DATA_LENGTH;
			ptr = (uint8_t *)((devConfig*)data);
			dataLength = datai;
			break;                
		}
		case DEV_GPS_DATA    :
		{
			datai =  GPS_VALUE_LENGTH;
			ptr = (uint8_t *)((gpsValue*)data);
			dataLength = datai;					
			break;
		}
		case DEV_FIREWARE_INF_DATA   :
		{
			datai =  FIREWARESTRUCT_LENGTH ;
			ptr = (uint8_t *)((fireWareStruct*)data);
			dataLength = datai;					
			break;	   
		}
		case DEV_PARAMTER_DATA   :
		{
			datai =  PARAMTER_DATA_LENGTH;
			ptr = (uint8_t *)((paramter*)data);
			dataLength = datai;					
			break;
		}
		case DEV_GPRS_ADDRESS_DATA :
		{
			datai =  GPRS_SEVER_ADDRESS_LENGTH;
			ptr = (uint8_t *)((serverAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<STATION_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}						
			break;
		}
	#if(MQTT_USE_CTR>0)
		case DEV_MQTT_ADDRESS_DATA :
		{
			datai =  MQTT_SEVER_ADDRESS_LENGTH;
			ptr = (uint8_t *)((mqttServerAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<MQTT_MAX_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}				
			break;
		}
		case DEV_SENSOR_RANGE_DATA:
		{
			datai =  SENSOR_RANGE_VALUE_LENGTH;
			ptr = (uint8_t *)((cigemsensorrange_t*)data);
			dataLength = 0;
			for(dataj=0;dataj<SENSORID_INDEX_NUM;dataj++)
			{
				dataLength += datai;
			}					
			break;
		}
		case DEV_MQTT_SENDADDR_DATA:
		{
			datai =  VALUE_TYPE_LENGTH;
			ptr = (uint8_t *)((value_t*)data);
			dataLength = 0;
			for(dataj=0;dataj<MQTT_MAX_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}					
			break;
		}
	 #if(SECOND_MQTT_ADDR>0)
		case DEV_MQTT_ADDRESS2_DATA :
		{
			datai =  MQTT_SECOND_ADDRESS_LENGTH;
			ptr = (uint8_t *)((secondServerAddr*)data);
			dataLength = 0;		
			for(dataj=0;dataj<SECOND_MULTI_CONNECTION_SIZE;dataj++)
			{
				dataLength += datai;
			}
			break;
		}	
	 #endif
	#endif		
		default :
		break;
	}

	if(datai ==0x00)
	{
		// return 0;
		bufLength  = 0;
	}
	else
	{
		//     dataLength = datai*10;
		for(dataj = 0x00;dataj < dataLength;dataj++)
		{
			buf[dataj] = ptr[dataj];//buf[dataj] = *ptr++;//201115
		}
		bufLength  = dataj;
	}

	return bufLength;
}



#define SEND_ERR_SLEEP_TIEM          60    
#define BEGIN_HOUR_TIME             ((24+ONE_DAY_START_HOUR)*60*60)
#define ONE_HOUR_TIME               (60*60)
#define FIX_SEND_ADJ_TIME           (0)
#define ONE_DAY_START_HOUR          0
#define START_HOUR_OFFSET_TIME      (ONE_DAY_START_HOUR*60*60)

uint32_t nextSendTime(uint8_t periodHour,devConfig sendTimeConfig)
{
	uint32_t curTime = 0;
	uint32_t nextTime = 0;
	uint32_t readySleepTime = 0;

	uint16_t nexttSendHour = 0;
	uint16_t nextSendMin = 0x00;
	//	   uint16_t startHour = 0x00;
	DS1302_TIME  rtcCurTime;
	readDs1302Time(&rtcCurTime);
    
	if(sendTimeConfig.devPeriodMin > 59)
	{
		sendTimeConfig.devPeriodMin = 10;
	}
	if(( periodHour> 23)||(periodHour == 0x00))//sendTimeConfig.devPeriodHour
	{
		 periodHour = 2;//sendTimeConfig.devPeriodHour
	}
//��ǰ�����ж� 
	if(sendTimeConfig.devPeriodMin  > rtcCurTime.min)
	{             
		nextSendMin =  sendTimeConfig.devPeriodMin - rtcCurTime.min;
	} 
//��ǰСʱ�ж�    
	nexttSendHour = (rtcCurTime.hour%periodHour);//sendTimeConfig.devPeriodHour 
	if((nexttSendHour == 0x00)&&(nextSendMin >0))//4
	{
		nexttSendHour = rtcCurTime.hour/periodHour;//sendTimeConfig.devPeriodHour;
	}
	else
	{
		nexttSendHour = rtcCurTime.hour/periodHour;//sendTimeConfig.devPeriodHour; 
		nexttSendHour = (nexttSendHour+1)*periodHour;//sendTimeConfig.devPeriodHour;
	}             
	curTime = rtcCurTime.hour*ONE_HOUR_TIME+rtcCurTime.min*60+rtcCurTime.sec;
	nextTime = nexttSendHour*ONE_HOUR_TIME +sendTimeConfig.devPeriodMin*60;

	if(nextTime >= BEGIN_HOUR_TIME)
	{
		nextTime = BEGIN_HOUR_TIME +sendTimeConfig.devPeriodMin*60+(periodHour)*ONE_HOUR_TIME;//sendTimeConfig.devPeriodHour 
		if(periodHour==0x01)//sendTimeConfig.devPeriodHour
		{
			nextTime = BEGIN_HOUR_TIME +sendTimeConfig.devPeriodMin*60;
		}
		if(sendTimeConfig.devPeriodMin == 0x00)
		{
			nextTime = BEGIN_HOUR_TIME;
		}
	}
	if(nextTime>=curTime)
	{
		readySleepTime = nextTime - curTime;
//		if(readySleepTime <120)//300
//		{
//			readySleepTime = readySleepTime +sendTimeConfig.devPeriodMin*60+(periodHour)*ONE_HOUR_TIME;  //sendTimeConfig.devPeriodHour               
//		} 
	}
	else
	{
		readySleepTime = sendTimeConfig.devPeriodMin*60+(periodHour)*ONE_HOUR_TIME;    //sendTimeConfig.devPeriodHour             
	}
	return readySleepTime;
}

int8_t sensorDataOkFlag(void)
{
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
//	uint8_t rxi = 0x00;
//	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      ST 6006676541 OK aaNN
	*/
	findStatus = findStrRxGprs(" OK aaNN",8);
	if(findStatus < FIND_ERROR)
		{

		  rxResult = 0;		
	    }
       return rxResult;      

}




static uint8_t rxGprs(uint8_t *code)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t errorStatus = UN_FIND_STATUS;
	//     char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t rxi = 0x00;	 

	status = findStrRxGprs("+IPD,",5);
	if(status < FIND_ERROR)
	{
		status = COMM_OK;
		*code =  RX_OK_CODE_ID;
	}
	else
	{
		status = UN_FIND_STATUS;
		gprsFlag =  COMM_SEND;
		//delay_GprsMs(20);
		commTxCount  = 0x00;
		gprsFlag =  COMM_ACK;

		delay_GprsMs(50);
		while(gprsFlag  == COMM_ACK)
		{
			delay_GprsMs(2);
		}
		if(gprsFlag  == COMM_TIMEOUT)
		{
			status = COMM_TIMEOUT;
			*code = RX_ERROR_CODE_ID;	  
		}
		errorStatus = findStrRxGprs("+IPD,",5);
		if(errorStatus < FIND_ERROR)
		{
			*code = RX_OK_CODE_ID;
			status = COMM_OK;
		}
		else
		{
			status =  COMM_ERR;
		}	
	}
	if(*code == RX_OK_CODE_ID)
	{
		errorStatus = findStrRxGprs("ST ",3);
		if(errorStatus < FIND_ERROR)
		{
			*code = RX_FRAME_OK;
			//�Ƚ�ң��վ��ַ
			for(rxi = 0x01;rxi < devPar.stationAddr[0];rxi ++)
			{
				if(devPar.stationAddr[rxi]!=GPRS_RX_Buffer[errorStatus++])
				{
					*code = RX_FRAME_ERROR;  
					break;
				}
			}
		}
	}
	return status	;		   	 
}


#define CONFIG_FRAME_FLAG    0x01
#define INSTRUCT_FRAME_FLAG  0x02
#define LIAISON_FRAME_FLAG   0x03
#define OK_FRAME_FLAG        0x05

#define ERROR_FRAME_FLAG     0x04

static int8_t dataOkJudeg(void)
{
     int8_t result = -1;
     uint16_t status = UN_FIND_STATUS;

     //configure frame find
     status = findStrRxGprs(" OK aaNN",8);
	 if(status < FIND_ERROR)
	 {
       result = 0x00;
     } 
       return result;

}
static int8_t dataFrameJudeg(void)
{
     int8_t result = -1;
     uint16_t status = UN_FIND_STATUS;
     uint16_t endStatus = 0x00;
     uint8_t  length = 0x00;

     //configure frame find
   

     status = findStrRxGprs(" ? ",3);
     if(status <FIND_ERROR)
     {
     status = findStrRxGprs(" aaNN",5);
     if(status < FIND_ERROR)
     {
       result = INSTRUCT_FRAME_FLAG;
       return result;
     }
     }
     status = findStrRxGprs(" PW ",4);
	 if(status < FIND_ERROR)
	 {
       result = CONFIG_FRAME_FLAG;
       return result;
     }     
     status = findStrRxGprs("NN",2);
	 if(status < FIND_ERROR)
	 {
     endStatus = findStrRxGprs("ST ",3);
     length = status -endStatus;
//     printf("�豸�������ڣ�[%d]��\r\n",length); 
     if(length==15) 
     {
       result = LIAISON_FRAME_FLAG;
       return result;
        }
     }
     status = findStrRxGprs(" OK ",4);
	 if(status < FIND_ERROR)
	 {
       result = OK_FRAME_FLAG;
       return result;
     } 
       result = ERROR_FRAME_FLAG;
       return result;
}

#define CONFIG_FRAME_OK         0x00
#define CONFIG_FRAME_KEY_ERROR  0x01

static int8_t configFrameOkJudeg(stationSend *hyDataLs)
{
   int8_t result = -1;
   uint8_t rxi = 0x00;
   uint16_t status = UN_FIND_STATUS;
   
     status = findStrRxGprs("PW",2);
	 if(status < FIND_ERROR)
	 {
          //�Ƚ�ң��վ��ַ
          printf("\n\r pw add %d \n\r",status);  
          result = 0x00;
          status ++;
          for(rxi = 0x01;rxi < devPar.stationKey[0];rxi ++)
          {
             if(devPar.stationKey[rxi]!=GPRS_RX_Buffer[status++])
             {
                result = 0x01;
                printf("\n\r pw add %d \n\r",status);
              	hyDataLs->confChangErrorFlag |= KEY_ERROR;
                break;
             }
          }

     }            
       return result;

}

static int8_t getFrameSerialNum(stationSend *hyDataLs)
{
      char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	findStatus = findStrRxGprs("CN ",3);
	if(findStatus < FIND_ERROR)
		{

		  rxResult = -2;
                for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
					 {
                                     valueBuf[rxi] = '\0';
									 
						 break;
					 }
					 else
					 {
					  valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
					 } 

				}
				//??
				if((rxi >0)&&(rxi < 5))
				{
					rxResult = 1;
					temp  = atoi(valueBuf);
                    if(temp < 10000)
                    {
                        rxResult = 0x00;
                       hyDataLs->downSerialNum = temp;
                     }               
				}
				else
					{
                       rxResult = -3;
				}				
	}
       return rxResult;
 

}


// static int8_t findFuzzyQuery(stationSend *hyDataLs)
// {
//      int8_t result = -1;
//      uint16_t status = UN_FIND_STATUS;

//      //configure frame find
//      status = findStrRxGprs(" OK aaNN",8);
// 	 if(status < FIND_ERROR)
// 	 {
//        result = 0x00;
//      } 
//        return result;

// }

static int8_t findStationWaterabcValue(soilabc *curAbcValue,stationSend *hyDataLs)
{
    char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint8_t abcBuf[200];
	uint16_t findStatus = UN_FIND_STATUS;
    float tempx = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxt = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxg  = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF01 ",5);
	if(findStatus < FIND_ERROR)
	{
	  rxResult = -2;   
       
      for(rxi = 0x00;rxi <200;rxi ++ )
      {
		if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		{
            abcBuf[rxi] = 0x2c;
            rxResult = 0;                         
			break;
		}
		else
		{
		    abcBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
		} 
	  }
		//??
      if(rxResult == 0)
      {
         hyDataLs->confChangeFlag |= CONFIG_ABC_0K;  
	     for(rxj = 0x00;rxj < rxi ;)
	     {
             for(rxt = 0x00;rxt < 3;rxt ++)
             {
                 for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
                 {
                       if(abcBuf[rxj]==0x2c)
                       {
                         valueBuf[rxh] = '\0';
						 rxj++;
							//??
						tempx = atof(valueBuf);		
							 break;
						}else
						{
                            valueBuf[rxh] = abcBuf[rxj++];
                            if(valueBuf[rxh]!= 0x2E)
                            {
                            if((valueBuf[rxh] > 0x3A)||(valueBuf[rxh]< 0x2F))
                            {
                            rxResult = -3;
                            break;
                            }
                            }
				        }		
					}
				   if((rxh < MAX_CONFIG_BUF_SIZE)&&(rxResult == 0))
				   {
					switch(rxt)
					{
                            case 0 :
							{
                               curAbcValue[rxg].a.f = tempx;
							   break;
							}
                            case 1 :
							{
                               curAbcValue[rxg].b1.f= tempx;
				               curAbcValue[rxg].b2.f = 1/tempx;
							   break;
							}
                            case 2 :
							{
                                curAbcValue[rxg].c.f= tempx;
							    rxg ++;
							   break;
							}									   
					}
				   }
				  else
				  {
                      rxResult = -2;
			       }
				}
	    if(rxResult != 0x00)
		{
              hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
              hyDataLs->confChangeFlag = 0;
                                      break;
	   }
                          
	  }
     }
	}
       return rxResult;
}


static int8_t findStationConfigSmsKey(devConfig *key,stationSend *hyDataLs)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      ??????
	*/
    if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF02 ",5);
	if(findStatus < FIND_ERROR)
	{		 
	   rxResult = -2;
	   if((GPRS_RX_Buffer[findStatus+1]==0x20))//???
	   {
		  if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  {
		  	rxResult = 0;
            key->devWorkState =  GPRS_RX_Buffer[findStatus];
            hyDataLs->confChangeFlag|= SMS_KEY_SET_OK;

			if(FUNCITONG_TURN_ON == key->devWorkState)
			{
               key->xyzSharkkey = FUNCITONG_TURN_OFF;
			}					  
		  }

	  }
		if(rxResult != 0x00)
		{
              hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
		}    
	}      	   
      return rxResult;
}

static int8_t findStationConfigSharkKey(devConfig *key,stationSend *hyDataLs)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      ??????
	*/
    if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF07 ",5);
	if(findStatus < FIND_ERROR)
		{		 
		  rxResult = -2;
		    if((GPRS_RX_Buffer[findStatus+1]==0x20))//???
		  	{
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  		{
		  			  rxResult = 0;
                      key->xyzSharkkey=  GPRS_RX_Buffer[findStatus];
                      hyDataLs->confChangeFlag|= SHARK_KEY_OK;

			  					  
		  		}

		  }
       		if(rxResult!= 0x00)
			{
                     hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;

			}          
	}
       	   
         return rxResult;
}


 int8_t findStationSharkSmsPhone(phone *sendPhone,stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF08 ",5);
	if(findStatus < FIND_ERROR)
    {
	   rxResult = -2;
       for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
       {
	     if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		 {
               valueBuf[rxi] = '\0';
               rxResult = 0;			 
			   break;
		}
		else
		{
		   valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
            if((valueBuf[rxi]> 0x3a)||(valueBuf[rxi] < 0x2F) )
            { 
                break;
            }
		} 
		}

		if(rxResult == 0x00)
		{
            for(rxj= 0x00;rxj < rxi;rxj++)
            {
                sendPhone->phoneNumORG[rxj] = valueBuf[rxj];
			}
				sendPhone->phonelength = rxj;
				sendPhone->phoneFlag = PHONE_SET; 
//				phoneFormat(sendPhone); 
                hyDataLs->confChangeFlag|= SMS_PHONE_OK;
		}
        else
        {
           hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
        }            
	}
       return rxResult;      

}


 int8_t findStationConfigSmsPhone(phone *sendPhone,stationSend *hyDataLs)
{
    char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
//	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj =0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF03 ",5);
	if(findStatus < FIND_ERROR)
	{

	  rxResult = -2;
      for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
      {
		if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		{
           valueBuf[rxi] = '\0';
           rxResult = 0;            
		    break;
		}
		else
		{
		   valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
           if((valueBuf[rxi] > 0x3a)||(valueBuf[rxi] < 0x2F) )
           {
               rxResult = -3;
               break;
           }                          
		} 
	  }

	if(rxResult == 0)
	{
         for(rxj= 0x00;rxj < rxi;rxj++)
         {
            sendPhone->phoneNumORG[rxj] = valueBuf[rxj];
		}
		sendPhone->phonelength = rxj;
		sendPhone->phoneFlag = PHONE_SET; 
//		phoneFormat(sendPhone); 
        hyDataLs->confChangeFlag|= SMS_PHONE_OK;
	}
    else
    {
       hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
    }                    
	}
       return rxResult;      

}


static int8_t findStationConfigDevPeriod(devConfig *period,stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
    if(hyDataLs->confChangErrorFlag > 0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF04 ",5);
	if(findStatus < FIND_ERROR)
    {
	   rxResult = -2;
       for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
       {
		  if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		  {
              valueBuf[rxi] = '\0';	
              temp  = atoi(valueBuf);
              if((temp < 601)&&(temp>4))
              {
              rxResult =  0; 
              temp *=60;
                       
              period->devPeriod = temp;
		      rxResult = 0;
		      period->devPeriodType = NORMAL_PERIOD_TYPE;
              hyDataLs->confChangeFlag|= PERIOD_SET_OK;     
              }                  
			  break;
		  }
		  else
		  {
			valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
            if((valueBuf[rxi] > 0x3a)||(valueBuf[rxi] < 0x2F) )
            {
                //hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
                rxResult = -3;
                break;
            }    
		 } 
		}
		if(rxResult!= 0x00)	
        {
          hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
        }			
	}
       return rxResult;
       
}



static int8_t findStationFixTimePeriod(devConfig *fixTimeperiod,stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	char fixBuf[MAX_CONFIG_BUF_SIZE]; 
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint16_t temp1 = 0x00;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag > 0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF06 ",5);
	if(findStatus < FIND_ERROR)
	{

	  rxResult = -2;
      for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
      {
		if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		{
           valueBuf[rxi] = 0x2c;
           rxResult = 0x00;           
		   break;
		}
		else
		{
			valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
		} 

	}
				//??
    if(rxResult ==0x00)
    {
        rxResult = -3;
	   for(rxj = 0x00;rxj <3;rxj ++)
	   {
           
         if(valueBuf[rxj]!= 0x2c)
          {
               fixBuf[rxj] = valueBuf[rxj];
               if((fixBuf[rxj] > 0x3a)||(fixBuf[rxj] < 0x2F) )
               {
 
                    break;
               } 
		  }
		  else
		  {
               fixBuf[rxj]  = '\0';
			   temp= atoi(fixBuf);
               if(temp<9)
               {
                 rxResult = 0x00;
               } 
			   break;
		  }
	   }
    }
    if(rxResult ==0x00)
    {
          
        rxj++;
	    for(rxt= 0x00;rxt<3;rxt++)
		{   
          if(valueBuf[rxj+rxt]!= 0x2c)
          {
             fixBuf[rxt] = valueBuf[rxj+rxt];
             if((fixBuf[rxt] > 0x3a)||(fixBuf[rxt] < 0x2F) )
             {
                  rxResult = -3;    
                  break;
              }             
		       }
		       else
		       {
                 fixBuf[rxt]  = '\0';
			     temp1= atoi(fixBuf);
                 if(temp1< 0x3c)
                 {  if(temp!= 0x00)
                     {
											fixTimeperiod->devPeriodHour = temp;
											fixTimeperiod->devPeriodMin = temp1;
											fixTimeperiod->devPeriodType = FIX_TIME_SEND_TYPE;
//											fixTimeperiod->transmissionDensity = 0x01;
											fixTimeperiod->devPeriod 			= temp*3600;	
		              rxResult = 0x00;  
                      hyDataLs->confChangeFlag|= FIX_PERIOD_SET_OK;
                     }
                     else
                     { 
                         if(temp1>4)
                         {     
                               rxResult = 0x00; 
                               fixTimeperiod->devPeriod = temp1*60;
                               hyDataLs->confChangeFlag|= PERIOD_SET_OK;
                			   fixTimeperiod->devPeriodType = NORMAL_PERIOD_TYPE;          
                          }

                      }                     
                 }                     
			     break;
		      }
		  }
    }
       if(rxResult != 0x00)
       {                                      
         hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR; 
         }		
	}
       return rxResult;
}



static int8_t findStationConfigDevTransmissDensity(uint16_t *density,stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	uint16_t findStatus = UN_FIND_STATUS;
	uint16_t temp = 0x00;
	uint8_t rxi = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag > 0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF05 ",5);
	if(findStatus < FIND_ERROR)
	{

	  rxResult = -2;
      for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
      {
	     if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//???
		 {
               valueBuf[rxi] = '\0';
               temp = atoi(valueBuf);  
               if(temp < 10)
               {
               *density = temp;
			   rxResult = 0;
               hyDataLs->confChangeFlag|= DENSITY_SET_OK;
               }               
			   break;
		}
		else
		{
		   valueBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
           if((valueBuf[rxi] > 0x3a)||(valueBuf[rxi] < 0x2F) )
           {

                 break;
           }    
		} 
	  }

      if(rxResult!= 0x00)
      {
        hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;    
      }
	}
       return rxResult;
}



static int8_t stationRequestDevGpsKey(uint8_t *key,stationSend *hyDataLs)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	if(hyDataLs->confChangErrorFlag>0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF13 ", 5);
	if(findStatus < FIND_ERROR)
    {                
		rxResult = -2;
	    if((GPRS_RX_Buffer[findStatus+1]==0x20))//�ָ���
		{
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON ))
		    {
		  		   rxResult = 0;
				   if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
                   { 
                      *key ++;
				   }	
                    hyDataLs->confChangeFlag|= GPS_LOCATION_SET_OK;                      
		  	}

		} 
        if(rxResult != 0x00)
		{
             hyDataLs->confChangeFlag|= CONFIG_VALUE_ERROR;            
		}            
	}
       	   
         return rxResult;
}

static int8_t findStationThresholdKey(uint8_t *key,stationSend *hyDataLs)
{
        uint16_t findStatus = UN_FIND_STATUS;
//       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   int8_t rxResult = -1;
//	   uint8_t rxi = 0x00;
	/*
      ??????
	*/
	if(hyDataLs->confChangErrorFlag > 0)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs("FF09 ",5);
	if(findStatus < FIND_ERROR)
		{                
		  rxResult = -2;
		  if((GPRS_RX_Buffer[findStatus+1]==0x20))//???
		  {
		  	if((GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_ON )||(GPRS_RX_Buffer[findStatus]== FUNCITONG_TURN_OFF ))
		  	{
		  			  rxResult = 0;
					  if(FUNCITONG_TURN_ON == GPRS_RX_Buffer[findStatus])
                                      { 
                                      *key = FUNCITONG_TURN_ON;
					  	}
					  
					   if(FUNCITONG_TURN_OFF == GPRS_RX_Buffer[findStatus])
                                      { 
                                      *key = FUNCITONG_TURN_OFF;
					  	}
					  
                      hyDataLs->confChangeFlag|= THRESHOLD_KEY_SET_OK;

            }

		  }
		if(rxResult != 0)
		{
                      hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;    
		}          
	}
       	   
         return rxResult;
}



static int8_t findStationThresholdValue(uint8_t thresholdType,devConfig *thresholdValue,stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   uint8_t xyzBuf[30];
	uint16_t findStatus = UN_FIND_STATUS;
       uint16_t tempx = 0.0;
	uint16_t thresholdTemp = 0x00;
//	float tempy  = 0.0;
//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
	uint8_t rxh = 0x00;
	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
      ???????
	*/
	if(hyDataLs->confChangErrorFlag!=0x00)
	{
       return rxResult = -6;
    }
	if(THRESHOLD_VALUE1== thresholdType)
	{
	findStatus = findStrRxGprs("FF10 ",5);
	}
	if(THRESHOLD_VALUE2== thresholdType)
	{
	findStatus = findStrRxGprs("FF11 ",5);
	}
    if(THRESHOLD_VALUE3== thresholdType)
	{
	findStatus = findStrRxGprs("FF12 ",5);
	}
	if(findStatus < FIND_ERROR)
    {
	  rxResult = -2;
      for(rxi = 0x00;rxi <MAX_CONFIG_BUF_SIZE;rxi ++ )
      {
	     if(GPRS_RX_Buffer[findStatus+rxi]==0x20)//???
		 {
             xyzBuf[rxi] = 0x2c;
             rxResult  = 0x00;             
			break;
		}
		else
		{
			xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
		} 
	 }
				//??
	if(rxResult  == 0x00)
    {
	for(rxj = 0x00;rxj < rxi;)
	{
	   for(rxh = 0x00;rxh < MAX_CONFIG_BUF_SIZE;rxh++)
	   {
		if(xyzBuf[rxj]!= 0x2c)
		{
		   valueBuf[rxh] = xyzBuf[rxj++];
		   if((valueBuf[rxh]< 0x2D)||(valueBuf[rxh] > 0x3A))
		   {  
			  rxResult = -3;
			break;
		   }
		}
		else
        {
			valueBuf[rxh] = '\0';
			rxj++;

			switch(rxt)
			{
				case 0 :
				{
					thresholdTemp = atoi(valueBuf);  
                    if(thresholdTemp >100)
                    {
                         rxResult = -5;
                    }

					rxt ++; 
				 break;
				}
				case 1 :
				{
					tempx = atoi(valueBuf);  
					if((tempx <5)||(tempx >301))
					{
                         rxResult = -5;
					}							     	 
						rxt ++;
					break;
					}
				default :
					 break;						  						   
				}
						break;

				}
		    }
			if(rxResult != 0x00)
			{
                hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;    
				break;
			}
		}
    }
	if(0x00 == rxResult)
	{   
       if(THRESHOLD_VALUE1== thresholdType)
	   {
          if((thresholdTemp >3)&&(thresholdTemp <20))
		  {                                                 
             hyDataLs->confChangeFlag|= THRESHOLD_VOULUE1_SET_OK;
	         thresholdValue->moistureThreshold1 =thresholdTemp;
			 thresholdValue->threshold1Period = tempx*60;                  
			}
           else
           { 
              rxResult = -5;
           }
	     }
	     if(THRESHOLD_VALUE2== thresholdType)
	     {
             if((thresholdTemp >3)&&(thresholdTemp <60))
		     {
                hyDataLs->confChangeFlag|= THRESHOLD_VOULUE2_SET_OK;
                thresholdValue->moistureThreshold2=thresholdTemp;
			    thresholdValue->threshold2Period = tempx*60;
              }else
              {
                   rxResult = -5;
              }
	     }
         if(THRESHOLD_VALUE3== thresholdType)
	     {
            if((thresholdTemp >3)&&(thresholdTemp <60))
		    {
               hyDataLs->confChangeFlag|= THRESHOLD_VOULUE3_SET_OK;
               thresholdValue->moistureThreshold1 =thresholdTemp;
			   thresholdValue->threshold1Period = tempx*60;
            }else
            {  
                     rxResult = -5;
            }
	      }

		}
       if(rxResult != 0x00)
	   {
          hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR; 
       }                
	}
       return rxResult;
}


static int8_t findStationRtcTime(stationSend *hyDataLs)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   uint8_t xyzBuf[30];
        DS1302_TIME setTime;        
	uint16_t findStatus = UN_FIND_STATUS;
       uint16_t  tempx = 0;
//	float tempy  = 0.0;
//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
//	uint8_t rxh = 0x00;
//	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
	/*
      Ѱ�����ð汾��
	*/
	if(hyDataLs->confChangErrorFlag!=0x00)
	{
       return rxResult = -6;
    }
	findStatus = findStrRxGprs(" TT ",4);
	if(findStatus < FIND_ERROR)
	{
		rxResult = -2;
        for(rxi = 0x00;rxi <12;rxi ++ )
        {
		   if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//�ָ���
		   {
                rxResult = 0x00;
                xyzBuf[rxi] = 0x2c;									 
						 break;
			}
			else
			{
			   xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
               if((xyzBuf[rxi]< 0x2D)||(xyzBuf[rxi] > 0x3A))
			   {  
				     break;
			   }
			}
					 
		}
				//ȡֵ
    if(rxResult == 0x00)
    {
         valueBuf[0] = xyzBuf[rxj++];
         valueBuf[1] = xyzBuf[rxj++];
         valueBuf[2] = '\0';
         if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 100))
          {
            setTime.year = tempx;
          }
          else
          {
            
            rxResult = -4;
          }
          valueBuf[0] = xyzBuf[rxj++];
          valueBuf[1] = xyzBuf[rxj++];
          valueBuf[2] = '\0';
         if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 13)&&(tempx >0))
          {
            setTime.month = tempx;
          }
          else
          {
            
            rxResult = -4;
          }          


           valueBuf[0] = xyzBuf[rxj++];
           valueBuf[1] = xyzBuf[rxj++];
           valueBuf[2] = '\0';
         if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 32)&&(tempx >0))
          {
            setTime.date = tempx;
          }
          else
          {
            
            rxResult = -4;
          }             
                 


           valueBuf[0] = xyzBuf[rxj++];
           valueBuf[1] = xyzBuf[rxj++];
           valueBuf[2] = '\0';
         if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 25))
          {
            setTime.hour = tempx;
          }
          else
          {
            
            rxResult = -4;
          } 
                 

            valueBuf[0] = xyzBuf[rxj++];
            valueBuf[1] = xyzBuf[rxj++];
            valueBuf[2] = '\0';
         if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 61))
          {
            setTime.min= tempx;
          }
          else
          {
            
            rxResult = -4;
          } 
             valueBuf[0] = xyzBuf[rxj++];
             valueBuf[1] = xyzBuf[rxj++];
             valueBuf[2] = '\0';
          if(valueBuf[0]==0x30)
		 {
            tempx = valueBuf[1] -0x30;
          }else
          {
             tempx = atoi(valueBuf);
             
          }
          if((tempx < 61))
          {
            setTime.sec= tempx;
          }
          else
          {
            
            rxResult = -4;
          }  
      
    }          
              

		            //showDs1302Time(setTime);					 
	if(rxResult == 0x00)
	{
		hyDataLs->confChangeFlag|= STATION_RTC_SET_OK;
		setTimeTask(setTime);	
	}else
	{
       hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;    

    }
                
    }
       return rxResult;
}



static int8_t findHistoryDataByTime(stationSend *hyDataLs,queryTime *time)
{
       char valueBuf[MAX_CONFIG_BUF_SIZE];
	   uint8_t xyzBuf[10];
       // DS1302_TIME setTime;        
	uint16_t findStatus = UN_FIND_STATUS;
    uint16_t findEndStatus = UN_FIND_STATUS;
       uint16_t  tempx = 0;
//	float tempy  = 0.0;
//	float tempz   = 0.0;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
//	uint8_t rxh = 0x00;
//	uint8_t rxt = 0x00;
	int8_t rxResult = -1;
     //configure frame find
     findStatus = findStrRxGprs(" TB ",4);
     findEndStatus = findStrRxGprs(" TE ",4);
	 if(findStatus < FIND_ERROR)
	 {//
		  rxResult = -2;
                for(rxi = 0x00;rxi <8;rxi ++ )
                	{
				     if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//�ָ���
					 {
                                     xyzBuf[rxi] = 0x2c;									 
						 break;
					 }
					 else
					 {
					  xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                      if((xyzBuf[rxi]< 0x2D)||(xyzBuf[rxi] > 0x3A))
					   {  
                          hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;   
						      rxResult = -3;
						      break;
						}
					}
					 
				}
				//ȡֵ
				if(rxi == 8)
				{
                rxResult = 0x00;
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->begMon= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->begMon = tempx;
                }
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->begDate= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->begDate = tempx;
                }  
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';                
                if(valueBuf[0]==0x30)
				{
                time->begHour= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->begHour= tempx;
                }
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->begMin= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->begMin= tempx;
                } 
		}
        else
        {
            rxResult = -4;
        }     
     }
     else
     {
           rxResult = -5;
           return rxResult;
     }

     if((findEndStatus < FIND_ERROR)&&(rxResult == 0))                
	 {//
		  //rxResult = -2;
         rxj = 0x00;
         for(rxi = 0x00;rxi <8;rxi ++ )
         {
		    if((GPRS_RX_Buffer[findEndStatus+rxi]==0x20))//�ָ���
			{
                 xyzBuf[rxi] = 0x2c;									 
				break;
			}
		    else
			{
			    xyzBuf[rxi] = GPRS_RX_Buffer[findEndStatus+rxi];
                if((xyzBuf[rxi]< 0x2D)||(xyzBuf[rxi] > 0x3A))
			    { 
                    hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;   
				    rxResult = -3;
				    break;
				}
			}					 
		}
				//ȡֵ
		if((rxi ==8)&&(rxResult == 0))
		{       
                rxResult = 0;
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->endMon= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->endMon = tempx;
                }
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->endDate= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->endDate = tempx;
                }
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';                
                if(valueBuf[0]==0x30)
				{
                time->endHour= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->endHour= tempx;
                }
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                time->endMin= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                time->endMin= tempx;
                } 	 
            hyDataLs->requestFlag|= REQUEST_HISTORY_DATA_BY_TIME;   
    
		    }
            else
            {
             rxResult = -4;
            }
                
       
     }
     else
     {
          hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;   
		  rxResult = -3;
     }

     return rxResult;

}
int8_t findBeginRequestTime(stationSend *hyDataLs,queryTime *time)
{
    char valueBuf[4];
	   char xyzBuf[10];
    uint16_t  tempx = 0;
    uint16_t findStatus = UN_FIND_STATUS;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
        	 
    int8_t rxResult = -1;
    
    findStatus = findStrRxGprs("TB ",3);

     if(FIND_ERROR > findStatus)
     {
		  rxResult = -2;
          for(rxi = 0x00;rxi <8;rxi ++ )
          {
			if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//�ָ���
			{
                 xyzBuf[rxi] = 0x2c;
                 printf("\n\r find %d  \n\r",rxi);
                rxResult = 0x00;                
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                if((xyzBuf[rxi]< 0x2F)||(xyzBuf[rxi] > 0x3A))
				{  
                      printf("\n\r find err %d  \n\r",rxi);				   
				   break;
			     }
			}			 
		  }
				//ȡֵ
       if((rxResult==0x00)&&(rxi == 8))
		  {
                rxResult = 0x00;
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                }
                if(tempx <13)
                {
                 time->begMon = tempx;
                }
                else
                {
                rxResult = -4;
                }
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
               
                }
                if((tempx <32))
                {
                 time->begDate = tempx;
                }
                else
                {
                rxResult = -4;
                }               
 
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
               
                }
                if((tempx <25))
                {
                 time->begHour = tempx;
                }
                else
                {
                rxResult = -4;
                }                 

                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                 if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
               
                }
                if((tempx <61))
                {
                  time->begMin = tempx;
                }
                else
                {
                rxResult = -4;
                }                
 
		}
        if(rxResult != 0x00)
        {
           hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;  
        }          
     }         
    return rxResult;
}

int8_t findEndRequestTime(stationSend *hyDataLs,queryTime *time)
{
    char valueBuf[4];
	char xyzBuf[10];
    uint16_t  tempx = 0;
    uint16_t findStatus = UN_FIND_STATUS;
	uint8_t rxi = 0x00;
	uint8_t rxj = 0x00;
        	 
    int8_t rxResult = -1;
    
    findStatus = findStrRxGprs("TE ",3);
     if(FIND_ERROR > findStatus)
     {
		  rxResult = -2;
          for(rxi = 0x00;rxi <8;rxi ++ )
          {
			if((GPRS_RX_Buffer[findStatus+rxi]==0x20))//�ָ���
			{
                 xyzBuf[rxi] = 0x2c;
                 printf("\n\r find %d  \n\r",rxi);
                rxResult = 0x00;                
				break;
			}
			else
			{
				xyzBuf[rxi] = GPRS_RX_Buffer[findStatus+rxi];
                if((xyzBuf[rxi]< 0x2F)||(xyzBuf[rxi] > 0x3A))
				{  
                      printf("\n\r find err %d  \n\r",rxi);				   
				   break;
			     }
			}			 
		  }
				//ȡֵ
       if((rxResult==0x00)&&(rxi == 8))
		  {
                rxResult = 0;
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                }
                if((tempx <13))
                {
                 time->endMon = tempx;
                }
                else
                {
                rxResult = -4;
                }           

                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                }
                if((tempx <32))
                {
                 time->endDate = tempx;
                }
                else
                {
                rxResult = -4;
                }
                
                valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                }
                if((tempx <25))
                {
                 time->endHour = tempx;
                }
                else
                {
                rxResult = -4;
                }
                
                 valueBuf[0] = xyzBuf[rxj++];
                valueBuf[1] = xyzBuf[rxj++];
                valueBuf[2] = '\0';
                if(valueBuf[0]==0x30)
				{
                tempx= valueBuf[1] -0x30;
                }else
                {
                tempx = atoi(valueBuf);
                }
                if((tempx <61))
                {
                 time->endMin = tempx;
                }
                else
                {
                rxResult = -4;
                }               
 
		}
        if(rxResult != 0x00)
        {
           hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;  
        }          
     }         
    return rxResult;
}

static int8_t instructionRequest(stationSend *hyDataLs)
{
    int8_t result = -1;
    //uint16_t rxi = 0x00;
    uint16_t findStatus = UN_FIND_STATUS;
    uint16_t findSataus1 = UN_FIND_STATUS;
    //uint16_t frameLength = 0x00;
  //  uint16_t begAddress = 0x00;
  //  uint16_t endAddress = 0x00;


    findStatus = findStrRxGprs(" TB ",4);
    findSataus1 = findStrRxGprs(" TE ",4);
	if((findStatus < FIND_ERROR)&&(findSataus1 < FIND_ERROR))
	{  
       result = 0x00;
      // frameLength += 6;
       hyDataLs->requestFlag |= HISTORY_DATA_REQUEST;
    }
    //
    findStatus = findStrRxGprs(" ST ? ",6);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 6;
       hyDataLs->requestFlag |= STATION_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" PW ? ",6);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 6;     
       hyDataLs->requestFlag |= PW_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" TT ? ",6);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 6;    
       hyDataLs->requestFlag |= STATION_RTC_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" FF01 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;    
       hyDataLs->requestFlag |= CONFIG_ABC_RQQUEST;
    }  
    //
    findStatus = findStrRxGprs(" FF02 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;    
       hyDataLs->requestFlag |=SMS_KEY_RQQUEST;
    }  
    //
    findStatus = findStrRxGprs(" FF03 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;      
       hyDataLs->requestFlag |=SMS_PHONE_RQQUEST;
    }  
    //
    findStatus = findStrRxGprs(" FF04 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=PERIOD_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" FF05 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;     
       hyDataLs->requestFlag |=DENSITY_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" FF06 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;     
       hyDataLs->requestFlag |=FIX_PERIOD_RQQUEST;
    }
    findStatus = findStrRxGprs(" FF07 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=SHARK_KEY_REQUEST;
    }
    findStatus = findStrRxGprs(" FF08 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=SHARK_PHONE_REQUEST;
    }     
    //
    findStatus = findStrRxGprs(" FF09 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=THRESHOLD_KEY_RQQUEST;
    }    
    //
    findStatus = findStrRxGprs(" FF10 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=THRESHOLD_VOULUE1_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" FF11 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=THRESHOLD_VOULUE2_RQQUEST;
    }
    //
    findStatus = findStrRxGprs(" FF12 ? ",8);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=THRESHOLD_VOULUE3_RQQUEST;
    }
    //
   
    //
//     findStatus = findStrRxGprs(" FF09 ? ",8);
// 	if(findStatus < FIND_ERROR)
// 	{  
//        result = 0x00;
//        frameLength += 8;  
//        hyDataLs->requestFlag |=THRESHOLD_KEY_RQQUEST;
//     }  
    findStatus = findStrRxGprs(" HOST1 ? ",9);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=HOST1_RQQUEST;
    }     
     findStatus = findStrRxGprs(" HOST2 ? ",9);
	if(findStatus < FIND_ERROR)
	{  
       result = 0x00;
       //frameLength += 8;  
       hyDataLs->requestFlag |=HOST2_RQQUEST;
    }      
    return result ;
}

#define ONE_HOUR_MIN      60
#define ONE_DATE_MIN      1440

static int8_t historyDataQueryOkJudge(stationSend *hyDataLs,queryTime time,DS1302_TIME curTime)
{
    int8_t result = 0x00;
    //uint16_t rxi = 0x00;
    //uint16_t status = UN_FIND_STATUS;
    
    uint16_t curTimeMin = 0x00;
    uint16_t endTime = 0x00;
    curTimeMin = curTime.date*ONE_DATE_MIN + curTime.hour*60 + curTime.min;
    endTime = time.endDate*ONE_DATE_MIN + time.endHour*60 + time.endMin;
    
    if(time.endMon > curTime.month)
    {
        hyDataLs->requestErrorFlag |= REQUEST_OVERRUN_ERROR;   
        hyDataLs->requestFlag = 0;       
          result = -1;
          return result;
    }
    else
    {
       if(time.endMon == curTime.month)
       {
          if(endTime > curTimeMin)
          {
        hyDataLs->requestErrorFlag |= REQUEST_OVERRUN_ERROR;   
        hyDataLs->requestFlag = 0;       
          result = -1;
          return result;
           }
       }
    }
   
    return result;
}

static int8_t instructionRequestJudge(stationSend *hyDataLs,queryTime *time,DS1302_TIME nowTeim)
{
    int8_t judgeResult = -1;

    judgeResult = instructionRequest(hyDataLs);
    if(judgeResult == 0x00)
    {
  //��ʷ���ݲ�ѯ����
       if(hyDataLs->requestFlag&HISTORY_DATA_REQUEST)
       {
           if(hyDataLs->historyDataFlag != FUNCITONG_TURN_OFF)
           {
              hyDataLs->requestErrorFlag |= CONFIG_VALUE_ERROR;
              hyDataLs->requestFlag &= (~HISTORY_DATA_REQUEST);
           }
           else
           {
//               (void)findBeginRequestTime(hyDataLs,time);
//               if(!findEndRequestTime(hyDataLs,time))
//               {
//                  historyDataQueryOkJudge(hyDataLs,*time,nowTeim);
//               }
              if(!findHistoryDataByTime(hyDataLs,time))
              {
                 historyDataQueryOkJudge(hyDataLs,*time,nowTeim);
              }
              else
              {
              hyDataLs->requestFlag &= (~HISTORY_DATA_REQUEST);
              }
           }
        }
      
    }
    return judgeResult;
}




static void stationNamCopy(uint8_t *bufIn,uint8_t *bufOut)
{
    uint8_t i = 0x00;
    for(i = 0x00;i< bufIn[0]+1;i++)
    {
        bufOut[i] = bufIn[i];
    }

}

void stationServerCopy(serverAddr serverIn,serverAddr *serverOut)
{
//   uint16_t datalength = 0x00;
//   uint16_t bufLength  = 0x00;
   uint8_t testi = 0x00;

   serverOut->userFlag = serverIn.userFlag;
   serverOut->setFlag = serverIn.setFlag;
   serverOut->sendPort = serverIn.sendPort;
   for(testi = 0x00;testi < serverIn.baseAddressBuf[0];testi ++)
   {
       serverOut->baseAddressBuf[testi] = serverIn.baseAddressBuf[testi];
   }
   for(testi = 0x00;testi < serverIn.sendAddressBuf[0];testi ++)
   {
       serverOut->sendAddressBuf[testi] = serverIn.sendAddressBuf[testi];
   }    
}

static int8_t senseorDataFindByTime(queryTime time,stationSend *curStSend,uint8_t *buf)
{
    DS1302_TIME dataTime;
//    uint8_t buf[SENSOR_DATA_STORE_SIZE+1];
    uint32_t readAddr = 0x00;
    uint16_t startTime = 0x00;
    uint16_t curDataTime = 0x00;
    uint16_t endTime = 0x00;
    uint8_t findSatae = 1;
    uint16_t data = 0;
//    uint16_t i = 0x00;
    uint8_t dataFlag = 0x00;
    int8_t findResult = -1;
		readAddr = curStSend->curSendAddress;

    startTime = time.begDate*ONE_DATE_MIN + time.begHour*ONE_HOUR_MIN + time.begMin;
    endTime = time.endDate*ONE_DATE_MIN + time.endHour*ONE_HOUR_MIN + time.endMin;
     printf("\r\n��ʼʱ�� [%d]  [%d]\r\n",startTime,endTime);
	 power331On();//�򿪴洢FLASH��Դ
 	 SPI_Flash_WAKEUP();
  	 uint16_t readLoopCount = 0;
  	 while(readLoopCount++ < 10000)
  	 	{
             //��ȡ����	 	   	
	    SPI_FLASH_BufferRead(buf,readAddr,SENSOR_DATA_STORE_SIZE);
        //usart2StoreBuffer(buf,21);	    
	    //���������Ƿ��ǺϷ����ݰ�
	   if((buf[0]==0x2c)&&(buf[1]==0x2c))
	   	{
           dataFlag = 0x00;
	       
//ʱ���ȡ
    dataTime.year  = buf[4];
	dataTime.month = buf[5];
	dataTime.date  = buf[6];
	dataTime.hour  = buf[7];
	dataTime.min   = buf[8];
	dataTime.sec   = buf[9];
    	showDs1302Time(dataTime);
    curDataTime = dataTime.date*ONE_DATE_MIN +dataTime.hour*ONE_HOUR_MIN + dataTime.min;
//�����ж�
     switch(findSatae)
     {
         case 1 :
         {   
             dataFlag = 0x00;
             if(time.endMon == dataTime.month)
             { 
                if(endTime>curDataTime)
                {
                    if((time.begMon!= dataTime.month))
                    {    
                    //������ʼʱ��
                     data ++;
                     printf("\r\n��ʼʱ�� %dr\n",curStSend->requestDataEndAddress); 
                     curStSend->requestDataEndAddress = readAddr;
                     findSatae ++;
                    }
                    else
                    {
                        if((time.begMon== dataTime.month)&&(startTime < curDataTime))
                        {
                          //������ʼʱ��
                          data ++;
                          printf("\r\n��ʼʱ�� %dr\n",curStSend->requestDataEndAddress); 
                          curStSend->requestDataEndAddress = readAddr;
                          findSatae ++;
                        }
                    }
                    
                }
             }
             if((time.begMon== dataTime.month)&&(startTime > curDataTime))
             {
             printf("\r\nδ��ѯ����ʷ���������˳�r\n"); 
 
                 dataFlag = 0x07;
             }
             
           break;
         }
         case 2 :
         {
             data ++;
             printf("\r\n��ʷ������ %dr\n",data); 

             dataFlag = 0x00;
             curStSend->requestDataReadAddress = readAddr;
             //������ʼʱ��
             if(time.begMon == dataTime.month)
             { 
                                     
                if(startTime >=curDataTime)
                {
                     data --;
                     curStSend->requestDataReadAddress = readAddr +SENSOR_DATA_STORE_SIZE;
                     //printf("\r\n��ʷ������ %d  %d\r\n",data,curStSend->requestDataReadAddress); 
                     if(curStSend->requestDataReadAddress >= SENSOR_DATA_END_ADDR )
                     {
                       curStSend->requestDataReadAddress = SENSOR_DATA_BASE_ADDR;
                     }
                     findSatae ++;
                    
                }

             }
           break;
         }
         default :
         {
                break;
         }
      }                       
	   }
       else
	   	{
           dataFlag ++;
	   }
	   if(dataFlag > 0x06)
	   	{
                break;
	   }
       if(data > 499)
       {
         break;
       }
       if(findSatae >=3)
       {
           break;
       }
	   readAddr -=SENSOR_DATA_STORE_SIZE;
	   if(SENSOR_DATA_BASE_ADDR >=readAddr)
	   {
                readAddr = SENSOR_DATA_END_ADDR - SENSOR_DATA_STORE_SIZE;
	   }
	    		 
	  }
	 SPI_Flash_PowerDown();
 	 power331Off();
     if((data > 0x00)&&(data < 501))
     {
       curStSend->requestData = data;
       findResult = 0x00;
     }
	  return findResult;   
}
void stationServerAddrDebug(void)
{
   uint16_t datalength = 0x00;
   uint16_t bufLength  = 0x00;
   uint8_t testi = 0x00;
   for(testi = 0x00;testi < STATION_MULTI_CONNECTION_SIZE;testi ++)
   { 
		 stationServerAddrBuf[testi].userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;
		 stationServerLSRLog[testi].linkErrorCounter = 0x00;
		 stationServerLSRLog[testi].sendErrorCounter = 0x00;
		 stationServerLSRLog[testi].rxErrorCounter = 0x00;
		 stationServerLSRLog[testi].dataOkCounter= 0x00;
   }

   stationServerAddrBuf[0].userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
   stationServerAddrBuf[0].sendPort = 80;

   datalength = 0x01;
   
   //117.139.194.62
   bufLength =  sprintf(stationServerAddrBuf[0].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"183.232.33.116\",\"9205\"\r\n");
//                    bufLength =  sprintf(tcpServerAddrBuf[1].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"121.40.142.131\",\"60157\"\r\n");

   stationServerAddrBuf[0].sendPort = 9205;
   stationServerAddrBuf[0].sendAddressBuf[0]=bufLength;                
   datalength +=bufLength;
            
   datalength = 0x01;
   bufLength =  sprintf(stationServerAddrBuf[0].baseAddressBuf+datalength, "%s", "183.232.33.116");
   datalength +=bufLength;
   stationServerAddrBuf[0].baseAddressBuf[0] = datalength;
            
   datalength = 0x01;
   bufLength = 0x00;
      datalength = 0x01;
   bufLength =  sprintf(stationServerAddrBuf[1].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"121.10.203.49\",\"9205\"\r\n");
//                    bufLength =  sprintf(tcpServerAddrBuf[1].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"121.40.142.131\",\"60157\"\r\n");

   stationServerAddrBuf[1].sendPort = 9205;
   stationServerAddrBuf[1].sendAddressBuf[0]=bufLength;                
   datalength +=bufLength;
            
   datalength = 0x01;
   bufLength =  sprintf(stationServerAddrBuf[1].baseAddressBuf+datalength, "%s", "121.10.203.49");
   datalength +=bufLength;
   stationServerAddrBuf[1].baseAddressBuf[0] = datalength;

 //               bufLength =  sprintf(tcpServerAddrBuf[1].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"sq.sunkingsystem.com\",\"9000\"\r\n");
            //  bufLength =  sprintf(tcpServerAddrBuf[0].sendAddressBuf+datalength, "%s", "AT+CIPSTART=\"TCP\",\"api.insentek.com\",\"80\"\r\n");

   datalength +=bufLength;
   //tcpServerAddrBuf[0].sendAddressBuf[0]= 0x2b;
   // serverCounter = 0x02;    
   //  tcpServerAddrBuf[1].sendAddressBuf[0]= 0x31;                  

}

void stationServerBufInt(void)
{
          uint8_t tcpLinkChannel = 0x00;
            for(tcpLinkChannel= 0x00;tcpLinkChannel < STATION_MULTI_CONNECTION_SIZE;tcpLinkChannel++)
            {
                if(stationServerAddrBuf[tcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
                {
                stationServerLSRLog[tcpLinkChannel].setTry = 0x00;
                stationServerLSRLog[tcpLinkChannel].allTry  = 0x00;
                stationServerLSRLog[tcpLinkChannel].linkTry = 0x00;
                stationServerLSRLog[tcpLinkChannel].sendTry = 0x00;
                stationServerLSRLog[tcpLinkChannel].rxTry = 0x00;
                stationServerLSRLog[tcpLinkChannel].rxDataTry= 0x00;
                stationServerLSRLog[tcpLinkChannel].sendFlag = UNSEND_FLAG;
                if(stationServerLSRLog[tcpLinkChannel].gprsRegErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].gprsRegErrorCounter = 0x00;
                 }                    
                if(stationServerLSRLog[tcpLinkChannel].gprsAttachErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].gprsAttachErrorCounter = 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].gprsBringUpErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].gprsBringUpErrorCounter= 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].gprsApnSetErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].gprsApnSetErrorCounter= 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].getIpErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].getIpErrorCounter = 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].linkErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].linkErrorCounter = 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].sendErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].sendErrorCounter= 0x00;
                 } 
                 if(stationServerLSRLog[tcpLinkChannel].rxErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].rxErrorCounter= 0x00;
                 }                  
                if(stationServerLSRLog[tcpLinkChannel].dataOkCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].dataOkCounter = 0x00;
                }
                 if(stationServerLSRLog[tcpLinkChannel].rxDataErrorCounter > 9800)
                {
                   stationServerLSRLog[tcpLinkChannel].rxDataErrorCounter = 0x00;
                 }                  
                    
                }
                else
                {
                stationServerLSRLog[tcpLinkChannel+1].allTry  = 0x04;              
                
                }       
            }
        
}

static uint16_t confErrorCodeLoad(char *buf,stationSend hyDataLs,uint16_t index)
{
    uint8_t sendi = 0x00;
    uint16_t sendj = 0x00;
    sendj = index;

       if(hyDataLs.confChangErrorFlag&FRAME_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",1);
	    sendj+=sendi;
        return 	sendj;
      }   
      if(hyDataLs.confChangErrorFlag&CONFIG_VALUE_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",5);
	    sendj+=sendi;
        return 	sendj;
      }
      if(hyDataLs.confChangErrorFlag&KEY_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",4);
	    sendj+=sendi;
        return 	sendj;
      }
      if(hyDataLs.confChangErrorFlag&CONFIG_ID_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",3);
	    sendj+=sendi;
        return 	sendj;
      }      
      return sendj;
}


static uint16_t requestErrorCodeLoad(char *buf,stationSend hyDataLs,uint16_t index)
{
    uint8_t sendi = 0x00;
    uint16_t sendj = 0x00;
    sendj = index;
	sendi =  sprintf(buf+sendj, " %s","ER");
	sendj+=sendi;
    
      if(hyDataLs.requestErrorFlag&CONFIG_VALUE_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",5);
	    sendj+=sendi;
        return 	sendj;
      }
      if(hyDataLs.requestErrorFlag&KEY_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",4);
	    sendj+=sendi;
        return 	sendj;
      }
      if(hyDataLs.requestErrorFlag&REQUEST_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",9);
	    sendj+=sendi;
        return 	sendj;
      }
       if(hyDataLs.requestErrorFlag&CONFIG_ID_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",3);
	    sendj+=sendi;
        return 	sendj;
      }    
       if(hyDataLs.requestErrorFlag&REQUEST_OVERRUN_ERROR)
      {
	    sendi =  sprintf(buf+sendj, " %d",6);
	    sendj+=sendi;
        return 	sendj;
      }       
      return sendj;
}


  
static uint16_t confAbcDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,soilabc *curAbcValue)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&CONFIG_ABC_0K)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF01 ");
    index += sendi;
    for(nodei = 0x00;nodei < devSenNum;nodei++)
    {
	sendi =  sprintf((char *)(buf+index), "%.4f,%.4f,%.4f,",curAbcValue[nodei].a.f,curAbcValue[nodei].b1.f,curAbcValue[nodei].c.f);
    index += sendi;
//     sendi =  sprintf((char *)(buf+index), ",%.4f",curAbcValue[nodei].b1.f);
//     index += sendi;
//     sendi =  sprintf((char *)(buf+index), "",curAbcValue[nodei].c.f);
//     index += sendi;
    
    }
    index --;

   }
       return index;
}

static uint16_t requestAbcDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,soilabc *curAbcValue)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&CONFIG_ABC_0K)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF01 ");
    index += sendi;
    for(nodei = 0x00;nodei < devSenNum;nodei++)
    {
	sendi =  sprintf((char *)(buf+index), "%.4f,%.4f,%.4f,",curAbcValue[nodei].a.f,curAbcValue[nodei].b1.f,curAbcValue[nodei].c.f);
    index += sendi;
    
    }
    index --;

   }
       return index;
}

 uint16_t confSmsKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig key)
{
   uint8_t sendi = 0x00;
   uint8_t data = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   
   if(hyDataLs.confChangeFlag&SMS_KEY_SET_OK)
   {
    //��ʾ��װ��
// 	sendi =  sprintf((char *)(buf+index), "%s","");
//     index += sendi;
    data = key.devWorkState - 0x30;
	sendi =  sprintf((char *)(buf+index), " FF02 %d",data);
    index += sendi;


   }
       return index;
}

 uint16_t requestSmsKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig key)
{
   uint8_t sendi = 0x00;
   uint8_t data = 0x00;

//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&SMS_KEY_SET_OK)
   {
    //��ʾ��װ��
    data = key.devWorkState - 0x30;
	sendi =  sprintf((char *)(buf+index), " FF02 %d",data);
    index += sendi;

   }
       return index;
}


 uint16_t confSmsPhoneDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,phone sendPhone)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&SMS_PHONE_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF03 ");
    index += sendi;
    for(nodei = 0x00;nodei < sendPhone.phonelength;nodei++)
    {
       buf[index++]= sendPhone.phoneNumORG[nodei];
    }
   }
       return index;
}

 uint16_t requestSmsPhoneDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,phone sendPhone)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&SMS_PHONE_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF03 ");
    index += sendi;
    for(nodei = 0x00;nodei < sendPhone.phonelength;nodei++)
    {
       buf[index++]= sendPhone.phoneNumORG[nodei];
    }
   }
       return index;
}




 uint16_t confSharkPhoneDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,phone sendPhone)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&SHARK_PHONE_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF08 ");
    index += sendi;
    for(nodei = 0x00;nodei < sendPhone.phonelength;nodei++)
    {
       buf[index++]= sendPhone.phoneNumORG[nodei];
    }
   }
       return index;
}

 uint16_t requestSharkPhoneDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,phone sendPhone)
{
   uint8_t sendi = 0x00;
   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&SHARK_PHONE_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," FF08 ");
    index += sendi;
    for(nodei = 0x00;nodei < sendPhone.phonelength;nodei++)
    {
       buf[index++]= sendPhone.phoneNumORG[nodei];
    }
   }
       return index;
}

static uint16_t confdevPerDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig period)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&PERIOD_SET_OK)
   {
    //��ʾ��װ��
// 	sendi =  sprintf((char *)(buf+index), "%s"," FF04 ");
//     index += sendi;
	sendi =  sprintf((char *)(buf+index), " FF04 %d",(period.devPeriod/60));
    index += sendi;


   }
       return index;
} 
static uint16_t requestdevPerDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig period)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&PERIOD_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF04 %d",(period.devPeriod/60));
    index += sendi;

   }
       return index;
} 

static uint16_t confdevFixPerDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig period)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&FIX_PERIOD_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF06 %d,%d",period.devPeriodHour,period.devPeriodMin);
    index += sendi;


   }
       return index;
} 


static uint16_t requestdevFixPerDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig period)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&FIX_PERIOD_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF06 %d,%d",period.devPeriodHour,period.devPeriodMin);
    index += sendi;

   }
       return index;
} 

static uint16_t confDensityDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint16_t density)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&DENSITY_SET_OK)
   {
    //��ʾ��װ��
// 	sendi =  sprintf((char *)(buf+index), "%s"," FF05 ");
//     index += sendi;
	sendi =  sprintf((char *)(buf+index), " FF05 %d",density);
    index += sendi;

   }
       return index;
} 

static uint16_t requestDensityDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint16_t density)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&DENSITY_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF05 %d",density);
    index += sendi;

   }
       return index;
} 



static uint16_t confThresholdKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t key)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&THRESHOLD_KEY_SET_OK)
   {
    //��ʾ��װ��
// 	sendi =  sprintf((char *)(buf+index), "%s"," FF09 ");
//     index += sendi;
	sendi =  sprintf((char *)(buf+index), " FF09 %d",(key-0x30));
    index += sendi;


   }
       return index;
}

static uint16_t requestThresholdKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t key)
{
   uint8_t sendi = 0x00;
//   uint8_t data = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&THRESHOLD_KEY_SET_OK)
   {
    //��ʾ��װ��

   // data = key -0x30;
	sendi =  sprintf((char *)(buf+index), " FF09 %d",(key-0x30));
    index += sendi;


   }
       return index;
}


static uint16_t confShareKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t key)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   uint8_t data = 0x00;
   if(hyDataLs.confChangeFlag&SHARK_KEY_OK)
   {
    //��ʾ��װ��

    data = key - 0x30;
	sendi =  sprintf((char *)(buf+index), " FF07 %d",data);
    index += sendi;


   }
       return index;
}
static uint16_t confResetDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
//   uint8_t data = 0x00;
   if(hyDataLs.confChangeFlag&STATION_RESET)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," 98");
    index += sendi;

   }
       return index;
}

static uint16_t confFlashResetDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
 //  uint8_t data = 0x00;
   if(hyDataLs.confChangeFlag&STATION_FLASH_RESET)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), "%s"," 97");
    index += sendi;

   }
       return index;
}

static uint16_t requestShareKKeyDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t key)
{
   uint8_t sendi = 0x00;
   uint8_t data = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&SHARK_KEY_OK)
   {
    //��ʾ��װ��
    data = key - 0x30;
	sendi =  sprintf((char *)(buf+index), " FF07 %d",data);
    index += sendi;


   }
       return index;
}

static uint16_t confThresholdValue1DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&THRESHOLD_VOULUE1_SET_OK)
   {
    //��ʾ��װ��

	sendi =  sprintf((char *)(buf+index), " FF10 %d,%d",thresholdValue.moistureThreshold1,(thresholdValue.threshold1Period/60));
    index += sendi;


   }
       return index;
}
static uint16_t requestThresholdValue1DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&THRESHOLD_VOULUE1_SET_OK)
   {
    //��ʾ��װ��

	sendi =  sprintf((char *)(buf+index), " FF10 %d,%d",thresholdValue.moistureThreshold1,(thresholdValue.threshold1Period/60));
    index += sendi;

   }
       return index;
}
static uint16_t confThresholdValue2DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&THRESHOLD_VOULUE2_SET_OK)
   {
    //��ʾ��װ��

	sendi =  sprintf((char *)(buf+index), " FF11 %d,%d",thresholdValue.moistureThreshold2,(thresholdValue.threshold2Period/60));
    index += sendi;


   }
       return index;
}
static uint16_t requestThresholdValue2DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&THRESHOLD_VOULUE2_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF11 %d,%d",thresholdValue.moistureThreshold2,(thresholdValue.threshold2Period/60));
    index += sendi;

   }
       return index;
}

static uint16_t confThresholdValue3DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.confChangeFlag&THRESHOLD_VOULUE3_SET_OK)
   {
    //��ʾ��װ��

	sendi =  sprintf((char *)(buf+index), " FF12 %d,%d",thresholdValue.moistureThreshold3,(thresholdValue.threshold3Period/60));
    index += sendi;


   }
       return index;
}

static uint16_t requestThresholdValue3DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,devConfig thresholdValue)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   if(hyDataLs.requestFlag&THRESHOLD_VOULUE3_SET_OK)
   {
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " FF12 %d,%d",thresholdValue.moistureThreshold3,(thresholdValue.threshold3Period/60));
    index += sendi;

   }
       return index;
}

static uint16_t confRtcDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;
   	DS1302_TIME time;
   
   if(hyDataLs.confChangeFlag&STATION_RTC_SET_OK)
   {
            readDs1302Time(&time); 
    //��ʾ��װ��
	sendi =  sprintf((char *)(buf+index), " TT %02d%02d%02d%02d%02d%02d",time.year,time.month,time.date,time.hour,time.min,time.sec);
    index += sendi;


   }
       return index;
}    


static uint16_t confStationDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t *station)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.confChangeFlag&STATION_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","ST");
     index +=sendi;
     for(sendi = 1;sendi < station[0];sendi++)
     {
        buf[index++] = station[sendi];
     }
   

   }
       return index;
}  


static uint16_t confStationPwDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t *station)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.confChangeFlag&PW_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","PW");
     index +=sendi;
     for(sendi = 1;sendi < station[0];sendi++)
     {
        buf[index++] = station[sendi];
     }
   

   }
       return index;
}  

static uint16_t confStationServer1DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,serverAddr station)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.confChangeFlag&HOST1_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","HOST1");
     index +=sendi;
     for(sendi = 1;sendi < station.baseAddressBuf[0];sendi++)
     {
        buf[index++] = station.baseAddressBuf[sendi];
     }
   
	 sendi =  sprintf((char *)(buf+index), ",%d",station.sendPort);
     index +=sendi;
   }
       return index;
}  

static uint16_t confStationServer2DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,serverAddr station)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.confChangeFlag&HOST2_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","HOST2");
     index +=sendi;
     for(sendi = 1;sendi < station.baseAddressBuf[0];sendi++)
     {
        buf[index++] = station.baseAddressBuf[sendi];
     }
 	 sendi =  sprintf((char *)(buf+index), ",%d",station.sendPort);
     index +=sendi;  

   }
       return index;
}  

static uint16_t confStationServerDelDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.confChangeFlag&HOST2_DEL_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","HOST2 del");
     index +=sendi;
   }
       return index;
}  

static uint16_t requestStationDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t *station)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.requestFlag&STATION_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","ST");
     index +=sendi;
     for(sendi = 1;sendi < station[0];sendi++)
     {
        buf[index++] = station[sendi];
     }
   

   }
   
       return index;
}

static uint16_t requestPwDataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,uint8_t *pwBuf)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.requestFlag&PW_SET_OK)
   {
	 sendi =  sprintf((char *)(buf+index), " %s ","PW");
     index +=sendi;
     for(sendi = 1;sendi < pwBuf[0];sendi++)
     {
        buf[index++] = pwBuf[sendi];
     }
   

   }
       return index;
}


static uint16_t requestStationServer1DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,serverAddr address)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.requestFlag&HOST1_SET_OK)
   {
     if(address.userFlag ==TCP_SERVER_ADDRESS_USE_FLAG)
     {
	 sendi =  sprintf((char *)(buf+index), " %s ","HOST1");
     index +=sendi;
     for(sendi = 1;sendi < address.baseAddressBuf[0];sendi++)
     {
        buf[index++] = address.baseAddressBuf[sendi];
         
     }
     sendi =  sprintf((char *)(buf+index), ",%d",address.sendPort);
     index +=sendi;
     }
     else
     {
	 sendi =  sprintf((char *)(buf+index), " %s","HOST1 no");
     index +=sendi;
     }
   

   }
       return index;
}  

static uint16_t requestStationServer2DataLoad(uint8_t *buf,stationSend hyDataLs,uint16_t index,serverAddr address)
{
   uint8_t sendi = 0x00;
//   uint8_t nodei = 0x00;
//   uint16_t sendj = 0x00;

   
   if(hyDataLs.requestFlag&HOST2_SET_OK)
   {
     if(address.userFlag ==TCP_SERVER_ADDRESS_USE_FLAG)
     {
	 sendi =  sprintf((char *)(buf+index), " %s ","HOST2");
     index +=sendi;
     for(sendi = 1;sendi < address.baseAddressBuf[0];sendi++)
     {
        buf[index++] = address.baseAddressBuf[sendi];
     }
     sendi =  sprintf((char *)(buf+index), ",%d",address.sendPort);
     index +=sendi;
     }
     else
     {
	 sendi =  sprintf((char *)(buf+index), " %s","HOST2 no");
     index +=sendi;
     }
   

   }
       return index;
}  

static uint16_t configReDataFromt(uint8_t *buf,stationSend hyDataLs)
{
   uint8_t sendi = 0x00;
   uint16_t length;
   // ͷ������װ��
   length = frHeadFromat(buf);
    //��ˮ������
    if(hyDataLs.confChangErrorFlag!=0x00)
    {
	sendi =  sprintf((char *)(buf+length), " %s","ER");
    }
    else
    {
     if(hyDataLs.confChangeFlag&RETURN_CN_FLAG)
     {
	sendi =  sprintf((char *)(buf+length), " RF %d",hyDataLs.configCounter);
        }

    }
    length +=sendi;
   // 
   return length;
     
}

static uint16_t requestReDataFromt(uint8_t *buf)
{
//   uint8_t sendi = 0x00;
   uint16_t length;
   // ͷ������װ��
   length = frHeadFromat(buf);
    //��ˮ������


   // 
   return length;

}
static uint16_t reSerialDataFromt(uint8_t *buf,uint16_t index,stationSend hyDataLs)
{
   uint8_t sendi = 0x00;
   // ͷ������װ��
	sendi =  sprintf((char *)(buf+index), " CN %d",hyDataLs.downSerialNum);
    index +=sendi;
   // 
   return index;
     
}
//���ݰ��ְ���־װ��
// static uint16_t diDataFromt(uint8_t *buf,uint16_t index,uint16_t unSend,uint8_t curSend)
// {
//     uint8_t sendi = 0x00;
// //    uint8_t diFlag = 0x00;
//    // ͷ������װ��
//    if((unSend - curSend)!= 0)
//    {
// 	sendi =  sprintf((char *)(buf+index), " %s","DI 1");
//     }
//     index +=sendi;
//    // 
//    return index;
//      
// }


#define HOST1     0x01
#define HOST2     0x02

static int8_t delHostNServer(uint8_t hostN,stationSend *hyDataLs)
{
       uint16_t findStatus = UN_FIND_STATUS;
	   int8_t rxResult = -1;
    if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
    {
       rxResult = -3;
    }
    if(hostN == HOST1)
    {
	findStatus = findStrRxGprs(" HOST1 del ",11);
    }
    else
    {
	findStatus = findStrRxGprs(" HOST2 del ",11);
    }
    if(findStatus < FIND_ERROR)
    {
         if(hostN == HOST1)
         {
            rxResult = -3;
            hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;
         }
          else
          {
            rxResult = 0x00;
	        hyDataLs->confChangeFlag |= HOST2_DEL_OK;
          }
         rxResult = 0x00;
    }

    return rxResult; 

}



static int8_t finsStationName(uint8_t *buf,stationSend *hyDataLs)
{
       int8_t rxResult = -1;
       uint8_t rxi = 0x00;
       uint16_t findStatus = UN_FIND_STATUS;
       if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
       {
       rxResult = -3;
       }       
	   findStatus = findStrRxGprs(" ST ",4);
	   if(findStatus < FIND_ERROR)
	   {
          rxResult = -2;
          for(rxi = 0x00;rxi <19;rxi ++ )
          {
			if(GPRS_RX_Buffer[findStatus+rxi]==0x20)//�ָ���
			{
                  buf[0] =rxi+1;
									 
				  break;
			}
		    else
		    {
					buf[rxi+1]= GPRS_RX_Buffer[findStatus+rxi];
			 } 

		  }
          
          if(rxi == 10)
          {    
               rxResult = 0x00;
	           hyDataLs->confChangeFlag |= STATION_SET_OK;
          }
          else
          {
               hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;

          }

       }
       return rxResult;

}


static int8_t finsStationPw(uint8_t *buf,stationSend *hyDataLs)
{
       int8_t rxResult = -1;
       uint8_t rxi = 0x00;
       uint16_t findStatus = UN_FIND_STATUS;

       if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
       {
       rxResult = -3;
       }    
       findStatus = findStrRxGprs(" PW ",4);
       if(findStatus < FIND_ERROR)
       {
       findStatus = findStrRxGprsFromAddress("PW ",3,findStatus);
        }
	   if(findStatus < FIND_ERROR)
	   {
          rxResult = -2;
          for(rxi = 0x00;rxi <19;rxi ++ )
          {
			if(GPRS_RX_Buffer[findStatus+rxi]==0x20)//�ָ���
			{
                  buf[0] =rxi+1;
									 
				  break;
			}
		    else
		    {
					buf[rxi+1]= GPRS_RX_Buffer[findStatus+rxi];
			 } 

		  }
          
          if(rxi == 6)
          {    
               rxResult = 0x00;
	           hyDataLs->confChangeFlag |= PW_SET_OK;
          }
          else
          {
               hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;

          }

       }
       return rxResult;

}


static int8_t finsStationReset(stationSend *hyDataLs)
{
       int8_t rxResult = -1;
//       uint8_t rxi = 0x00;
       uint16_t findStatus = UN_FIND_STATUS;
       uint16_t find1Status = UN_FIND_STATUS;

       if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
       {
       rxResult = -3;
       }    
       findStatus = findStrRxGprs(" PW ",4);
       if(findStatus < FIND_ERROR)
       {
       find1Status = findStrRxGprsFromAddress(" 98 ",4,findStatus);
        }
	   if((findStatus < FIND_ERROR)&&(find1Status < FIND_ERROR))
	   {
          rxResult = -2;
         
          if((find1Status - findStatus)==10)
          {
               hyDataLs->confChangeFlag |= STATION_RESET;
               rxResult = 0x00;
          }

       }
       return rxResult;

}


static int8_t finsStationFlashReset(stationSend *hyDataLs)
{
       int8_t rxResult = -1;
//       uint8_t rxi = 0x00;
       uint16_t findStatus = UN_FIND_STATUS;
       uint16_t find1Status = UN_FIND_STATUS;

       if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
       {
       rxResult = -3;
       }    
       findStatus = findStrRxGprs(" PW ",4);
       if(findStatus < FIND_ERROR)
       {
       find1Status = findStrRxGprsFromAddress(" 97 ",4,findStatus);
        }
	   if((findStatus < FIND_ERROR)&&(find1Status < FIND_ERROR))
	   {
          rxResult = -2;
          if((find1Status - findStatus)==10)
          {
               hyDataLs->confChangeFlag |= STATION_FLASH_RESET;
                              rxResult = 0x00;
          }

       }
       return rxResult;

}



static int8_t findHostServer(serverAddr *bAddr,uint8_t hostN,stationSend *hyDataLs)
{
       uint16_t findStatus = UN_FIND_STATUS;
       char valueBuf[50];
       char portBuf[10];
	   int8_t rxResult = -1;
	   uint8_t rxi = 0x00;
       uint8_t rxj = 0x00;
	   uint8_t rxLength = 0;
	   uint8_t  bLength =0;
       
    if(hyDataLs->confChangErrorFlag&CONFIG_VALUE_ERROR)
    {
       rxResult = -3;
    }
    if(hyDataLs->confChangeFlag&HOST2_DEL_OK)
    {
       rxResult = -4;
       return rxResult;
    }
    if(hostN == HOST1)
    {
	findStatus = findStrRxGprs(" HOST1 ",7);
    }
    else
    {
	findStatus = findStrRxGprs(" HOST2 ",7);
    }
	if(findStatus < FIND_ERROR)
    {	
	   rxResult = -2;
       for(rxi = 0x00;rxi <50;rxi ++ )
       {
		 if(GPRS_RX_Buffer[findStatus+rxi]==0x20)//�ָ���
		 {
               valueBuf[rxi] =0x2c;
			   rxResult = 0x00;			 
			   break;
		 }
		 else
		 {
			valueBuf[rxi]= GPRS_RX_Buffer[findStatus+rxi];
		} 

	  }
		if((rxi < 50)&&(rxResult == 0x00))
		{
              printf("server 1length %d\r\n",rxi);
			  rxLength = 0x01;
			  bLength=  sprintf(bAddr-> sendAddressBuf+rxLength, "%s", "AT+CIPSTART=\"TCP\",\"");
			  bLength++;
              for(rxLength = 0;rxLength < rxi;rxLength++)
              {
                  if(valueBuf[rxLength]!= 0x2C)
                  {
                         bAddr-> sendAddressBuf[bLength++] = valueBuf[rxLength];
                         bAddr-> baseAddressBuf[rxLength+1] = valueBuf[rxLength];  
				  }else
				  {    
                      bAddr-> baseAddressBuf[0] = rxLength;
                      break;
				  }                   
			  }
                printf("server 2length %d  s%d\r\n",bLength,rxLength);
				findStatus =  sprintf(bAddr-> sendAddressBuf+bLength, "%s", "\",\"");
					   //��ȡ�˿ں�
				bLength+= findStatus ;
               rxLength++;
               printf("server 3length %d  s%d\r\n",bLength,rxLength);
               rxResult = -2;
			   for(;rxLength < rxi;rxLength++)
			   {
                   if(valueBuf[rxLength]!= 0x2c)
                   {
                         bAddr-> sendAddressBuf[bLength++] = valueBuf[rxLength];
                         portBuf[rxj++] = valueBuf[rxLength];
                         if((valueBuf[rxLength] < 0x2F)||(valueBuf[rxLength]>0x3A))
                         {
                              rxResult = -3;
                         }
                                                							       				   
					}else
				    {
                          portBuf[rxj++] =  '\0';
                          rxResult = 0;
                          break;	
					}

				}
                if(rxResult ==0x00)
                {
                      printf("server 4length %d  s%d\r\n",bLength,rxLength);
					   findStatus =  sprintf(bAddr-> sendAddressBuf+bLength, "%s", "\"\r\n");
					   //��ȡ�˿ں�
					    bLength+= findStatus ;
                        printf("server 4length %d  s%d\r\n",bLength,rxLength);
                       bAddr->sendPort = atoi(portBuf);
					   bAddr-> sendAddressBuf[0] = bLength;
					   bAddr->setFlag = TCP_SERVER_ADDRESS_SET_FLAG ;
                       bAddr->userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
                       if(hostN == HOST1)
                       {
                          hyDataLs->confChangeFlag |= HOST1_SET_OK;
                        }
                        else
                        {
	                      hyDataLs->confChangeFlag |= HOST2_SET_OK;
                        }

				}
		}
        if(rxResult!= 0x00)
        {
               hyDataLs->confChangErrorFlag |= CONFIG_VALUE_ERROR;

        }
	}
              return rxResult;
}


uint16_t configCounterDataFromt(uint8_t *buf,stationSend hyDataLs,uint16_t index)
{
   uint8_t sendi = 0x00;
     if(hyDataLs.confChangeFlag&RETURN_CN_FLAG)
     {
   	sendi =  sprintf((char *)(buf+index), " %d",hyDataLs.configCounter);
    index += sendi;
        }
    return index;
}

uint16_t configReDataLoad(uint8_t *buf,stationSend hyDataLs,uint8_t *name,uint8_t *pwBuf,serverAddr serverBuf)
{
    uint16_t configReLength = 0x00;
  
   configReLength = configReDataFromt(buf,hyDataLs);
   if(hyDataLs.confChangErrorFlag== 0)
   {
     // configReLength = configCounterDataFromt(buf,hyDataLs,configReLength);

      configReLength = confAbcDataLoad(buf,hyDataLs,configReLength,curAbc);
  //    configReLength = confSmsKeyDataLoad(buf,hyDataLs,configReLength,configValue);
 //     configReLength = confSmsPhoneDataLoad(buf,hyDataLs,configReLength,smsNum[0]);
//      configReLength = confSharkPhoneDataLoad(buf,hyDataLs,configReLength,ctrNum[1]);
      configReLength = confdevPerDataLoad(buf,hyDataLs,configReLength,configValue);
      configReLength = confdevFixPerDataLoad(buf,hyDataLs,configReLength,configValue);
      configReLength = confDensityDataLoad(buf,hyDataLs,configReLength,configValue.transmissionDensity);
      configReLength = confThresholdKeyDataLoad(buf,hyDataLs,configReLength,configValue.moistureThresholdKey);
      configReLength = confThresholdValue1DataLoad(buf,hyDataLs,configReLength,configValue);
      configReLength = confThresholdValue2DataLoad(buf,hyDataLs,configReLength,configValue);
      configReLength = confThresholdValue3DataLoad(buf,hyDataLs,configReLength,configValue);
      configReLength = confRtcDataLoad(buf,hyDataLs,configReLength);
      configReLength = confStationDataLoad(buf,hyDataLs,configReLength,name);
      configReLength = confStationPwDataLoad(buf,hyDataLs,configReLength,pwBuf);
      configReLength = confStationServer1DataLoad(buf,hyDataLs,configReLength,serverBuf);
      configReLength = confStationServer2DataLoad(buf,hyDataLs,configReLength,serverBuf);
      configReLength = confStationServerDelDataLoad(buf,hyDataLs,configReLength);
      configReLength = confShareKeyDataLoad(buf,hyDataLs,configReLength,configValue.xyzSharkkey);
      configReLength =  confResetDataLoad(buf,hyDataLs,configReLength);
      configReLength =  confFlashResetDataLoad(buf,hyDataLs,configReLength);
  }
  else
  {
       configReLength = confErrorCodeLoad((char *)buf,hyDataLs,configReLength);
   }
                                     //��ˮ��װ��
     if(hyDataLs.confChangeFlag&RETURN_CN_FLAG)
     {
     configReLength = reSerialDataFromt(buf,configReLength,hyDataLs);
     }
      configReLength = packetCrc(buf,configReLength);
      return configReLength;

}

uint16_t requestReDataLoad(uint8_t *buf,stationSend hyDataLs,uint8_t *name)
{
   uint16_t requestLength = 0x00;

   
  requestLength = requestReDataFromt((uint8_t *)sensorBuf);
  if(hyDataLs.requestFlag!= 0)
  {

  requestLength = requestAbcDataLoad(buf,hyDataLs,requestLength,curAbc);
 // requestLength = requestSmsKeyDataLoad(buf,hyDataLs,requestLength,configValue);
 // requestLength = requestSmsPhoneDataLoad(buf,hyDataLs,requestLength,smsNum[0]);
 // requestLength = requestSharkPhoneDataLoad(buf,hyDataLs,requestLength,ctrNum[1]);
  requestLength = requestdevPerDataLoad(buf,hyDataLs,requestLength,configValue);
  requestLength = requestdevFixPerDataLoad(buf,hyDataLs,requestLength,configValue);
  requestLength = requestDensityDataLoad(buf,hyDataLs,requestLength,configValue.transmissionDensity);
  requestLength = requestThresholdKeyDataLoad(buf,hyDataLs,requestLength,configValue.moistureThresholdKey);
  requestLength = requestThresholdValue1DataLoad(buf,hyDataLs,requestLength,configValue);
  requestLength = requestThresholdValue2DataLoad(buf,hyDataLs,requestLength,configValue);
  requestLength = requestThresholdValue3DataLoad(buf,hyDataLs,requestLength,configValue);
  requestLength = requestStationDataLoad(buf,hyDataLs,requestLength,name);
  requestLength = requestPwDataLoad(buf,hyDataLs,requestLength,&devPar.stationKey[0]);
  requestLength = requestShareKKeyDataLoad(buf,hyDataLs,requestLength,configValue.xyzSharkkey);
  requestLength = requestStationServer1DataLoad(buf,hyDataLs,requestLength,stationServerAddrBuf[0]);
  requestLength = requestStationServer2DataLoad(buf,hyDataLs,requestLength,stationServerAddrBuf[1]);

  }else
                                     {
  requestLength = requestErrorCodeLoad((char *)buf,hyDataLs,requestLength);
                                     }
                                     //��ˮ��װ��
  requestLength = reSerialDataFromt(buf,requestLength,hyDataLs);

  requestLength = packetCrc(buf,requestLength);

   return requestLength;
}





void gsmIntDeal(void)
{
  gsmIntFlag ++;
    //printf("\n\r sms or call \n\r");  

}
void gsmInt_Config(void)
{

/* Private typedef -----------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0E;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
uint16_t gsmFormatAscii(uint8_t *buf,uint16_t index)
{             
      uint8_t datai = 0x00;
      uint16_t bLength = 0x00;    
      bLength += index;   
         
         // buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","RSSI");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
//                 datai =  sprintf((char *)(buf+bLength), "RSSI %d GSMERR %d",rssiValue,netGsmErrorCounter);
//                 bLength +=datai;
                
//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","GSMERR");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
//                 datai =  sprintf((char *)(buf+bLength), "%d",netGsmErrorCounter);
//                 bLength +=datai;                

//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","GSMLAC");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
                datai =  sprintf((char *)(buf+bLength), "RSSI %d GSMERR %d GSMLAC %d CELLID %d",rssiValue,netGsmErrorCounter,lacValue,cellIdValue);
                bLength +=datai;    
//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","CELLID");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
//                 datai =  sprintf((char *)(buf+bLength), "%d",cellIdValue);
//                 bLength +=datai;
      return bLength;               
}

//���ݰ��ְ���־װ��
// static uint16_t diDataFromt(uint8_t *buf,uint16_t index,uint16_t unSend,uint8_t curSend)
// {
//     uint8_t sendi = 0x00;
//     uint8_t diFlag = 0x00;
//    // ͷ������װ��
//    if((unSend - curSend)!= 0)
//    {
// 	sendi =  sprintf((char *)(buf+index), " %s","DI 1");
//     }
//     index +=sendi;
//    // 
//    return index;
//      
// }


uint16_t tcpStatusAscii(uint8_t *buf,uint8_t channel,uint16_t index,uint8_t dataType,uint16_t unSend,uint8_t curSend)
{
    uint16_t bLength = 0x00;    
    uint8_t datai = 0x00;
         bLength += index;   
         
//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","GPRSERR");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
                datai =  sprintf((char *)(buf+bLength), " GPRSERR %d",stationServerLSRLog[channel].gprsRegErrorCounter);
                bLength +=datai;
                
//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","CONERR");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
                datai =  sprintf((char *)(buf+bLength), " CONERR %d",stationServerLSRLog[channel].linkErrorCounter);
                bLength +=datai;

//           buf[bLength++]    = 0x20; 
//              datai =  sprintf((char *)(buf+bLength), "%s","SENDERR");
//                 bLength +=datai;                
//           buf[bLength++]    = 0x20; 
          datai =  sprintf((char *)(buf+bLength), " SENDERR %d",stationServerLSRLog[channel].sendErrorCounter);
          bLength +=datai;
//           buf[bLength++]    = 0x20; 
//           datai =  sprintf((char *)(buf+bLength), "%s","RECERR");
//           bLength +=datai;  
//           buf[bLength++]    = 0x20; 
          datai =  sprintf((char *)(buf+bLength), " RECERR %d",stationServerLSRLog[channel].rxDataErrorCounter);
          bLength +=datai;    
// 		  bufOut[datat++]	= 0x20;	
// 	      datai =  sprintf((char *)(bufOut+datat), "%s","BV");
// 		  datat +=datai; 	
// 		  bufOut[datat++]	= 0x20;	
          buf[bLength++]    = 0x20; 
          datai =  sprintf((char *)(buf+bLength), "BV %d ",bvTempValue);
          bLength +=datai;    
          //buf[bLength++]    = 0x20; 
          if(dataType == DEV_HISTORY_DATA)
          {
          if((unSend - curSend)!= 0)
          {
              datai =  sprintf((char *)(buf+bLength),"%s ","DI 1");

           }
           else
           {
              datai =  sprintf((char *)(buf+bLength),"%s ","DI 0");
           }
           bLength +=datai;           
          }
//           datai =  sprintf((char *)(buf+bLength), "%s","DC");
//           bLength +=datai;                
//           buf[bLength++]    = 0x20; 
          datai =  sprintf((char *)(buf+bLength), "DC %d",stationServerLSRLog[channel].dataOkCounter);
          bLength +=datai;
                
          return   bLength;             
}

void gprsStationServerSendRxTask(httpConfig *rxResult,stationSend *curStSend)
{
    DS1302_TIME sendTime;
        /*��ǰ����TCP*/
    int8_t frameType = -1;
    int8_t configFramState = -1;

    int8_t readFlashDataResult = -1;   

    uint8_t firstLinkFlag = 0x00;
    uint8_t stationName[15];
    uint8_t stationPw[10];

    uint8_t httpCode = 0x00;
    uint8_t sendDataFlag = 0x00;
    uint8_t packType = 0x00;
    uint16_t gprsSendBufLength = 0x00;
    uint16_t gprsSendDataLength = 0x00;
         
//    uint16_t packLengthAddr = 0x00;
//    uint32_t configChange = 0x00;
    uint16_t curUnsendPacket = 0x00;

    uint8_t txRxTaskState =GPRS_RUN;
    uint8_t gprsDataTypeFlag =  CONFIG_REQUEST_DATA;
    queryTime curQueryTime;

    uint8_t curSendPacket =0x00;

    uint16_t signSendCounter = 0x00;
    serverAddr tcpAddrBuf;
           
    rxResult->rtcErrorFlag = UNCHANGE_FLAG;   
/*
������������ѡ��                                    
*/
        
/*
��ʷ���ݷ���
*/                           
     if(curStSend[0].requestData ==0x00)
     {
        curStSend[0].historyDataFlag = FUNCITONG_TURN_OFF;
        curStSend[0].requestDataType = REQUEST_DATA_IDLE;
     }
     if(curStSend[1].requestData ==0x00)
     {
         curStSend[1].historyDataFlag = FUNCITONG_TURN_OFF;
         curStSend[1].requestDataType = REQUEST_DATA_IDLE;
     }

         
     gprsDataTypeFlag = SENSOR_DATA;
     cureTcpLinkChannel = 0x00;
     gprsState =GPRS_SINGEL_CLOSE;
          /*
          ����ָ��֡�޸ĺ�ˢ��
          */
     if((curStSend[1].confChangeFlag > 0))
     { 

       if(curStSend[1].confChangeFlag&HOST1_SET_OK)
       {
         rxResult->stationServerChangeFlag = 0x01;
       }
       if(curStSend[1].confChangeFlag&HOST2_SET_OK)
       {
          rxResult->stationServerChangeFlag = 0x02;
       }           
      }

          
      if((curStSend[0].confChangeFlag > 0))
      {
         if(curStSend[0].confChangeFlag&HOST1_SET_OK)
         {
             rxResult->stationServerChangeFlag= 0x01;
         }
        if(curStSend[0].confChangeFlag&HOST2_SET_OK)
        {
             rxResult->stationServerChangeFlag = 0x02;
        }
            gprsDataTypeFlag = CONFIG_RE_DATA;
      }
          


          
      if(rxResult->stationServerChangeFlag > 0x00)
      {
        // stationServerCopy(stationServerAddrBuf[rxResult->stationServerChangeFlag-1],&tcpAddrBuf);
          tcpAddrBuf = stationServerAddrBuf[rxResult->stationServerChangeFlag-1];
         rxResult->stationServerChangeFlag = 0x00;
      }

  
            
      readDs1302Time(&sendTime);

          // �����ң��վ��ַ
      stationNamCopy(&devPar.stationAddr[0],stationName);
      stationNamCopy(&devPar.stationKey[0],stationPw); 

      if(rxResult->callMode>0x00)
      {
          gprsDataTypeFlag = CONFIG_REQUEST_DATA;
          rxResult->callMode = 0x00;
       }


         
      while(GPRS_RUN==txRxTaskState ) 
      {
         switch(gprsState) 
         {
            case GPRS_SEND_DATA_READY :
            {
                 switch(gprsDataTypeFlag)
                 {
                     case SENSOR_DATA :
                     {
                        if(curStSend[cureTcpLinkChannel].unSendPacket> 0x00)   
                        {       
//�Ĵ�Э�����ݰ�����

                          readFlashDataResult =  frMultSensorDataLoad((uint8_t *)sensorBuf,curStSend[cureTcpLinkChannel].curSendAddress,&gprsSendBufLength,curStSend[cureTcpLinkChannel].unSendPacket,&curSendPacket ,&packType);                                    
                          if(readFlashDataResult == 1)
                          {   

                             curUnsendPacket = curStSend[cureTcpLinkChannel].unSendPacket;
                             gprsSendBufLength = gsmFormatAscii(sensorBuf,gprsSendBufLength);                                                             
                             gprsState = GSM_SEND_DATA_HEARD;
                          }
                          else
                            {
                                if(curStSend[cureTcpLinkChannel].unSendPacket > 0x00)
                                {
                                    curStSend[cureTcpLinkChannel].curSendAddress += SENSOR_DATA_STORE_SIZE;
                                    curStSend[cureTcpLinkChannel].unSendPacket --;
                                }
                            }
                         }
                         else
                         {
                             //������������ȴ��˳�
                           gprsDataTypeFlag =    CONFIG_REQUEST_DATA;                                  
                             //�˳�
                           if(curStSend[cureTcpLinkChannel].requestDataType == REQUEST_DATA_BY_COUNTER)
                           {
                               gprsState =   FIND_HISTORY_DATA;        
                               curStSend[cureTcpLinkChannel].requestDataReadAddress = curStSend[cureTcpLinkChannel].curSendAddress;
                           }

                           if(curStSend[cureTcpLinkChannel].historyDataFlag == FUNCITONG_TURN_ON)
                           {
                               gprsState =   GPRS_SEND_DATA_READY;
                               gprsDataTypeFlag =    DEV_HISTORY_DATA;                       
                           }
                           if(curStSend[cureTcpLinkChannel].requestDataType == REQUEST_DATA_IDLE)
                           {
                                curStSend[cureTcpLinkChannel].historyDataFlag = FUNCITONG_TURN_OFF;
//chuli zhuan                               //���������
                                gprsDataTypeFlag =	CONFIG_REQUEST_DATA;    
                                            //cureTcpLinkChannel ++;                    
                           }
                                               
                          }
                                           
                            if(cureTcpLinkChannel >=STATION_MULTI_CONNECTION_SIZE)
                            {
                                                //�˳�
                               firstLinkFlag ++;
                               gprsState = GPRS_SINGEL_CLOSE;
                            }
                                   
                            break;
                      }
//                     case DEV_REG_DATA :
//                     {
//                         gprsState =  GPRS_SEND;  
//                         break;
//                     }
          case CONFIG_REQUEST_DATA :
          {
              gprsSendDataLength = frLiaisonPackage((uint8_t *)sensorBuf);
              packHeadLength = 0x00;
              gprsState =   GPRS_SEND;  
              break;                                        
          }
          case CONFIG_RE_DATA :
          {

             gprsSendDataLength = configReDataLoad((uint8_t *)sensorBuf,curStSend[cureTcpLinkChannel],stationName,stationPw,tcpAddrBuf);
             packHeadLength = 0x00;
             gprsState =   GPRS_SEND;                                          
             break;
          }
          case REQUEST_RE_DATA :
          {
              gprsSendDataLength =  requestReDataLoad((uint8_t *)sensorBuf,curStSend[cureTcpLinkChannel],stationName);
              gprsState =   GPRS_SEND;                                          
              break;
          }                            
          case DEV_HISTORY_DATA :
          {
             if(curStSend[cureTcpLinkChannel].requestData >0)
             {
                readFlashDataResult =  frMultSensorDataLoad((uint8_t *)sensorBuf,curStSend[cureTcpLinkChannel].requestDataReadAddress,&gprsSendBufLength,curStSend[cureTcpLinkChannel].requestData,&curSendPacket ,&packType);             
                if(readFlashDataResult == 1)
                {
                                        
                  curUnsendPacket = curStSend[cureTcpLinkChannel].requestData;
                  gprsSendBufLength = gsmFormatAscii(sensorBuf,gprsSendBufLength);                                                          
                                             
                  gprsState = GSM_SEND_DATA_HEARD;
                                                     
                 }
                 else
                 {
                     if(curStSend[cureTcpLinkChannel].requestData > 0x00)
                     {
                          curStSend[cureTcpLinkChannel].requestDataReadAddress += SENSOR_DATA_STORE_SIZE;
                          curStSend[cureTcpLinkChannel].requestData --;
                     }
                 }

              }
              else
              {
                                        
                  curStSend[cureTcpLinkChannel].historyDataFlag = FUNCITONG_TURN_OFF;
                  curStSend[cureTcpLinkChannel].requestDataType = REQUEST_DATA_IDLE;
                   //ת��������������֡
                  gprsDataTypeFlag =	CONFIG_REQUEST_DATA;                                 
              }

               break;
             }
             default :
             {
                   //ת��������������֡
                  gprsDataTypeFlag =	CONFIG_REQUEST_DATA;  
                  break;                 
             }
             
           }                                                                      
            break;
           }  
      case GPRS_SET :
      {
          printf("\n\r enter server %d set task\n\r",cureTcpLinkChannel);
          if(gprsSet(&stationServerLSRLog[cureTcpLinkChannel])==COMM_OK)
          {
             gprsState =GPRS_LINK; ;          
          }
          else
          {                                                   
            stationServerLSRLog[cureTcpLinkChannel].setTry++;
            if(stationServerLSRLog[cureTcpLinkChannel].setTry >=0x03)
            {
//               rxResult->contuineSendErrorCounter ++;   
               stationServerLSRLog[cureTcpLinkChannel].setTry = 0x00;
               stationServerLSRLog[cureTcpLinkChannel].allTry ++;
         
             }      
             gprsState = GPRS_LINK;                              
          }                                                                      
             break;
        }
                                
        case GPRS_LINK  :
        {
           if(stationServerAddrBuf[cureTcpLinkChannel].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
           {
              if(stationServerLSRLog[cureTcpLinkChannel].allTry < 0x04)
              {
                 printf("\n\r enter server %d link task\n\r",cureTcpLinkChannel);  
                 if(gprsLink((uint8_t *)stationServerAddrBuf[cureTcpLinkChannel].sendAddressBuf,LINK_NUM_INSENTEK)==COMM_OK)
                 {
                    stationServerLSRLog[cureTcpLinkChannel].linkTry = 0x00;
                    gprsState =   GPRS_SEND_DATA_READY;                           
                 }
                 else
                 {                                                       
                    stationServerLSRLog[cureTcpLinkChannel].linkTry ++;
                    gprsState = GPRS_SINGEL_CLOSE;
                    stationServerLSRLog[cureTcpLinkChannel].linkErrorCounter ++; 
                    printf("\n\r link server error counter %d\n\r",stationServerLSRLog[cureTcpLinkChannel].linkErrorCounter);
                    if(stationServerLSRLog[cureTcpLinkChannel].linkTry > 0x02)
                    {
//                        rxResult->contuineSendErrorCounter ++;    
                        stationServerLSRLog[cureTcpLinkChannel].allTry++;
                        //stationServerLSRLog[cureTcpLinkChannel].linkTry= 0x00;
                    }                  
                  }
               }
               else
               {
                  cureTcpLinkChannel ++;
             
               }
           }
           else
           {
                  cureTcpLinkChannel ++;
           }
                                  
           if(cureTcpLinkChannel >=STATION_MULTI_CONNECTION_SIZE)
           {
                                   //�˳�
                  firstLinkFlag ++;
                  gprsState = GPRS_SINGEL_CLOSE;             
           }
                  break;
         }

        case GSM_SEND_DATA_HEARD :
        {                      
           gprsSendDataLength =tcpStatusAscii(sensorBuf,(cureTcpLinkChannel),gprsSendBufLength,gprsDataTypeFlag,curUnsendPacket,curSendPacket); 
                              //
          // gprsSendDataLength = diDataFromt(sensorBuf,gprsSendDataLength,curUnsendPacket,curSendPacket); 
           if(gprsDataTypeFlag == DEV_HISTORY_DATA)
           { 
               gprsSendDataLength = reSerialDataFromt((uint8_t *)sensorBuf,gprsSendDataLength,curStSend[cureTcpLinkChannel]);
           }
           if(packType&TEST_MODE_FLAG)
           {
              gprsSendDataLength = testModePacketEnd((uint8_t *)sensorBuf,gprsSendDataLength);
           }
           else
           {
              gprsSendDataLength = packetCrc((uint8_t *)sensorBuf,gprsSendDataLength);
           }
             // usart2StoreBuffer(sensorBuf,gprsSendDataLength);
              packHeadLength = 0x00;
              gprsState = GPRS_SEND;                                                                 
              break;
          }
         case  GPRS_SEND :
         { 
                           
              GPRS_RX_ptr_Out = 0x00;
              GPRS_RX_ptr_Store = 0x00;
              if(gprsSend((uint8_t *)sensorBuf,gprsSendDataLength,OTHER_DATA,LINK_NUM_INSENTEK)==COMM_OK)  //���ʹ���������
              {
                  stationServerLSRLog[cureTcpLinkChannel].sendTry = 0x00;
                  gprsState = GPRS_RX;
                  signSendCounter ++;
              }
              else
              {
                         
                  stationServerLSRLog[cureTcpLinkChannel].sendTry++;
                  gprsState =GPRS_SINGEL_CLOSE;
                  stationServerLSRLog[cureTcpLinkChannel].sendErrorCounter ++;
                  printf("\n\r send data error counter %d \n\r",stationServerLSRLog[cureTcpLinkChannel].sendErrorCounter);

                 if(stationServerLSRLog[cureTcpLinkChannel].sendTry >=0x03)
                 {
//                   rxResult->contuineSendErrorCounter ++;     
                  // stationServerLSRLog[cureTcpLinkChannel].sendTry  =0x00;
                   stationServerLSRLog[cureTcpLinkChannel].allTry++;    
                 }
              }

              break;
          }
          case GPRS_RX   :
          {
            if(rxGprs(&httpCode)==COMM_OK)
            {                              
              if(httpCode ==RX_FRAME_OK)
              {
                 stationServerLSRLog[cureTcpLinkChannel].rxTry= 0x00;
                            
                 switch(gprsDataTypeFlag)
                 {
                     case SENSOR_DATA :
                     {
                        gprsState = GPRS_SENSOR_DATA_RE ;
                        break;
                     }
                     case CONFIG_REQUEST_DATA :
                     {
                       gprsState = GPRS_CONFIG_DATA_RE ;
                       break;
                     }   
                     case CONFIG_RE_DATA :
                     {
                        gprsState = GPRS_CONFIG_RE_DATA;
                        break;
                     }
                     case REQUEST_RE_DATA :
                     {
                        gprsState = GPRS_REQUEST__RE_DATA ;
                        break;
                     }
                     case DEV_HISTORY_DATA :
                     {
                        gprsState = GPRS_HISTORY_DATA ;
                        break;
                     }   
                                     
                   }
                                  
               stationServerLSRLog[cureTcpLinkChannel].dataOkCounter ++;
                           
#if GPRS_DEBUG > 0
               printf("\n\r receive server data ok\n\r");
#endif  
           }
           else
           {
               if(httpCode ==RX_FRAME_ERROR)
               {
                   printf("\n\r rx frame error \n\r");

               }
                   gprsState = GPRS_SEND_DATA_READY;
           }
               printf("\n\r data type %d\n\r",gprsDataTypeFlag);
               if(cureTcpLinkChannel >=STATION_MULTI_CONNECTION_SIZE)
               {
                                                      //�˳�
                    firstLinkFlag ++;
                    gprsState = GPRS_SINGEL_CLOSE;           
               }
               break;
        }
        else
        {
#if GPRS_DEBUG > 0
            printf("\n\r receive server data error\n\r");
            printf("\n\r server %d data error\n\r",cureTcpLinkChannel);
#endif
//            rxResult->contuineSendErrorCounter ++;
            //������ֵ������
            stationServerLSRLog[cureTcpLinkChannel].rxErrorCounter ++;
            printf("\n\r rx server data error counter %d\n\r",stationServerLSRLog[cureTcpLinkChannel].rxErrorCounter);
            stationServerLSRLog[cureTcpLinkChannel].rxTry ++;
            gprsState = GPRS_SINGEL_CLOSE;

            if(stationServerLSRLog[cureTcpLinkChannel].rxTry >0x01)
            {
                stationServerLSRLog[cureTcpLinkChannel].allTry++;

               if(stationServerLSRLog[cureTcpLinkChannel].rxTry > 0x03)
               {
                      cureTcpLinkChannel ++;
               }
            }                    
         }
        break;
       }
       case GPRS_REQUEST__RE_DATA :
       {
            gprsState = GPRS_SEND_DATA_READY; 
            curStSend[cureTcpLinkChannel].requestFlag = 0x00;
            curStSend[cureTcpLinkChannel].requestErrorFlag = 0x00;
            gprsDataTypeFlag = CONFIG_REQUEST_DATA;                               
            break;
       }   
       case GPRS_CONFIG_RE_DATA :
       {
#if GPRS_DEBUG > 0
           printf("\n\r config re data\n\r");
                                  
#endif                            
           gprsState = GPRS_SEND_DATA_READY; 
           gprsDataTypeFlag = CONFIG_REQUEST_DATA;
           curStSend[cureTcpLinkChannel].confChangErrorFlag =  0;
           curStSend[cureTcpLinkChannel].confChangeFlag = 0;
           if(curStationSend[cureTcpLinkChannel].requestData > 0x00) 
           {
               gprsDataTypeFlag = DEV_HISTORY_DATA;
           }
           if(curStationSend[cureTcpLinkChannel].unSendPacket > 0x00) 
           {
               gprsDataTypeFlag = SENSOR_DATA;
           }
                            
           break;
        }

//         case GPRS_REG_DATA_RE :
//         {
//            break;
//         }
        case GPRS_CONFIG_DATA_RE :
        {
           frameType = dataFrameJudeg();
           switch(frameType)
           {
              case  OK_FRAME_FLAG :
              case LIAISON_FRAME_FLAG :
              {
                 cureTcpLinkChannel ++;
                 gprsDataTypeFlag = SENSOR_DATA;
                 if(curStSend[cureTcpLinkChannel].confChangeFlag!=0)
                 {
                    gprsDataTypeFlag = CONFIG_RE_DATA;;
                 }
                 gprsState = GPRS_SINGEL_CLOSE;                                     
                                  
                 break;
              }
              case CONFIG_FRAME_FLAG :
              {
                 gprsState =  GPRS_CONFIG_DATA_FIND;   
                 break;
               }
              case INSTRUCT_FRAME_FLAG :
              {
                 gprsState = GPRS_INSTRUCT_DATA_FIND;   
                 break;
              } 
              case ERROR_FRAME_FLAG :
              {
                   /*
                   �յ���������֡
                   */
                if(!getFrameSerialNum(&curStSend[cureTcpLinkChannel]))
                {
 #if GPRS_DEBUG > 0
                  printf("\n\r station down serial num  %d \n\r",curStSend[cureTcpLinkChannel].downSerialNum);
 #endif 
                }                                    
                curStSend[cureTcpLinkChannel].confChangErrorFlag |= FRAME_ERROR;    
                gprsDataTypeFlag = CONFIG_RE_DATA;
                gprsState = GPRS_SEND_DATA_READY;                                     
                                  
                break;
              }
              default :
              {
               firstLinkFlag ++;
               gprsState = GPRS_SINGEL_CLOSE;
              }
             }

            if(cureTcpLinkChannel >=STATION_MULTI_CONNECTION_SIZE)
            {
                                                //�˳�
               firstLinkFlag ++;
               gprsState = GPRS_SINGEL_CLOSE;
                              
            }                           
               break;
            }
                       
           case GPRS_SENSOR_DATA_RE :
           {
               if(!dataOkJudeg())
               {

                   // gprsSendSucFlagWrite(curGprsSendAddr,curSendPacket,sendTime);
                 if(curStSend[cureTcpLinkChannel].unSendPacket >= curSendPacket)
                 {                     
                 curStSend[cureTcpLinkChannel].unSendPacket -= curSendPacket;
                     
                 curStSend[cureTcpLinkChannel].curSendAddress  += (SENSOR_DATA_STORE_SIZE*curSendPacket);
                 }
                 else
                 {
                         curStSend[cureTcpLinkChannel].unSendPacket    = 0x00;
                         curStSend[cureTcpLinkChannel].curSendAddress = sensorDataWriteAddr;
                  }   
                 if(curStSend[cureTcpLinkChannel].unSendPacket>1000)
                 {
                          curStSend[cureTcpLinkChannel].unSendPacket    = 0x00;
                         curStSend[cureTcpLinkChannel].curSendAddress = sensorDataWriteAddr;                 
                 }                     
                 printf("\n\r send  data%d  %d   ,%d \n\r",cureTcpLinkChannel,curSendPacket,curStSend[cureTcpLinkChannel].unSendPacket);          
                 sendDataFlag = 0;     
                 curSendPacket = 0;
                 gprsState = GPRS_SEND_DATA_READY;  

                 if(curStSend[cureTcpLinkChannel].unSendPacket==0)
                 { 
        /*
�豸����                   
        */                     
                     gprsDataTypeFlag = CONFIG_REQUEST_DATA;
//                     rxResult->sendErrorCounter = SEND_OK;    
                     if(curStSend[cureTcpLinkChannel].requestDataType == REQUEST_DATA_BY_COUNTER)
                     {
                        gprsState =   FIND_HISTORY_DATA;        
                        curStSend[cureTcpLinkChannel].requestDataReadAddress = curStSend[cureTcpLinkChannel].curSendAddress;
                     }

                     if (curStSend[cureTcpLinkChannel].historyDataFlag == FUNCITONG_TURN_ON)
                     {
                         gprsState =   GPRS_SEND_DATA_READY;
                         gprsDataTypeFlag =    DEV_HISTORY_DATA;                       
                     }
                     if(curStSend[cureTcpLinkChannel].requestDataType == REQUEST_DATA_IDLE)
                     {
                          curStSend[cureTcpLinkChannel].historyDataFlag = FUNCITONG_TURN_OFF;
                                                      
                     }
                    }
              }
              else
              {
                   gprsState = GPRS_SEND_DATA_READY; 
                   sendDataFlag++;
                   if(sendDataFlag > 0x01)
                   {
                       curStSend[cureTcpLinkChannel].unSendPacket --;
                       curStSend[cureTcpLinkChannel].curSendAddress  += SENSOR_DATA_STORE_SIZE;            

                    }
               }
             break;
            }
          case GPRS_HISTORY_DATA :
          {   
             if(curStSend[cureTcpLinkChannel].requestData > curSendPacket)
             {                 
              curStSend[cureTcpLinkChannel].requestDataReadAddress += SENSOR_DATA_STORE_SIZE*curSendPacket;
              curStSend[cureTcpLinkChannel].requestData -= curSendPacket;
             }else
             {
                curStSend[cureTcpLinkChannel].requestData = 0x00;
              }
              printf("\n\r send history  data  %d   ,%d \n\r",curSendPacket,curStSend[cureTcpLinkChannel].requestData);        
              curSendPacket = 0x00;                    
//              rxResult->sendErrorCounter = SEND_OK;  
              gprsState = GPRS_SEND_DATA_READY;
              if(curStSend[cureTcpLinkChannel].requestDataReadAddress == sensorDataWriteAddr)
              {
                   curStSend[cureTcpLinkChannel].requestData = 0x00;
              }
              if(0x00 >=curStSend[cureTcpLinkChannel].requestData)
              {
                   curStSend[cureTcpLinkChannel].historyDataFlag = FUNCITONG_TURN_OFF;
                   curStSend[cureTcpLinkChannel].requestDataType = REQUEST_DATA_IDLE;                                                                                                    //�˳�
                   gprsDataTypeFlag =  CONFIG_REQUEST_DATA;  
              }

               break;
           }
           case GPRS_SINGEL_CLOSE :
           {
              if(closeSingleTcp(LINK_NUM_INSENTEK)==COMM_OK)
              {    
                gprsState = GPRS_LINK;                                    
              }
              else
              {
                 gprsState = GPRS_CLOSE;
               }
             if(firstLinkFlag > 0x00)
             {
               gprsState = GPRS_SLEEP_MODE;  
             }   
               break;
            }
           case GPRS_CLOSE :
           {
               (void)closeTcp();
               gprsState = GPRS_SET;
               if(firstLinkFlag > 0x00)
               {
                   gprsState = GPRS_SLEEP_MODE;  
               }                          
               break;
           } 
          case FIND_HISTORY_DATA :
          {
              printf("\n\rEnter find flash history  task   \n\r");
                   
              gprsState =  GPRS_SEND_DATA_READY;
              if(!senseorDataFindByTime(curQueryTime,&curStSend[cureTcpLinkChannel],(uint8_t *)sensorBuf))
              {
                 printf("\n\r find flash history  data  %d \n\r",curStSend[cureTcpLinkChannel].requestData); 
                 curStSend[cureTcpLinkChannel].historyDataFlag = FUNCITONG_TURN_ON;
                 gprsDataTypeFlag =    DEV_HISTORY_DATA;              
              }
              else
              {
                        //�˳�
                       //txRxTaskState = GPRS_SLEEP;
                      //δ�ҵ����ݣ���δ���
                    gprsDataTypeFlag =  REQUEST_RE_DATA;   
                    curStSend[cureTcpLinkChannel].requestErrorFlag |= REQUEST_ERROR;
                    curStSend[cureTcpLinkChannel].requestDataType = REQUEST_DATA_IDLE;         
               }
                   printf("\n\rfind flash history  task  End \n\r");
                  break;
               }
        case GPRS_INSTRUCT_DATA_FIND :
        {
            printf("\n\r instruct frame rx   \n\r");
            if(!getFrameSerialNum(&curStSend[cureTcpLinkChannel]))
            {
#if GPRS_DEBUG > 0
               printf("\n\r station down serial num  %d \n\r",curStSend[cureTcpLinkChannel].downSerialNum);
#endif 
            }

            if(!instructionRequestJudge(&curStSend[cureTcpLinkChannel],&curQueryTime,sendTime))
            {
#if GPRS_DEBUG > 0                        
               printf("\n\r find instruction frame \n\r"); 
#endif                    
            }
            else
            {
                curStSend[cureTcpLinkChannel].requestErrorFlag |= CONFIG_ID_ERROR;

            }
               gprsState =    GPRS_SENSOR_DATA_RE; 
               gprsDataTypeFlag =	REQUEST_RE_DATA;  
                  //��ʷ���ݲ�ѯ����
               if(curStSend[cureTcpLinkChannel].requestFlag&HISTORY_DATA_REQUEST)
               {
                  gprsState =    FIND_HISTORY_DATA;
                  curStSend[cureTcpLinkChannel].requestFlag = 0x00;
                  curStSend[cureTcpLinkChannel].requestDataType = REQUEST_DATA_BY_COUNTER;
               }
                break;
          }
         case  GPRS_CONFIG_DATA_FIND :
         {
//��������
            printf("\n\r config frame rx \n\r");
            configFramState = configFrameOkJudeg(&curStSend[cureTcpLinkChannel]);
            printf("\n\r config frame rx %d\n\r",configFramState);  
            if(!getFrameSerialNum(&curStSend[cureTcpLinkChannel]))
            {
 #if GPRS_DEBUG > 0
              printf("\n\r station down serial num  %d \n\r",curStSend[cureTcpLinkChannel].downSerialNum);
 #endif 
            }
            if(configFramState ==0x00)
            //if(1)
            {
               curStSend[cureTcpLinkChannel].configCounter = 0x00;
                    //Ѱ�����ò���
              if(!findStationWaterabcValue(curAbc,&curStSend[cureTcpLinkChannel]))
              {
                  rxResult->storeFlagAbc ++;
                  curStSend[cureTcpLinkChannel].configCounter ++;
                      //devAbcStoreFlash(curAbc);
#if GPRS_DEBUG > 0
                   printf("\n\r�������Ͳ�������\n\r");
#endif	
              }
              if(!findStationConfigSmsKey(&configValue,&curStSend[cureTcpLinkChannel]))
              {
                   rxResult->storeFlagConfig ++;
                   curStSend[cureTcpLinkChannel].configCounter ++;
                    
#if GPRS_DEBUG > 0
                    printf("\n\r station sms data key state %d \n\r",configValue.devWorkState);
#endif      
              }
//              if(!findStationConfigSmsPhone(&smsNum[0],&curStSend[cureTcpLinkChannel]))
//              {
//                   rxResult->storeFlagDataSmsPhone ++;
//                   curStSend[cureTcpLinkChannel].configCounter ++;
//                      
//#if GPRS_DEBUG > 0
//                    printf("\n\r station sms phone find \n\r");
//#endif
//              }
             if(!stationRequestDevGpsKey(&configValue.gpsLocation,&curStSend[cureTcpLinkChannel]))
             {
#if GPRS_DEBUG > 0
                    printf("\n\r gps location request%d \n\r",configValue.gpsLocation);
#endif
             }
             if(!findStationConfigDevPeriod(&configValue,&curStSend[cureTcpLinkChannel]))
             {
                   rxResult->storeFlagConfig ++;
                   curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                  printf("\n\r station device period %d\n\r",configValue.devPeriod);
#endif		
              }
              
             if(!findStationFixTimePeriod(&configValue,&curStSend[cureTcpLinkChannel]))
             {
                   rxResult->storeFlagConfig ++;
                   curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                   printf("\n\r fix time hour:%d,min:%d \n\r",configValue.devPeriodHour,configValue.devPeriodMin);
#endif
              }
              
              if(!findStationConfigDevTransmissDensity(&configValue.transmissionDensity,&curStSend[cureTcpLinkChannel]))
		      {
                  rxResult->storeFlagConfig ++;
                  curStSend[cureTcpLinkChannel].configCounter ++;
                  rxResult->devDensity = configValue.transmissionDensity; 
#if GPRS_DEBUG > 0
                  printf("\n\r station density %d\n\r",configValue.transmissionDensity);
#endif		
			  } 
                   
             if(!findStationThresholdKey(&configValue.moistureThresholdKey,&curStSend[cureTcpLinkChannel]))
		     {
                  rxResult->storeFlagConfig ++;
                  curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                  printf("\n\r threshold key state %d \n\r",configValue.moistureThresholdKey);
#endif
              }
              if(!findStationThresholdValue(THRESHOLD_VALUE1,&configValue,&curStSend[cureTcpLinkChannel]))
              {
                  rxResult->storeFlagConfig ++;
                  curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                  printf("\n\r threshold  value1:%d,period1:%d \n\r",configValue.moistureThreshold1,configValue.threshold1Period);
#endif

             }
             if(!findStationThresholdValue(THRESHOLD_VALUE2,&configValue,&curStSend[cureTcpLinkChannel]))
             {
                   rxResult->storeFlagConfig ++;
                   curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                   printf("\n\r threshold  value2:%d,period2:%d \n\r",configValue.moistureThreshold2,configValue.threshold2Period);
#endif

              }
             if(!findStationThresholdValue(THRESHOLD_VALUE3,&configValue,&curStSend[cureTcpLinkChannel]))
             {
                   rxResult->storeFlagConfig ++;
                   curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                  printf("\n\r threshold  value3:%d,period3:%d \n\r",configValue.moistureThreshold3,configValue.threshold3Period);
#endif

              } 
              if(!findStationRtcTime(&curStSend[cureTcpLinkChannel]))
              {
                  curStSend[cureTcpLinkChannel].configCounter ++;
                      //����ʱ�ӱ��ı��־λ
                  rxResult->rtcSetFlag = CHANGE_FLAG;
                      
#if GPRS_DEBUG > 0
                  printf("\n\r station rtc reset ok \n\r");
#endif
              }
              if(!delHostNServer(HOST2,&curStSend[cureTcpLinkChannel]))
              {
                  rxResult->storeFlagStationServer ++;
                  rxResult->delFlagHost2 ++;

                  curStSend[cureTcpLinkChannel].configCounter ++;

#if GPRS_DEBUG > 0
                  printf("\n\r host2 del  ok \n\r");
#endif
               }
               if(!finsStationName(stationName,&curStSend[cureTcpLinkChannel]))
               {
                   rxResult->storeFlagParam ++;
                   rxResult->storeFlagStationName ++;
                           
                   curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                   printf("\n\r station name set  ok \n\r");
#endif
               }
               if(!findHostServer(&tcpAddrBuf,HOST2,&curStSend[cureTcpLinkChannel]))
               {
                   curStationSend[1].unSendPacket = 0x00;
                   curStationSend[1].curSendAddress = curGprsSendAddr;
                   rxResult->storeFlagStationServer ++;
                   rxResult->stationServerChangeFlag = HOST2;
                   curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                   printf("\n\r host2 set  ok \n\r");
#endif

               }
               if(!findHostServer(&tcpAddrBuf,HOST1,&curStSend[cureTcpLinkChannel]))
               {
                   rxResult->storeFlagStationServer ++;
                   rxResult->stationServerChangeFlag = HOST1;
                   curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                   printf("\n\r host1 set  ok \n\r");
#endif

               }
              if(!findStationConfigSharkKey(&configValue,&curStSend[cureTcpLinkChannel]))
              {
                    rxResult->storeFlagConfig ++;
                    curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                    printf("\n\r shark key set  ok %d\n\r",configValue.xyzSharkkey);
#endif

              }
//              if(!findStationSharkSmsPhone(&ctrNum[1],&curStSend[cureTcpLinkChannel]))
//              {
//                    rxResult->storeFlagAlarmSmsPhone ++;
//                    curStSend[cureTcpLinkChannel].configCounter ++;
//#if GPRS_DEBUG > 0
//                   printf("\n\r shark phone set  ok %\n\r");
//#endif
//              }
              if(!finsStationPw(stationPw,&curStSend[cureTcpLinkChannel]))
              {
                     rxResult->storeFlagStationPw ++;
                     rxResult->storeFlagParam ++;
                     curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                     printf("\n\r shark phone set  ok %\n\r");
#endif
              }
              if(!finsStationReset(&curStSend[cureTcpLinkChannel]))
              {
                     rxResult->storeFlagParam ++;
                     rxResult->storeFlagConfig ++;
                     rxResult->staionResetFlag++;
                     curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                     printf("\n\r staion flsah reset   ok %\n\r");
#endif
              }
              if(!finsStationFlashReset(&curStSend[cureTcpLinkChannel]))
              {
                           //rxResult->storeFlagParam ++;
                    rxResult->staionFlashResetFlag++;
                    curStSend[cureTcpLinkChannel].configCounter ++;
#if GPRS_DEBUG > 0
                    printf("\n\r staion reset   ok %\n\r");
#endif
              }                   
              if(curStSend[cureTcpLinkChannel].confChangeFlag == 0)
	          {
                   curStSend[cureTcpLinkChannel].confChangErrorFlag |= CONFIG_ID_ERROR;

              }
              else
              {
                 if(cureTcpLinkChannel== 0x00)
                 {
                 
                    curStSend[1].confChangeFlag |= curStSend[cureTcpLinkChannel].confChangeFlag;
                 }
                 if(cureTcpLinkChannel== 0x01)
                 {
                    curStSend[0].confChangeFlag |= curStSend[cureTcpLinkChannel].confChangeFlag;                       
                  }
               }

                  gprsDataTypeFlag = CONFIG_RE_DATA;
            }
            else
            {
                    
               if(configFramState == 0x01)
               {
                  curStSend[cureTcpLinkChannel].confChangErrorFlag |= KEY_ERROR;    
                  gprsDataTypeFlag = CONFIG_RE_DATA;
                }
               else
               {
                     gprsDataTypeFlag = CONFIG_REQUEST_DATA;

                 }
              }
              curStSend[cureTcpLinkChannel].confChangeFlag |= RETURN_CN_FLAG;
              gprsState = GPRS_SEND_DATA_READY;                                 
      
                  break;
           }
          case GPRS_SLEEP_MODE :
          {						
							txRxTaskState = GPRS_SLEEP;						
            break;
          }
            default :
            {
               firstLinkFlag ++;
               gprsState = GPRS_SINGEL_CLOSE;
                break;
             }
              
         }
                    //������෢��200������
         if(signSendCounter > 300)
         {    //�˳�
              firstLinkFlag ++;
              gprsState = GPRS_SINGEL_CLOSE;
      
         }
        }
        gprsState = GSM_MOUDLE_IDLE;
//         if(rxResult->sendErrorCounter>  0x04)
//         {
//                    rxResult->sendErrorCounter = SEND_OK;
//         }
 						


         if(rxResult->delFlagHost2 >0x00)
         {
#if GPRS_DEBUG > 0
              printf("\n\r adress  2 del OK\n\r");
#endif             
              stationServerAddrBuf[1].userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;
         }
         if(rxResult->storeFlagStationPw > 0x00)
         {
              stationNamCopy(stationPw,&devPar.stationKey[0]);
         }
         if(rxResult->storeFlagStationName >0x00)
         {     
            //ң��վ��ַװ��   ң��վ��ַ�洢
             stationNamCopy(stationName,&devPar.stationAddr[0]);
         }
         if(rxResult->stationServerChangeFlag > 0x00)
         {
#if GPRS_DEBUG > 0
             printf("\n\r adress  %d\n\r",(rxResult->stationServerChangeFlag-1));
#endif          
           stationServerAddrBuf[rxResult->stationServerChangeFlag-1] = tcpAddrBuf;             
             //stationServerCopy(tcpAddrBuf,&stationServerAddrBuf[rxResult->stationServerChangeFlag-1]);
         }          
      
         
}


static void stationPwReset(void)
{
	 devPar.stationKey[1] = 3+0x30;
	 devPar.stationKey[2] = 7+0x30;
	 devPar.stationKey[3] = 1+0x30;
	 devPar.stationKey[4] = 8+0x30;
	 devPar.stationKey[5] = 0+0x30;
	 devPar.stationKey[6] = 0+0x30;
	 devPar.stationKey[0] = 7;
}
static void stationReset(void)
{
    uint8_t i = 0x00;
//ң��վ�ų�ʼ��    
    for(i = 1;i < 11;i++)
    {
      devPar.stationAddr[i] = devSnBuf[i+4];
    }
    devPar.stationAddr[0] = i; 
//�����ʼ��
         stationPwReset();
//
   configValue.xyzSharkkey =   FUNCITONG_TURN_OFF;
   configValue.devWorkState =  FUNCITONG_TURN_OFF;
   configValue.moistureThresholdKey =  FUNCITONG_TURN_OFF;
}




//uint16_t  sensorDataReadFromFlashToBuf(uint32_t readAddress,uint8_t *readBuf,uint16_t *readLength)




