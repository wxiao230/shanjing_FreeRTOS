#include "main.h"
#include "stm32l1xx.h"

#include "usart.h"
#include "mode.h"
#include "sensor.h"
#include "w_ads1x15.h"
#include "ds1302.h"
#include	"deviceset.h"

#include "power.h"
#include "spi_flash.h"
#include "deviceset.h"
#include "rtc.h"
#include "spi_flash.h"
#include <stdlib.h>
#include "gprs.h"
#include "crc16.h"
#include "qingxie.h"
#if(MQTT_USE_CTR>0)
	#include "mqttTransport.h"
	#include "cigemPacket.h"
#endif
#if((SK60_CTR>0)||(KXYL_CTR>0))
	#include "sk60.h"
#endif
#include <math.h>

//RS232����
#define RS232_TASK_DEBUG        1

//#define MAX_ERRORS              (5)

//�����ַ�
//#define END_CHARCCTER          0xFE

#define MODE_TIMEOUT             (0x100000)
/**
  * @brief  Receive byte from sender
  * @param  c: Character
  * @param  timeout: Timeout
  * @retval 0: Byte received
  *        -1: Timeout
  */
static  int32_t Receive_Byte (uint8_t *c, uint32_t timeout)
{
  while (timeout-- > 0)
  {
    if(SerialKeyPressed(c) == 1)
    {
      return 0;
    }
  }
  return -1;
}


uint16_t recCounter = 0x00;


#define PACK_HEAD_REC        0x00
#define PACK_DATA_REC        0x01
#define PACK_CRC_CHECK       0x02
#define PACK_IDLE_REC        0x03

static uint8_t recPacState = PACK_IDLE_REC;

#define PACKET_HEADE_BEGIN     0xDD
#define PACKET_HEADE_END       0xDD

#define PACKET_DATA_BEGIN_ADDRESS    0x06

#define PACK_HEAD_LENGTH       0x06


//
 void sensorDataRead(uint32_t *writeAddress,uint16_t readCounter);

//uint8_t rs232Buf[MAX_RS232_BUF_SIZE];
/**
  * @brief  Receive a packet from sender
  * @param  data
  * @param  length
  * @param  timeout
  *     0: end of transmission
  *    -1: abort by sender
  *    >0: packet length
  * @retval 0: normally return
  *        -1: timeout
  *         1: abort by user
  *         2: packet head error
  *		    3: pcaket data length error
  *			4: packet crc  error
  *
  *
  *
  *
  */
static int32_t Receive_Packet(uint8_t *data, int32_t *length, uint32_t timeout)
{
  uint16_t i, packet_size;

  uint8_t c;
  uint8_t crc= 0x00;

  uint8_t buf[PACK_HEAD_LENGTH];
  *length = 0;
  recCounter = 0x00;

//  c = GetKey();
  if (Receive_Byte(&c,(3*timeout)) != 0)
  {
    return -1;
  }
  
  if(c==0xdd)
  {
    recPacState = PACK_HEAD_REC; 
  }
  else
  {
    return 2;
  }
  
  if(recPacState == PACK_HEAD_REC)
  {
      buf[0]= PACKET_HEADE_BEGIN;
  	  for(i = 1;i < (PACK_HEAD_LENGTH);i ++)
	  {
            if(Receive_Byte(&buf[i], timeout) != 0)
            {
               return -1;
            }
				      
	  }

	  if(buf[0]==buf[5])
	  {
	     packet_size = buf[1]+buf[2]*256;
	  }
	  else
	  {
               return 2;
	  
	  }
	  if(packet_size==(buf[3]+buf[4]*256))
	  {
	     recPacState =PACK_DATA_REC;
	  }
	  else
	  {
               return 3;	    
	  }
      
  }
  if(recPacState ==PACK_DATA_REC)
  {
  	  for(i = 0;i < packet_size;i ++)
	  {
            if(Receive_Byte(&data[i], timeout) != 0)
            {
               return -1;
            }	      
            crc+= data[i];
	  }     
  	recPacState = PACK_CRC_CHECK;
  }
  if(recPacState == PACK_CRC_CHECK)
  {
     if (Receive_Byte(&c, timeout) != 0)
     {
        return -1;
     }
	 if(c== crc)
	 {
         *length = packet_size;
	     return 0;
	 }
	 else
	 {
	     return 4;
	 }      
  }
      return -2;
}

void recAckSend(uint8_t ackType,uint8_t dataOrder)
{
    uint8_t buf[20];
	uint8_t bufi = 0;
	uint8_t bufLength = 0;
	uint8_t crc = 0;
//����ͷ����Ϣװ��
	buf[0] =  PACKET_HEADE_BEGIN;

	buf[1] = 0x04;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0x00;
	buf[5] =  PACKET_HEADE_BEGIN;
	bufLength = PACKET_DATA_BEGIN_ADDRESS;
//����������װ��
	buf[bufLength++]  =  DATA_UP_FLAG;

	buf[bufLength++]  =  dataOrder;
	buf[bufLength++]  =  FIR_ACK_COMM;
 	buf[bufLength++]  =  ackType;

	for(bufi =PACKET_DATA_BEGIN_ADDRESS;bufi < 0x0A;bufi++ )
	{
	    crc += buf[bufi];
	}
	buf[bufLength++]= crc;
	//��������
	usart2StoreBuffer(buf,bufLength);
}


void linkMaster(uint8_t dataOrder,uint8_t linkType)
{
    uint8_t buf[20];
	uint8_t bufi = 0;
	uint8_t bufLength = 0;
	uint8_t crc = 0;
//����ͷ����Ϣװ��
	buf[0] =  PACKET_HEADE_BEGIN;

	buf[1] = 0x04;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0x00;
	buf[5] =  PACKET_HEADE_BEGIN;
	bufLength = PACKET_DATA_BEGIN_ADDRESS;
//����������װ��
	buf[bufLength++]  =  DATA_UP_FLAG;

	buf[bufLength++]  =  dataOrder;
	buf[bufLength++]  =  FIR_LINK_COMM;
 	buf[bufLength++]  =  linkType;

	for(bufi =PACKET_DATA_BEGIN_ADDRESS;bufi < 0x0A;bufi++ )
	{
	    crc += buf[bufi];
	}
	buf[bufLength++]= crc;
	//��������
	usart2StoreBuffer(buf,bufLength);   

}

int16_t judgeAck(uint8_t *buf)
{
	 int16_t status =0;
	 if(buf[FIR_COMMAND_ADDRESS]== FIR_ACK_COMM)
	 {
	       switch(buf[SEC_COMMAND_ADDRESS])
		   {
		      case  SEC_ACK_OK_COMM	 :
			  {
			  	status = 0;
				break;
			  }
		      case  SEC_HEAD_ERROR_COMM	 :
			  {
			  	status = SEC_HEAD_ERROR_COMM;			  
				break;
			  }
		      case  SEC_LENGTH_COMM	 :
			  {
			  	status = SEC_LENGTH_COMM;			  
			  
				break;
			  }
		      case  SEC_TIMEOUT_COMM	 :
			  {
			  	status = SEC_TIMEOUT_COMM;			  
			  
				break;
			  }	
		      case  SEC_CRC_ERROR_COMM	 :
			  {
			  	status = SEC_CRC_ERROR_COMM;			  
			  
				break;
			  }
		      default :
			  {
			  	status = -1;			  
			  
				break;
			  }			  			  		  			  		   
		   }
	 }
	 else
	 {
			  	status = -2;			  	 
	 }
	 return status;

}

static void delay_modeMs(uint32_t delay)
{
    uint16_t i;
	while(delay--)
	{
	for(i=0;i<8300;i++);
	}
} 
/**
  * @brief  Receive a file using the ymodem protocol.
  * @param  buf: Address of the first byte.
  * @retval The size of the file.
  */
#define CALL_MASTER_STATE 0x03
#define REC_STATE      0x00
#define DEAL_STATE     0x01
#define EXIT_STATE     0x02


uint8_t recTaskState = REC_STATE;

#define MAX_TIMEOUT_COUNTER     0x03
#define MAX_ERROR_COUNTER       0x03
#define MAX_LINK_TRY_NUM        0x03

#define LINK_STATUS            0x01
#define UNLINK_STATUS          0x02

#define TASK_RUN_FLAG          0x01
#define TASK_EXIT_FLAG         0x00

#define MAX_RS232_TASK_COUNTER         (100)


extern void gprsSendAddrStore(uint8_t setFlag);
extern uint16_t unSendPackNumAdj(uint32_t sendAddr);
extern void mqttAddressSet(void);
#if(MQTT_USE_CTR>0)
	static void clearMqttAddrStroe(void);//20210225
#endif
void devSendPeriodSet(uint8_t* buf)
{
	value_t temp; 
#if(MQTT_USE_CTR>0)
//	uint8_t tmpi=0;
#endif
	//�������ڸ�ֵ
	//���ݲɼ�����
	temp.bytes.value1 = buf[0]; 
	temp.bytes.value2 = buf[1]; 

	//��������ת��Ϊ��
	//��������ת��Ϊ��
	configValue.devPeriod 		= temp.j*60; 
	configValue.devPeriodType = NORMAL_PERIOD_TYPE;
	configValue.devPeriodMin 	= 0x00;
	configValue.devPeriodHour = 0x01;
	
	configValue.configVersion = 0x00;
	configValue.curfireWareId = 0x31;
	configValue.devWorkState = FUNCITONG_TURN_OFF;
	configValue.transmissionDensity = 0x01;
	configValue.xyzSharkkey = XYZ_SHARK_OFF;	
	configValue.xValue = 10;
	configValue.yValue = 256;
	configValue.zValue = 10;		 
	
	configValue.gpsLocation = 0x00;
	configValue.gpsKey 			= FUNCITONG_TURN_ON;
	devPar.gpsInterVal			= GPS_INTERVAL_INIT;	
	devPar.usartKey					= FUNCITONG_INIT_USART;	
	devPar.insentekKey			=	INSENTEK_KEY_INIT;
	devPar.mqttDataType			=	MQTT_DATA_TYPE_INIT;
	devPar.gsmRunMode					=	FUNCITONG_TURN_OFF;//20210225
	
	configValue.moistureThresholdKey = FUNCITONG_TURN_OFF;
	configValue.threshold1Period = 600;
	configValue.threshold2Period = 600;
	configValue.threshold3Period = 600;			 
	configValue.moistureThreshold1 = 10;
	configValue.moistureThreshold2 = 50;
	configValue.moistureThreshold3 = 5;
	devPar.timeZone = 8;

	//sms���ű���ͨ��
	devPar.centerSmsFlag = 0x00;
	//        devPar.center1Sms[0] = 0x00;
	devPar.center2Sms[0] = 0x00;
	devPar.configChangeCounter = 0x00;
	devPar.pwChangeCounter = 0x00;        
	devPar.configChangeCounter = 0x00;
	devPar.pwChangeCounter = 0x00; 

	devPar.bvLowCounter = 0x00;        
	devPar.flashResetCounter = 0x00;
	
	devPar.workState = 0x02; 	
	stationServerAddrBuf[1].userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;
	stationServerAddrBuf[0].userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;            

	devPar.plus_intv	= 0;
#if(MQTT_USE_CTR>0)
//	devPar.calCounter	=	calAllCounter.i;
//	for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)
//	{
//		mqttServerAddrBuf[tmpi].userFlag = TCP_SERVER_ADDRESS_UNUSE_FLAG;
//		curMqttSendAddr[tmpi].i = 0;	
//	}

	devPar.stationMode	=	FUNCITONG_TURN_ON;
//	mqttAddressSet();	
	clearMqttAddrStroe();//20210225
#else
	devPar.stationMode	=	FUNCITONG_TURN_OFF;	
#endif

#if(SK60_CTR>0)
	devPar.sk60Fuction = FUNCITONG_SK60_INIT;
#endif
#if(KXYL_CTR>0)
	devPar.kxylFuction = FUNCITONG_KXYL_INIT;
#endif
	devConfigValueFlashStore(configValue);
	devParamterStore(devPar);	
	gprsAddressSet();
}
//�豸ID �ڵ�������Ϣ���豸ͨ�ŷ�ʽ���豸ʱ�ӡ��¶ȴ�����ID����
void rs232DevSet(uint8_t *buf)
{
   devIdWrite(&buf[0]);
   devSendPeriodSet(&buf[4]);
   devInfWrite(&buf[8]);
   rtcTimeRs232Set(&buf[12]);
   devCommunictionSet(&buf[19]);
   b20IdWrite(&buf[23]);
   //�豸ע����Ϣ����
   devRegAuthCodeUnSet();
   //Flash�洢���ݳ�ʼ��
    sensorDataWriteAddr = findDataWriteAddr();//SENSOR_DATA_BASE_ADDR;
		sensorDataWriteAddrStore(DEV_SELF_SET_FLAG);
    curGprsSendAddr 		= sensorDataWriteAddr;
		gprsSendAddrStore(DEV_SELF_SET_FLAG);
		ClearData_SWReset();
    calAllCounter.i = 0x00;
    calCounter = 0x00;
}

void devfawSet(uint8_t *buf,int32_t bufLength)
{
  uint16_t devi = 0x00;
	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;
	int32_t dataLength = 0x00;
	DS1302_TIME readTime;
	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;

	dataLength = bufLength;

	//����������еĶ�������
	for(devj = 0x00;devj < DEV_MAX_NODE_NUM;devj ++)
	{
	   curFaw[devj].Fa.bytes.value1 = 0x00; 
		 curFaw[devj].Fa.bytes.value2 = 0x00; 
		 curFaw[devj].Fa.bytes.value3 = 0x00;  
		 curFaw[devj].Fa.bytes.value4 = 0x00; 
		 
		 curFaw[devj].Fw.bytes.value1 = 0x00;  
		 curFaw[devj].Fw.bytes.value2 = 0x00; 
		 curFaw[devj].Fw.bytes.value3 = 0x00; 
		 curFaw[devj].Fw.bytes.value4 = 0x00; 
		 curFaw[devj].Faw.f = 0;
//		if(eachDevNum == HIG_LOW_FRE_BOTH)
//		{
//			curFaw[devj].Fal.bytes.value1 = 0x00; 
//			curFaw[devj].Fal.bytes.value2 = 0x00; 
//			curFaw[devj].Fal.bytes.value3 = 0x00;  
//			curFaw[devj].Fal.bytes.value4 = 0x00; 

//			curFaw[devj].Fwl.bytes.value1 = 0x00;  
//			curFaw[devj].Fwl.bytes.value2 = 0x00; 
//			curFaw[devj].Fwl.bytes.value3 = 0x00; 
//			curFaw[devj].Fwl.bytes.value4 = 0x00; 
//			curFaw[devj].Fawl.f = 0;		 
//		}	    
	}
//������ֵ��ȡ
	for(devj= 0x00;devj < dataLength ;devj ++)
	{
		curFaw[devj].Fa.bytes.value1 = buf[devi++]; 
		curFaw[devj].Fa.bytes.value2 = buf[devi++];
		curFaw[devj].Fa.bytes.value3 = buf[devi++];  
		curFaw[devj].Fa.bytes.value4 = buf[devi++];
//		if(eachDevNum == HIG_LOW_FRE_BOTH)
//		{
	// 	   curFaw[devj].Fal.bytes.value1 = buf[devi++]; 
	// 		 curFaw[devj].Fal.bytes.value2 = buf[devi++];
	// 		 curFaw[devj].Fal.bytes.value3 = buf[devi++];  
	// 		 curFaw[devj].Fal.bytes.value4 = buf[devi++];
//		}	  
		if(buf[devi]==0x2c)
		{
			devi++;
			break;
		}
		if(devi > bufLength)
		{
			 break;
		}
	}
//ˮ��ֵ��ȡ
//        dataLength = bufLength - devi ;
	for(devj= 0x00;devj < bufLength ;devj ++)
	{
		curFaw[devj].Fw.bytes.value1 = buf[devi++]; 
		curFaw[devj].Fw.bytes.value2 = buf[devi++];
		curFaw[devj].Fw.bytes.value3 = buf[devi++];  
		curFaw[devj].Fw.bytes.value4 = buf[devi++];
//	if(eachDevNum == HIG_LOW_FRE_BOTH)
//	{
// 	   curFaw[devj].Fwl.bytes.value1 = buf[devi++];
// 		 curFaw[devj].Fwl.bytes.value2 = buf[devi++];
// 		 curFaw[devj].Fwl.bytes.value3 = buf[devi++];
// 		 curFaw[devj].Fwl.bytes.value4 = buf[devi++];
//	}
		if(devi > dataLength)
		{
			break;
		}
	}
//�����С�ˮ�в�ֵ����
	for(devj= 0x00;devj < devSenNum ;devj ++)
	{
		curFaw[devj].Faw.f = curFaw[devj].Fa.f - curFaw[devj].Fw.f; 
//		if(eachDevNum == HIG_LOW_FRE_BOTH)
//		{
	//curFaw[devj].Fawl.f = curFaw[devj].Fal.f - curFaw[devj].Fwl.f;		 
//		}		
	}
//�洢
  dataWriteAddress = findFlashWriteAddr(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] |= RS232_SET_FLAG;
	writeBufLength = struct2Buf(curFaw,DEV_WATER_AIR_DATA,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
  FlashWrite(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_256_PACK_SIZE);
}

void devFawRead(void)
{
//	int8_t result = -1;
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705

	dataWriteAddress = findFlashWriteAddr(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
	dataReadAddress = findFlashReadAddr(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,dataWriteAddress,BYTE_256_PACK_SIZE,writeBuf);
 //   SPI_FLASH_BufferRead(writeBuf,dataReadAddress,BYTE_256_PACK_SIZE);
	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < BYTE_256_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
//result	=	0;
					writeBufLength = datai-10; 	
					buf2Struct(curFaw,DEV_WATER_AIR_DATA,&writeBuf[10],writeBufLength);
					break;
				}
			}
		}
	}
//	if(result != 0)
//	{
//		for(datai = 0x00;datai < DEV_MAX_NODE_NUM;datai++)
//		{
//			curFaw[datai].Fa.f 	= UN_INIT_WATER_FA;
//			curFaw[datai].Fw.f 	= UN_INIT_WATER_FW;
//			curFaw[datai].Faw.f = curFaw[datai].Fa.f - curFaw[datai].Fw.f;
//		}
//	}
}



void devAbcSet(uint8_t *buf,int32_t bufLength)
{
	uint16_t devi = 0x00;
	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;

	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;
	for(devj = 0x00;devj < DEV_MAX_NODE_NUM;devj++)
	{
		curAbc[devj].a.bytes.value1 = 0x00; 
		curAbc[devj].a.bytes.value2 = 0x00; 
		curAbc[devj].a.bytes.value3 = 0x00;  
		curAbc[devj].a.bytes.value4 = 0x00; 

		curAbc[devj].b1.bytes.value1 = 0x00; 
		curAbc[devj].b1.bytes.value2 = 0x00; 
		curAbc[devj].b1.bytes.value3 = 0x00; 
		curAbc[devj].b1.bytes.value4 = 0x00; 

		curAbc[devj].c.bytes.value1 = 0x00; 
		curAbc[devj].c.bytes.value2 = 0x00; 
		curAbc[devj].c.bytes.value3 = 0x00; 
		curAbc[devj].c.bytes.value4 = 0x00; 
	}
		//����������ж�������
//ˮ��ֵ��ȡ
	 for(devj= 0x00;devj < devSenNum ;devj ++)
	 {
		curAbc[devj].a.bytes.value1 = buf[devi++]; 
		curAbc[devj].a.bytes.value2 = buf[devi++];
		curAbc[devj].a.bytes.value3 = buf[devi++];  
		curAbc[devj].a.bytes.value4 = buf[devi++];
		devi++;
		curAbc[devj].b1.bytes.value1 = buf[devi++]; 
		curAbc[devj].b1.bytes.value2 = buf[devi++];
		curAbc[devj].b1.bytes.value3 = buf[devi++];  
		curAbc[devj].b1.bytes.value4 = buf[devi++];
		devi++;
		curAbc[devj].c.bytes.value1 = buf[devi++]; 
		curAbc[devj].c.bytes.value2 = buf[devi++];
		curAbc[devj].c.bytes.value3 = buf[devi++];  
		curAbc[devj].c.bytes.value4 = buf[devi++];
		devi++;
		if(devi > bufLength)
		{
			break;
		}
	}
//�洢
  dataWriteAddress = findFlashWriteAddr(ABC_VALUE_BASE_ADDR,ABC_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;
	writeBufLength = struct2Buf(curAbc,DEV_ABC_DATA,&writeBuf[10]);
	writeBufLength +=10;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	FlashWrite(ABC_VALUE_BASE_ADDR,ABC_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_256_PACK_SIZE);
}
void devfawStoreFlash(soilfaw *curFawValue)
{
	uint16_t devi = 0x00;
	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;
	//	int32_t dataLength = 0x00;
	DS1302_TIME readTime;
	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	
//�����С�ˮ�в�ֵ����
	for(devj= 0x00;devj < devSenNum ;devj ++)
	{
		curFawValue[devj].Faw.f =	curFawValue[devj].Fa.f - curFawValue[devj].Fw.f; 	 		 
	}
//�洢
  dataWriteAddress = findFlashWriteAddr(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++]= GPRS_SET_FLAG;
	writeBufLength = struct2Buf(curFawValue,DEV_WATER_AIR_DATA,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
  FlashWrite(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_256_PACK_SIZE);

}
void devAbcStoreFlash(soilabc *curAbc)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
    uint8_t crcData = 0x00;

	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;
//�洢
  dataWriteAddress = findFlashWriteAddr(ABC_VALUE_BASE_ADDR,ABC_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
    //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = GPRS_SET_FLAG ;
    writeBufLength = struct2Buf(curAbc,DEV_ABC_DATA,&writeBuf[10]);
	writeBufLength +=10;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
    FlashWrite(ABC_VALUE_BASE_ADDR,ABC_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_256_PACK_SIZE);


}



void devabcRead(void)
{
	int8_t result = -1;
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
  dataWriteAddress = findFlashWriteAddr(ABC_VALUE_BASE_ADDR,ABC_VALUE_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("abc�洢��ַ%d\n\r",dataWriteAddress);
#endif
	dataReadAddress = findFlashReadAddr(DEV_WATER_AIR_VALUE_BASE_ADDR,DEV_WATER_AIR_VALUE_END_ADDR,dataWriteAddress,BYTE_256_PACK_SIZE,writeBuf);

//      SPI_FLASH_BufferRead(writeBuf,dataWriteAddress,BYTE_256_PACK_SIZE);
	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < BYTE_256_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					result = 0;
					writeBufLength = datai-10; 	
					buf2Struct(curAbc,DEV_ABC_DATA,&writeBuf[10],writeBufLength);
					break;
				}
			}
		}
	}
	if(result != 0)
	{
		for(datai = 0x00;datai < DEV_MAX_NODE_NUM;datai++)
		{
			curAbc[datai].a.f 	= 0.3649;
			curAbc[datai].b1.f 	= 0.2662;
			curAbc[datai].c.f 	= 0.0;
		}
	}
}



#define SEP_CHARACTER                0x2C//,
#define DEAL_DATA_ADDRESS            0x04

void devDepSet(uint8_t *buf ,int32_t bufLength)
{
	uint16_t devi = 0x00;
	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[BYTE_128_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;//\

	//����������ж�������
	for(devj = 0x00;devj < DEV_MAX_NODE_NUM;devj++)
	{
		curDep[devj].dep.bytes.value1 = 0x00; 
		curDep[devj].dep.bytes.value2 = 0x00; 
		curDep[devj].dep.bytes.value3 = 0x00; 
		curDep[devj].dep.bytes.value4 = 0x00; 	    
	}

//ˮ��ֵ��ȡ
	for(devj= 0x00;devj < devSenNum ;devj ++)
	{
		curDep[devj].dep.bytes.value1 = buf[devi++]; 
		curDep[devj].dep.bytes.value2 = buf[devi++];
		curDep[devj].dep.bytes.value3 = buf[devi++];  
		curDep[devj].dep.bytes.value4 = buf[devi++];
		if(devi > bufLength)
		{
			break;
		}
	}
			
//�洢
  dataWriteAddress = findFlashWriteAddr(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,BYTE_128_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
 	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;
  writeBufLength = struct2Buf(curDep,DEV_DEP_DATA,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
#ifdef MODE_C
		printf("dep�洢����%d\n\r",writeBufLength);
#endif
  FlashWrite(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_128_PACK_SIZE);
}


void devDepSetByGprs(void)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[BYTE_128_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;//

//�洢
  dataWriteAddress = findFlashWriteAddr(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,BYTE_128_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
  //���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;
    writeBufLength = struct2Buf(curDep,DEV_DEP_DATA,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
#ifdef MODE_C
		printf("dep�洢����%d\n\r",writeBufLength);
#endif
  FlashWrite(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_128_PACK_SIZE);

}
void devDepRead(void)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[BYTE_128_PACK_SIZE];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
  dataWriteAddress = findFlashWriteAddr(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,BYTE_128_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("dep�洢��ַ%d\n\r",dataWriteAddress);
#endif
	dataReadAddress = findFlashReadAddr(DEV_DEP_VALUE_BASE_ADDR,DEV_DEP_VALUE_END_ADDR,dataWriteAddress,BYTE_128_PACK_SIZE,writeBuf);

//      SPI_FLASH_BufferRead(writeBuf,dataWriteAddress,BYTE_128_PACK_SIZE);
	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 11;datai < BYTE_128_PACK_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
             	writeBufLength =( datai-10); 	
                 buf2Struct(curDep,DEV_DEP_DATA,&writeBuf[10],writeBufLength);
			     break;
			 }
		 
		 }
	}
	}
    
}



void calCounterTest(void)
{
   uint16_t i = 0x00;
   calAllCounter.i = 0;
   SPI_FLASH_SectorErase(DEV_SENSOR_COUNTER_BASE_ADDR);
   for(i= 0;i< 80;i++){
   calAllCounter.i++;
   printf("�ܹ��Ѳɼ�[%d]��\r\n",calAllCounter.i); 
   devCalCounterSetByInt(DEV_SELF_SET_FLAG);
  }
}
void devCalCounterSetByInt(uint8_t setFlag)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;//
	
	   //���Ӵ洢ʱ���¼ �������÷�ʽ
        readDs1302Time(&readTime);
        dataWriteAddress = findFlashWriteAddr(DEV_SENSOR_COUNTER_BASE_ADDR,DEV_SENSOR_COUNTER_END_ADDR,BYTE_32_PACK_SIZE,writeBuf);
			//�洢����
			writeBuf[writeBufLength++]= 0x2c;
         	writeBuf[writeBufLength++]= 0x2c;
	        writeBuf[writeBufLength++]= 0x2c;	
	        writeBuf[writeBufLength++]= readTime.year;
         	writeBuf[writeBufLength++]= readTime.month;
        	writeBuf[writeBufLength++]= readTime.date;
	        writeBuf[writeBufLength++]= readTime.hour;
	        writeBuf[writeBufLength++]= readTime.min;
         	writeBuf[writeBufLength++]= readTime.sec;
			writeBuf[writeBufLength++] = setFlag;
            writeBufLength = struct2Buf(&calAllCounter,DEV_VALUE_DATA ,&writeBuf[10]);
        	writeBufLength +=10; 
        	for(devi = 0x00;devi < writeBufLength;devi++)
        	{
         	 crcData += writeBuf[devi];
         	}
        	writeBuf[writeBufLength++]= crcData;

        	writeBuf[writeBufLength++]= END_CHARCCTER;
	        writeBuf[writeBufLength++]= END_CHARCCTER;
	        writeBuf[writeBufLength++]= END_CHARCCTER;

      FlashWrite(DEV_SENSOR_COUNTER_BASE_ADDR,DEV_SENSOR_COUNTER_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_32_PACK_SIZE);
		 	   

}

void devCalCounterRead(DS1302_TIME *readTime)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
	
	dataWriteAddress = findFlashWriteAddr(DEV_SENSOR_COUNTER_BASE_ADDR,DEV_SENSOR_COUNTER_END_ADDR,BYTE_32_PACK_SIZE,writeBuf);
	//    dataWriteAddress = 	DEV_SENSOR_COUNTER_BASE_ADDR +3488;
	dataReadAddress = findFlashReadAddr(DEV_SENSOR_COUNTER_BASE_ADDR,DEV_SENSOR_COUNTER_END_ADDR,dataWriteAddress,BYTE_32_PACK_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < BYTE_32_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					writeBufLength =( datai-10); 	
					buf2Struct(&calAllCounter,DEV_VALUE_DATA ,&writeBuf[10],writeBufLength);
					readTime->year	= writeBuf[3];
					readTime->month	= writeBuf[4];
					readTime->date	= writeBuf[5];
					readTime->hour	= writeBuf[6];
					readTime->min		= writeBuf[7];
					readTime->sec		= writeBuf[8];
					readTime->day		= 1;
					break;
				}
			}
		}
	}
	else
	{
		calAllCounter.i = 0x00;;
	}
}


void firewareWrite(uint8_t *buf,uint32_t *writeAddr,uint16_t bufLength)
{
   	uint16_t writeBufLength = 0x00;
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[FIRE_WARE_STORE_SIZE+2];
	uint16_t writeri = 0x00;
	DS1302_TIME readTime;
//    	readDs1302Time(&readTime);
	readTime = readTime;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;
    for(writeri = 0x00;writeri < bufLength;writeri ++)
	{
	  writeBuf[writeBufLength++] = buf[writeri];
	}
	for(devi = 0x0A;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//���ݴ洢
    //���Ӵ洢ʱ���¼ �������÷�ʽ
  FlashWrite(DEV_FIREWARE2_BASE_ADDR,DEV_FIREWARE2_END_ADDR,writeAddr,writeBuf,writeBufLength,FIRE_WARE_STORE_SIZE);
  
}

int8_t firewareCheck(uint32_t readAddr)//210705
{
	int8_t result = -1;
	uint8_t readBuf[FIRE_WARE_STORE_SIZE+1];
	uint32_t readAddress =0;
	uint16_t i =0;
	uint16_t crcj = 0;
	uint8_t crcData = 0x00;
	
	readAddress = readAddr - FIRE_WARE_STORE_SIZE;
	
	if((readAddress<DEV_FIREWARE2_BASE_ADDR)||(readAddress>=DEV_FIREWARE2_END_ADDR))return result;
	
	result = -2;
	SPI_FLASH_BufferRead(readBuf,readAddress,FIRE_WARE_STORE_SIZE);
	
	if((readBuf[0]==0x2C)&&(readBuf[1]==0x2C)&&(readBuf[2]==0x2C))
	{
		result = -3;
		for(i = (FIRE_WARE_STORE_SIZE-1);i>2;i--)
		{
			if(readBuf[i] ==END_CHARCCTER )
			{
				if((readBuf[i-1] ==END_CHARCCTER )&&(readBuf[i-2] ==END_CHARCCTER ))
				{
					/*У����*/
					for(crcj = 0x0A;crcj < i -3;crcj ++)
					{
						crcData += readBuf[crcj];   
					}
					if(crcData == readBuf[i-3])
					{
						result = 1;						
						break;
					}
				}		 
			}			    
		}		 
	}	
#if GPRS_DEBUG>1
	printf("\n\r read firwmare result %d ",result);
#endif
	return result;
}

#if(MQTT_USE_CTR>0)
void mqttAddressClear(void)
{
	uint16_t tmpi=0;
	uint16_t	len=0;
	char *buf;
	buf = &mqttServerAddrBuf[0].userFlag;
	len = MQTT_MAX_MULTI_CONNECTION_SIZE*MQTT_SEVER_ADDRESS_LENGTH;
	for(tmpi=0;tmpi<len;tmpi++)buf[tmpi]=0;
}
void mqttAddressSet(void)
{
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint16_t  writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	uint8_t writeBuf[MQTT_ADDR_STORE_SIZE+2];
	DS1302_TIME readTime;
	
	dataWriteAddress = findFlashWriteAddr(DEV_MQTT_SERVER_BASE_ADDR,DEV_MQTT_SERVER_END_ADDR,MQTT_ADDR_STORE_SIZE,writeBuf);

	//���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++]= GPRS_SET_FLAG;

	writeBufLength  = struct2Buf(mqttServerAddrBuf,DEV_MQTT_ADDRESS_DATA,&writeBuf[10]);
	writeBufLength += 10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//	printf("\r\n gprsAddrSave size:%d,",writeBufLength);
	//���Ӵ洢ʱ���¼ �������÷�ʽ
	FlashWrite(DEV_MQTT_SERVER_BASE_ADDR,DEV_MQTT_SERVER_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,MQTT_ADDR_STORE_SIZE);
}

static void mqttAddressRead(void)
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[MQTT_ADDR_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
	
	dataWriteAddress = findFlashWriteAddr(DEV_MQTT_SERVER_BASE_ADDR,DEV_MQTT_SERVER_END_ADDR,MQTT_ADDR_STORE_SIZE,writeBuf);
	dataReadAddress = findFlashReadAddr(DEV_MQTT_SERVER_BASE_ADDR,DEV_MQTT_SERVER_END_ADDR,dataWriteAddress,MQTT_ADDR_STORE_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < MQTT_ADDR_STORE_SIZE;datai ++)
		{
			if((writeBuf[datai] ==END_CHARCCTER )&&(writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			{
				if((writeBuf[datai+3] ==0xFF )&&(writeBuf[datai+4] ==0xFF ))
				{
				//	writeBufLength = datai-11; 
					buf2Struct(mqttServerAddrBuf,DEV_MQTT_ADDRESS_DATA,&writeBuf[10],0);
					break;
				}
			}
		}
	}  
}
#endif
void gprsAddressSet(void)
{
 //   uint8_t bufi = 0x00;
//	uint8_t bufj = 0x00;
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint16_t  writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
    uint8_t writeBuf[GPRS_ADDR_STORE_SIZE];
			DS1302_TIME readTime;
			//�����������ж�������

  dataWriteAddress = findFlashWriteAddr(DEV_GPRS_SERVER_BASE_ADDR,DEV_GPRS_SERVER_END_ADDR,GPRS_ADDR_STORE_SIZE,writeBuf);

//	serAddrBuf[0] = (uint8_t)bufLength; 

	  //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = GPRS_SET_FLAG;

    writeBufLength = struct2Buf(stationServerAddrBuf,DEV_GPRS_ADDRESS_DATA,&writeBuf[10]);

	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//�洢

#ifdef MODE_C
		printf("gprs�洢��ַ%d\n\r",dataWriteAddress);
#endif
  //���Ӵ洢ʱ���¼ �������÷�ʽ
  FlashWrite(DEV_GPRS_SERVER_BASE_ADDR,DEV_GPRS_SERVER_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,GPRS_ADDR_STORE_SIZE);

}

void gprsAddressRead(void)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
//	uint16_t dataj = 0x00;
	uint8_t writeBuf[GPRS_ADDR_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
  dataWriteAddress = findFlashWriteAddr(DEV_GPRS_SERVER_BASE_ADDR,DEV_GPRS_SERVER_END_ADDR,GPRS_ADDR_STORE_SIZE,writeBuf);
#ifdef MODE_C
		printf("gprs�洢��ַ%d\n\r",dataWriteAddress);
#endif
	dataReadAddress = findFlashReadAddr(DEV_GPRS_SERVER_BASE_ADDR,DEV_GPRS_SERVER_END_ADDR,dataWriteAddress,GPRS_ADDR_STORE_SIZE,writeBuf);
#ifdef MODE_C
		printf("gprs������ַ%d\n\r",dataReadAddress);
#endif //    SPI_FLASH_BufferRead(writeBuf,dataReadAddress,BYTE_128_PACK_SIZE);
	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < GPRS_ADDR_STORE_SIZE;datai ++)
	{
	   if((writeBuf[datai] ==END_CHARCCTER )&&(writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			{
				if((writeBuf[datai+3] ==0xFF )&&(writeBuf[datai+4] ==0xFF ))
			 {
                 writeBufLength = datai-10; 
                 buf2Struct(stationServerAddrBuf,DEV_GPRS_ADDRESS_DATA,&writeBuf[10],writeBufLength);
			     break;
			 }
		 
		 }
	}
	}
  
}

void devSnWrite(uint8_t * buf,uint16_t bufLength)
{
    uint8_t bufi = 0x00;
	uint8_t bufj = 0x00;
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;

	uint16_t  writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	DS1302_TIME readTime;
			//�����������ж�������
	for(bufi = 0x00;bufi < DEV_SN_SIZE;bufi++)
	{
	   devSnBuf[bufi]  = 0x20;
	}


  dataWriteAddress = findFlashWriteAddr(DEV_SN_BASE_ADDR,DEV_SN_END_ADDR,BYTE_32_PACK_SIZE,writeBuf);

	devSnBuf[0] = (uint8_t)bufLength; 

	  //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;

	for(bufi = 0x00;bufi < bufLength;bufi++)
	{
	   devSnBuf[bufi+1]  = buf[bufi];
	   //writeBuf[11+bufi] = buf[bufi];
	}

	bufi++;
	for(bufj = 0x00;bufj< DEV_SN_SIZE;bufj++)
	{
	 writeBuf[writeBufLength++] =devSnBuf[bufj]; 
	}
//	writeBuf[10] = serAddrBuf[0];
//	writeBufLength = bufi +11;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//�洢

#ifdef MODE_C
		printf("dev sn�洢��ַ%d\n\r",dataWriteAddress);
#endif
  //���Ӵ洢ʱ���¼ �������÷�ʽ
  FlashWrite(DEV_SN_BASE_ADDR,DEV_SN_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_32_PACK_SIZE);
    
}




void devSnRead(void)
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
//	uint32_t dataReadAddress = 0x00;
  dataWriteAddress = findFlashWriteAddr(DEV_SN_BASE_ADDR,DEV_SN_END_ADDR,BYTE_32_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("sn�洢��ַ%d\n\r",dataWriteAddress);
#endif
	(void)findFlashReadAddr(DEV_SN_BASE_ADDR,DEV_SN_END_ADDR,dataWriteAddress,BYTE_32_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("sn������ַ%d\n\r",dataReadAddress);
#endif //    SPI_FLASH_BufferRead(writeBuf,dataReadAddress,BYTE_128_PACK_SIZE);
	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < BYTE_32_PACK_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
                 for(dataj = 0x00;dataj < (datai-10);dataj ++)
				 {
				 
				 	devSnBuf[dataj] = writeBuf[dataj+10];
				 }
			     break;
			 }
		 
		 }
	}
	}
  
}


void devAuthCodeWrite(uint8_t * buf,uint16_t bufLength)
{
    uint8_t bufi = 0x00;
	uint8_t bufj = 0x00;
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint16_t  writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
		uint8_t writeBuf[BYTE_64_PACK_SIZE+2];
			DS1302_TIME readTime;
			//�����������ж�������
	for(bufi = 0x00;bufi < AUTH_CODE_SIZE;bufi++)
	{
	   authCodeBuf[bufi]  = 0x00;
	}
  dataWriteAddress = findFlashWriteAddr(DEV_AUTH_CODE_BASE_ADDR,DEV_AUTH_CODE_END_ADDR,BYTE_64_PACK_SIZE,writeBuf);

	authCodeBuf[0] = (uint8_t)bufLength; 

	  //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;

	for(bufi = 0x00;bufi < bufLength;bufi++)
	{
	   authCodeBuf[bufi+1]  = buf[bufi];
//	   writeBuf[11+bufi] = buf[bufi];
	}

	bufi++;
	for(bufj = 0x00;bufj< bufi;bufj++)
	{
	 writeBuf[writeBufLength++] =authCodeBuf[bufj]; 
	}
//	writeBuf[10] = serAddrBuf[0];
//	writeBufLength = bufi +11;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//�洢

#ifdef MODE_C
		printf("dev sn�洢��ַ%d\n\r",dataWriteAddress);
#endif
  //���Ӵ洢ʱ���¼ �������÷�ʽ
  FlashWrite(DEV_AUTH_CODE_BASE_ADDR,DEV_AUTH_CODE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,BYTE_64_PACK_SIZE);
    
}


void devAuthCodeRead(void)
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[BYTE_64_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
//	uint32_t dataReadAddress = 0x00;
  dataWriteAddress = findFlashWriteAddr(DEV_AUTH_CODE_BASE_ADDR,DEV_AUTH_CODE_END_ADDR,BYTE_64_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("sn�洢��ַ%d\n\r",dataWriteAddress);
#endif
	(void)findFlashReadAddr(DEV_AUTH_CODE_BASE_ADDR,DEV_AUTH_CODE_END_ADDR,dataWriteAddress,BYTE_64_PACK_SIZE,writeBuf);
#ifdef MODE_C
		printf("sn������ַ%d\n\r",dataReadAddress);
#endif //    SPI_FLASH_BufferRead(writeBuf,dataReadAddress,BYTE_128_PACK_SIZE);
	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < BYTE_64_PACK_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
                 for(dataj = 0x00;dataj < (datai-10);dataj ++)
				 {
				 
				 	authCodeBuf[dataj] = writeBuf[dataj+10];
				 }
			     break;
			 }
		 
		 }
	}
	}
}

void devMainFastWrite(uint8_t * buf,uint16_t bufLength)
{
//    uint16_t bufi = 0x00;
	uint16_t bufj = 0x00;
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint16_t  writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	uint8_t writeBuf[MAIN_FAST_STORE_SIZE+2];
	DS1302_TIME readTime;
			//�����������ж�������

    dataWriteAddress = findFlashWriteAddr(DEV_MAINFEST_BASE_ADDR,DEV_MAINFEST_END_ADDR,MAIN_FAST_STORE_SIZE,writeBuf);


	  //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = RS232_SET_FLAG;

	for(bufj = 0x00;bufj< bufLength;bufj++)
	{
	 writeBuf[writeBufLength++] =buf[bufj]; 
	}
//	writeBuf[10] = serAddrBuf[0];
//	writeBufLength = bufi +11;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
//�洢

#ifdef MODE_C
		printf("dev sn�洢��ַ%d\n\r",dataWriteAddress);
#endif
  //���Ӵ洢ʱ���¼ �������÷�ʽ
  FlashWrite(DEV_MAINFEST_BASE_ADDR,DEV_MAINFEST_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,MAIN_FAST_STORE_SIZE);
    
}

void devSnAndMainFastWrite(uint8_t *buf,uint16_t length)
{
   uint16_t i = 0x00;

   for(i = 0x00;i < length;)
   {
   	 if(buf[i]!=0x2c)
	 {
	  i++;
	 }
	 else
	 {
	 break;
	 }
   }
  devSnWrite(buf,i);
  i++;
  devMainFastWrite(&buf[i],(length-i));
}

uint16_t  devMainFastRead(uint8_t *buf)
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[MAIN_FAST_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
//	uint32_t dataReadAddress = 0x00;
  dataWriteAddress = findFlashWriteAddr(DEV_MAINFEST_BASE_ADDR,DEV_MAINFEST_END_ADDR,MAIN_FAST_STORE_SIZE,writeBuf);
	(void)findFlashReadAddr(DEV_MAINFEST_BASE_ADDR,DEV_MAINFEST_END_ADDR,dataWriteAddress,MAIN_FAST_STORE_SIZE,writeBuf);

	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < MAIN_FAST_STORE_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
         for(dataj = 0x00;dataj < (datai-10);dataj ++)
				 {					
				 	buf[dataj] = writeBuf[dataj+10];
				 }
				 
			     break;
			 }
		 
		 }
	}
	}
  	return dataj;
}





uint32_t devGpsFlashWriteAddressFind(void)
{
	uint8_t writeBuf[GPS_DATA_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;	
	dataWriteAddress = findFlashWriteAddr(DEV_GPS_BASE_ADDR,DEV_GPS_END_ADDR,GPS_DATA_STORE_SIZE,writeBuf);

	return dataWriteAddress;
}


void devGpsValueFlashStore(gpsValue loactionValue ,uint32_t *address)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
    uint8_t crcData = 0x00;

	uint8_t writeBuf[GPS_DATA_STORE_SIZE+2];
	uint16_t writeBufLength = 0x00;
//	uint32_t dataWriteAddress = 0x00;
//	DS1302_TIME readTime;

//�洢
//  dataWriteAddress = findFlashWriteAddr(DEV_GPS_BASE_ADDR,DEV_GPS_END_ADDR,BYTE_64_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
    //���Ӵ洢ʱ���¼ �������÷�ʽ
 // 	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= gpsRtcTime.year;
	writeBuf[writeBufLength++]= gpsRtcTime.month;
	writeBuf[writeBufLength++]= gpsRtcTime.date;
	writeBuf[writeBufLength++]= gpsRtcTime.hour;
	writeBuf[writeBufLength++]= gpsRtcTime.min;
	writeBuf[writeBufLength++]= gpsRtcTime.sec;

  writeBufLength = struct2Buf(&loactionValue,DEV_GPS_DATA,&writeBuf[9]);
	writeBufLength +=9;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
  FlashWrite(DEV_GPS_BASE_ADDR,DEV_GPS_END_ADDR,address,writeBuf,writeBufLength,GPS_DATA_STORE_SIZE);


}

int8_t devGpsValueFlashRead(uint32_t readAddress,gpsValue *loactionValue)
{
       int8_t readResult = -1;
       uint8_t crcdata = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[GPS_DATA_STORE_SIZE+2];

  (void)findFlashReadAddr(DEV_GPS_BASE_ADDR,DEV_GPS_END_ADDR,readAddress,GPS_DATA_STORE_SIZE,writeBuf);
	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < GPS_DATA_STORE_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
				 readResult = -2;
         for(dataj = 0x00;dataj < (datai-1);dataj ++)
				 {
				      crcdata += writeBuf[dataj];
			
				 }
				if(crcdata == writeBuf[datai-1])	
				{
					readResult =0;
          buf2Struct(loactionValue,DEV_GPS_DATA,&writeBuf[9],dataj);
				}
			     break;
			 }
		 
		 }
	}
	}
  	return readResult;   
}


uint32_t devXyzFlashWriteAddressFind(void)
{
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;	
       dataWriteAddress = findFlashWriteAddr(DEV_XYZ_BASE_ADDR,DEV_XYZ_END_ADDR,BYTE_32_PACK_SIZE,writeBuf);

       return dataWriteAddress;
}



void devFireWareInfFlashStore(fireWareStruct config)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
    uint8_t crcData = 0x00;

	uint8_t writeBuf[BYTE_128_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;

//�洢
      dataWriteAddress = findFlashWriteAddr(DEV_FIREWARE_INF_BASE_ADDR,DEV_FIREWARE_INF_END_ADDR,BYTE_128_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
    //���Ӵ洢ʱ���¼ �������÷�ʽ
  	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;

      writeBufLength = struct2Buf(&config,DEV_FIREWARE_INF_DATA,&writeBuf[9]);
	writeBufLength +=9;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
    FlashWrite(DEV_FIREWARE_INF_BASE_ADDR,DEV_FIREWARE_INF_END_ADDR,&dataWriteAddress ,writeBuf,writeBufLength,BYTE_128_PACK_SIZE);


}


int8_t devFireWareInfFlashRead(fireWareStruct *config)
{
       int8_t readResult = -1;
       uint8_t crcdata = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[BYTE_128_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0;

//�洢
	dataWriteAddress = findFlashWriteAddr(DEV_FIREWARE_INF_BASE_ADDR,DEV_FIREWARE_INF_END_ADDR,BYTE_128_PACK_SIZE,writeBuf);
	(void)findFlashReadAddr(DEV_FIREWARE_INF_BASE_ADDR,DEV_FIREWARE_INF_END_ADDR,dataWriteAddress,BYTE_128_PACK_SIZE,writeBuf);

	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
	for(datai = 10;datai < BYTE_64_PACK_SIZE;datai ++)
	{
	     if(writeBuf[datai] ==END_CHARCCTER )
		 {
		     if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			 {
			 readResult = -2;
                             for(dataj = 0x00;dataj < (datai-1);dataj ++)
				 {
				      crcdata += writeBuf[dataj];
			
				 }
				if(crcdata == writeBuf[datai-1])	
					{
					readResult =0;
                                  buf2Struct(config,DEV_FIREWARE_INF_DATA,&writeBuf[9],dataj);
				}
			     break;
			 }
		 
		 }
	}
	}

  	return readResult;   
}


void defaultConfigValueSet(void)
{
	devConfig defaultConfig;
	defaultConfig.configVersion = 0x00;
	defaultConfig.curfireWareId = 0x31;
	defaultConfig.devWorkState = FUNCITONG_TURN_OFF;
	
	defaultConfig.devPeriodType		=	FIX_TIME_SEND_TYPE;
	defaultConfig.devPeriod 			= 3600;
	defaultConfig.devPeriodMin 		= 0x00;
	defaultConfig.devPeriodHour 	= 0x01;
	defaultConfig.transmissionDensity = 0x01;
	
	defaultConfig.xyzSharkkey = XYZ_SHARK_OFF;	
	defaultConfig.xValue = 1.0;
	defaultConfig.yValue = 1.0;
	defaultConfig.zValue = 1.0;
	
	defaultConfig.gpsLocation = 0x00;
	defaultConfig.gpsKey	= FUNCITONG_TURN_ON;
	
	devConfigValueFlashStore(defaultConfig);
}



void devConfigValueFlashStore(devConfig config)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
    uint8_t crcData = 0x00;

	uint8_t writeBuf[BYTE_64_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;

//�洢
      dataWriteAddress = findFlashWriteAddr(DEV_CONFIG_BASE_ADDR,DEV_CONFIG_END_ADDR,BYTE_64_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
    //���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;

  writeBufLength = struct2Buf(&config,DEV_CONFIG_DATA,&writeBuf[9]);
	writeBufLength +=9;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
  FlashWrite(DEV_CONFIG_BASE_ADDR,DEV_CONFIG_END_ADDR,&dataWriteAddress ,writeBuf,writeBufLength,BYTE_64_PACK_SIZE);


}


int8_t devConfigValueFlashRead(devConfig *config)
{
	int8_t readResult = -1;
	uint8_t crcdata = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[BYTE_64_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0;
	uint32_t dataReadAddress = 0x00;//210705
//�洢
	dataWriteAddress = findFlashWriteAddr(DEV_CONFIG_BASE_ADDR,DEV_CONFIG_END_ADDR,BYTE_64_PACK_SIZE,writeBuf);
	dataReadAddress = findFlashReadAddr(DEV_CONFIG_BASE_ADDR,DEV_CONFIG_END_ADDR,dataWriteAddress,BYTE_64_PACK_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < BYTE_64_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					readResult = -2;
					for(dataj = 0x00;dataj < (datai-1);dataj ++)
					{
						crcdata += writeBuf[dataj];				
					}
					if(crcdata == writeBuf[datai-1])	
					{
						readResult =0;
						buf2Struct(config,DEV_CONFIG_DATA,&writeBuf[9],dataj);
						if((config->gpsKey != FUNCITONG_TURN_OFF)&&(config->gpsKey!= FUNCITONG_TURN_ON))config->gpsKey = FUNCITONG_TURN_ON;
					}
					break;
				}			 
			}
		}	
	}
	if(0x00!= readResult)
	{
		config->devPeriodType		=	FIX_TIME_SEND_TYPE;
		config->devPeriod 			= 3600;
		config->devPeriodMin 		= 0x00;
		config->devPeriodHour 	= 0x01;

		config->configVersion 	= 0x00;	
		config->curfireWareId 	= 0x31;
		config->devWorkState	 	= 0x00;
		config->gpsLocation 		= 0x00;	
		config->transmissionDensity = 0x01;
		config->xValue = 10;
		config->yValue = 256;
		config->zValue = 10;
		config->xyzSharkkey = 0x00;
		config->gpsKey 			= FUNCITONG_TURN_ON;		
	}
	return readResult;   
}



void devParamterStore(paramter config)
{
    uint16_t devi = 0x00;
//	uint8_t devj = 0x00;
    uint8_t crcData = 0x00;

	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;

//�洢
  dataWriteAddress = findFlashWriteAddr(DEV_PARAMTER_BASE_ADDR,DEV_PARAMTER_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
  //���Ӵ洢ʱ���¼ �������÷�ʽ
    //���Ӵ洢ʱ���¼ �������÷�ʽ
  readDs1302Time(&readTime);
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;

  writeBufLength = struct2Buf(&devPar,DEV_PARAMTER_DATA,&writeBuf[9]);
	writeBufLength +=9;
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
	 crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;	 
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
  FlashWrite(DEV_PARAMTER_BASE_ADDR,DEV_PARAMTER_END_ADDR,&dataWriteAddress ,writeBuf,writeBufLength,BYTE_256_PACK_SIZE);
}


int8_t devParamterRead(paramter *config)
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint8_t writeBuf[BYTE_256_PACK_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	int8_t readResult = -1;
	uint8_t crcdata = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
  dataWriteAddress = findFlashWriteAddr(DEV_PARAMTER_BASE_ADDR,DEV_PARAMTER_END_ADDR,BYTE_256_PACK_SIZE,writeBuf);
	dataReadAddress = findFlashReadAddr(DEV_PARAMTER_BASE_ADDR,DEV_PARAMTER_END_ADDR,dataWriteAddress,BYTE_256_PACK_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < BYTE_256_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
				readResult = -2;
				for(dataj = 0x00;dataj < (datai-1);dataj ++)
				{
					crcdata += writeBuf[dataj];
				}
				if(crcdata == writeBuf[datai-1])	
				{
					readResult =0;
					buf2Struct(config,DEV_PARAMTER_DATA,&writeBuf[9],dataj);
					if((config->mqttDataType<MQTT_DATA_TYPE_CIGEM)||(config->mqttDataType>MQTT_DATA_TYPE_TREND))config->mqttDataType = MQTT_DATA_TYPE_INIT;	
					if((config->usartKey < FUNCITONG_TURN_OFF)||(config->usartKey > FUNCITONG_TURN_ON))config->usartKey = FUNCITONG_INIT_USART;
					if((config->insentekKey<INSENTEK_KEY_OFF)||(config->insentekKey>INSENTEK_KEY_ON))config->insentekKey	=	INSENTEK_KEY_INIT;
					if((config->timeZone<-12)||(config->timeZone>12))config->timeZone=8;
					if((config->plus_intv<60)||(config->plus_intv>14400))config->plus_intv = 600;
					if((config->stationMode<FUNCITONG_TURN_OFF)||(config->stationMode>FUNCITONG_TURN_ON))config->stationMode=FUNCITONG_TURN_ON;
					if((config->gsmRunMode<FUNCITONG_TURN_OFF)||(config->gsmRunMode>FUNCITONG_TURN_ON))config->gsmRunMode=FUNCITONG_TURN_OFF;
					if((config->qxkey<FUNCITONG_TURN_OFF)||(config->qxkey>FUNCITONG_TURN_ON))config->qxkey=FUNCITONG_TURN_ON;
				#if(ACCE_CTR>0)
					if((config->accelerometerkey<FUNCITONG_TURN_OFF)||(config->accelerometerkey>FUNCITONG_TURN_ON))config->accelerometerkey=FUNCITONG_TURN_OFF;
				#endif
					if((config->powerSavingKey<FUNCITONG_TURN_OFF)||(config->powerSavingKey>FUNCITONG_TURN_ON))config->powerSavingKey=POWER_SAVING_KEY_INIT;
				#if(SK60_CTR>0)
					if((config->sk60Fuction<FUNCITONG_SK60_OFF)||(config->sk60Fuction>FUNCITONG_SK60_RAW))config->sk60Fuction = FUNCITONG_SK60_INIT;
				#endif 
				#if(KXYL_CTR>0)
					if((config->kxylFuction<FUNCITONG_TURN_OFF)||(config->kxylFuction>FUNCITONG_TURN_ON))config->kxylFuction = FUNCITONG_KXYL_INIT;
				#endif 
				}
				break;
				}
			}
		}
	}
	if(0x00!= readResult)
	{
		config->timeZone= 0x08;
//	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)	
//		datai =  sprintf(config->apnBuf+1, "%s","AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
//	#else
//		datai =  sprintf(config->apnBuf+1, "%s","AT+CSTT=\"CMNET\"\r\n");
//	#endif		
//		config->apnBuf[0]= datai ;	  
		config->gpsInterVal = GPS_INTERVAL_INIT;
		config->usartKey 		= FUNCITONG_INIT_USART;
		config->insentekKey	=	INSENTEK_KEY_INIT;
		config->qxkey				=	FUNCITONG_TURN_ON;
	#if(ACCE_CTR>0)
		config->accelerometerkey	=	FUNCITONG_TURN_OFF;
	#endif
	#if(MQTT_USE_CTR>0)
		config->mqttDataType 		= MQTT_DATA_TYPE_INIT;
		config->stationMode			=	FUNCITONG_TURN_ON;	
		config->gsmRunMode			=	FUNCITONG_TURN_OFF;
		config->powerSavingKey	=	POWER_SAVING_KEY_INIT;
		config->plus_intv		= 600;
	#else
		config->stationMode	=	FUNCITONG_TURN_OFF;				
	#endif
	#if(SK60_CTR>0)
		config->sk60Fuction = FUNCITONG_SK60_INIT;
	#endif
	#if(KXYL_CTR>0)
		config->kxylFuction = FUNCITONG_KXYL_INIT;
	#endif
	}	
	return readResult;	  
}

void sensorDataWriteAddrStore(uint8_t setFlag)
{
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[SENSOR_WRITEADDR_STORE_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;//
	value_t dataTmp;

	//���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);

	dataWriteAddress = findFlashWriteAddr(SENSOR_DATA_WRITE_BASE_ADDR,SENSOR_DATA_WRITE_END_ADDR,SENSOR_WRITEADDR_STORE_SIZE,writeBuf);
	//�洢����
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = setFlag;
	dataTmp.i = sensorDataWriteAddr;
	writeBufLength = struct2Buf(&dataTmp,DEV_VALUE_DATA ,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;

	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;

	if(xSemaphoreTake(xSPIFlashMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
	{
		FlashWrite(SENSOR_DATA_WRITE_BASE_ADDR,SENSOR_DATA_WRITE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,SENSOR_WRITEADDR_STORE_SIZE);
		xSemaphoreGive(xSPIFlashMutex);
	}

}

void sensorDataWriteAddrRead(void)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	value_t dataTmp = {0};
	uint8_t writeBuf[SENSOR_WRITEADDR_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint32_t dataReadAddress = 0x00;//210705
	dataWriteAddress = findFlashWriteAddr(SENSOR_DATA_WRITE_BASE_ADDR,SENSOR_DATA_WRITE_END_ADDR,SENSOR_WRITEADDR_STORE_SIZE,writeBuf);	
	dataReadAddress = findFlashReadAddr(SENSOR_DATA_WRITE_BASE_ADDR,SENSOR_DATA_WRITE_END_ADDR,dataWriteAddress,SENSOR_WRITEADDR_STORE_SIZE,writeBuf);
	
	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < SENSOR_WRITEADDR_STORE_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{				
					writeBufLength =( datai-10); 	
					buf2Struct(&dataTmp.i,DEV_VALUE_DATA ,&writeBuf[10],writeBufLength);
//					result = 0;
					break;
				}
			}
		}
	}
	sensorDataWriteAddr = dataTmp.i;
}


void gprsSendAddrStore(uint8_t setFlag)
{
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[GPRS_SENDADDR_STORE_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;	
	value_t dataTmp;
	DS1302_TIME readTime;//
	
	readDs1302Time(&readTime);
	//���Ӵ洢ʱ���¼ �������÷�ʽ

	dataWriteAddress = findFlashWriteAddr(GPRS_SENDADDR_BASE_ADDR,GPRS_SENDADDR_END_ADDR,GPRS_SENDADDR_STORE_SIZE,writeBuf);
	//�洢����
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = setFlag;
	dataTmp.i = curGprsSendAddr;
	writeBufLength = struct2Buf(&dataTmp,DEV_VALUE_DATA ,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;

	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;

	if(xSemaphoreTake(xSPIFlashMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
	{
		FlashWrite(GPRS_SENDADDR_BASE_ADDR,GPRS_SENDADDR_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,GPRS_SENDADDR_STORE_SIZE);
		xSemaphoreGive(xSPIFlashMutex);
	}

}

void gprsSendAddrRead(void)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	value_t dataTmp;
	uint8_t writeBuf[GPRS_SENDADDR_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	int8_t result = -1;
	uint32_t dataReadAddress = 0x00;//210705
	dataWriteAddress = findFlashWriteAddr(GPRS_SENDADDR_BASE_ADDR,GPRS_SENDADDR_END_ADDR,GPRS_SENDADDR_STORE_SIZE,writeBuf);	
	dataReadAddress = findFlashReadAddr(GPRS_SENDADDR_BASE_ADDR,GPRS_SENDADDR_END_ADDR,dataWriteAddress,GPRS_SENDADDR_STORE_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < GPRS_SENDADDR_STORE_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					writeBufLength =(datai-10); 	
					buf2Struct(&dataTmp.i,DEV_VALUE_DATA ,&writeBuf[10],writeBufLength);
					result = 0;
					break;
				}
			}
		}
	}
	if(result == 0)
	{
		if((dataTmp.i<SENSOR_DATA_BASE_ADDR)||(dataTmp.i>=SENSOR_DATA_END_ADDR))
		{
			curGprsSendAddr = sensorDataWriteAddr;
		}
		else
		{
			curGprsSendAddr = dataTmp.i;
		}
	}	
	else {curGprsSendAddr = sensorDataWriteAddr;}
	
}
void gprsSendAddrSave(void)
{
	uint32_t gprsSendAddr = SENSOR_DATA_BASE_ADDR;
	uint8_t rxi;
	gprsSendAddr = curGprsSendAddr;
	
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
	delay_ms(5);
	for(rxi=0;rxi<2;rxi++)//210705
	{
		gprsSendAddrStore(DEV_SELF_SET_FLAG);
		gprsSendAddrRead();
		if(gprsSendAddr == curGprsSendAddr)break;
	}
	if(gprsSendAddr != curGprsSendAddr)curGprsSendAddr = gprsSendAddr;
	SPI_Flash_PowerDown();
	power331Off();
}

uint16_t unSendPackNumAdj(uint32_t sendAddr)
{
	uint16_t unSendSize	=	0;
	uint16_t sizeTmp		=	0;
	if(sendAddr < SENSOR_DATA_BASE_ADDR)
	{
		unSendSize = 0;
	}
	else if(sendAddr < sensorDataWriteAddr)
	{
		unSendSize = (sensorDataWriteAddr - sendAddr)/SENSOR_DATA_STORE_SIZE;
	}
	else if(sendAddr > sensorDataWriteAddr)
	{
		sizeTmp			 = (SENSOR_DATA_END_ADDR - sendAddr)/SENSOR_DATA_STORE_SIZE;
		unSendSize	 = (sensorDataWriteAddr - SENSOR_DATA_BASE_ADDR)/SENSOR_DATA_STORE_SIZE;
		unSendSize	+= sizeTmp;
	}
	
	return unSendSize;
}

#if(MQTT_USE_CTR>0)
void mqttSendAddrStore(uint8_t setFlag)
{
	uint16_t devi = 0x00;
	uint8_t crcData = 0x00;
	uint8_t writeBuf[MQTT_SENDADDR_STORE_SIZE+2];
	uint16_t writeBufLength = 0x00;
	uint32_t dataWriteAddress = 0x00;
	DS1302_TIME readTime;//
	
	readDs1302Time(&readTime);

	dataWriteAddress = findFlashWriteAddr(MQTT_SENDADDR_BASE_ADDR,MQTT_SENDADDR_END_ADDR,MQTT_SENDADDR_STORE_SIZE,writeBuf);
	//�洢����
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++] = setFlag;

	writeBufLength = struct2Buf(curMqttSendAddr,DEV_MQTT_SENDADDR_DATA ,&writeBuf[10]);
	writeBufLength +=10; 
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;

	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;

	FlashWrite(MQTT_SENDADDR_BASE_ADDR,MQTT_SENDADDR_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,MQTT_SENDADDR_STORE_SIZE);

}

void mqttSendAddrRead(void)
{
	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	value_t dataTmp[MQTT_MAX_MULTI_CONNECTION_SIZE] = {0};
	uint8_t writeBuf[MQTT_SENDADDR_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	int8_t result = -1;
	uint32_t dataReadAddress = 0x00;//210705
	dataWriteAddress = findFlashWriteAddr(MQTT_SENDADDR_BASE_ADDR,MQTT_SENDADDR_END_ADDR,MQTT_SENDADDR_STORE_SIZE,writeBuf);	
	dataReadAddress = findFlashReadAddr(MQTT_SENDADDR_BASE_ADDR,MQTT_SENDADDR_END_ADDR,dataWriteAddress,MQTT_SENDADDR_STORE_SIZE,writeBuf);

	if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 11;datai < MQTT_SENDADDR_STORE_SIZE;datai ++)
		{
			if((writeBuf[datai] ==END_CHARCCTER )&&(writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
			{
				if((writeBuf[datai+3] == 0xFF)&&(writeBuf[datai+4] ==0xFF)&&(writeBuf[datai+5] ==0xFF))
				{			
					buf2Struct(dataTmp,DEV_MQTT_SENDADDR_DATA ,&writeBuf[10],writeBufLength);
					result = 0;
					break;
				}
			}
		}
	}
	if(result == 0)
	{
		for(datai=0;datai<MQTT_MAX_MULTI_CONNECTION_SIZE;datai++)
		{
			if((dataTmp[datai].i<SENSOR_DATA_BASE_ADDR)||(dataTmp[datai].i>=SENSOR_DATA_END_ADDR))
			{
				dataTmp[datai].i = sensorDataWriteAddr;
			}
			curMqttSendAddr[datai].i = dataTmp[datai].i;
		}
	}	
	else
	{
	 for(datai=0;datai<MQTT_MAX_MULTI_CONNECTION_SIZE;datai++)curMqttSendAddr[datai].i = sensorDataWriteAddr;
	}
}
void mqttSendAddrSave(void)
{
	power331On();
	delay_modeMs(10);
	SPI_Flash_WAKEUP();
	delay_modeMs(5);
	mqttSendAddrStore(DEV_SELF_SET_FLAG);
	SPI_Flash_PowerDown();
	power331Off();
}
 #if(SECOND_MQTT_ADDR>0)
	void mqttSecondAddressStore(secondServerAddr * secondServerBuf)
	{
		uint16_t devi = 0x00;
		uint8_t crcData = 0x00;
		uint16_t  writeBufLength = 0x00;
		uint32_t dataWriteAddress = 0x00;
		uint8_t writeBuf[MQTT_ADDR2_STORE_SIZE+2];
		DS1302_TIME readTime;//
		
		dataWriteAddress = findFlashWriteAddr(DEV_MQTT_SERVER2_BASE_ADDR,DEV_MQTT_SERVER2_END_ADDR,MQTT_ADDR2_STORE_SIZE,writeBuf);

//	readDs1302Time(&readTime);
		readTime	=	rtcTime;
		writeBufLength = 0x00;
		writeBuf[writeBufLength++]= 0x2c;
		writeBuf[writeBufLength++]= 0x2c;
		writeBuf[writeBufLength++]= 0x2c;	
		writeBuf[writeBufLength++]= readTime.year;
		writeBuf[writeBufLength++]= readTime.month;
		writeBuf[writeBufLength++]= readTime.date;
		writeBuf[writeBufLength++]= readTime.hour;
		writeBuf[writeBufLength++]= readTime.min;
		writeBuf[writeBufLength++]= readTime.sec;
		writeBuf[writeBufLength++]= GPRS_SET_FLAG;

		writeBufLength  = struct2Buf(secondServerBuf,DEV_MQTT_ADDRESS2_DATA,&writeBuf[10]);
		writeBufLength += 10; 
		for(devi = 0x00;devi < writeBufLength;devi++)
		{
			crcData += writeBuf[devi];
		}
		writeBuf[writeBufLength++]= crcData;
		writeBuf[writeBufLength++]= END_CHARCCTER;
		writeBuf[writeBufLength++]= END_CHARCCTER;
		writeBuf[writeBufLength++]= END_CHARCCTER;
	//	printf("\r\n gprsAddrSave size:%d,",writeBufLength);
		//���Ӵ洢ʱ���¼ �������÷�ʽ
		FlashWrite(DEV_MQTT_SERVER2_BASE_ADDR,DEV_MQTT_SERVER2_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,MQTT_ADDR2_STORE_SIZE);
	}
	void clearSecondAddrBuffer(secondServerAddr * secondServerBuf)
	{
		secondServerBuf->userFlag 		= 0;
		secondServerBuf->endpoint[0]	= 0;secondServerBuf->endpoint[1]	= '0';secondServerBuf->endpoint[2]	= '\0';
		secondServerBuf->port 				= 0;
		secondServerBuf->clientId[0] 	= 0;secondServerBuf->clientId[1]	= '0';secondServerBuf->clientId[2]	= '\0';
		secondServerBuf->username[0] 	= 0;secondServerBuf->username[1]	= '0';secondServerBuf->username[2]	= '\0';
		secondServerBuf->password[0] 	= 0;secondServerBuf->password[1]	= '0';secondServerBuf->password[2]	= '\0';	
		secondServerBuf->httpaddr[0]	= 0;secondServerBuf->httpaddr[1]	= '0';secondServerBuf->httpaddr[2]	= '\0';
		secondServerBuf->httpport 		= 0;
		secondServerBuf->deviceId[0] 	= 0;secondServerBuf->deviceId[1]	= '0';secondServerBuf->deviceId[2]	= '\0';
		secondServerBuf->apikey[0] 		= 0;secondServerBuf->apikey[1]		= '0';secondServerBuf->apikey[2]		= '\0';	
	}
	void clearSecondAddrStore(void)
	{	
		secondServerAddr secondServerBuf[SECOND_MULTI_CONNECTION_SIZE];	
		uint8_t tmpi;
		power331On();
		delay_modeMs(10);
		SPI_Flash_WAKEUP();
		delay_modeMs(5);
		for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)
		{
			clearSecondAddrBuffer(&secondServerBuf[tmpi]);			
		}
		mqttSecondAddressStore(secondServerBuf);//20210208
		mqttServerNumSecond = 0;
		SPI_Flash_PowerDown();
		power331Off();
	}

	static int8_t mqttSecondAddressRead(secondServerAddr * secondServerBuf)
	{
		int8_t result = -1;
		uint16_t writeBufLength = 0x00;
		uint16_t datai = 0x00;
		uint8_t writeBuf[MQTT_ADDR2_STORE_SIZE+2];
		uint32_t dataWriteAddress = 0x00;
		uint32_t dataReadAddress = 0x00;//210705
		dataWriteAddress = findFlashWriteAddr(DEV_MQTT_SERVER2_BASE_ADDR,DEV_MQTT_SERVER2_END_ADDR,MQTT_ADDR2_STORE_SIZE,writeBuf);
//		printf(" writeAddr:0x%X,",dataWriteAddress);
		dataReadAddress = findFlashReadAddr(DEV_MQTT_SERVER2_BASE_ADDR,DEV_MQTT_SERVER2_END_ADDR,dataWriteAddress,MQTT_ADDR2_STORE_SIZE,writeBuf);
//		printf(" readAddr:0x%X,",dataReadAddress);
		if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c)&&(writeBuf[2]==0x2c))
		{
			for(datai = 10;datai < MQTT_ADDR2_STORE_SIZE;datai ++)
			{
				if((writeBuf[datai] ==END_CHARCCTER)&&(writeBuf[datai+1] ==END_CHARCCTER)&&(writeBuf[datai+2] ==END_CHARCCTER))
				{
					if((writeBuf[datai+3] == 0xFF)&&(writeBuf[datai+4] ==0xFF)&&(writeBuf[datai+5] ==0xFF))
					{
//						writeBufLength = datai-11; 					
						buf2Struct(secondServerBuf,DEV_MQTT_ADDRESS2_DATA,&writeBuf[10],writeBufLength);					
						result = 0;						
						break;
					}
				}
			}
		}
		if(result == 0)
		{		
			if(secondServerBuf[0].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)result = 1;
			
		#if((SECOND_MULTI_CONNECTION_SIZE>1)&&(SECOND_MQTT_ADDR>1))
			if(devPar.secondAddrKey == MQTT_SECOND_ADDR_NUM_2)
			{
				if(secondServerBuf[1].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)result |= 2;
			}else			
		#endif
			{
				if((secondServerBuf[0].userFlag != TCP_SERVER_ADDRESS_USE_FLAG)&&(secondServerBuf[1].userFlag == TCP_SERVER_ADDRESS_USE_FLAG))
				{
					secondServerBuf[0] = secondServerBuf[1];
					result =1;
				}
				clearSecondAddrBuffer(&secondServerBuf[1]);							
			}		
		}
		return result;
	}
	void mqttSecondServerAddrRead(secondServerAddr * secondServerBuf)
	{
		power331On();
		delay_modeMs(10);
		SPI_Flash_WAKEUP();
		delay_modeMs(5);		
		mqttServerNumSecond = mqttSecondAddressRead(secondServerBuf);
		SPI_Flash_PowerDown();
		power331Off();
	}
 #endif
	static void clearMqttAddrStroe(void)
	{
		uint8_t tmpi;
	 #if(SECOND_MQTT_ADDR>0)
		secondServerAddr secondServerBuf[SECOND_MULTI_CONNECTION_SIZE];			
	#endif		
		mqttAddressClear();
		mqttAddressSet();
		mqttServerNumFirst	=	0;
		mqttServerNum	=	0;
		for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)curMqttSendAddr[tmpi].i	=	0;	
	 #if(SECOND_MQTT_ADDR>0)
		for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)clearSecondAddrBuffer(&secondServerBuf[tmpi]);	
		mqttSecondAddressStore(secondServerBuf);//20210208
		mqttServerNumSecond = 0;
	#endif		
	}
 #if(UPGRADE_FRIMWARE_CTR>0)
	void mqttFirewareClear(void)
	{
		power331On();
		delay_ms(10);
		SPI_Flash_WAKEUP();
		delay_ms(5);
		devFlashErare(MQTT_FIREWARE_BASE_ADDR,MQTT_FIREWARE_END_ADDR);
		SPI_Flash_PowerDown();
		power331Off();
	}
	void mqttFirewareStore(uint16_t index,uint8_t *buf,uint16_t buflen)
	{
		uint16_t devi = 0x00;
		uint8_t crcData = 0x00;
		uint8_t writeBuf[FIRE_WARE_STORE_SIZE2+2];
		uint16_t writeBufLength = 0x00;
		uint32_t dataWriteAddress = 0x00;
	
		DS1302_TIME readTime;//		
		if(buflen<=(FIRE_WARE_STORE_SIZE2-18))
		{	
//			readDs1302Time(&readTime);
			readTime	= rtcTime;
			//�洢����
			writeBuf[writeBufLength++]= 0x2c;
			writeBuf[writeBufLength++]= 0x2c;
			writeBuf[writeBufLength++]= 0x2c;	
			writeBuf[writeBufLength++]= readTime.year;
			writeBuf[writeBufLength++]= readTime.month;
			writeBuf[writeBufLength++]= readTime.date;
			writeBuf[writeBufLength++]= readTime.hour;
			writeBuf[writeBufLength++]= readTime.min;
			writeBuf[writeBufLength++]= readTime.sec;
			writeBuf[writeBufLength++] =(uint8_t)((index>>8)&0xFF);
			writeBuf[writeBufLength++] =(uint8_t)((index)&0xFF);
			for(devi=0;devi<buflen;devi++)
			{
				writeBuf[writeBufLength++] = buf[devi];
			}
	//		printf("\r\n save index:%d %X_%X_%X %X_%X_%X ",index,buf[0],buf[1],buf[2],buf[buflen-3],buf[buflen-2],buf[buflen-1]);
			for(devi = 0;devi < writeBufLength;devi++)
			{
				crcData += writeBuf[devi];
			}
			writeBuf[writeBufLength++]= crcData;

			writeBuf[writeBufLength++]= END_CHARCCTER;
			writeBuf[writeBufLength++]= END_CHARCCTER;
			writeBuf[writeBufLength++]= END_CHARCCTER;
			
			dataWriteAddress	=	MQTT_FIREWARE_BASE_ADDR+index*FIRE_WARE_STORE_SIZE2;
			SPI_FLASH_BufferWrite(writeBuf,dataWriteAddress,writeBufLength);
		}
	}
	void mqttFirewareSave(uint16_t index,uint8_t *buf,uint16_t buflen)
	{
		power331On();
		delay_ms(10);
		SPI_Flash_WAKEUP();
		delay_ms(10);
		mqttFirewareStore(index,buf,buflen);
		delay_ms(10);
		SPI_Flash_PowerDown();
		power331Off();
	}
	int8_t mqttFirewarRead(uint16_t index,uint8_t *buf,uint16_t *buflen)
	{
		int8_t 		result = -1;
		uint8_t 	writeBuf[FIRE_WARE_STORE_SIZE2+1];
		uint32_t 	dataReadAddress = 0x00;	
		uint16_t  dataLen = 0x00;
		uint16_t  datai = 0x00;
		uint8_t 	crcData = 0x00;
		
		dataReadAddress	=	MQTT_FIREWARE_BASE_ADDR + index*FIRE_WARE_STORE_SIZE2;
		if(dataReadAddress>=MQTT_FIREWARE_END_ADDR)return -2;
		SPI_FLASH_BufferRead(writeBuf,dataReadAddress,FIRE_WARE_STORE_SIZE2);
		
		datai	=	((uint16_t)writeBuf[9]<<8)+writeBuf[10];
		if(datai != index)return -3;
		
		for(datai = 11;datai < FIRE_WARE_STORE_SIZE2;datai ++)
		{		
			if((writeBuf[datai] ==END_CHARCCTER )&&(writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER))
			{			
				if((writeBuf[datai+3] ==0xFF)&&(writeBuf[datai+4] ==0xFF)&&(writeBuf[datai+5] ==0xFF))
				{
					dataLen	=	datai-1;
					result  = 1;
	//				printf(" dataLen:%d ",dataLen);
					break;
				}
			}		
		}
		if(result == 1)
		{		
			for(datai = 0;datai < dataLen;datai ++)
			{
				crcData += writeBuf[datai];
			}
			if(crcData == writeBuf[datai])
			{
				result = 0;
				dataLen -= 11;
				*buflen	=	dataLen;
				for(datai = 0;datai < dataLen;datai ++)
				{
					buf[datai] = writeBuf[datai+11];
				}
	//			printf("\r\n read index:%d %X_%X_%X %X_%X_%X ",index,buf[0],buf[1],buf[2],buf[dataLen-3],buf[dataLen-2],buf[dataLen-1]);
			}
		}
		return result;
	}
	int8_t mqttFirewarCheck(uint16_t index)
	{
		int8_t 		result = -1;
		uint8_t 	writeBuf[FIRE_WARE_STORE_SIZE+1];
		uint32_t 	dataReadAddress = 0x00;	
		uint16_t  dataLen = 0x00;
		uint16_t  datai = 0x00;
		uint8_t 	crcData = 0x00;
		
		dataReadAddress	=	DEV_FIREWARE2_BASE_ADDR + index*FIRE_WARE_STORE_SIZE;
		if(dataReadAddress>=DEV_FIREWARE2_END_ADDR)return -2;
		SPI_FLASH_BufferRead(writeBuf,dataReadAddress,FIRE_WARE_STORE_SIZE);
			
		for(datai = 11;datai < FIRE_WARE_STORE_SIZE;datai ++)
		{		
			if((writeBuf[datai] ==END_CHARCCTER )&&(writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER))
			{			
				if((writeBuf[datai+3] ==0xFF)&&(writeBuf[datai+4] ==0xFF)&&(writeBuf[datai+5] ==0xFF))
				{
					dataLen	=	datai-1;				
					result  = 1;
					break;
				}
			}		
		}
		if(result == 1)
		{		
			for(datai = 0x0A;datai < dataLen;datai ++)
			{
				crcData += writeBuf[datai];
			}
			if(crcData == writeBuf[datai])
			{
				result = 0;
			}
		}
		return result;
	}
	int8_t mqttFirewareSaveTask(uint8_t *buf,uint16_t firewarenum)
	{
		int8_t result = -1;
		int8_t	readresult	= -1;
		uint16_t index=0;
		uint16_t devi=0;	
		uint16_t	storeptr=0;
		uint16_t	datalen=0;
		uint32_t	fireWareAddr = DEV_FIREWARE2_BASE_ADDR;
		uint16_t	savenum=0;

		power331On();
		delay_ms(10);
		SPI_Flash_WAKEUP();
		delay_ms(10);
		devFlashErare(fireWareAddr,DEV_FIREWARE2_END_ADDR);
		
		for(index=0;index<firewarenum;index++)
		{		
			readresult = mqttFirewarRead(index,buf+storeptr,&datalen);
			printf("\r\n fireware index:%d_%03d %d ptr:%03d_%d_",firewarenum,index,readresult,storeptr,datalen);
			if(readresult!=0)break;
			storeptr += datalen;
			printf("%d",storeptr);
			while(storeptr >= FIREWARE_PACKET_SIZE)
			{
				//�洢fireware���̼�2��ַ
				firewareWrite(buf,&fireWareAddr,FIREWARE_PACKET_SIZE);delay_ms(10);
				storeptr -= FIREWARE_PACKET_SIZE;
				savenum++;
				for(devi=0;devi<storeptr;devi++)
				{
					buf[devi] = buf[devi+FIREWARE_PACKET_SIZE];
				}
				printf(" nextptr:%d",storeptr);
			}
			if(index == (firewarenum-1))
			{
				if(storeptr >0)
				{
					firewareWrite(buf,&fireWareAddr,storeptr);
					savenum++;
					printf(" savenum:%d",savenum);
				}
				result = 1;
			}
			datalen = 0;		
		}
		if(result==1)
		{
			for(index=0;index<savenum;index++)
			{
				if(mqttFirewarCheck(index)!=0)break;
			}
			if(index==savenum)result=0;
		}
		SPI_Flash_PowerDown();
		power331Off();
		return result;
	}

#endif

	void sensorRangeStore(uint8_t setFlag)
	{
		uint16_t devi = 0x00;
		uint8_t crcData = 0x00;
		uint8_t writeBuf[SENSOR_RANGE_STORE_SIZE+2];
		uint16_t writeBufLength = 0x00;
		uint32_t dataWriteAddress = 0x00;
		DS1302_TIME readTime;
		
		//���Ӵ洢ʱ���¼ �������÷�ʽ
		readDs1302Time(&readTime);

		dataWriteAddress = findFlashWriteAddr(SNESOR_RANGE_BASE_ADDR,SNESOR_RANGE_END_ADDR,SENSOR_RANGE_STORE_SIZE,writeBuf);
		//�洢����
		writeBuf[writeBufLength++]= 0x2c;
		writeBuf[writeBufLength++]= 0x2c;
		writeBuf[writeBufLength++]= 0x2c;	
		writeBuf[writeBufLength++]= readTime.year;
		writeBuf[writeBufLength++]= readTime.month;
		writeBuf[writeBufLength++]= readTime.date;
		writeBuf[writeBufLength++]= readTime.hour;
		writeBuf[writeBufLength++]= readTime.min;
		writeBuf[writeBufLength++]= readTime.sec;
		writeBuf[writeBufLength++] = setFlag;
		
		writeBufLength = struct2Buf(cigemsensorlimit,DEV_SENSOR_RANGE_DATA ,&writeBuf[10]);
		writeBufLength +=10; 
		for(devi = 0x00;devi < writeBufLength;devi++)
		{
			crcData += writeBuf[devi];
		}
		writeBuf[writeBufLength++]= crcData;

		writeBuf[writeBufLength++]= END_CHARCCTER;
		writeBuf[writeBufLength++]= END_CHARCCTER;
		writeBuf[writeBufLength++]= END_CHARCCTER;

		FlashWrite(SNESOR_RANGE_BASE_ADDR,SNESOR_RANGE_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,SENSOR_RANGE_STORE_SIZE);//20210513 GPRS_SENDADDR_STORE_SIZE
	}

	void sensorRangeRead(void)
	{
		uint16_t datai 	= 0x00;
		int8_t  status	=	-1,tmpi=0;
		uint8_t writeBuf[SENSOR_RANGE_STORE_SIZE+2];
		uint32_t dataWriteAddress = 0x00;
		uint32_t dataReadAddress = 0x00;//210705
		cigemsensorrange_t	datatmp[SENSORID_INDEX_NUM];
		
		dataWriteAddress = findFlashWriteAddr(SNESOR_RANGE_BASE_ADDR,SNESOR_RANGE_END_ADDR,SENSOR_RANGE_STORE_SIZE,writeBuf);	
		dataReadAddress = findFlashReadAddr(SNESOR_RANGE_BASE_ADDR,SNESOR_RANGE_END_ADDR,dataWriteAddress,SENSOR_RANGE_STORE_SIZE,writeBuf);

		if((dataReadAddress!=0)&&(writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
		{
			for(datai = 11;datai < SENSOR_RANGE_STORE_SIZE;datai ++)
			{
				if(writeBuf[datai] ==END_CHARCCTER )
				{
					if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
					{			
						buf2Struct(datatmp,DEV_SENSOR_RANGE_DATA,&writeBuf[10],0);						
						status = 0;
						for(tmpi=0;tmpi<SENSORID_INDEX_NUM;tmpi++)
						{
							cigemsensorlimit[tmpi] = datatmp[tmpi];
						}
						break;
					}
				}
			}
		}
		if(status!=0)
		{
			cigemSensorRangeInit();
		}			
	}	
	void cigemUnsendDataClear(void)
	{
		uint8_t tmpi=0;
		for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)
		{
			if(mqttServerAddrBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
			{			
				if(curMqttSendAddr[tmpi].i>SENSOR_DATA_BASE_ADDR)
				{
					curMqttSendAddr[tmpi].i = sensorDataWriteAddr - SENSOR_DATA_STORE_SIZE;
				}
				else
				{
					curMqttSendAddr[tmpi].i = SENSOR_DATA_BASE_ADDR;
				}
				unSendDataMqtt[tmpi] = 1;
			}
		}
	}
	void cigemServerAddrdeal(void)
	{
		uint8_t tmpi=0;	
	#if(SECOND_MQTT_ADDR>0)
		secondServerAddr secondServerBuf[SECOND_MULTI_CONNECTION_SIZE];	
	#endif
		mqttServerNum = 0;
	
		mqttAddressRead();
		if(devPar.stationMode == FUNCITONG_TURN_ON)
		{			
			for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)
			{
				if(mqttServerAddrBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
				{
					mqttServerNum++;				
				}
				else{curMqttSendAddr[tmpi].i = 0;}
			}
			if(mqttServerNum>0)
			{
				printf("\r\n serverNum:%d,",mqttServerNum);		
				mqttServerNumFirst = mqttServerNum;
			#if(SECOND_MQTT_ADDR>0)
				mqttServerNumSecond = mqttSecondAddressRead(secondServerBuf);
				printf("serverNumSecond:%d,",mqttServerNumSecond);
				if((mqttServerNumSecond == 1)||(mqttServerNumSecond == 2)||(mqttServerNumSecond == 3))
				{
					if(mqttServerNumFirst<3)
					{
						for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)
						{
							if(secondServerBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
							{
								mqttServerAddrBuf[mqttServerNum].userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
								memcpy(mqttServerAddrBuf[mqttServerNum].endpoint,secondServerBuf[tmpi].endpoint,SERVER_ENDPOINT_SIXE+1);
								mqttServerAddrBuf[mqttServerNum].port	=	secondServerBuf[tmpi].port;
								memcpy(mqttServerAddrBuf[mqttServerNum].deviceId,secondServerBuf[tmpi].clientId,SERVER_DEVICEID_SIZE+1);
								memcpy(mqttServerAddrBuf[mqttServerNum].username,secondServerBuf[tmpi].username,SERVER_USERNAME_SIZE+1);
								memcpy(mqttServerAddrBuf[mqttServerNum].password,secondServerBuf[tmpi].password,SERVER_PASSWORD_SIZE+1);
								mqttServerNum++;
							}
						}
					}
					else if((mqttServerNumSecond==1)||(mqttServerNumSecond==2))
					{
						for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)
						{
							if(secondServerBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
							{
								mqttServerAddrBuf[3].userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
								memcpy(mqttServerAddrBuf[3].endpoint,secondServerBuf[tmpi].endpoint,SERVER_ENDPOINT_SIXE+1);
								mqttServerAddrBuf[3].port	=	secondServerBuf[tmpi].port;
								memcpy(mqttServerAddrBuf[3].deviceId,secondServerBuf[tmpi].clientId,SERVER_DEVICEID_SIZE+1);
								memcpy(mqttServerAddrBuf[3].username,secondServerBuf[tmpi].username,SERVER_USERNAME_SIZE+1);
								memcpy(mqttServerAddrBuf[3].password,secondServerBuf[tmpi].password,SERVER_PASSWORD_SIZE+1);								
								mqttServerNumFirst = 3;
								mqttServerNum=4;
								break;
							}
						}
					}
					else
					{
						mqttServerNumFirst = 2;
						mqttServerNum = 2;
						for(tmpi=0;tmpi<SECOND_MULTI_CONNECTION_SIZE;tmpi++)
						{
							if(secondServerBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
							{
								mqttServerAddrBuf[mqttServerNum].userFlag = TCP_SERVER_ADDRESS_USE_FLAG;
								memcpy(mqttServerAddrBuf[mqttServerNum].endpoint,secondServerBuf[tmpi].endpoint,SERVER_ENDPOINT_SIXE+1);
								mqttServerAddrBuf[mqttServerNum].port	=	secondServerBuf[tmpi].port;
								memcpy(mqttServerAddrBuf[mqttServerNum].deviceId,secondServerBuf[tmpi].clientId,SERVER_DEVICEID_SIZE+1);
								memcpy(mqttServerAddrBuf[mqttServerNum].username,secondServerBuf[tmpi].username,SERVER_USERNAME_SIZE+1);
								memcpy(mqttServerAddrBuf[mqttServerNum].password,secondServerBuf[tmpi].password,SERVER_PASSWORD_SIZE+1);
								mqttServerNum++;
							}
						}
					}
				}
			#endif
				printf("\r\n insentek set");
				printf("serverNum:%d|",mqttServerNumFirst);
				for(tmpi=0;tmpi<mqttServerNum;tmpi++)
				{			
					if(tmpi<mqttServerNumFirst)
					{				
						printf("tcp,%s,",&mqttServerAddrBuf[tmpi].endpoint[1]);
						printf("%d,",mqttServerAddrBuf[tmpi].port);
						printf("%s,",&mqttServerAddrBuf[tmpi].deviceId[1]);
						printf("%s,",&mqttServerAddrBuf[tmpi].username[1]);
						printf("%s|",&mqttServerAddrBuf[tmpi].password[1]);
					}
					else
					{
						printf("\r\n   second set");
						printf("serverNum:%d,",tmpi+1);
						printf("endpoint:%s,",&mqttServerAddrBuf[tmpi].endpoint[1]);
						printf("port:%d,",mqttServerAddrBuf[tmpi].port);
						printf("deviceId:%s,",&mqttServerAddrBuf[tmpi].deviceId[1]);
						printf("username:%s,",&mqttServerAddrBuf[tmpi].username[1]);
						printf("password:%s,",&mqttServerAddrBuf[tmpi].password[1]);
					}				
				}
			}
		}
	}
#if(SECOND_MQTT_ADDR>0)	
	void cigemSendAddrAdj(uint8_t num_first,uint8_t num_last,uint8_t num_now)
	{
		uint32_t nowSendAddr=0;
		uint8_t tmpi=0;
		
		if(devPar.insentekKey == INSENTEK_KEY_ON)
		{
			nowSendAddr = curGprsSendAddr;
		}
		else
		{nowSendAddr = sensorDataWriteAddr;}
	
		if(num_now==0)//xx_00
		{
			for(tmpi=num_first;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)curMqttSendAddr[tmpi].i=0;			
		}
		else if(num_last==0)//00_01/10
		{			
			curMqttSendAddr[num_first].i=nowSendAddr;
		}
		else if(num_now == num_last)//00_01/10
		{			
			curMqttSendAddr[num_first].i=nowSendAddr;
		#if((SECOND_MQTT_ADDR>1)&&(SECOND_MULTI_CONNECTION_SIZE>1))
			if((num_now>2)&&((num_first+1)<MQTT_MAX_MULTI_CONNECTION_SIZE))
			{
				curMqttSendAddr[num_first+1].i=nowSendAddr;
			}
		#endif
		}
	#if((SECOND_MQTT_ADDR>1)&&(SECOND_MULTI_CONNECTION_SIZE>1))
		else if(num_now>num_last)//01_11 10_11
		{
			if(num_last == 1){curMqttSendAddr[num_first+1].i=nowSendAddr;}
			else if(num_last == 2)
			{
				curMqttSendAddr[num_first+1].i=	curMqttSendAddr[num_first].i;
				curMqttSendAddr[num_first].i	=	nowSendAddr;
			}
		}	
		else if(num_now<num_last)//11_01 11_10
		{
			if(num_now == 1){curMqttSendAddr[num_first+1].i=0;}
			else if(num_now == 2)
			{
				curMqttSendAddr[num_first].i		=	curMqttSendAddr[num_first+1].i;
				
				curMqttSendAddr[num_first+1].i	=	0;
			}
		}
		if((num_now>2)&&((num_first+1)<MQTT_MAX_MULTI_CONNECTION_SIZE))
		{
			if(curMqttSendAddr[num_first+1].i <SENSOR_DATA_BASE_ADDR)curMqttSendAddr[num_first+1].i = nowSendAddr;
		}
	#endif
		if((num_now>0)&&(curMqttSendAddr[num_first].i <SENSOR_DATA_BASE_ADDR))curMqttSendAddr[num_first].i = nowSendAddr;

//		printf("\r\n first:%d,last:%d,now:%d",num_first,num_last,num_now);
//		printf(" firstAddr:0x%X,nextAddr:0x%X ",curMqttSendAddr[num_first].i,curMqttSendAddr[num_first+1].i);
				
	}
	void cigemServerLinkClose(void)
	{
		uint16_t linkStatus1 = 0;
		linkStatus1 = gprsLinkStatusAll();
		if((linkStatus1&(1<<(mqttServerNumFirst+1)))>0)
		{
			closeSingleTcp(mqttServerNumFirst+1);
		}
	#if(SECOND_MQTT_ADDR>1)
		if((linkStatus1&(1<<(mqttServerNumFirst+2)))>0)
		{
			closeSingleTcp(mqttServerNumFirst+2);
		}
	#endif
	}
	void cigemServerAddrAdj(secondServerAddr * secondServerBuf)
	{
		uint8_t num_last = 0;
		num_last = mqttServerNumSecond;			
		power331On();
		delay_modeMs(10);
		SPI_Flash_WAKEUP();
		delay_modeMs(10);
		mqttSecondAddressStore(secondServerBuf);
		cigemServerAddrdeal();
		cigemSendAddrAdj(mqttServerNumFirst,num_last,mqttServerNumSecond);
		
		mqttSendAddrStore(DEV_SELF_SET_FLAG);
		delay_modeMs(10);
		SPI_Flash_PowerDown();
		power331Off();
		cigemServerLinkClose();
	}
#endif	
	void cigemParamInit(void)
	{
		uint8_t tmpi=0;
		mqttServerNum = 0;
		for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)
		{
			if(mqttServerAddrBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
			{
				mqttServerNum++;
				if(devPar.insentekKey == INSENTEK_KEY_ON)
				{
					curMqttSendAddr[tmpi].i = curGprsSendAddr;
					unSendDataMqtt[tmpi] = unSendPackNumAdj(curMqttSendAddr[tmpi].i)+1;
				}			
			}
		}
		if(mqttServerNum>0)
		{
			cigemSensorRangeInit();
			cigemConfigInit();
			if(devPar.insentekKey != INSENTEK_KEY_ON)cigemUnsendDataClear();
		}
	}
#endif
void devFlashErare(uint32_t startAddr,uint32_t endAddr)
{
	for(;startAddr <endAddr;)
	{
		SPI_FLASH_SectorErase(startAddr);                 
		startAddr += FLASH_SECTOR_SIZE;
	}
}


void devFlashInit(void)
{
	devFlashErare(DEV_SENSOR_COUNTER_BASE_ADDR,DEV_SENSOR_COUNTER_END_ADDR);
	devFlashErare(DEV_SN_BASE_ADDR,DEV_SN_END_ADDR);
}



void serAddrTest(void)
{
   uint32_t readAddress = 0x00;
   uint8_t readBuf[BYTE_32_PACK_SIZE];
   for(readAddress =DEV_SENSOR_COUNTER_BASE_ADDR;readAddress <DEV_GPRS_SERVER_END_ADDR;readAddress +=BYTE_128_PACK_SIZE)
   {
   	   SPI_FLASH_BufferRead(readBuf,readAddress,BYTE_128_PACK_SIZE);
	   usart1StoreBuffer(readBuf,BYTE_128_PACK_SIZE);
   }
}

void flashWriterTest(uint32_t baseAddress,uint32_t endAddress,uint16_t packSize)
{
    uint32_t readAddress = 0x00;
	uint8_t readBuf[BYTE_512_PACK_SIZE];
	readAddress = baseAddress;
   for(readAddress =baseAddress;readAddress <endAddress ;readAddress +=packSize)
   {
   	   SPI_FLASH_BufferRead(readBuf,readAddress,packSize);
//	   printf("\r\nflash address :%d:",readAddress); 	 
	   usart2StoreBuffer(readBuf,packSize);
	   printf("\r\n"); 
   }
}

#define DEV_SN_DEBUG 1
#if(DEV_APN_SET_CTR>0) //23037
static void gprsApnRead(void);
#endif
void devFlashInfRead(void)
{
#if(MQTT_USE_CTR>0)
	uint8_t tmpi=0;
#endif

	DS1302_TIME readTime;
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();

//	devFawRead();
	devCalCounterRead(&readTime);
	if((rtcSaveFlag == RTC_SAVE_NONE)&&(calAllCounter.i>0))
	{
		if(rtcCompare(readTime,rtcTime) == RTCOK)
		{			
			rtcTime	=	readTime;	
			setTimeTask(readTime);
		}
	}
	rtcSaveFlag = RTC_SAVE_NONE;	
	devSnRead();
	devAuthCodeRead();
#if(DEV_APN_SET_CTR>0) //230508
	gprsApnRead();//230508	
#endif
	gpsDataWriteAddr 		= devGpsFlashWriteAddressFind();
	if(devGpsValueFlashRead(gpsDataWriteAddr,&gpsBuf[0])==0)//gpsBuf[1] gpsBuf[0]_220525
	{	
	#if(GPRS_MODE_TYPE==GPRS_MODE_SIM7100)		
		if(gpsBuf[0].gpsSource != PLATFORM_SOURCE)
		{			
			devLatitude		= gpsValueAdj(gpsBuf[0].latitude.f);
			devLongitude 	= gpsValueAdj(gpsBuf[0].longitude.f);
		}
		else
	#endif
		{
			devLongitude = gpsBuf[0].longitude.f;
			devLatitude  = gpsBuf[0].latitude.f;
		}
	#if GPRS_DEBUG > 0
//		printf("\n\r devLongitude:%f, devLatitude:%f\n\r",devLongitude,devLatitude);
	#endif
	}
	
	xyzDataWriteAddr 		=	devXyzFlashWriteAddressFind();
	sensorDataWriteAddrRead();
	if((sensorDataWriteAddr<SENSOR_DATA_BASE_ADDR)||(sensorDataWriteAddr>SENSOR_DATA_END_ADDR))
	{
		sensorDataWriteAddr = findDataWriteAddr();
	}
	printf("\r\n writeAddr��0x%X ",sensorDataWriteAddr);
//	curGprsSendAddr	= sensorDataWriteAddr;			
	gprsSendAddrRead();	
	printf(",sendAddr��0x%X\r\n",curGprsSendAddr);	
	if((sensorDataWriteAddr<SENSOR_DATA_BASE_ADDR)||(sensorDataWriteAddr>=SENSOR_DATA_END_ADDR))
	{
//		sensorDataWriteAddr	=	SENSOR_DATA_BASE_ADDR;
		curGprsSendAddr			=	sensorDataWriteAddr;
	}
	unSendDataPacket = unSendPackNumAdj(curGprsSendAddr);
//	if(unSendDataPacket>5000)//2000 5000_210705 220525_����
//	{
//		unSendDataPacket 	= 0;
//		curGprsSendAddr		=	sensorDataWriteAddr;
//	}
//	if(readNum(SEND_DATA_FLAG_ADDR)==REG_STATE)
//	{
//		serialNumWrite(SEND_DATA_FLAG_ADDR,0);
//		devFlashErare(DEV_RUN_STATUS_BASE_ADDR,DEV_RUN_STATUS_END_ADDR);
//	}
	(void)devConfigValueFlashRead(&configValue);
	devParamterRead(&devPar);
//	gprsAddressRead();
	devabcRead();
	devDepRead();
	devFawRead();
	
#if(MQTT_USE_CTR>0)
	cigemServerAddrdeal();
	mqttSendAddrRead();
	if(devPar.stationMode == FUNCITONG_TURN_ON)
	{			
		for(tmpi=0;tmpi<MQTT_MAX_MULTI_CONNECTION_SIZE;tmpi++)
		{
			if(mqttServerAddrBuf[tmpi].userFlag == TCP_SERVER_ADDRESS_USE_FLAG)
			{				
				if(curMqttSendAddr[tmpi].i<SENSOR_DATA_BASE_ADDR)curMqttSendAddr[tmpi].i = curGprsSendAddr;
				unSendDataMqtt[tmpi] = unSendPackNumAdj(curMqttSendAddr[tmpi].i);
				if(unSendDataMqtt[tmpi]>2000)
				{
					unSendDataMqtt[tmpi] = 0;
					curMqttSendAddr[tmpi].i	= sensorDataWriteAddr;
				}
			#if DEV_SN_DEBUG > 0
				printf("\r\n serverNum:%d,unsendPack:%d,",tmpi+1,unSendDataMqtt[tmpi]);
				printf("curSendAddr:0x%X,",curMqttSendAddr[tmpi].i);					
			#endif
			}
			else{curMqttSendAddr[tmpi].i = 0;}
		}
		if(mqttServerNum>0)
		{	
			sensorRangeRead();
			cigemConfigInit();
		}
	}	
 #if DEV_SN_DEBUG > 0
	printf("\r\n mqttServerNum:%d",mqttServerNum);
 #endif	
#endif	
#if DEV_SN_DEBUG > 0
	printf("\r\n unSendPacket_save:%d",unSendDataPacket);	
	printf("\r\n flash size:%d,sensor data baseAddr:%d,",FLASH_END_ADDR/FLASH_SECTOR_SIZE,SENSOR_DATA_BASE_ADDR/FLASH_SECTOR_SIZE);
	printf("\r\n writeAddr��0x%X\r\n",sensorDataWriteAddr);
#endif	

	SPI_Flash_PowerDown();
	power331Off();
}
void recTask(uint8_t *buf)
{
	int32_t packLength =0;
	int32_t recStatus =0;
	uint32_t taskCounter = 0x00;
//	uint32_t  sensorDataReadAddr = 0x00;
	uint32_t fireWareAddr = 0x00;
	int16_t lingkStatus =0;
//	uint16_t flashReadCounter = 0x00;
	uint16_t tiemOut = 0;
	uint16_t taski = 0;
	uint16_t errorCounter = 0;
	uint8_t dataOrder = 0x00;
	uint8_t devLinkStatus = UNLINK_STATUS;
	uint8_t breakFlag = 0x00;
	uint8_t linkTry = 0x00;
	uint8_t frameOrder = 0x00;
	uint8_t taskExitFlag  = TASK_RUN_FLAG;
	
	recTaskState = CALL_MASTER_STATE;
//	sensorDataReadAddr	=  sensorDataWriteAddr;
	fireWareAddr = 	DEV_FIREWARE_BASE_ADDR;
	power331On();//�򿪴洢FLASH��Դ
	SPI_Flash_WAKEUP();
	while(taskExitFlag)
	{
		switch(recTaskState)
		{
			case CALL_MASTER_STATE :
			{
				if(linkTry <MAX_LINK_TRY_NUM)
				{
				 devLinkStatus = UNLINK_STATUS;
					//������������
				linkMaster(dataOrder,SEC_LINK_MASTER);
				linkTry++;
				dataOrder++;
				if(dataOrder == 0xFF)
				{
					dataOrder = 0x00;
				}
				}
				else
				{
					taskExitFlag  = TASK_EXIT_FLAG;
					recTaskState = EXIT_STATE;  
					break;
				}
			}
			case REC_STATE :
			{	
				packLength = 0x00;
				for(taski = 0x00;taski < MAX_RS232_BUF_SIZE;taski ++)
				{
					buf[taski] = 0x00;		
				}
				recStatus = Receive_Packet(buf, &packLength, MODE_TIMEOUT);		
				switch(recStatus)
				{
					case 0 :
					{
						taskCounter =0;
						if(devLinkStatus == UNLINK_STATUS) //�豸δ����״̬
						{	//�ж��豸�����ַ�
								lingkStatus = judgeAck(buf);
								if(lingkStatus == SEC_ACK_OK_COMM)
								{
									devLinkStatus = LINK_STATUS;
									recTaskState = REC_STATE; 
									breakFlag ++;
									linkTry = 0x00;
									//���ͻ�Ӧ�ַ�
									//	recAckSend(SEC_ACK_OK_COMM,dataOrder);
									dataOrder++;
									if(dataOrder == 0xFF)
									{
										dataOrder = 0x00;
									}
								}
							}
							else	 //�豸����״̬
							{
								breakFlag = 0x00;    
								recTaskState = DEAL_STATE;//
							}
							tiemOut = 0;
							errorCounter = 0;
							break;
						}
					case -1 :
					{
						tiemOut ++;
						breakFlag ++;
						/*
						if(recTaskState == CALL_MASTER_STATE)
						{
							if(tiemOut >MAX_TIMEOUT_COUNTER)
							{
								recTaskState = EXIT_STATE;  
							}
						}*///

						taskCounter++;
	//				printf("��ʱ%d\n\r",taskCounter);
						if(taskCounter > MAX_RS232_TASK_COUNTER)
						{
						//taskCounter = 0x00;
							taskExitFlag  = TASK_EXIT_FLAG;				     
						}
						//���ͻ�Ӧ�ַ�
						recAckSend(SEC_TIMEOUT_COMM,dataOrder);
						dataOrder++;
						if(dataOrder == 0xFF)
						{
							dataOrder = 0x00;
						}			   //
						break;
					}
					case 2 :
					{
						errorCounter ++;
						tiemOut = 0;
						breakFlag ++;
						if(errorCounter > MAX_ERROR_COUNTER)
						{
						//	recTaskState = EXIT_STATE;  			       
						}
						//���ͻ�Ӧ�ַ�
						recAckSend(SEC_HEAD_ERROR_COMM,dataOrder);
						dataOrder++;
						if(dataOrder == 0xFF)
						{
							dataOrder = 0x00;
						}
						break;
					}
					case 3 :
					{
						errorCounter ++;
						breakFlag ++;
						tiemOut = 0;
						if(errorCounter > MAX_ERROR_COUNTER)
						{
						//	recTaskState = EXIT_STATE;  			       
						}
						//���ͻ�Ӧ�ַ�
						recAckSend(SEC_LENGTH_COMM,dataOrder);
						dataOrder++;
						if(dataOrder == 0xFF)
						{
							dataOrder = 0x00;
						}	
						break;
					}
					case 4 :
					{
						errorCounter ++;
						breakFlag ++;
						tiemOut = 0;
						if(errorCounter > MAX_ERROR_COUNTER)
						{
						//	recTaskState = EXIT_STATE; 			       
						}
						//���ͻ�Ӧ�ַ�
						recAckSend(SEC_CRC_ERROR_COMM,dataOrder);
						dataOrder++;
						if(dataOrder == 0xFF)
						{
							dataOrder = 0x00;
						}	
						break;
					}														
				}
				if(breakFlag > 0x00)
				{
					breakFlag = 0x00;
					break;
				}
			}
			case DEAL_STATE :
			{
				switch(buf[FIR_COMMAND_ADDRESS])
				{
					case FIR_DEV_SET_COMMAND :
					{
						recTaskState = REC_STATE; 
						breakFlag ++;
						switch(buf[SEC_COMMAND_ADDRESS])
						{
							case SEC_DEV_ID_SET_COMM :
							{
								rs232DevSet(&buf[DEAL_DATA_ADDRESS]);
								break;
							}
							case SEC_DEV_TIME_SET_COMM  :
							{
								rtcTimeRs232Set(&buf[DEAL_DATA_ADDRESS]);
								break;
							}
							case SEC_DEV_COMMUNICT_SET_COMM :
							{
								devCommunictionSet(&buf[DEAL_DATA_ADDRESS]);
								break;
							}
							case SEC_DEV_ABC_SET_COMM  :
							{
								devAbcSet(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_AW_SET_COMM :
							{
								devfawSet(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_DEP_SET_COMM :
							{
								devDepSet(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_CTR_PHONE_SET_COMM :
							{
							//	devPhoneSet(&buf[DEAL_DATA_ADDRESS],&ctrNum[0],SEC_DEV_CTR_PHONE_SET_COMM);
								break;
							}
							case SEC_DEV_SMS_PHONE_SET_COMM :
							{
							//	devPhoneSet(&buf[DEAL_DATA_ADDRESS],&smsNum[0],SEC_DEV_SMS_PHONE_SET_COMM);
								break;
							}
							case SEC_DEV_PERIOD_SET_COMM :
							{
							// devPerWrite(&buf[DEAL_DATA_ADDRESS]);
								break;
							}
							case SEC_DEV_GPRS_ADDR_SET_COMM :
							{
							//gprsAddressSet(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_AUTHCODE_SET_COMM :
							{
								devAuthCodeWrite(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_SN_SET_COMM :
							{
								devSnAndMainFastWrite(&buf[DEAL_DATA_ADDRESS],(packLength-4));
								break;
							}
							case SEC_DEV_FIRWARE_SET_COMM :
							{ //���ݰ�����
								if(frameOrder != buf[FRAME_ORDER_ADDRESS])
								{
								//����ͨ�������ݴ洢
									firewareWrite(&buf[DEAL_DATA_ADDRESS],&fireWareAddr,(packLength-4));
								}				   
								break;
							}
							case SEC_DEV_FIRWARE_SET_OK_COMM :
							{
							//���ù̼����±�־λ
							//����ͨ�����ӶϿ���־
							//��λϵͳ
								break;
							}  
							case SEC_DEV_STATUS_SET_COMM :
							{
								devStatusSet(&buf[DEAL_DATA_ADDRESS]);
								break;
							}	
							case SEC_DEV_STATUS_INIT :
							{
								devFlashInit();
								eepromInint();
								break;
							}				   			   				   				   				   				   				   				
						}
						break;
					}
					case FIR_DEV_READ_COMM :
					{
						recTaskState = REC_STATE; 
						breakFlag ++;
						switch(buf[SEC_COMMAND_ADDRESS])
						{  
							case SEC_DEV_DATA_READ_COMM :
							{
							 //		 
//								flashReadCounter = buf[FIREST_USER_DATA_ADDRESS]+	buf[SECOND_USER_DATA_ADDRESS]*256;
//								sensorDataRead(&sensorDataReadAddr,flashReadCounter);
								break;
							}				
						}
						break;
					}
					case FIR_LINK_COMM :
					{
						recTaskState = REC_STATE; 
						breakFlag ++;
						switch(buf[SEC_COMMAND_ADDRESS])
						{
							case SEC_UNLINK_MASTER :
							{
								taskExitFlag  = TASK_EXIT_FLAG;				     				   
								break;
							} 
						}
						break;
					}
				}	     
				//���ͻ�Ӧ�ַ�
				recAckSend(SEC_ACK_OK_COMM,dataOrder);
				dataOrder++;
				if(dataOrder == 0xFF)
				{
					dataOrder = 0x00;
				}
				if(breakFlag >= 0x01)
				{
					breakFlag = 0x00;
					break;
				}	
				frameOrder = buf[FRAME_ORDER_ADDRESS];	
			}	
			case EXIT_STATE :
			{	     
				break;
			}	
		}
	}
	SPI_Flash_PowerDown();
	power331Off();
}

uint16_t sensorStoreDataForanmt(uint8_t *buf,uint8_t dataType)
{
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
	uint16_t crc16 = 0;
	uint16_t crcLength = 0;
	value_t tempId ;

  //�洢����ͷ��
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;

//��������װ��	
	buf[bufLength++]= dataType;
//�ɼ�ʱ������װ��
	buf[bufLength++]= rtcTime.year;
	buf[bufLength++]= rtcTime.month;
	buf[bufLength++]= rtcTime.date;
	buf[bufLength++]= rtcTime.hour;
	buf[bufLength++]= rtcTime.min;
	if(rtcTime.sec<6)rtcTime.sec=0;
	buf[bufLength++]= rtcTime.sec;

//�豸�ܵĲɼ�����װ��
	buf[bufLength++] = calAllCounter.bytes. value1;
	buf[bufLength++] = calAllCounter.bytes. value2;
	buf[bufLength++] = calAllCounter.bytes. value3;
	buf[bufLength++] = calAllCounter.bytes. value4;

//�ɼ�����װ��
	tempId.j = calCounter;
	buf[bufLength++] = tempId.bytes. value1;
	buf[bufLength++] = tempId.bytes. value2;
//��ص�ѹ
	tempId.f = batStatus;//*6.144/ADS_MAXVALUE;
	//��ص�ѹ��Ϣװ��
	buf[bufLength++] = tempId.bytes.value1;
	buf[bufLength++] = tempId.bytes.value2;
	buf[bufLength++] = tempId.bytes.value3;
	buf[bufLength++] = tempId.bytes.value4;
//﮵�س�������ѹ
/*   	 tempId.f = adschargeValue*6.144/ADS_MAXVALUE;
	//��ص�ѹ��Ϣװ��
   	buf[bufLength++] = tempId.bytes.value1;
	buf[bufLength++] = tempId.bytes.value2;
   	buf[bufLength++] = tempId.bytes.value3;
   	buf[bufLength++] = tempId.bytes.value4;
   	*/
//�ⲿ�����ѹ
	tempId.f = adsexterValue;//*6.144/ADS_MAXVALUE;
	//��ص�ѹ��Ϣװ��
	buf[bufLength++] = tempId.bytes.value1;
	buf[bufLength++] = tempId.bytes.value2;
	buf[bufLength++] = tempId.bytes.value3;
	buf[bufLength++] = tempId.bytes.value4;
//�ر��¶�װ��
	if(topSensorFlag == TOP_SENSOR_USE)
	{
		buf[bufLength++] = result[0].value_t.bytes. valueH;
		buf[bufLength++] = result[0].value_t.bytes. valueL;
	}
	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
//�¶�����װ��
		buf[bufLength++] = result[nodei+1].value_t.bytes. valueH;
		buf[bufLength++] = result[nodei+1].value_t.bytes. valueL;		  
//��Ƶˮ������װ��
		buf[bufLength++] = curWat[nodei].fh.bytes.value1;
		buf[bufLength++] = curWat[nodei].fh.bytes.value2;
		buf[bufLength++] = curWat[nodei].fh.bytes.value3;
		buf[bufLength++] = curWat[nodei].fh.bytes.value4;
//��Ƶ�η�����װ��
		if(eachDevNum == 0x02)
		{		  
			buf[bufLength++] = curWat[nodei].fl.bytes. value1;
			buf[bufLength++] = curWat[nodei].fl.bytes. value2;
			buf[bufLength++] = curWat[nodei].fl.bytes. value3;
			buf[bufLength++] = curWat[nodei].fl.bytes. value4;
		}	   
	}
	//����CRC16У�� 
	crcLength = bufLength -3;
	crc16 = CRC16_1(&buf[3],crcLength); 
	//װ��CRC16����
	buf[bufLength++] = (crc16&0xFF00)>>8;
	buf[bufLength++] = (crc16&0x00FF);   
  
	for(nodei = 0x03;nodei < bufLength;nodei ++)
	{
		crc += buf[nodei];
	}    
	buf[bufLength++] = crc; 
     
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	return bufLength;//32+10*10=132 
}

uint16_t gpsValueFormat(double longitude,double latitude,uint8_t saveMode,uint8_t *buf)
{
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
	dou2Buf temp ;
    //�洢����ͷ��
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;

//�ɼ�ʱ������װ��
	buf[bufLength++]= rtcTime.year;
	buf[bufLength++]= rtcTime.month;
	buf[bufLength++]= rtcTime.date;
	buf[bufLength++]= rtcTime.hour;
	buf[bufLength++]= rtcTime.min;
	buf[bufLength++]= rtcTime.sec;

//		  
	temp.f = longitude;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
	buf[bufLength++] = temp.bytes.value5; 
	buf[bufLength++]= temp.bytes.value6; 
	buf[bufLength++] = temp.bytes.value7; 
	buf[bufLength++] = temp.bytes.value8; 

	temp.f = latitude;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
	buf[bufLength++] = temp.bytes.value5; 
	buf[bufLength++]= temp.bytes.value6; 
	buf[bufLength++] = temp.bytes.value7; 
	buf[bufLength++] = temp.bytes.value8; 
//
	buf[bufLength++] = saveMode; 

	for(nodei = 0x03;nodei < bufLength;nodei ++)
	{
		crc += buf[nodei];
	}

	buf[bufLength++] = crc;

	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++]= END_CHARCCTER;
	buf[bufLength++]= END_CHARCCTER;
	return bufLength;
}

void gpsValueStoreWithSensorData(uint32_t address,gpsValue value)
{
	uint8_t  buf[BYTE_32_PACK_SIZE];
	uint16_t bufLength = 0x00;
	uint32_t writeAddress = 0x00;	

	//���ݴ洢
	bufLength = gpsValueFormat(value.longitude.f,value.latitude.f,value.saveMode,buf);
	//�洢��ַ�任
	writeAddress = address -   SENSOR_DATA_STORE_SIZE+ GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;

	SPI_FLASH_BufferWrite(buf,writeAddress,bufLength);
}   



uint16_t xyzValueFormat(float x,float y,float z,uint8_t *buf,DS1302_TIME time)
{
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
	value_t temp ;


//�洢����ͷ��
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;

//�ɼ�ʱ������װ��
	buf[bufLength++]= time.year;
	buf[bufLength++]= time.month;
	buf[bufLength++]= time.date;
	buf[bufLength++]= time.hour;
	buf[bufLength++]= time.min;
	buf[bufLength++]= time.sec;
//
	temp.f = x;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
//
	temp.f = y;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
//
	temp.f = z;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
	for(nodei = 0x03;nodei < bufLength;nodei ++)
	{
		crc += buf[nodei];
	}

	buf[bufLength++] = crc;

	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	return bufLength;
}

void xyzValueStoreWithSensorData(uint32_t address,tg_ADXL345_TYPE value)
{
	uint8_t  buf[BYTE_32_PACK_SIZE];
	uint16_t bufLength = 0x00;
	uint32_t writeAddress = 0x00;	
	bufLength = xyzValueFormat(value.xg,value.yg,value.zg,buf,rtcTime);

	//���ݴ洢

	//�洢��ַ�任
	writeAddress = address -   SENSOR_DATA_STORE_SIZE+ XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	//дFLASH
	SPI_FLASH_BufferWrite(buf,writeAddress,bufLength);
} 

void xyzValueStoreFlash(uint32_t *addr,tg_ADXL345_TYPE value)
{
	uint8_t  buf[BYTE_32_PACK_SIZE+2];
	uint16_t bufLength = 0x00;
	DS1302_TIME writeTime ;
	readDs1302Time(&writeTime);
	bufLength = xyzValueFormat(value.ax,value.ay,value.az,buf,writeTime);
	FlashWrite( DEV_XYZ_BASE_ADDR,DEV_XYZ_BASE_ADDR,addr,buf,bufLength,BYTE_32_PACK_SIZE);
}

int8_t xyzValueReadFlash(uint32_t addr,DS1302_TIME *readTime,tg_ADXL345_TYPE *value)
{
	int8_t readResult = -1;
	uint8_t crcdata = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint16_t bufLength = 0x00;
	uint8_t writeBuf[BYTE_32_PACK_SIZE+2];
	value_t temp ;
	SPI_FLASH_BufferRead(writeBuf, addr,BYTE_32_PACK_SIZE);
	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < BYTE_32_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					readResult = -2;
					for(dataj = 0x00;dataj < (datai-1);dataj ++)
					{
						crcdata += writeBuf[dataj];
					}
					if(crcdata == writeBuf[datai-1])	
					{
						readResult =0;
						//װ������
						//�ɼ�ʱ������װ��
						bufLength = 0x03;
						readTime->year   = writeBuf[bufLength++];
						readTime->month = writeBuf[bufLength++];
						readTime->date   = writeBuf[bufLength++];
						readTime->hour = writeBuf[bufLength++];
						readTime->min = writeBuf[bufLength++];
						readTime->sec = writeBuf[bufLength++];
						//
						temp.bytes.value1= writeBuf[bufLength++];    
						temp.bytes.value2= writeBuf[bufLength++];
						temp.bytes.value3= writeBuf[bufLength++];
						temp.bytes.value4= writeBuf[bufLength++];
						value->ax =    temp.f;
						//
						temp.bytes.value1= writeBuf[bufLength++];    
						temp.bytes.value2= writeBuf[bufLength++];
						temp.bytes.value3= writeBuf[bufLength++];
						temp.bytes.value4= writeBuf[bufLength++];
						value->ay =    temp.f;
						//
						temp.bytes.value1= writeBuf[bufLength++];    
						temp.bytes.value2= writeBuf[bufLength++];
						temp.bytes.value3= writeBuf[bufLength++];
						temp.bytes.value4= writeBuf[bufLength++];
						value->az =    temp.f;
					}
					break;
				}
			}
		}
	}
	return readResult;   	
}

uint16_t qxValueFormat(uint8_t *bufIn,uint8_t *buf,DS1302_TIME time)
{
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
#if(QX_CTR>1)		
	value_t temp ;
#endif
	//�洢����ͷ��
	buf[bufLength++]= 0x2c;  //0
	buf[bufLength++]= 0x2c;  //1
	buf[bufLength++]= 0x2c;  //2

	//�ɼ�ʱ������װ�� 
	buf[bufLength++]= time.year; 
	buf[bufLength++]= time.month;
	buf[bufLength++]= time.date; 
	buf[bufLength++]= time.hour; 
	buf[bufLength++]= time.min;  
	buf[bufLength++]= time.sec;  

	for(nodei = 0x00;nodei < 96;nodei++)
	{
		buf[bufLength++] = bufIn[nodei];
	}
	for(nodei = 0x03; nodei < bufLength; nodei++)
	{
		crc += buf[nodei];
	}

	buf[bufLength++] = crc;

	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
#if(QX_CTR>1)					
	if(tiltTypeCheck(bufIn)==0)
	{
		temp.f = sensorValue.dipZ.f;
		buf[bufLength++] = temp.bytes.value1;      
		buf[bufLength++] = temp.bytes.value2; 
		buf[bufLength++] = temp.bytes.value3; 
		buf[bufLength++] = temp.bytes.value4; 
		temp.f = sensorValue.angle.f;
		buf[bufLength++] = temp.bytes.value1;      
		buf[bufLength++] = temp.bytes.value2; 
		buf[bufLength++] = temp.bytes.value3; 
		buf[bufLength++] = temp.bytes.value4; 
	}
#endif
	return bufLength;
}



void little_end_to_big_end(uint8_t *data, uint8_t len)
{
	uint8_t i, temp;

	for(i = 0; i < len/2; i++)
	{
		temp = data[i];
		data[i] = data[len-1-i];
		data[len-1-i] = temp;
	}
}
void qxValuePrint(uint8_t *source)
{
	uint16_t i;
	for(i=0;i<20;i++)	sensorValue.sign[i]			=	source[i];
	for(i=0;i<4;i++)	sensorValue.qxt.buf[i]	=	source[i+20];//20~23
	for(i=0;i<8;i++)	sensorValue.dipX.buf[i]	=	source[i+24];//24~31
	for(i=0;i<8;i++)	sensorValue.valX.buf[i]	=	source[i+32];
	for(i=0;i<8;i++)	sensorValue.origX.buf[i]	=	source[i+40];
	for(i=0;i<8;i++)	sensorValue.dipY.buf[i]		=	source[i+48];//48~55
	for(i=0;i<8;i++)	sensorValue.valY.buf[i]		=	source[i+56];
	for(i=0;i<8;i++)	sensorValue.origY.buf[i]	=	source[i+64];
	for(i=0;i<8;i++)	sensorValue.dipYaw.buf[i]	=	source[i+72];//72~79
	for(i=0;i<8;i++)	sensorValue.valYaw.buf[i]	=	source[i+80];
	for(i=0;i<8;i++)	sensorValue.origYaw.buf[i]=	source[i+88];
	sensorValue.angle.f = qxAngleCaculate(sensorValue.dipX.f,sensorValue.dipY.f);
	if(xlResult[32].ay<0) sensorValue.angle.f *= -1.0;//20210315
	sensorValue.dipZ.f 	= qxZCaculate(sensorValue.angle.f);
	printf(" X_angle:%lf,Y_angle:%lf,Z_angle:%lf",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);
	printf(",angle:%f,Yaw_angle:%f,t:%f",sensorValue.angle.f,sensorValue.dipYaw.f,sensorValue.qxt.f);
	printf("\r\n X_std:%lf,X_stdev:%lf,Y_std:%lf,",sensorValue.valX.f,sensorValue.origX.f,sensorValue.valY.f);
	printf("Y_stdev:%lf,Yaw_std:%lf,Yaw_stdev:%lf \r\n",sensorValue.origY.f,sensorValue.valYaw.f,sensorValue.origYaw.f);
}

void qxValueStoreWithSensorData(uint32_t address,DS1302_TIME readTime)//,uint8_t *source
{
   	uint32_t writeAddress = 0x00;
    uint16_t length = 0x00;
		uint16_t i;
    uint8_t dataBuf[BYTE_128_PACK_SIZE];
		uint8_t source[BYTE_128_PACK_SIZE];
		for(i=0;i<20;i++)	source[i] = sensorValue.sign[i];
		for(i=0;i<4;i++)	source[i+20] = sensorValue.qxt.buf[i];//20~23
		for(i=0;i<8;i++)	source[i+24] = sensorValue.dipX.buf[i];//24~31
		for(i=0;i<8;i++)	source[i+32] = sensorValue.valX.buf[i];
		for(i=0;i<8;i++)	source[i+40] = sensorValue.origX.buf[i];
		for(i=0;i<8;i++)	source[i+48] = sensorValue.dipY.buf[i];//48~55
		for(i=0;i<8;i++)	source[i+56] = sensorValue.valY.buf[i];
		for(i=0;i<8;i++)	source[i+64] = sensorValue.origY.buf[i];
		for(i=0;i<8;i++)	source[i+72] = sensorValue.dipYaw.buf[i];//72~79
		for(i=0;i<8;i++)	source[i+80] = sensorValue.valYaw.buf[i];
		for(i=0;i<8;i++)	source[i+88] = sensorValue.origYaw.buf[i];
	#if(QX_DEBUG>0)
		printf("ԭʼ��б���ݣ�\r\n");
		for(i = 0; i < 96; i++){
			if(i % 8 == 0)
				printf("\r\n");
			printf("%02x ", source[i]);
		}
		printf("\r\n");
		printf("ԭʼС�����ݣ�\r\n");
		for(i = 24; i < 96; i++){
				if(i > 24 && i % 8 == 0)
						printf("\r\n");
				printf("%02x ", source[i]);
		}
		printf("\r\n");
	#endif
		//��боƬ�ڲ��¶�С��ת���   
		little_end_to_big_end(&source[20], 4);
    
		//��б����С��ת��� 
		for(i = 24; i < 96; i += 8)
				little_end_to_big_end(&source[i], 8);
	#if(QX_DEBUG>0)	
		printf("ת��������ݣ�\r\n"); 
		for(i = 24; i < 96; i++){
				if(i > 24 && i % 8 == 0)
						printf("\r\n");
				printf("%02x ", source[i]);
		}
		printf("\r\n");
		
		printf("�洢��б����Ϊ��\r\n");
		for(i = 0; i < 96; i++){
			if(i % 8 == 0)
				printf("\r\n");
			printf("%02x ", source[i]);
		}
		printf("\r\n");
	#endif
    length = qxValueFormat(source,dataBuf,readTime);
    writeAddress = address - SENSOR_DATA_STORE_SIZE + QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST;
    SPI_FLASH_BufferWrite(dataBuf,writeAddress,length);
}

int8_t qxValueReadFlash(uint32_t addr,DS1302_TIME *readTime,uint8_t *readBuf)
{
	int8_t readResult = -1;
	uint8_t crcdata = 0x00;
	uint16_t datai = 0x00;
	uint16_t dataj = 0x00;
	uint16_t bufLength = 0x00;
	uint8_t writeBuf[BYTE_128_PACK_SIZE+2];
	// value_t temp ;
	SPI_FLASH_BufferRead(writeBuf, addr,BYTE_128_PACK_SIZE);
	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < BYTE_128_PACK_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					readResult = -2;
					for(dataj = 0x00;dataj < (datai-1);dataj ++)
					{
						crcdata += writeBuf[dataj];
					}
					if(crcdata == writeBuf[datai-1])	
					{
						readResult =0;
						//װ������
						//�ɼ�ʱ������װ��
						bufLength = 0x03;
						readTime->year	= writeBuf[bufLength++];
						readTime->month = writeBuf[bufLength++];
						readTime->date	= writeBuf[bufLength++];
						readTime->hour 	= writeBuf[bufLength++];
						readTime->min 	= writeBuf[bufLength++];
						readTime->sec 	= writeBuf[bufLength++];
						//  
						for(datai = 0x00;datai < 96;datai ++)
						{
							readBuf[datai] = writeBuf[bufLength++];                  
						}
					}
					break;
				}
			}
		}
	}
	return readResult;
}

int8_t  qxDataReadWithSensorFlash(uint32_t readAddress,uint8_t *readBuf)
{
	uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	//	value_t tempId ;

	uint8_t buf[BYTE_128_PACK_SIZE+1];
	//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
	bufLength = 0x09;
	readAddress += QX_VALUE_STORE_WITH_SENSOR_DATA_OFFST;
	//��ȡ����������
	readResult = flashStoreDataRead(readAddress,BYTE_128_PACK_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		for(datai = 0x00;datai < 96;datai ++)
		{
			readBuf[datai] = buf[bufLength++];                  
		}	
	}
	return 	readResult;
}
int8_t sensorDataWrite(__IO uint32_t *address,uint8_t dataType)
{
	uint16_t length = 0x00;
	uint8_t dataBuf[SENSOR_DATA_STORE_SIZE+1]; 
	
	int8_t readResult = -1;
	uint16_t crc16 = 0;
	uint16_t bufCrc16 = 0;
	uint16_t crc16Length = 0;
	uint32_t readAddress;	
	uint8_t nodei;
		
//	length = sensorStoreDataForanmt(dataBuf,dataType);	//221013
	for(nodei=0;nodei<2;nodei++)
	{	
		length = sensorStoreDataForanmt(dataBuf,dataType);	//221013
		readAddress	=	*address;
		FlashWrite(SENSOR_DATA_BASE_ADDR,SENSOR_DATA_END_ADDR,address,dataBuf,length,SENSOR_DATA_STORE_SIZE);
		delay_modeMs(10);	
		readResult = flashStoreDataRead(readAddress,SENSOR_DATA_STORE_SIZE,dataBuf,&length);
		if(readResult == 1)
		{
			readResult 	= -1;  
			crc16Length = length -5;
			crc16 = CRC16_1(&dataBuf[3],crc16Length);
			bufCrc16 =(((uint16_t)(dataBuf[length-2]<<8))&0xFF00) + dataBuf[length-1];

			if(crc16 == bufCrc16)
			{
				readResult = 1;	
				break;
			}
		}
	}
	return readResult;
}
 
#define SENSOR_DATA_FIND     0x01
#define XYZ_DATA_FIND        0x02
#define GPS_DATA_FIND        0x04

int8_t  sensorDataReadFromFlash(uint32_t readAddress,DS1302_TIME *time,value_t *allConuter,uint16_t *conuter,b20_value_t *b20Buf,soilwat *hlfBuf)
{
//    uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	value_t tempId ;

	uint8_t buf[SENSOR_DATA_STORE_SIZE+1];
	uint8_t nodei = 0x00;
	int8_t result = -1;
	int8_t readResult = -1;
	bufLength = 0x03;
	//��ȡ����������
	readResult = flashStoreDataRead(readAddress,SENSOR_DATA_STORE_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		//�ɼ�ʱ������װ��
		time->year  = buf[bufLength++];
		time->month = buf[bufLength++];
		time->date  = buf[bufLength++];
		time->hour  = buf[bufLength++];
		time->min   = buf[bufLength++];
		time->sec   = buf[bufLength++];
//	 if(rtcCheck(*time)==RTCOK)
	{
		//�豸�ܵĲɼ�����װ��
		allConuter->bytes.value1 = buf[bufLength++];
		allConuter->bytes.value2 = buf[bufLength++];
		allConuter->bytes.value3 = buf[bufLength++];
		allConuter->bytes.value4 = buf[bufLength++];

		//�ɼ�����װ��
		tempId.bytes.value1 = buf[bufLength++] ;
		tempId.bytes.value2 = buf[bufLength++] ;
		*conuter = tempId.j;
//��ص�ѹ
		bufLength+=4;
//����ѹ
		bufLength+=4;
//�ر��¶�װ��
		if(topSensorFlag == TOP_SENSOR_USE)
		{
			b20Buf[0].value_t.bytes.valueH	= buf[bufLength++];
			b20Buf[0].value_t.bytes.valueL	= buf[bufLength++];			
			//b20Buf[0].value_t.bytes.valueH1 = buf[bufLength++];
			//b20Buf[0].value_t.bytes.valueH2 = buf[bufLength++];
			b20Buf[0].rf	=	b20Buf[0].value_t.i*(0.0625);
		}

		for(nodei = 0x00;nodei < devSenNum ;nodei ++)
		{
	//�¶�����װ��
			b20Buf[nodei +1].value_t.bytes.valueH	 = buf[bufLength++];
			b20Buf[nodei +1].value_t.bytes.valueL	 = buf[bufLength++];
		//	b20Buf[nodei +1].value_t.bytes.valueH1 = buf[bufLength++];
		//	b20Buf[nodei +1].value_t.bytes.valueH2 = buf[bufLength++];
			b20Buf[nodei +1].rf	=	b20Buf[nodei +1].value_t.i*(0.0625);
				
	//��Ƶˮ������װ��
			hlfBuf[nodei].fh.bytes.value1 =	buf[bufLength++];
			hlfBuf[nodei].fh.bytes.value2 =	buf[bufLength++];
			hlfBuf[nodei].fh.bytes.value3 =	buf[bufLength++];
			hlfBuf[nodei].fh.bytes.value4 =	buf[bufLength++];
	//��Ƶ�η�����װ��
			if(eachDevNum == 0x02)
			{
				hlfBuf[nodei].fl.bytes.value1 =	buf[bufLength++];
				hlfBuf[nodei].fl.bytes.value2 =	buf[bufLength++];
				hlfBuf[nodei].fl.bytes.value3 =	buf[bufLength++];
				hlfBuf[nodei].fl.bytes.value4 =	buf[bufLength++];
			}	   
		}	
		result = 0x00;
	 }
	}
	return result ;
}

int8_t  xyzDataReadWithSensorFlash(uint32_t readAddress,tg_ADXL345_TYPE *xyzBuf)
{
//    uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	value_t tempId ;

	uint8_t buf[BYTE_32_PACK_SIZE+1];
//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
	bufLength = 0x09;
	readAddress += XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	//��ȡ����������
	readResult = flashStoreDataRead(readAddress,BYTE_32_PACK_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		tempId.bytes.value1 = buf[bufLength++] ;
		tempId.bytes.value2 = buf[bufLength++] ;
		tempId.bytes.value3 = buf[bufLength++] ;
		tempId.bytes.value4 = buf[bufLength++] ;
		xyzBuf->xg = tempId.f;

		tempId.bytes.value1 = buf[bufLength++] ;
		tempId.bytes.value2 = buf[bufLength++] ;
		tempId.bytes.value3 = buf[bufLength++] ;
		tempId.bytes.value4 = buf[bufLength++] ;
		xyzBuf->yg = tempId.f;

		tempId.bytes.value1 = buf[bufLength++] ;
		tempId.bytes.value2 = buf[bufLength++] ;
		tempId.bytes.value3 = buf[bufLength++] ;
		tempId.bytes.value4 = buf[bufLength++] ;
		xyzBuf->zg = tempId.f;
	}
	return 	readResult;
}
int8_t  xyzDataReadFromFlashToBuf(uint32_t readAddress,uint8_t *readBuf,uint16_t *readLength)
{
	uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	value_t tempId ;

	uint8_t buf[BYTE_32_PACK_SIZE+1];
	//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
	bufLength = 0x09;
	datai = *readLength;
	readAddress += XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	//��ȡ����������
	readResult = flashStoreDataRead(readAddress,BYTE_32_PACK_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		//�豸��ص�ѹ����װ��
		readBuf[datai++]= 1;
		tempId.i = AX_ID;
		readBuf[datai++] = tempId.bytes.value4;
		readBuf[datai++] = tempId.bytes.value3;
		readBuf[datai++] = tempId.bytes.value2;
		readBuf[datai++] = tempId.bytes.value1;

		readBuf[datai++] = buf[bufLength+3];
		readBuf[datai++] = buf[bufLength+2];
		readBuf[datai++] = buf[bufLength+1];
		readBuf[datai++] = buf[bufLength];
		bufLength +=4;
		//�豸��ص�ѹ����װ��
		readBuf[*readLength++]= 1;
		tempId.i = AY_ID;
		readBuf[datai++] = tempId.bytes.value4;
		readBuf[datai++] = tempId.bytes.value3;
		readBuf[datai++] = tempId.bytes.value2;
		readBuf[datai++] = tempId.bytes.value1;

		readBuf[datai++] = buf[bufLength+3];
		readBuf[datai++] = buf[bufLength+2];
		readBuf[datai++] = buf[bufLength+1];
		readBuf[datai++] = buf[bufLength];
		bufLength +=4;

		//�豸��ص�ѹ����װ��
		readBuf[*readLength++]= 1;
		tempId.i = AZ_ID;
		readBuf[datai++] = tempId.bytes.value4;
		readBuf[datai++] = tempId.bytes.value3;
		readBuf[datai++] = tempId.bytes.value2;
		readBuf[datai++] = tempId.bytes.value1;

		readBuf[datai++] = buf[bufLength+3];
		readBuf[datai++] = buf[bufLength+2];
		readBuf[datai++] = buf[bufLength+1];
		readBuf[datai++] = buf[bufLength];
		bufLength +=4;
		*readLength =	datai;
	}
	return 	readResult;
}
int8_t  gpsDataReadWithSensorFlash(uint32_t readAddress,gpsValue *gpsBuf)
{
	//	uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	dou2Buf temp ;

	uint8_t buf[BYTE_32_PACK_SIZE+1];
	//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
	bufLength = 0x03;
	//��ȡ����������
	bufLength = 0x09;
	readAddress += GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	readResult = flashStoreDataRead(readAddress,BYTE_32_PACK_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		temp.bytes.value1 = buf[bufLength++];      
		temp.bytes.value2 = buf[bufLength++];    
		temp.bytes.value3 = buf[bufLength++];    
		temp.bytes.value4 = buf[bufLength++];    
		temp.bytes.value5 = buf[bufLength++];    
		temp.bytes.value6 = buf[bufLength++];    
		temp.bytes.value7 = buf[bufLength++];    
		temp.bytes.value8 = buf[bufLength++];    
		gpsBuf->longitude = temp;

		temp.bytes.value1 = buf[bufLength++];      
		temp.bytes.value2 = buf[bufLength++];    
		temp.bytes.value3 = buf[bufLength++];    
		temp.bytes.value4 = buf[bufLength++];    
		temp.bytes.value5 = buf[bufLength++];    
		temp.bytes.value6 = buf[bufLength++];    
		temp.bytes.value7 = buf[bufLength++];    
		temp.bytes.value8 = buf[bufLength++];    
		gpsBuf->latitude = temp;
		//
		gpsBuf->saveMode = buf[buf[bufLength++]]; 
	}
	return readResult;
}

int8_t  gpsDataReadFormFlashToBuf(uint32_t readAddress,uint8_t *readBuf,uint16_t *readLength)
{
	uint16_t datai = 0x00;
	uint16_t length = 0x00;
	uint16_t bufLength = 0x00;
	value_t tempId ;

	uint8_t buf[BYTE_32_PACK_SIZE+1];
	//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
	bufLength = 0x03;
	//��ȡ����������
	bufLength = 0x09;
	datai = *readLength;
	readAddress += GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	readResult = flashStoreDataRead(readAddress,BYTE_32_PACK_SIZE,buf,&length);
	if(readResult == 0x00)
	{
		readBuf[datai++]= 1;
		tempId.i = GPS_LONGITUDE_ID ;
		readBuf[datai++] = tempId.bytes.value4;
		readBuf[datai++] = tempId.bytes.value3;
		readBuf[datai++] = tempId.bytes.value2;
		readBuf[datai++] = tempId.bytes.value1;

		readBuf[datai++] = buf[bufLength+7];
		readBuf[datai++] = buf[bufLength+6];
		readBuf[datai++] = buf[bufLength+5];
		readBuf[datai++] = buf[bufLength+4];
		readBuf[datai++] = buf[bufLength+3];
		readBuf[datai++] = buf[bufLength+2];
		readBuf[datai++] = buf[bufLength+1];
		readBuf[datai++] = buf[bufLength];
		bufLength +=8;
		//		  
		readBuf[datai++]= 1;
		tempId.i = GPS_LATITUDE_ID;
		readBuf[datai++] = tempId.bytes.value4;
		readBuf[datai++] = tempId.bytes.value3;
		readBuf[datai++] = tempId.bytes.value2;
		readBuf[datai++] = tempId.bytes.value1;

		readBuf[datai++] = buf[bufLength+7];
		readBuf[datai++] = buf[bufLength+6];
		readBuf[datai++] = buf[bufLength+5];
		readBuf[datai++] = buf[bufLength+4];
		readBuf[datai++] = buf[bufLength+3];
		readBuf[datai++] = buf[bufLength+2];
		readBuf[datai++] = buf[bufLength+1];
		readBuf[datai++] = buf[bufLength];
		//		  bufLength +=8;
		*readLength = datai;	
	}
	return readResult;
}

int8_t sensorDataSendSucJudge(uint32_t readAddress)
{
//    uint16_t datai = 0x00;
	uint16_t length = 0x00;
//	uint16_t bufLength = 0x00;
//	dou2Buf temp ;

	uint8_t buf[BYTE_32_PACK_SIZE+1];
//	uint8_t nodei = 0x00;

	int8_t readResult = -1;
//	bufLength = 0x03;
	//��ȡ����������
//	bufLength = 0x09;
	readAddress += SENSOR_DATA_SEND_OK_FLAG_OFFSET;
	readResult = flashStoreDataRead(readAddress,BYTE_32_PACK_SIZE,buf,&length);
	if(0==readResult)
	{
	   if(('o'!=buf[9])||('k'!=buf[10]))
	   {
	      readResult = -1;
	   }
	}
	return readResult;
}

int8_t  sensorDataSendFlagWrite(uint32_t address,DS1302_TIME time)
{
	uint8_t  buf[BYTE_32_PACK_SIZE]; 
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
	int8_t result = 0;
    //�洢����ͷ��
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;

//�ɼ�ʱ������װ��
	buf[bufLength++]= time.year;
	buf[bufLength++]= time.month;
	buf[bufLength++]= time.date;
	buf[bufLength++]= time.hour;
	buf[bufLength++]= time.min;
	buf[bufLength++]= time.sec;

	buf[bufLength++]=	'o';
	buf[bufLength++]= 'k';

	for(nodei = 0x0A;nodei < bufLength;nodei ++)
	{
		crc += buf[nodei];
	}

	buf[bufLength++] = crc;

	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	//���ݴ洢

	//�洢��ַ�任
	 address +=   SENSOR_DATA_SEND_OK_FLAG_OFFSET;//SENSOR_DATA_STORE_SIZE+ 
	//дFLASH
 	SPI_FLASH_BufferWrite(buf,address,bufLength); 
	return result;   
}



// static void sendEndPacket(void)
// {
//    uint8_t data[100];
//    uint16_t dataSendLength =0;
//    uint16_t length = 0;
// 	DS1302_TIME  rtcTime1;
//       length =  sprintf((char *)(data+dataSendLength), "ts&%d",devId.bytes.value1);
// 				 dataSendLength +=length; 

//    length =  sprintf((char *)(data+dataSendLength), ".%d",devId.bytes.value2);
// 				 dataSendLength +=length; 
//    length =  sprintf((char *)(data+dataSendLength), ".%d",devId.bytes.value3);
// 				dataSendLength +=length; 
//    length =  sprintf((char *)(data+dataSendLength), ".%d&",devId.bytes.value4);
// 				dataSendLength +=length;
//    	readDs1302Time(&rtcTime1);

// 	  /*
// 	sensorBuf[DATE_YEAR_ADDRESS] = RTC_DateStructure.RTC_Year;
// 	sensorBuf[DATE_MON_ADDRESS] = RTC_DateStructure.RTC_Month;
// 	sensorBuf[DATE_DAY_ADDRESS] = RTC_DateStructure.RTC_Date;
// 	sensorBuf[DATE_HOUR_ADDRESS] = RTC_TimeStructure.RTC_Hours;
// 	sensorBuf[DATE_MIN_ADDRESS] = RTC_TimeStructure.RTC_Minutes;
// 	sensorBuf[DATE_SEC_ADDRESS] = RTC_TimeStructure.RTC_Seconds;
// 	*/
//   length =  sprintf((char *)(data+dataSendLength), "%d",(rtcTime1.year+2000));
// 				 dataSendLength +=length; 

//   length =  sprintf((char *)(data+dataSendLength), "-%02d",rtcTime1.month);
// 				 dataSendLength +=length; 

//   length =  sprintf((char *)(data+dataSendLength), "-%02d&",rtcTime1.date);
// 				 dataSendLength +=length; 


//   length =  sprintf((char *)(data+dataSendLength), "%02d:",rtcTime1.hour);
// 				 dataSendLength +=length; 
//   length =  sprintf((char *)(data+dataSendLength), "%02d:",rtcTime1.min);
// 				 dataSendLength +=length; 
//   length =  sprintf((char *)(data+dataSendLength), "%02d&",rtcTime1.sec);
// 				 dataSendLength +=length; 

// //������Ϣװ��				  
//    length =  sprintf((char *)(data+dataSendLength), "%d&",3);
// 				 dataSendLength +=length; 
//    			  length =  sprintf((char *)(data+dataSendLength), "%s", "te\n\r");
// 				 dataSendLength +=length; 
// 				 			   delay_modeMs(20);
// 				 usart2StoreBuffer(data,dataSendLength);
// }


uint8_t flashDataRead(uint32_t dataCountern,uint32_t *readAddress)
{
	uint8_t buf[SENSOR_DATA_STORE_SIZE+1];

	uint32_t addressTemp =0;
	uint16_t i =0;
	uint16_t j =0;
	uint8_t sendDataState =0;
	uint8_t counterFlag = 0;
	//  readAddress += FLASH_PACKET_SIZE;
	// for(;readAddress>0;readAddress -=FLASH_PACKET_SIZE)
	addressTemp =0;
	dataCountern +=5;
	power331On();//�򿪴洢FLASH��Դ
	delay_modeMs(5);
	SPI_Flash_WAKEUP();
	for(addressTemp =SENSOR_DATA_BASE_ADDR;addressTemp < SENSOR_DATA_END_ADDR;addressTemp +=SENSOR_DATA_STORE_SIZE)
	{
		SPI_FLASH_BufferRead(buf,*readAddress,SENSOR_DATA_STORE_SIZE);
		*readAddress -= SENSOR_DATA_STORE_SIZE;
		if(*readAddress ==SENSOR_DATA_BASE_ADDR)
		{
			*readAddress = SENSOR_DATA_END_ADDR;
		}
		if(buf[0]==0x74)
		{
			for(i =0;i<SENSOR_DATA_STORE_SIZE;i++)
			{
				if(((buf[i]==0x26)&&(buf[i+1]==0x74))&&(buf[i+2]==0x65))
				{
					break;
				}
			}
			if(i<SENSOR_DATA_STORE_SIZE)
			{
				counterFlag =0;
				dataCountern --;
				//��������
				j++;
				//			   send_RS232Ms(20);
				usart2StoreBuffer(buf,(i+5));
				//			   usart2StoreBuffer("\n\r",3);
				#ifdef FLASH_DEBUG
				printf("\n\r����[%d]���洢����Ҫ����\n\r",(dataCountern-5));
				printf("\n\r���͵�[%d]���洢����\n\r",j);
				#endif
			}
			if(i==SENSOR_DATA_STORE_SIZE)
			{
				counterFlag ++;
			}
		}

		if(dataCountern ==0)
		{
			sendDataState =1;
		#ifdef FLASH_DEBUG
			printf("\n\r flash data read end1\n\r");
		#endif
			break;
		}
		if(dataCountern <=5)
		{
			sendDataState =2; 
		#ifdef FLASH_DEBUG
			printf("\n\r flash data read end2\n\r");
		#endif
			break;
		}
	}
	SPI_Flash_PowerDown();
	power331Off();
	// sendEndPacket();
	power331Off();
	return sendDataState;
}



int8_t flashStoreDataRead(uint32_t readAddr,uint16_t packSize,uint8_t *readBuf,uint16_t *dataLentth)
{
    uint16_t datai = 0;
	uint16_t crcj = 0;
	uint8_t crcData = 0;
	int8_t readStatus = -1;
//��ȡ����
	SPI_FLASH_BufferRead(readBuf,readAddr,packSize);
//���������Ƿ��ǺϷ����ݰ�
	if((readBuf[0]==0x2c)&&(readBuf[1]==0x2c))
	{
		for(datai = 10;datai < packSize;datai ++)
		{
			if((readBuf[datai] ==END_CHARCCTER )&&(readBuf[datai+1] ==END_CHARCCTER )&&(readBuf[datai+2] ==END_CHARCCTER ))
			{
				if((readBuf[datai+3] ==0xFF )&&(readBuf[datai+4] ==0xFF ))
				{
					for(crcj = 0x03;crcj < datai -1;crcj ++)
					{
						crcData += readBuf[crcj]; 
					}
					if(crcData == readBuf[datai-1])
					{
						readStatus = 0x01;
						*dataLentth = crcj;
						break;
					}
				}
			}
		}
	}
	return readStatus;
}




uint16_t FlashWrite(uint32_t baseAddr,uint32_t endAddr,__IO uint32_t *WriteAddr,uint8_t *buf,uint16_t length,uint16_t packSize)
{
	uint16_t writeStatus = 0x00;
	if(((*WriteAddr)%packSize)!=0)	   //�����洢�ֽ�����
	{
	#ifdef FLASH_DEBUG
		printf("\n\r flash ��ַ���� \n\r");
	#endif
		*WriteAddr = 	*WriteAddr +packSize- ((*WriteAddr)%packSize);   
	}

	if((*WriteAddr)==endAddr)
	{
		*WriteAddr = baseAddr;
	}
	/* 
	if((*WriteAddr)==baseAddr)
	{
	SPI_FLASH_SectorErase(baseAddr);
	}  
	*/
	if((((*WriteAddr)%FLASH_SECTOR_SIZE)==0)&&((*WriteAddr)!=endAddr))//����һ������
	{
	#ifdef FLASH_DEBUG
		printf("\n\r flash �������� \n\r");
	#endif
		SPI_FLASH_SectorErase(*WriteAddr);
	}

	SPI_FLASH_BufferWrite(buf,*WriteAddr,length);
	*WriteAddr = 	*WriteAddr +packSize;
	if((*WriteAddr)==endAddr)
	{
		*WriteAddr = baseAddr;
	}
	
	if((((*WriteAddr)%FLASH_SECTOR_SIZE)==0)&&((*WriteAddr)!=endAddr))//����һ������
	{
	#ifdef FLASH_DEBUG
		printf("\n\r flash �������� \n\r");
	#endif
		SPI_FLASH_SectorErase(*WriteAddr);
	}
	
	return writeStatus;
}



 void sensorDataRead(uint32_t *writeAddress,uint16_t readCounter)
{
		   uint8_t dataBuf[SENSOR_DATA_STORE_SIZE];
		   
		   (void)flashSensorDataRead(SENSOR_DATA_BASE_ADDR,SENSOR_DATA_END_ADDR,writeAddress,dataBuf,SENSOR_DATA_STORE_SIZE,readCounter);
            //���ͽ�������֡
}

void dataSent(uint8_t *buf,uint16_t bufLength,uint8_t firComm,uint8_t secComm,uint8_t *dataOrder)
{
    uint16_t datai = 0x00;
	uint16_t dataLength = 0x00;

	uint8_t headData[20];

	uint8_t crc = 0x00;

	headData[0] = PACKET_HEADE_BEGIN;
	headData[5] = PACKET_HEADE_BEGIN;

	headData[6] = DATA_UP_FLAG;
	headData[7] = *dataOrder;
	*dataOrder = *dataOrder +1;
	if((*dataOrder) == 0xFF)
	{
	   *dataOrder = 0x00;
	}
	headData[8] = firComm;
	headData[9] = secComm;
	for(datai = 6;datai < 10;datai++)
	{
	    crc += headData[datai];
	}
	for(datai = 0x00;datai < bufLength;datai++)
	{
	   crc += buf[datai];
	}
	dataLength = bufLength +4;
 	headData[1] = (dataLength%256);
	headData[2] = (dataLength/256);
	headData[3] = (dataLength%256);
	headData[4] = (dataLength/256);
	headData[11] = crc;
   	usart2StoreBuffer(headData,10);
   	usart2StoreBuffer(buf,bufLength);
   	usart2StoreBuffer(&headData[11],1);

}

void devStatusDataSend(uint8_t *buf,uint16_t bufLength,uint8_t *order)
{
   dataSent(buf,bufLength,FIR_DEV_STATUS_DIS,SEC_DEV_DIS,order);

}



void wakeupByRxConfig(uint8_t state)
{

/* Private typedef -----------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  if(state ==0x01)
  {  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  }
  else
  {
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;  
  }
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
   if(state ==0x01)
  {  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  }
  else
  {
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;  
  }

  NVIC_Init(&NVIC_InitStructure);
}
#define DECIMALS_CTR	1 
double decimal(double data,uint8_t number)
{
	double intPart;
	double decimalPart;
	double result;
	double addV;
	double foldV;
	switch(number)
	{
		case 0:addV	=	0.5;foldV	=	1;break;
		case 1:addV	= 0.05;foldV = 10;break;
		case 2:addV	= 0.005;foldV = 100;break;
		case 4:addV	= 0.00005;foldV = 10000;break;
		case 5:addV	= 0.000005;foldV = 100000;break;
		case 6:addV	= 0.0000005;foldV = 1000000;break;
		default:addV	=	0.0005;foldV = 1000;break;
	}
	intPart	= (int64_t)data;
	decimalPart	= data - intPart;
	if(intPart>=0)
	{
		decimalPart	+= addV;
	}
	else
	{
		decimalPart	-= addV;
	}
	decimalPart	*= foldV;
	decimalPart	= (double)((int64_t)decimalPart)/foldV;
	result = intPart + decimalPart;	
	return result;
}
#if((SK60_CTR>0)||(KXYL_CTR>0))
uint16_t sk60ValueFormat(sk60_t sk60V,uint8_t *buf,DS1302_TIME time)
{
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t bufLength = 0x00;
	value_t temp ;

//�洢����ͷ��
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;
	buf[bufLength++]= 0x2c;

//�ɼ�ʱ������װ��
	buf[bufLength++]= time.year;
	buf[bufLength++]= time.month;
	buf[bufLength++]= time.date;
	buf[bufLength++]= time.hour;
	buf[bufLength++]= time.min;
	buf[bufLength++]= time.sec;
#if(SK60_CTR>0)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	temp.f = sk60V.t;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
//
	temp.f = sk60V.v;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
//
	temp.f = sk60V.dis;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4;

	temp.f = sk60OffsetDistance;//sk60Displacement;//210705
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
	
	temp.words.valueL = sk60V.riss;
	temp.bytes.value3 = sk60V.err;
	temp.bytes.value4 = sk60InitFlag;
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4; 
}else
#endif
{
	bufLength += 20;
}

#if(KXYL_CTR>0)	
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	temp.f = kxylValue[0];
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4;
	
	temp.f = kxylValue[1];
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4;
#if(KXYL_SENSOR_NUM>3)	//221128
	temp.f = kxylValue[2];
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4;
	
	temp.f = kxylValue[3];
	buf[bufLength++] = temp.bytes.value1;      
	buf[bufLength++] = temp.bytes.value2; 
	buf[bufLength++] = temp.bytes.value3; 
	buf[bufLength++] = temp.bytes.value4;
#endif
}else
#endif
{
	bufLength += 8;
}

	for(nodei = 0x03;nodei < bufLength;nodei ++)
	{
		crc += buf[nodei];
	}

	buf[bufLength++] = crc;

	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	buf[bufLength++] = END_CHARCCTER;
	return bufLength;//13+4*5 +4*2=29+8
}

void sk60ValueStoreWithSensorData(uint32_t address,sk60_t value)
{
	uint8_t  buf[EXTERNAL_DATA_STORE_SIZE];
	uint16_t bufLength = 0x00;
	uint32_t writeAddress = 0x00;	
	bufLength = sk60ValueFormat(value,buf,rtcTime);
	//�洢��ַ�任
	writeAddress = address - SENSOR_DATA_STORE_SIZE+ SK60_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST;
	//дFLASH
	SPI_FLASH_BufferWrite(buf,writeAddress,bufLength);	
} 

void sk60DataSendValueFlash(uint8_t *bufin,uint8_t num)
{
	uint16_t dataInLength = 0x00;
	value_t  	tempValue;
	
#if(SK60_CTR>0)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];	
	sensorValue.sk60t.f	= tempValue.f;

	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	sensorValue.sk60v.f	= tempValue.f;

	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];
	sensorValue.sk60dis.f	= tempValue.f;
	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];		
	sk60OffsetDistance		 = tempValue.f;//sk60Displacement sk60OffsetDistance_210705
	
	if(sk60OffsetDistance>SK60_DIS_MAX)sk60OffsetDistance=0;//210820
	if(devPar.sk60Fuction == FUNCITONG_SK60_YEWEI)
	{
		sensorValue.sk60disp.f	=	SK60_DIS_YEWEI_INIT - sk60OffsetDistance;
		if((sensorValue.sk60disp.f<0)||((sk60OffsetDistance==0)&&(sensorValue.sk60dis.f==0)))sensorValue.sk60disp.f = 0;	
	}
	else
	{
		sensorValue.sk60disp.f	=	sk60OffsetDistance;
	}
	
//	if(sensorValue.sk60dis.f>0)//210705 sk60Value.dis sensorValue.sk60dis.f_210708
//	{
//		if(devPar.sk60Fuction == FUNCITONG_SK60_YEWEI)
//		{			
//			sensorValue.sk60disp.f	=	SK60_DIS_YEWEI_INIT - sensorValue.sk60dis.f + sk60BaseDistance;//sk60Value.dis sensorValue.sk60dis.f_210714
//			if(sensorValue.sk60disp.f<0)sensorValue.sk60disp.f=0;			
//		}
//		else
//		{
//			sensorValue.sk60disp.f	=	sensorValue.sk60dis.f - sk60BaseDistance;//sk60Value.dis sensorValue.sk60dis.f_210714
//		}
//		if(sensorValue.sk60disp.f>SK60_DIS_MAX)sensorValue.sk60disp.f = 0;
//	}
//	else{sensorValue.sk60disp.f = 0;}//210820_����
	
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];	
	sensorValue.sk60status.i	= tempValue.i;
}else
#endif
{
	dataInLength += 20;
}

#if(KXYL_CTR>0)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];	
	sensorValue.kxylv1.f	= tempValue.f;

	tempValue.bytes.value1 = bufin[dataInLength++];
	tempValue.bytes.value2 = bufin[dataInLength++];
	tempValue.bytes.value3 = bufin[dataInLength++];
	tempValue.bytes.value4 = bufin[dataInLength++];	
	sensorValue.kxylv2.f	= tempValue.f;
 #if(KXYL_SENSOR_NUM>3)	//221128
	if(num>3)
	{
		tempValue.bytes.value1 = bufin[dataInLength++];
		tempValue.bytes.value2 = bufin[dataInLength++];
		tempValue.bytes.value3 = bufin[dataInLength++];
		tempValue.bytes.value4 = bufin[dataInLength++];	
		sensorValue.kxylv3.f	= tempValue.f;

		tempValue.bytes.value1 = bufin[dataInLength++];
		tempValue.bytes.value2 = bufin[dataInLength++];
		tempValue.bytes.value3 = bufin[dataInLength++];
		tempValue.bytes.value4 = bufin[dataInLength++];	
		sensorValue.kxylv4.f	= tempValue.f;
	}
	else
	{
		sensorValue.kxylv3.f	= 0.0;
		sensorValue.kxylv4.f	= 0.0;
	}
 #endif
}
#endif
}

uint16_t formatSK60FlashData(uint8_t *bufOut,uint16_t index)
{
	uint16_t dataOutLength= 0x00;

	value_t		tempId ;
	value_t  	tempValue;
	dataOutLength	=	index;
#if(SK60_CTR>0)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	bufOut[dataOutLength++]= 0x01;
	tempId.i = SK60_T_ID ;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.sk60t.f;
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,2);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%.2f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = SK60_V_ID ;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.sk60v.f;	  
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,4);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%.2f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = SK60_DIS_ID ;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.sk60dis.f;
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,4);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%.3f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = SK60_DISPLACEMENT_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.sk60disp.f;
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,4);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif

	bufOut[dataOutLength++]= 0x01;
	tempId.i = SK60_STATUS_ID ;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,",tempId.i);    
#endif	
	tempId.i = sensorValue.sk60status.i;	
	tempValue.i	= (tempId.bytes.value3*10000+tempId.words.valueL);
  if(tempId.bytes.value4>0)tempValue.i += tempId.bytes.value4*1000000;
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("     ֵ%d\n\r",tempValue.i);    
#endif
}
#endif

#if(KXYL_CTR>0)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	bufOut[dataOutLength++]= 0x01;
	tempId.i = KXYL_VALUE1_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.kxylv1.f;
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,2);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif
	bufOut[dataOutLength++]= 0x01;
	tempId.i = KXYL_VALUE2_ID;
	bufOut[dataOutLength++] = tempId.bytes.value4;
	bufOut[dataOutLength++] = tempId.bytes.value3;
	bufOut[dataOutLength++] = tempId.bytes.value2;
	bufOut[dataOutLength++] = tempId.bytes.value1;
	
	tempValue.f = sensorValue.kxylv2.f;	
#if(DECIMALS_CTR>0)
	tempValue.f	=	decimal(tempValue.f,2);
#endif
	bufOut[dataOutLength++] = tempValue.bytes.value4;
	bufOut[dataOutLength++] = tempValue.bytes.value3;
	bufOut[dataOutLength++] = tempValue.bytes.value2;
	bufOut[dataOutLength++] = tempValue.bytes.value1;	
#if( SENSOR_DATA_FORMAT_KEY  >0)		  
	printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
#endif
#if(KXYL_SENSOR_NUM>3)	//221128
		bufOut[dataOutLength++]= 0x01;
		tempId.i = KXYL_VALUE3_ID;
		bufOut[dataOutLength++] = tempId.bytes.value4;
		bufOut[dataOutLength++] = tempId.bytes.value3;
		bufOut[dataOutLength++] = tempId.bytes.value2;
		bufOut[dataOutLength++] = tempId.bytes.value1;
		
		tempValue.f = sensorValue.kxylv3.f;
	#if(DECIMALS_CTR>0)
		tempValue.f	=	decimal(tempValue.f,2);
	#endif
		bufOut[dataOutLength++] = tempValue.bytes.value4;
		bufOut[dataOutLength++] = tempValue.bytes.value3;
		bufOut[dataOutLength++] = tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;	
	#if( SENSOR_DATA_FORMAT_KEY  >0)		  
		printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
	#endif
		bufOut[dataOutLength++]= 0x01;
		tempId.i = KXYL_VALUE4_ID;
		bufOut[dataOutLength++] = tempId.bytes.value4;
		bufOut[dataOutLength++] = tempId.bytes.value3;
		bufOut[dataOutLength++] = tempId.bytes.value2;
		bufOut[dataOutLength++] = tempId.bytes.value1;
		
		tempValue.f = sensorValue.kxylv4.f;	
	#if(DECIMALS_CTR>0)
		tempValue.f	=	decimal(tempValue.f,2);
	#endif
		bufOut[dataOutLength++] = tempValue.bytes.value4;
		bufOut[dataOutLength++] = tempValue.bytes.value3;
		bufOut[dataOutLength++] = tempValue.bytes.value2;
		bufOut[dataOutLength++] = tempValue.bytes.value1;	
	#if( SENSOR_DATA_FORMAT_KEY  >0)		  
		printf("\n\r����ID:%d,     ֵ%f\n\r",tempId.i,tempValue.f);    
	#endif
#endif
}
#endif
//	dataOutLength = dataOutLength + index;
	return dataOutLength;		
}

#endif

extern int8_t tiltTypeCheck(__IO uint8_t *buf);
void sensorValueCurrent(void)
{
	uint8_t nodei;
	//�ɼ�ʱ��
	sensorValue.time = rtcTimeSample;	
	if(rtcTimeSample.sec<6){sensorValue.time.sec = 0;}
	//�豸�ܵĲɼ�����װ��
	sensorValue.countAll.i	= calAllCounter.i;	
	//�ɼ�����װ��	
	sensorValue.count.i			= calCounter;	
	//��ص�ѹ
	sensorValue.batV.f 			=	ROUND3(batStatus);//*6.144/ADS_MAXVALUE
	//����ѹ����װ��	
	sensorValue.inputV.f 		=	ROUND3(adsexterValue);//*6.144/ADS_MAXVALUE
//�ر��¶�
	sensorValue.t.f	 = result[0].rf;
	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
		sensorValue.soilt[nodei].f	= result[nodei+1].rf;
		sensorValue.fh[nodei]				= curWat[nodei].fh;
		sensorValue.fl[nodei]	 			= curWat[nodei].fl;
		sensorValue.wat[nodei].f	 	= curWat[nodei].wat.f;
	}
#if(QX_CTR>1)
	if(tiltTypeCheck(&sensorValue.sign[0])!=0)
#endif
	{
		sensorValue.ax.f = xlResult[32].xg;
		sensorValue.ay.f = xlResult[32].yg;
		sensorValue.az.f = xlResult[32].zg;
	}
	sensorValue.longitude.f	=	devLongitude;
	sensorValue.latitude.f	=	devLatitude;
#if(SK60_CTR>0)
if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
{
	sensorValue.sk60t.f 	= sk60Value.t;
	sensorValue.sk60v.f 	= sk60Value.v;
	sensorValue.sk60dis.f	= sk60Value.dis;
//	sensorValue.sk60disp.f 	=	sk60Displacement;//210705
	if(devPar.sk60Fuction == FUNCITONG_SK60_YEWEI)//210820
	{
		sensorValue.sk60disp.f	=	SK60_DIS_YEWEI_INIT - sk60OffsetDistance;
		if((sensorValue.sk60disp.f<0)||((sk60OffsetDistance==0)&&(sensorValue.sk60dis.f==0)))sensorValue.sk60disp.f = 0;
	}
	else
	{
		sensorValue.sk60disp.f	=	sk60OffsetDistance;
	}	
//	if(sensorValue.sk60dis.f>0)//210705 sk60Value.dis sensorValue.sk60dis.f_210708
//	{
//		if(devPar.sk60Fuction == FUNCITONG_SK60_YEWEI)
//		{
//			sensorValue.sk60disp.f	=	SK60_DIS_YEWEI_INIT - sensorValue.sk60dis.f + sk60BaseDistance; //sk60Value.dis sensorValue.sk60dis.f_210714
//			if(sensorValue.sk60disp.f<0)sensorValue.sk60disp.f=0;			
//		}
//		else
//		{
//			sensorValue.sk60disp.f	=	sensorValue.sk60dis.f - sk60BaseDistance;	//sk60Value.dis sensorValue.sk60dis.f_210714
//		}
//		if(fabs(sensorValue.sk60disp.f-tempValue.f)<SK60_DISP_MAX){sk60DispErrTimes = 0;}else{sk60DispErrTimes++;}
//		
//		if(sensorValue.sk60disp.f>SK60_DIS_MAX)sensorValue.sk60disp.f = 0;
//	}
//	else{sensorValue.sk60disp.f = 0;}//210820_����
	
	sensorValue.sk60status.words.valueL = sk60Value.riss;
	sensorValue.sk60status.bytes.value3 = sk60Value.err;
	sensorValue.sk60status.bytes.value4 = sk60InitFlag;
}
#endif
#if(KXYL_CTR>0)
if(devPar.kxylFuction == FUNCITONG_TURN_ON)
{
	sensorValue.kxylv1.f	=	kxylValue[0];
	sensorValue.kxylv2.f	=	kxylValue[1];
 #if(KXYL_SENSOR_NUM>3)	//221128
	sensorValue.kxylv3.f	=	kxylValue[2];
	sensorValue.kxylv4.f	=	kxylValue[3];
 #endif
}
#endif
}

#if(DEV_APN_SET_CTR>0)
int8_t 	devRxCounter = 0;
//char  	apnBufSet[APN_LEN_MAX+2] =	{0,0,0};//230307
//int8_t 	devApnSetStatus = APN_SAVE_STATUS_INIT;//230307

void gprsApnSave(uint8_t status)//230307
{
	uint16_t 	devi = 0x00;
	uint8_t 	crcData = 0x00;
	uint16_t  writeBufLength = 0x00;
	uint32_t 	dataWriteAddress = 0x00;
	uint8_t 	writeBuf[GPRS_APN_STORE_SIZE];
	DS1302_TIME readTime;
	
	dataWriteAddress = findFlashWriteAddr(DEV_GPRS_APN_BASE_ADDR,DEV_GPRS_APN_END_ADDR,GPRS_APN_STORE_SIZE,writeBuf);

	//���Ӵ洢ʱ���¼ �������÷�ʽ
	readDs1302Time(&readTime);
	writeBufLength = 0x00;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;
	writeBuf[writeBufLength++]= 0x2c;	
	writeBuf[writeBufLength++]= readTime.year;
	writeBuf[writeBufLength++]= readTime.month;
	writeBuf[writeBufLength++]= readTime.date;
	writeBuf[writeBufLength++]= readTime.hour;
	writeBuf[writeBufLength++]= readTime.min;
	writeBuf[writeBufLength++]= readTime.sec;
	writeBuf[writeBufLength++]= RS232_SET_FLAG;
	writeBuf[writeBufLength++]= status;	
	for(devi =0;devi<APN_LEN_MAX;devi++) 
	{
		writeBuf[writeBufLength++] = apnBufSet[devi];
		if((apnBufSet[devi+1]=='\r')&&(apnBufSet[devi+2]=='\n'))break;
	}
	writeBuf[writeBufLength++] = '\r';
	writeBuf[writeBufLength++] = '\n';
	for(devi = 0x00;devi < writeBufLength;devi++)
	{
		crcData += writeBuf[devi];
	}
	writeBuf[writeBufLength++]= crcData;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;
	writeBuf[writeBufLength++]= END_CHARCCTER;

	//���Ӵ洢ʱ���¼ �������÷�ʽ
	FlashWrite(DEV_GPRS_APN_BASE_ADDR,DEV_GPRS_APN_END_ADDR,&dataWriteAddress,writeBuf,writeBufLength,GPRS_APN_STORE_SIZE);
}

static void gprsApnRead(void)//230307
{
//	uint16_t writeBufLength = 0x00;
	uint16_t datai = 0x00;
	uint8_t writeBuf[GPRS_APN_STORE_SIZE+2];
	uint32_t dataWriteAddress = 0x00;
	uint8_t	dataj;
	int8_t 	status = -1;
	uint8_t	crcData = 0x00;
	
	dataWriteAddress = findFlashWriteAddr(DEV_GPRS_APN_BASE_ADDR,DEV_GPRS_APN_END_ADDR,GPRS_APN_STORE_SIZE,writeBuf);
	(void)findFlashReadAddr(DEV_GPRS_APN_BASE_ADDR,DEV_GPRS_APN_END_ADDR,dataWriteAddress,GPRS_APN_STORE_SIZE,writeBuf);

	if((writeBuf[0]==0x2c)&&(writeBuf[1]==0x2c))
	{
		for(datai = 10;datai < GPRS_APN_STORE_SIZE;datai ++)
		{
			if(writeBuf[datai] ==END_CHARCCTER )
			{
				if((writeBuf[datai+1] ==END_CHARCCTER )&&(writeBuf[datai+2] ==END_CHARCCTER ))
				{
					if((writeBuf[10] == APN_SAVE_STATUS_NONE)||(writeBuf[10] == APN_SAVE_STATUS_DONE))
					{
						for(dataj=0;dataj<datai-1;dataj++)crcData += writeBuf[dataj]; 
						if(writeBuf[datai-1] == crcData)
						{
							for(dataj=0;dataj<APN_LEN_MAX;dataj++)
							{
								apnBufSet[dataj]= writeBuf[11+dataj];
								if((writeBuf[11+dataj+1]=='\r')&&(writeBuf[11+dataj+2]=='\n')){status=0;break;}					
							}
							if(status==0)
							{
								devApnSetStatus	=	writeBuf[10];
								dataj += 1;
								apnBufSet[dataj++]='\r';
								apnBufSet[dataj++]='\n';
								for(;dataj<(APN_LEN_MAX+2);dataj++)apnBufSet[dataj]= 0;
								printf("\r\napnSet %d:%s",devApnSetStatus,apnBufSet);
							}
						}						
					}
					break;
				}
			}
		}
	}
	if(status!=0)
	{
		devApnSetStatus	=	APN_SAVE_STATUS_INIT;
		for(datai=0;datai<APN_LEN_MAX;datai++)apnBufSet[datai] = 0;
	}
}
void devApnInit(void)//230307
{
	devFlashErare(DEV_GPRS_APN_BASE_ADDR,DEV_GPRS_APN_END_ADDR);
}

int8_t devConfigRx(uint8_t *buf)
{	
	uint8_t i;
	for(i=0;i<10;i++)
	{
		Receive_Byte(buf,(3*NAK_TIMEOUT));
		if(buf[0]=='t')break;	
	}	
	if(buf[0]!='t')return -1;	
	if(Receive_Byte(buf+1,(NAK_TIMEOUT)) != 0)return -1;	
	if(buf[1]!='s')return -2;	
	if(Receive_Byte(buf+2,(NAK_TIMEOUT)) != 0)return -1;	
	if(buf[2]!='&')return -2;
	
	for(i=3;i<127;i++)
	{
		if(Receive_Byte(buf+i,(NAK_TIMEOUT)) != 0)break;		
	}	
	return i;
}
#include<string.h>
uint8_t findStrRxDev(char *buf)
{
	uint8_t findi		= 0x00;
	uint8_t findj		= 0x00;
	uint8_t ptrOut 	= 0x00;
	uint8_t ptr    	= 0x00;
	uint8_t bufLength = strlen(buf);
	for(findi = 0x00;findi < devRxCounter;findi++)
	{
		if(GPRS_RX_Buffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(GPRS_RX_Buffer[ptr++] == buf[findj])
				{
					ptr &= GPRS_RX_DATA_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut &= GPRS_RX_DATA_SIZE;
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
			ptrOut &= GPRS_RX_DATA_SIZE;
		} 
	}
	return 	0xFF;
}

void devConfigTask(void)
{		
	uint8_t i;
	uint8_t i_begin,i_end;
	int8_t 	status 	= -1;
	uint8_t apnLen	=	0;
	char  	apnBufTmp[APN_LEN_MAX+2];
	uint8_t apnFlag = 0;
		
	for(i=0;i<128;i++)GPRS_RX_Buffer[i]=0;
	devRxCounter = devConfigRx(GPRS_RX_Buffer);
	if(devRxCounter>6)
	{	
		SPI_Flash_WAKEUP();
		delay_ms(10);
		i_begin = findStrRxDev("apn:");
		if(i_begin<0xFF){apnFlag = 1;}
		else
		{
			i_begin = findStrRxDev("apnNet:");
			if(i_begin<0xFF){apnFlag = 2;}
		}
		(void)devParamterRead(&devPar);
		gprsApnRead();
		if(i_begin<0xFF)
		{			
			i_end 	= findStrRxDev("&te");
			if(i_end>3)i_end -= 3;
			if(i_end>i_begin)apnLen = i_end-i_begin;
			if((apnLen>0)&&(apnLen<APN_LEN_MAX))
			{
				for(i=0;i<apnLen;i++)
				{
					apnBufTmp[i]= GPRS_RX_Buffer[i+i_begin];			
				}
				apnBufTmp[i++]  = '\r';
				apnBufTmp[i++] 	= '\n';
				for(;i<(APN_LEN_MAX+2);i++)apnBufTmp[i] = 0;			
				status = 0;	
				
				printf("\r\napnIn :%s",apnBufTmp);
				if((apnLen==3)&&(apnBufTmp[0]=='0')&&(apnBufTmp[1]=='0')&&(apnBufTmp[2]=='0'))
				{
					for(i=0;i<(APN_LEN_MAX+2);i++){apnBufSet[i] = 0;devPar.apnBuf[i] = 0;}
					if(devApnSetStatus != APN_SAVE_STATUS_INIT)
					{
						devApnSetStatus = APN_SAVE_STATUS_INIT;
						devApnInit();						
					}
					printf("\r\napnSet:%s",apnBufSet);
					devParamterStore(devPar);
					printf("\r\napnNet:%s",devPar.apnBuf);
				}
				else if((apnLen==3)&&(apnBufTmp[0]=='0')&&(apnBufTmp[1]=='0')&&(apnBufTmp[2]=='1'))
				{
					for(i=0;i<(APN_LEN_MAX+2);i++)apnBufSet[i] = 0;
					if(devApnSetStatus != APN_SAVE_STATUS_INIT)
					{
						devApnSetStatus = APN_SAVE_STATUS_INIT;
						devApnInit();
					}
					printf("\r\napnSet:%s",apnBufSet);					
				}
				else if((apnLen==3)&&(apnBufTmp[0]=='0')&&(apnBufTmp[1]=='0')&&(apnBufTmp[2]=='2'))
				{
//					(void)devParamterRead(&devPar);
					for(i=0;i<(APN_LEN_MAX+2);i++)devPar.apnBuf[i] = 0;
					devParamterStore(devPar);
					printf("\r\napnNet:%s",devPar.apnBuf);
					if(devApnSetStatus ==	APN_SAVE_STATUS_NONE)
					{
						devApnSetStatus =	APN_SAVE_STATUS_DONE;
						gprsApnSave(devApnSetStatus);	
					}
				}
				else
				{
					if(apnFlag == 1)
					{
						for(i=0;i<(APN_LEN_MAX+2);i++)apnBufSet[i] = apnBufTmp[i];
						devApnSetStatus =	APN_SAVE_STATUS_DONE;
						gprsApnSave(devApnSetStatus);						
						printf("\r\napnSet:%s",apnBufSet);
					}
					else
					{
//						devPar.apnBuf[0] = apnLen;//230630
						for(i=0;i<(APN_LEN_MAX+1);i++)devPar.apnBuf[i] = apnBufTmp[i];		//apnBuf[i+1]				
						devParamterStore(devPar);
						if(devApnSetStatus == APN_SAVE_STATUS_DONE)
						{
							devApnSetStatus = APN_SAVE_STATUS_NONE;
							gprsApnSave(devApnSetStatus);	
						}
						printf("\r\napnNet:%s",devPar.apnBuf);
					}	
				}						
			}
		}
		if(status!=0)//230625
		{			
			i_begin = findStrRxDev("clearUnsendData");
			if(i_begin<0xFF)
			{
				if((GPRS_RX_Buffer[i_begin+1]=='&')&&(GPRS_RX_Buffer[i_begin+2]=='t')&&(GPRS_RX_Buffer[i_begin+3]=='e'))
				{			
					status = 0;
					unSendDataPacket = 0;
					curGprsSendAddr  = sensorDataWriteAddr;
					printf("\r\n���δ���ʹ�����");
				}
			}
		}
//		if(status!=0)
//		{			
//			i_begin = findStrRxDev((uint8_t *)"usartKey:",9);
//			if(i_begin<0xFF)
//			{
//				if((GPRS_RX_Buffer[i_begin+1]=='&')&&(GPRS_RX_Buffer[i_begin+2]=='t')&&(GPRS_RX_Buffer[i_begin+3]=='e'))
//				{
//					if((GPRS_RX_Buffer[i_begin]=='0')||(GPRS_RX_Buffer[i_begin]=='1'))
//					{
//						status = 1;
//						devPar.usartKey	=	GPRS_RX_Buffer[i_begin];						
//						printf("\r\nusartKey:%c",devPar.usartKey);	
//						devParamterStore(devPar);						
//					}
//				}
//			}
//		}
//		if((status!=0)&&(status!=1))
//		{
//			(void)devConfigValueFlashRead(&configValue);
//			i_begin = findStrRxDev((uint8_t *)"gpsKey:",7);
//			if(i_begin<0xFF)
//			{
//				if((GPRS_RX_Buffer[i_begin+1]=='&')&&(GPRS_RX_Buffer[i_begin+2]=='t')&&(GPRS_RX_Buffer[i_begin+3]=='e'))
//				{
//					if((GPRS_RX_Buffer[i_begin]=='0')||(GPRS_RX_Buffer[i_begin]=='1'))
//					{
//						status = 1;						
//						configValue.gpsKey	=	GPRS_RX_Buffer[i_begin];
//						if(configValue.gpsKey == '1')configValue.gpsLocation = 1;						
//						printf("\r\ngpsKey:%c",configValue.gpsKey);
//						(void)devConfigValueFlashStore(configValue);
//					}
//				}
//			}
//		}
//		if(status==1)
//		{			
//			status = 0;
//		}
//		if(status!=0)
//		{
//			i_begin = findStrRxDev((uint8_t *)"clearFlash:",11);
//			if(i_begin<0xFF)
//			{
//				if(findStrRxDev((uint8_t *)"allFlash&te",6)<0xFF)
//				{								
//					devFlashInit();						
//					status = 0;
//					printf("\r\nclearFlash:allFlash ok");
//				}
//				else if(findStrRxDev((uint8_t *)"historyData&te",14)<0xFF)
//				{
//					devFlashHistoryInit();
//					status = 0;
//					printf("\r\nclearFlash:historyData ok");
//				}				
//				else if(findStrRxDev((uint8_t *)"sn&te",5)<0xFF)
//				{								
//					devFlashSNInit();
//					status = 0;
//					printf("\r\nclearFlash:sn ok");
//				}
//			}
//		}
		SPI_Flash_PowerDown();
	}
	if(status!=0)printf("\r\ndevConfig err\r\n");
}

void devPutString(uint8_t *s)
{
  while (*s != '\0')
  {
    SerialPutChar(*s);		
    s++;
  }
}

void Config_Menu(void)
{
  uint8_t key = 0;
  uint8_t waitTimes = 0x00;
  uint8_t menuTaskState = 0x01;
//  int8_t operationResult = 0x00;
  int8_t getKeyResult = -1; 
//	uint8_t i;
	(void)USART_ReceiveData(USART2);

  while (menuTaskState)
  {
		devPutString((uint8_t*)"\r\n==================== �豸����  ===========================\r\n\n");
//    devPutString((uint8_t*)"  ����manifest�ļ� -----------------------------���� 1\r\n\n");
//    devPutString((uint8_t*)"  �¶�У׼--------------------------------------���� 2\r\n\n");
//		devPutString((uint8_t*)"  ����У׼--------------------------------------���� 3\r\n\n");
//		devPutString((uint8_t*)"  ����У׼--------------------------------------���� 4\r\n\n");
		devPutString((uint8_t*)"  ��������--------------------------------------���� 5\r\n\n");
//	#if(KXYL_CTR>0)
//		devPutString((uint8_t*)"  ͨ������--------------------------------------���� 6\r\n\n");
//	#endif
		devPutString((uint8_t*)"  �˳�����--------------------------------------���� a\r\n\n");
    devPutString((uint8_t*)"==========================================================\r\n\n");

    /* Receive key */		
		getKeyResult =Receive_Byte(&key,NAK_TIMEOUT);
		
		if(getKeyResult!=(-1))
		{
//			if (key == 0x31)//'1'
//			{
//				waitTimes	=	0;
//				/* Download user application in the Flash */
//				SerialPutString("\r\nDownloading manifest file and Set Device SN---------\r\n\n");
//				SerialDownload(FILE_MAINFEST);
////				if(operationResult== 0x00)
////				{
////					menuTaskState = 0x00;	 
////				}			
//			}			
//			else if(key == 0x32)//'2'
//			{
//				waitTimes	=	0;
//				SerialPutString("\r\nConfiguration Device-----------------\r\n\n");				
//				operationResult	=	SerialDownload(FILE_CONFIG);

//				if(resulttmp == 0)
//				{
//					printf("DevDep Set ok!\r\n");
//					printf("DevDep:\r\n");
//					for(i=0;i<4;i++)printf("%f ",curDep[i].dep.f);
//				}

//				if(operationResult== 0x00)
//				{
//					menuTaskState = 0x00;	 
//				}
//			}
//			else if(key == 0x33)//'3'
//			{
//				waitTimes	=	0;
//				SerialPutString("\r\n  calCounter Resetting--------\r\n\n");
//				calCounter_Reset();
//			}
//			else if(key == 0x34)//'4'
//			{
//				waitTimes	=	0;
//			//	devPutString("\r\n  ����������ģʽ-----------\r\n\n");
//				operationResult	=	sloarCalibration();
//				if(operationResult== 0x00)
//				{
//				//	menuTaskState = 0x00;	 
//				}
//			}
//			else 
			if(key == 0x35)//'5'
			{
				waitTimes	=	0;
				devPutString((uint8_t*)"\r\n  �����������ģʽ-----------\r\n\n");
				devConfigTask();			
			}
			else if((key == 'a')||(key == 'A'))
			{
				menuTaskState = 0x00;
				devPutString((uint8_t*)"\r\n  �û��˳� -----------------------------------------------\r\n\n");	
				break;
			}
			else
			{
				devPutString((uint8_t*)"\r\n  �Ƿ����� ==> ����Ӧ��Ϊ'1','2','3','4','a'\r\n");
			}			
		}
		waitTimes ++;
		if(waitTimes > 2)
		{
		 menuTaskState = 0x00;
		 devPutString((uint8_t*)"\r\n  ��ʱ�˳� ------------------------------------------\r\n\n");		
		}
  }	
}
#endif

