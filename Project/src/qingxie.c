#include "qingxie.h"
#include <stdlib.h>

#include "common.h"
#include <stdio.h>
#include "sensor.h"
#include "string.h"

extern uint8_t * aPacketData;//[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];


extern USART_TypeDef* UartHandle;

void usart3_init(void)
{
	GPIO_InitTypeDef usart3_gpio_initstructure;
	USART_InitTypeDef	usart3_initstructure;
//	NVIC_InitTypeDef NVIC_InitStructure;			

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);		//enable gpio clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//enable usart3 clock

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);	//connect PC.11 to usart3's tx
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);	//connect PC.10 to usart3's rx
	
	/* Configure USART Tx as alternate function push-pull */
  usart3_gpio_initstructure.GPIO_Pin 		= GPIO_Pin_11;		
  usart3_gpio_initstructure.GPIO_Mode 	= GPIO_Mode_AF;
  usart3_gpio_initstructure.GPIO_Speed	= GPIO_Speed_40MHz;
  usart3_gpio_initstructure.GPIO_OType 	= GPIO_OType_PP;
  usart3_gpio_initstructure.GPIO_PuPd 	= GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &usart3_gpio_initstructure);
    
  /* Configure USART Rx as alternate function push-pull */
  usart3_gpio_initstructure.GPIO_Pin =GPIO_Pin_10;		
  usart3_gpio_initstructure.GPIO_Mode = GPIO_Mode_AF;
  usart3_gpio_initstructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC,&usart3_gpio_initstructure);

	usart3_initstructure.USART_BaudRate = 115200;

  usart3_initstructure.USART_WordLength = USART_WordLength_8b;
  usart3_initstructure.USART_StopBits = USART_StopBits_1;
  usart3_initstructure.USART_Parity = USART_Parity_No;
  usart3_initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart3_initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &usart3_initstructure); 		
	USART_Cmd(USART3, ENABLE);										

	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);					

//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //NVIC_PriorityGroup_1 
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 	   			
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 	 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 					
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	UartHandle = USART3;
	
}
void usart3_lowpower(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    /* Enable USART */
  USART_Cmd(USART3, DISABLE);

  USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, DISABLE);
    /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode =	GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

}
void usart3_delay(uint32_t nTime)
{
  for(; nTime != 0; nTime--);
}

void usart3_sendbyte(unsigned char * byte, unsigned char len)
{
	unsigned char i = 0;
	for(i = 0; i < len; i++)
	{
		USART_SendData(USART3, byte[i]);
		usart3_delay(2000);
	}
}




uint8_t aFileName[64];
uint8_t * aFileBody;
uint32_t aFileSize;
 
//memory use 96+1030+64 Bytes
//uint8_t aFileName[FILE_NAME_LENGTH];

//uint8_t fix_memory_start[12*8+1024+4+2];
uint8_t * fix_memory_start  = (uint8_t *)sensorBuf;

uint8_t *fix_buffer = (uint8_t *) sensorBuf +12*8;
uint8_t *tilt_sensorData = (uint8_t *)sensorBuf; //length 12*8

//memory use



uint8_t * aFileBody;
uint32_t aFileSize;

extern USART_TypeDef* UartHandle;

uint32_t k;
     
COM_StatusTypeDef SessionGetAFile(uint8_t * fName,uint8_t** pData,uint32_t * size)
{
    uint8_t status = CRC16;
    uint32_t len=0;
	  COM_StatusTypeDef result;
    
		//printf(">Now begin to receive FILE...... \n\r"  );
		//1. send a "C"  to burn the whole file transport process session!!!!!
    HAL_UART_Transmit(UartHandle, &status, 1, NAK_TIMEOUT);  
    //2.call  a Ymodem_Receive（）process until the end of file-trans
	  result = Ymodem_Receive(fName,pData, &len);
		if (result == COM_OK)
		{
      *size = len;
      printf(">receive via UART fname(%s) size(%d)sucessefully!\n\r",aFileName,len);      
		}
		else if (result == COM_LIMIT)
		{
			printf("\n\n\r>The image size is higher than the allowed space memory\n\r");
		}
		else if (result == COM_DATA)
		{
			printf("\n\n\r>Verification failed!\n\r");
		}
		else if (result == COM_ABORT)
		{
			printf("\r\n\n>Aborted by user.\n\r");
		}
		else
		{
			printf("\n\r>Failed to receive the file!\n\r");
		} 
    return result;
}

uint8_t SessionPutAFile(uint8_t * fName,uint8_t* pData,uint32_t  size)
{
	uint8_t status = 1;
	//1.receive the "C"(byte) sent by the other side to see if its a CRC16(0x43) startup code.
	//	again: //2018-6-19去掉
	HAL_UART_Receive(UartHandle, &status, 1, RX_TIMEOUT);
	//2.call SerialSEND() to do transfering
	if ( status == CRC16)
	{
		// printf("\n\rBegin to send FILE.... \n\r");
		status = Ymodem_Transmit(fName,pData,size); // TLV styler
		//status = Ymodem_Transmit(((uint8_t *)"CMD.transSensorData"),pData,size); // TLV styler
		if (status != 0)
		{
			printf("\n\r>Error Occurred while Sending File(%s)\n\r",fName); 
		}
		else  
		{
			printf("\n\r>File(%s)(%d) Sent successfully \n\r",fName,size);
		}
	}
	else
	{
		status = 2;
	//		goto again; //2018-6-19去掉
		printf(">ERR:received a non CRC16 cmd[0x%x]n\r",status);
	}
	return status;
}
  

COM_StatusTypeDef Initiate_A_Talk_sensorData(uint8_t ** pp_sensorData)
{
	uint8_t status=1;
	COM_StatusTypeDef result = COM_ERROR;
	aPacketData = fix_buffer;

	//usart3SendBuf((uint8_t *)"123456789",9);
	//1. initiate the first time
	status = SessionPutAFile(((uint8_t *)"CMD.transSensorData"),0x0,0);    //上位机-> 下位机ymodem_transmit

	//2.get file on demand
	aFileBody = *pp_sensorData;
	if(status == 0)result = SessionGetAFile(aFileName,(uint8_t**)&aFileBody,&aFileSize);  //下位机-> 上位机ymodem_receive

	/*Here collect the data to send up-net, pointed by aFileBody with aFileSize length*/

#if 0
	//3.ask for the endding of talk(may shutdown)
	SessionPutAFile((uint8_t *)"CMD.askForEndingTalk",0x0,0); 

	//4.wait the power-off indicator.	
	aFileBody = *pp_sensorData;
	SessionGetAFile(aFileName,(uint8_t**) &aFileBody,&aFileSize);

	if(!strcmp((   char *)aFileName,(  char *)"CMD.mayPowerOff")){
	//cut off it's power supply
	}else{
	//cut off it's power supply anyway
	}
#endif
	return result;
}


void Initiate_A_Talk_upgrade_file_fromFlash(char *ptr,int size)
{
		
		//1.issue the  upgrade command
			SessionPutAFile((uint8_t *)"CMD.receiveFW",0x0,0);
  
	//	1.5 in advance tell the file name and size only.
			SessionPutAFile((uint8_t *)"FW.js_z_enc.ver.major_1.0_minor_1_170808",0x0/*temperorily null*/,size);
		//2. put FW data downwards
	#ifdef _USE_FLASH_SOURCE_
			SessionPutAFile((uint8_t *)"FW.js_z_enc.ver.major_1.0_minor_1_170808",(uint8_t*)ptr,size); //ptr points to address of extern flash(0~8MB)
	#else
			SessionPutAFile((uint8_t *)"FW.js_z_enc.ver.major_1.0_minor_1_170808",(uint8_t*)ptr,size);
	#endif	
	
		//3.ask for the endding of talk(may shutdown)
			SessionPutAFile((uint8_t *)"CMD.askForEndingTalk",0x0,0); 
			
		//4.wait the power-off indicator.	
			aFileBody = 0x0;
			SessionGetAFile(aFileName,(uint8_t**)& aFileBody,&aFileSize);
			
			if(!strcmp((   char *)aFileName,(  char *)"CMD.mayPowerOff")){
				//cut off it's power supply
			}else{
				//cut off it's power supply
			}
		return;
}
/****************tilt sensor******************/
#include "usart.h"
#include "crc16.h"

#define TILT_MODBUD_ADDRESS 	0x98
#define TILT_READ_HOLDREG			0x03
#define TILT_READ_INPUTREG		0x04
#define TILT_WRITE_SINGLEREG	0x06
#define TILT_CONFIG_REG				0x006C
#define TILT_STATUS_REG				0x0190
#define TILT_DATA_REG					0x019B
#define TILT_DATA_REG_LEN			0x2A
#define TILT_DATA_LEN					0x54

#define TILT_AXIS_TYPE_Z		0x10
#define TILT_AXIS_TYPE_Y		0x20
#define TILT_AXIS_TYPE			TILT_AXIS_TYPE_Y
#define TILT_KM_COUNTER			128


#define TILT_STATUS_NOINIT				0x0F
#define TILT_STATUS_WAITINIT			0x0E
#define TILT_STATUS_SAMPLE				0x06
#define TILT_STATUS_NORMAL				0x02

#define TILT_RX_TIME_OUT			0x30000	//	30ms
#define tiltSendData					uart3StoreBuffer

#define TILT_STATUS_NONE	0x00
#define TILT_STATUS_DONE	0x00

#define TILT_EN_PIN					GPIO_Pin_12
#define TILT_EN_Source			GPIOB
#define TILT_EN_RCC					RCC_AHBPeriph_GPIOB
#define TILT_EN_ON()				TILT_EN_Source->BSRRH = TILT_EN_PIN	//0
#define TILT_EN_OFF()				TILT_EN_Source->BSRRL = TILT_EN_PIN	//1

uint8_t gAxisType	=	0;
uint8_t kmCounter	=	0;
void qx_delay_ms(uint32_t delay)
{
  uint16_t i;
	while(delay--)
	{
		for(i=0;i<8300;i++);
	}
}
void tiltPowerOn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable PIN clock */
	RCC_AHBPeriphClockCmd(TILT_EN_RCC, ENABLE); 
	/* pins configuration */
	GPIO_InitStructure.GPIO_Pin 	= TILT_EN_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(TILT_EN_Source, &GPIO_InitStructure); 
	TILT_EN_ON();
}

void tiltPowerOff(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TILT_EN_OFF();
	 /* Enable PIN clock */
	 RCC_AHBPeriphClockCmd(TILT_EN_RCC, ENABLE);
	/* pins configuration */
	GPIO_InitStructure.GPIO_Pin 	= 	TILT_EN_PIN;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  =   GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType =   GPIO_OType_OD;//
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_400KHz;		
	GPIO_Init(TILT_EN_Source, &GPIO_InitStructure); 		 
}
void tiltSendCmd(uint8_t cmd,uint16_t address,uint16_t len)
{
	uint8_t buf[10];
	value16_t crc16;
	buf[0] 	= TILT_MODBUD_ADDRESS;
	buf[1] 	= cmd;
	crc16.i = address;
	buf[2] 	= crc16.bytes.valueH;
	buf[3] 	= crc16.bytes.valueL;
	crc16.i = len;
	buf[4] 	= crc16.bytes.valueH;
	buf[5] 	= crc16.bytes.valueL;
	crc16.i	= CRC16_1(buf,6);	
	buf[6]	= crc16.bytes.valueL;
	buf[7]	= crc16.bytes.valueH;
	tiltSendData(buf,8);
}
int8_t tiltReceiveByte (uint8_t *data, uint32_t timeout)
{
  while (timeout-- > 0)
  {
    if( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
		{
			*data = (uint8_t)USART3->DR;
			return 0;
		}
  }
  return -1;
}
uint16_t tiltReceivedata(uint8_t *buf,uint16_t buflen,uint32_t timeout)
{
	uint16_t rxi 		= 0;
	uint8_t data		=	0;
	uint8_t status 	= 1;
	while(timeout--)
	{
		if(tiltReceiveByte(&data,TILT_RX_TIME_OUT)!=0){timeout = 0;break;}
		if(status == 1)
		{
			if(data == TILT_MODBUD_ADDRESS)
			{							
				status = 2;
			}
		}
		else if(status == 2)
		{
			if((data == 0x03)||(data == 0x04)||(data == 0x06))
			{		
				buf[0] 	= TILT_MODBUD_ADDRESS;
				buf[1] 	= data;
				rxi			=	2;
				status 	= 3;	
				timeout += 10;
			}
			else if(data == TILT_MODBUD_ADDRESS)
			{
				buf[0] = data;
			}
			else
			{
				status = 1;
			}
		}
		else if(status == 3)
		{		
			if(rxi==2)
			{
				if((buf[1]==0x03)||(buf[1]==0x04))timeout += data;			
			}
			buf[rxi++] = data;			
			if(rxi>=buflen){timeout = 0;break;}			
		}
	}
	return rxi;
}
uint16_t tiltFindRxdata(uint8_t *buf,uint8_t bufLength,uint8_t *bufIn,uint16_t bufInlen)
{
	uint16_t findi 		= 0x00;
	uint16_t findj 		= 0x00;
	uint16_t ptr    	= 0x00;
	
	if(bufLength> bufInlen)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < bufInlen;findi++)
	{
		if(bufIn[ptr]== buf[0])
		{	
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(bufIn[ptr] != buf[findj])
				{				
					findj = 0x00;					
					break;
				}
				ptr++;
				if(ptr>=bufInlen)break;
			}
			if(findj == bufLength)
			{
				return ptr;
			} 
		}
		else
		{
			ptr++;
		 if(ptr>=bufInlen)break;
		} 
	}
	return 	UN_FIND_STATUS;
}

//读取设备状态
//发送：98 04 01 90 00 01 2D D2	
//成功：98 04 02 22 80 BC 2C	
int8_t tiltReadStatus(uint8_t *bufIn,uint16_t bufInlen)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t datalen = 0;
	uint8_t buf[8];
	
	tiltSendCmd(TILT_READ_INPUTREG,TILT_STATUS_REG,1);
	datalen = tiltReceivedata(bufIn,bufInlen,50);
	if(datalen>6)
	{
		buf[0] = TILT_MODBUD_ADDRESS;
		buf[1] = TILT_READ_INPUTREG;
		buf[2] = 0x02;
		status = tiltFindRxdata(buf,3,bufIn,datalen);
		if(status<FIND_ERROR)
		{
			printf("\r\n tilt status:0x%02X_%d ",bufIn[status],bufIn[status+1]);
			if((datalen - status)>3)
			{
				if((bufIn[status]&0xF0)==TILT_AXIS_TYPE)
				{
					if((bufIn[status]&0x0F)<TILT_STATUS_WAITINIT)
					{
						if(bufIn[status+1] == TILT_KM_COUNTER)return 0;
						return 3;
					}
					return 2;
				}
				return 1;
			}
			return -3;
		}
		return -2;
	}
	return -1;
}
//Y轴设置为G轴,同时滤波次数128
//发送：98 06 00 6C 82 80 35 1E	
//返回：98 06 00 6C 02 00 54 DE	
//成功：98 04 02 22 80 BC 2C	
int8_t tiltConfig(uint8_t *bufIn,uint16_t bufInlen,uint8_t axisType,uint8_t filtcount)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t datalen = 0;
	uint8_t buf[8];
	datalen	=	0x8000|(axisType<<4)|filtcount;
	tiltSendCmd(TILT_WRITE_SINGLEREG,TILT_CONFIG_REG,datalen);
	datalen = tiltReceivedata(bufIn,bufInlen,50);
	if(datalen>7)
	{
		buf[0] = TILT_MODBUD_ADDRESS;
		buf[1] = TILT_WRITE_SINGLEREG;
		buf[2] = TILT_CONFIG_REG>>8;
		buf[3] = TILT_CONFIG_REG&0xFF;
		status = tiltFindRxdata(buf,4,bufIn,datalen);
		if(status<FIND_ERROR)
		{
			return 0;
		}
		return -2;
	}
	return -1;
}
//读取当前倾角数据
//发送：98 04 01 9B 00 28 9D CE	
//返回：98 04 50 …… ……
void tiltdatadeal(uint8_t *buf)
{
	uint16_t dataInLength = 0x00; 
	doubleV_t doubletmp;
	value_t		valuetmp;
	doubletmp.buf[1]=buf[dataInLength++];
	doubletmp.buf[0]=buf[dataInLength++];
	doubletmp.buf[3]=buf[dataInLength++];
	doubletmp.buf[2]=buf[dataInLength++];
	doubletmp.buf[5]=buf[dataInLength++];
	doubletmp.buf[4]=buf[dataInLength++];
	doubletmp.buf[7]=buf[dataInLength++];
	doubletmp.buf[6]=buf[dataInLength++];
	sensorValue.dipX.f = doubletmp.f;
	
	doubletmp.buf[1]=buf[dataInLength++];
	doubletmp.buf[0]=buf[dataInLength++];
	doubletmp.buf[3]=buf[dataInLength++];
	doubletmp.buf[2]=buf[dataInLength++];
	doubletmp.buf[5]=buf[dataInLength++];
	doubletmp.buf[4]=buf[dataInLength++];
	doubletmp.buf[7]=buf[dataInLength++];
	doubletmp.buf[6]=buf[dataInLength++];
	sensorValue.dipY.f = doubletmp.f;
	
	doubletmp.buf[1]=buf[dataInLength++];
	doubletmp.buf[0]=buf[dataInLength++];
	doubletmp.buf[3]=buf[dataInLength++];
	doubletmp.buf[2]=buf[dataInLength++];
	doubletmp.buf[5]=buf[dataInLength++];
	doubletmp.buf[4]=buf[dataInLength++];
	doubletmp.buf[7]=buf[dataInLength++];
	doubletmp.buf[6]=buf[dataInLength++];
	sensorValue.dipZ.f = doubletmp.f;	
	
	doubletmp.buf[1]=buf[dataInLength++];
	doubletmp.buf[0]=buf[dataInLength++];
	doubletmp.buf[3]=buf[dataInLength++];
	doubletmp.buf[2]=buf[dataInLength++];
	doubletmp.buf[5]=buf[dataInLength++];
	doubletmp.buf[4]=buf[dataInLength++];
	doubletmp.buf[7]=buf[dataInLength++];
	doubletmp.buf[6]=buf[dataInLength++];
	sensorValue.dipYaw.f = doubletmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.angle.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	doubletmp.f 		= valuetmp.f;
	sensorValue.ax.f = doubletmp.f*0.001;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	doubletmp.f 		= valuetmp.f;
	sensorValue.ay.f = doubletmp.f*0.001;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	doubletmp.f 		= valuetmp.f;
	sensorValue.az.f = doubletmp.f*0.001;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.qxt.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.origX.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.valX.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.origY.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.valY.f = valuetmp.f;
	
	dataInLength += 8;//valZ origZ
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.origYaw.f = valuetmp.f;
	
	valuetmp.buf[1] = buf[dataInLength++];
	valuetmp.buf[0] = buf[dataInLength++];
	valuetmp.buf[3] = buf[dataInLength++];
	valuetmp.buf[2] = buf[dataInLength++];
	sensorValue.valYaw.f = valuetmp.f;
	
	printf("\r\n qxX:%f,qxY:%f,qxZ:%f",sensorValue.dipX.f,sensorValue.dipY.f,sensorValue.dipZ.f);
	printf(",angle:%f,qxYaw:%f,t:%f",sensorValue.angle.f,sensorValue.dipYaw.f,sensorValue.qxt.f);
	printf(",xg:%f,yg:%f,zg:%f",sensorValue.ax.f,sensorValue.ay.f,sensorValue.az.f);
	printf(",X_std_filt:%f,X_std_raw:%f",sensorValue.valX.f,sensorValue.origX.f);
	printf(",Y_std_filt:%f,Y_std_raw:%f",sensorValue.valY.f,sensorValue.origY.f);
	printf(",Yaw_std_filt:%f,Yaw_std_raw:%f \r\n",sensorValue.valYaw.f,sensorValue.origYaw.f);
}

int8_t tiltReceiveData(uint8_t *bufIn,uint16_t bufInlen,uint8_t sendStatus)
{
	uint16_t status = UN_FIND_STATUS;
	uint16_t datalen = 0;
	uint8_t buf[8];
	value16_t crc16;
	if(sendStatus == TILT_STATUS_DONE)tiltSendCmd(TILT_READ_INPUTREG,TILT_DATA_REG,TILT_DATA_REG_LEN);
	datalen = tiltReceivedata(bufIn,bufInlen,500);
	if(datalen>TILT_DATA_LEN)
	{
		buf[0] = TILT_MODBUD_ADDRESS;
		buf[1] = TILT_READ_INPUTREG;	
		buf[2] = TILT_DATA_LEN;	
		status = tiltFindRxdata(buf,3,bufIn,datalen);
		if(status<FIND_ERROR)
		{
			printf("\r\n tilt datalen:0x%02X_%d",bufIn[status-1],status);
			if(datalen-status>TILT_DATA_LEN)
			{
				crc16.bytes.valueL = bufIn[status+TILT_DATA_LEN];
				crc16.bytes.valueH = bufIn[status+TILT_DATA_LEN+1];				
				if(crc16.i == CRC16_1(&bufIn[status-3],TILT_DATA_LEN+3))
				{
					tiltdatadeal(&bufIn[3]);
					return 0;
				}
				return -4;
			}
			return -3;
		}
		return -2;
	}
	return -1;
}
//采集1次数据
//发送：98 06 00 65 00 01 45 DC	
//返回：98 06 00 65 00 01 45 DC
//返回：98 04 50 …… ……
void tiltTask(void)
{
	int8_t result = -1;
	uint8_t tiltStep = 1;
	uint8_t	trytimes=0;
	tiltType = 0;
	
	while(tiltStep)
	{
		switch(tiltStep)
		{
			case 1:
			{
				result = tiltReadStatus(sensorBuf,MAX_SENSOR_BUF_SIZE+1);
				if(result==0)
				{
					tiltStep = 2;
					trytimes = 0;
				}
				else if(result>0)
				{
					tiltStep = 3;
					trytimes = 0;
				}				
				else if(result<0)
				{
					trytimes++;
					if(trytimes>5){tiltStep = 0;break;}
					qx_delay_ms(500);
				}
				break;
			}
			case 2:
			{
				result = tiltReceiveData(sensorBuf,MAX_SENSOR_BUF_SIZE+1,TILT_STATUS_DONE);
				if(result == 0)
				{
					tiltStep = 0;
					tiltType = 1;
				}
				else
				{
					trytimes++;
					if(trytimes>5){tiltStep = 0;break;}
					qx_delay_ms(500);
				}
				break;
			}
			case 3:
			{
				result = tiltConfig(sensorBuf,MAX_SENSOR_BUF_SIZE+1,TILT_AXIS_TYPE,TILT_KM_COUNTER);
				if(result == 0)
				{
					tiltStep = 2;
					trytimes = 0;
					qx_delay_ms(5000);
				}
				else
				{
					trytimes++;
					if(trytimes>5){tiltStep = 0;break;}
					qx_delay_ms(500);
				}
				break;
			}
		}		
	}
}
int8_t tiltTypeCheck(__IO uint8_t *buf)
{
	uint8_t tmpi=0,tmpj=0;
	if((buf[0]=='Q')&&(buf[1]=='Y')&&(buf[2]==':'))
	{
		for(tmpi=3;tmpi<20;tmpi++)
		{
			if(buf[tmpi] == ',')tmpj++; 
		}
		if(tmpj==2)return 0;
	}
	return -1;
}
/****************tilt sensor end******************/
