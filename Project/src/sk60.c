/**
  ******************************************************************************
  * @file    Project/user/sk60.c 
  * @author  insentek wxh
  * @version V1.0.0
  * @date    9-September-2019
  * @brief   
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
		
/* Includes ------------------------------------------------------------------*/
	#include "sk60.h"
	#include "usart.h"
	#include "key.h"
	#include "gprs.h"
	#include "math.h"
	#include "deviceset.h"
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
#define sk60RxBuffer	GPRS_RX_Buffer
//	__IO uint8_t sk60RxBuffer[SK60_RX_BUFFER_SIZE+1];
const sk60_t 	sk60VInit	= {0.0,0.0,0.0,0,0xFF};
__IO sk60_t 	sk60Value	= {0.0,0.0,0.0,0,0};
sk60_t sk60Buffer[SK60_BUFFER_SIZE+1]={'\0'};
//__IO float 	sk60Displacement = 0.0;
__IO float 	sk60BaseDistance = 0.0;
__IO float 	sk60OffsetDistance = 0.0;
__IO float 	sk60OffsetDistanceLast = 0.0;
__IO uint8_t	sk60UsartStatus	=	0;
__IO uint8_t	sk60InitFlag		=	0;//SK60_INIT_FLAG; 0_210705
__IO uint8_t	sk60RxState		= UN_RX;
__IO uint8_t sk60RxPtrStore	= 0;
uint8_t sk60RxPtrOut				=	0;
uint16_t	sk60usartBand			=	SK60_USART_Baud;
uint16_t	sk60TimerCounter	=	0;
uint16_t	sk60TimerRx		=	0;
uint8_t 	sk60TestMode	=	0;
uint8_t 	sk60TestTimes	=	0;
uint16_t	sk60DispErrTimes = 0;

#if(KXYL_CTR>0)
	#define KXYL_DEBUG 				SK60_DEBUG
	#define KXYL_URX					UN_RX
	#define KXYL_RXSTATE			RX_STATE
	#define KXYL_RXOVER				RX_OVER
	#define kxylRxStatus			sk60RxState

	#define STATUS_NONE				0x00
	#define STATUS_DONE				0x01	
	#define KXYL_RX_OVERTIME	5

	uint16_t	kxylTimerCounter	=	0;
	uint16_t	kxylTimerRx				=	0;
	uint8_t 	kxylusartkey			=	STATUS_NONE;
#endif
/* Private functions ---------------------------------------------------------*/ 
	
void sk60PowerOn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable PIN clock */
	RCC_AHBPeriphClockCmd(SK60_EN_RCC, ENABLE); 
	/* pins configuration */
	GPIO_InitStructure.GPIO_Pin 	= SK60_EN_PIN;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(SK60_EN_Source, &GPIO_InitStructure); 
	SK60_EN_ON();
}

void sk60PowerOff(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	SK60_EN_OFF();
	 /* Enable PIN clock */
	 RCC_AHBPeriphClockCmd(SK60_EN_RCC, ENABLE);
	/* pins configuration */
	GPIO_InitStructure.GPIO_Pin 	= 	SK60_EN_PIN;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  =   GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType =   GPIO_OType_OD;//
	GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_400KHz;		
	GPIO_Init(SK60_EN_Source, &GPIO_InitStructure); 		 
}

	/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void sk60Usart_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(SK60_USART_PIN_RCC, ENABLE);
  
  /* Enable USART clock */
  SK60_USART_RCC_Cmd(SK60_USART_RCC, ENABLE);
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(SK60_USART_Source,SK60_USART_TX_Source,SK60_USART_AF);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(SK60_USART_Source,SK60_USART_RX_Source,SK60_USART_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;//GPIO_PuPd_UP GPIO_PuPd_NOPULL_210729
  GPIO_InitStructure.GPIO_Pin 	= SK60_USART_TX_PIN|SK60_USART_RX_PIN;
  GPIO_Init(SK60_USART_Source, &GPIO_InitStructure);
  

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = sk60usartBand;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(SK60_USART, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = SK60_USART_IRQHandler;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(SK60_USART, ENABLE);
	(void)USART_ReceiveData(SK60_USART);
	USART_ClearITPendingBit(SK60_USART,USART_FLAG_RXNE);
  USART_ITConfig(SK60_USART, USART_IT_RXNE, ENABLE);
	sk60UsartStatus	=	1;
}
void sk60UartLowPower(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable USART */
	USART_Cmd(SK60_USART, DISABLE);
	sk60UsartStatus	=	0;
	USART_ITConfig(SK60_USART, USART_IT_RXNE, DISABLE);
	/* Enable USART clock */
	SK60_USART_RCC_Cmd(SK60_USART_RCC, DISABLE);
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(SK60_USART_PIN_RCC, ENABLE);

	GPIO_InitStructure.GPIO_Pin 	= SK60_USART_TX_PIN|SK60_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;  
	GPIO_Init(SK60_USART_Source, &GPIO_InitStructure);
}

void sk60Putchar(uint8_t buf)
{
	uint16_t i=10000;	
/* Loop until the end of transmission */
	while ((USART_GetFlagStatus(SK60_USART, USART_FLAG_TXE) == RESET)&&(i--))//
	{
	}
	USART_SendData(SK60_USART,buf);	
}
void sk60Receive(void)
{
#if(KXYL_CTR>0)
	if((kxylusartkey == STATUS_DONE)&&(sk60RxState == UN_RX))
	{
		sk60RxPtrStore	=	0;
		sk60RxState			= RX_STATE;
	}
	kxylTimerRx	 =	0;
#endif
	if(sk60RxState == RX_STATE)
	{
		sk60RxBuffer[sk60RxPtrStore++] = USART_ReceiveData(SK60_USART);
		sk60RxPtrStore &= SK60_RX_BUFFER_SIZE;
	}
	else
	{
		(void)USART_ReceiveData(SK60_USART);
	}
}
static void sk60_delay_ms(uint16_t nTime)
{
	uint16_t i;
	while(nTime)
	{
		nTime--;
		for(i=0;i<8300;i++);
	}  
}
void sk60RxBufferClear(void)
{
	uint16_t i;
	sk60RxState			= UN_RX;
	for(i=0;i<SK60_RX_BUFFER_SIZE+1;i++)sk60RxBuffer[i]='\0';	
	sk60RxPtrStore 	= 0;
	sk60RxPtrOut		=	0;
}
void sk60RxBufferClearPart(uint16_t status,uint16_t len)
{
	uint16_t rxi=0;
	for(rxi = 1;rxi<= len;rxi++)
	{
		sk60RxBuffer[(status-rxi)&SK60_RX_BUFFER_SIZE] = 0xFE;					
	}
}
uint16_t sk60FindRxStr(uint8_t *buf,uint8_t bufLength)
{
	uint16_t findi 		= 0x00;
	uint16_t findj 		= 0x00;
	uint32_t ptrOut 	= 0x00;
	uint16_t counter 	= 0x00;
	uint32_t ptr    	= 0x00;
	ptrOut 						= sk60RxPtrOut;
	ptr								=	sk60RxPtrStore;
	if(ptrOut < ptr)
	{
		counter = ptr - ptrOut;
	}
	else
	{	
		counter = SK60_RX_BUFFER_SIZE+1 + ptr - ptrOut;
	}
	#if(SK60_DEBUG>1)
		printf("\r\nsk60Rx %d:%s end\r\n",counter,sk60RxBuffer);
	#endif
	if(bufLength > counter)
	{
		return 	FIND_ERROR;   
	}
	for(findi = 0x00;findi < counter;findi++)
	{
		if(sk60RxBuffer[ptrOut]== buf[0])
		{
			ptr = ptrOut;
			for(findj = 0x00;findj < bufLength; findj++)
			{
				if(sk60RxBuffer[ptr++] == buf[findj])
				{
					ptr &= SK60_RX_BUFFER_SIZE;
				}
				else
				{
					findj = 0x00;
					ptrOut ++;
					ptrOut &= SK60_RX_BUFFER_SIZE;
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
			ptrOut &= SK60_RX_BUFFER_SIZE;
		} 
	}
	return 	UN_FIND_STATUS;
}

int8_t sk60LedControl(uint8_t cmd)
{
	int8_t 		rxResult 		= -1;	
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	uint16_t	timeout			=	5000;
	sk60RxBufferClear();
	sk60RxState	= RX_STATE;	
	sk60Putchar(cmd);
	sk60_delay_ms(SK60_CMD_CTR_TIME);
	while((sk60RxPtrStore<SK60_CMD_CTR_SIZE)&&(timeout--))sk60_delay_ms(1);
	findStatus = sk60FindRxStr((uint8_t *)"OK!",3);//",OK!",4) 220915 
	if(findStatus<FIND_ERROR)
	{
		sk60RxBufferClearPart(findStatus,4);
		rxResult = 0;
	#if(SK60_DEBUG>1)
		printf("\r\nsk60cmd:%s,OK!",&cmd);
	#endif
	}
	else
	{
		rxResult = -1;
	#if(SK60_DEBUG>0)
		printf("\r\nsk60cmd:%s,Err!\r\n",&cmd);
	#endif
	}

	return rxResult;
}
#include <stdlib.h>
int8_t sk60ReadStatus(float *t,float *v)//210729
{
	int8_t 		rxResult 		= -1;	
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	uint16_t 	findStatus2 	= UN_FIND_STATUS;	
	uint16_t	timeout			=	5000;
	float 		tmpT				=	0.0;
	float 		tmpV				=	0.0;
	char valueBuf[10];
	uint8_t rxi;
	sk60RxBufferClear();
	sk60RxState	= RX_STATE;	
	sk60Putchar(SK60_CMD_STATUS);
	sk60_delay_ms(SK60_CMD_STATUS_TIME);
//S:-09.8'C,3.2V
//S: 00.5'C,3.2V
	while((sk60RxPtrStore<SK60_CMD_STATUS_SIZE)&&(timeout--))sk60_delay_ms(1);
	findStatus 	= sk60FindRxStr((uint8_t *)"S:",2);
	findStatus2 = sk60FindRxStr((uint8_t *)"C,",2);
	if((findStatus<FIND_ERROR)&&(findStatus2<FIND_ERROR))//&&(sk60RxBuffer[findStatus+3]=='.')&&(sk60RxBuffer[findStatus+5]=='`'))
	{		
		sk60RxBufferClearPart(findStatus,2);
		sk60RxBufferClearPart(findStatus2,2);
		if(sk60RxBuffer[findStatus]==0x20){findStatus++;}//' ' 0x20
		for(rxi=0;rxi<8;rxi++)//210729
		{
			if((findStatus==(findStatus2-3))||(sk60RxBuffer[findStatus]==0x60)||(sk60RxBuffer[findStatus]==0x27))// `_0x60¡¢'_0x27
			{
				valueBuf[rxi] = '\0';
				break;
			}
			else if((sk60RxBuffer[findStatus] == '-')&&(rxi==0))
			{
				valueBuf[rxi] = sk60RxBuffer[findStatus];
			}
			else if(((sk60RxBuffer[findStatus]>='0')&&(sk60RxBuffer[findStatus]<='9'))||(sk60RxBuffer[findStatus]=='.'))
			{
				valueBuf[rxi] = sk60RxBuffer[findStatus];
			}
			 
			findStatus++;
		}		
		tmpT = atof(valueBuf);
		for(rxi=0;rxi<8;rxi++)valueBuf[rxi] = '\0';
		findStatus = findStatus2;
		for(rxi=0;rxi<5;rxi++)
		{
			if(sk60RxBuffer[findStatus]>='V')	
			{
				valueBuf[rxi] = '\0';
				break;
			}
			else if(((sk60RxBuffer[findStatus]>='0')&&(sk60RxBuffer[findStatus]<='9'))||(sk60RxBuffer[findStatus]=='.'))
			{
				valueBuf[rxi] = sk60RxBuffer[findStatus];
			}
			 
			findStatus++;
		}		
		tmpV = atof(valueBuf);
//	
//		tmpT  = (sk60RxBuffer[findStatus+2]-0x30)*10.0;
//		tmpT += (sk60RxBuffer[findStatus+2]-0x30);
//		tmpT += (sk60RxBuffer[findStatus+4]-0x30)*0.1;
//	
//		if(sk60RxBuffer[findStatus+1]=='-')tmpT *= (-1.0);		
					
//		if((sk60RxBuffer[findStatus+7]==',')&&(sk60RxBuffer[findStatus+9]=='.'))
//		{
//			tmpV  = (sk60RxBuffer[findStatus+8]-0x30);
//			tmpV += (sk60RxBuffer[findStatus+10]-0x30)*0.1;		
//		}			
	#if(SK60_DEBUG>1)
		printf("\r\nsk60T:%.1f¡æ,sk60V:%.1fV",tmpT,tmpV);
	#endif
		if((tmpT>SK60_T_MAX)||(tmpT<SK60_T_MIN))tmpT = 0.0;
		if((tmpV>SK60_V_MAX)||(tmpV<SK60_V_MIN))tmpV = 0.0;
		rxResult = 0;
	}
	else
	{
		rxResult = -1;
	#if(SK60_DEBUG>0)
		printf("\r\nsk60 read status Err!\r\n");
	#endif
	}
	
	*t = tmpT;
	*v = tmpV;
	return rxResult;
}

int8_t sk60Measure(uint8_t cmd,float *distance,uint16_t *riss,uint16_t *err)
{
	int8_t 		rxResult 		= -1;	
	uint16_t 	findStatus 	= UN_FIND_STATUS;	
	uint16_t 	findStatus2	= UN_FIND_STATUS;	
	uint16_t	timeout			=	5000;
	float 		tmpDis			=	0.0;
	uint16_t 	tmpRiss			=	0;
	uint16_t 	tmpErr			=	0xFF;
	uint8_t 	rxi=0;

	char valueBuf[10];

	sk60RxBufferClear();
	sk60RxState	= RX_STATE;	
	sk60Putchar(cmd);
	sk60_delay_ms(SK60_CMD_MEASURE_TIME);
//D: 0.216m,0116
//D:10.153m,0127
//D:Er08!
	while((sk60RxPtrStore<SK60_CMD_MEASURE_SIZE)&&(timeout--)){if((timeout%3000)==0)sk60Putchar(cmd);sk60_delay_ms(1);}
//	printf("\r\nrx:");for(timeout=0;timeout<sk60RxPtrStore;timeout++)printf("%c",sk60RxBuffer[timeout]);
	switch(cmd)
	{
		case SK60_CMD_MEASURE_AUTO:findStatus = sk60FindRxStr((uint8_t *)"D:",2);break;
		case SK60_CMD_MEASURE_SLOW:findStatus = sk60FindRxStr((uint8_t *)"M:",2);break;
		case SK60_CMD_MEASURE_FAST:findStatus = sk60FindRxStr((uint8_t *)"F:",2);break;
	}	
	if((findStatus<FIND_ERROR))
	{
		sk60_delay_ms(2);//210820 10 2_220914
		sk60RxBufferClearPart(findStatus,2);
		findStatus2 = sk60FindRxStr((uint8_t *)"m,",2);		//220914
		if((findStatus2<FIND_ERROR))
		{
			sk60RxBufferClearPart(findStatus2,2);
			if(sk60RxBuffer[findStatus] == 0x20){findStatus++;}
			tmpDis = 0.0;						
			for(rxi=0;rxi<8;rxi++)
			{			
				if((sk60RxBuffer[findStatus]=='m')||(findStatus == (findStatus2 -2)))
				{
					valueBuf[rxi]= '\0';
					break;
				}
				else if((sk60RxBuffer[findStatus] == '-')&&(rxi==0))
				{
					valueBuf[rxi] = sk60RxBuffer[findStatus];
				}
				else if((sk60RxBuffer[findStatus]=='.')||((sk60RxBuffer[findStatus]<='9')&&(sk60RxBuffer[findStatus]>='0')))
				{						
					valueBuf[rxi]=sk60RxBuffer[findStatus];
				}
				findStatus++;
			}			
			tmpDis = atof(valueBuf);
			findStatus = findStatus2;
			for(rxi=0;rxi<8;rxi++)valueBuf[rxi] = '\0';
			for(rxi=0;rxi<5;rxi++)
			{	
				if((sk60RxBuffer[findStatus]=='\r')&&(sk60RxBuffer[findStatus]=='\n'))
				{
					valueBuf[rxi] = '\0';
					break;
				}
				else if((sk60RxBuffer[findStatus]<='9')&&(sk60RxBuffer[findStatus]>='0'))
				{						
					valueBuf[rxi]=sk60RxBuffer[findStatus];
				}
				findStatus++;
			}			
			tmpRiss = atoi(valueBuf);
		#if(SK60_DEBUG>1)
			printf("\r\nsk60 distance %s:%.3fm,riss:%d",&cmd,tmpDis,tmpRiss);
		#endif			
			if((tmpDis>SK60_DIS_MAX)||(tmpDis<SK60_DIS_MIN))
			{
				tmpDis = 0.0;
			}
			else 
			{
				tmpErr = 0;
			}
		}
		else if((sk60RxBuffer[findStatus]=='E')&&(sk60RxBuffer[findStatus+1]=='r')&&(sk60RxBuffer[findStatus+4]=='!'))
		{			
			tmpErr  = (sk60RxBuffer[findStatus+2]-0x30)*10;
			tmpErr += (sk60RxBuffer[findStatus+3]-0x30);
			
		#if(SK60_DEBUG>0)
			printf("\r\nsk60 %s measure Err:%d %d: ",&cmd,tmpErr,sk60RxPtrStore);
			for(rxi=0;rxi<sk60RxPtrStore;rxi++)printf("%c",sk60RxBuffer[rxi]);
//			printf("\r\n");
		#endif
		}
		rxResult = 0;
	}
	else
	{
		rxResult = -1;
	#if(SK60_DEBUG>0)
		printf("\r\nsk60 %s measure cmd Err! %d ",&cmd,sk60RxPtrStore);
		for(rxi=0;rxi<sk60RxPtrStore;rxi++)printf("%c",sk60RxBuffer[rxi]);
		printf("\r\n");
	#endif
	}
//	sk60RxBufferClear();
	*distance = tmpDis;
	*riss 		= tmpRiss;
	*err			= tmpErr;
	return rxResult;
}
void sk60waitTimeSec(uint8_t times)
{
	uint8_t sk60i		=	0;	
	for(sk60i=0;sk60i<times;sk60i++)
	{
		sk60_delay_ms(500);
	}			
}
static void sk60Reset(uint8_t powerOffTime)
{
	sk60UartLowPower();
	sk60PowerOff();
	sk60waitTimeSec(powerOffTime);
	sk60PowerOn();
	sk60Usart_Config();
	sk60waitTimeSec(2);
}
static void sk60LedRun(uint16_t runTime)
{
	if(!sk60LedControl(SK60_CMD_ON))
	{
		sk60waitTimeSec(runTime);
		sk60LedControl(SK60_CMD_OFF);
		sk60waitTimeSec(10);
	}
}
uint16_t sk60DisErrTimes 		= 0;
uint16_t sk60OffsetErrTimes = 0;
void sk60Task(uint8_t sk60Measurecmd)
{
	uint8_t sk60i=0,sk60j=0;	
	sk60_t	sk60Vsum;
	sk60_t	sk60Vmax;
	sk60_t	sk60Vmin;
	uint8_t sk60OkTimes=0;
	float 	offsetDistance = 0.0;//210820

	sk60UsartStatus	=	1;
	sk60RxBufferClear();
	sk60PowerOn();
	sk60usartBand	=	SK60_USART_Baud;
	sk60Usart_Config();
	sk60_delay_ms(1000);
	for(sk60i=0;sk60i<=SK60_BUFFER_SIZE;sk60i++)sk60Buffer[sk60i]=sk60VInit;	
	sk60Vsum = sk60VInit;
	if((devPar.sk60Fuction == FUNCITONG_SK60_DISP)||(devPar.sk60Fuction == FUNCITONG_SK60_RAW))//210707 FUNCITONG_SK60_RAW_210820
	{
		if(keyPressTimes>2)
		{
			keyPressTimes	= 0;
			sk60TestMode 	= FUNCITONG_TURN_ON;
			sk60InitFlag 	= SK60_PRESS_FLAG;
		}
		if(sk60TestMode == FUNCITONG_TURN_ON)
		{			
			sk60LedRun(180);//240 180_210820
			sk60TestTimes++;
			if(sk60TestTimes>=10)
			{
				sk60TestMode 	= 0;
				sk60TestTimes	=	0;
				sk60InitFlag  = SK60_INIT_FLAG;
			}
		}
		else if(sk60TestTimes>0){sk60TestTimes	=	0;}
	}else{sk60TestMode = 0;keyPressTimes	= 0;}
	if(!sk60LedControl(SK60_CMD_OFF))//SK60_CMD_ON SK60_CMD_OFF_210729
	{
		for(sk60j=0;sk60j<2;sk60j++)
		{
			sk60Vmax 	= sk60VInit;
			sk60Vmin 	= sk60VInit;
			sk60Vsum 	= sk60VInit;	//210820
			sk60OkTimes	=	0;			//210708			
			for(sk60i=0;sk60i<=SK60_BUFFER_SIZE;sk60i++)
			{
				sk60Buffer[sk60i]=sk60VInit;//210820
				sk60_delay_ms(100);//210729
				if(!sk60ReadStatus(&sk60Buffer[sk60i].t,&sk60Buffer[sk60i].v))
				{
					sk60_delay_ms(100);//210729
					if(!sk60Measure(sk60Measurecmd,&sk60Buffer[sk60i].dis,&sk60Buffer[sk60i].riss,&sk60Buffer[sk60i].err))
					{
//						sk60LedControl(SK60_CMD_ON);//20210729
						if(sk60Buffer[sk60i].err == 0)
						{
							if((sk60Vmax.t == 0.0)&&(sk60Vmin.t == 0.0)&&(sk60Vmax.dis == 0.0)&&(sk60Vmin.dis == 0.0))
							{
								sk60Vmin.t 		= sk60Buffer[sk60i].t;
								sk60Vmin.v 		= sk60Buffer[sk60i].v;
								sk60Vmin.dis	= sk60Buffer[sk60i].dis;
								sk60Vmax.t 		= sk60Buffer[sk60i].t;
								sk60Vmax.v 		= sk60Buffer[sk60i].v;
								sk60Vmax.dis	= sk60Buffer[sk60i].dis;							
								sk60Vsum.err 	=	0;
							}
							if(sk60Vmin.t > sk60Buffer[sk60i].t)			{sk60Vmin.t = sk60Buffer[sk60i].t;}
							else if(sk60Vmax.t < sk60Buffer[sk60i].t)	{sk60Vmax.t = sk60Buffer[sk60i].t;}
							
							if(sk60Vmin.v > sk60Buffer[sk60i].v)			{sk60Vmin.v = sk60Buffer[sk60i].v;}
							else if(sk60Vmax.v < sk60Buffer[sk60i].v)	{sk60Vmax.v = sk60Buffer[sk60i].v;}
							
							if(sk60Vmin.dis > sk60Buffer[sk60i].dis)			{sk60Vmin.dis = sk60Buffer[sk60i].dis;}
							else if(sk60Vmax.dis < sk60Buffer[sk60i].dis)	{sk60Vmax.dis = sk60Buffer[sk60i].dis;}
							
							sk60Vsum.t 		+= sk60Buffer[sk60i].t;
							sk60Vsum.v 		+= sk60Buffer[sk60i].v;
							sk60Vsum.dis	+= sk60Buffer[sk60i].dis;
							sk60Vsum.riss	+= sk60Buffer[sk60i].riss;							
							sk60OkTimes++;
						#if(SK60_DEBUG>0)
							printf("\r\nsk60 %d_%d:%s,dis:%.3fm,",SK60_BUFFER_SIZE+1,sk60i+1,&sk60Measurecmd,sk60Buffer[sk60i].dis);
							printf("t:%.1f¡æ,v:%.1fV,",sk60Buffer[sk60i].t,sk60Buffer[sk60i].v);
							printf("riss:%04d\r\n",sk60Buffer[sk60i].riss);
						#endif
						}
						else if(sk60Buffer[sk60i].err != 0xFF)
						{
							sk60Vsum.err	= sk60Buffer[sk60i].err;
//							sk60Vsum.riss	= sk60Buffer[sk60i].riss;// 210729ÆÁ±Î
						#if(SK60_DEBUG>0)
							printf("\r\nsk60 Err %d_%d:%d riss:%d\r\n",SK60_BUFFER_SIZE+1,sk60i+1,sk60Vsum.err,sk60Buffer[sk60i].riss);
						#endif
						}
//						sk60_delay_ms(200);//210729
					}						
				}			
			}
			
			if(sk60OkTimes>1)
			{
				sk60Vsum.riss/=sk60OkTimes;
				sk60Vsum.t 	-= sk60Vmax.t;
				sk60Vsum.v 	-= sk60Vmax.v;
				sk60Vsum.dis-= sk60Vmax.dis;
				sk60OkTimes--;
				if(sk60OkTimes>1)
				{
					sk60Vsum.t 	-= sk60Vmin.t;
					sk60Vsum.v 	-= sk60Vmin.v;
					sk60Vsum.dis-= sk60Vmin.dis;
					sk60OkTimes--;
				}
				if(sk60OkTimes>1)
				{
					sk60Vsum.t 	/= sk60OkTimes;
					sk60Vsum.v 	/= sk60OkTimes;
					sk60Vsum.dis/= sk60OkTimes;
				}
			}
			if(calCounter == 1){sk60Value.dis = sk60Vsum.dis;}
			if((sk60j == 0)&&((sk60OkTimes == 0)||(fabs(sk60Vsum.dis - sk60Value.dis)>SK60_DISP_MAX)))//210820
			{
				sk60Reset(5);
				sk60OkTimes = 0;
			}
			if((devPar.sk60Fuction == FUNCITONG_SK60_DISP)||(devPar.sk60Fuction == FUNCITONG_SK60_RAW))//210707 FUNCITONG_SK60_RAW_210820
			{
				if(keyPressTimes>2)
				{
					keyPressTimes = 0;
					sk60TestMode 	= FUNCITONG_TURN_ON;
					sk60InitFlag 	= SK60_PRESS_FLAG;
					sk60LedRun(180);//240 180_210820
					sk60TestTimes++;				
				}else{if(sk60OkTimes>0)break;}//210708
			}
			else{if(sk60OkTimes>0)break;}
		}
	}
	else if((devPar.sk60Fuction == FUNCITONG_SK60_DISP)||(devPar.sk60Fuction == FUNCITONG_SK60_RAW))//210707 FUNCITONG_SK60_RAW_210820
	{
		if(sk60TestMode == FUNCITONG_TURN_ON)sk60TestTimes++;
	}
	sk60Value	= sk60Vsum;
	
	sk60UartLowPower();
	sk60PowerOff();
	sk60UsartStatus	=	0;
	
	if(sk60InitFlag == SK60_INIT_FLAG)
	{
		if(sk60Value.dis>0.0)
		{
			if((devPar.sk60Fuction == FUNCITONG_SK60_DISP)||(devPar.sk60Fuction == FUNCITONG_SK60_RAW))//210707 FUNCITONG_SK60_RAW_210820
			{
				sk60BaseDistance 		= sk60Value.dis;				
				sk60OffsetDistance	= 0.0;	
				sk60DisErrTimes			= 0;
				sk60OffsetErrTimes	= 0;
			#if(SK60_CTR>0)
				sk60OffsetValueSave(sk60BaseDistance);//210820
			#endif
			}					
		}
	}
	else
	{
		if((sk60InitFlag == SK60_SET_FLAG)||(sk60InitFlag == SK60_PRESS_FLAG)){}//210705
		else if(sk60Measurecmd == SK60_CMD_MEASURE_AUTO){sk60InitFlag = 1;}
		else if(sk60Measurecmd == SK60_CMD_MEASURE_SLOW){sk60InitFlag = 2;}
		else if(sk60Measurecmd == SK60_CMD_MEASURE_FAST){sk60InitFlag = 3;}
	}
	if(sk60Value.dis>0.0)//&&(sk60BaseDistance>0.0) //210705  //210820
	{
		offsetDistance 	= sk60Value.dis-sk60BaseDistance;		
		if((devPar.sk60Fuction == FUNCITONG_SK60_DISP)&&(sk60BaseDistance==0)){sk60OffsetDistance=0;}
		else if(devPar.sk60Fuction == FUNCITONG_SK60_RAW){sk60OffsetDistance 	= offsetDistance;}
		else if((calCounter == 1)||(fabs(offsetDistance-sk60OffsetDistance)<SK60_DISP_MAX)||(fabs(offsetDistance-sk60OffsetDistanceLast)<SK60_DISP_MAX))
		{
			sk60OffsetErrTimes 	= 0;
			sk60DisErrTimes			=	0;	
			sk60OffsetDistance 	= offsetDistance;
		}
		else
		{
			sk60DisErrTimes++;
			sk60OffsetErrTimes++;
			if(sk60OffsetErrTimes>=3)
			{
				sk60OffsetErrTimes	=	0;
				sk60OffsetDistance 	= offsetDistance;					
			}
		}
		if(sk60OffsetDistance!=0)sk60OffsetDistanceLast = sk60OffsetDistance;//210822
	}
	else
	{
		if(devPar.sk60Fuction == FUNCITONG_SK60_RAW)sk60DisErrTimes=6;
		sk60DisErrTimes++;			
		if(sk60DisErrTimes>=6)
		{
			sk60DisErrTimes 		= 0;
//			if(sk60OffsetDistance!=0)sk60OffsetDistanceLast = sk60OffsetDistance;
			sk60OffsetDistance 	= 0;
		}
	}
	
	if(sk60OkTimes == 0)//210820
	{
		for(sk60i=0;sk60i<=SK60_BUFFER_SIZE;sk60i++)
		{
			if(sk60Buffer[sk60i].t!=0){sk60Value.t = sk60Buffer[sk60i].t;break;}
		}
	}
//	if((sk60Value.err != 0)&&(sk60OkTimes == 0)){sk60Value.t = sk60Buffer[sk60i-1].t;sk60Value.riss = sk60Buffer[sk60i-1].riss;}//210708 210729_sk60i-1
#if(SK60_DEBUG>0)
	printf("\r\nsk60Value %d,riss:%04d,t:%.2f¡æ,dis:%.4fm,",sk60Value.err,sk60Value.riss,sk60Value.t,sk60Value.dis);
	printf("v:%.2fV,baseDis:%.4fm,¡÷dis:%.4fm\r\n",sk60Value.v,sk60BaseDistance,sk60OffsetDistance);
#endif
	if(sk60Value.err>55)sk60Value.err = 55;
	sk60Value.v = sk60BaseDistance;	//sk60OffsetDistance sk60BaseDistance_210705
//	sk60Displacement	=	sk60Value.dis;	//210705
}

#if(KXYL_CTR>0)
	#include "crc16.h"
	#include "time.h"
	#define kxyl_id 						1
	#define kxyl_cmd						0x04

	#define KXYL_AVG_TIMES			10
	#define KXYL_SETTLINGTIME		250// 250*20ms=5s
	#define KXYL_OVERTIME				500
//-100~200k 0~65535
//	#define KXYL_ADC_MAX	65535
//	#define KXYL_V_MIN		-100.0
//	#define KXYL_V_MAX		200.0
//	#define KXYL_COF_A		300.0/65535
//	#define KXYL_COF_B		-100.0
	
	#define KXYL_STATE_SLEEP	0x00//PM25_STATE_SLEEP
	#define KXYL_STATE_RUN		0x01//PM25_STATE_RUN
	#define KXYL_STATE_TX			0x02//PM25_STATE_TX
	#define KXYL_STATE_RX			0x03//PM25_STATE_RX

	#define kxylRxBuffer			sk60RxBuffer
	#define KXYL_BUFFER_SIZE	SK60_RX_BUFFER_SIZE
	#define kxylRxOutPtr			sk60RxPtrOut
	#define kxylRxStorePtr		sk60RxPtrStore		
	#define kxylusartBaud			sk60usartBand
	#define kxyl_USART_Baud		9600
	#define kxylUSART_RUN			sk60Usart_Config
	#define	kxylUSART_SLEEP		sk60UartLowPower
	#define kxylStoreBuffer 	sk60StoreBuffer

//	#define kxylTimerCounter	pm25TimerCounter
//	#define kxylPowerStatus		pm25PowerStatus

//	#define	kxylRxRun				pm25RxRun
	#define	kxyl_delay_ms			sk60_delay_ms
	#define kxylClearBuffer		sk60RxBufferClear
//	#define kxylPowerOff			pm25PowerOff	
	#define	kxyl_NVIC_TIMER_Status	NVIC_Time3_Status
	#define	kxyl_TIMER_Init_Status	Time3_Init_Status
	
	#define KXYL_EN_PIN					GPIO_Pin_4
	#define KXYL_EN_Source			GPIOC
	#define KXYL_EN_RCC					RCC_AHBPeriph_GPIOC
	#define KXYL_EN_ON()				KXYL_EN_Source->BSRRL = KXYL_EN_PIN	//1
	#define KXYL_EN_OFF()				KXYL_EN_Source->BSRRH = KXYL_EN_PIN	//0
	
	uint16_t kxylBuf[KXYL_SENSOR_NUM];
	float kxylValue[KXYL_SENSOR_NUM];
	float kxylParam[KXYL_SENSOR_NUM][2];
		
	uint8_t		kxylPowerStatus	=	STATUS_NONE;
	uint8_t		kxylusartStatus	=	KXYL_URX;
	
	void kxylPowerOn(void)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Enable PIN clock */
		RCC_AHBPeriphClockCmd(KXYL_EN_RCC, ENABLE); 
		/* pins configuration */
		GPIO_InitStructure.GPIO_Pin 	= KXYL_EN_PIN;
		GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(KXYL_EN_Source, &GPIO_InitStructure); 
		KXYL_EN_ON();
		kxylPowerStatus	=	STATUS_DONE;
	}
	void kxylPowerOff(void)
	{
		GPIO_InitTypeDef GPIO_InitStructure;	
		kxylPowerStatus	=	STATUS_NONE;
		KXYL_EN_OFF();
		 /* Enable PIN clock */
		 RCC_AHBPeriphClockCmd(KXYL_EN_RCC, ENABLE);
		/* pins configuration */
		GPIO_InitStructure.GPIO_Pin 	= 	KXYL_EN_PIN;
		GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd  =   GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_OType =   GPIO_OType_OD;//
		GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_400KHz;		
		GPIO_Init(KXYL_EN_Source, &GPIO_InitStructure); 	
	}
	
	void kxylRxRun(void)
	{
	#if(KXYL_DEBUG>0)
		if(kxylusartkey == STATUS_DONE)printf("\r\nkxylUSART run:%d\r\n",kxylTimerCounter);		
	#endif
		kxylusartBaud		=	kxyl_USART_Baud;	
		kxylRxStatus		=	KXYL_URX;
		kxylTimerCounter	=	0;	
		kxylUSART_RUN();	
	}

	void kxylRun(void)
	{		
		kxylPowerOn();
		kxylClearBuffer();
		kxylTimerCounter = 0;
		kxylTimerRx = 0;
		kxylRxStatus 	= KXYL_URX;
		kxylusartStatus		=	KXYL_URX;
		kxyl_NVIC_TIMER_Status(TIME_START);
		kxyl_TIMER_Init_Status(TIME_START);	
	}
	void kxylSleep(void)
	{
		kxylPowerOff();
		kxylUSART_SLEEP();
		kxyl_NVIC_TIMER_Status(TIME_END);
		kxyl_TIMER_Init_Status(TIME_END);	
	}
	void kxylTimeDeal(void)
	{
		kxylTimerCounter++;
		if(kxylRxStatus == KXYL_RXSTATE)
		{
			kxylTimerRx++;
			if(kxylTimerRx>KXYL_RX_OVERTIME)
			{
				kxylTimerRx		= 0; 
				kxylRxStatus 	= KXYL_URX;
				kxylusartStatus		=	KXYL_RXOVER;
			}
		}
	}

	void kxylTxTask(uint8_t num)	//221128
	{//01 04 01 00 00 02 70 37
		uint8_t buf[10];
		value16_t crc16;
		buf[0]	=	kxyl_id;
		buf[1]	=	kxyl_cmd;
		buf[2]	=	0x01;
		buf[3]	=	0x00;
		buf[4]	=	0x00;
		buf[5]	=	num;//0x02;
		crc16.i	= CRC16_1(buf,6);
		buf[6]	=	crc16.bytes.valueL;//0x70;
		buf[7]	=	crc16.bytes.valueH;//0x37;
		kxylStoreBuffer(buf,8);
		kxylRxOutPtr			=	0;
		kxylTimerCounter	=	0;
		kxylRxStatus 			= KXYL_URX;
	}
	
	int8_t kxyRxTask(value16_t *dataOut)
	{
		int8_t 	result 		= -1;
		uint8_t ptrOut		=	0;
		uint8_t ptrStore	=	0;
		uint8_t counter		=	0;
		value16_t	data[KXYL_SENSOR_NUM];
		value16_t crc16;
		uint8_t rxi=0,rxj=0;
		if(kxylusartStatus ==	KXYL_RXOVER)
		{
			ptrOut		=	kxylRxOutPtr;
			ptrStore	=	kxylRxStorePtr;
			if(ptrStore>=ptrOut)
			{
				counter = ptrStore - ptrOut;
			}
			else
			{
				counter = KXYL_BUFFER_SIZE +1+ ptrStore - ptrOut;
			}
//			printf("\r\n ts %d:",counter);
//			devStoreBuffer(kxylRxBuffer+ptrOut,counter);
//			printf("te\r\n");
			while(counter>=(5+(KXYL_SENSOR_NUM<<1)))
			{
				if(kxylRxBuffer[ptrOut] == kxyl_id)
				{
					if(kxylRxBuffer[ptrOut+1] == kxyl_cmd)
					{
						if(kxylRxBuffer[ptrOut+2] == (KXYL_SENSOR_NUM<<1))
						{
							rxj= 3;
							for(rxi=0;rxi<KXYL_SENSOR_NUM;rxi++)	//221128
							{
								data[rxi].bytes.valueH = kxylRxBuffer[ptrOut+3+(rxi<<1)];
								data[rxi].bytes.valueL = kxylRxBuffer[ptrOut+4+(rxi<<1)];
								rxj += 2;
							}
							crc16.i	= CRC16_1((uint8_t *)(kxylRxBuffer+ptrOut),rxj);
							
//							data[0].bytes.valueH = kxylRxBuffer[ptrOut+3];
//							data[0].bytes.valueL = kxylRxBuffer[ptrOut+4];
//							data[1].bytes.valueH = kxylRxBuffer[ptrOut+5];
//							data[1].bytes.valueL = kxylRxBuffer[ptrOut+6];

//							crc16.i	= CRC16_1((uint8_t *)(kxylRxBuffer+ptrOut),7);
							
							if((crc16.bytes.valueL == kxylRxBuffer[ptrOut+rxj])&&(crc16.bytes.valueH == kxylRxBuffer[ptrOut+rxj+1]))//7 8
							{
								counter			 -= (rxj+1);//8
								kxylRxOutPtr	= (ptrOut+rxj+2)&KXYL_BUFFER_SIZE;//9							
								result = 0;
								for(rxi=0;rxi<KXYL_SENSOR_NUM;rxi++)
								{
									dataOut[rxi] = data[rxi];
								}								
							}
						}
					}
					ptrOut++;
					ptrOut &= KXYL_BUFFER_SIZE;
					counter--;
				}
			}
			kxylusartStatus = KXYL_URX;
		}
		return result;
	}
	
	void kxylTask(void)
	{
		uint8_t kxylState	=	KXYL_STATE_RUN;
		uint8_t kxylok[KXYL_SENSOR_NUM]	= {0,0};
		uint8_t kxylokTimes = 0;
		value16_t	data[KXYL_SENSOR_NUM]			= {0,0};
		uint32_t	dataSum[KXYL_SENSOR_NUM] 	= {0,0};
		uint8_t kxyli=0;
		
		kxylusartkey = STATUS_DONE;
		for(kxyli = 0;kxyli<KXYL_SENSOR_NUM;kxyli++)
		{
			kxylok[kxyli] 		= 0;
			kxylBuf[kxyli] 		= 0;
			kxylValue[kxyli] 	= 0.0;
		}
		kxylRun();
		
		if(kxylPowerStatus == STATUS_DONE)
		{
			if((kxylTimerCounter >= KXYL_SETTLINGTIME))
			{
				kxylRxRun();
				kxylState	=	KXYL_STATE_TX;
			}
			else
			{
			#if(KXYL_DEBUG>0)
			//	if(debugStatus == DEBUG_STATUS_DONE)
				{
					printf("\r\n enter kxyl task:%d\r\n",KXYL_SETTLINGTIME-kxylTimerCounter);
				}
			#endif
			}
			while(kxylTimerCounter < KXYL_OVERTIME)
			{
				switch(kxylState)
				{
					case KXYL_STATE_RUN:
					{
						if((kxylTimerCounter >= KXYL_SETTLINGTIME))
						{
							kxylRxRun();		
							kxylState	=	KXYL_STATE_TX;						
						}
						else
						{
						//	printf("\r\n kxylTimer: %d\r\n", kxylTimerCounter);
							kxyl_delay_ms(2);
						}
						break;
					}
					case KXYL_STATE_TX:
					{
						kxylTxTask(KXYL_SENSOR_NUM);						
						kxylState	=	KXYL_STATE_RX;
					}
					case KXYL_STATE_RX:
					{
						if(kxyRxTask(data)==0)
						{
							kxylokTimes++;
						#if(KXYL_DEBUG>0)
							if(debugStatus == DEBUG_STATUS_DONE)
							{
								if(kxylokTimes==1)
									printf("\r\nkxyl data %d:",kxylokTimes);
							}
						#endif
							for(kxyli = 0;kxyli<KXYL_SENSOR_NUM;kxyli++)	//221128
							{
								dataSum[kxyli] += data[kxyli].i;
								if(data[kxyli].i>0)kxylok[kxyli]++;
							#if(KXYL_DEBUG>0)
								if(debugStatus == DEBUG_STATUS_DONE)
								{
									if(kxylokTimes==1)
										printf("%d ",data[kxyli].i);
								}
							#endif
							}
//							dataSum[0] += data[0].i;
//							dataSum[1] += data[1].i;
//							kxylokTimes++;
//							if(data[0].i>0)kxylok[0]++;
//							if(data[1].i>0)kxylok[1]++;
//						#if(KXYL_DEBUG>0)							
//							{
//								if(kxylokTimes==1)printf("\r\n kxyl data %d:%d,%d",kxylokTimes,data[0].i,data[1].i);
//							}
//						#endif
							if(kxylokTimes >= KXYL_AVG_TIMES)
							{								
								kxylState	=	KXYL_STATE_SLEEP;
							}
							else
							{	
								kxyl_delay_ms(100);								
								kxylState	=	KXYL_STATE_TX;
							}	
						}
						else
						{
							kxyl_delay_ms(2);
						}
						break;
					}
					case KXYL_STATE_SLEEP:
					{
						if(kxylokTimes >0)
						{//-100~200k 0~65535
							for(kxyli = 0;kxyli<KXYL_SENSOR_NUM;kxyli++)	//221128
							{
								kxylBuf[kxyli] = dataSum[kxyli]/kxylok[kxyli];
								if(kxylBuf[kxyli]>0)
								{									
									kxylValue[kxyli] = kxylBuf[kxyli]*kxylParam[kxyli][0]+kxylParam[kxyli][1];									
								}
							}
//							kxylBuf[0] = dataSum[0]/kxylok[0];
//							kxylBuf[1] = dataSum[1]/kxylok[1];
//							if(kxylBuf[0]>0)kxylValue[0] = kxylBuf[0]*KXYL_COF_A+KXYL_COF_B;
//							if(kxylBuf[1]>0)kxylValue[1] = kxylBuf[1]*KXYL_COF_A+KXYL_COF_B;							
						}
					}
					default:
					{
						kxylTimerCounter = KXYL_OVERTIME;
						break;
					}
				}
			}
		#if(KXYL_DEBUG>0)		
			{
//				printf("\r\n kxyl value:%.3f,%.3f,buf:%d,%d\r\n",kxylValue[0],kxylValue[1],kxylBuf[0],kxylBuf[1]);
				printf("\r\nkxyl value ");
				for(kxyli = 0;kxyli<KXYL_SENSOR_NUM;kxyli++)
				{
					printf("%d:%d,%.3f ",kxyli+1,kxylBuf[kxyli],kxylValue[kxyli]);
				}
			}
		#endif			
		}
		kxylSleep();
		kxylusartkey = STATUS_NONE;
	}
#endif

/*****************************END OF FILE*****************************/

