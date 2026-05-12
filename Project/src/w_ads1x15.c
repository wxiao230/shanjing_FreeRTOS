/**
  ******************************************************************************
  * @file    Project/user/W_ADS1x15.c 
  * @author  MCD Application Team
  * @version V1.0.5
  * @date    28-October-2013
  * @brief  
  ******************************************************************************    
  */  
/* Includes ------------------------------------------------------------------*/
	#include "w_ads1x15.h"
//	#include "main.h"  
	#include "delay.h"
	
/* Private typedef -----------------------------------------------------------*/  
/* Private define ------------------------------------------------------------*/ 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/ 
//电池电压
uint16_t adsbatValue = 0;
//锂电池充电电压
//	__IO uint16_t adschargeValue = 0;
//外部电池输入电压
float adsexterValue = 0;



#define ADS1x15_DEBUG    0
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/ 

/*ADS1x15_RUN*/
	void ADS1x15_RUN(void)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
	#if(ADS_I2C_STM32L>0)
		I2C_InitTypeDef I2C_InitStructure;
	#endif
		/* Enable ADS1x15 clock */
		RCC_AHBPeriphClockCmd(ADS1x15_EN_RCC|ADS_RDY_RCC, ENABLE); 
		/* ADS1x15 pins configuration */
		GPIO_InitStructure.GPIO_Pin 	= ADS1x15_EN_PIN;
		GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  	= GPIO_PuPd_NOPULL;	
		GPIO_Init(ADS1x15_EN_Source, &GPIO_InitStructure);
		
		/* ADS_RDY pins configuration 	 	  
		GPIO_InitStructure.GPIO_Pin 	= ADS_RDY_PIN;	
		GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;  
		GPIO_Init(ADS_RDY_Source, &GPIO_InitStructure);  */   		
	#if(ADS_I2C_STM32L>0)
		/*!< I2C Periph clock enable  */
		I2C_DeInit(ADS_I2C);		  
		/* Enable the ADS_I2C peripheral */			
		RCC_APB1PeriphClockCmd(ADS_I2C_RCC, ENABLE);
		/* Enable ADS1x15 SCL SDA clock */
		RCC_AHBPeriphClockCmd(ADS_PIN_RCC, ENABLE);
		/* ADS_SCL SDA pins configuration */
		GPIO_PinAFConfig(ADS_Source,ADS_SCL_PinSource,ADS_GPIO_AF_I2C); 
		GPIO_PinAFConfig(ADS_Source,ADS_SDA_PinSource,ADS_GPIO_AF_I2C); 

		GPIO_InitStructure.GPIO_Pin   = ADS_SCL_PIN|ADS_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(ADS_Source, &GPIO_InitStructure); 
		
		I2C_InitStructure.I2C_ClockSpeed			= I2C_SPEED;
		I2C_InitStructure.I2C_Mode					= I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle				= I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1			= OwnAddress;
		I2C_InitStructure.I2C_Ack					= I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit;	  	
		I2C_Init(ADS_I2C,&I2C_InitStructure);		
		/* I2C ENABLE */ 		
		I2C_Cmd(ADS_I2C, ENABLE);
	#else
		GPIO_InitStructure.GPIO_Pin   = ADS_SCL_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(ADS_Source, &GPIO_InitStructure); 
		ADS_SCL_LOW();
		GPIO_InitStructure.GPIO_Pin   = ADS_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
		GPIO_Init(ADS_Source, &GPIO_InitStructure); 
		GPIO_ResetBits(ADS_Source,ADS_SDA_PIN);
		ADS_SCL_HIGH();	
	#endif
//		ADS1x15_Enable(); 		
//		ADS1x15_Reset();//		 	
	}
/*ADS1x15_SLEEP*/
	void ADS1x15_SLEEP(void)
	{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  ADS1x15_Disable();
// 	#if(ADS_I2C_STM32L>0)
// 	  I2C_Cmd(ADS_I2C, DISABLE);
// 	  RCC_APB1PeriphClockCmd(ADS_I2C_RCC, DISABLE);
// 	#endif 
// 	   /* Enable ADS1x15 clock */
// 	   RCC_AHBPeriphClockCmd(ADS1x15_EN_RCC|ADS_PIN_RCC|ADS_RDY_RCC, ENABLE);
// 	  /* SOLAR_UV_ADC pins configuration */
// 	  GPIO_InitStructure.GPIO_Pin 	= 	ADS1x15_EN_PIN;
// 	  GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AN;
// 	  GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_400KHz;
// 	  GPIO_InitStructure.GPIO_OType =   GPIO_OType_OD;
//   	  GPIO_InitStructure.GPIO_PuPd  =   GPIO_PuPd_UP;		
// 	  GPIO_Init(ADS1x15_EN_Source, &GPIO_InitStructure); 
// 	  /* ADS_RDY pins configuration   
// 	  GPIO_InitStructure.GPIO_Pin 	= 	ADS_RDY_PIN;  
// 	  GPIO_Init(ADS_RDY_Source, &GPIO_InitStructure); */
// 	#if(ADS_I2C_STM32L>0)	
// 	   /* Disable the GPRS_USART peripheral */
// 	  RCC_APB1PeriphClockCmd(ADS_I2C_RCC, DISABLE); 
// 	#endif
// 	  /* ADS_SDA SCL pins configuration */
// 	  GPIO_InitStructure.GPIO_Pin 	= 	ADS_SDA_PIN|ADS_SCL_PIN;  
// 	  GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_AN;
// 	  GPIO_InitStructure.GPIO_Speed = 	GPIO_Speed_400KHz;
// 	  GPIO_InitStructure.GPIO_OType =   GPIO_OType_OD;
//   	  GPIO_InitStructure.GPIO_PuPd  =   GPIO_PuPd_UP;	      
// 	  GPIO_Init(ADS_Source, &GPIO_InitStructure);  

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  //RCC_APB2Periph_AFIO
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; 
  GPIO_Init(GPIOB,&GPIO_InitStructure);
	}

#if(ADS_I2C_STM32L>0)
/*I2C_WaitEvent*/
	ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, uint32_t I2C_Event)
	{
	  uint16_t i;
	
	  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
	    if (I2C_CheckEvent(I2Cx, I2C_Event) == SUCCESS)
	        return SUCCESS;
	
	  return ERROR;
	} 

/*I2C_Start_Transmitter*/
	uint8_t I2C_Start_Transmitter(uint8_t addr,uint8_t frist_byte)
	{
		uint8_t  status = I2C_OK;
		uint16_t i      = I2C_MAX_CHECK_CNT; 
		while((I2C_GetFlagStatus(ADS_I2C,I2C_FLAG_BUSY))&&(--i));//等待总线空闲	
		if(i==0)status=I2C_BUSY_ERR;	
		
		I2C_GenerateSTART(ADS_I2C,ENABLE);			  

		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_MODE_SELECT))status=I2C_MASTERSELECT_ERR;//return(ERROR);//等待开始	 Ev5 SB=1			
		I2C_Send7bitAddress(ADS_I2C,addr,I2C_Direction_Transmitter);
		
		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))status=I2C_MODESELECT_ERR;//return(ERROR);//Ev6 ADDR=1	;
		//I2C_Cmd(ADS_I2C, ENABLE);		   
		I2C_SendData(ADS_I2C,frist_byte);
		return status;	
	}
/*I2C_Stop_Transmitter*/
	uint8_t I2C_Stop_Transmitter(void)
	{
		uint8_t status=I2C_OK;
		uint16_t i=I2C_MAX_CHECK_CNT; 
		
		while((!I2C_GetFlagStatus(ADS_I2C,I2C_FLAG_TXE))&&(--i));//等待最后字节发送完毕
		if(i==0)status=I2C_TRAN_ERR;
		I2C_GenerateSTOP(ADS_I2C,ENABLE);  		
		return status;	
	}
/*I2C_SendByte*/
	uint8_t I2C_SendByte(uint8_t byte)
	{
		uint8_t status=I2C_OK;
		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTING))status=I2C_SEND_ERR;//等待开始 DR寄存器为空Ev8_1 TXE=1		   
		I2C_SendData(ADS_I2C,byte); 
		return status;
	}	
 
 /*ADS1x15_ReadRegister*/
 	uint16_t ADS1x15_ReadRegister(void) //ADS1x15_REG_CONVERSION ADS1x15_REG_CONFIG
	{
		uint8_t 	status			= I2C_OK;
		uint16_t 	i				= I2C_MAX_CHECK_CNT;
		ads_t 		ConvertValue	= {0};
	
		for(i=0;i<I2C_MAX_CHECK_CNT;i++);	
		while((I2C_GetFlagStatus(ADS_I2C,I2C_FLAG_BUSY))&&(--i));//等待总线空闲	
		if(i==0)status=I2C_BUSY_ERR;
	//	if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED))status=I2C_SEND_ERR;//等待最后一个字节，Ev8_2 TXE=1、BTF=1   		
		I2C_GenerateSTART(ADS_I2C,ENABLE);
		
		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_MODE_SELECT))status=I2C_MASTERSELECT_ERR;//等待开始	 Ev5 SB=1			
		I2C_Send7bitAddress(ADS_I2C,ADS1x15_ADDR,I2C_Direction_Receiver); 
	  /*
		i=I2C_MAX_CHECK_CNT;
		while((!I2C_GetFlagStatus(ADS_I2C,I2C_FLAG_ADDR))&&(--i));//Ev6 ADDR=1	;
		if(i==0)status=I2C_MODESELECT_ERR;	
		I2C_NACKPositionConfig(ADS_I2C,I2C_NACKPosition_Next);
	  */
		I2C_AcknowledgeConfig(ADS_I2C,ENABLE);
			
		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED))status=I2C_MODESELECT_ERR;//等待最后一个字节，Ev8_2 TXE=1、BTF=1 
		I2C_AcknowledgeConfig(ADS_I2C,DISABLE);			
		ConvertValue.bytes.valueH=I2C_ReceiveData(ADS_I2C);
		
		if(!I2C_WaitEvent(ADS_I2C,I2C_EVENT_MASTER_BYTE_RECEIVED))status=I2C_RECEICE_ERR;//等待最后一个字节，Ev8_2 TXE=1、BTF=1   
	    ConvertValue.bytes.valueL=I2C_ReceiveData(ADS_I2C);

		
		I2C_GenerateSTOP(ADS_I2C,ENABLE);
	#if (ADS_TYPE==ADS1015_TYPE)
		ConvertValue.i>>=4;
	#endif
		if(status!=I2C_OK)ConvertValue.i=status+ADS_MAXVALUE-1;
	
		return (ConvertValue.i);
	}
#else
	/*ADS_SDA_LOW	
	void ADS_SDA_LOW(void)	//开漏输出低电平
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin   = ADS_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(ADS_Source, &GPIO_InitStructure); 
		ADS_Source->BSRRH=ADS_SDA_PIN;	
	}				  */
	/*ADS_SDA_HIGH 
	void ADS_SDA_HIGH(void)//输入模式
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin   = ADS_SDA_PIN;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(ADS_Source, &GPIO_InitStructure); 	
	}		 		 */
	/*ADS_delay_us*/
	void ADS_delay_us(uint16_t _us)
	{
		uint8_t i;
		while(_us--)
			for(i=32;i>0;i--);	
	/*	delay_1us(_us);		   */
	}						  
	/*ADS_delay_ms*/ 
	void ADS_delay_ms(uint16_t _ms)
	{
		uint16_t i;
		while(_ms--)
			for(i=32000;i>0;i--);  
	/*	delay_ms(_ms);			 */
	}
	/*ADS_I2C_Start*/
	void ADS_I2C_Start(void)
	{
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_HIGH();
		ADS_delay_us(1);
		ADS_SCL_HIGH();
		ADS_delay_us(1);
		ADS_SDA_LOW();
		ADS_delay_us(1);
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_HIGH();			
	}
	/*ADS_I2C_Stop*/
	void ADS_I2C_Stop(void)
	{
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_LOW();
		ADS_delay_us(1);
		ADS_SCL_HIGH();
		ADS_delay_us(1);
		ADS_SDA_HIGH();
		ADS_delay_us(1);
	//	ADS_SCL_LOW();
	} 	
	/*I2C_SendByte*/
	uint8_t I2C_SendByte(uint8_t byte)
	{
		uint8_t status=I2C_OK;
		uint8_t data;
		uint8_t i;
		data=byte;
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_HIGH();
		ADS_delay_us(1);
		for(i=8;i>0;i--)
		{										
			status=data&0x80;
			if((status)!=0x80)
			{
				ADS_SDA_LOW();
			} 			
			ADS_delay_us(1);
			ADS_SCL_HIGH();
			ADS_delay_us(2);
			ADS_SCL_LOW();
			data<<=1;
			ADS_SDA_HIGH();
			ADS_delay_us(1);
		}
		ADS_delay_us(1); 
		ADS_SCL_HIGH();		
		status=ADS_SDA_PORT();
		if(status!=0x00){status=I2C_NoACK_ERR;}
		else {status=I2C_OK;} 
		ADS_delay_us(2);
		return status;
	}
	/*I2C_ReadByte*/
	uint8_t I2C_ReadByte(void)
	{
		uint8_t data=0;
		uint8_t i; 
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_HIGH();
	//	ADS_delay_us(2);	
		for(i=8;i>0;i--)
		{ 		
			data<<=1;
			ADS_delay_us(1);
			ADS_SCL_HIGH();
			ADS_delay_us(1);
			data|=ADS_SDA_PORT();
			ADS_delay_us(1);
			ADS_SCL_LOW();
			ADS_delay_us(1);				
		}  	
		ADS_SDA_LOW();
		ADS_delay_us(1);
		ADS_SCL_HIGH();
		ADS_delay_us(2);	
		ADS_SCL_LOW();
		ADS_delay_us(1);
		ADS_SDA_HIGH();
		return data;
	}
	/*I2C_Start_Transmitter*/
	uint8_t I2C_Start_Transmitter(uint8_t addr,uint8_t frist_byte)
	{
		uint8_t status=I2C_OK;
		ADS_I2C_Start();
		status=I2C_SendByte(addr);
		status=I2C_SendByte(frist_byte);
		return status;	
	}
	/*I2C_Stop_Transmitter*/
	uint8_t I2C_Stop_Transmitter(void)
	{
		ADS_I2C_Stop();
		return I2C_OK;	
	}
	/*ADS1x15_ReadRegister*/
	uint16_t ADS1x15_ReadRegister(void)
	{
		uint8_t 	status			= I2C_OK;
		ads_t 		ConvertValue	= {0};
		ADS_I2C_Start();
		status=I2C_SendByte(ADS1x15_ADDR_READ);	
		ConvertValue.bytes.valueH=I2C_ReadByte();
		ConvertValue.bytes.valueL=I2C_ReadByte();
		ADS_I2C_Stop();
	#if (ADS_TYPE==ADS1015_TYPE)
		ConvertValue.i>>=4;
	#endif
		if(status!=I2C_OK)ConvertValue.i=status+ADS_MAXVALUE-1;
	
		return (ConvertValue.i);
	}
#endif
/*ADS1x15_WritePointer*/
	uint8_t ADS1x15_WritePointer(uint8_t addr,uint8_t PointerReg)
	{	
		uint8_t 	status	= I2C_OK;
		status=I2C_Start_Transmitter(addr,PointerReg); 
		status=I2C_Stop_Transmitter();
		return status;	
	}
/*ADS1x15_Reset*/
	uint8_t ADS1x15_Reset(void)
	{		 
		return(ADS1x15_WritePointer(ADS1x15_ADDR_RST,ADS1x15_RST_BYTE));   		 
	}
/*ADS1x15_Config*/
	uint8_t ADS1x15_Config(uint16_t ads_channel,uint16_t ads_pga)
	{	
		uint8_t 	status		= I2C_OK;
		ads_t 		ads_config	= {ADS1x15_CONFIG_Default};
	
		ads_config.i	&= (~ADS1x15_MUX_MASK);
		ads_config.i	|= ads_channel;			  
		ads_config.i	&= (~ADS1x15_PGA_MASK);
		ads_config.i	|= ads_pga;
		ads_config.i	|= ADS1x15_OS_BeginConvert;	  //开始转换
	//	ads_config.i	&= (~ADS1x15_MODE_MASK);	 //连续转换模式

		status=I2C_Start_Transmitter(ADS1x15_ADDR,ADS1x15_REG_CONFIG);
	   	status=I2C_SendByte(ads_config.bytes.valueH);
		status=I2C_SendByte(ads_config.bytes.valueL);
		status=I2C_Stop_Transmitter();
	
		return status;
	}
/*ADS1x15_ConvertAverage*/
	uint16_t ADS1x15_ConvertAverage(uint16_t ads_channel,uint16_t ads_pga)
	{
		uint32_t TempSum=0;
		uint16_t TempAverage=0;
		uint16_t Tempvalue;		
		uint16_t TempMax=0;
		uint16_t TempMin=0;
		uint16_t i;
		uint8_t j;
		ADS1x15_Reset(); 	  		
		for(j=0;j<10;j++)
		{										
			ADS1x15_Config(ads_channel,ads_pga); 
			for(i=0;i<I2C_MAX_CHECK_CNT;i++);
			ADS1x15_WritePointer(ADS1x15_ADDR,ADS1x15_REG_CONVERSION);	 			
			Tempvalue=ADS1x15_ReadRegister();
			if(Tempvalue>=ADS_MAXVALUE)return Tempvalue;
			TempSum+=Tempvalue;
			if(j==0){TempMax=Tempvalue;TempMin=Tempvalue;}
			if(Tempvalue>TempMax){TempMax=Tempvalue;}	
			if(Tempvalue<TempMin){TempMin=Tempvalue;}
			for(i=0;i<I2C_MAX_CHECK_CNT;i++);
			for(i=0;i<I2C_MAX_CHECK_CNT;i++); 
		}
		TempSum-=TempMax;
		TempSum-=TempMin;
		TempAverage=(uint16_t)(TempSum>>3);
		return(TempAverage);
	}


/*Read_BatValue*/
uint16_t Read_Value(uint16_t whichChannel)
{	 	
		uint16_t bValue;   
//		float channelValue = 0;
   	ADS1x15_RUN();

	bValue		= ADS1x15_ConvertAverage(whichChannel,ADS1x15_PGA_6144);//电池电压分压一半后采样
	ADS1x15_SLEEP();
#if defined (NEW_VERSION)
     if(SUN_channel ==whichChannel)
	 {
	 bValue = bValue*3;
	 }else
	 {
	 bValue = bValue*2;	 
	 }
#else
	 bValue = bValue*2;
#endif

	#if (ADS1x15_DEBUG>0)
		printf("\r\nbatValue:%d\r\n",bValue);
//		batValue=(float)bValue*6.144/ADS_MAXVALUE*2;
		printf("\r\n batValue:%f\r\n",(bValue*6.144/ADS_MAXVALUE));
	#endif	 
	 return bValue;
}

/*Read_BatValue*/

/*
void Read_BatValue(void)
	{	 	
		uint16_t bValue;   
	
	ADS1x15_RUN(); 
	//	delay_ms(1000);
	#if(BAT_SAMPLE>0)		
		bValue		= ADS1x15_ConvertAverage(Channel_BAT,ADS1x15_PGA_4096); //电池电压直接采样
	#else
//		bValue		= ADS1x15_ConvertAverage(Channel_BAT,ADS1x15_PGA_2048);//电池电压分压一半后采样
		bValue		= ADS1x15_ConvertAverage(Channel_BAT,ADS1x15_PGA_6144);//电池电压分压一半后采样
	ADS1x15_SLEEP();
	#endif
	#if(SOLAR_UV_DELAY==0)
  	//	ADS1x15_SLEEP();
	#endif
	#if (ADS1x15_DEBUG>0)
		printf("\r\nbatValue:%d\r\n",bValue);
	#endif	 
//		batValue=(float)bValue*6.144/ADS_MAXVALUE*2;
//				printf("\r\batValue:%f\r\n",batValue);
	}
*/


