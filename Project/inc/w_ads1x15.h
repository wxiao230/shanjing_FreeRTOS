/**
  ******************************************************************************
  * @file    Project/user/w_ads1x15.h 
  * @author  insentek wxh
  * @version V1.0.2
  * @date    01-August-2013
  * @brief   Header for w_ads1x15.c module
  ******************************************************************************   
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_W_ADS1X15_H
#define _W_ADS1X15_H 		
/* Includes ------------------------------------------------------------------*/
	#include "stm32l1xx.h" 
	#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef union
{
  uint16_t i;   
  struct{
    uint8_t valueL;
    uint8_t valueH;	    
  }bytes;   
}ads_t;
/* Exported variables --------------------------------------------------------*/

//电池电压
extern 	 uint16_t adsbatValue;
//锂电池充电电压
//extern	__IO uint16_t adschargeValue;
//外部电池输入电压
extern	float adsexterValue;

#define ADS_I2C_STM32L     0

/* Exported constants --------------------------------------------------------*/  
	/**/
	#define  ADS1015_TYPE			0
	#define  ADS1115_TYPE			1
	#define  ADS_TYPE				ADS1015_TYPE
	

#if(ADS_TYPE==ADS1015_TYPE)
	#define ADS_MAXVALUE			2048  //0x0800
#else
	#define ADS_MAXVALUE			32768 //0x8000
#endif
/*SOLAR UV ADC PIN I2C1 */
	//SOLAR UV ADC power control
	#define ADS1x15_EN_PIN			GPIO_Pin_2
	#define ADS1x15_EN_Source		GPIOD	
	#define ADS1x15_EN_RCC			RCC_AHBPeriph_GPIOD
	#define ADS1x15_Enable()		ADS1x15_EN_Source->BSRRL=ADS1x15_EN_PIN//GPIO_SetBits(ADS1x15_EN_Source,ADS1x15_EN_PIN)
	#define ADS1x15_Disable()		ADS1x15_EN_Source->BSRRH=ADS1x15_EN_PIN//GPIO_ResetBits(ADS1x15_EN_Source,ADS1x15_EN_PIN)
	//ADS1x15 RDY control
	#define ADS_RDY_PIN				GPIO_Pin_13
	#define ADS_RDY_Source			GPIOC	
	#define ADS_RDY_RCC				RCC_AHBPeriph_GPIOC
	//ADS1x15 SCL control
	#define ADS_SCL_PIN				GPIO_Pin_8	
	//ADS1x15 SDA control
	#define ADS_SDA_PIN				GPIO_Pin_9
	#define ADS_Source				GPIOB	
	#define ADS_PIN_RCC				RCC_AHBPeriph_GPIOB	
#if (ADS_I2C_STM32L>0)
	#define ADS_I2C_RCC 			RCC_APB1Periph_I2C1  
	#define ADS_SCL_PinSource		GPIO_PinSource8
	#define ADS_SDA_PinSource		GPIO_PinSource9
	#define ADS_GPIO_AF_I2C			GPIO_AF_I2C1
	#define ADS_I2C					I2C1
	#define I2C_SPEED				100000
	#define OwnAddress				0x30 

#else 	
	#define ADS_SDA_HIGH()		ADS_Source->MODER&=0xFFF3FFFF  //输入模式
	#define ADS_SDA_LOW()		ADS_Source->MODER&=0xFFF3FFFF;ADS_Source->MODER|=0x00040000;ADS_Source->BSRRH=ADS_SDA_PIN //推挽输出低电平
	#define ADS_SDA_PORT()		GPIO_ReadInputDataBit(ADS_Source,ADS_SDA_PIN)//((ADS_Source->IDR&(ADS_SDA_PIN))?1:0) //读取SDA的电平
	#define ADS_SCL_HIGH()		ADS_Source->BSRRL=ADS_SCL_PIN//GPIO_SetBits(SHT_SCL_Source,SHT_SCL_PIN)
	#define ADS_SCL_LOW()		ADS_Source->BSRRH=ADS_SCL_PIN//GPIO_ResetBits(SHT_SCL_Source,SHT_SCL_PIN)	
#endif
/*ADS1x15*/
	//ADS1x15_Reset
	#define ADS1x15_ADDR_RST		0x00
	#define ADS1x15_RST_BYTE		0x06
	//ADS1x15_ADDR
	#define ADS1x15_ADDR			0x90   //GND 0x90,VDD 0x91,SDA 0x92,SCL 0x93
	#define ADS1x15_ADDR_WRITE		ADS1x15_ADDR
	#define ADS1x15_ADDR_READ		ADS1x15_ADDR+1
	//ADS1x15_PointerRegister
	#define ADS1x15_REG_CONVERSION	0x00
	#define ADS1x15_REG_CONFIG		0x01
	#define ADS1x15_REG_LoThresh	0x02
	#define ADS1x51_REG_HiThresh	0x03
    //ADS1x15_ConfigRegister
	#define ADS1x15_CONFIG_Default	0x85E3//0x0583 //AIN0_1 2048 single//1600SPS Disable Comparator

	#define	ADS1x15_OS_MASK			0x8000
	#define ADS1x15_OS_NoEffect 	0x0000
	#define ADS1x15_OS_BeginConvert	0x8000
	#define ADS1x15_OS_Converting	0x0000
	#define ADS1x15_OS_NoConvert	0x8000
	///ADS1x15_InputMultiplexer
	#define ADS1x15_MUX_MASK		0x7000
	#define ADS1x15_MUX_AIN0_1		0x0000
	#define ADS1x15_MUX_AIN0_3		0x1000
	#define ADS1x15_MUX_AIN1_3		0x2000
	#define ADS1x15_MUX_AIN2_3		0x3000
	#define ADS1x15_MUX_AIN0_G		0x4000
	#define ADS1x15_MUX_AIN1_G		0x5000
	#define ADS1x15_MUX_AIN2_G		0x6000
	#define ADS1x15_MUX_AIN3_G		0x7000
	//ADS1x15_GainAmplifier
	#define ADS1x15_PGA_MASK		0x0E00
	#define ADS1x15_PGA_6144		0x0000
	#define ADS1x15_PGA_4096		0x0200
	#define ADS1x15_PGA_2048		0x0400
	#define ADS1x15_PGA_1024		0x0600
	#define ADS1x15_PGA_512			0x0800
	#define ADS1x15_PGA_256			0x0A00 //0x0C00//0x0E00
	//ADS1x15_Mode
	#define ADS1x15_MODE_MASK		0x0100
	#define ADS1x15_MODE_CONTINUOUS 0x0000
	#define ADS1x15_MODE_SINGLE		0x0100
	//ADS1x15_DataRate
	#define ADS1x15_DR_MASK			0x00E0
	#define ADS1x15_DR_128			0x0000
	#define ADS1x15_DR_250			0x0020
	#define ADS1x15_DR_490			0x0040
	#define ADS1x15_DR_920			0x0060
	#define ADS1x15_DR_1600			0x0080
	#define ADS1x15_DR_2400			0x00A0
	#define ADS1x15_DR_3300			0x00C0//0xE0
	//ADS1x15_COMPMODE
	#define ADS1x15_COMPMODE_MASK	0x0010
	#define ADS1x15_COMPMODE_NORMAL	0x0000
	#define ADS1x15_COMPMODE_WINDOW	0x0010
	//ADS1x15_COMPPOL
	#define ADS1x15_COMPPOL_MASK	0x0008
	#define ADS1x15_COMPPOL_LOW		0x0000
	#define ADS1x15_COMPPOL_HIGH	0x0008
	//ADS1x15_COMPLAT
	#define ADS1x15_COMPLAT_MASK	0x0004
	#define ADS1x15_COMPLAT_DISABLE	0x0000
	#define ADS1x15_COMPLAT_ENABLE	0x0004
	//ADS1x15_COMPQUE
	#define ADS1x15_COMPQUE_MASK	0x0003
	#define ADS1x15_COMPQUE_ONE		0x0000
	#define ADS1x15_COMPQUE_TOW		0x0001
	#define ADS1x15_COMPQUE_FOUR	0x0002
	#define ADS1x15_COMPQUE_DISABLE	0x0003   

	#define Channel_SOLAR			 ADS1x15_MUX_AIN0_G
	#define Channel_UV				 ADS1x15_MUX_AIN2_G
	#define Channel_BAT				 ADS1x15_MUX_AIN1_G
#define SUN_channel       ADS1x15_MUX_AIN0_G
#define EXTER_channel	  ADS1x15_MUX_AIN1_G
#define BAT_channel		  ADS1x15_MUX_AIN2_G
	
	#define I2C_MAX_CHECK_CNT 		10000  /* Max number of I2C flag checks */
	#define I2C_OK					0xFF
	#define I2C_BUSY_ERR			0x01
	#define I2C_MASTERSELECT_ERR 	0x02
	#define I2C_MODESELECT_ERR 		0x03
	#define I2C_TRAN_ERR			0x04
	#define I2C_SEND_ERR			0x05   
	#define I2C_RECEICE_ERR			0x06
	#define I2C_NoACK_ERR			0x07

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */



	extern void ADS1x15_RUN(void);
	extern void ADS1x15_SLEEP(void);
	extern ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, uint32_t I2C_Event);
	extern uint8_t I2C_Start_Transmitter(uint8_t addr,uint8_t frist_byte);
	extern uint8_t I2C_Stop_Transmitter(void);
	extern uint8_t I2C_SendByte(uint8_t byte);
	extern uint8_t ADS1x15_WritePointer(uint8_t addr,uint8_t PointerReg);
	extern uint16_t ADS1x15_ReadRegister(void);
	extern uint8_t ADS1x15_Reset(void);
	extern uint8_t ADS1x15_Config(uint16_t ads_channel,uint16_t ads_pga);	
	extern uint16_t ADS1x15_ConvertAverage(uint16_t ads_channel,uint16_t ads_pga);
	extern void Read_BatValue(void);
	extern void Solar_UV_Task(void);
	extern void ClearBaleData_Solar(void);
	extern void ClearBaleData_UV(void);
	extern void ADS1x15_Test(void);
	extern uint16_t Read_Value(uint16_t whichChannel);
#endif 

/*****************************END OF FILE*****************************/

