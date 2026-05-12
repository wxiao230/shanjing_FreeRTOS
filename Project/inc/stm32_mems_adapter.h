/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : stm32_mems_adapter.h
* Author             : Industrial & Multi-Market Competence Center Europe - Connectivity & Sensors Team
* Version            : V1.0.1
* Date               : 07/20/2009
* Description        : Provides function prototypes and defines to work with
*                      SMT32-MEMS demonstration board.
********************************************************************************
* History:
* 07/20/2009: V1.0.1 
* 05/26/2009: V1.0.0 Initial revision
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_MEMS_ADAPTER_H
#define __STM32_MEMS_ADAPTER_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x_gpio.h"
#include "stm32l1xx.h"
#include "stm32_mems.h"
#include "XL345.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Uncomment ONE line corresponding to your STM32 demonstration board */
#define STM3210B_EVAL
//#define STM3210E_EVAL
//#define STM3210B_SK_IAR

/*=============================================================================
IMPORTANT NOTE
When using the MEMS Library on the STM3210E_EVAL board, some JTAG signals of the
STM32 MCU can be remapped to the GPIO functionality by the library functions
MEMS_SPI_Setup and MEMS_I2C_Setup. This means that after the program startup,
debugging or flashing the MCU via the JTAG will not be possible.
In order to be able to re-flash the MCU via the JTAG, you have to power-up the
board with BOOT0 and BOOT1 switches set to position 1 and then flash the MCU.
Finally, to run the program from the Flash, power-up the board with switches
BOOT0 and BOOT1 set to position 0.
===============================================================================*/


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------           STM3210B_EVAL           ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#ifdef STM3210B_EVAL

/* Analog MEMS constants =====================================================*/
#define MEMS_ANL_RCC_APB2Periph_ADC   RCC_APB2Periph_ADC1
#define MEMS_ANL_ADC                  ADC1
#define MEMS_ANL_ADC_Channel_X        ADC_Channel_10
#define MEMS_ANL_ADC_Channel_Y        ADC_Channel_11
#define MEMS_ANL_ADC_Channel_Z        ADC_Channel_13
#define MEMS_ANL_ADC_DR_Address       ((u32)0x4001244C)


#define MEMS_ANL_RCC_AHBPeriph_DMA    RCC_AHBPeriph_DMA1
#define MEMS_ANL_DMA_Channel          DMA1_Channel1

#define MEMS_ANL_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOE
#define MEMS_ANL_GPIO_X               GPIOC
#define MEMS_ANL_GPIO_Y               GPIOC
#define MEMS_ANL_GPIO_Z               GPIOC
#define MEMS_ANL_GPIO_FS              GPIOE
#define MEMS_ANL_GPIO_PD              GPIOE
#define MEMS_ANL_GPIO_Pin_X           GPIO_Pin_0
#define MEMS_ANL_GPIO_Pin_Y           GPIO_Pin_1
#define MEMS_ANL_GPIO_Pin_Z           GPIO_Pin_3
#define MEMS_ANL_GPIO_Pin_FS          GPIO_Pin_3
#define MEMS_ANL_GPIO_Pin_PD          GPIO_Pin_2

/* Digital MEMS constants ====================================================*/
/* Common Digital MEMS constants */
#define MEMS_DIG_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB 
#define MEMS_DIG_GPIO_CS              GPIOB
#define MEMS_DIG_GPIO_Int1            GPIOB
#define MEMS_DIG_GPIO_Int2            GPIOB
#define MEMS_DIG_GPIO_Pin_CS          GPIO_Pin_8
#define MEMS_DIG_GPIO_Pin_Int1        GPIO_Pin_5
#define MEMS_DIG_GPIO_Pin_Int2        GPIO_Pin_7

#define MEMS_DIG_GPIO_PortSourceGPIO_Int1 EXTI_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int1  GPIO_PinSource5
#define MEMS_DIG_EXTI_Line_Int1       EXTI_Line5

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI0_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI0_IRQChannel
#endif

#define MEMS_DIG_GPIO_PortSourceGPIO_Int2 EXTI_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int2  GPIO_PinSource7
#define MEMS_DIG_EXTI_Line_Int2       EXTI_Line13

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI15_10_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI15_10_IRQChannel
#endif

/* Digital MEMS over SPI constants -------------------------------------------*/
#define MEMS_SPI_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOA
#define MEMS_SPI_GPIO                 GPIOA
#define MEMS_SPI_GPIO_Pin_SCK         GPIO_Pin_5
#define MEMS_SPI_GPIO_Pin_SDI         GPIO_Pin_7
#define MEMS_SPI_GPIO_Pin_SD0         GPIO_Pin_6

#define MEMS_SPI_RCC_APB2Periph_SPI   RCC_APB2Periph_SPI1
#define MEMS_SPI                      SPI1

/* Digital MEMS over I2C constants -------------------------------------------*/
#define MEMS_I2C_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB
#define MEMS_I2C_GPIO                 GPIOB
#define MEMS_I2C_GPIO_Pin_SCL         GPIO_Pin_6
#define MEMS_I2C_GPIO_Pin_SDA         GPIO_Pin_7

#define MEMS_I2C_RCC_APB1Periph_I2C   RCC_APB1Periph_I2C1
#define MEMS_I2C                      I2C1

#endif /* STM3210B_EVAL */

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------           STM3210E_EVAL           ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#ifdef STM3210E_EVAL

/* Analog MEMS constants =====================================================*/
#define MEMS_ANL_RCC_APB2Periph_ADC   RCC_APB2Periph_ADC1
#define MEMS_ANL_ADC                  ADC1
#define MEMS_ANL_ADC_Channel_X        ADC_Channel_8
#define MEMS_ANL_ADC_Channel_Y        ADC_Channel_15
#define MEMS_ANL_ADC_Channel_Z        ADC_Channel_9
#define MEMS_ANL_ADC_DR_Address       ((u32)0x4001244C)


#define MEMS_ANL_RCC_AHBPeriph_DMA    RCC_AHBPeriph_DMA1
#define MEMS_ANL_DMA_Channel          DMA1_Channel1

#define MEMS_ANL_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOG | RCC_AHBPeriph_GPIOF
#define MEMS_ANL_GPIO_X               GPIOB
#define MEMS_ANL_GPIO_Y               GPIOC
#define MEMS_ANL_GPIO_Z               GPIOB
#define MEMS_ANL_GPIO_FS              GPIOG
#define MEMS_ANL_GPIO_PD              GPIOF
#define MEMS_ANL_GPIO_Pin_X           GPIO_Pin_0
#define MEMS_ANL_GPIO_Pin_Y           GPIO_Pin_5
#define MEMS_ANL_GPIO_Pin_Z           GPIO_Pin_1
#define MEMS_ANL_GPIO_Pin_FS          GPIO_Pin_0
#define MEMS_ANL_GPIO_Pin_PD          GPIO_Pin_13

/* Digital MEMS constants ====================================================*/
/* Common Digital MEMS constants */
#define MEMS_DIG_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB
#define MEMS_DIG_GPIO_CS              GPIOB
#define MEMS_DIG_GPIO_Int1            GPIOB
#define MEMS_DIG_GPIO_Int2            GPIOB
#define MEMS_DIG_GPIO_Pin_CS          GPIO_Pin_6
#define MEMS_DIG_GPIO_Pin_Int1        GPIO_Pin_5
#define MEMS_DIG_GPIO_Pin_Int2        GPIO_Pin_7

#define MEMS_DIG_GPIO_PortSourceGPIO_Int1 GPIO_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int1  GPIO_PinSource5
#define MEMS_DIG_EXTI_Line_Int1       EXTI_Line5

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI15_10_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI15_10_IRQChannel
#endif

#define MEMS_DIG_GPIO_PortSourceGPIO_Int2 GPIO_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int2  GPIO_PinSource7
#define MEMS_DIG_EXTI_Line_Int2       EXTI_Line7

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI9_5_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI9_5_IRQChannel
#endif

/* Digital MEMS over SPI constants -------------------------------------------*/
#define MEMS_SPI_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_AFIO
/* AFIO is used for reconfiguration of JTAG pins */
#define MEMS_SPI_GPIO                 GPIOB
#define MEMS_SPI_GPIO_Pin_SCK         GPIO_Pin_3
#define MEMS_SPI_GPIO_Pin_SDI         GPIO_Pin_5
#define MEMS_SPI_GPIO_Pin_SD0         GPIO_Pin_4

#define MEMS_SPI_RCC_APB1Periph_SPI   RCC_APB1Periph_SPI3
#define MEMS_SPI                      SPI3

/* Digital MEMS over I2C constants --------------------------------------------*/
#define MEMS_I2C_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB | RCC_APB2Periph_AFIO
/* AFIO is used for reconfiguration of JTAG and I2C1 pins */
#define MEMS_I2C_GPIO                 GPIOB
#define MEMS_I2C_GPIO_Pin_SCL         GPIO_Pin_8
#define MEMS_I2C_GPIO_Pin_SDA         GPIO_Pin_9

#define MEMS_I2C_RCC_APB1Periph_I2C   RCC_APB1Periph_I2C1
#define MEMS_I2C                      I2C1


#endif /* STM3210E_EVAL */


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*------------------           STM3210B_SK_IAR           ---------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

#ifdef STM3210B_SK_IAR

/* Analog MEMS constants =====================================================*/
#define MEMS_ANL_RCC_APB2Periph_ADC   RCC_APB2Periph_ADC1
#define MEMS_ANL_ADC                  ADC1
#define MEMS_ANL_ADC_Channel_X        ADC_Channel_5
#define MEMS_ANL_ADC_Channel_Y        ADC_Channel_6
#define MEMS_ANL_ADC_Channel_Z        ADC_Channel_7
#define MEMS_ANL_ADC_DR_Address       ((u32)0x4001244C)


#define MEMS_ANL_RCC_AHBPeriph_DMA    RCC_AHBPeriph_DMA1
#define MEMS_ANL_DMA_Channel          DMA1_Channel1

#define MEMS_ANL_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOA
#define MEMS_ANL_GPIO_X               GPIOA
#define MEMS_ANL_GPIO_Y               GPIOA
#define MEMS_ANL_GPIO_Z               GPIOA
#define MEMS_ANL_GPIO_FS              GPIOA
#define MEMS_ANL_GPIO_PD              GPIOA
#define MEMS_ANL_GPIO_Pin_X           GPIO_Pin_5
#define MEMS_ANL_GPIO_Pin_Y           GPIO_Pin_6
#define MEMS_ANL_GPIO_Pin_Z           GPIO_Pin_7
#define MEMS_ANL_GPIO_Pin_FS          GPIO_Pin_9
#define MEMS_ANL_GPIO_Pin_PD          GPIO_Pin_10

/* Digital MEMS constants ====================================================*/
/* Common Digital MEMS constants */
#define MEMS_DIG_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_AFIO
#define MEMS_DIG_GPIO_CS              GPIOB
#define MEMS_DIG_GPIO_Int1            GPIOB
#define MEMS_DIG_GPIO_Int2            GPIOB
#define MEMS_DIG_GPIO_Pin_CS          GPIO_Pin_12
#define MEMS_DIG_GPIO_Pin_Int1        GPIO_Pin_11
#define MEMS_DIG_GPIO_Pin_Int2        GPIO_Pin_10

#define MEMS_DIG_GPIO_PortSourceGPIO_Int1 GPIO_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int1  GPIO_PinSource11
#define MEMS_DIG_EXTI_Line_Int1       EXTI_Line11

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI15_10_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int1       EXTI15_10_IRQChannel
#endif


#define MEMS_DIG_GPIO_PortSourceGPIO_Int2 GPIO_PortSourceGPIOB
#define MEMS_DIG_GPIO_PinSource_Int2  GPIO_PinSource10
#define MEMS_DIG_EXTI_Line_Int2       EXTI_Line10

#ifdef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library used */
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI15_10_IRQn
#else
#define MEMS_DIG_EXTI_IRQn_Int2       EXTI15_10_IRQChannel
#endif

/* Digital MEMS over SPI constants -------------------------------------------*/
#define MEMS_SPI_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB
#define MEMS_SPI_GPIO                 GPIOB
#define MEMS_SPI_GPIO_Pin_SCK         GPIO_Pin_13
#define MEMS_SPI_GPIO_Pin_SDI         GPIO_Pin_15
#define MEMS_SPI_GPIO_Pin_SD0         GPIO_Pin_14

#define MEMS_SPI_RCC_APB1Periph_SPI   RCC_APB1Periph_SPI2
#define MEMS_SPI                      SPI2

/* Digital MEMS over I2C constants -------------------------------------------*/
#define MEMS_I2C_RCC_APB2Periph_GPIOs RCC_AHBPeriph_GPIOB
#define MEMS_I2C_GPIO                 GPIOB
#define MEMS_I2C_GPIO_Pin_SCL         GPIO_Pin_6
#define MEMS_I2C_GPIO_Pin_SDA         GPIO_Pin_7

#define MEMS_I2C_RCC_APB1Periph_I2C   RCC_APB1Periph_I2C1
#define MEMS_I2C                      I2C1


#endif /* STM3210B_SK_IAR */


/* Communication control settings */
/* SPI */
#define MEMS_SPI_WRITE          0x00
#define MEMS_SPI_READ           0x80
#define MEMS_SPI_MULTIPLE_BYTES 0x40

/* I2C */
#define MEMS_I2C_REPETIR        0x80


#define ADXL345_SLEEP     0X00
#define ADXL345_IDLE      0X01
#define ADXL345_INI2       0X02
#define ADXL345_INI1       0X02

extern __IO uint8_t adxIntStatus ;
extern __IO uint8_t adxlStatus;
extern __IO uint16_t num_of_int2;


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void MEMS_ANL_Setup (void);
void MEMS_ANL_Drive_FS (BitAction BitVal);
void MEMS_ANL_Drive_PD (BitAction BitVal);
void MEMS_ANL_ADC_Restart(void);
void MEMS_ANL_Get_Axis(s16 *x, s16 *y, s16 *z);

void MEMS_DIG_Setup_Int1(FunctionalState NewState);
void MEMS_DIG_Setup_Int2(FunctionalState NewState);

void MEMS_SPI_Setup (void);
ErrorStatus MEMS_SPI_WriteReg (u8 RegAddress, u8 Data);
ErrorStatus MEMS_SPI_ReadReg (u8 RegAddress, u8 *Data);
ErrorStatus MEMS_SPI_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes);
ErrorStatus MEMS_SPI_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes);

void MEMS_I2C_Setup (u8 MEMS_I2C_Address);
void MEMS_I2C_Set_Address (u8 MEMS_I2C_Address);
ErrorStatus MEMS_I2C_WriteReg (u8 RegAddress, u8 Data);
ErrorStatus MEMS_I2C_ReadReg (u8 RegAddress, u8 *Data);
ErrorStatus MEMS_I2C_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes);
ErrorStatus MEMS_I2C_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes);

extern ErrorStatus Find_MEMS(void);
extern ErrorStatus LIS302DL_I2C_Init (void);
extern ErrorStatus LIS302DL_I2C_ReadAxes (t_mems_data *MEMS_Data);


//



extern ErrorStatus ADXL345_Init(void);
extern void ADXL345_Read(tg_ADXL345_TYPE * ptResult);
extern void ADXL345_Printf(tg_ADXL345_TYPE * ptResult);

extern void aint1_Config(void);
extern void adxl345Init(void);
extern void aint2_Config(void);
extern void inActInterrupt(void);
extern void in2ActInterrupt(void);
extern void adxl345SelfTest(void);
extern void ADXL345_SetRegisterValue(uint8_t registerAddress,uint8_t registerValue);
extern unsigned char ADXL345_GetRegisterValue(unsigned char registerAddress);
extern void ADXL345_SetPowerMode(unsigned char pwrMode);
extern void ADXL345_SetTapDetection(unsigned char tapType,
                             unsigned char tapAxes,
                             unsigned char tapDur,
                             unsigned char tapLatent,
                             unsigned char tapWindow,
                             unsigned char tapThresh,
                             unsigned char tapInt);
extern void ADXL345_SetInactivityDetection(unsigned char inactOnOff,
                                    unsigned char inactAxes,
                                    unsigned char inactAcDc,
                                    unsigned char inactThresh,
                                    unsigned char inactTime,
                                    unsigned char inactInt);
extern void ADXL345_SetOffset(unsigned char xOffset,unsigned char yOffset,unsigned char zOffset);

extern void adxl345InitInt(void);


/**/
#define XYZ_SHARK_ON     0x31
#define XYZ_SHARK_OFF    0x30

#define ADXL345_STATE_CHAGE   0x01
#define ADXL345_INT_IDLE      0x02
extern __IO uint8_t adxl345Status;

/*----------------------------------------------------------------------------*/

#endif /* __STM32_MEMS_ADAPTER_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
