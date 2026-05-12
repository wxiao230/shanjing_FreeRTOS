/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : stm32_mems_adapter.c
* Author             : Industrial & Multi-Market Competence Center Europe - Connectivity & Sensors Team
* Version            : V1.0.1
* Date               : 07/20/2009
* Description        : Provides functions to work with STM32-MEMS
*                      demonstration board.
********************************************************************************
* History:
* 07/20/2009: V1.0.1 I2C speed increased to 400kHz 
* 05/26/2009: V1.0.0 Initial revision
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/


#include "stm32_mems_adapter.h"

#include "delay.h"
#include "XL345.h"
#include "rtc.h"
#include "stdio.h"
#include "math.h"
#include <stdlib.h>
/*
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_exti.h"
*/

#ifndef USE_STDPERIPH_DRIVER /* CMSIS compliant FW library not used */
#include "stm32f10x_nvic.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Max number of I2C flag checks */
#define I2C_MAX_CHECK_CNT 10000
/* Max number of SPI flag checks */
#define SPI_MAX_CHECK_CNT 10000
/* Private variables ---------------------------------------------------------*/

DMA_InitTypeDef DMA_InitStructure;
s16 ADC_DataValue[3];
u8 MEMS_I2C_DEVICE_ADDRESS=0;

/* Private function prototypes -----------------------------------------------*/

void MEMS_ANL_ADC_Stop(void);
void MEMS_ANL_ADC_Start(void);

ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, u32 I2C_Event);

ErrorStatus SPI_I2S_WaitFlag (SPI_TypeDef* SPIx, u16 SPI_I2S_FLA,
                              FlagStatus SPI_I2S_FLAValue);

/* Private functions ---------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------              Analog               ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

tg_ADXL345_TYPE xlResult[33];


//__IO uint8_t adxIntStatus;
__IO uint8_t adxlStatus =ADXL345_IDLE;
/*******************************************************************************
* Function Name  : MEMS_ANL_Setup
* Description    : Setups all peripherals related to analog MEMS.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Setup (void) {
//  GPIO_InitTypeDef GPIO_InitStructure;
//  ADC_InitTypeDef   ADC_InitStructure;

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable ADC and GPIO clocks */
/*
  RCC_APB2PeriphClockCmd(MEMS_ANL_RCC_APB2Periph_ADC |
                         MEMS_ANL_RCC_APB2Periph_GPIOs, ENABLE);
*/
  /* Enable DMA clock */
//  RCC_AHBPeriphClockCmd(MEMS_ANL_RCC_AHBPeriph_DMA, ENABLE);

  /* Configure GPIOs ---------------------------------------------------------*/
  /* Configure ADC_IN pins */
  /*
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_X;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(MEMS_ANL_GPIO_X, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_Y;
  GPIO_Init(MEMS_ANL_GPIO_Y, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_Z;
  GPIO_Init(MEMS_ANL_GPIO_Z, &GPIO_InitStructure);
*/
  /* Configure FS for analog MEMS control */
 /*
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_FS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(MEMS_ANL_GPIO_FS, &GPIO_InitStructure);
*/
  /* Configure PD for analog MEMS control */
  /*
  GPIO_InitStructure.GPIO_Pin = MEMS_ANL_GPIO_Pin_PD;
  GPIO_Init(MEMS_ANL_GPIO_PD, &GPIO_InitStructure);
  */
  /* Set full scale to 0 (2g) */
 // MEMS_ANL_Drive_FS (Bit_RESET);
  /* Set power down to 0 (MEMS in active state) */
 // MEMS_ANL_Drive_PD (Bit_RESET);

  /* ADC configuration -------------------------------------------------------*/
 /*
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(MEMS_ANL_ADC, &ADC_InitStructure);
 */
  /* ADC regular channel configuration */
 /*
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_X,  1,
                           ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_Y,  2,
                           ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(MEMS_ANL_ADC, MEMS_ANL_ADC_Channel_Z,  3,
                           ADC_SampleTime_239Cycles5);
  */
  /* Enable ADC DMA */
 // ADC_DMACmd(MEMS_ANL_ADC, ENABLE);
  /* Enable ADC */
//  ADC_Cmd(MEMS_ANL_ADC, ENABLE);

  /* Enable ADC reset calibaration register */
//  ADC_ResetCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC reset calibration register */
//  while(ADC_GetResetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC calibaration */
 // ADC_StartCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC calibration */
//  while(ADC_GetCalibrationStatus(MEMS_ANL_ADC));

 /* DMA channel configuration ------------------------------------------------*/
 /*
  DMA_DeInit(MEMS_ANL_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = MEMS_ANL_ADC_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_DataValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(MEMS_ANL_DMA_Channel, &DMA_InitStructure);
 */
  /* Enable DMA channel */
//  DMA_Cmd(MEMS_ANL_DMA_Channel, ENABLE);

  /* Enable interrupt on buffer half- and full */
//  DMA_ITConfig(MEMS_ANL_DMA_Channel, DMA_IT_TC, ENABLE);

}

/*******************************************************************************
* Function Name  : MEMS_ANL_Drive_FS
* Description    : Drives FS pin of analog MEMS.
* Input          : BitVal - new value of the FS pin
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Drive_FS (BitAction BitVal) {
  GPIO_WriteBit(MEMS_ANL_GPIO_FS, MEMS_ANL_GPIO_Pin_FS, BitVal);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_Drive_PD
* Description    : Drives PD pin of analog MEMS.
* Input          : BitVal - new value of the PD pin
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_Drive_PD (BitAction BitVal) {
  GPIO_WriteBit(MEMS_ANL_GPIO_PD, MEMS_ANL_GPIO_Pin_PD, BitVal);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Stop
* Description    : Stops ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Stop(void)
{
/*
  ADC_Cmd(MEMS_ANL_ADC, DISABLE);
  DMA_Cmd(MEMS_ANL_DMA_Channel, DISABLE);
  */
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Start
* Description    : Starts ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Start(void)
{
//  DMA_Init(MEMS_ANL_DMA_Channel, &DMA_InitStructure);
//  DMA_Cmd(MEMS_ANL_DMA_Channel, ENABLE);

  /* enable ADC again */
//  ADC_Cmd(MEMS_ANL_ADC, ENABLE);

  /* Enable ADC reset calibaration register */
//  ADC_ResetCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC reset calibration register */
//  while(ADC_GetResetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC calibaration */
//  ADC_StartCalibration(MEMS_ANL_ADC);
  /* Check the end of ADC calibration */
//  while(ADC_GetCalibrationStatus(MEMS_ANL_ADC));

  /* Start ADC Software Conversion */
//  ADC_SoftwareStartConvCmd(MEMS_ANL_ADC, ENABLE);
}

/*******************************************************************************
* Function Name  : MEMS_ANL_ADC_Restart
* Description    : Restarts ADC and DMA.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_ANL_ADC_Restart(void)
{
/*
  MEMS_ANL_ADC_Stop();
  MEMS_ANL_ADC_Start();
  */
}

/*******************************************************************************
* Function Name  : MEMS_ANL_Get_Axis
* Description    : Gets values of all MEMS axis.
* Input          : none
* Output         : x - Value of X axe
*                  y - Value of Y axe
*                  z - Value of Z axe
* Return         : none
*******************************************************************************/
void MEMS_ANL_Get_Axis(s16 *x, s16 *y, s16 *z)
{
/*
  *x = ADC_DataValue[0];
  *y = ADC_DataValue[1];
  *z = ADC_DataValue[2];
  */
}



/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------              Digital              ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_DIG_Setup_Int1
* Description    : Enables or disables EXTI for the Int1 interrupt signal.
* Input          : NewState: New state of the interrupt. This parameter
*                  can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void MEMS_DIG_Setup_Int1(FunctionalState NewState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Initialize the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int1;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line */
  else
  {
    /* Enable the External Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = MEMS_DIG_EXTI_IRQn_Int1;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure Int1 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_Int1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(MEMS_DIG_GPIO_Int1, &GPIO_InitStructure);

    /* Configure Int1 pin for external interrupt */
    SYSCFG_EXTILineConfig(MEMS_DIG_GPIO_PortSourceGPIO_Int1, MEMS_DIG_GPIO_PinSource_Int1);

    /* Clear the the EXTI line  */
    EXTI_ClearITPendingBit(MEMS_DIG_EXTI_Line_Int1);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int1;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/*******************************************************************************
* Function Name  : MEMS_DIG_Setup_Int2
* Description    : Enables or disables EXTI for the Int2 interrupt signal.
* Input          : NewState: New state of the interrupt. This parameter
*                  can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void MEMS_DIG_Setup_Int2(FunctionalState NewState)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Initialize the EXTI_InitStructure */
  EXTI_StructInit(&EXTI_InitStructure);

  /* Disable the EXTI line */
  if(NewState == DISABLE)
  {
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int2;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
  /* Enable the EXTI line */
  else
  {
    /* Enable the External Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = MEMS_DIG_EXTI_IRQn_Int2;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure Int2 pin as input floating */
    GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_Int2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(MEMS_DIG_GPIO_Int2, &GPIO_InitStructure);

    /* Configure Int2 pin for external interrupt */
    SYSCFG_EXTILineConfig(MEMS_DIG_GPIO_PortSourceGPIO_Int2, MEMS_DIG_GPIO_PinSource_Int2);

    /* Clear the the EXTI line  */
    EXTI_ClearITPendingBit(MEMS_DIG_EXTI_Line_Int2);

    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Line = MEMS_DIG_EXTI_Line_Int2;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  }
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------               SPI                 ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_SPI_Setup
* Description    : Setups all peripherals related to digital MEMS connected over SPI.
* Input          : none
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_SPI_Setup (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable SPI and GPIO clocks */

  RCC_APB2PeriphClockCmd(MEMS_DIG_RCC_APB2Periph_GPIOs |
                         MEMS_SPI_RCC_APB2Periph_GPIOs, ENABLE);

#ifdef STM3210B_EVAL
  RCC_APB2PeriphClockCmd(MEMS_SPI_RCC_APB2Periph_SPI, ENABLE);
#endif

#ifdef STM3210E_EVAL
  RCC_APB1PeriphClockCmd(MEMS_SPI_RCC_APB1Periph_SPI, ENABLE);

  /* Disable JTAG pins PBx */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif

#ifdef STM3210B_SK_IAR
  RCC_APB1PeriphClockCmd(MEMS_SPI_RCC_APB1Periph_SPI, ENABLE);
#endif


  /* Configure GPIOs    ------------------------------------------------------*/
  /* Configure CS pin */
  GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(MEMS_DIG_GPIO_CS, &GPIO_InitStructure);
  /* Drive CS pin high */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  /* Configure SPI pins */
  GPIO_InitStructure.GPIO_Pin = MEMS_SPI_GPIO_Pin_SCK | MEMS_SPI_GPIO_Pin_SDI |
                                MEMS_SPI_GPIO_Pin_SD0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(MEMS_SPI_GPIO, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; /* No hardware CS generation */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(MEMS_SPI, &SPI_InitStructure);

  /* Enable SPI */
  SPI_Cmd(MEMS_SPI, ENABLE);

}

/*******************************************************************************
* Function Name  : SPI_I2S_WaitFlag
* Description    : Waits till the SPI flag is set to the defined value.
* Input          : - SPIx: where x can be 1 or 2 to select the SPI peripheral.
*                  - SPI_I2S_FLA: specifies the flag to be checked.
*                  - SPI_I2S_FLAValue: specifies the flag status to be checked.
* Output         : None
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Flag was set correctly
*                       - ERROR: Flag was not not set correctly
*******************************************************************************/
ErrorStatus SPI_I2S_WaitFlag (SPI_TypeDef* SPIx, u16 SPI_I2S_FLA,
                              FlagStatus SPI_I2S_FLAValue)
{
  u16 i;

  for (i=0; i < SPI_MAX_CHECK_CNT; i++)
    if (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLA) == SPI_I2S_FLAValue)
      return SUCCESS;

  return ERROR;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_SendFrame
* Description    : Sends one frame over SPI.
* Input          : - RegAddress: Address of register.
*                  - pBuffer: Pointer to buffer with data.
*                  - NoOfBytes: Number of bytes to be sent.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame was sent
*                       - ERROR: Frame was not sent
*******************************************************************************/
ErrorStatus MEMS_SPI_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u8 i;
  u8 addr = RegAddress + MEMS_SPI_WRITE;

  if (NoOfBytes > 1)
    addr += MEMS_SPI_MULTIPLE_BYTES;

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_RESET);

  /* Send address byte */
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;

  SPI_I2S_SendData(MEMS_SPI, addr);
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
  SPI_I2S_ReceiveData(MEMS_SPI);

  /* Send data bytes */
  for(i=0; i < NoOfBytes; i++) {
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
    SPI_I2S_SendData(MEMS_SPI, *(pBuffer+i));
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
    /* Empty receive FIFO */
    SPI_I2S_ReceiveData(MEMS_SPI);
  }

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_ReceiveFrame
* Description    : Receives one frame over SPI.
* Input          : - RegAddress: Address of source register.
*                  - NoOfBytes: Number of bytes to be received.
* Output         : - pBuffer: Pointer to output buffer.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame was received
*                       - ERROR: Frame was not received
*******************************************************************************/
ErrorStatus MEMS_SPI_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u8 i;
  u16 addr = RegAddress + MEMS_SPI_READ;
  u16 dummy = 0xAA58;

  if (NoOfBytes > 1)
    addr += MEMS_SPI_MULTIPLE_BYTES;

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_RESET);

  /* Send Address byte */
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
  SPI_I2S_SendData(MEMS_SPI, addr);
  if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
  SPI_I2S_ReceiveData(MEMS_SPI); /* skip byte received with the address */

  /* Receive Data byte(s) */
  for(i=0; i < NoOfBytes; i++) {
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_TXE, SET) != SUCCESS) return ERROR;
    SPI_I2S_SendData(MEMS_SPI, dummy); /* send dummy byte to generate clock */
    if (SPI_I2S_WaitFlag(MEMS_SPI, SPI_I2S_FLAG_RXNE, SET) != SUCCESS) return ERROR;
    *(pBuffer+i) = SPI_I2S_ReceiveData(MEMS_SPI);
  }

  /* Drive SSEL-manual pin */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_SPI_WriteReg
* Description    : Writes data to register of MEMS over SPI.
* Input          : - RegAddress: Address of register.
*                  - Data: Data to be written.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Register was written
*                       - ERROR: Register was not written
*******************************************************************************/
ErrorStatus MEMS_SPI_WriteReg (u8 RegAddress, u8 Data)
{
  return MEMS_SPI_SendFrame (RegAddress, &Data, 1);
}

/*******************************************************************************
* Function Name  : MEMS_SPI_ReadReg
* Description    : Reads data to register of MEMS over SPI.
* Input          : - RegAddress: Address of register.
* Output         : - Data: Data read.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Register was read
*                       - ERROR: Register was not read
*******************************************************************************/
ErrorStatus MEMS_SPI_ReadReg (u8 RegAddress, u8 *Data)
{
  return MEMS_SPI_ReceiveFrame (RegAddress, Data, 1);
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*-------------------               I2C                 ----------------------*/
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*******************************************************************************
* Function Name  : MEMS_I2C_Setup
* Description    : Setups all peripherals related to digital MEMS connected over I2C.
* Input          : MEMS_I2C_Address - I2C address of MEMS.
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_I2C_Setup (u8 MEMS_I2C_Address)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* Store I2C address of the MEMS sensor */
  MEMS_I2C_Set_Address (MEMS_I2C_Address);

  /* Enable clocks -----------------------------------------------------------*/
  /* Enable I2C and GPIO clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);



  /* Configure GPIOs    ------------------------------------------------------*/
  /* Configure I2C  pins     -------------------------------------------------*/
 
  GPIO_PinAFConfig(MEMS_I2C_GPIO,GPIO_PinSource6,GPIO_AF_I2C1);
  GPIO_PinAFConfig(MEMS_I2C_GPIO,GPIO_PinSource7,GPIO_AF_I2C1);


  GPIO_InitStructure.GPIO_Pin = MEMS_I2C_GPIO_Pin_SCL|MEMS_I2C_GPIO_Pin_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(MEMS_I2C_GPIO, &GPIO_InitStructure);

  /* Configure CS and SDO for I2C MEMS control*/
  /* Configure CS pin */
  GPIO_InitStructure.GPIO_Pin = MEMS_DIG_GPIO_Pin_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;


  GPIO_Init(MEMS_DIG_GPIO_CS, &GPIO_InitStructure);
  /* Drive CS pin high */
  GPIO_WriteBit(MEMS_DIG_GPIO_CS, MEMS_DIG_GPIO_Pin_CS, Bit_SET);

  /* Configure SDO pin */
//  GPIO_InitStructure.GPIO_Pin = MEMS_SPI_GPIO_Pin_SD0;
//  GPIO_Init(MEMS_SPI_GPIO, &GPIO_InitStructure);
  /* Drive SDO (LSB of I2C address) pin low */
//  GPIO_WriteBit(MEMS_SPI_GPIO, MEMS_SPI_GPIO_Pin_SD0, Bit_RESET);

  /* I2C configuration -------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x0A;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_Init(MEMS_I2C, &I2C_InitStructure);

  /* Enable I2C */
  I2C_Cmd (MEMS_I2C, ENABLE);

}

/*******************************************************************************
* Function Name  : MEMS_I2C_Set_address
* Description    : Sets address of MEMS for I2C communication.
* Input          : MEMS_I2C_Address - I2C address of MEMS.
* Output         : none
* Return         : none
*******************************************************************************/
void MEMS_I2C_Set_Address (u8 MEMS_I2C_Address)
{
    MEMS_I2C_DEVICE_ADDRESS = MEMS_I2C_Address;
}

/*******************************************************************************
* Function Name  : I2C_WaitEvent
* Description    : Waits till the last I2Cx Event is equal to the one passed
*                  as parameter.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_EVENT: specifies the event to be checked.
* Output         : None
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Last event is equal to the I2C_EVENT
*                       - ERROR: Last event is different from the I2C_EVENT
*******************************************************************************/
ErrorStatus I2C_WaitEvent (I2C_TypeDef* I2Cx, u32 I2C_Event)
{
  u16 i;

  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if (I2C_CheckEvent(I2Cx, I2C_Event) == SUCCESS)
        return SUCCESS;

  return ERROR;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_SendFrame
* Description    : Sends one frame over I2C.
* Input          : - RegAddress: Address of register.
*                  - pBuffer: Pointer to buffer with data.
*                  - NoOfBytes: Number of bytes to be sent.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame sent.
*                       - ERROR: Frame was not sent.
*******************************************************************************/
ErrorStatus MEMS_I2C_SendFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u16 i;

  /* Send START condition */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  /* Test on EV5 and clear it */
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;
  /* Send slave address for write */
  I2C_Send7bitAddress(MEMS_I2C,MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Transmitter);
  /* Test on EV6 and clear it */
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    return ERROR;
  I2C_SendData (MEMS_I2C, RegAddress);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;
  /* While there is data to be written */
  while (NoOfBytes-- > 0)
  {
    /* Send the current byte */
    I2C_SendData (MEMS_I2C, *pBuffer);
    if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;
    /* Point to the next byte to be written */
    pBuffer++;
  }

  /* Send STOP condition */
  I2C_GenerateSTOP(MEMS_I2C, ENABLE);
  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if ((MEMS_I2C->CR1&0x200) != 0x200) break;
  if ((MEMS_I2C->CR1&0x200) == 0x200) return ERROR;

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_ReceiveFrame
* Description    : Receives one frame over I2C.
* Input          : - RegAddress: Address of source register.
*                  - NoOfBytes: Number of bytes to be received.
* Output         : - pBuffer: Pointer to output buffer.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Frame transmitted.
*                       - ERROR: Frame was not transmitted.
*******************************************************************************/
ErrorStatus MEMS_I2C_ReceiveFrame (u8 RegAddress, u8 *pBuffer, u8 NoOfBytes)
{
  u16 i;
  u8 addr = RegAddress;
/*
  if (NoOfBytes > 1) {
    I2C_AcknowledgeConfig (MEMS_I2C, ENABLE);
    addr += MEMS_I2C_REPETIR;
  } else
    I2C_AcknowledgeConfig (MEMS_I2C, DISABLE);
 */
     I2C_AcknowledgeConfig (MEMS_I2C, ENABLE);
    addr += MEMS_I2C_REPETIR;
  /* StartBit */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;

  /* Send Address of device & TRANSMITTER mode */
  I2C_Send7bitAddress(MEMS_I2C, MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Transmitter);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    return ERROR;

  /* Send Register address */
  I2C_SendData (MEMS_I2C, addr);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS) return ERROR;

   I2C_GenerateSTOP (MEMS_I2C, ENABLE);
  /* Repeat StartBit */
  I2C_GenerateSTART(MEMS_I2C, ENABLE);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS) return ERROR;

  /* Send Address of device & RECEIVER mode */
  I2C_Send7bitAddress(MEMS_I2C, MEMS_I2C_DEVICE_ADDRESS, I2C_Direction_Receiver);
  if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    return ERROR;

  /* Receive bytes */
  for (i=0;i<NoOfBytes;i++)
  {
 /*   if (i==(NoOfBytes-2))
      I2C_AcknowledgeConfig (MEMS_I2C, DISABLE);
  */
  //  if (i==(NoOfBytes-1)) /* StopBit */
  //    I2C_GenerateSTOP (MEMS_I2C, ENABLE);


    if (I2C_WaitEvent(MEMS_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS) return ERROR;

    *(pBuffer+i) = I2C_ReceiveData(MEMS_I2C);
	if(i+1 < NoOfBytes)
	{
	  I2C_AcknowledgeConfig (MEMS_I2C, ENABLE);
	}
	else
	{
	   I2C_AcknowledgeConfig (MEMS_I2C, DISABLE);
	   I2C_GenerateSTOP (MEMS_I2C, ENABLE);
	}
  }

  /* Make sure that the STOP bit is cleared by Hardware before CR1 write access */
  for (i=0; i < I2C_MAX_CHECK_CNT; i++)
    if ((MEMS_I2C->CR1&0x200) != 0x200) break;
  if ((MEMS_I2C->CR1&0x200) == 0x200) return ERROR;

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : MEMS_I2C_WriteReg
* Description    : Writes data to register of MEMS over I2C.
* Input          : - RegAddress: Address of register.
*                  - Data: Data to be written.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Data was written.
*                       - ERROR: Data was not written.
*******************************************************************************/
ErrorStatus MEMS_I2C_WriteReg (u8 RegAddress, u8 Data)
{
  return MEMS_I2C_SendFrame (RegAddress, &Data, 1);
}

/*******************************************************************************
* Function Name  : MEMS_I2C_ReadReg
* Description    : Reads data from register of MEMS over I2C.
* Input          : - RegAddress: Address of register.
* Output         : - Data: Data read.
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Data was read.
*                       - ERROR: Data was not read.
*******************************************************************************/
ErrorStatus MEMS_I2C_ReadReg (u8 RegAddress, u8 *Data)
{
  return MEMS_I2C_ReceiveFrame (RegAddress, Data, 1);
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/


/*******************************************************************************
* Function Name  : Find_MEMS
* Description    : Finds MEMS connected to board. It also initializes GPIOs.
*                  Detects MEMS connected via SPI, I2C or analog MEMS.                  
* Input          : none
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Initialization finished successfully.
*                       - ERROR: Initialization finished with error.

*******************************************************************************/

uint8_t who_is = 0; /* Contains WHO_AM_I value of MEMS connected */
/* Maximum number of tries when finding interface*/
#define MAX_TRIES 10


uint8_t xl345Status =0;
ErrorStatus Find_MEMS(void) 
{
	u8 i;
	ErrorStatus stat;
	// GPIO_InitTypeDef GPIO_InitStructure;
	I2C_DeInit(I2C1);
	/* Step 1: Try I2C ---------------------------------------------------------*/
	MEMS_I2C_Setup(ADXL345_Addr);
	//  Delay(4); /* Wait after reset before initializing MEMS */
	//  Delay(40); /* Wait just to leave message on LCD */
	delay_ms(500);
	/* Check for LIS302DL */
	stat = ERROR;
	for(i=0; i < MAX_TRIES && stat == ERROR; i++)
	stat = MEMS_I2C_ReadReg(XL345_DEVID, &who_is);
	xl345Status = stat;
	if (stat == SUCCESS && who_is == LIS302DL_WHO_AM_I_VALUE) 
	{
		return SUCCESS;
	} 
	return ERROR;
}

/*******************************************************************************
* Function Name  : LIS302DL_I2C_Init
* Description    : Initializes LIS302DL MEMS sensor.
* Input          : none
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Initialization finished successfully.
*                       - ERROR: Initialization finished with error.
*******************************************************************************/
ErrorStatus LIS302DL_I2C_Init (void)
{
  u8 i = 0;

  /* IMPORTANT NOTE: These settings differ for different MEMS part numbers! */
  /* Following are setting for LIS302DL. */

  /* CTRL_REG1 Register - Data rate 400Hz, power up, enable all axes */
//  i += MEMS_I2C_WriteReg (LIS302DL_CTRL_REG1, 0xC7); 
/* CTRL_REG1 Register - Data rate 100Hz, power up, enable all axes */
  i += MEMS_I2C_WriteReg (LIS302DL_CTRL_REG1, 0x47); 
  /* CTRL_REG2 Register - Keep default settings */
  i += MEMS_I2C_WriteReg (LIS302DL_CTRL_REG2, 0x00);
  /* CTRL_REG3 Register - Enable click interrupt */
  i += MEMS_I2C_WriteReg (LIS302DL_CTRL_REG3, 0x00);

  if (i != 3)
    return ERROR;

  return SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS302DL_I2C_ReadAxes
* Description    : Reads data from all axes of LIS302DL sensor and adjusts it
*                  for PC GUI application.
* Input          : - MEMS_Data: Data to be written.
* Output         : none
* Return         : An ErrorStatus enumeration value:
*                       - SUCCESS: Data was read.
*                       - ERROR: Data was not read.
*******************************************************************************/
ErrorStatus LIS302DL_I2C_ReadAxes (t_mems_data *MEMS_Data)
{
  u8 i2c_buffer[6];
  ErrorStatus stat;

  stat = MEMS_I2C_ReceiveFrame (LIS302DL_OUTX, i2c_buffer, 6);

  MEMS_Data->outx_h = i2c_buffer[0] >> 4;
  if (i2c_buffer[0] & 0x80) MEMS_Data->outx_h |= 0xF0;
  MEMS_Data->outx_l = i2c_buffer[0] << 4;

  MEMS_Data->outy_h = i2c_buffer[2] >> 4;
  if (i2c_buffer[2] & 0x80) MEMS_Data->outy_h |= 0xF0;
  MEMS_Data->outy_l = i2c_buffer[2] << 4;

  MEMS_Data->outz_h = i2c_buffer[4] >> 4;
  if (i2c_buffer[4] & 0x80) MEMS_Data->outz_h |= 0xF0;
  MEMS_Data->outz_l = i2c_buffer[4] << 4;

  return stat;
}
/******************************************************************************
/ 函数功能:初始化ADX345
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
ErrorStatus ADXL345_Init(void)
{
 /*
    Single_Write(ADXL345_Addr,0x31,0x0B);   //测量范围,正负16mg，13位模式
    // Single_Write(ADXL345_Addr,0x2C,0x0e);   //速率设定为100hz 参考pdf13页
    Single_Write(ADXL345_Addr,0x2D,0x08);   //选择电源模式   参考pdf24页
    Single_Write(ADXL345_Addr,0x2E,0x80);   //使能 DATA_READY 中断,在INT1上
    // Single_Write(ADXL345_Addr,0x1E,0x00);   //X 偏移量 根据测试传感器的状态写入pdf29页
    // Single_Write(ADXL345_Addr,0x1F,0x00);   //Y 偏移量 根据测试传感器的状态写入pdf29页
    // Single_Write(ADXL345_Addr,0x20,0x05);   //Z 偏移量 根据测试传感器的状态写入pdf29页
*/
 // MEMS_I2C_Setup(ADXL345_Addr);
  uint8_t i=0;
  i += MEMS_I2C_WriteReg (0x31,0x03);
  i += MEMS_I2C_WriteReg (0x2D,0x08);
  i += MEMS_I2C_WriteReg (0x2E,0x80);  
  
  if (i != 3)
    return ERROR;

  return SUCCESS;    
}

/******************************************************************************
/ 函数功能:读取ADX345的数据
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
uint8_t xl345Buf[6];

void ADXL345_Read(tg_ADXL345_TYPE * ptResult)
{
    uint8_t tmp[6];
	uint8_t i =0;
 /*   
    tmp[0]=Single_Read(ADXL345_Addr,0x32);//OUT_X_L_A
    tmp[1]=Single_Read(ADXL345_Addr,0x33);//OUT_X_H_A
    
    tmp[2]=Single_Read(ADXL345_Addr,0x34);//OUT_Y_L_A
    tmp[3]=Single_Read(ADXL345_Addr,0x35);//OUT_Y_H_A
    
    tmp[4]=Single_Read(ADXL345_Addr,0x36);//OUT_Z_L_A
    tmp[5]=Single_Read(ADXL345_Addr,0x37);//OUT_Z_H_A
*/
//	  ErrorStatus stat;
//	 MEMS_I2C_WriteReg (0x31,0x0B);
//	 MEMS_I2C_WriteReg (0x2D,0x08);
//	 MEMS_I2C_WriteReg (0x2E,0x80);
//
//	 delay_ms(2000);
  (void)MEMS_I2C_ReceiveFrame (0x32,tmp,6);

    	for(i=0;i<6;i++)
		{
		    xl345Buf[i]= tmp[i];
		}
    ptResult->ax    = (int16_t)((tmp[1]<<8)|tmp[0]);  //合成数据
    ptResult->ay    = (int16_t)((tmp[3]<<8)|tmp[2]);
    ptResult->az    = (int16_t)((tmp[5]<<8)|tmp[4]);
}


/******************************************************************************
/ 函数功能:校准ADXL345
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void ADXL345_Calibrate(void)
{
    int i;
    tg_ADXL345_TYPE tmp_adxl345;
    int32_t ax,ay,az;
    
    //Delayms(2);
	 delay_ms(20);
/*    Single_Write(ADXL345_Addr,0x31,0x0B);   //测量范围,正负16g，13位模式
    // Single_Write(ADXL345_Addr,0x2C,0x0e);   //速率设定为100hz 参考pdf13页
    Single_Write(ADXL345_Addr,0x2D,0x08);   //选择电源模式   参考pdf24页
    Single_Write(ADXL345_Addr,0x2E,0x80);   //使能 DATA_READY 中断
    Single_Write(ADXL345_Addr,0x1E,0);      //先将补偿参数清零,然后再校正
    Single_Write(ADXL345_Addr,0x1F,0);
    Single_Write(ADXL345_Addr,0x20,0);
 */   
   MEMS_I2C_WriteReg (0x31,0x0B);
	 MEMS_I2C_WriteReg (0x2D,0x08);
	 MEMS_I2C_WriteReg (0x2E,0x80);
	 MEMS_I2C_WriteReg (0x1E,0); 
	 MEMS_I2C_WriteReg (0x1F,0);
	 MEMS_I2C_WriteReg (0x20,0);
    ax = ay = az = 0;
    for(i=0; i<100; i++)
    {
 //       Delayms(12);
        delay_ms(20);
        ADXL345_Read(&tmp_adxl345);
				ADXL345_Printf(&tmp_adxl345);
        
        ax += tmp_adxl345.ax;
        ay += tmp_adxl345.ay;
        az += tmp_adxl345.az;
    }

	printf("\n\r power ok,device working11  \n\r");
    ax = -(ax/400);
    ay = -(ay/400);
    az = -(az/100 -256)/4;
    if(abs(ax) > 255)ax=0;
    if(abs(ay) > 255)ay=0;
    if(abs(az) > 255)az=0;
    //修正偏移值
	/*
    Single_Write(ADXL345_Addr,0x1E,(uint8_t)ax);
    Single_Write(ADXL345_Addr,0x1F,(uint8_t)ay);
    Single_Write(ADXL345_Addr,0x20,(uint8_t)az);
	*/
   MEMS_I2C_WriteReg (0x1E,(uint8_t)ax); 
	 MEMS_I2C_WriteReg (0x1F,(uint8_t)ay);
	 MEMS_I2C_WriteReg (0x20,(uint8_t)az);

}
void adxl345SelfTest(void)
{
     int i;
    tg_ADXL345_TYPE tmp_adxl345;
    int32_t ax,ay,az;
   MEMS_I2C_WriteReg (0x31,0x03);
	 MEMS_I2C_WriteReg (0x2C,XL345_RATE_800);
	 MEMS_I2C_WriteReg (0x2D,0x08);
	 MEMS_I2C_WriteReg (0x2E,0x80);
	 MEMS_I2C_WriteReg (0x31,XL345_SELFTEST|0x03);	 
    ax = ay = az = 0;
    for(i=0; i<100; i++)
    {
 //       Delayms(12);
        delay_ms(20);
        ADXL345_Read(&tmp_adxl345);
				ADXL345_Printf(&tmp_adxl345);

        ax += tmp_adxl345.ax;
        ay += tmp_adxl345.ay;
        az += tmp_adxl345.az;
    }
		 MEMS_I2C_WriteReg (0x31,0x0B);		    

}
/******************************************************************************
/ 函数功能:打印ADX345的数据
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void ADXL345_Printf(tg_ADXL345_TYPE * ptResult)
{
    float tempX,tempY,tempZ;
    
    //temp=(float)dis_data*3.9;  //计算数据和显示,查考ADXL345快速入门第4页

    tempX = (float)ptResult->ax * 0.0039;
    tempY = (float)ptResult->ay * 0.0039;
    tempZ = (float)ptResult->az * 0.0039;
	ptResult->xg= tempX;
	ptResult->yg= tempY;
	ptResult->zg= tempZ;
 /*	 int16_t tempX,tempY,tempZ;
    tempX = ptResult->ax ;
    tempY = ptResult->ay ;
    tempZ = ptResult->az ;    */	
  //printf("ADXL345:\tax: %.4fg,\tay: %.4fg,\tsz: %.4fg\r\n",tempX,tempY,tempZ);
 //  printf("ADXL345:\tax: %dg,\tay: %dg,\tsz: %dg\n\r",tempX,tempY,tempZ);

}


/**************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 *
 * @return None.
******************************************************************************/
void ADXL345_SetRegisterValue(uint8_t registerAddress,uint8_t registerValue)
{
     MEMS_I2C_WriteReg (registerAddress,registerValue);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue - Value of the register.
*******************************************************************************/
unsigned char ADXL345_GetRegisterValue(unsigned char registerAddress)
{
//	  ErrorStatus stat;
    unsigned char readData[2] = {0, 0};
 
    unsigned char registerValue = 0;
 
  (void)MEMS_I2C_ReceiveFrame (registerAddress,readData,1);
  registerValue = readData[0];
  return (registerValue);

}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                    Example: 0x0 - standby mode.
 *                             0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;
    
    oldPowerCtl = ADXL345_GetRegisterValue(XL345_POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
    ADXL345_SetRegisterValue(XL345_POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType - Tap type (none, single, double).
 *                    Example: 0x0 - disables tap detection.
 *                             ADXL345_SINGLE_TAP - enables single tap detection.
 *                             ADXL345_DOUBLE_TAP - enables double tap detection.
 * @param tapAxes - Axes which participate in tap detection.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_TAP_X_EN - enables x-axis participation.
 *                             ADXL345_TAP_Y_EN - enables y-axis participation.
 *                             ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tapDur - Tap duration.
 * @param tapLatent - Tap latency.
 * @param tapWindow - Tap window. 
 * @param tapThresh - Tap threshold.
 * @param tapInt - Interrupts pin.
 *                   Example: 0x0 - interrupts on INT1 pin.
 *                            ADXL345_SINGLE_TAP - single tap interrupts on
 *                                                 INT2 pin.
 *                            ADXL345_DOUBLE_TAP - double tap interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetTapDetection(unsigned char tapType,
                             unsigned char tapAxes,
                             unsigned char tapDur,
                             unsigned char tapLatent,
                             unsigned char tapWindow,
                             unsigned char tapThresh,
                             unsigned char tapInt)
{
    unsigned char oldTapAxes    = 0;
    unsigned char newTapAxes    = 0;
    unsigned char oldIntMap     = 0;
    unsigned char newIntMap     = 0;
    unsigned char oldIntEnable  = 0;
    unsigned char newIntEnable  = 0;
    
    oldTapAxes = ADXL345_GetRegisterValue(XL345_TAP_AXES);
    newTapAxes = oldTapAxes & ~(ADXL345_TAP_X_EN |
                                ADXL345_TAP_Y_EN |
                                ADXL345_TAP_Z_EN);
    newTapAxes = newTapAxes | tapAxes;
    ADXL345_SetRegisterValue(XL345_TAP_AXES, newTapAxes);
    ADXL345_SetRegisterValue(XL345_DUR, tapDur);
    ADXL345_SetRegisterValue(XL345_LATENT, tapLatent);
    ADXL345_SetRegisterValue(XL345_WINDOW, tapWindow);
    ADXL345_SetRegisterValue(XL345_THRESH_TAP, tapThresh);
    oldIntMap = ADXL345_GetRegisterValue(XL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntMap = newIntMap | tapInt;
    ADXL345_SetRegisterValue(XL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(XL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntEnable = newIntEnable | tapType;
    ADXL345_SetRegisterValue(XL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inactAxes - Axes which participate in detecting inactivity.
 *                      Example: 0x0 - disables axes participation.
 *                               ADXL345_INACT_X_EN - enables x-axis.
 *                               ADXL345_INACT_Y_EN - enables y-axis.
 *                               ADXL345_INACT_Z_EN - enables z-axis.
 * @param inactAcDc - Selects dc-coupled or ac-coupled operation.
 *                      Example: 0x0 - dc-coupled operation.
 *                               ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity.
 * @param inactTime - Inactivity time.
 * @patam inactInt - Interrupts pin.
 *                     Example: 0x0 - inactivity interrupts on INT1 pin.
 *                              ADXL345_INACTIVITY - inactivity interrupts on
 *                                                   INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetInactivityDetection(unsigned char inactOnOff,
                                    unsigned char inactAxes,
                                    unsigned char inactAcDc,
                                    unsigned char inactThresh,
                                    unsigned char inactTime,
                                    unsigned char inactInt)
{
    unsigned char oldActInactCtl    = 0;
    unsigned char newActInactCtl    = 0;
    unsigned char oldIntMap         = 0;
    unsigned char newIntMap         = 0;
    unsigned char oldIntEnable      = 0;
    unsigned char newIntEnable      = 0;
    
    oldActInactCtl = ADXL345_GetRegisterValue(XL345_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_INACT_ACDC |
                                        ADXL345_INACT_X_EN |
                                        ADXL345_INACT_Y_EN |
                                        ADXL345_INACT_Z_EN);
    newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
    ADXL345_SetRegisterValue(XL345_ACT_INACT_CTL, newActInactCtl);
    ADXL345_SetRegisterValue(XL345_THRESH_INACT, inactThresh);
    ADXL345_SetRegisterValue(XL345_TIME_INACT, inactTime);
    oldIntMap = ADXL345_GetRegisterValue(XL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_INACTIVITY);
    newIntMap = newIntMap | inactInt;
    ADXL345_SetRegisterValue(XL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(XL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_INACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_INACTIVITY * inactOnOff);
    ADXL345_SetRegisterValue(XL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Calibrates the accelerometer.
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetOffset(unsigned char xOffset,
                       unsigned char yOffset,
                       unsigned char zOffset)
{
    ADXL345_SetRegisterValue(XL345_OFSX, xOffset);
    ADXL345_SetRegisterValue(XL345_OFSY, yOffset);
    ADXL345_SetRegisterValue(XL345_OFSZ, yOffset);
}


__IO uint16_t num_of_int =  0; 
void inActInterrupt(void)
{
    unsigned char interruptSource;
		unsigned char count = 0;					   // (JWS) old was an int
		unsigned char entryCount = 0;
//		unsigned char databuf[8];

		tg_ADXL345_TYPE xl345Result1;

			printf("\n\r enter isp   \n\r");
	interruptSource = ADXL345_GetRegisterValue(XL345_INT_SOURCE);
    
	num_of_int++; // count number of interrutps

	if (interruptSource & XL345_OVERRUN) {
//		DEBUG_PRINTF("Overrun @ %d\n", T2VAL);
//		overrun_flag = 1;

	printf("\n\r overrun   \n\r");

	}

	if (interruptSource & XL345_WATERMARK) {
 	printf("\n\r watemark   \n\r");
	 	entryCount = ADXL345_GetRegisterValue(XL345_FIFO_STATUS);
		 		// get the lower 6 bits
		entryCount &= 0x3f;
		 //	entryCount = 31;
	for (count = 0; count < entryCount; count++) { 
	
		     ADXL345_Read(&xl345Result1);
	  ADXL345_Printf(&xl345Result1);
	}
	}
	if (interruptSource &XL345_INACTIVITY) {
 	printf("\n\r XL345_INACTIVITY   \n\r");
	}
	if (interruptSource &XL345_ACTIVITY) {
 	printf("\n\r XL345_ACTIVITY  \n\r");
	}
	
	
				printf("\n\r exit isp   \n\r");

}

//__IO uint8_t adxlStatus =ADXL345_IDLE;


#include "led.h"
__IO uint16_t num_of_int2 =  0; 

void in2ActInterrupt(void)
{
//    unsigned char interruptSource;
//		unsigned char count = 0;					   // (JWS) old was an int
//		unsigned char entryCount = 0;
//		unsigned char databuf[8];
		adxlStatus =ADXL345_INI2;
		rtcWakeUpFlig |= ADXL345_WAKE_UP_FLAG;		
		led_Turn_On(LED_ALL);
	num_of_int2++;
}

#define ADXL345_INTERRUPT    1


void adxl345Init(void)
{
  unsigned char buffer[8];
	/* soft reset for safety */
	ADXL345_SetRegisterValue(XL345_RESERVED1,XL345_SOFT_RESET);

	delay_ms(100);
	ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ);/* TIME_INACT - 5 seconds 2 minutes*/
	
	ADXL345_SetRegisterValue(XL345_INT_ENABLE,0x00);//20191229
	/*--------------------------------------------------
	TAP Configuration	*/
	//   ADXL345_SetRegisterValue(XL345_THRESH_TAP,80);	 /* THRESH_TAP = 5 Gee (1 lsb = 1/16 gee) */	

	//   ADXL345_SetRegisterValue(XL345_DUR,13);	    /* DUR = 8ms 0.6125ms/lsb */
	//    ADXL345_SetRegisterValue(XL345_LATENT,80);  /* LATENT = 100 ms 1.25ms/lsb */	
	//   ADXL345_SetRegisterValue(XL345_WINDOW,240);	/* WINDOW 300ms 1.25ms/lsb */

	//   ADXL345_SetRegisterValue(XL345_TAP_AXES,ADXL345_TAP_X_EN|ADXL345_TAP_Y_EN|ADXL345_TAP_Z_EN);	/* WINDOW 300ms 1.25ms/lsb */

	/*--------------------------------------------------
	activity - inactivity 
	--------------------------------------------------*/
	ADXL345_SetRegisterValue(XL345_THRESH_ACT,8); /* THRESH_ACT = 80/16 = 5 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_THRESH_INACT,2); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_TIME_INACT,2);/* TIME_INACT - 5 seconds 2 minutes*/

	buffer[0]=XL345_ACT_AC | XL345_ACT_X_ENABLE | XL345_ACT_Y_ENABLE | XL345_INACT_AC | XL345_INACT_X_ENABLE  ;
	ADXL345_SetRegisterValue(XL345_ACT_INACT_CTL,buffer[0]);/* TIME_INACT - 5 seconds 2 minutes*/
	//  ADXL345_SetRegisterValue(XL345_INT_MAP,XL345_INACTIVITY|XL345_ACTIVITY|XL345_WATERMARK|XL345_OVERRUN); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_INT_MAP,0x00); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */


	/*--------------------------------------------------
	Power, bandwidth-rate, interupt enabling
	--------------------------------------------------*/
	ADXL345_SetRegisterValue(XL345_BW_RATE,XL345_RATE_12_5|XL345_LOW_POWER); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	//    ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ|XL345_MEASURE);/* TIME_INACT - 5 seconds 2 minutes*/

	// set the FIFO control
	ADXL345_SetRegisterValue(XL345_FIFO_CTL,XL345_FIFO_MODE_STREAM|0|30); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_BW_RATE,XL345_RATE_12_5|XL345_LOW_POWER); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */

	// turn on the watermark interrupt and set the watermark interrupt to int1
#if(ADXL345_INTERRUPT>0)
//	ADXL345_SetRegisterValue(XL345_INT_ENABLE,XL345_ACTIVITY|XL345_WATERMARK|XL345_OVERRUN|31); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
//	ADXL345_SetRegisterValue(XL345_INT_ENABLE,XL345_WATERMARK|XL345_OVERRUN|31); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
#endif
	ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ|XL345_MEASURE);/* TIME_INACT - 5 seconds 2 minutes*/
	delay_ms(20);
}

void adxl345InitInt(void)
{
  unsigned char buffer[8];
    /* soft reset for safety */
	ADXL345_SetRegisterValue(XL345_RESERVED1,XL345_SOFT_RESET);
   
	delay_ms(20);
	ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ);/* TIME_INACT - 5 seconds 2 minutes*/
	
	/*--------------------------------------------------
	TAP Configuration	*/
	//   ADXL345_SetRegisterValue(XL345_THRESH_TAP,80);	 /* THRESH_TAP = 5 Gee (1 lsb = 1/16 gee) */	

	//   ADXL345_SetRegisterValue(XL345_DUR,13);	    /* DUR = 8ms 0.6125ms/lsb */
	//    ADXL345_SetRegisterValue(XL345_LATENT,80);  /* LATENT = 100 ms 1.25ms/lsb */	
	//   ADXL345_SetRegisterValue(XL345_WINDOW,240);	/* WINDOW 300ms 1.25ms/lsb */

	//   ADXL345_SetRegisterValue(XL345_TAP_AXES,ADXL345_TAP_X_EN|ADXL345_TAP_Y_EN|ADXL345_TAP_Z_EN);	/* WINDOW 300ms 1.25ms/lsb */

	/*--------------------------------------------------
	activity - inactivity 
	--------------------------------------------------*/
	ADXL345_SetRegisterValue(XL345_THRESH_ACT,8); /*8 THRESH_ACT = 80/16 = 5 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_THRESH_INACT,2); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_TIME_INACT,2);/* TIME_INACT - 5 seconds 2 minutes*/

	buffer[0]=XL345_ACT_AC|XL345_ACT_X_ENABLE|XL345_ACT_Y_ENABLE|XL345_ACT_Z_ENABLE|XL345_INACT_AC|XL345_INACT_X_ENABLE;//XL345_ACT_Z_ENABLE//20191228
	ADXL345_SetRegisterValue(XL345_ACT_INACT_CTL,buffer[0]);/* TIME_INACT - 5 seconds 2 minutes*/
	//  ADXL345_SetRegisterValue(XL345_INT_MAP,XL345_INACTIVITY|XL345_ACTIVITY|XL345_WATERMARK|XL345_OVERRUN); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_INT_MAP,0x00); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	
	
	/*--------------------------------------------------
	Power, bandwidth-rate, interupt enabling
	--------------------------------------------------*/
//	ADXL345_SetRegisterValue(XL345_BW_RATE,XL345_RATE_12_5|XL345_LOW_POWER); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	//    ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ|XL345_MEASURE);/* TIME_INACT - 5 seconds 2 minutes*/

	// set the FIFO control XL345_RATE_12_5
	ADXL345_SetRegisterValue(XL345_FIFO_CTL,XL345_FIFO_MODE_STREAM|0|30); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_BW_RATE,XL345_RATE_25|XL345_LOW_POWER); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */

	// turn on the watermark interrupt and set the watermark interrupt to int1
#if(ADXL345_INTERRUPT>0)
//	ADXL345_SetRegisterValue(XL345_INT_ENABLE,XL345_INACTIVITY|XL345_ACTIVITY|XL345_WATERMARK|XL345_OVERRUN|31); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
	ADXL345_SetRegisterValue(XL345_INT_ENABLE,XL345_INACTIVITY|XL345_ACTIVITY); /*XL345_INACTIVITY|XL345_WATERMARK| XL345_OVERRUN THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
//	ADXL345_SetRegisterValue(XL345_INT_ENABLE,XL345_WATERMARK|XL345_OVERRUN|31); /* THRESH_INACT = 14/16 .25 Gee (1 lsb = 1/16 gee) */
#endif
	ADXL345_SetRegisterValue(XL345_POWER_CTL,ADXL345_PCTL_LINK|ADXL345_PCTL_AUTO_SLEEP|XL345_WAKEUP_8HZ|XL345_MEASURE);/* TIME_INACT - 5 seconds 2 minutes*/
	delay_ms(100);
}




#if defined (NEW_VERSION)

#define ANT2_INPUT                                   GPIO_Pin_5
#define ANT2_INPUT_EXTI_LINE                         EXTI_Line5
#define ANT2_INPUT_EXTI_PORT_SOURCE                  EXTI_PortSourceGPIOB
#define ANT2_INPUT_EXTI_PIN_SOURCE                   EXTI_PinSource5
#define ANT2_INPUT_EXTI_IRQn                         EXTI9_5_IRQn

#define ANT1_INPUT                                   GPIO_Pin_4
#define ANT1_INPUT_EXTI_LINE                         EXTI_Line4
#define ANT1_INPUT_EXTI_PORT_SOURCE                  EXTI_PortSourceGPIOB
#define ANT1_INPUT_EXTI_PIN_SOURCE                   EXTI_PinSource4
#define ANT1_INPUT_EXTI_IRQn                         EXTI4_IRQn
	
#else

#define ANT1_INPUT                                   GPIO_Pin_5
#define ANT1_INPUT_EXTI_LINE                         EXTI_Line5
#define ANT1_INPUT_EXTI_PORT_SOURCE                  EXTI_PortSourceGPIOB
#define ANT1_INPUT_EXTI_PIN_SOURCE                   EXTI_PinSource5
#define ANT1_INPUT_EXTI_IRQn                         EXTI9_5_IRQn

#define ANT2_INPUT                                   GPIO_Pin_4
#define ANT2_INPUT_EXTI_LINE                         EXTI_Line4
#define ANT2_INPUT_EXTI_PORT_SOURCE                  EXTI_PortSourceGPIOB
#define ANT2_INPUT_EXTI_PIN_SOURCE                   EXTI_PinSource4
#define ANT2_INPUT_EXTI_IRQn                         EXTI4_IRQn
	
#endif



void aint1_Config(void)
{

/* Private typedef -----------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = ANT1_INPUT;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,ANT1_INPUT_EXTI_PIN_SOURCE);

   EXTI_ClearITPendingBit(EXTI_Line4);
  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = ANT1_INPUT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = ANT1_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}





void aint2_Config(void)
{

/* Private typedef -----------------------------------------------------------*/
   EXTI_InitTypeDef   EXTI_InitStructure;
   GPIO_InitTypeDef   GPIO_InitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = ANT2_INPUT;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Connect EXTI0 Line to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,ANT2_INPUT_EXTI_PIN_SOURCE);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = ANT2_INPUT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = ANT2_INPUT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
