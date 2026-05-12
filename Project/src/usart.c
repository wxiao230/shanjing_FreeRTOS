  
#include "stm32l1xx.h"

#include "usart.h"
#include "main.h"
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  

#define USART_DEBUG      0

		 /**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */

__IO uint8_t	usart1Status	= USART_OFF;
__IO uint8_t	usart2Status	= USART_OFF;
__IO uint8_t	usart3Status	= USART_OFF;
__IO uint8_t rs232BaudRate = DEFAULT_BAUDRATE;

void USART2_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
	USART_DeInit(USART2);
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

/*

*/	  	

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
/*  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
*/  
  /* Enable USART */
  USART_Cmd(USART2, ENABLE);
	usart2Status	= USART_ON;
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#if   (USART_DEBUG > 0)
	printf("\n\r����ӯʤ̩��ũҵ������v3.0��\n\r");
#endif
//  usart2StoreBuffer("12345",5);
}

		 /**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
void USART1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_DeInit(USART1);
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_9|GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 230400 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
  
  /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
	usart1Status	= USART_ON;
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


uint8_t uartPutchar(USART_TypeDef* USARTx,__IO uint8_t ch)
{
	uint16_t i = 10000;	
	while ((USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)&&(i--))
	{
	}
	USART_SendData(USARTx,(uint8_t)ch);
	return ch;
}
void usart1StoreBuffer(uint8_t *buf,uint16_t length)
{
	uint16_t i=0;
	if(xSemaphoreTake(xUSART1Mutex, portMAX_DELAY) == pdTRUE)
	{
		for(i=0;i<length;i++)
		{
			uartPutchar(USART1,buf[i]);
		}
		i=10000;	
		while ((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)&&(i--))
		{
		}
		xSemaphoreGive(xUSART1Mutex);
	}
}
void usart2StoreBuffer(uint8_t *buf,uint16_t length)
{
	uint16_t i=0;	
	for(i=0;i<length;i++)
	{
		uartPutchar(USART2,buf[i]);
	}
	i=10000;	
	while ((USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)&&(i--))
	{
	}
}
void uart3StoreBuffer(uint8_t *buf,uint16_t length)
{
	uint16_t i=0;
	if(xSemaphoreTake(xUSART3Mutex, portMAX_DELAY) == pdTRUE)
	{
		for(i=0;i<length;i++)
		{
			uartPutchar(USART3,buf[i]);
		}
		i=10000;	
		while ((USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)&&(i--))
		{
		}
		xSemaphoreGive(xUSART3Mutex);
	}
}
void usart1SendBuf(char const *buf,uint16_t length)
{
	uint16_t i=0;
	if(xSemaphoreTake(xUSART1Mutex, portMAX_DELAY) == pdTRUE)
	{
		for(i=0;i<length;i++)
		{
			uartPutchar(USART1,buf[i]);
		}
		i=10000;	
		while ((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)&&(i--))
		{
		}
		xSemaphoreGive(xUSART1Mutex);
	}
}
void usart2SendBuf(char const *buf,uint16_t length)
{
	uint16_t i=0;	
	for(i=0;i<length;i++)
	{
		uartPutchar(USART2,buf[i]);
	}
	i=10000;	
	while ((USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)&&(i--))
  {
  }
}
void usart3SendBuf2(char const *buf,uint16_t length)
{
	uint16_t i=0;
	if(xSemaphoreTake(xUSART3Mutex, portMAX_DELAY) == pdTRUE)
	{
		for(i=0;i<length;i++)
		{
			uartPutchar(USART3,buf[i]);
		}
		i=10000;	
		while ((USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)&&(i--))
		{
		}
		xSemaphoreGive(xUSART3Mutex);
	}
}

void uart1LowPower(void)
{
//  USART_InitTypeDef USART_InitStructure;
 // NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  usart1Status	= USART_OFF;
    /* Enable USART */
  USART_Cmd(USART1, DISABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
    /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;   
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void uart2LowPower(void)
{
//  USART_InitTypeDef USART_InitStructure;
 // NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  usart2Status	= USART_OFF;
    /* Enable USART */
  USART_Cmd(USART2, DISABLE);

  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    /* Enable USART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);
    /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode =	GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_400KHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_UP;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  * @by     WWW.ARMJISHU.COM 
  */
PUTCHAR_PROTOTYPE
{
  /* ARMJISHU ��ȴ���Printf���ڴ�ӡ����������
            д��Buffer����������(Bufferδ��ʱ) */

 
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}
  USART_SendData(USART2, (uint8_t) ch);


// return ch;
  return ch;
}

/**
  * @brief  Test to see if a key has been pressed on the HyperTerminal
  * @param  key: The key pressed
  * @retval 1: Correct
  *         0: Error
  */
uint32_t SerialKeyPressed(uint8_t *key)
{

  if( USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
  {
    *key = (uint8_t)USART2->DR;
    return 1;
  }
  else
  {
    return 0;
  }
}


/**
  * @brief  Print a character on the HyperTerminal
  * @param  c: The character to be printed
  * @retval None
  */
void SerialPutChar(uint8_t c)
{
  USART_SendData(USART2, c);
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
  {
  }
}

/**
  * @brief  Get a key from the HyperTerminal
  * @param  None
  * @retval The Key Pressed
  */
uint8_t GetKey(void)
{
  uint8_t key = 0;

  /* Waiting for user input */
  while (1)
  {
    if (SerialKeyPressed((uint8_t*)&key)) break;
  }
  return key;

}



