#ifndef USART_H
#define USART_H
#include "stm32l1xx.h"

#define USART_OFF						0x00
#define USART_ON						0x01

#define BAUD_1200							1200
#define BAUD_2400							2400
#define BAUD_4800							4800
#define BAUD_9600							9600
#define BAUD_19200						19200
#define BAUD_38400						38400
#define BAUD_57600						57600
#define BAUD_115200						115200
#define BAUD_230400						230400	
#define BAUD_460800						460800	
#define IS_USART_BAUD(BAUD) 		(((BAUD) == BAUD_1200) || \
                                 ((BAUD) == BAUD_2400) || \
                                 ((BAUD) == BAUD_4800) || \
                                 ((BAUD) == BAUD_9600) || \
																 ((BAUD) == BAUD_19200)|| \
																 ((BAUD) == BAUD_38400)|| \
																 ((BAUD) == BAUD_57600)|| \
																 ((BAUD) == BAUD_230400)|| \
																 ((BAUD) == BAUD_460800)|| \
                                 ((BAUD) == BAUD_115200))
#define BAUD_INIT_USART1			BAUD_115200
#define BAUD_INIT_USART2			BAUD_9600
#define BAUD_INIT_USART3			BAUD_115200
#define BAUD_INIT_USART32			BAUD_115200

#define BAUDRATE_9600    			0x01
#define BAUDRATE_14400   			0x02
#define BAUDRATE_19200   			0x03
#define BAUDRATE_38400   			0x04
#define BAUDRATE_56000   			0x05
#define BAUDRATE_57600   			0x06
#define BAUDRATE_115200  			0x00
#define DEFAULT_BAUDRATE    				0x00

#define RX_OVER		0x02
#define RX_STATE  0x01
#define UN_RX     0x00

#define UN_FIND_STATUS   0x00000FFF
#define FIND_ERROR       0x00000FFE


#define USART2_INTERPT_DISABLE    0x01
#define USART2_INTERPT_ENABLE     0x02


extern __IO uint8_t		usart1Status;
extern __IO uint8_t		usart2Status;
extern __IO uint8_t		usart3Status;
extern __IO uint8_t rs232BaudRate;
extern void USART2_Config(void);
extern void uart2LowPower(void);
extern void uart1LowPower(void);
extern void USART1_Config(void);
extern void usart1SendBuf(char const *buf,uint16_t length);
extern void usart2SendBuf(char const *buf,uint16_t length);
extern void usart3SendBuf2(char const *buf,uint16_t length);
extern void usart1StoreBuffer(uint8_t  *buf,uint16_t length);
extern void usart2StoreBuffer(uint8_t  *buf,uint16_t length);
extern void uart3StoreBuffer(uint8_t  *buf,uint16_t length);

extern uint32_t SerialKeyPressed(uint8_t *key);
extern void SerialPutChar(uint8_t c);
extern uint8_t GetKey(void);
#endif 
