/**
  ******************************************************************************
  * @file    IAP_Main/Src/ymodem.c 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides all the software functions related to the ymodem 
  *          protocol.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/** @addtogroup STM32L4xx_IAP
  * @{
  */ 
  
/* Includes ------------------------------------------------------------------*/
//#include "flash_if.h"
#include "common.h"
#include "ymodem.h"
#include "string.h"
#include <stdio.h>
#include "spi_flash.h"
#include "qingxie.h"
//#include "usart_1.h"
////#include "main.h"
//#include "menu.h"

 
extern USART_TypeDef* UartHandle;
 
#ifdef _USE_HW_CRC_
	extern CRC_HandleTypeDef CrcHandle;
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CRC16_F       /* activate the CRC16 integrity */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* @note ATTENTION - please keep this variable 32bit alligned */
 
uint8_t * aPacketData;//[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];

/* Private function prototypes -----------------------------------------------*/
static void PrepareIntialPacket(uint8_t *p_data, const uint8_t *p_file_name, uint32_t length);
static void PreparePacket(uint8_t *p_source, uint8_t *p_packet, uint8_t pkt_nr, uint32_t size_blk);
static HAL_StatusTypeDef ReceivePacket(uint8_t *p_data, uint32_t *p_length, uint32_t timeout);
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);
uint8_t CalcChecksum(const uint8_t *p_data, uint32_t size);




extern uint8_t   aFileName[FILE_NAME_LENGTH];
extern uint8_t * aFileBody;


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Transmit a data packet using the ymodem protocol
  * @param  data
  * @param  length
  * @retval None
  */
void Ymodem_SendPacket(uint8_t *data, uint16_t length)
{
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Send_Byte(data[i]);
    i++;
  }
}

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
    if (SerialKeyPressedUsart3(c) == 1)
    {			
      return 0;
    }
  }
  return -1;
}

/**
  * @brief  Send a byte
  * @param  c: Character
  * @retval 0: Byte sent
  */
static uint32_t Send_Byte (uint8_t c)
{
  SerialPutCharUsart3(c);
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive (USART_TypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout){
	uint16_t i;
	for(i=0;i<Size;i++){
		if (Receive_Byte(pData + i, Timeout) != 0){
      return HAL_ERROR;
    }
	}
	return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(USART_TypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	uint16_t i=0;
  while (i < Size) {
    Send_Byte(pData[i++]);		
  }
	return HAL_OK;
}

/**
  * @brief  Receive a packet from sender
  * @param  data
  * @param  length
  *     0: end of transmission
  *     2: abort by sender
  *    >0: packet length
  * @param  timeout
  * @retval HAL_OK: normally return
  *         HAL_BUSY: abort by user
  */
static HAL_StatusTypeDef ReceivePacket(uint8_t *p_data, uint32_t *p_length, uint32_t timeout)
{
  uint32_t crc,crc_cal;
  uint32_t packet_size = 0;
  HAL_StatusTypeDef status;
  uint8_t char1=0;
  int i;
  *p_length = 0;
  status = HAL_UART_Receive(UartHandle, &char1, 1, timeout);

  if (status == HAL_OK)
  {
    switch (char1)
    {
      case SOH:
        packet_size = PACKET_SIZE;
        break;
      case STX:
        packet_size = PACKET_1K_SIZE;
        break;
      case EOT:
        break;
      case CA:
        if ((HAL_UART_Receive(UartHandle, &char1, 1, timeout) == HAL_OK) && (char1 == CA))
        {
          packet_size = 2;
        }
        else
        {
          status = HAL_ERROR;
        }
        break;
      case ABORT1:
      case ABORT2:
        status = HAL_BUSY;
        break;
      default:
        status = HAL_ERROR;
        break;
    }
    *p_data = char1;    
	*(p_data+1) = char1;
    
    if (packet_size >= PACKET_SIZE )
    {
      status = HAL_UART_Receive(UartHandle, &p_data[PACKET_NUMBER_INDEX], packet_size + PACKET_OVERHEAD_SIZE, timeout);

      /* Simple packet sanity check */
      if (status == HAL_OK )
      {
        if (p_data[PACKET_NUMBER_INDEX] != ((p_data[PACKET_CNUMBER_INDEX]) ^ NEGATIVE_BYTE))
        {
          packet_size = 0;
          status = HAL_ERROR;
        }
        else
        {
	      // printf("negative OK!(%d)(%d)[char1=%d]\n\r",p_data[PACKET_NUMBER_INDEX],p_data[PACKET_CNUMBER_INDEX],char1);
          /* Check packet CRC */
          crc = p_data[ packet_size + PACKET_DATA_INDEX ] << 8;
          crc += p_data[ packet_size + PACKET_DATA_INDEX + 1 ];
	#ifdef _USE_HW_CRC_
          crc_cal =  HAL_CRC_Calculate(&CrcHandle, (uint32_t*)&p_data[PACKET_DATA_INDEX], packet_size);
	#else
					crc_cal =  Cal_CRC16( (uint8_t*)&p_data[PACKET_DATA_INDEX], packet_size);	
	#endif
          if (crc_cal != crc )
          {
            packet_size = 0;
            status = HAL_ERROR;
            printf("crc error[0x%x][0x%x]\n\r",crc,crc_cal);
        
            printf("pkg datasize(%d),char1=0x%x\n\r",packet_size ,char1);//,ramsource);
            //for(i=0;i<packet_size+2;i++)
              //  printf("0x%x ", p_data[ i + PACKET_DATA_INDEX ]);
            printf("\n\r");
          }else{
            // printf("pkg CRC OK![0x%x][crc_cal=0x%x](aPacketData=0x%x)!\n\r",crc,crc_cal,(uint32_t)aPacketData);
					 
						for(i=0;i<packet_size+6;i++){
							//printf("%2x|", p_data[ i ]);
						}
              // printf("\n\r");
          }
        }
      }
      else
      {
        packet_size = 0;
      }
    }
  }
  *p_length = packet_size;
  return status;
}

/**
  * @brief  Prepare the first block
  * @param  p_data:  output buffer
  * @param  p_file_name: name of the file to be sent
  * @param  length: length of the file to be sent in bytes
  * @retval None
  */
static void PrepareIntialPacket(uint8_t *p_data, const uint8_t *p_file_name, uint32_t length)
{
  uint32_t i, j = 0;
  uint8_t astring[10]={0};

  /* first 3 bytes are constant */
  p_data[PACKET_START_INDEX] = SOH;
  p_data[PACKET_NUMBER_INDEX] = 0x00;
  p_data[PACKET_CNUMBER_INDEX] = 0xff;

  /* Filename written */
  for (i = 0; (p_file_name[i] != '\0') && (i < FILE_NAME_LENGTH); i++)
  {
    p_data[i + PACKET_DATA_INDEX] = p_file_name[i];
  }

  p_data[i + PACKET_DATA_INDEX] = 0x00;

  /* file size written */
  Int2Str (astring, length);
  i = i + PACKET_DATA_INDEX + 1;
  while (astring[j] != '\0')
  {
    p_data[i++] = astring[j++];
  }

  /* padding with zeros */
  for (j = i; j < PACKET_SIZE + PACKET_DATA_INDEX; j++)
  {
    p_data[j] = 0;
  }
}

/**
  * @brief  Prepare the data packet
  * @param  p_source: pointer to the data to be sent
  * @param  p_packet: pointer to the output buffer
  * @param  pkt_nr: number of the packet
  * @param  size_blk: length of the block to be sent in bytes
  * @retval None
  */
static void PreparePacket(uint8_t *p_source, uint8_t *p_packet, uint8_t pkt_nr, uint32_t size_blk)
{
  uint8_t *p_record;
  uint32_t i, size, packet_size;

  /* Make first three packet */
  packet_size = size_blk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
  size = size_blk < packet_size ? size_blk : packet_size;
  if (packet_size == PACKET_1K_SIZE)
  {
    p_packet[PACKET_START_INDEX] = STX;
  }
  else
  {
    p_packet[PACKET_START_INDEX] = SOH;
  }
  p_packet[PACKET_NUMBER_INDEX] = pkt_nr;
  p_packet[PACKET_CNUMBER_INDEX] = (~pkt_nr);
  p_record = p_source;

  /* Filename packet has valid data */
  for (i = PACKET_DATA_INDEX; i < size + PACKET_DATA_INDEX;i++)
  {
#ifdef _USE_FLASH_SOURCE_
      
       SPI_FLASH_BufferRead((uint8_t *)&p_packet[i],(uint32_t) p_record++,1);//take p_record as flash address(24b)
#else
		p_packet[i] = *p_record++;																			//take p_record as memory address(32b)
#endif
	 //printf("%c",p_packet[i]);
  }
	
  
	
  if ( size  <= packet_size)
  {
    for (i = size + PACKET_DATA_INDEX; i < packet_size + PACKET_DATA_INDEX; i++)
    {
      p_packet[i] = 0;//x1A; /* EOF (0x1A) or 0x00 */
    }
  }
}

/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = UpdateCRC16(crc, *p_data++);
 
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return crc&0xffffu;
}

/**
  * @brief  Calculate Check sum for YModem Packet
  * @param  p_data Pointer to input data
  * @param  size length of input data
  * @retval uint8_t checksum value
  */
uint8_t CalcChecksum(const uint8_t *p_data, uint32_t size)
{
  uint32_t sum = 0;
  const uint8_t *p_data_end = p_data + size;

  while (p_data < p_data_end )
  {
    sum += *p_data++;
  }

  return (sum & 0xffu);
}

/* Public functions ---------------------------------------------------------*/
/**
  * @brief  Receive a file using the ymodem protocol with CRC16.
  * @param  p_size The size of the file.
  * @retval COM_StatusTypeDef result of reception/programming
  */ 
COM_StatusTypeDef Ymodem_Receive (  uint8_t *p_file_name,   uint8_t **p_buf, uint32_t *p_size )
{
  uint32_t i, packet_length, session_done = 0, file_done, errors = 0, session_begin = 0;
  uint32_t flashdestination, ramsource;
	int32_t filesize;
  uint8_t *file_ptr;
	uint32_t	errTimes=0;
  uint8_t file_size[FILE_SIZE_LENGTH]={0}/*BugFix:compiler will not zero it.*/, tmp, packets_received;
  COM_StatusTypeDef result = COM_OK;

  /* Initialize flashdestination variable */
  flashdestination =0;// malloc();//APPLICATION_ADDRESS;

  while ((session_done == 0) && (result == COM_OK))
  {
    packets_received = 0;
    file_done = 0;
    while ((file_done == 0) && (result == COM_OK))
    {
      switch (ReceivePacket(aPacketData, &packet_length, DOWNLOAD_TIMEOUT))
      {
        case HAL_OK:
          errors = 0;
          switch (packet_length)
          {
            case 2:
              /* Abort by sender */
              Serial_PutByte(ACK);
              result = COM_ABORT;
              break;
            case 0:
              /* End of transmission */
              Serial_PutByte(ACK);
              file_done = 1;
              break;
            default:
              /* Normal packet */
              if (aPacketData[PACKET_NUMBER_INDEX] != packets_received)
              {
                printf("number err[%d][%d]\n\r",packets_received,aPacketData[PACKET_NUMBER_INDEX]);
                Serial_PutByte(NAK);
              }
              else
              {
                if (packets_received == 0)
                {
                  /* File name packet */
                  if (aPacketData[PACKET_DATA_INDEX] != 0)
                  {
                    /* File name extraction */
                    i = 0;
										file_ptr = aPacketData + PACKET_DATA_INDEX;
                    while ( (*file_ptr != 0) && (i < FILE_NAME_LENGTH))
                    {
                      p_file_name[i++] = *file_ptr++;
                    } 
                  
                  //   Int2Str(file_size,1567);   Str2Int(file_size, &filesize);
                  //   printf("Int2Str(file_size,1567),file_size=%s. transformed int=%d.file_size[4]=0x%x.\n\r",file_size,filesize,file_size[4]);
                    
                    /* File size extraction */
                    p_file_name[i++] = '\0';
                    i = 0;
                    file_ptr ++;//skip the zero byte between name and size
                    while ( /*(*file_ptr != ' ')BIGBUG!!*/(*file_ptr>='0') &&(*file_ptr<='9') && (i < FILE_SIZE_LENGTH))
                    {
                      file_size[i++] = *file_ptr++;
                    }
                    file_size[i++] = '\0';
                    Str2Int(file_size, &filesize);

                    /* Test the size of the image to be sent */
                    /* Image size is greater than Flash size */
										#ifdef _USE_FLASH_SOURCE_										
											if (*p_size > (_HOST_RECV_SIZE_LIMIT_1KB_ + 1))
										#else
											if (*p_size > (USER_FLASH_SIZE + 1))
										#endif
                    
										{
                      /* End session */
                      tmp = CA;
                      HAL_UART_Transmit(UartHandle, &tmp, 1, NAK_TIMEOUT);
                      HAL_UART_Transmit(UartHandle, &tmp, 1, NAK_TIMEOUT);
                      result = COM_LIMIT;
                    }
                    /* erase user application area */
                    //FLASH_If_Erase(APPLICATION_ADDRESS);
                    *p_size = filesize;										
									
											Serial_PutByte(ACK); 
											Serial_PutByte(CRC16);   //??receiver required this??
										}
										/* File header packet is empty, end session */
										else
                  {
                    Serial_PutByte(ACK);
                    file_done = 1;
                    session_done = 1;
                    break;
                  }
                }
                else /* Data packet */
                {
									
									if(!flashdestination){
										if(!(*p_buf) && filesize ){// its aFileBody(NULL),C buffer ,will need allocating space.
										#ifndef _USE_FLASH_SOURCE_
 										 	 *p_buf/*aFileBody*/ = malloc((size_t)filesize);  //GX: the right place to malloc space for out buffer. will free it outside the function somewhere
											 if(!(*p_buf)){
													printf("MALLOC failed!\n\r");
												}else{
													memset((uint8_t *)(*p_buf),0,filesize);// maybe important.
													printf("ALLOCATED [0x%x](filesize=%d)\n\r",(uint32_t)*p_buf,filesize);	
												}
									   #endif
										}
										flashdestination = (uint32_t)*p_buf;// JS buffer,already allocated space.										
									}
									
                  /* Write received data to out buffer */
                 // FLASH_If_Write(flashdestination, (uint32_t*) ramsource, packet_length/4);
									if(flashdestination){
									  ramsource = (uint32_t) & aPacketData[PACKET_DATA_INDEX];
										memcpy((uint8_t*)flashdestination, (uint8_t*)ramsource , packet_length);  
                                        printf("\r\nx:%d,y:%s\r\n",flashdestination,(uint8_t*)ramsource); 
										flashdestination += packet_length; 
										// printf("ramsource(%d)\n\r",packet_length);//,ramsource);                  
									}
                	  Serial_PutByte(ACK);
                }
                packets_received++;
                session_begin = 1;
              }
              break;
          }
          break;
        case HAL_BUSY: /* Abort actually */
          Serial_PutByte(CA);
          Serial_PutByte(CA);
          result = COM_ABORT;
          break;
        default:
          if (session_begin > 0)
          {
            errors ++;
          }
          if (errors > MAX_ERRORS)
          {
            /* Abort communication */
            Serial_PutByte(CA);
            Serial_PutByte(CA);
          }
          else
          {
            Serial_PutByte(CRC16); /* Ask for a packet */
						if(errTimes++>50)result = COM_ERROR; 
          }
          break;
      }
    }
  }
  return result;
}

/**
  * @brief  Transmit a file using the ymodem protocol
  * @param  p_buf: Address of the first byte
  * @param  p_file_name: Name of the file sent
  * @param  file_size: Size of the transmission
  * @retval COM_StatusTypeDef result of the communication
  */
 uint8_t blk_number ;

COM_StatusTypeDef Ymodem_Transmit (uint8_t *p_file_name,   uint8_t *p_buf, uint32_t file_size)
{
  uint32_t errors = 0, ack_recpt = 0, size = 0, pkt_size;
  uint8_t *p_buf_int;
  COM_StatusTypeDef result = COM_OK;
//  uint8_t blk_number = 0x01;
  uint8_t a_rx_ctrl[2];
  uint8_t i;
//	uint8_t stat = 0;
#ifdef CRC16_F    
  uint32_t temp_crc;
#else /* CRC16_F */   
  uint8_t temp_chksum;
#endif /* CRC16_F */  

  /* Prepare first block - header */
  PrepareIntialPacket(aPacketData, p_file_name, file_size);
  blk_number = 0x01;
			
  while (( !ack_recpt ) && ( result == COM_OK ))
  {
    /* Send Packet */
    HAL_UART_Transmit(UartHandle, &aPacketData[PACKET_START_INDEX], PACKET_SIZE + PACKET_HEADER_SIZE, NAK_TIMEOUT);
		//	printf("\r\n ~~~ send data OK \r\n");
	
    /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
   #ifdef _USE_HW_CRC_
          temp_crc =  HAL_CRC_Calculate(&CrcHandle, (uint32_t*)&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
	 #else
					temp_crc =  Cal_CRC16( (uint8_t*)&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);	
	 #endif
 //  printf("Header: calculted sent CRC:0x%x\n\r",temp_crc);
    Serial_PutByte(temp_crc >> 8);
    Serial_PutByte(temp_crc & 0xFF);
#else /* CRC16_F */   
    temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(temp_chksum);
#endif /* CRC16_F */

    /* Wait for Ack and 'C' BUG HERE*/
    //if (HAL_UART_Receive(&UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK)
    if (HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 2, NAK_TIMEOUT) == HAL_OK)
    {
      if ((a_rx_ctrl[0] == ACK)&& (a_rx_ctrl[1] == CRC16))
      {
        ack_recpt = 1;
      }
      else if (a_rx_ctrl[0] == CA)
      {
        if ((HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK) && (a_rx_ctrl[0] == CA))
        {
         // HAL_Delay( 2 );
          //__HAL_UART_FLUSH_DRREGISTER(&UartHandle);
          result = COM_ABORT;
        }
      }
			else//2018-6-30
			{
				errors++;
			}
    }
    else
    {
      errors++;
    }
    if (errors >= MAX_ERRORS)
    {
      result = COM_ERROR;
    }
  }

  p_buf_int = p_buf;
  size = file_size;

  /* Here 1024 bytes length is used to send the packets */
  while ((p_buf_int)&&(size) &&( size<FLASH_STORAGE_MAX_SIZE) && (result == COM_OK ))
  {
    /* Prepare next packet */
    PreparePacket(p_buf_int, aPacketData, blk_number, size);
    ack_recpt = 0;
    a_rx_ctrl[0] = 0;
    errors = 0;

    /* Resend packet if NAK for few times else end of communication */
    while (( !ack_recpt ) && ( result == COM_OK ))
    {
      /* Send next packet */
      if (size >= PACKET_1K_SIZE)
      {
        pkt_size = PACKET_1K_SIZE;
      }
      else
      {
        pkt_size = PACKET_SIZE;
      }

      HAL_UART_Transmit(UartHandle, &aPacketData[PACKET_START_INDEX], pkt_size + PACKET_HEADER_SIZE, NAK_TIMEOUT);
      
      /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
     #ifdef _USE_HW_CRC_
          temp_crc = HAL_CRC_Calculate(&CrcHandle, (uint32_t*)&aPacketData[PACKET_DATA_INDEX], pkt_size);
		#else
		  		temp_crc =  Cal_CRC16( (uint8_t*)&aPacketData[PACKET_DATA_INDEX], pkt_size);	
		#endif
   //   printf("\n\rBody:calculted sent CRC:0x%x\n\r",temp_crc);
      Serial_PutByte(temp_crc >> 8);
      Serial_PutByte(temp_crc & 0xFF);
#else /* CRC16_F */   
      temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], pkt_size);
      Serial_PutByte(temp_chksum);
#endif /* CRC16_F */
      
      /* Wait for Ack */
      if ((HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK) && (a_rx_ctrl[0] == ACK))
      {
        ack_recpt = 1;
        if (size > pkt_size)
        {
          p_buf_int += pkt_size;
          size -= pkt_size;
          if (blk_number == (USER_FLASH_SIZE / PACKET_1K_SIZE))
          {
            result = COM_LIMIT; /* boundary error */
          }
          else
          {
            blk_number++;
          }
        }
        else
        {
          p_buf_int += pkt_size;
          size = 0;
        }
      }
      else
      {
        errors++;
      }

      /* Resend packet if NAK  for a count of 10 else end of communication */
      if (errors >= MAX_ERRORS)
      {
        result = COM_ERROR;
      }
    }
  }

  /* Sending End Of Transmission char */
  ack_recpt = 0;
  a_rx_ctrl[0] = 0x00;
  errors = 0;
	
  while (( !ack_recpt ) && ( result == COM_OK ))
  {
    Serial_PutByte(EOT);

    /* Wait for Ack */
    if (HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK)
    {
      if (a_rx_ctrl[0] == ACK)
      {
        ack_recpt = 1;
      }
      else if (a_rx_ctrl[0] == CA)
      {
        if ((HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK) && (a_rx_ctrl[0] == CA))
        {
          usart3_delay(2000);// usart_dealy HAL_Delay( 2 );
          //__HAL_UART_FLUSH_DRREGISTER(&UartHandle);
          result = COM_ABORT;
        }
      }
			else//2018-6-30
			{
				errors++;
			}
    }
    else
    {
      errors++;
    }

    if (errors >=  MAX_ERRORS)
    {
      result = COM_ERROR;
    }
  }

  /* Empty packet sent - some terminal emulators need this to close session */
  if ( result == COM_OK )
  {
    /* Preparing an empty packet */
    aPacketData[PACKET_START_INDEX] = SOH;
    aPacketData[PACKET_NUMBER_INDEX] = 0;
    aPacketData[PACKET_CNUMBER_INDEX] = 0xFF;
    for (i = PACKET_DATA_INDEX; i < (PACKET_SIZE + PACKET_DATA_INDEX); i++)
    {
      aPacketData [i] = 0x00;
    }

    /* Send Packet */
    HAL_UART_Transmit(UartHandle, &aPacketData[PACKET_START_INDEX], PACKET_SIZE + PACKET_HEADER_SIZE, NAK_TIMEOUT);

    /* Send CRC or Check Sum based on CRC16_F */
#ifdef CRC16_F    
					#ifdef _USE_HW_CRC_
						temp_crc =  HAL_CRC_Calculate(&CrcHandle, (uint32_t*)&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
					#else
						temp_crc =  Cal_CRC16( (uint8_t*)&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);	
					#endif
  //  printf("\n\rTail: calculted sent CRC:0x%x\n\r",temp_crc);
    Serial_PutByte(temp_crc >> 8);
    Serial_PutByte(temp_crc & 0xFF);
#else /* CRC16_F */   
    temp_chksum = CalcChecksum (&aPacketData[PACKET_DATA_INDEX], PACKET_SIZE);
    Serial_PutByte(temp_chksum);
#endif /* CRC16_F */

    /* Wait for Ack and 'C' */
    if (HAL_UART_Receive(UartHandle, &a_rx_ctrl[0], 1, NAK_TIMEOUT) == HAL_OK)
    {
      if (a_rx_ctrl[0] == CA)
      {
          usart3_delay( 2 );
         // __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
          result = COM_ABORT;
      }
    }
  }

  return result; /* file transmitted successfully */
}

/**
  * @}
  */

/*******************(C)COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
