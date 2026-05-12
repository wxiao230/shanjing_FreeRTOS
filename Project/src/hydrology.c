
#include "hydrology.h"

#include "sensor.h"

#include "stm32_mems_adapter.h"
#include "ds1302.h"
#include "mode.h"
#include "gprs.h"
#include "power.h"
#include "spi_flash.h"
#include "usart.h"

uint16_t  bvTempValue= 0x00;
// sensorId devSensorId[9];

// void sensorIdRead(sensorId *id,devDep dep)
// {
//       uint8_t i = 0x00;
//       for(i= 0x00;i<devSenNum;i++);
// 	  	{
// 			id[i]->idUseFlag = SENSORID_USE_FLAG;
//                   switch(dep[i]->i)
//                   	{
//                            case 10 :
// 						{
//                                               id[i]->tempDataId = THE_10CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_10CM_SOIL_MOISTURE;
// 						    break;

// 						   }

//                            case 20 :
// 						{
//                                               id[i]->tempDataId = THE_20CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_20CM_SOIL_MOISTURE;
// 						    break;

// 						   }
//                            case 30 :
// 						{
//                                               id[i]->tempDataId = THE_30CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_30CM_SOIL_MOISTURE;
// 						    break;

// 						   }
//                            case 40 :
// 						{
//                                               id[i]->tempDataId = THE_40CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_40CM_SOIL_MOISTURE;
// 						    break;

// 						   }		
//                            case 50 :
// 						{
//                                              id[i]->tempDataId = THE_50CM_TEMPERATURT;
// 						   id[i]->waterDataId = THE_50CM_SOIL_MOISTURE;
// 						    break;

// 						   }	
//                            case 60 :
// 						{
//                                              id[i]->tempDataId = THE_60CM_TEMPERATURT;
// 						   id[i]->waterDataId = THE_60CM_SOIL_MOISTURE;
// 						    break;

// 						   }	
//                            case 80 :
// 						{
//                                               id[i]->tempDataId = THE_80CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_80CM_SOIL_MOISTURE;
// 						    break;

// 						   }	
//                            case 100 :
// 						{
//                                               id[i]->tempDataId = THE_100CM_TEMPERATURT;
// 						    id[i]->waterDataId = THE_100CM_SOIL_MOISTURE;
// 						    break;

// 						   }	
// 						   default :
// 						   	id[i]->idUseFlag = SENSORID_UNUSE_FLAG;
// 						   	break;
// 				  }

// 	  }


// }

/*
水文规约变量定义
*/




/**
  * @brief  Update CRC16 for input byte
  * @param  CRC input value 
  * @param  input byte
  * @retval None
  */
//uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
//{
//  uint32_t crc = crcIn;
//  uint32_t in = byte | 0x100;

//  do
//  {
//    crc <<= 1;
//    in <<= 1;
//    if(in & 0x100)
//      ++crc;
//    if(crc & 0x10000)
//      crc ^= 0x1021;
//  }
//  
//  while(!(in & 0x10000));

//  return crc & 0xffffu;
//}


/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
//uint16_t Cal_CRC16(const uint8_t* data, uint32_t size)
//{
//  uint32_t crc = 0;
//  const uint8_t* dataEnd = data+size;

//  while(data < dataEnd)
//    crc = UpdateCRC16(crc, *data++);
// 
//  crc = UpdateCRC16(crc, 0);
//  crc = UpdateCRC16(crc, 0);

//  return crc&0xffffu;
//}


uint8_t bcd2hex(uint8_t bcdData)
{
   uint8_t temp;
	 temp = (bcdData/16*10+bcdData%16);
	 return temp;
}

uint8_t hex2bcd(uint8_t hexData)
{
   uint8_t temp;
	 temp = (hexData/10*16 + hexData%10);
   return temp;
}

uint16_t frFromatAscii(uint8_t *bufin,uint8_t *bufOut)
{


    uint16_t datai = 0x00;
	uint16_t datat= 0x00;
	uint16_t dep = 0x00;
	uint8_t nodei = 0x00;
    uint16_t tempInt = 0x00;
    int16_t temp2int = 0;
    //uint16_t bvTemp = 0x00;
//	uint16_t dataOutLength= 0x00;
	uint16_t dataInLength = 0x00;
//	char dataTimeBuf[10];
	DS1302_TIME time;
//	value_t tempId ;
	value_t tempValue;
	value_t tempIdInt;

			
/*采集时间装载*/
	time.year  = bufin[dataInLength++];
	time.month = bufin[dataInLength++];
	time.date  = bufin[dataInLength++];
	time.hour  = bufin[dataInLength++];
	time.min   = bufin[dataInLength++];
	time.sec   = bufin[dataInLength++];	
	

	 datai =  sprintf((char *)(bufOut+datat), "TT %02d%02d%02d%02d",time.month,time.date,time.hour,time.min);
	 datat +=datai; 	
// 	 bufOut[datat++]	= 0x20;
//  	 datai =  sprintf((char *)(bufOut+datat), "%02d",time.month);
//      datat +=datai; 	 
//  	 datai =  sprintf((char *)(bufOut+datat), "%02d",time.date);
//      datat +=datai; 	
//  	 datai =  sprintf((char *)(bufOut+datat), "%02d",time.hour);
//      datat +=datai; 
//  	 datai =  sprintf((char *)(bufOut+datat), "%02d",time.min);
//      datat +=datai; 

     dataInLength = 20;          
//地表温度
    if(topSensorFlag == TOP_SENSOR_USE)
	   {
// 		  bufOut[datat++]	= 0x20;	
// 	      datai =  sprintf((char *)(bufOut+datat), "%s","GTP");
// 		  datat +=datai; 	
// 		  bufOut[datat++]	= 0x20;		
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
          temp2int = (int16_t)(tempValue.f*10);
	      datai =  sprintf((char *)(bufOut+datat), " GTP %d",temp2int);
		  datat +=datai; 	
		 }			 
//
	for(nodei = 0x00;nodei < devSenNum ;nodei ++)
	{
		  dep = (uint16_t)curDep[nodei].dep.f;
// 		  bufOut[datat++]	= 0x20;	
// 	      datai =  sprintf((char *)(bufOut+datat), "TTP%d",dep);
// 		  datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
	
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
          temp2int = (int16_t)(tempValue.f*10);
	      datai =  sprintf((char *)(bufOut+datat), " TTP%d %d",dep,temp2int);
		  datat +=datai; 		
	//高频
	      tempIdInt.bytes.value1 = bufin[dataInLength++];
		  tempIdInt.bytes.value2 = bufin[dataInLength++];
          tempIdInt.bytes.value3 = bufin[dataInLength++];
		  tempIdInt.bytes.value4 = bufin[dataInLength++];
					
// 		  bufOut[datat++]	= 0x20;
// 	      datai =  sprintf((char *)(bufOut+datat), "M%d",dep);
// 		  datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
//计算土壤水分            
                             // printf("当前水分值1为%.1f\n\r",tempIdInt.f);
          tempIdInt.f = calFth(tempValue.f,tempIdInt.f);      //温度补偿
                             //printf("当前水分值为2[%.1f]\n\r",tempIdInt.f);
          tempIdInt.f=  calSf(curFaw[nodei].Fa.f,curFaw[nodei].Faw.f, tempIdInt.f);  
													   //printf("当前水分值为 3  [%.5f]\n\r",tempIdInt.f);
          tempIdInt.f = calWater(tempIdInt.f,curAbc[nodei].a.f,curAbc[nodei].b2.f,curAbc[nodei].c.f);	
						                 //printf("当前水分值为4[%.1f]\n\r",tempIdInt.f);
          
         //datai =  sprintf((char *)(bufOut+datat), "%.1f",tempIdInt.f);
        // datai =  sprintf((char *)(bufOut+datat), "%.1f",(1.57+nodei));
                  tempInt = (uint16_t)(tempIdInt.f*10);
		  datai =  sprintf((char *)(bufOut+datat)," M%d %d",dep,tempInt);
		  datat +=datai; 
//低频盐分数据装载
         if(eachDevNum == 0x02)
		 {	
	         tempValue.bytes.value1 = bufin[dataInLength++];
		     tempValue.bytes.value2 = bufin[dataInLength++];
             tempValue.bytes.value3 = bufin[dataInLength++];
		     tempValue.bytes.value4 = bufin[dataInLength++];
         	}
	
	}
             
        
         dataInLength = 6;       
//采集次数装载
       
               //zong
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
                //caiji
		  tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","CJC");
// 		  datat +=datai; 	
// 		  bufOut[datat++]	= 0x20;
		  

          datai =  sprintf((char *)(bufOut+datat), " CJC %d",tempValue.j);
				datat +=datai; 
//电池电压
	
// 		  bufOut[datat++]	= 0x20;	
// 	      datai =  sprintf((char *)(bufOut+datat), "%s","BV");
// 		  datat +=datai; 	
// 		  bufOut[datat++]	= 0x20;	
		  tempValue.bytes.value1 = bufin[dataInLength++];			
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];		 
	      bvTempValue = (uint16_t)(tempValue.f*10);
// 	      datai =  sprintf((char *)(bufOut+datat), "%d",tempInt);
// 		  datat +=datai; 
//外部输入电压
// 		  bufOut[datat++]	= 0x20;	
// 	      datai =  sprintf((char *)(bufOut+datat), "%s","VTIN");
// 		  datat +=datai; 	
// 		  bufOut[datat++]	= 0x20;	
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
	      tempInt = (uint16_t)(tempValue.f*10);
	      datai =  sprintf((char *)(bufOut+datat), " VTIN %d",tempInt);
	      datat +=datai; 	
          	
          //endAddress =  dataInLength;    
          //printf("当前水分值1为%d  %d \n\r",begAddress,endAddress); 
		  
           return datat;
}


uint16_t  xyzFromatAscii(uint8_t *bufin,uint8_t *bufOut,uint16_t index)
{
	
       uint16_t datai = 0x00;
	uint16_t datat= 0x00;	
	uint16_t dataInLength = 0x00;
    int16_t tempInt = 0;
//	value_t tempId ;
	value_t tempValue;	
//x轴

// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","XACC");
// 				datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
                            tempInt = (int16_t)(tempValue.f*1000);
		  datai =  sprintf((char *)(bufOut+datat)," XACC %d",tempInt);
              //  datai =  sprintf((char *)(bufOut+datat), "%.3f",tempValue.f);
				datat +=datai; 			  
		  
//y轴

// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","YACC");
// 				datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
		  
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
		  
                            tempInt = (int16_t)(tempValue.f*1000);
		  datai =  sprintf((char *)(bufOut+datat)," YACC %d",tempInt);
				datat +=datai; 			  
//z轴

// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","ZACC");
// 				datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
          tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
          tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
		  
                            tempInt = (int16_t)(tempValue.f*1000);
		  datai =  sprintf((char *)(bufOut+datat)," ZACC %d",tempInt);
				datat +=datai; 	
				datat += index;
		  return datat;
}


uint16_t  gpsFromatAscii(uint8_t *bufin,uint8_t *bufOut,uint16_t index)
{
	//uint16_t dataOutLength= 0x00;
	uint16_t dataInLength = 0x00;
    uint16_t datai = 0x00;
    uint32_t temp = 0;
	uint16_t datat= 0x00;
//	value_t   tempId ;
	dou2Buf  tempValue;

//经度
// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","LNG");
// 				datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
                tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
                tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
                tempValue.bytes.value5 = bufin[dataInLength++];
		  tempValue.bytes.value6 = bufin[dataInLength++];
                tempValue.bytes.value7 = bufin[dataInLength++];
		  tempValue.bytes.value8 = bufin[dataInLength++];
                temp = (uint32_t)tempValue.f*100000;
               // datai =  sprintf((char *)(bufOut+datat), "%.5f",tempValue.f);
               datai =  sprintf((char *)(bufOut+datat), " LNG %d",temp);
				datat +=datai; 

//纬度
// 		  bufOut[datat++]	= 0x20;	
// 	         datai =  sprintf((char *)(bufOut+datat), "%s","LAT");
// 				datat +=datai;				
// 		  bufOut[datat++]	= 0x20;	
                tempValue.bytes.value1 = bufin[dataInLength++];
		  tempValue.bytes.value2 = bufin[dataInLength++];
                tempValue.bytes.value3 = bufin[dataInLength++];
		  tempValue.bytes.value4 = bufin[dataInLength++];
                tempValue.bytes.value5 = bufin[dataInLength++];
		  tempValue.bytes.value6 = bufin[dataInLength++];
                tempValue.bytes.value7 = bufin[dataInLength++];
		  tempValue.bytes.value8 = bufin[dataInLength++];		  
                temp = (uint32_t)tempValue.f*100000;
               // datai =  sprintf((char *)(bufOut+datat), "%.5f",tempValue.f);
               datai =  sprintf((char *)(bufOut+datat), " LAT %d",temp);
				datat +=datai; 
				datat += index;
		  return datat;				
}



/*
从FLASH中读取出数据，并还原成相关数值
*/
uint8_t frReadFromFlash(uint32_t readAddr,uint8_t *readBuf,uint8_t *datType)
{
	uint16_t length = 0x00;
	int8_t readResult = -1;
	uint8_t nodei = 0x00;
	uint8_t crc = 0x00;
	uint16_t readBufLength = 0x00;
	uint8_t buf[SENSOR_DATA_STORE_SIZE+1];

	//读取传感器数据
	readResult = flashStoreDataRead(readAddr,SENSOR_DATA_STORE_SIZE,buf,&length);
	if(readResult ==1)
	{
		 *datType = buf[3];
		 readBufLength = frFromatAscii(&buf[4],readBuf);
		//捡出三轴数据
		//crc = 0x00;
		if((buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST]==0x2c)&&(buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+1]==0x2c))
		{               
			//crc = 0x00; 
			for(nodei = 0;nodei < 18;nodei++)
			{
				crc += buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+3+nodei];
			}

			if(crc == buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+21])
			{
													 //usart2StoreBuffer(&buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST],21);	   
				readBufLength =xyzFromatAscii(&buf[XYZ_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST +9],&readBuf[readBufLength],readBufLength); 
			}
		}					
		crc = 0x00;
		//捡出三轴数据
		if((buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST]==0x2c)&&(buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST+1]==0x2c))
		{
			for(nodei = 143;nodei < 166;nodei++)
			{
				crc += buf[nodei];
			} 
			if(crc == buf[166])
			{
				readBufLength = gpsFromatAscii(&buf[GPS_VALUE_STORE_WIHH_SENSOR_DATA_OFFEST +9],&readBuf[readBufLength],readBufLength); 	   
			}
		}	   
	}
	return readBufLength;
}


uint16_t frHeadFromat(uint8_t *buf)
{
    uint8_t sendi = 0x00;
    uint8_t sendj = 0x00;
    
	sendi =  sprintf((char *)buf, "%s ","ST");
	sendj+=sendi;				
	//buf[sendj++] = 0x20;
    
	for(sendi   = 0x01;sendi  <devPar.stationAddr[0];sendi ++)
	{
			buf[sendj++] = devPar.stationAddr[sendi];
	}    	
    return sendj;
}

uint16_t frLiaisonPackage(uint8_t *buf)
{
    uint16_t length = 0x00;
    length = frHeadFromat(buf);
    length = packetCrc(buf,length);
    
    return length;    
}

#define MAX_STATION_PACKET   0x03
int8_t frMultSensorDataLoad(uint8_t *buf,uint32_t sendAddr, uint16_t *lentth,uint16_t readSendPacket,uint8_t *sendPacket,uint8_t *dataType)
	{
	uint8_t sendi = 0x00;
	uint8_t sendj = 0x00;
	//	uint8_t sendPacket = 0x00;
	uint16_t pcketLength = 0x00;

	int8_t 	result = -1;
	pcketLength = 0x00 ;
	*lentth = 0x00;
		
	sendi =  sprintf((char *)buf, "%s","ST");
	sendj+=sendi;				
	buf[sendj++]	= 0x20;	

	for(sendi   = 0x01;sendi  <devPar.stationAddr[0];sendi ++)
	{
		buf[sendj++] = devPar.stationAddr[sendi];
	}	
	buf[sendj++]	= 0x20;	
	*lentth = sendj;

	//多包数据发送         
	if(readSendPacket > signPackNUm)
	{
		sendj = signPackNUm;
	} 
	else
	{
		sendj = readSendPacket;
	}
	//sendj = 0x01;
	power331On();//打开存储FLASH电源
	SPI_Flash_WAKEUP();
	//装载数据
	//	printf("\n\r本次数据发送4:%d\n\r",pcketLength);

	for(sendi = 0x00;sendi < sendj; )
	{
		//读取并装载
		if(sendAddr == sensorDataWriteAddr)
		{      
			sendj= sendi;
			break;
		}

		pcketLength =  frReadFromFlash(sendAddr,&buf[*lentth],dataType);
		if(pcketLength!= 0x00)
		{
			sendi ++ ;
			//加入分割字符
			//printf("\n\r本次数据发送3:%d\n\r",pcketLength);
			//扣除时间字符串长度
			/*
			packetLoadLength.i = pcketLength -12;
			*lengthAddress = *lentth +8;	   
			buf[(*lentth)+8] =    packetLoadLength.bytes.value4;
			buf[(*lentth)+9] =    packetLoadLength.bytes.value3;
			buf[(*lentth)+10] =  packetLoadLength.bytes.value2;
			buf[(*lentth)+11] =  packetLoadLength.bytes.value1;  
			*/
			(*lentth)   = (*lentth)  +pcketLength; 
			buf[*lentth]	= 0x20;	
			*lentth =*lentth+1;		 
		}
		sendAddr += SENSOR_DATA_STORE_SIZE;
		if(sendAddr >= SENSOR_DATA_END_ADDR)
		{
			sendAddr = SENSOR_DATA_BASE_ADDR;
		}
		// printf("\n\r packet:%d ,length:%d,addr:%d \n\r",sendi,*lentth,sendAddr);
	}
	SPI_Flash_PowerDown();
	power331Off();
	if(sendi != 0x00)
	{
		*sendPacket = sendj;
		result = 1;
	}
	return result;
}

void stationAddressTest(void)
{
      uint8_t i = 0x00;
	  for(i = 0x01;i<11;i++)
	  	{   if(i>3)
            {
            devPar.stationAddr[i] = 0x30+i-3;
            }else
            {
             devPar.stationAddr[i] = 0x30; 
            }

	  } 
//          devPar.stationAddr[1] = 'S';
//          devPar.stationAddr[2] = 'S';
//          devPar.stationAddr[3] = 'X';
//          devPar.stationAddr[4] = 'T';
//          devPar.stationAddr[5] = 0x30;
//          devPar.stationAddr[6] = 0x30;
//          devPar.stationAddr[7] = 0x30;
//          devPar.stationAddr[8] = 0x31;
//          devPar.stationAddr[9] = 0x37;
//          devPar.stationAddr[10] = 0x37;    
//          devPar.stationAddr[0] = 11; 
    
         devPar.stationAddr[0] = 11;          
         devPar.stationKey[1] = 3+0x30;
         devPar.stationKey[2] = 7+0x30;
         devPar.stationKey[3] = 1+0x30;
         devPar.stationKey[4] = 8+0x30;
         devPar.stationKey[5] = 0+0x30;
         devPar.stationKey[6] = 0+0x30;
         devPar.stationKey[0] = 7;
          
}

/*
四川规约校验方法
*/
uint16_t packetCrc(uint8_t *buf,uint16_t index)
{
     uint8_t crc = 0x00;
	 uint8_t crcTemp = 0x00;
     uint16_t i = 0x00;
     value_t temp;	 
     buf[index++] = 0x20;  
	 for(i= 0x00;i < index ;i++)
	 	{
        crc += buf[i];
	 }
 	 crcTemp = ~crc;
	 //
    crcTemp +=1;
    temp.bytes.value2 = crcTemp&0x0F;
	temp.bytes.value1 = ((crcTemp&0xF0)>>4); 

	temp.j += 0x6161;
    
	buf[index++] = temp.bytes.value1;
	buf[index++] = temp.bytes.value2;	
   // printf("\r\n 2crc:[%d] [%d]\r\n",temp.bytes.value1,temp.bytes.value2);
	buf[index++] = 0x4E;	
	buf[index++] = 0x4E;	
	return index;
}

uint16_t testModePacketEnd(uint8_t *buf,uint16_t index)
{
    buf[index++] = 0x20; 
 	buf[index++] = 0x3A;	
	buf[index++] = 0x3A;    
 	buf[index++] = 0x4E;	
	buf[index++] = 0x4E;	
	return index;   
}


void crcTestPacket(void)
{
   uint8_t testChar[100] ="ST 6006676541 ZA01 52200 DZA01 60";
   uint16_t length = 0;
   length = packetCrc(testChar,33);
   usart2StoreBuffer(testChar,length);
   




}


