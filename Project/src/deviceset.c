#include "usart.h"
#include "deviceset.h"
#include "ds18b20.h"
#include "time.h"
#include "delay.h"
#include "rtc.h"
#include "power.h"
#include "spi_flash.h"
#include "ds1302.h"
#include "stdio.h"
#include "gprs.h"
#if(USART_USE_CTR>0)
	#include "rs485.h"
#endif
#if(MQTT_USE_CTR>0)
	#include "cigemPacket.h"
#endif
#if((SK60_CTR>0)||(KXYL_CTR>0))
	#include "sk60.h"
#endif

//__IO uint8_t  uart2RxTimeCounter = 0x00;
//__IO uint32_t setTimeCounter = 0x00;
//__IO uint32_t setTimeALLCounter = 0x00;
//
//__IO uint16_t readIdCounter = 0x00;

#define REQUEST_INIT_STATE    0x01
#define REQUEST_STATE         0x02
#define REQUEST_END_STATE     0x04


#define WAIT_ACK              0x01
#define SEND_DATA             0x02
#define SEND_ACK              0x04
#define NO_ACTION             0x08

/*
串口相关标志位
*/
#define UART_DATA_FLAG     0x01

/*
设置相关标志位
*/
#define DEVICE_SET_QUIT    0x80

#define USART2_RX_DATA_SIZE    0xFF

//__IO uint8_t actionFlag = 0x00;
//uint8_t  UATT2_RX_Buffer [USART2_RX_DATA_SIZE+1];
//__IO uint8_t uart2State = UN_RX;
//__IO uint32_t UATT2_RX_ptr_Out = 0;
//__IO uint32_t UATT2_RX_ptr_Store = 0;
//__IO uint32_t UATT2_RXCounter = 0;

//void dealSetTime(void)
//{
//   if(uart2State == RX_STATE)
//   {
//     uart2RxTimeCounter ++;
//	 if(uart2RxTimeCounter == 0x01)
//	 {
//	   uart2State = UN_RX ;
//	  // actionFlag |= UART_DATA_FLAG; 
//	 }  
//   }
//   if(readFlag&READ_TEMP_FLAG)
//   {
//   	   readIdCounter ++;
//	   if(readIdCounter >= 1500)	 //等待两秒
//	   {
//	      readIdCounter = 0x00;
//		   readFlag |= READ_TEMP_FAIL;
//	   }
//   }
//
//}
//

__IO uint8_t rtcSaveFlag = RTC_SAVE_NONE;
#define REQUEST_TIME_OUT   10000

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//
//void eepromTest(void)
//{
//   uint32_t Address = 0;
//     /* Unlock the FLASH PECR register and Data EEPROM memory */
//  DATA_EEPROM_Unlock();
//
//    /* Clear all pending flags */      
//  FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
//                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
//  
//  /*  Data EEPROM Fast Word program of FAST_DATA_32 at addresses defined by 
//      DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR */  
//  Address = DATA_EEPROM_START_ADDR;
//    /* Check the correctness of written data */
//
//    if(*(__IO uint32_t*)Address != 0x0)
//    {
//      DataMemoryProgramStatus = FAILED;
//	  printf("不等于零\n\r");
//    }
//	else
//	{
//	FLASHStatus = DATA_EEPROM_EraseWord(Address);
//	    FLASHStatus = DATA_EEPROM_FastProgramWord(Address, FAST_DATA_32);
//
//    if(FLASHStatus == FLASH_COMPLETE)
//    {
//      	  printf("写入成功\n\r");
//
//	  //Address = Address + 4;
//    }
//    else
//    {
//      	  printf("写入失败\n\r");
//
//      FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
//                           | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR);
//    }
////    Address = Address + 4;
//	}
//  DATA_EEPROM_Lock();
//}

#define MAX_REQUEST_TRY     0x03

// uint8_t gprsSendFlag = UN_USE_THIS_COMMUNICTION;
 //uint8_t smsSendFlag  = UN_USE_THIS_COMMUNICTION;
// uint8_t canSendFlag  = UN_USE_THIS_COMMUNICTION;


// uint8_t smsFlag = SMS_UNSEND;
uint8_t deviceInit(void)
{
//    DATA_EEPROM_Unlock();
// 	uint8_t initi = 0;
//	uint8_t initj = 0;
//		 value_t  initb;
	uint32_t Address = 0;
#if(USART_USE_CTR>0)	
	value_t	dataTmp;
	if(devPar.usartKey == FUNCITONG_TURN_ON)
	{
		Address 	= DATA_EEPROM_MODBUSADDR_ADDR;
		dataTmp.i = (*(__IO uint32_t*)Address);
//		printf("\r\n modbus 0x%X",dataTmp.i);
		if(dataTmp.bytes.value4 == DEVICE_INIT)
		{		
			if((dataTmp.bytes.value1 >0)&&(dataTmp.bytes.value1<=MODBUS_ADDRESS_MAX))
				modbusAddress		= dataTmp.bytes.value1;
			  devSleepCtr			= dataTmp.bytes.value2;				
				usartTimeLimit	=	dataTmp.bytes.value3;
				if(usartTimeLimit>240){rs485WaitTime = (usartTimeLimit-240)*3600;}
				else if(usartTimeLimit>120){rs485WaitTime = (usartTimeLimit-120)*60;}
				else if(usartTimeLimit>0){rs485WaitTime = usartTimeLimit;}
				else {rs485WaitTime = RS485_WAIT_TIME_INIT;}				
		}
		else
		{
			modbusAddress	= MODBUS_ADDRESS_Init;
		}
	}
#endif	
#if(SK60_CTR>0) //210705
	sk60BaseDistance	=	sk60OffsetValueRead();
#endif
#if(KXYL_CTR>0)
 kxylParamRead();
#endif
	Address = DATA_EEPROM_START_ADDR;
	if(*(__IO uint32_t*)Address != FAST_DATA_32)
	{
		return DEVICE_UN_INIT;
	}
	else
	{
		Address = DATA_EEPROM_ID_ADDR ;
/*
设备ID
*/		  
//		  devId.i = (*(__IO uint32_t*)Address);
/*
设备信息
*/	
		Address = DATA_EEPROM_INF_ADDR  ;				  
		devInfor.i = (*(__IO uint32_t*)Address);

		devOutPutType = devInfor.bytes.value1;	//设备信息
		topSensorFlag = devInfor.bytes.value2;	//地表温度
		devSenNum 		= devInfor.bytes.value3;	//传感器数量
		if(devSenNum>DEV_MAX_NODE_NUM)devSenNum = DEV_MAX_NODE_NUM;
		printf("\r\n ****1*** devSenNum = %d *****\r\n",devSenNum);
		eachDevNum = devInfor.bytes.value4;			//每节传感器高低频率采集值控制
		  //串口设备相关信息读取
/*
设备设置
*/
		Address = DATA_EEPROM_SETINF_ADDR; 
		devSetValue.i = (*(__IO uint32_t*)Address);
		devSetStatus 	= devSetValue.bytes.value1;
		rs232BaudRate = devSetValue.bytes.value2;
		devRegCtr 		= devSetValue.bytes.value3;
		devGpsCtr 		=	devSetValue.bytes.value4;
/*
设备通信方式
*/
     //设备通信方式标志位信息装载
		Address = DEV_COMMUNICT_SET_ADDRESS ;
		devCommunictionValue.i = (*(__IO uint32_t*)Address);
// 		 if(devCommunictionValue.bytes.value1&GPRS_COMMUNICTION_TYPE)
// 		 {
// 		    gprsSendFlag = USE_THIS_COMMUNITCTION;
// 		 }
// 		 if(devCommunictionValue.bytes.value2&GSM_SMS_COMMUNICTION_TYPE)
// 		 {
// 		    smsSendFlag = USE_THIS_COMMUNITCTION;
// 		 }
// 		 if(devCommunictionValue.bytes.value3&CAN_BUS_COMMUNICTION_TYPE)
// 		 {
// 		    canSendFlag = USE_THIS_COMMUNITCTION;
// 		 }

// 		   Address = DATA_EEPROM_B20_ADDR ; 
// 		  //温度传感器相关信息读取
// 		  //地表温度处理

// 		  for(initi = 0x00;initi < devSenNum +1;initi++)
// 		  {

// 					  initb.i = (*(__IO uint32_t*)Address);
//             		  Address = Address +4 ;
// 		    result[initi].b20Id[0]=initb.bytes.value1;
// 		    result[initi].b20Id[1]=initb.bytes.value2 ;
// 		    result[initi].b20Id[2]=initb.bytes.value3;
// 		    result[initi].b20Id[3]=initb.bytes.value4;

// 					  initb.i = (*(__IO uint32_t*)Address);
//             		  Address = Address +4 ;
//   		    result[initi].b20Id[4]=initb.bytes.value1 ;
// 		    result[initi].b20Id[5]=initb.bytes.value2 ;
// 		    result[initi].b20Id[6]=initb.bytes.value3; 
// 		    result[initi].b20Id[7]=initb.bytes.value4 ;
// 			result[initi].b20Id[8]= 0x2c;
// 		  }

		return DEVICE_INIT;
	}
// 	DATA_EEPROM_Lock();
}


void b20IdRead(uint8_t *idBuf,uint8_t idOrder)
{
	uint32_t Address = DATA_EEPROM_B20_ADDR ; 
	value_t  initb; 

	Address = Address+ idOrder*8;
	initb.i = (*(__IO uint32_t*)Address);
	Address = Address +4 ;
	idBuf[0]=initb.bytes.value1;
	idBuf[1]=initb.bytes.value2 ;
	idBuf[2]=initb.bytes.value3;
	idBuf[3]=initb.bytes.value4;    
	initb.i = (*(__IO uint32_t*)Address);
	idBuf[4]=initb.bytes.value1 ;
	idBuf[5]=initb.bytes.value2 ;
	idBuf[6]=initb.bytes.value3; 
	idBuf[7]=initb.bytes.value4 ;
	idBuf[8]= 0x2c;    
}

void rtcTimeRs232Set(uint8_t *buf)
{
	DS1302_TIME time;
	if(bufTods1302(&time,buf)==RTCOK)
	{
		DIS_INT();
		//时间信息,设置时间	
		rtcTimeSet(buf); 
		EN_INT(); 
		SetDs1302TimeByGprs(buf); 
	}
}


void eepromInint(void)
{
  uint32_t address = DATA_EEPROM_ID_ADDR;

//	FLASH_Status flashStatus = FLASH_COMPLETE;	
	DIS_INT();
  DATA_EEPROM_Unlock();
	for(;address < DATA_EEPROM_B20_ADDR;address+=4)
	{
		FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

		(void)DATA_EEPROM_EraseWord(address);
	}
	DATA_EEPROM_Lock();
	EN_INT(); 
}
uint32_t eepromValueRead(uint32_t address)
{
	value_t temp;
	temp.i = (*(__IO uint32_t*)address);	
	return temp.i; 
}


void eepromValueWrite(uint32_t addressWr,uint32_t temp)
{
//     FLASH_Status flashStatus = FLASH_COMPLETE;
	DIS_INT();
	DATA_EEPROM_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
	(void)DATA_EEPROM_FastProgramWord(addressWr,temp); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 
}
#if(SK60_CTR>0)
	void sk60OffsetValueSave(__IO float value)
	{
		value_t tempV;
		tempV.f = value;
		eepromValueWrite(DATA_EEPROM_SK60OFFSET_ADDR,tempV.i);
	}
	float sk60OffsetValueRead(void)
	{
		value_t tempV;
		float value = 0.0;
		tempV.i = eepromValueRead(DATA_EEPROM_SK60OFFSET_ADDR);
		if((tempV.f>-60.0)&&(tempV.f<60.0))
		{
			value = tempV.f;
		}		
		return value;
	}
#endif
uint32_t readFireWareState(void)
{
	uint32_t fireWareState = 0x00;
	fireWareState = eepromValueRead(DATA_EEPROM_FIREWARE_CHG_ADDR);
	return fireWareState;
}

void fireWareStateWrite(uint32_t state)
{
	eepromValueWrite(DATA_EEPROM_FIREWARE_CHG_ADDR,state);
}

uint32_t readFireWareBootId(void)
{
	uint32_t fireWareState = 0x00;
	fireWareState = eepromValueRead(DATA_EEPROM_BOOT_ID_ADDR);
	return fireWareState;
}

void fireWareVerionWrite(uint32_t state)
{
	eepromValueWrite(DATA_EEPROM_SOFTWARE_VER_ADDR,state);
}
uint32_t readFireWareVerion(void)
{
	uint32_t fireWareState = 0x00;
	fireWareState = eepromValueRead(DATA_EEPROM_SOFTWARE_VER_ADDR);
	return fireWareState;
}

void fireWareBootIdWrite(uint32_t state)
{
	eepromValueWrite(DATA_EEPROM_BOOT_ID_ADDR,state);
}

void  devIdWrite(uint8_t *buf)
{
//	FLASH_Status flashStatus = FLASH_COMPLETE;
//	FLASH_Status FLASHStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	/* Clear all pending flags */      
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
	addressWr = DATA_EEPROM_START_ADDR;

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr, FAST_DATA_32); //错误判断

	DATA_EEPROM_Lock();
	EN_INT();   
}

void devModbusAddrSave(void)
{
#if(USART_USE_CTR>0)
	uint32_t address = 0x00;
	value_t	data;
	DIS_INT();
	DATA_EEPROM_Unlock();
	
	data.bytes.value1 = modbusAddress; 
	data.bytes.value2 = devSleepCtr;
	data.bytes.value3 = usartTimeLimit;
	data.bytes.value4 = DEVICE_INIT;

	address = DATA_EEPROM_MODBUSADDR_ADDR;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(address);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(address,data.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 
//printf("\r\n modbus save 0x%X",data.i);	
#endif
}

//
//void devPerWrite(uint8_t *buf)
//{
//               FLASH_Status flashStatus = FLASH_COMPLETE;
//			   uint32_t addressWr = 0x00;
//                DIS_INT();
//
//                DATA_EEPROM_Unlock();
//
//		 //数据采集周期
//		 devSenTime.bytes.value1 = buf[0]; 
//		 devSenTime.bytes.value2 = buf[1]; 
//		 devSenTime.bytes.value3 = buf[2];  
//		 devSenTime.bytes.value4 = buf[3];  
//          
//		   devSenTime.i = devSenTime.i*60;;
//	   	  addressWr = DATA_EEPROM_SENTIME_ADDR ;
//       FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
//                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
//
//		flashStatus = DATA_EEPROM_EraseWord(addressWr);
//       FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
//                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
//
//	    flashStatus = DATA_EEPROM_FastProgramWord(addressWr,devSenTime.i); //错误判断
//        DATA_EEPROM_Lock();
//		EN_INT(); 
//}

//void rs232DevPerWrite(void)
//{
//   uint8_t buf[4];
//   		 //数据采集周期
//	buf[0]	= devSenTime.bytes.value1 ; 
//	buf[1]	= devSenTime.bytes.value2 ; 
//	buf[2] = 0x00;  
//	buf[3] = 0x00 ; 
//	devPerWrite(buf);
//
//}
////void serialNumWrite(uint32_t addressWr,uint32_t num)
////{
////   uint32_t i = 0x00;
////	
////	 i = num;
////	 DIS_INT();
////     DATA_EEPROM_Unlock();

////		        FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
////                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

////		(void)DATA_EEPROM_EraseWord(addressWr);
////       FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
////                  | FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
////	   	(void)DATA_EEPROM_FastProgramWord(addressWr,i); //错误判断
////        DATA_EEPROM_Lock();
////		EN_INT();

////}
////uint32_t readNum(uint32_t which)
////{
////	 return (*(__IO uint32_t*)which);
////}
void devInfWrite(uint8_t *buf)
{
//设备id
//	FLASH_Status FLASHStatus = FLASH_COMPLETE;
//	FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//设备传感器节数
	devInfor.bytes.value3 = buf[0];
	devSenNum = devInfor.bytes.value3;
	printf("\r\n ****2*** devSenNum = %d *****\r\n",devSenNum);
	//设备节点状态
	//		 devInfor.bytes.value1 = UATT2_RX_Buffer[((status+8)&0xFF)]; 
	
	//设备类型
	devInfor.bytes.value2 = buf[1];
	topSensorFlag = devInfor.bytes.value2;

	//保留有两个字节的数据没有使用
	devInfor.bytes.value4 = buf[2];
	eachDevNum = devInfor.bytes.value4;
	//设备输出状态信息
	devInfor.bytes.value1 = buf[3];
	devOutPutType = devInfor.bytes.value1;	//设备信息	

	//每节传感器个数
	//		 devInfor.bytes.value4 = UATT2_RX_Buffer[((status+11)&0xFF)]; 
	//
	addressWr = DATA_EEPROM_INF_ADDR ;

	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devInfor.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 
}


void b20IdWrite(uint8_t *buf)
{
	uint8_t seti = 0;
	uint8_t setj = 0;
	value_t  setb;
	//	FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	uint8_t b20IdBuf[10];
	
	DIS_INT();
	DATA_EEPROM_Unlock();
	addressWr = DATA_EEPROM_B20_ADDR ;

	for(seti = 0;seti < devSenNum +1;seti++)
	{
		for(setj = 0;setj< 9;setj++)
		{
			b20IdBuf[setj] = buf[seti*9+setj];//buf[seti*9+8];	  //取数
		}
		setb.bytes.value1 = b20IdBuf[0];
		setb.bytes.value2 = b20IdBuf[1];
		setb.bytes.value3 = b20IdBuf[2];
		setb.bytes.value4 = b20IdBuf[3];

		(void)DATA_EEPROM_EraseWord(addressWr);
		FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

		(void)DATA_EEPROM_FastProgramWord(addressWr, setb.i); //错误判断
		FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	
		
		addressWr = addressWr+4;
		
		setb.bytes.value1 = b20IdBuf[4];
		setb.bytes.value2 = b20IdBuf[5];
		setb.bytes.value3 = b20IdBuf[6];
		setb.bytes.value4 = b20IdBuf[7];
		
		(void)DATA_EEPROM_EraseWord(addressWr);
		FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

		(void)DATA_EEPROM_FastProgramWord(addressWr, setb.i); //错误判断
		FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
			| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

//	SPI_FLASH_BufferWrite(&result[seti].b20Id[0],Address,9);
//	SPI_FLASH_BufferRead(&result[seti+4].b20Id[0],Address,9);
		addressWr = addressWr+4;
	}  
	EN_INT();
}

void devCommunictionSet(uint8_t *buf)
{
//	FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//数据采集周期
	devCommunictionValue.bytes.value1 = buf[0]; 
	devCommunictionValue.bytes.value2 = buf[1]; 
	devCommunictionValue.bytes.value3 = buf[2];  
	devCommunictionValue.bytes.value4 = buf[3];  
//  		 if(devCommunictionValue.bytes.value1&GPRS_COMMUNICTION_TYPE)
// 		 {
// 		    gprsSendFlag = USE_THIS_COMMUNITCTION;
// 		 }
// 		 if(devCommunictionValue.bytes.value1&GSM_SMS_COMMUNICTION_TYPE)
// 		 {
// 		    smsSendFlag = USE_THIS_COMMUNITCTION;
// 		 }
// 		 if(devCommunictionValue.bytes.value1&CAN_BUS_COMMUNICTION_TYPE)
// 		 {
// 		    canSendFlag = USE_THIS_COMMUNITCTION;
// 		 }         
	addressWr = DEV_COMMUNICT_SET_ADDRESS;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devCommunictionValue.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 	
}

void devStatusSet(uint8_t *buf)
{
//	FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//数据采集周期
	devSetValue.bytes.value1 = buf[0]; 
//		 devSetValue.bytes.value2 = buf[1]; 
//		 devSetValue.bytes.value3 = buf[2];  
//		 devSetValue.bytes.value4 = buf[3];  
	devSetStatus =   devSetValue.bytes.value1;
	addressWr = DATA_EEPROM_SETINF_ADDR;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devSetValue.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT();	
}
void devRs232BaudRateSet(uint8_t buf)
{
//               FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//数据采集周期
	devSetValue.bytes.value2 = buf; 
	rs232BaudRate =  devSetValue.bytes.value2;
//		 devSetValue.bytes.value2 = buf[1]; 
//		 devSetValue.bytes.value3 = buf[2];  
//		 devSetValue.bytes.value4 = buf[3];  

	addressWr = DATA_EEPROM_SETINF_ADDR;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devSetValue.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 	
}


void devRegAuthCodeSet(void)
{
 //              FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//AuthCode标识
	devSetValue.bytes.value3 =DEV_REG; 
	devRegCtr =  devSetValue.bytes.value3;
//		 devSetValue.bytes.value2 = buf[1]; 
//		 devSetValue.bytes.value3 = buf[2];  
//		 devSetValue.bytes.value4 = buf[3];  

	addressWr = DATA_EEPROM_SETINF_ADDR;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devSetValue.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 	
}

void devRegAuthCodeUnSet(void)
{
 //              FLASH_Status flashStatus = FLASH_COMPLETE;
	uint32_t addressWr = 0x00;
	DIS_INT();
	DATA_EEPROM_Unlock();

	//清除AuthCode标识
	devSetValue.bytes.value3 =DEV_UNREG; 
	devRegCtr =  devSetValue.bytes.value3;
//		 devSetValue.bytes.value2 = buf[1]; 
//		 devSetValue.bytes.value3 = buf[2];  
//		 devSetValue.bytes.value4 = buf[3];  

	addressWr = DATA_EEPROM_SETINF_ADDR;
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_EraseWord(addressWr);
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR
		| FLASH_FLAG_SIZERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_OPTVERRUSR);	

	(void)DATA_EEPROM_FastProgramWord(addressWr,devSetValue.i); //错误判断
	DATA_EEPROM_Lock();
	EN_INT(); 	
}


/**
  * @brief  SaveData when software reset.
  * @param  SaveData_SWReset.
  * @retval None 
  */  
#include <string.h>
#if(MQTT_USE_CTR>0)
void mqttParamSave(void)
{
	param_save_t saveParam;
	uint32_t address = 0x00;
	uint8_t i;
	saveParam.value.cigemFlat.bytes.value1 = SAVE_FLAG_DONE;
//	saveParam.value.cigemFlat.bytes.value2 = serverConfig.gsmRunMode;
	saveParam.value.cigemFlat.bytes.value3 = cigemrebootflag|(cigemlinknumSave<<4);
	saveParam.value.cigemFlat.bytes.value4 = cigemworkmode;
	
	memcpy(saveParam.value.cigemmsgid,cigemmsgid,CIGEM_MSGID_LEN*MQTT_MAX_MULTI_CONNECTION_SIZE);
	
	saveParam.value.cigemcmdFlat.bytes.value1 = cigemcmdflag[0];		
	saveParam.value.cigemcmdFlat.bytes.value2 = cigemcmdflag[1];
	saveParam.value.cigemcmdFlat.bytes.value3 = cigemcmdflag[2];
	saveParam.value.cigemcmdFlat.bytes.value4 = cigemcmdflag[3];
		
	address = DATA_EEPROM_SAVE_MQTT_ADDR;
//	printf("\r\n save %x ",address);
	for(i=DATA_SAVE_SENSOR_ADDR_LEN;i<(DATA_SAVE_ADDR_LEN-2);i++)
	{		
		eepromValueWrite(address,saveParam.buf[i]);
		address += 4;
//		printf("%d_%x ",i,saveParam.buf[i]);
	}
}
void mqttParamRead(void)
{	
	uint32_t address = 0x00;
	param_save_t saveParam;
	uint8_t i=0;
	address	=	DATA_EEPROM_SAVE_MQTT_ADDR;
	saveParam.value.cigemFlat.i = eepromValueRead(address);
	printf(" read %x ",saveParam.value.cigemFlat.i);
	if(saveParam.value.cigemFlat.bytes.value1 == SAVE_FLAG_DONE)
	{
		for(i=DATA_SAVE_SENSOR_ADDR_LEN+1;i<(DATA_SAVE_ADDR_LEN-2);i++)
		{				
			address += 4;			
			saveParam.buf[i]	=	eepromValueRead(address);		
//			printf("%d_%x ",i,saveParam.buf[i]);			
		}
//		serverConfig.gsmRunMode	=	saveParam.value.cigemFlat.bytes.value2;
		cigemrebootflag					=	saveParam.value.cigemFlat.bytes.value3&0x0F;
		cigemlinknumSave				=	saveParam.value.cigemFlat.bytes.value3>>4;
		cigemworkmode						= saveParam.value.cigemFlat.bytes.value4;
		
		memcpy(cigemmsgid,saveParam.value.cigemmsgid,CIGEM_MSGID_LEN*MQTT_MAX_MULTI_CONNECTION_SIZE);
		cigemcmdflag[0]	=	saveParam.value.cigemcmdFlat.bytes.value1;
		cigemcmdflag[1]	=	saveParam.value.cigemcmdFlat.bytes.value2;
		cigemcmdflag[2]	=	saveParam.value.cigemcmdFlat.bytes.value3;
		cigemcmdflag[3]	=	saveParam.value.cigemcmdFlat.bytes.value4;			
	}
}
void mqttParamClear(void)
{	
	uint32_t	address;
	uint8_t 	tmpi;
	
	address	=	DATA_EEPROM_SAVE_MQTT_ADDR;
	for(tmpi=0;tmpi<(DATA_SAVE_MQTT_ADDR_LEN);tmpi++)
	{
		eepromValueWrite(address,0);
		address += 4;
	}
}
#endif
extern uint8_t enterLowPowerMode(void);
 void SaveData_SWReset(void)
 {
#if(MQTT_USE_CTR>0)	 
	value_t	data;	
	uint32_t address = 0x00;
if(rstStatus>0)
{
	flashStoreTask(serverConfig);
	servStoreFlagInit(&serverConfig);
	if(rstStatus != RST_STATUS_KEY)
	{
	#if(GPRS_USE_CTR>0)
		if(gprsPowerStatus == STATUS_DONE){enterLowPowerMode();gprsPowerOff();}//delay_ms(1000);
	#endif
	#if(USART_USE_CTR>0)
		if(rs485Power	== rs485_DONE)rs485_SLEEP();
	#endif
	}
	data.bytes.value1 = SAVE_FLAG_DONE; 		
	data.bytes.value2 = rtcTime.date;
	data.bytes.value3 = rtcTime.hour;
	data.bytes.value4 = rstStatus;
	address = DATA_EEPROM_SAVE_BEGIN_ADDR;		
	eepromValueWrite(address,data.i);	
}
	mqttParamSave();
#endif
	
	
}											  
 /**
  * @brief  ReadData when software reset.
  * @param  ReadData_SWReset.
  * @retval None 
  */ 

void ReadData_SWReset(void)
{
#if(MQTT_USE_CTR>0)
	value_t		data;	
	uint32_t	address;
	uint8_t		datetmp,hourtmp;

	address	=	DATA_EEPROM_SAVE_BEGIN_ADDR;
	data.i	=	eepromValueRead(address);	
	if(data.bytes.value1 == SAVE_FLAG_DONE)
	{
		datetmp	=	data.bytes.value2;
		hourtmp	= data.bytes.value3;
		if((datetmp>0)&&(datetmp<32)&&(hourtmp<24))
		{			
			rstStatus	 = data.bytes.value4;
			if(rstStatus>0)
			{				
				printf("\r\n rstStatus:%d,",rstStatus);					
			
			}			
		}		
	}

	mqttParamRead();

	printf(" data reload end\r\n");
#endif
}
void ClearData_SWReset(void)
{	
#if(MQTT_USE_CTR>0)
	uint32_t	address;
	uint8_t 	tmpi;
	
	address	=	DATA_EEPROM_SAVE_BEGIN_ADDR;
	for(tmpi=0;tmpi<DATA_SAVE_SENSOR_ADDR_LEN+1;tmpi++)
	{
		eepromValueWrite(address,0);
		address += 4;
	}
#endif
}
void setTimeTask(DS1302_TIME setTime)
{
//#if(MQTT_USE_CTR>0)
	uint8_t buf[10];
	setTime.day = 0x01;

	showDs1302Time(setTime);

	if(rtcCheck(setTime)==RTCOK)
	{
		SetDs1302Time(setTime);
		ds1302tobuf(setTime,buf);//readDs1302TimeForRtc(buf);
		DIS_INT();	
		rtcTimeSet(buf); 
		EN_INT();
	}
//#endif
}
#if(KXYL_CTR>0)	//221128
//#include "sk60.h"
void kxylParamRead(void)
{
	uint8_t tmpi=0;
	value_t tmpA,tmpB;
	uint32_t address = DATA_EEPROM_KXYL_ADDR;
	for(tmpi=0;tmpi<KXYL_SENSOR_NUM;tmpi++)
	{
		tmpA.i = eepromValueRead(address);
		address += 4;
		tmpB.i = eepromValueRead(address);
		address += 4;
		if((tmpA.f>0.001)||(tmpA.f<(-0.001)))//if(tmpA.f!=0.0)
		{
			kxylParam[tmpi][0] = tmpA.f;
			kxylParam[tmpi][1] = tmpB.f;
		}
		else
		{
			kxylParam[tmpi][0] = KXYL_COF_A;
			kxylParam[tmpi][1] = KXYL_COF_B;
		}		
	}
}
void kxylParamSave(uint8_t index,float paramA,float paramB)
{
	value_t data;
	uint32_t address = DATA_EEPROM_KXYL_ADDR;
	if(index<8)
	{		
		address += (index*8);
		data.f = paramA;
		eepromValueWrite(address,data.i);
		address += 4;
		data.f = paramB;
		eepromValueWrite(address,data.i);
	}
}
#endif

