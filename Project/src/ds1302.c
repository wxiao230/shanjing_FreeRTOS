#include "ds1302.h"
#include "delay.h"
#include "deviceset.h"
#include<stdio.h>

/*******************************************************************************************************
*    DS1302 Ê±ÖÓÃüÁî
*
*
*******************************************************************************************************/
#define  READ_SEC       0x81     //¶ÁÃëÃüÁî
#define  WRITE_SEC      0x80    //Ð´ÃëÃüÁî
#define  READ_MIN       0x83     //¶Á·ÖÃüÁî
#define  WRITE_MIN      0x82    //Ð´·ÖÃüÁî
#define  READ_HOUR      0x85    //¶ÁÐ¡Ê±ÃüÁî
#define  WRITE_HOUR     0x84   //Ð´Ð¡Ê±ÃüÁî
#define  READ_DATE      0x87    //¶ÁÈÕÆÚÃüÁî
#define  WRITE_DATE     0x86    //Ð´ÈÕÆÚÃüÁî
#define  READ_MON       0x89      //¶ÁÔÂ·ÝÃüÁî
#define  WRITE_MON      0x88     //Ð´ÔÂ·ÝÃüÁî
#define  READ_DAY       0x8B      //¶ÁÐÇÆÚÃüÁî
#define  WRITE_DAY      0x8C     //Ð´ÐÇÆÚÃüÁî
#define  READ_YEAR      0x8D     //¶ÁÄê·ÝÃüÁî
#define  WRITE_YEAR     0x8C    //Ð´Äê·ÝÃüÁî


#define  READ_WP        0x8F     //¶ÁÐ´±£»¤ÃüÁî
#define  WRITE_WP       0x8E    //ÉèÖÃÐ´±£»¤ÃüÁî

#define  READ_CHARGE    0x91   //¶Á³äµç¼Ä´æÆ÷ÃüÁî
#define  WRITE_CHARGE    0x90  //Ð´³äµç¼Ä´æÆ÷ÃüÁî

#define  READ_CLOCK_BURST    0xBF    // Ê±ÖÓÍ»·¢¶ÁÃüÁî
#define  WRITE_CLOCK_BURST    0xBE   //Ê±ÖÓÍ»·¢Ð´ÃüÁî

#define  READ_RAM_BURST    0xFF      //ARMÍ»·¢¶ÁÃüÁî
#define  WRITE_RAM_BURST   0xFE      //ARMÍ»·¢Ð´ÃüÁî




#define DS_RTC_PORT                                     GPIOC	   //Ñ¡ÔñÍâÉèGPIOA

#define RCC_RTC_GPIO_DS                                 RCC_AHBPeriph_GPIOC

/*
ÎÂ¶È´«¸ÐÆ÷1Òý½Å¶¨Òå
*/
#define DS_RTC_IO       GPIO_Pin_1 	  //Ñ¡Ôñ¹Ü½ÅPa1
#define DS_RTC_CLK      GPIO_Pin_0 	  //Ñ¡Ôñ¹Ü½ÅPa1
#define DS_RTC_EN       GPIO_Pin_2 	  //Ñ¡Ôñ¹Ü½ÅPa1

#define DS1302DATA_LOW()    DS_RTC_PORT->BSRRH = DS_RTC_IO  //À­µÍDQ	
#define DS1302DATA_HIGHT()      DS_RTC_PORT->BSRRL = DS_RTC_IO  				   
#define GetRtcIO()     ((DS_RTC_PORT->IDR&(DS_RTC_IO))?1:0) //¶ÁÈ¡DQµÄµçÆ½((DS_PORT->IDR&(DS_DQIO))?1:0)

#define DS1302SCK_LOW()        DS_RTC_PORT->BSRRH = DS_RTC_CLK  //À­µÍDQ	
#define DS1302SCK_HIGHT()      DS_RTC_PORT->BSRRL = DS_RTC_CLK

#define DS1302_EN()       DS_RTC_PORT->BSRRL = DS_RTC_EN	
#define DS1302_UNEN()      DS_RTC_PORT->BSRRH = DS_RTC_EN  //À­µÍDQ

DS1302_TIME  rtcTime;
DS1302_TIME  rtcTimeSample;
__IO uint32_t	curSecondTime 	= 0;

static void delay_rtc_Us(__IO uint32_t delay)
{
    static __IO uint32_t i= 0x00;
    static __IO uint16_t j = 0x00;
	for(i= 0;i< delay;i++)
	{
        j = 32;
	   while(j)
       {
          j--;
       }
	}
}

void reSetRtc(void)
{
   DS1302SCK_LOW();
   delay_rtc_Us(4);
   DS1302_UNEN();
   delay_rtc_Us(4);

   DS1302_EN(); 
}


void rtcIoOut(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   RCC_AHBPeriphClockCmd(RCC_RTC_GPIO_DS, ENABLE); 
 
  GPIO_InitStructure.GPIO_Pin = DS_RTC_IO|DS_RTC_CLK|DS_RTC_EN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //¿ªÂ©Êä³ö
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
  GPIO_Init(DS_RTC_PORT,&GPIO_InitStructure);
     DS1302_UNEN();
}

void rtcIoLowPower(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   RCC_AHBPeriphClockCmd(RCC_RTC_GPIO_DS, ENABLE); 
 	DS1302_UNEN();
  GPIO_InitStructure.GPIO_Pin = DS_RTC_IO|DS_RTC_CLK|DS_RTC_EN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//¿ªÂ©Êä³ö
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     
  GPIO_Init(DS_RTC_PORT,&GPIO_InitStructure);
     
}
void rtcIoIn(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   RCC_AHBPeriphClockCmd(RCC_RTC_GPIO_DS, ENABLE); 
 
  GPIO_InitStructure.GPIO_Pin = DS_RTC_IO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //¿ªÂ©Êä³ö
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    
  GPIO_Init(DS_RTC_PORT,&GPIO_InitStructure);   
}




/*******************************************************************************************************
*write one byte to ds1302
*
*
*
********************************************************************************************************/

static void writeOneByteDs1302(uint8_t data)
{

  uint8_t i = 0;

  
  for(i = 0;i < 8; i++)
  {

     DS1302DATA_LOW();
     if(data&0x01)
     {
        DS1302DATA_HIGHT();
     
     }
	  DS1302SCK_LOW();
		//delay_1us(2);
     delay_rtc_Us(2);
     DS1302SCK_HIGHT();
     delay_rtc_Us(2);
     //	delay_1us(2);
     data >>=1;
     //delay_1us(2);
     delay_rtc_Us(2);
  }
}

/*******************************************************************************************************
*write data to ds1302
*
*cmd : address
*data: data
*
********************************************************************************************************/
static void writeSetDs1302(uint8_t cmd,uint8_t data)
{
  
	 rtcIoOut();
     DS1302_UNEN();
     DS1302SCK_LOW();
     DS1302_EN();
//     	delay_1us(150);
     //  delay_1us(20);
    delay_rtc_Us(20);
     writeOneByteDs1302(cmd);
//	     	delay_1us(150);
      // delay_1us(20);
     delay_rtc_Us(20);
     writeOneByteDs1302(data);
     DS1302SCK_HIGHT();
     DS1302_UNEN();    


}

static uint8_t readOneByteFromDs1302(void)
{
  

  
     uint8_t data = 0;
     uint8_t i = 0;
	 uint8_t temp =0;
         rtcIoIn();
         for(i = 0;i< 8;i++)
		 {
          DS1302SCK_HIGHT();
         //	delay_1us(2);
             delay_rtc_Us(2);
         DS1302SCK_LOW();
         	//delay_1us(2);
             delay_rtc_Us(2);
		temp = 	GetRtcIO();
		temp<<=7;
		data >>=1;
		data |= temp; 
		}   
     return data;
}

uint8_t readDataFromDs1302(uint8_t cmd)
{

    uint8_t data;
	 rtcIoOut();
//    DS1302SCK_OUT();
//    DS1302RST_OUT();
    DS1302_UNEN();
 //   	delay_1us(50);
        //delay_1us(20);
    delay_rtc_Us(20);
    DS1302SCK_LOW();
 //   	delay_1us(50);
       // delay_1us(20);
    delay_rtc_Us(20);
    DS1302_EN();
 //   	delay_1us(50);
     //   delay_1us(20);
    delay_rtc_Us(20);
    writeOneByteDs1302(cmd);
    data = readOneByteFromDs1302();
    DS1302SCK_HIGHT();
    //    delay_1us(20);
    delay_rtc_Us(20);
  //  	delay_1us(50);
    DS1302_UNEN();
    
    return data; 

}
uint8_t readTimeFromDs1302(uint8_t cmd)
{
   uint8_t data;
//   uint8_t temp = 0;
   data = readDataFromDs1302(cmd);
   switch(cmd)
   {
      case READ_SEC :
       {
          data &= 0x7f;//bit7ÎªDS1302¹¤×÷×´Ì¬Ö¸Ê¾Î»
          data = (data >>4)*10 + (data&0x0F);

	      break;
       }
      case READ_MIN :
      case READ_HOUR :
      case READ_DATE :
      case READ_MON : 
      case READ_DAY  :
      case READ_YEAR :
       {
          
           data = (data >>4)*10 + (data&0x0F);
          break;
       } 
     default :
       {
          break;
       }      
   }

   return data;
}


void ds1302Time2Bcd(DS1302_TIME *readTime)
{
   readTime->sec = ((readTime->sec/10)<<4|(readTime->sec%10));
   readTime->min = ((readTime->min/10)<<4|(readTime->min%10));   
   readTime->hour = ((readTime->hour/10)<<4|(readTime->hour%10));
   readTime->date = ((readTime->date/10)<<4|(readTime->date%10));
   readTime->month = ((readTime->month/10)<<4|(readTime->month%10));   
   readTime->year = ((readTime->year/10)<<4|(readTime->year%10));   
}
uint8_t readTimeFromDs1302Bcd(uint8_t cmd)
{
   uint8_t data;
//   uint8_t temp = 0;
   data = readDataFromDs1302(cmd);
   if(READ_SEC== cmd)
   {
     data &= 0x7f;//bit7ÎªDS1302¹¤×÷×´Ì¬Ö¸Ê¾Î»
   }
   return data;
}

void readDs1302TimeBcd(DS1302_TIME *readTime)
{
//   readTime->sec =  readTimeFromDs1302Bcd(READ_SEC);
//   readTime->min =  readTimeFromDs1302Bcd(READ_MIN);
//   readTime->hour = readTimeFromDs1302Bcd(READ_HOUR);
//   readTime->date = readTimeFromDs1302Bcd(READ_DATE);
//   readTime->month = readTimeFromDs1302Bcd(READ_MON);
//   readTime->year =  readTimeFromDs1302Bcd(READ_YEAR);
   uint8_t readTimeBuf[10];
    
   rtcIoOut();
   reSetRtc(); 
   writeOneByteDs1302(READ_CLOCK_BURST);
   readTimeBuf[0] = readOneByteFromDs1302();
   readTimeBuf[1] = readOneByteFromDs1302();
   readTimeBuf[2] = readOneByteFromDs1302();
   readTimeBuf[3] = readOneByteFromDs1302();       
   readTimeBuf[4] = readOneByteFromDs1302();
   readTimeBuf[5] = readOneByteFromDs1302();
   readTimeBuf[6] = readOneByteFromDs1302();
   readTimeBuf[7] = readOneByteFromDs1302();
   reSetRtc();
   readTimeBuf[0] &= 0x7f;//bit7ÎªDS1302¹¤×÷×´Ì¬Ö¸Ê¾Î»
   readTime->sec = readTimeBuf[0] ;
   readTime->min = readTimeBuf[1] ;
   readTime->hour = readTimeBuf[2] ;
   readTime->date = readTimeBuf[3] ;
   readTime->month =readTimeBuf[4] ;
   readTime->year = readTimeBuf[6] ;   
}

void ds1302BrustReadClock(DS1302_TIME *readTime)
{
   uint8_t readTimeBuf[10];
    
   rtcIoOut();
   reSetRtc(); 
   writeOneByteDs1302(READ_CLOCK_BURST);
   readTimeBuf[0] = readOneByteFromDs1302();
   readTimeBuf[1] = readOneByteFromDs1302();
   readTimeBuf[2] = readOneByteFromDs1302();
   readTimeBuf[3] = readOneByteFromDs1302();       
   readTimeBuf[4] = readOneByteFromDs1302();
   readTimeBuf[5] = readOneByteFromDs1302();
   readTimeBuf[6] = readOneByteFromDs1302();
   readTimeBuf[7] = readOneByteFromDs1302();
   reSetRtc();
   readTimeBuf[0] &= 0x7f;//bit7ÎªDS1302¹¤×÷×´Ì¬Ö¸Ê¾Î»
   readTime->sec = (readTimeBuf[0] >>4)*10 + (readTimeBuf[0]&0x0F);
   readTime->min = (readTimeBuf[1] >>4)*10 + (readTimeBuf[1]&0x0F);
   readTime->hour = (readTimeBuf[2] >>4)*10 + (readTimeBuf[2]&0x0F);
   readTime->date = (readTimeBuf[3] >>4)*10 + (readTimeBuf[3]&0x0F);   
   readTime->month = (readTimeBuf[4] >>4)*10 + (readTimeBuf[4]&0x0F);
   readTime->year = (readTimeBuf[6] >>4)*10 + (readTimeBuf[6]&0x0F);     
 // printf("\n\r  The ds1302  date is :   %0.2d:%0.2d:%0.2d \n\r", readTime->year,readTime->month,readTime->date);
 // printf("\n\r  The ds1302 time is :   %0.2d:%0.2d:%0.2d \n\r", readTime->hour,readTime->min,readTime->sec);
}


void readDs1302Time(DS1302_TIME *readTime)
{
//   readTime->sec =  readTimeFromDs1302(READ_SEC);
//   readTime->min =  readTimeFromDs1302(READ_MIN);
//   readTime->hour = readTimeFromDs1302(READ_HOUR);
//   readTime->date = readTimeFromDs1302(READ_DATE);
//   readTime->month = readTimeFromDs1302(READ_MON);
//   readTime->year =  readTimeFromDs1302(READ_YEAR);
   uint8_t readTimeBuf[10];
	uint8_t tryi=0;
DIS_INT(); 
for(tryi=0;tryi<2;tryi++)
{	
   rtcIoOut();
   reSetRtc(); 
   writeOneByteDs1302(READ_CLOCK_BURST);
   readTimeBuf[0] = readOneByteFromDs1302();
   readTimeBuf[1] = readOneByteFromDs1302();
   readTimeBuf[2] = readOneByteFromDs1302();
   readTimeBuf[3] = readOneByteFromDs1302();       
   readTimeBuf[4] = readOneByteFromDs1302();
   readTimeBuf[5] = readOneByteFromDs1302();
   readTimeBuf[6] = readOneByteFromDs1302();
   readTimeBuf[7] = readOneByteFromDs1302();
   reSetRtc();
		readTimeBuf[0] &= 0x7f;//bit7ÎªDS1302¹¤×÷×´Ì¬Ö¸Ê¾Î»
   readTime->sec = (readTimeBuf[0] >>4)*10 + (readTimeBuf[0]&0x0F);
   readTime->min = (readTimeBuf[1] >>4)*10 + (readTimeBuf[1]&0x0F);
   readTime->hour = (readTimeBuf[2] >>4)*10 + (readTimeBuf[2]&0x0F);
   readTime->date = (readTimeBuf[3] >>4)*10 + (readTimeBuf[3]&0x0F);   
   readTime->month = (readTimeBuf[4] >>4)*10 + (readTimeBuf[4]&0x0F);
   readTime->year = (readTimeBuf[6] >>4)*10 + (readTimeBuf[6]&0x0F);      
	 if(rtcCheck(*readTime)==RTCOK)break;
}
EN_INT();
}

//uint8_t rtcTimeBuf[7];

void readDs1302TimeForRtc(uint8_t *buf)
{
DIS_INT();
	buf[0] =	readTimeFromDs1302(READ_SEC);
	buf[1] =	readTimeFromDs1302(READ_MIN);
	buf[2] =	readTimeFromDs1302(READ_HOUR);
	buf[3] = 	readTimeFromDs1302(READ_DATE);
	buf[4] =	readDataFromDs1302(READ_DAY);
	buf[5] =  readDataFromDs1302(READ_MON);
	buf[6] =  readTimeFromDs1302(READ_YEAR);
EN_INT();
//    uint8_t readTimeBuf[10];
//     
//    rtcIoOut();
//    reSetRtc(); 
//    writeOneByteDs1302(READ_CLOCK_BURST);
//    readTimeBuf[0] = readOneByteFromDs1302();
//    readTimeBuf[1] = readOneByteFromDs1302();
//    readTimeBuf[2] = readOneByteFromDs1302();
//    readTimeBuf[3] = readOneByteFromDs1302();       
//    readTimeBuf[4] = readOneByteFromDs1302();
//    readTimeBuf[5] = readOneByteFromDs1302();
//    readTimeBuf[6] = readOneByteFromDs1302();
//    readTimeBuf[7] = readOneByteFromDs1302();
//    reSetRtc();
  
}


void showDs1302Time(DS1302_TIME showTime)
{
  printf("\n\r The ds1302 rtc is : 20%0.2d-%0.2d-%0.2d ",showTime.year,showTime.month,showTime.date);
  printf(" %0.2d:%0.2d:%0.2d \n\r", showTime.hour,showTime.min,showTime.sec);

}
/*******************************************************************************************************
*ÉèÖÃDS1302µÄÊ±¼ä
*Èë¿Ú²ÎÊý£ºÒªÉèÖÃµÄÊ±¼ä
*·µ»Ø²ÎÊý£ºÎÞ
*
*******************************************************************************************************/
void SetDs1302Time(DS1302_TIME setTime)
{
DIS_INT();	
	if(setTime.sec == 59)setTime.sec = 58;
	rtcIoOut();
	writeSetDs1302(WRITE_WP,0x00);//½â³ýÐ´±£»¤
	writeSetDs1302(WRITE_SEC,((setTime.sec/10)<<4|(setTime.sec%10)));
	writeSetDs1302(WRITE_MIN,((setTime.min/10)<<4|(setTime.min%10)));
	setTime.hour= ((setTime.hour/10<<4)|(setTime.hour%10));
	setTime.hour &=0x7F;//ÉèÖÃ24Ð¡Ê±Ê±¼äÖÆ


	writeSetDs1302(WRITE_HOUR,setTime.hour); 
	writeSetDs1302(WRITE_DATE,((setTime.date/10)<<4|(setTime.date%10)));
	writeSetDs1302(WRITE_MON,((setTime.month/10)<<4|(setTime.month%10))); 
	writeSetDs1302(WRITE_DAY,((setTime.day/10)<<4|(setTime.day%10)));
	writeSetDs1302(WRITE_YEAR,((setTime.year/10)<<4|(setTime.year%10)));
	writeSetDs1302(WRITE_CHARGE,0x00); //½ûÖ¹³äµç
	writeSetDs1302(WRITE_WP,0x80);//ÉèÖÃÐ´±£»¤
EN_INT();
}


void SetDs1302TimeBcd(DS1302_TIME setTime)
{
      rtcIoOut();
      writeSetDs1302(WRITE_WP,0x00);//½â³ýÐ´±£»¤
      writeSetDs1302(WRITE_SEC,setTime.sec);
      writeSetDs1302(WRITE_MIN,setTime.min);
      //setTime.hour= ((setTime.hour/10<<4)|(setTime.hour%10));
      setTime.hour &=0x7F;//ÉèÖÃ24Ð¡Ê±Ê±¼äÖÆ
      
      
      writeSetDs1302(WRITE_HOUR,setTime.hour); 
      writeSetDs1302(WRITE_DATE,setTime.date);
      writeSetDs1302(WRITE_MON,setTime.date); 
      writeSetDs1302(WRITE_DAY,setTime.day);
      writeSetDs1302(WRITE_YEAR,setTime.year);
      writeSetDs1302(WRITE_CHARGE,0x00); //½ûÖ¹³äµç
      writeSetDs1302(WRITE_WP,0x80);//ÉèÖÃÐ´±£»¤

}
/*******************************************************************************************************/
void SetDs1302TimeByGprs(uint8_t *buf)
{
	DIS_INT();
	rtcIoOut();
	writeSetDs1302(WRITE_WP,0x00);//½â³ýÐ´±£»¤
	writeSetDs1302(WRITE_WP,0x00);//½â³ýÐ´±£»     
	writeSetDs1302(WRITE_SEC,((buf[0]/10)<<4|(buf[0]%10)));
	writeSetDs1302(WRITE_MIN,((buf[1]/10)<<4|(buf[1]%10)));
	buf[2]= ((buf[2]/10<<4)|(buf[2]%10));

	buf[2] &=0x7F;//ÉèÖÃ24Ð¡Ê±Ê±¼äÖÆ

	writeSetDs1302(WRITE_HOUR,buf[2]); 
	writeSetDs1302(WRITE_DATE,((buf[3]/10)<<4|(buf[3]%10)));
	writeSetDs1302(WRITE_MON,((buf[5]/10)<<4|(buf[5]%10)));
	writeSetDs1302(WRITE_DAY,((buf[4]/10)<<4|(buf[4]%10)));
	writeSetDs1302(WRITE_YEAR,((buf[6]/10)<<4|(buf[6]%10)));
	writeSetDs1302(WRITE_CHARGE,0x00); //½ûÖ¹³äµç
	writeSetDs1302(WRITE_WP,0x80);//ÉèÖÃÐ´±£»¤
	EN_INT();
}
/******************************************************************************************************
*³õÊ¼»¯DS1302µÄÊ±ÖÓÎª2011Äê11ÔÂ17ÈÕ12Ê±0·Ö0Ãë
*Èë¿Ú²ÎÊý£ºÎÞ
*·µ»ØÖµ£º  ÎÞ
*
*******************************************************************************************************/

void initDs1302(uint8_t initctr)
{
	DS1302_TIME initTime;

	uint8_t flag =0;

//	initTime.time = 0x00000000;
//	initTime.baseYear = 2000;
	initTime.sec 		= 0x00;
	initTime.min 		= 21;     
	initTime.hour		= 19;     
	initTime.date		= 18;
	initTime.day 		= 5;
	initTime.month 	= 12;     
	initTime.year		= 20;
	
	rtcIoOut();
		    //       delay_1us(20);
	delay_rtc_Us(20);
	flag = readDataFromDs1302(READ_SEC);

	if((flag&0x80)||(initctr==1))
	{
		SetDs1302Time(initTime);
	}
	
}
void ds1302tobuf(DS1302_TIME time,uint8_t *buf)
{
	buf[0] =	time.sec;		//readTimeFromDs1302(READ_SEC);
	buf[1] =	time.min;		//readTimeFromDs1302(READ_MIN);
	buf[2] =	time.hour;	//readTimeFromDs1302(READ_HOUR);
	buf[3] = 	time.date;	//readTimeFromDs1302(READ_DATE);
	buf[4] =	time.day;		//readDataFromDs1302(READ_DAY);
	buf[5] =  time.month;	//readDataFromDs1302(READ_MON);
	buf[6] =  time.year;	//readTimeFromDs1302(READ_YEAR);
}
uint8_t bufTods1302(DS1302_TIME *timeout,uint8_t *buf)
{
	DS1302_TIME time;
	time.sec 		= buf[0];		//readTimeFromDs1302(READ_SEC);
	time.min 		= buf[1];		//readTimeFromDs1302(READ_MIN);
	time.hour		= buf[2];	//readTimeFromDs1302(READ_HOUR);
	time.date		= buf[3];	//readTimeFromDs1302(READ_DATE);
	time.day 		= buf[4];		//readDataFromDs1302(READ_DAY);
	time.month 	= buf[5];	//readDataFromDs1302(READ_MON);
	time.year		= buf[6];	//readTimeFromDs1302(READ_YEAR);
	if(rtcCheck(time)==RTCOK)
	{
		*timeout = time;
		return RTCOK;
	}
	return RTCERR;
}
uint8_t rtcCheck(__IO DS1302_TIME Time)
{
	if(Time.year>99)return RTCERR;
	if((Time.month>12)||(Time.month<1))return RTCERR;
	if((Time.date>31)||(Time.date<1))return RTCERR;
	if(Time.hour>23)return RTCERR;	
	if(Time.min>59)return RTCERR;
	if(Time.sec>59)return RTCERR;
	return RTCOK;
}
uint8_t rtcCompare(__IO DS1302_TIME inTime,DS1302_TIME baseTime)
{
	if(rtcCheck(inTime) == RTCERR)return RTCERR;
	if(inTime.year<baseTime.year)return RTCERR;
	if(inTime.year==baseTime.year)
	{
		if(inTime.month<baseTime.month)return RTCERR;
		if(inTime.month==baseTime.month)
		{
			if(inTime.date<baseTime.date)return RTCERR;
			if(inTime.date==baseTime.date)
			{
				if(inTime.hour<baseTime.hour)return RTCERR;
				if(inTime.hour==baseTime.hour)
				{
					if(inTime.min<baseTime.min)return RTCERR;					
					if(inTime.min==baseTime.min)
					{
						if(inTime.sec<baseTime.sec)return RTCERR;
					}
				}
			}
		}
	}
	return RTCOK;
}
#define LEAP_YEAR_ADJ 1 //ÈòÄêÐÞÕý
uint32_t queryDayToHour(uint8_t year,uint8_t mon,uint8_t date)
{
   uint32_t day = 0x00;
	 uint32_t curyear = 0;
	 uint32_t hour  = 0; 
#if(LEAP_YEAR_ADJ>0)	//211025
	uint8_t yearAdd = 0;
	uint8_t leapyear = 0;
#endif
	 curyear = year+2000;
	 switch(mon)
	 {
		 case 1 :
		 {
		    day = date-1;
			  break;
		 }
		 case 2 :
		 {
		    day = 30+date;//31+date-1
		    break;
		 }
		 case 3 :
		 {
		    day = 30+date;//31+data-1	30=(mon-2)*30 
        break;		    
		 }
		 case 4 :
		 {
		    day = 61+date;//1yue+3yue-1 61=(mon-2)*30+2-1			
        break;		    
		 }	
		 case 5 :
		 {
		    day = 91 +date;//1+3+4	91=(mon-2)*30+2-1			
        break;		    
		 }	
		 case 6 :
		 {
		    day = 122 +date;//1+3+4+5	122=(mon-2)*30+3-1		
        break;		    
		 }
		 case 7 :
		 {
		    day = 152 +date;//1+3+4+5+6	152=(mon-2)*30+3-1		
        break;		    
		 }	
		 case 8 :
		 {
		    day = 183 +date;//1+3+4+5+6+7		183=(mon-2)*30+4-1	
        break;		    
		 }	
		 case 9 :
		 {
		    day = 214 +date;//1+3+4+5+6+7+8 214=(mon-2)*30+5-1
        break;		    
		 }	
		 case 10 :
		 {
		    day = 244 +date;//1+3+4+5+6+7+8+9		244=(mon-2)*30+5-1		
        break;		    
		 }		
		 case 11 :
		 {
		    day = 275 +date ;//1+3+4+5+6+7+8+9+10 275=(mon-2)*30+6-1			
        break;		    
		 }
		 case 12 :
		 {
		    day = 305 +date;//1+3+4+5+6+7+8+9+10+11	305=(mon-2)*30+6-1			
        break;		    
		 }			 
	 }
	 if(mon>2) //+2
	 {
		 if((((curyear%4)==0x00)&&((curyear%100)!=0x00))||((curyear%400)==0x00))
			{
				 day +=29;	
			}
			else
			{
				 day +=28;	
			}
	 }
#if(LEAP_YEAR_ADJ>0)	//211025	
	yearAdd = curyear-1970;
	hour	=	yearAdd*8760+day*24;//8760=365*24
	leapyear = yearAdd/4;
	if((yearAdd%4)>2)leapyear += 1;
	hour += leapyear*24;
#else
	 hour	=	year*8760+day*24;//8760=365*24
#endif
	 return hour;
}
uint32_t getcurminute(void)
{
	uint32_t minute=0;	
	readDs1302Time(&rtcTime);
	
	minute = (queryDayToHour(rtcTime.year,rtcTime.month,rtcTime.date)+rtcTime.hour)*60+rtcTime.min;

	return minute;
}



