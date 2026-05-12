#include "stm32l1xx.h"
#include "ymodem.h"



extern uint8_t *tilt_sensorData;
extern void usart3_init(void);
extern void usart3_lowpower(void);
extern void usart3_delay(uint32_t nTime);
extern COM_StatusTypeDef Initiate_A_Talk_sensorData(uint8_t ** pp_sensorData);
extern void Initiate_A_Talk_upgrade_file_fromFlash(char *ptr,int size);
extern void tiltPowerOn(void);
extern void tiltPowerOff(void);
extern void tiltTask(void);
extern int8_t tiltTypeCheck(__IO uint8_t *buf);

