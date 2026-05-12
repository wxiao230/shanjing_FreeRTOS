#ifndef KEY_H
#define KEY_H

#if defined (NEW_VERSION)
#define READ_KEY_STATUS()						             ((GPIOA->IDR &GPIO_Pin_0)?0:1)
#else
#define READ_KEY_STATUS()						             ((GPIOA->IDR &GPIO_Pin_0)?1:0)
#endif




/*
define the key state

*/
#define KEY_UP_STATE     0x01
#define KEY_DOWN_STATE   0x02


extern __IO uint8_t keyPressTimes;


extern __IO uint8_t keyStatus;
extern void keyInterrupt(void);
extern void keyTimeIniterrupt(void);
extern void key_Config(void);




#endif

