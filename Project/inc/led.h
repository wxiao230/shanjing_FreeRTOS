
//#include "stm32f10x.h"
#include "stm32l1xx.h"
#include <stdint.h>


#define LED_ALL   0x01
#define LED_WORK  0x02
#define LED_COMM  0x03


//void LED_config(void);
extern void led_Turn_On(uint8_t led);
extern void led_Turn_Off(uint8_t led);

