#ifndef _DS18B20_H_
#define _DS18B20_H_

#include "delay.h"
#include "stm32l1xx.h"
#include "sensor.h"
#include <stdint.h>

#define skipRom     0xcc
#define convert     0x44
#define readTemp    0xbe
#define readId      0x33
#define macthId     0x55

#define MAX_B20_NUM          11

typedef struct
{
    union
    {
        int16_t i;
        float f;
        struct
        {
            uint8_t valueH;
            uint8_t valueL;
            uint8_t valueH1;
            uint8_t valueH2;
        }bytes;
    }value_t;
    float rf;
}b20_value_t;

#define TOP_SENSOR_UNUSE     0x01
#define TOP_SENSOR_USE       0x00

extern __IO uint8_t topSensorFlag;
extern b20_value_t result[MAX_B20_NUM];

extern void readAllTemperature(void);
extern void ds18b20_lowPower(void);

#endif




