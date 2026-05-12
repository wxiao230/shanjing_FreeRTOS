#include "ds18b20.h"
#include "power.h"
#include "deviceset.h"
#include "time.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* Pin definitions */
#define DS_PORT                     GPIOB
#define RCC_GPIO_DS                 RCC_AHBPeriph_GPIOB
#if defined(NEW_VERSION)
#define DS_DQIO                     GPIO_Pin_13
#else
#define DS_DQIO                     GPIO_Pin_14
#endif

#define ResetDQ()    DS_PORT->BSRRH = DS_DQIO
#define SetDQ()      DS_PORT->BSRRL = DS_DQIO
#define GetDQ()      ((DS_PORT->IDR & (DS_DQIO)) ? 1 : 0)

/* DS18B20 conversion time at 12-bit resolution */
#define B20_CONVERT_DELAY_MS        800

b20_value_t result[MAX_B20_NUM];
__IO uint8_t topSensorFlag = TOP_SENSOR_USE;

/* Locally-calibrated microsecond spin-delay for 1-Wire protocol timing.
   Runs with interrupts disabled inside critical sections. */
static void b20_delay_us(__IO uint32_t us)
{
    __IO uint32_t i;
    for (i = 0; i < us; i++);
}

static void gpio_dq_out(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DS_DQIO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(DS_PORT, &GPIO_InitStructure);
}

static void gpio_dq_in(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = DS_DQIO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(DS_PORT, &GPIO_InitStructure);
}

void ds18b20_lowPower(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_GPIO_DS, ENABLE);
    GPIO_InitStructure.GPIO_Pin = DS_DQIO;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(DS_PORT, &GPIO_InitStructure);
}

static uint8_t ds18b20_reset(void)
{
    uint8_t presence;
    gpio_dq_out();
    ResetDQ();
    b20_delay_us(500);
    SetDQ();
    b20_delay_us(70);
    gpio_dq_in();
    b20_delay_us(5);
    presence = GetDQ();
    b20_delay_us(45);
    gpio_dq_out();
    SetDQ();
    b20_delay_us(500);
    return presence;
}

static void write_bit(uint8_t dat)
{
    ResetDQ();
    if (dat & 0x01)
    {
        b20_delay_us(10);
        SetDQ();
        b20_delay_us(130);
    }
    else
    {
        b20_delay_us(120);
        SetDQ();
        b20_delay_us(20);
    }
}

static uint8_t read_bit(void)
{
    ResetDQ();
    b20_delay_us(2);
    SetDQ();
    b20_delay_us(12);
    return GetDQ();
}

static void write_byte(uint8_t dat)
{
    uint8_t i;
    gpio_dq_out();
    for (i = 8; i > 0; i--)
    {
        write_bit(dat & 0x01);
        dat >>= 1;
    }
    b20_delay_us(220);
}

static uint8_t read_byte(void)
{
    uint8_t i;
    uint8_t dat;
    gpio_dq_out();
    dat = 0;
    for (i = 0; i < 8; i++)
    {
        if (read_bit())
            dat |= 0x01 << i;
        b20_delay_us(230);
    }
    return dat;
}

/* Match a specific DS18B20 by its ROM ID and execute a command. Returns 0 on success. */
static uint8_t match_and_cmd(uint8_t *id, uint8_t cmd)
{
    uint8_t j;
    if (ds18b20_reset())
        return 1;
    write_byte(macthId);
    for (j = 0; j < 8; j++)
        write_byte(id[j]);
    write_byte(cmd);
    return 0;
}

/* Start conversion on all sensors in parallel via Skip ROM broadcast. Returns 0 on success. */
static uint8_t start_all_conversions(void)
{
    uint8_t ret;
    taskENTER_CRITICAL();
    ret = ds18b20_reset();
    if (ret == 0)
    {
        write_byte(skipRom);
        write_byte(convert);
    }
    taskEXIT_CRITICAL();
    return ret;
}

/* Read temperature from all sensors after conversion completes */
static void read_all_results(void)
{
    uint8_t readi;
    uint8_t curB20Id[10];
    const float d1 = 0.0625f;

    for (readi = topSensorFlag; readi < devSenNum + 1; readi++)
    {
        b20IdRead(curB20Id, readi);
        taskENTER_CRITICAL();
        if (match_and_cmd(curB20Id, readTemp) == 0)
        {
            result[readi].value_t.bytes.valueH = read_byte();
            result[readi].value_t.bytes.valueL = read_byte();
        }
        taskEXIT_CRITICAL();
        result[readi].rf = (float)result[readi].value_t.i * d1;
    }
}

void readAllTemperature(void)
{
    uint8_t readi;

    RCC_AHBPeriphClockCmd(RCC_GPIO_DS, ENABLE);

    if (start_all_conversions())
    {
        for (readi = topSensorFlag; readi < devSenNum + 1; readi++)
            result[readi].rf = 0.0f;
        ds18b20_lowPower();
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(B20_CONVERT_DELAY_MS));

    read_all_results();

    for (readi = topSensorFlag; readi < devSenNum + 1; readi++)
    {
        if (result[readi].rf > 80.0f)
            result[readi].rf = 0.0f;
    }
    ds18b20_lowPower();
}