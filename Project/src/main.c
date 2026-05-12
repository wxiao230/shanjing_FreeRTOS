#include "main.h"
#include "led.h"
#include "usart.h"
#include "rtc.h"
#include "sensor.h"
#include "power.h"
#include "ds18b20.h"
#include "spi_flash.h"
#include "time.h"
#include "key.h"
#include "deviceset.h"
#include "gprs.h"
#include "ds1302.h"
#include "mode.h"
#include "w_ads1x15.h"
#include "stm32_mems_adapter.h"
#include <stdlib.h>
#include "hydrology.h"
#include "qingxie.h"
#include "common.h"
#include "string.h"
#if(MQTT_USE_CTR>0)
#include "mqttTransport.h"
#include "cigemPacket.h"
#endif
#if((SK60_CTR>0)||(KXYL_CTR>0))
#include "sk60.h"
#endif
#if(USART_USE_CTR>0)
#include "rs485.h"
#endif
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xCommTaskHandle = NULL;
TaskHandle_t xStorageTaskHandle = NULL;
TaskHandle_t xMonitorTaskHandle = NULL;
TaskHandle_t xPowerTaskHandle = NULL;

QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xCommQueue = NULL;

SemaphoreHandle_t xSPIFlashMutex = NULL;
SemaphoreHandle_t xUSART1Mutex = NULL;
SemaphoreHandle_t xUSART3Mutex = NULL;
SemaphoreHandle_t xSensorBufMutex = NULL;
SemaphoreHandle_t xADXL345Semaphore = NULL;

#define XL345_INT_DATA       0x01
#define XL345_IDLE_DATA      0x02

__IO uint8_t xl345DataState = XL345_IDLE_DATA;
__IO uint8_t intSource = 0;
__IO uint8_t mqttrunstatus = STATUS_NONE;
__IO uint8_t rtcAdjust = 0;
__IO uint8_t gprsActive = 0;
__IO uint8_t gprsInitFlag = 0;

static uint32_t beginSleepTime = 0;
static uint32_t beginAlarmTime = 0;
static __IO uint16_t curGpsInterVal = 0;
static uint32_t gpsIntervalMinuteLast = 0;
static uint8_t curPeriodHour = 0;
static uint8_t lowPowerFlag = 0;

__IO uint32_t usart1OverrunCount = 0;
__IO uint32_t usart2OverrunCount = 0;
__IO uint32_t usart3OverrunCount = 0;

__IO uint32_t beginTime = 0;
__IO uint32_t castWorkTime = 0;
USART_TypeDef* UartHandle;

/* Phase 3: offline cache mode */
#define GPRS_OFFLINE_MAX_RETRY    3
#define GPRS_OFFLINE_BACKOFF_SEC  300
static uint8_t gprsOfflineConsecutiveFails = 0;
static uint8_t gprsOfflineFlag = 0;
static uint32_t gprsOfflineBackoffStart = 0;

static uint8_t gprsSendAllowed(void)
{
    if(gprsOfflineFlag)
    {
        if(castTimeAdj(gprsOfflineBackoffStart) >= GPRS_OFFLINE_BACKOFF_SEC)
        {
            gprsOfflineFlag = 0;
            gprsOfflineConsecutiveFails = 0;
            return 1;
        }
        return 0;
    }
    return 1;
}

static void gprsSendFailed(void)
{
    gprsOfflineConsecutiveFails++;
    if(gprsOfflineConsecutiveFails >= GPRS_OFFLINE_MAX_RETRY)
    {
        gprsOfflineFlag = 1;
        gprsOfflineBackoffStart = getCurrterTime();
    }
}

static void gprsSendSucceeded(void)
{
    gprsOfflineConsecutiveFails = 0;
    gprsOfflineFlag = 0;
}

__IO uint8_t rstStatus = 0;
uint8_t debugStatus = DEBUG_STATUS_DONE;
httpConfig serverConfig;
__IO uint32_t curDevPeriod = 3600;
__IO uint32_t curDevSendPeriod = 3600;
__IO uint8_t mcuPowerStatus = MCU_ON_STATE;
__IO uint8_t mcuRunStatus = 0;
uint8_t sendDataCounter = SEND_STATUS_INIT;
uint8_t fireWareId = 0x00;
__IO uint8_t tiltType = 0;
uint8_t curPeriodType = 0;
__IO uint16_t saveCounter = 0;
#if(USART_USE_CTR==0)
__IO uint8_t sampleTimesContinus = 0;
#endif

#define SENSOR_QUEUE_LEN    8
#define COMM_QUEUE_LEN      8

#define SENSOR_TASK_PRIO    2
#define COMM_TASK_PRIO      3
#define STORAGE_TASK_PRIO   1
#define MONITOR_TASK_PRIO   3
#define POWER_TASK_PRIO     1

#define SENSOR_TASK_STACK   256
#define COMM_TASK_STACK     384  /* Increased from 256 for gprs.c deep call stack */
#define STORAGE_TASK_STACK  128
#define MONITOR_TASK_STACK  128
#define POWER_TASK_STACK    128

enum sensorCmd {
    SENSOR_CMD_SAMPLE = 0x01,
};

enum commCmd {
    COMM_CMD_SEND = 0x01,
    COMM_CMD_MQTT_INIT,
    COMM_CMD_MQTT_PUBLISH,
    COMM_CMD_SEND_LATEST = 0x04,
};

/* Forward declarations for functions defined after systemInit */
void initallInputIo(void);
void resetTimsesAdj(void);
void xl345InitSet(uint8_t intStatus);
void sensorDataSave(void);
uint32_t samplePeriodAdj(uint32_t period, uint32_t currentTime);

/* original main() init code moved here */
static void systemInit(void)
{
#if defined(USE_IAP_BOOT)
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#else
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
#endif

    initallInputIo();
    led_Turn_On(LED_ALL);
    delay_ms(100);
    led_Turn_Off(LED_ALL);
    devUSART_RUN();
    delay_ms(100);
    usart3_init();

    if(readFireWareVerion()==BOOT_SOFTWARE_VER1)
    {
        fireWareStateWrite(BOOT_SOFTWARE_OK);
        printf("boot fireware ver[%d] \r\n",BOOT_SOFTWARE_VER1);
    }
    else
    {
        printf(" boot fireware ver err!\r\n");
#if(BOOT_CHECK_CTR>0)
        fireWareBootIdWrite(BOOT1_ID);
        fireWareStateWrite(BOOT_SOFTWARE_WRITE);
        delay_ms(1000);
        sysSoftReset();
#endif
    }
    resetTimsesAdj();
#if(SWRESET_CTR>0)
    sofeWareWatchBegin();
#endif
    initDs1302(0);
    printf("\r\nrtc init\r\n");
    rtcConfigurationJudge();
    printf("\r\nrtc init ok\r\n");

    httpConfigParameInit(&serverConfig);
    ReadData_SWReset();
    wakeUPNvic();
    NVIC_Time4_Status(TIME_START);
    key_Config();

    /* battery check loop (original for() loop) */
    while(1)
    {
        batStatus = Read_Value(BAT_channel)*6.144/ADS_MAXVALUE;
        readDs1302Time(&rtcTime);
        showDs1302Time(rtcTime);
        printf(" batV:%.2f\r\n",batStatus);
        if(batStatus < BAT_LOWER_VALULE)
        {
            printf("\r\n low power, device stop\r\n");
            delay_ms(5);
#if(SWRESET_CTR>0)
            sofeWareWatchEnd();
#endif
            lowPowerEnter(1200);
#if(SWRESET_CTR>0)
            sofeWareWatchBegin();
#endif
        }
        else{break;}
    }

    SPI_FLASH_Init();
    delay_ms(10);
    recTask((uint8_t *)sensorBuf);

#if(DEV_APN_SET_CTR>0)
    Config_Menu();
#endif
    devFlashInfRead();
    deviceInit();
    eachDevNum = HIG_FRE_ONLY;
    serverAddrDebug();

    readDs1302Time(&rtcTime);
    if(rtcCheck(rtcTime)!=RTCOK)
    {
        initDs1302(1);
        readDs1302Time(&rtcTime);
    }
    ds1302tobuf(rtcTime,sensorBuf);
    rtcTimeSet((uint8_t *)sensorBuf);

    if(Find_MEMS()!=0x00)
        printf("\r\n find mems sensor\r\n");
    else
        printf("\r\n no mems sensor\r\n");

    ADXL345_GetRegisterValue(XL345_INT_SOURCE);
    aint1_Config();
    adxlValue.xyzIntStatue = configValue.xyzSharkkey;
    xl345InitSet(adxlValue.xyzIntStatue);
    xl345DataState =XL345_IDLE_DATA;

    beginSleepTime =  getCurrterTime();
    rtcWakeUpFlig |=  RTC_WAKE_UP_FLAG;

    devGpsCtr = DEV_GSM_GPS;
    if((devPar.gpsInterVal<GPS_INTERVAL_MIN)||(devPar.gpsInterVal>GPS_INTERVAL_MAX))
        devPar.gpsInterVal = GPS_INTERVAL_INIT;
    curGpsInterVal = devPar.gpsInterVal;
    gpsBuf[1].saveMode = GPS_SAVE_MODE;
    configValue.gpsLocation = GPS_LOCATION_INIT;
    gpsIntervalMinuteLast = getcurminute();

    serverConfig.devDensity = configValue.transmissionDensity;
    serverConfig.inItModeCounter = 0;
    serverConfig.insentekKey = devPar.insentekKey;
    if((serverConfig.insentekKey<INSENTEK_KEY_OFF)||(serverConfig.insentekKey>INSENTEK_KEY_ON))
        serverConfig.insentekKey = INSENTEK_KEY_INIT;

    if(eepromValueRead(DATA_EEPROM_BOOT_ID_ADDR)==BOOT1_ID)
        configValue.curfireWareId = 0x31;
    else
        configValue.curfireWareId = 0x32;
    fireWareId = configValue.curfireWareId;

    serverConfig.devAlarmSmsFlag = DEV_ALARM_DATA2;
    serverConfig.devState = DEVICE_OK;
    serverConfig.devAlarm = DEVICE_UNALARM;
    serverConfig.devAlarmSmsFlag = EMPTEY_DATA_TYPE;
    serverConfig.devWaterRelValueState = -1;
    serverConfig.gsmRunMode = devPar.gsmRunMode;
    serverConfig.powerSavingKey = devPar.powerSavingKey;
    serverConfig.networkMode = FUNCITONG_NETMODE_AUTO;

    gsmIntFlag = 0x01;

    led_Turn_Off(LED_ALL);
    usart3_init();
    beginAlarmTime = beginSleepTime;
    if(configValue.devPeriodHour<1)configValue.devPeriodHour=1;
    curPeriodType = configValue.devPeriodType;
    curPeriodHour = configValue.devPeriodHour;
    curDevPeriod = configValue.devPeriod;
    unSendDataPacketUSART = unSendDataPacket;

    if((devPar.dipXThreshold==0)||(devPar.dipXThreshold>90))devPar.dipXThreshold = 40;
    if((devPar.dipYThreshold==0)||(devPar.dipYThreshold>90))devPar.dipYThreshold = 40;

    mcuRunStatus = MCURUNSTATUS_Init;
    mcuPowerStatus = MCU_ON_STATE;

#if(MQTT_USE_CTR>0)
    cigemperiodrunmodeInit();
#endif
}

static void gprsfrimwareTaskInsentek(void)
{
    if((serverConfig.insentekKey != INSENTEK_KEY_OFF)&&(gprsPowerStatus == STATUS_DONE))
    {
        if(serverConfig.gsmStartResult == 1)
        {
            if(serverConfig.firewareFlag == FIREWARE_QEQUEST)
            {
                serverConfig.firewareFlag = FIREWARE_IDLE;
                SWRESET_TIMER_INIT();
                gprsFireWareRxTask(&serverConfig);
                if(serverConfig.firewareFlag == FIREWARE_UPDATA)
                {
                    serverConfig.storeFlagConfig++;
                    configValue.curfireWareId = 0x32;
                }
            }
        }
    }
}

static void sensorTask(void *pvParameters)
{
    uint8_t cmd;
    BaseType_t xStatus;

    for(;;)
    {
        xStatus = xQueueReceive(xSensorQueue, &cmd, portMAX_DELAY);
        if(xStatus != pdPASS) continue;

        switch(cmd)
        {
        case SENSOR_CMD_SAMPLE:
            mcuRunStatus = MCURUNSTATUS_Start;
            mcuRunStatus = MCURUNSTATUS_Sample;

#if(SK60_CTR>0)
            if(devPar.sk60Fuction!=FUNCITONG_SK60_OFF)
            {
#if((QX_CTR==1)&&(SK60_CTR>2))
                tiltPowerOn();
#endif
                sk60Task(SK60_CMD_MEASURE_SLOW);
#if((QX_CTR==1)&&(SK60_CTR>2))
                tiltPowerOff();
#endif
            }
#endif
#if(KXYL_CTR>0)
            if(devPar.kxylFuction == FUNCITONG_TURN_ON)kxylTask();
#endif
            cal_sensor();

#if(QX_CTR>0)
            if(devPar.qxkey != FUNCITONG_TURN_OFF)
            {
                usart3_init();
                qxPower33On();
                {
                    uint8_t qx_i;
                    COM_StatusTypeDef qx_result;
                    double dipXLast = sensorValue.dipX.f;
                    double dipYLast = sensorValue.dipY.f;
#if(QX_CTR>1)
                    tiltPowerOn();
                    for(qx_i=0;qx_i<20;qx_i++)sensorValue.sign[qx_i]=0;
                    tiltTask();
                    if((tiltType>0)&&((fabs(sensorValue.dipX.f-dipXLast)>2)||(fabs(sensorValue.dipY.f-dipYLast)>2)))
                        tiltTask();
                    tiltPowerOff();
                    if(tiltType>0)
                    {
                        char buftmp[30];
                        sprintf(buftmp,"QY:%.2f,%.2f,%.2f",xlResult[32].xg,xlResult[32].yg,xlResult[32].zg);
                        for(qx_i=0;qx_i<20;qx_i++)sensorValue.sign[qx_i]=buftmp[qx_i];
                        scl3300xyz.xg = sensorValue.ax.f;
                        scl3300xyz.yg = sensorValue.ay.f;
                        scl3300xyz.zg = sensorValue.az.f;
                        qxPower33Off();usart3_lowpower();
                    }
                    else
#endif
                    {
                        qxPower33Off();usart3_lowpower();vTaskDelay(pdMS_TO_TICKS(1000));
                        for(qx_i=0;qx_i<4;qx_i++)
                        {
                            memset(sensorBuf,0,96);
                            printf("***************************power qingxie %d***************************\n\r",qx_i);
                            qxPower33On();usart3_init();
                            qx_result = Initiate_A_Talk_sensorData((uint8_t **)&tilt_sensorData);
                            printf("\r\n debug y:%s,result:%d\r\n",(uint8_t*)aPacketData+4,qx_result);
                            qxPower33Off();usart3_lowpower();
                            if(qx_result == COM_OK)
                            {
                                qxValuePrint((uint8_t *)&sensorBuf);
                                if((tiltType==0)&&((fabs(sensorValue.dipX.f-dipXLast)>2)||(fabs(sensorValue.dipY.f-dipYLast)>2)))
                                {qx_i=0;tiltType = 1;}
                                else
                                {break;}
                            }
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                        tiltType = 0;
                    }
                }
            }
#endif
            serverConfig.devWaterRelValueState = devWaterOutput(curWat,curFaw,curAbc);
            if(calCounter==0x01)
            {
                oldWat[ALARM_NODE_1].f = curWat[ALARM_NODE_1].wat.f;
                oldWat[ALARM_NODE_2].f = curWat[ALARM_NODE_2].wat.f;
                oldWatSum.f = curWatSum.f;
            }
            mcuRunStatus = MCURUNSTATUS_Save;
            xQueueSend(xCommQueue, &(uint8_t){COMM_CMD_SEND}, 0);
            break;
        }
    }
}

static void commTask(void *pvParameters)
{
    uint8_t cmd;
    BaseType_t xStatus;

    for(;;)
    {
        xStatus = xQueueReceive(xCommQueue, &cmd, pdMS_TO_TICKS(1000));
        if(xStatus != pdPASS) continue;

        switch(cmd)
        {
        case COMM_CMD_SEND:
#if(GPRS_USE_CTR>0)
            gprsActive = 0;
            if((sendDataCounter > SEND_STATUS_NORMAL)||(rtcAdjust==0)) gprsActive = 4;
            else if(serverConfig.rtcSetFlag == FUNCITONG_TURN_ON) gprsActive = 3;
            else if((mqttServerNum>0)&&(mqttunsendpacketCheck(&serverConfig)==0)) gprsActive = 2;
            else if((unSendDataPacket >= serverConfig.devDensity)&&(serverConfig.insentekKey != INSENTEK_KEY_OFF)) gprsActive = 5;
            if(gprsActive > 0)
            {
                if(!gprsSendAllowed()) break;
                gprsInitFlag = 0;
                serverConfig.gsmStartResult = gsmTaskInit(gprsInitFlag);
                if(serverConfig.gsmStartResult == 1)
                {
                    serverConfig.gsmStartResult = gprsStart();
                    if(serverConfig.gsmStartResult == 1)
                    {
                        gprsSendRxTask(&serverConfig);
                        gprsfrimwareTaskInsentek();
                        serverConfig.dataSentFlag = 0x00;
                        serverConfig.gpsValueFlag = GPS_VALUE_VALID;
                        gsmTaskEnd(LINK_NUM_INSENTEK);
                        gprsActive = 0;
                        gprsSendSucceeded();
                        if(gprsPowerStatus == STATUS_NONE)gprsInitFlag=0;
                        else gprsInitFlag=1;
                    }
                    else { gprsSendFailed(); }
                }
                else { gprsSendFailed(); }
            }
#endif
#if(MQTT_USE_CTR>0)
            if((mqttServerNum>0)&&(devPar.stationMode == FUNCITONG_TURN_ON))
            {
                xQueueSend(xCommQueue, &(uint8_t){COMM_CMD_MQTT_INIT}, 0);
            }
#endif
            break;

        case COMM_CMD_MQTT_INIT:
#if(MQTT_USE_CTR>0)
            for(uint8_t ch=0; ch<mqttServerNum; ch++)
            {
                uint8_t retry;
                mqttLinkChannel = ch;
                mqttrunstatus = MCURUNSTATUS_mqtt_init;
                for(retry = 0; retry < MQTT_RETRY_TIMES_RX; retry++)
                {
                    if(mqttserverinit(ch)!=0) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    mqttrunstatus = MCURUNSTATUS_mqtt_portOpen;
                    if(transport_open(mqtthost,mqttport,ch+1)!=COMM_OK) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    mqttrunstatus = MCURUNSTATUS_mqtt_connect;
                    mqttlen = MQTTSerialize_connect(mqttTxBuffer, MQTT_TXBUFFER_LEN+1, &connectData);
                    if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)!=COMM_OK)
                        { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    mqttrunstatus = MCURUNSTATUS_mqtt_receive;
                    if(transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1)!=1)
                        { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    if(connack_rc != MQTT_CONNECTION_ACCEPTED)
                        { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    mqttrunstatus = MCURUNSTATUS_mqtt_subscribe;
                    mqttlen = MQTTSerialize_subscribe(mqttTxBuffer,MQTT_TXBUFFER_LEN+1,1,&subcribedata);
                    if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)!=COMM_OK)
                        { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    if(transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1)!=1)
                        { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                    mqttrunstatus = MCURUNSTATUS_mqtt_publishdata;
                    while(unSendDataMqtt[ch] > 0)
                    {
                        uint16_t mqttSendlen = 0;
                        if(sensorDataReadToBuf(SEND_TYPE_MQTT,curMqttSendAddr[ch].i,sensorBuf,&mqttSendlen)!=1)
                        {
                            unSendDataMqtt[ch]--;
                            curMqttSendAddr[ch].i += SENSOR_DATA_STORE_SIZE;
                            if(curMqttSendAddr[ch].i >= SENSOR_DATA_END_ADDR)
                                curMqttSendAddr[ch].i = SENSOR_DATA_BASE_ADDR;
                        }
                        if(mqttSendlen>0)
                        {
                            mqttpublishsensordata(&pubdata,sensorBuf,pubdata.dup,sensorValue.countAll.i,connectData.clientID.cstring);
                            mqttlen = MQTTSerialize_publish(mqttTxBuffer, MQTT_TXBUFFER_LEN+1,pubdata);
                            if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)==COMM_OK)
                            {
                                if(transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1)!=1)
                                    { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
                                if(mqttmsgtype == PUBACK)
                                {
                                    unSendDataMqtt[ch]--;
                                    curMqttSendAddr[ch].i += SENSOR_DATA_STORE_SIZE;
                                }
                            }
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    mqttrunstatus = MCURUNSTATUS_mqtt_end;
                    break;
                }
                if(retry >= MQTT_RETRY_TIMES_RX)
                {
                    printf("MQTT ch%d init fail after retry\r\n", ch);
                    mqttSeverStatus[ch] = serverLinkErr;
                }
            }
            mqttrunstatus = STATUS_DONE;
#endif
            break;

        case COMM_CMD_SEND_LATEST:
#if(GPRS_USE_CTR>0) && (MQTT_USE_CTR>0)
            if(mqttServerNum > 0)
            {
                if(!gprsSendAllowed()) break;
                gprsInitFlag = 0;
                serverConfig.gsmStartResult = gsmTaskInit(gprsInitFlag);
                if(serverConfig.gsmStartResult == 1)
                {
                    serverConfig.gsmStartResult = gprsStart();
                    if(serverConfig.gsmStartResult == 1)
                    {
                        for(uint8_t ch=0; ch<mqttServerNum; ch++)
                        {
                            uint8_t retry;
                            mqttLinkChannel = ch;
                            for(retry = 0; retry < MQTT_RETRY_TIMES_RX; retry++)
                            {
                                if(mqttserverinit(ch)!=0) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                if(transport_open(mqtthost,mqttport,ch+1)!=COMM_OK) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                mqttlen = MQTTSerialize_connect(mqttTxBuffer,MQTT_TXBUFFER_LEN+1,&connectData);
                                if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)!=COMM_OK)
                                    { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                if(transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1)!=1)
                                    { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                if(connack_rc != MQTT_CONNECTION_ACCEPTED)
                                    { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                mqttlen = MQTTSerialize_subscribe(mqttTxBuffer,MQTT_TXBUFFER_LEN+1,1,&subcribedata);
                                if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)!=COMM_OK)
                                    { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                if(transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1)!=1)
                                    { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
                                pubdata.dup = 0;
                                pubdata.packetid = sensorValue.countAll.i;
                                mqttpublishsensordata(&pubdata,sensorBuf,0,sensorValue.countAll.i,connectData.clientID.cstring);
                                mqttlen = MQTTSerialize_publish(mqttTxBuffer,MQTT_TXBUFFER_LEN+1,pubdata);
                                if(transport_sendPacketBuffer(mqttTxBuffer,mqttlen,MQTT_WAIT_ACK_DONE,ch+1)==COMM_OK)
                                {
                                    transport_rxTask(sensorBuf,MAX_SENSOR_BUF_SIZE+1,&mqttmsgtype,ch+1);
                                }
                                break;
                            }
                            if(retry >= MQTT_RETRY_TIMES_RX)
                                mqttSeverStatus[ch] = serverLinkErr;
                        }
                        gsmTaskEnd(LINK_NUM_INSENTEK);
                        gprsActive = 0;
                        gprsSendSucceeded();
                    }
                    else { gprsSendFailed(); }
                }
                else { gprsSendFailed(); }
            }
#endif
            break;
        }
    }
}

static void storageTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60000));

        if(xSemaphoreTake(xSPIFlashMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            flashStoreTask(serverConfig);
            servStoreFlagInit(&serverConfig);
            xSemaphoreGive(xSPIFlashMutex);
        }
        else
        {
            printf("ERROR: SPI Flash mutex timeout in storageTask!\r\n");
        }
    }
}

static void monitorTask(void *pvParameters)
{
    for(;;)
    {
        if(xSemaphoreTake(xADXL345Semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            if(adxlStatus == ADXL345_INI2)
            {
                intSource = ADXL345_GetRegisterValue(XL345_INT_SOURCE);
                if(intSource & XL345_WATERMARK)
                {
                    uint8_t entryCount = ADXL345_GetRegisterValue(XL345_FIFO_STATUS) & 0x3f;
                    uint8_t count;
                    xlResult[32].ax = 0; xlResult[32].ay = 0; xlResult[32].az = 0;
                    for(count = 0; count < entryCount; count++)
                    {
                        ADXL345_Read(&xlResult[count]);
                        xlResult[32].ax += xlResult[count].ax;
                        xlResult[32].ay += xlResult[count].ay;
                        xlResult[32].az += xlResult[count].az;
                    }
                    xlResult[32].ax /= entryCount;
                    xlResult[32].ay /= entryCount;
                    xlResult[32].az /= entryCount;

                    if(xlResult[32].ay > 0)
                    {
                        if((abs(xlResult[32].ay-configValue.yValue)) > 80)
                        {
                            if(serverConfig.devAlarm == DEVICE_UNALARM)
                            {
                                beginAlarmTime = getCurrterTime();
                                serverConfig.devState = DEVICE_UNPLUG;
                                serverConfig.devAlarm = DEVICE_ALARM;
                            }
                        }
                        else
                        {
                            if(serverConfig.devAlarm == DEVICE_UNALARM)
                            {
                                beginAlarmTime = getCurrterTime();
                                serverConfig.devState = DEVICE_VIBRATION;
                                serverConfig.devAlarm = DEVICE_ALARM;
                            }
                        }
                    }
                    xl345DataState = XL345_INT_DATA;
                }
            }
        }

        /* periodic ADXL345 read (every second, when no interrupt) */
        readDs1302Time(&rtcTime);
        if(xl345DataState == XL345_IDLE_DATA)
        {
            uint8_t count;
            ADXL345_GetRegisterValue(XL345_INT_SOURCE);
            xlResult[32].ax = 0; xlResult[32].ay = 0; xlResult[32].az = 0;
            for(count = 0; count < 10; count++)
            {
                ADXL345_Read(&xlResult[count]);
                xlResult[32].ax += xlResult[count].ax;
                xlResult[32].ay += xlResult[count].ay;
                xlResult[32].az += xlResult[count].az;
            }
            xlResult[32].ax /= 10; xlResult[32].ay /= 10; xlResult[32].az /= 10;
            xlResult[32].xg /= 10; xlResult[32].yg /= 10; xlResult[32].zg /= 10;
            xl345DataState = XL345_IDLE_DATA;
        }

#if(ALARM_CTR>0)
        if(FUNCITONG_TURN_ON == configValue.moistureThresholdKey)
        {
            uint16_t autoThresh1, autoThresh2, autoThresh3;
            if(oldWat[ALARM_NODE_1].f >= curWat[ALARM_NODE_1].wat.f)
                autoThresh1 = (uint16_t)(oldWat[ALARM_NODE_1].f - curWat[ALARM_NODE_1].wat.f);
            else
                autoThresh1 = (uint16_t)(curWat[ALARM_NODE_1].wat.f - oldWat[ALARM_NODE_1].f);
            if(oldWat[ALARM_NODE_2].f >= curWat[ALARM_NODE_2].wat.f)
                autoThresh2 = (uint16_t)(oldWat[ALARM_NODE_2].f - curWat[ALARM_NODE_2].wat.f);
            else
                autoThresh2 = (uint16_t)(curWat[ALARM_NODE_2].wat.f - oldWat[ALARM_NODE_2].f);
            if(oldWatSum.f >= curWatSum.f)
                autoThresh3 = (uint16_t)(oldWatSum.f - curWatSum.f);
            else
                autoThresh3 = (uint16_t)(curWatSum.f - oldWatSum.f);

            if((autoThresh1 > configValue.moistureThreshold1)||(autoThresh2 > configValue.moistureThreshold1)||(autoThresh3 > configValue.moistureThreshold1))
            {
                curPeriodType = NORMAL_PERIOD_TYPE;
                curDevPeriod = configValue.threshold1Period;
            }
        }
#endif
    }
}

static void powerTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint32_t lastSendTime = 0;

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));

        readDs1302Time(&rtcTime);
        showDs1302Time(rtcTime);
        batStatus = Read_Value(BAT_channel)*6.144/ADS_MAXVALUE;
        adsexterValue = Read_Value(SUN_channel)*6.144/ADS_MAXVALUE;
        printf(" batV:%.2f,chargeV:%.2f\r\n",batStatus,adsexterValue);

        if(batStatus < BAT_LOWER_VALULE)
        {
            lowPowerFlag = 1;
            printf(" low power,enter sleep\r\n");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            lowPowerFlag = 0;
        }

        /* check if it's time to sample */
        uint32_t castSleepTime = castTimeAdj(beginSleepTime);
        if((castSleepTime >= curDevPeriod) && (mcuPowerStatus == MCU_OFF_STATE))
        {
            sendDataCounter = SEND_STATUS_NORMAL;
            lastSendTime = getCurrterTime();
            xQueueSend(xSensorQueue, &(uint8_t){SENSOR_CMD_SAMPLE}, 0);
        }

        /* check if it's time to send independently (when sendPeriod < samplePeriod) */
        if(curDevSendPeriod < curDevPeriod)
        {
            uint32_t sendElapsed = castTimeAdj(lastSendTime);
            if((sendElapsed >= curDevSendPeriod) && (mcuPowerStatus == MCU_OFF_STATE))
            {
                lastSendTime = getCurrterTime();
                xQueueSend(xCommQueue, &(uint8_t){COMM_CMD_SEND_LATEST}, 0);
            }
        }

        if((mcuRunStatus == MCURUNSTATUS_Save)||(mcuRunStatus == MCURUNSTATUS_Send))
        {
            if(xSemaphoreTake(xSPIFlashMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
            {
                sensorDataSave();
                xSemaphoreGive(xSPIFlashMutex);
            }
            else
            {
                printf("ERROR: SPI Flash mutex timeout in powerTask!\r\n");
            }
        }

        if(mcuRunStatus == MCURUNSTATUS_Sleep)
        {
            uint32_t devSleepTime;
            if(curPeriodType != NORMAL_PERIOD_TYPE)
                devSleepTime = nextSendTime(curPeriodHour,configValue);
            else
            {
                uint32_t realTime = samplePeriodAdj(curDevPeriod,beginSleepTime);
                uint32_t elapsed = castTimeAdj(beginSleepTime);
                devSleepTime = (realTime > elapsed) ? (realTime - elapsed) : 0;
            }

            printf(" sleep %d sec\r\n", devSleepTime);
            if(devSleepTime > 0)
            {
                mcuPowerStatus = MCU_OFF_STATE;
                rtcIoLowPower();
                uart2LowPower();
                vTaskDelay(pdMS_TO_TICKS(devSleepTime * 1000));
                mcuPowerStatus = MCU_ON_STATE;
                beginSleepTime = getCurrterTime();
                rtcWakeUpFlig |= RTC_WAKE_UP_FLAG;
            }
        }
    }
}

void xl345InitSet(uint8_t intStatus)
{
    uint16_t delayi=0;
    if(intStatus==XYZ_SHARK_ON)
    {
        printf("\n\r xl345 int on \n\r");
        adxl345InitInt();
        while(adxlStatus == ADXL345_IDLE)
        {
            delayi++;if(delayi>1000)break;delay_ms(10);
        }
        adxlStatus  = ADXL345_IDLE;
        num_of_int2 = 0;
    }
    else
    {
        printf("\n\r xl345 int off \n\r");
        adxl345Init();
    }
}

void gpsIntervalInit(void)
{
    gpsIntervalMinuteLast = getcurminute();
}

void reboot1task(void)
{
    configValue.curfireWareId = 0x31;
    fireWareStateWrite(BOOT_SOFTWARE_WRITE);
    fireWareBootIdWrite(BOOT1_ID);
    sysSoftReset();
}

void sysSoftReset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

#define RESET_TIMES_FLAG  0xAA55
#define RESET_TIMES_MAX   10

__IO uint8_t resetTimes = 0;

#define GPIO_A_PIN  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_15
#define GPIO_B_PIN  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_3|GPIO_Pin_4
#define GPIO_C_PIN  GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13

void initallInputIo(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_A_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_A_PIN);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_C_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void resetTimesClear(void)
{
    value_t data = {0};
    data.words.valueH = RESET_TIMES_FLAG;
    data.words.valueL = 0;
    resetTimes = 0;
    if((*(__IO uint32_t*)DATA_EEPROM_RESETIMES_ADDR)!=data.i)
    {
        eepromValueWrite(DATA_EEPROM_RESETIMES_ADDR,data.i);
    }
}

void resetTimsesAdj(void)
{
    value_t data = {0};
    data.i = (*(__IO uint32_t*)DATA_EEPROM_RESETIMES_ADDR);
    if(data.words.valueH == RESET_TIMES_FLAG)
    {
        resetTimes = data.bytes.value1;
    }
    resetTimes++;
    if(resetTimes>RESET_TIMES_MAX)
    {
        resetTimesClear();
        fireWareBootIdWrite(BOOT1_ID);
        fireWareStateWrite(BOOT_SOFTWARE_WRITE);
        delay_ms(1000);
        sysSoftReset();
    }
    else
    {
        data.words.valueH = RESET_TIMES_FLAG;
        data.bytes.value1 = resetTimes;
        eepromValueWrite(DATA_EEPROM_RESETIMES_ADDR,data.i);
    }
}

void sensorDataSave(void)
{
    uint8_t qx_i;
    uint32_t calcountTmp;
    power331On();
    delay_ms(5);
    SPI_Flash_WAKEUP();
    delay_ms(5);
    calcountTmp = calAllCounter.i;
    for(qx_i=0;qx_i<2;qx_i++)
    {
        devCalCounterSetByInt(DEV_SELF_SET_FLAG);
        devCalCounterRead(&rtcTime);
        if(calcountTmp == calAllCounter.i)break;
    }
    if(calcountTmp != calAllCounter.i)calAllCounter.i = calcountTmp;

    if(serverConfig.testMode == FUNCITONG_TURN_ON)
    {
        sensorDataFlag |= TEST_MODE_FLAG;
    }
    sensorValueCurrent();
    rtcTime = rtcTimeSample;
    if(sensorDataWrite(&sensorDataWriteAddr,sensorDataFlag)==1)
    {
        if((devGpsCtr == DEV_GSM_GPS)&&((configValue.gpsKey == FUNCITONG_TURN_ON)||(gpsBuf[1].useFlag == BUF_UNUSE_FLAG)))
        {
            gpsValueStoreWithSensorData(sensorDataWriteAddr,gpsBuf[1]);
        }
    #if(QX_CTR>1)
        if(tiltType>0)
        {
            xyzValueStoreWithSensorData(sensorDataWriteAddr,scl3300xyz);
        }else
    #endif
        {
            xyzValueStoreWithSensorData(sensorDataWriteAddr,xlResult[32]);
        }
        if((gpsBuf[1].gpsSource != NONE_SOURCE)&&(gpsBuf[1].useFlag == BUF_UNUSE_FLAG))
        {
            gpsBuf[1].useFlag = BUF_USED_FLAG;
        }
    #if(QX_CTR>0)
        if(devPar.qxkey != FUNCITONG_TURN_OFF)qxValueStoreWithSensorData(sensorDataWriteAddr,rtcTime);
    #endif
        printf("\r\nsave sensor data ok %d\r\n",qx_i);
    #if((SK60_CTR>0)||(KXYL_CTR>0))
        if((devPar.kxylFuction == FUNCITONG_TURN_ON)||(devPar.sk60Fuction != FUNCITONG_SK60_OFF))sk60ValueStoreWithSensorData(sensorDataWriteAddr,sk60Value);
        if(devPar.sk60Fuction != FUNCITONG_SK60_OFF)
        {
            if((sk60InitFlag>0)&&(sk60TestMode != FUNCITONG_TURN_ON))sk60InitFlag = 0;
        }
    #endif
        saveCounter++;
    }
    calcountTmp = sensorDataWriteAddr;
    for(qx_i=0;qx_i<2;qx_i++)
    {
        sensorDataWriteAddrStore(DEV_SELF_SET_FLAG);
        sensorDataWriteAddrRead();
        if(calcountTmp == sensorDataWriteAddr)break;
    }
    if(calcountTmp != sensorDataWriteAddr)sensorDataWriteAddr = calcountTmp;
    SPI_Flash_PowerDown();
    power331Off();
    if(rstStatus > 0)
    {
        rstStatus = 0;
        ClearData_SWReset();
    }
}

uint32_t samplePeriodAdj(uint32_t period,uint32_t currentTime)
{
    uint32_t currentTimeTmp;
    uint32_t period_remain;

    currentTimeTmp = currentTime%3600;

    if(period<=3600)
    {
        if(period<60)
        {
            if((60%period)==0)
            {
                period_remain = period- (currentTimeTmp%period);
            }
            else
            {
                period_remain = period- (currentTimeTmp%10);
            }
        }
        else if(((3600%period)==0)&&(period>=300))
        {
            if(currentTimeTmp>=period)currentTimeTmp%=period;
            period_remain = period-currentTimeTmp;
        }
        else if((period%600)==0)
        {
            period_remain = period-(currentTimeTmp%600);
        }
        else if((period%300)==0)
        {
            period_remain = period-(currentTimeTmp%300);
        }
        else
        {
            if(serverConfig.gsmRunMode == FUNCITONG_TURN_ON){period_remain = period- (currentTime%10);return period_remain;}
            if(period>60){period_remain=period-currentTimeTmp%60;}
            else{period_remain=60-currentTimeTmp%60;}
        }
    }
    else
    {
        if((period%3600)==0)period_remain=period-(currentTimeTmp%3600);
        else{period_remain=period-(currentTimeTmp%60);}
    }
    return period_remain;
}

int main(void)
{
    systemInit();

    xSPIFlashMutex = xSemaphoreCreateMutex();
    xUSART1Mutex = xSemaphoreCreateMutex();
    xUSART3Mutex = xSemaphoreCreateMutex();
    xADXL345Semaphore = xSemaphoreCreateBinary();
    xSensorBufMutex = xSemaphoreCreateMutex();

    xSensorQueue = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(uint8_t));
    xCommQueue = xQueueCreate(COMM_QUEUE_LEN, sizeof(uint8_t));

    configASSERT(xSPIFlashMutex != NULL);
    configASSERT(xUSART1Mutex != NULL);
    configASSERT(xUSART3Mutex != NULL);
    configASSERT(xADXL345Semaphore != NULL);
    configASSERT(xSensorBufMutex != NULL);
    configASSERT(xSensorQueue != NULL);
    configASSERT(xCommQueue != NULL);

    xTaskCreate(sensorTask, "Sensor", SENSOR_TASK_STACK, NULL, SENSOR_TASK_PRIO, &xSensorTaskHandle);
    xTaskCreate(commTask, "Comm", COMM_TASK_STACK, NULL, COMM_TASK_PRIO, &xCommTaskHandle);
    xTaskCreate(storageTask, "Storage", STORAGE_TASK_STACK, NULL, STORAGE_TASK_PRIO, &xStorageTaskHandle);
    xTaskCreate(monitorTask, "Monitor", MONITOR_TASK_STACK, NULL, MONITOR_TASK_PRIO, &xMonitorTaskHandle);
    xTaskCreate(powerTask, "Power", POWER_TASK_STACK, NULL, POWER_TASK_PRIO, &xPowerTaskHandle);

    delaySchedulerReady();
    vTaskStartScheduler();

    printf("FATAL: Scheduler start failed!\r\n");
    NVIC_SystemReset();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("Stack overflow: %s\r\n", pcTaskName);
    NVIC_SystemReset();
}
