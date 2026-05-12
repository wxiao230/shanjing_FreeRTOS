# Architecture

**Last updated:** 2026-05-10
**Focus:** arch

## System Overview

FreeRTOS-based firmware for STM32L151RCT6 geological hazard monitoring device. Collects data from multiple sensors, logs to SPI Flash, and transmits via 4G cellular using MQTT with a private binary protocol.

## Task Architecture

5 FreeRTOS tasks with queue-based communication:

| Task | Priority | Stack (words) | Function |
|------|----------|---------------|----------|
| **Sensor** | 2 | 256 (1KB) | Sample all sensors on command |
| **Comm** | 3 | 256 (1KB) | GPRS/MQTT communication |
| **Storage** | 1 | 128 (512B) | SPI Flash data logging |
| **Monitor** | 3 | 128 (512B) | ADXL345 monitoring, alarms |
| **Power** | 1 | 128 (512B) | Battery management, sleep/wake |

## Data Flow

```
Power Task (5s loop)
  │
  ├──→ checks battery voltage
  ├──→ when castSleepTime >= curDevPeriod:
  │     sends SENSOR_CMD_SAMPLE to xSensorQueue
  │
  ▼
Sensor Task
  ├── sk60Task() (if SK60 enabled)
  ├── kxylTask() (if KXYL enabled)
  ├── cal_sensor() — read moisture/temp on 10 nodes
  ├── Initiate_A_Talk_sensorData() — read inclination
  ├── calculate water content
  └── sends COMM_CMD_SEND to xCommQueue
      │
      ▼
  Comm Task
  ├── GPRS init → TCP connect
  ├── MQTT init → publish → receive
  └── Handle OTA commands
      │
      ▼
  Server (api.insentek.com:54862)

Storage Task (60s period)
  └── flashStoreTask() — flush/compress data in SPI Flash

Monitor Task
  ├── On ADXL345 INT2: read FIFO, detect tilt/vibe
  └── Periodic poll: average 10 samples, check thresholds
```

## Communication Stack

```
┌─────────────────────────────────┐
│ cigemPacket (private protocol)  │  ← Application data encoding
├─────────────────────────────────┤
│ mqttClient (MQTT 3.1.1)        │  ← Packet serialize/deserialize
├─────────────────────────────────┤
│ mqttTransport (TCP over AT)    │  ← Connection lifecycle
├─────────────────────────────────┤
│ gprs (SIM7600CE AT driver)      │  ← AT command engine
├─────────────────────────────────┤
│ USART1 (PA9/PA10, 115200 baud)  │  ← Physical layer
└─────────────────────────────────┘
```

## USART Multiplexing

| USART | Primary | Alternate | Mutex |
|-------|---------|-----------|-------|
| USART1 | SIM7600CE 4G (115200) | SK60 laser (time-division) | `xUSART1Mutex` |
| USART2 | Debug console (9600) | — | — |
| USART3 | Inclination sensor (115200) | RS485/Modbus | `xUSART3Mutex` |

## I2C Bus

| Bus | Pins | Devices |
|-----|------|---------|
| I2C1 (HW) / bit-bang | PB8(SCL)/PB9(SDA) | ADS1015 ADC |
| I2C (SW emulated) | PB6(SCL)/PB7(SDA) | ADXL345 accelerometer |

## MCU Lifecycle State Machine

```
Init → Start → Sample → Clock → Save → Send → Sleep → (RTC wake) → Start
```

Controlled by `mcuRunStatus` global variable.

## MQTT Connection State Machine

```
mqtt_init → portOpen → connect → subscribe → publishdata
  → receive → end → unsubscribe → ping (keepalive)
```

Controlled by `mqttrunstatus` global.

## Power Management

1. **Active sampling**: Power on sensors → read → power off
2. **Sleep**: RTC wakeup timer → `lowPowerEnter()` → deep sleep
3. **Battery management**: 3.05V cutoff, 2.8V extended sleep
4. **Peripheral low-power**: individual deactivation per UART/SPI/I2C
5. **FreeRTOS tickless idle**: Idle task enters sleep between ticks

## Data Persistence

- Circular buffer in SPI Flash (`SENSOR_DATA_BASE_ADDR` to `SENSOR_DATA_END_ADDR`)
- Records include: timestamp, sensor values, battery, ADXL345, inclination, SK60
- Unread data tracked via `unSendDataPacket` per MQTT channel
- Data sent in FIFO order

## Entry Points

| Entry | File | Description |
|-------|------|-------------|
| Reset vector | `startup_stm32l1xx_mdp.s` | Stack init → `__main` → `main()` |
| `main()` | `Project/src/main.c:936` | System init → task creation → scheduler start |
| `systemInit()` | `Project/src/main.c:121` | GPIO, clock, peripherals, boot checks |
| Exception handlers | `Project/src/stm32l1xx_it.c` | HardFault, NMI, EXTI, TIM, USART, RTC |
