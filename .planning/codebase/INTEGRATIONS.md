# External Integrations

**Last updated:** 2026-05-10
**Focus:** tech

## Cloud / Server

| Service | Protocol | Details |
|---------|----------|---------|
| **Insentek MQTT Server** | MQTT 3.1.1 over TCP | `api.insentek.com:54862` (default). Up to 4 simultaneous MQTT links. Private binary protocol (`cigemPacket`) on top of MQTT. |
| **Firmware OTA** | MQTT + Ymodem | Remote firmware upgrade via MQTT command channel. Firmware binary transferred and written to flash. |

## Communication Modules

| Module | Interface | Protocol | Driver |
|--------|-----------|----------|--------|
| **SIM7600CE** (4G) | USART1 (PA9/PA10) | AT commands | `gprs.c` (~12,000 lines, full AT command engine) |
| **SIM7100** (alternative) | USART1 | AT commands | Selected via `GPRS_MODE_SIM7100` define |
| **SIM808** (alternative) | USART1 | AT commands | Commented-out in code |

## On-board Sensors

| Sensor | Interface | Protocol | Driver |
|--------|-----------|----------|--------|
| **DS18B20** (Temperature) | 1-Wire (GPIO bit-bang) | Dallas 1-Wire | `ds18b20.c` — up to 11 sensors on bus |
| **ADXL345** (3-axis accelerometer) | I2C (bit-bang on PB6/PB7) | I2C + FIFO + Interrupts | `stm32_mems_adapter.c`, `XL345.h` |
| **ADS1015** (ADC, 12-bit) | I2C1 (PB8/PB9) | I2C | `w_ads1x15.c` — battery/solar/external voltage |
| **DS1302** (External RTC) | GPIO bit-bang | 3-wire protocol | `ds1302.c` — backup timekeeping |
| **Inclination/Tilt** | USART3 (PC10/PC11) | Ymodem-like serial | `qingxie.c`, `ymodem.c` |
| **SK60 Laser** (optional) | USART1 (shared with 4G) | SK60 proprietary | `sk60.c` — time-division with GPRS |

## Soil Moisture Sensor Array

- **10 sensor nodes** selected via GPIO lines (PB0, PB1, PC5, PB14, PA11, PC8, PA8, PC9, PD2, PA12)
- **Frequency-domain measurement**: TIM2 gating + EXTI9_5 pulse counting
- **5V power rail** (PA15) must be enabled before measurement

## Storage

| Component | Interface | Driver |
|-----------|-----------|--------|
| **SPI Flash** | SPI1 (PA5/PA6/PA7), CS PB10 | `spi_flash.c` — circular buffer data logging |

## Debug

| Interface | Pins | Baud | Purpose |
|-----------|------|------|---------|
| **USART2** | PA2(TX)/PA3(RX) | 9600 | Debug serial console (`printf`) |

## Power Rails

| Rail | GPIO Control | Function |
|------|-------------|----------|
| 3.3V main | PB15 or PB12 | Main sensor power |
| 3.3V-1 | PB0 (active-low) | Sub-rail 1 |
| 3.3V-2 | PB12 (active-low) | Sub-rail 2 |
| 5V | PA15 or PA12 | Sensor 5V (moisture sensors) |
| Tilt sensor | PC12 | Inclination sensor power |
| 4G module | PB11 | SIM7600CE power control |
