# Directory Structure

**Last updated:** 2026-05-10
**Focus:** arch

## Top Level

```
shanjing_FreeRTOS/
├── .planning/            # GSD planning artifacts
├── Doc/                  # Documentation (empty)
├── Project/              # All source code and build files
├── Temp/                 # Temporary files (empty)
└── Tools/                # Tools (empty)
```

## Project Source Tree

```
Project/
├── README.md
├── c_cpp_properties.json           # VS Code IntelliSense config (GCC-based)
│
├── inc/                            # Application headers (32 files)
│   ├── main.h                      # Master config, feature toggles (_CTR)
│   ├── FreeRTOSConfig.h            # FreeRTOS kernel tuning (8KB heap, tickless)
│   ├── stm32l1xx_conf.h            # StdPeriph configuration
│   ├── stm32l1xx_it.h              # IRQ handler declarations
│   │
│   ├── cigemPacket.h               # Private binary protocol packet
│   ├── common.h                    # Common utilities
│   ├── crc16.h                     # CRC-16 checksum
│   ├── delay.h                     # Blocking delay
│   ├── deviceset.h                 # Device configuration, EEPROM params
│   ├── ds1302.h                    # External RTC (DS1302)
│   ├── ds18b20.h                   # Temperature sensor (DS18B20)
│   ├── gprs.h                      # SIM7600CE 4G module AT driver
│   ├── hydrology.h                 # Moisture/salinity protocol
│   ├── key.h                       # Button/key interrupt
│   ├── led.h                       # LED control
│   ├── mode.h                      # Operating mode state machine
│   ├── mqttClient.h                # MQTT 3.1.1 serialization
│   ├── mqttTransport.h             # MQTT transport layer (TCP)
│   ├── power.h                     # Power rail control
│   ├── qingxie.h                   # Inclination/tilt sensor
│   ├── rs485.h                     # RS485/Modbus
│   ├── rtc.h                       # STM32 RTC + sleep management
│   ├── sensor.h                    # Soil moisture frequency sensor
│   ├── sk60.h                      # SK60 laser distance sensor
│   ├── spi_flash.h                 # SPI Flash driver
│   ├── stm32_mems.h                # ST MEMS adapter layer
│   ├── stm32_mems_adapter.h        # ADXL345 adapter
│   ├── time.h                      # Time utilities
│   ├── usart.h                     # USART 1/2/3 config
│   ├── w_ads1x15.h                 # ADS1015/1115 ADC driver
│   ├── XL345.h                     # ADXL345 register map
│   └── ymodem.h                    # Ymodem file transfer
│
├── src/                            # Application sources (29 .c, 2 .s)
│   ├── main.c                      # (~958 lines) Entry, tasks, system init
│   ├── startup_stm32l1xx_md.s      # Medium density startup
│   ├── startup_stm32l1xx_mdp.s     # Medium+ density startup (used)
│   ├── stm32l1xx_it.c              # Interrupt handlers
│   ├── system_stm32l1xx.c          # System clock init
│   ├── cigemPacket.c               # (~2,400 lines) Binary protocol
│   ├── common.c                    # Common utilities
│   ├── crc16.c                     # CRC-16
│   ├── delay.c                     # Delay functions
│   ├── deviceset.c                 # Device config management
│   ├── ds1302.c                    # DS1302 RTC driver
│   ├── ds18b20.c                   # DS18B20 temperature driver
│   ├── gprs.c                      # (~12,000 lines) SIM7600CE AT engine
│   ├── hydrology.c                 # Hydrology protocol
│   ├── key.c                       # Key interrupt handler
│   ├── led.c                       # LED control
│   ├── mode.c                      # (~5,800 lines) State machine
│   ├── mqttClient.c                # MQTT packet serialization
│   ├── mqttTransport.c             # MQTT TCP transport
│   ├── power.c                     # Power management
│   ├── qingxie.c                   # Inclination sensor
│   ├── rs485.c                     # RS485/Modbus
│   ├── rtc.c                       # RTC + sleep
│   ├── sensor.c                    # Soil moisture frequency measurement
│   ├── sk60.c                      # SK60 laser
│   ├── spi_flash.c                 # SPI Flash
│   ├── stm32_mems_adapter.c        # ADXL345 via MEMS adapter
│   ├── time.c                      # Time utilities
│   ├── usart.c                     # USART config
│   ├── w_ads1x15.c                 # ADS1015/1115 ADC
│   └── ymodem.c                    # Ymodem protocol
│
├── lib/                            # Third-party libraries
│   ├── FreeRTOS/                   # FreeRTOS Kernel v10.x
│   │   ├── tasks.c, queue.c, list.c, timers.c, ...
│   │   ├── include/                # FreeRTOS API headers
│   │   ├── portable/
│   │   │   ├── RVDS/ARM_CM3/       # port.c, portmacro.h (used)
│   │   │   ├── MemMang/            # heap_4.c (used)
│   │   │   ├── GCC/ARM_CM3/        # (present, unused)
│   │   │   └── ...
│   │   └── .github/workflows/      # FreeRTOS kernel CI (unused by project)
│   │
│   └── Libraries/
│       ├── CMSIS/
│       │   ├── Include/            # core_cm3.h, arm_math.h
│       │   └── Device/ST/STM32L1xx/Include/
│       └── STM32L1xx_StdPeriph_Driver/
│           ├── inc/                # 25 peripheral headers
│           └── src/                # 27 peripheral sources
│
└── build/
    └── MDK-ARM/
        ├── Project.uvprojx         # Keil project
        ├── Project.uvoptx          # Keil options
        ├── Project.uvguix.wxh     # User preferences
        ├── Project_STM32L151RC.dep # Dependencies
        ├── RTE/                    # Run-Time Environment (auto-generated)
        ├── Listings/               # Map file, listings
        └── Objects/                # .o, .axf, .sct, .lnp
```

## Key File Locations

| Purpose | Path |
|---------|------|
| Main entry | `Project/src/main.c:936` |
| System init | `Project/src/main.c:121` |
| Master config | `Project/inc/main.h` |
| FreeRTOS config | `Project/inc/FreeRTOSConfig.h` |
| IRQ handlers | `Project/src/stm32l1xx_it.c` |
| Startup (used) | `Project/src/startup_stm32l1xx_mdp.s` |
| Keil project | `Project/build/MDK-ARM/Project.uvprojx` |
| Linker scatter | `Project/build/MDK-ARM/Objects/Project.sct` |
| Final binary | `Project/build/MDK-ARM/Objects/Project.axf` |
| Map file | `Project/build/MDK-ARM/Listings/Project.map` |
