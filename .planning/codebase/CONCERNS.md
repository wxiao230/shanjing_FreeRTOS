# Concerns & Technical Debt

**Last updated:** 2026-05-10
**Focus:** concerns

## Critical Issues

### 1. FreeRTOS Heap Size Dangerously Low

- `configTOTAL_HEAP_SIZE = 8 * 1024` (8KB) for the entire system
- Total task stack allocation: ~3.5KB (sensor:1KB, comm:1KB, storage:512B, monitor:512B, power:512B)
- Remaining heap for queues, mutexes, timers, and dynamic allocations is very tight
- Risk of heap exhaustion during complex operations (MQTT packet processing, GPRS AT command buffering)

**Location:** `Project/inc/FreeRTOSConfig.h:13`

### 2. Ymodem Firmware Update Bugs

Three known bugs in Ymodem implementation:
- `ymodem.h:98` — `DOWNLOAD_TIMEOUT` excessively long (`0x100000` iterations)
- `ymodem.c:471` — Commented-out buggy code with workaround
- `ymodem.c:621` — Race condition / timing bug in ACK handling

**Risk:** Firmware upgrade could hang or corrupt flash. No signature verification for OTA images.

**Location:** `Project/src/ymodem.c`, `Project/inc/ymodem.h`

### 3. Credentials Stored Unencrypted in SPI Flash

- Device ID, MQTT username, password (48 bytes), and API keys stored in external SPI Flash
- Auth code (`authCodeBuf[33]`) stored unencrypted
- No encryption at rest for sensitive configuration data

**Location:** `Project/inc/gprs.h:418-455`, `Project/inc/sensor.h:487-488`

## High Concerns

### 4. Large Monolithic Files

| File | Lines | Issue |
|------|-------|-------|
| `gprs.c` | ~12,000 | SIM7600CE AT command engine — tightly coupled, hard to maintain |
| `mode.c` | ~5,800 | Operating mode state machine — monolithic, should be decomposed |
| `cigemPacket.c` | ~2,400 | Binary protocol encoding/decoding |

These files mix multiple concerns (protocol, state management, hardware control) making testing and modification difficult.

### 5. Debug Flags Enabled in Production

```c
#define GPRS_DEBUG          1   // Affects timing and binary size
#define GPRS_STATUS_DEBUG   1
#define MQTT_DEBUG          1
#define SENSOR_DEBUG        1
#define rs485_DEBUG         1
#define CIGE_SENSOR_DATA_DEBUG 1
```

Debug print statements execute on every code path, impacting timing (especially critical for USART and GPRS operations) and increasing code size.

**Location:** `Project/src/gprs.c`, `Project/src/cigemPacket.c`, `Project/src/mqttClient.c`, etc.

### 6. Tight Coupling via Global State

- 50+ `extern` global variables shared across modules
- `mcuRunStatus` used alongside FreeRTOS tasks — incomplete migration from original state-machine architecture
- Flash address map hardcoded with `#define` regions — any layout change requires recompilation
- Global sensor data buffers accessed by multiple tasks without explicit ownership

### 7. No Application Test Coverage

- Zero unit tests, integration tests, or automated validation
- No CI pipeline for application code
- Only manual HW-in-the-loop testing via USART2 debug output
- Regression testing is impossible without hardware

### 8. USART Resource Conflicts

- USART1 shared between 4G module and SK60 laser (mutex-protected but fragile)
- USART3 shared between inclination sensor and RS485/Modbus
- Time-division multiplexing adds complexity and potential for missed data

## Medium Concerns

### 9. No Wear Leveling for SPI Flash

- Circular buffer writes to fixed flash sectors
- Repeated writes to same locations will eventually wear out flash cells
- No bad block management

**Location:** `Project/inc/gprs.h:186-285` (flash address definitions)

### 10. Inconsistent Naming Conventions

Hybrid of `PascalCase`, `camelCase`, and `snake_case` across:
- Functions: `Int2Str()`, `cal_sensor()`, `gprsReceive()`
- Variables: `mcuRunStatus`, `debugStatus`, `mqttrunstatus`
- Macros: Some use conventional `UPPER_CASE`, others don't

### 11. Board Revision Pin Mappings

- `#if defined(NEW_VERSION)` conditionals for GPIO differences
- Hardware revisions not formally documented
- Risk of firmware/hardware mismatch in the field

### 12. Memory Constraints

| Resource | Size | Utilization |
|----------|------|-------------|
| Flash | 256KB | ~49KB source + libraries (estimated ~60% used) |
| RAM | 32KB | FreeRTOS 8KB heap + stack + globals (estimated ~70% used) |
| MDP variant | 384KB | More headroom |

## Low Concerns

### 13. No System Tick Jitter Handling

Tickless idle uses RTC wakeup, but no compensation for tick jitter in timing-critical operations.

### 14. Single Point of Failure for Power Management

Battery monitoring depends on ADS1015 via I2C — if I2C bus hangs, battery level cannot be read, and system cannot make informed power decisions.

### 15. Limited Documentation

- `Doc/` directory is empty
- Hardware schematic PDF exists (`Project/主板原理图 MCUV5.105.pdf`)
- Protocol specification exists (`Project/op服务器通信协议 Insentek Service.xlsx`)
- No architecture decision records (ADRs)
- No inline documentation for complex state machine logic
