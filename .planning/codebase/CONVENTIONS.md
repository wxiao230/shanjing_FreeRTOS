# Coding Conventions

**Last updated:** 2026-05-10
**Focus:** quality

## Naming Conventions

| Category | Convention | Examples |
|----------|------------|----------|
| **Functions** | Mixed: `camelCase`, `snake_case`, `PascalCase` | `Int2Str()`, `cal_sensor()`, `gprsReceive()`, `SPI_FLASH_Init()` |
| **Interrupt handlers** | `PascalCase_Handler` | `HardFault_Handler`, `USART1_IRQHandler` |
| **Global variables** | `camelCase` | `mcuRunStatus`, `debugStatus`, `serverConfig` |
| **FreeRTOS objects** | `x` prefix + `PascalCase` | `xSensorTaskHandle`, `xCommQueue`, `xSPIFlashMutex` |
| **Feature toggles** | `UPPER_SNAKE_CASE` with `_CTR` suffix | `GPRS_USE_CTR`, `MQTT_USE_CTR`, `SK60_CTR` |
| **Macros (constants)** | `UPPER_SNAKE_CASE` | `DEV_TYPE_SOIL`, `MCURUNSTATUS_Init`, `SEND_STATUS_NONE` |
| **Typedefs** | `_t` suffix | `value16_t`, `sensorValue_t`, `iic_value_t` |
| **Include guards** | `__UPPER_SNAKE_CASE_H` | `__MAIN_H`, `__COMMON_H` |

## Code Style

| Element | Style |
|---------|-------|
| **Indentation** | Tabs (no fixed width) |
| **Brace placement** | K&R — opening brace on same line, closing on own line |
| **Spacing** | One space before `(` in control flow keywords: `if (`, `while (`, `for (` |
| **Comments** | Mix of `//` and `/* */`. Chinese comments for logic explanation. ASCII art section headers. |
| **Line length** | No strict limit, some lines exceed 120 chars |

## File Organization Pattern

Headers follow a consistent section structure (from STM32 StdPeriph):
```
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
```

## Error Handling Patterns

| Pattern | Usage |
|---------|-------|
| **Return codes** | `uint32_t` (1=success, 0=error) or `int8_t` (0=success, -1=error) |
| **Error counters** | Global counters incremented on failure, not immediately acted upon |
| **Software reset** | `sysSoftReset()` / `reboot1task()` on critical failures |
| **`assert_param()`** | Defined as no-op (`USE_FULL_ASSERT` not defined) |
| **`while(1)` loop** | Fatal faults (HardFault, MemManage) |

```
Example error pattern:
if (SIM7600CE_AT_cmd("AT+CREG?", ...) != 0) {
    netGsmErrorCounter++;
    printf("Network registration failed\n\r");
}
```

## Configuration Pattern

Compile-time feature gating via `_CTR` macros:
```c
#if (SK60_CTR > 0)
    sk60Task();
#endif
```

Runtime configuration stored in `httpConfig` struct in SPI Flash, loaded at boot.

## Common Patterns

| Pattern | Description |
|---------|-------------|
| **State machine** | `mcuRunStatus` (lifecycle) + `mqttrunstatus` (MQTT connection) — simple variable transitions |
| **Queue-based dispatch** | `xQueueSend(xCommQueue, &cmd, 0)` for inter-task commands |
| **ISR-to-task signaling** | `xSemaphoreGiveFromISR(xADXL345Semaphore, ...)` with `portYIELD_FROM_ISR()` |
| **Union-based type punning** | `value_t` union for packing float/uint32/bytes |
| **Debug print gating** | Per-module debug levels: `#define GPRS_DEBUG 1` + `#if(GPRS_DEBUG > 0)` |
| **Pin abstraction via macros** | Board revision differences handled by `#if defined(NEW_VERSION)` |

## Synchronization Primitives

| Object | Type | Protects |
|--------|------|----------|
| `xSensorQueue` | Queue | Commands to sensor task |
| `xCommQueue` | Queue | Commands to comm task |
| `xSPIFlashMutex` | Mutex | SPI Flash access |
| `xUSART1Mutex` | Mutex | USART1 (4G vs SK60) |
| `xUSART3Mutex` | Mutex | USART3 (tilt vs RS485) |
| `xADXL345Semaphore` | Binary semaphore | ADXL345 interrupt → monitor task |
