# Testing

**Last updated:** 2026-05-10
**Focus:** quality

## Test Infrastructure

**No application-level test infrastructure exists.**

| Item | Status |
|------|--------|
| Unit test framework | **None** (no Unity, CUnit, CMock, Ceedling, etc.) |
| Test directory | **None** — no `test/`, `tests/`, or `spec/` directory |
| Mock files | **None** — no mock or test doubles |
| CI pipeline | **None** for application code |
| HW-in-the-loop tests | **None** — no automation |

## Debug Support

The only testing mechanism is runtime debug output via USART2:

```c
#define GPRS_DEBUG          1   // Enabled in production
#define GPRS_STATUS_DEBUG   1
#define MQTT_DEBUG          1
#define SENSOR_DEBUG        1
#define rs485_DEBUG         1
#define CIGE_SENSOR_DATA_DEBUG 1
```

Debug output is gated by `#if (MODULE_DEBUG > 0)` blocks and printed via `printf()` to USART2 (9600 baud).

## Known Debug/Test Utilities

| Function | File | Purpose |
|----------|------|---------|
| `flashWriterTest()` | `Project/src/mode.c` | Test SPI Flash write functionality |
| `MQTT_TEST_SCH_CTR` | compile flag | Test MQTT command schedule mode |

## FreeRTOS Kernel Tests (Not Applicable)

FreeRTOS kernel vendor has its own test suite (`Project/lib/FreeRTOS/Test/CMock/`) with GitHub Actions CI (`.github/workflows/unit-tests.yml`, `ci.yml`). This tests only the FreeRTOS kernel itself, **not** the application code.

## Development Workflow

Testing is entirely manual:
1. Flash firmware via Keil MDK-ARM
2. Observe behavior via USART2 debug serial
3. Verify sensor readings and MQTT communication
4. Check SPI Flash data retention after power cycle

## Recommendations

- Add Unity or Ceedling for unit testing sensor math and protocol encoding
- Add HWIL (hardware-in-the-loop) testing for sensor drivers
- Disable `*_DEBUG` defines for release builds
- Add compile-time assertion checks for buffer sizes and configuration invariants
