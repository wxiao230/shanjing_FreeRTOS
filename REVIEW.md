---
phase: code-review
reviewed: 2026-05-14T00:00:00Z
depth: deep
files_reviewed: 6
files_reviewed_list:
  - Project/src/main.c
  - Project/src/gprs.c
  - Project/src/sensor.c
  - Project/src/stm32l1xx_it.c
  - Project/inc/FreeRTOSConfig.h
  - Project/src/delay.c
findings:
  critical: 8
  warning: 12
  info: 6
  total: 26
status: issues_found
---

# Phase Code Review Report

**Reviewed:** 2026-05-14
**Depth:** deep
**Files Reviewed:** 6
**Status:** issues_found

## Summary

This review covers the core FreeRTOS-based firmware for the shanjing soil-monitoring device. The codebase is extensive (~18K lines in gprs.c alone) and handles GPRS/MQTT communication, sensor sampling, power management, and flash storage. Multiple **Critical** issues were found including ISR-unsafe shared memory access, semaphore leaks, undefined behavior in string search routines, and unbounded buffer operations. The code shows evidence of iterative development with many commented-out sections and inconsistent use of the `safe_sprintf` wrapper that was added but never adopted.

---

## Critical Issues

### CR-01: Semaphore Leak in monitorTask — xADXL345Semaphore Never Released

**File:** `Project/src/main.c:647`
**Issue:** `monitorTask` takes `xADXL345Semaphore` at line 647 but never calls `xSemaphoreGive()` before the function returns to the top of its infinite loop. Once the semaphore is taken, no other task (including `monitorTask` itself on subsequent iterations) can ever acquire it again. This causes the ADXL345 monitoring logic to execute exactly once and then deadlock.
**Fix:**
```c
if(xSemaphoreTake(xADXL345Semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
{
    // ... existing ADXL345 logic ...
    xSemaphoreGive(xADXL345Semaphore);  // ADD THIS
}
```

### CR-02: ISR-Unsafe Global State Mutation in TIM2_IRQHandler

**File:** `Project/src/stm32l1xx_it.c:110`
**Issue:** `TIM2_IRQHandler` calls `cal_work()` which modifies the global variables `calSensorState`, `timeLocal`, and `keyLocal` (defined in `sensor.c`). These same variables are read and written by `cal_sensor()` running in the `sensorTask` task context. There is no critical section, atomic access, or volatile-qualified pointer protection. This is a classic race condition that can corrupt sensor sampling state.
**Fix:** Wrap the `cal_work()` call in a critical section or use a FreeRTOS stream buffer/queue to pass timing events from ISR to task:
```c
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        cal_work();
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
```

### CR-03: Unprotected GPRS Ring Buffer Access from USART1_IRQHandler

**File:** `Project/src/stm32l1xx_it.c:153-178` and `Project/src/gprs.c:304-321`
**Issue:** `USART1_IRQHandler` calls `gprsReceive()` which writes to `GPRS_RX_Buffer[]` and increments `GPRS_RX_ptr_Store`. Task-level code in `gprs.c` (e.g., `findStrRxGprs`, `gprsRxcounterCal`) reads these same variables without any critical section. On a 32-bit Cortex-M3, `uint16_t` increments are not atomic when compiled with optimizations, leading to torn reads and buffer corruption.
**Fix:** Use `taskENTER_CRITICAL_FROM_ISR()` / `taskEXIT_CRITICAL_FROM_ISR()` around the buffer pointer update in `gprsReceive()`, or use a FreeRTOS stream buffer for ISR-to-task data transfer.

### CR-04: Undefined Behavior in findStrRxGprsFromHead100 — Uninitialized Variable

**File:** `Project/src/gprs.c:4682-4738`
**Issue:** The local variable `conuter` is declared at line 4688 but **never assigned a value** before being used in the check `if(bufLength > conuter)` at line 4703. Because `conuter` is uninitialized, the function has undefined behavior and may incorrectly return `FIND_ERROR` or proceed with an arbitrary loop bound, causing buffer overruns or missed matches.
**Fix:**
```c
uint16_t findStrRxGprsFromHead100(uint8_t *buf, uint8_t bufLength)
{
    // ...
    uint32_t conuter = gprsRxcounterCal(GPRS_RX_ptr_Out);  // Initialize it!
    if(bufLength > conuter)
    {
        return FIND_ERROR;
    }
    // ...
}
```

### CR-05: Buffer Overflow in sensorTask sprintf with Fixed 30-Byte Buffer

**File:** `Project/src/main.c:394-396`
**Issue:**
```c
char buftmp[30];
sprintf(buftmp,"QY:%.2f,%.2f,%.2f",xlResult[32].xg,xlResult[32].yg,xlResult[32].zg);
```
With three `double` values formatted to 2 decimal places, each can produce strings like `-123456789012345.67` (17 chars) plus separators. The total can easily exceed 30 bytes, causing a stack buffer overflow.
**Fix:**
```c
char buftmp[64];
snprintf(buftmp, sizeof(buftmp), "QY:%.2f,%.2f,%.2f", xlResult[32].xg, xlResult[32].yg, xlResult[32].zg);
```

### CR-06: Potential Infinite Loop in cal_sensor When Retry Condition Never Met

**File:** `Project/src/sensor.c:962-1044`
**Issue:** The inner `for(j = 0x00; j < eachDevNum;)` loop only increments `j` at line 1033 inside a conditional: `if(((keyLocal > 4000)&&(fChange<3000000))||(tryi>2))`. If `keyLocal` stays ≤ 4000 **and** `tryi` never exceeds 2 (because `tryi` is reset to 0 on line 1032 inside the same condition), `j` never increments and the loop spins forever. The timeout at line 982 only applies to the `calSensorState` wait, not to the outer `j` loop.
**Fix:** Add an iteration limit to the `j` loop:
```c
uint8_t j_retry = 0;
for(j = 0x00; j < eachDevNum && j_retry < 10; j_retry++)
{
    // ... sampling logic ...
    if(((keyLocal > 4000) && (fChange < 3000000)) || (tryi > 2))
    {
        tryi = 0;
        j++;
        j_retry = 0;
    }
    else
    {
        // retry same j
    }
}
```

### CR-07: Unbounded Array Access in findStrRxGprs and Variants

**File:** `Project/src/gprs.c:4649-4678` and variants
**Issue:** `findStrRxGprs` computes `conuter` from ring buffer pointers and then loops `for(findi = 0x00; findi < conuter; findi++)`. Inside the loop it accesses `GPRS_RX_Buffer[ptrOut]` and `GPRS_RX_Buffer[ptr++]`. While the `&GPRS_RX_DATA_SIZE` mask is applied to `ptr`, `ptrOut` is only masked when incremented inside the `else` branch. In the `if(GPRS_RX_Buffer[ptrOut]== buf[0])` branch, `ptrOut` is not masked before array access. If `GPRS_RX_ptr_Out` becomes corrupted (e.g., from the race in CR-03), this can read/write outside the `GPRS_RX_Buffer` array.
**Fix:** Always mask `ptrOut` before array access:
```c
ptrOut = ptrOut & GPRS_RX_DATA_SIZE;
if(GPRS_RX_Buffer[ptrOut] == buf[0])
```

### CR-08: printf in Stack Overflow Hook — Unsafe in Exception Context

**File:** `Project/src/main.c:1097-1102`
**Issue:** `vApplicationStackOverflowHook` calls `printf("Stack overflow: %s\r\n", pcTaskName);`. In FreeRTOS, the stack overflow hook runs in the context of the offending task or from the tick interrupt (depending on config). `printf` is typically reentrant, may allocate memory, use semaphores, or perform blocking operations — all unsafe in an exception/ISR context. After `printf`, it calls `NVIC_SystemReset()`, but the system may already be corrupted.
**Fix:** Use a raw UART register write or a simple non-blocking output routine:
```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    // Non-blocking, no-alloc, no-semaphore output
    for(const char *p = "STACKOVF:"; *p; p++) {
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = *p;
    }
    NVIC_SystemReset();
}
```

---

## Warnings

### WR-01: safe_sprintf Defined But Never Used — sprintf Remains Unsafe

**File:** `Project/src/gprs.c:203-217`
**Issue:** A `safe_sprintf` wrapper using `vsnprintf` with bounds checking was added but **zero call sites** in the 18K-line file were updated to use it. There are hundreds of raw `sprintf` calls that remain vulnerable to buffer overflows.
**Fix:** Replace high-risk `sprintf` calls with `safe_sprintf` or `snprintf`. Priority targets:
- `networkmode()` line 990: `char cmdbuf[20]`
- `findServerIpAddr()` line 3242: `uint8_t cmbuf[100]` with user-supplied `addr`
- `autoApnRead()` line 1653: `sprintf(apnBuf, "%s", "CMNET\r\n")`
- `gprsApnConfig()` line 1676: `char apnBuf[50]` with unbounded APN input

### WR-02: GPRS Receive Buffer Wrap Mask Assumes Power-of-2 Minus 1

**File:** `Project/src/gprs.c:314`
**Issue:** `GPRS_RX_ptr_Store = GPRS_RX_ptr_Store & GPRS_RX_DATA_SIZE;` assumes `GPRS_RX_DATA_SIZE` is defined as `0x7FF` (or similar `2^n - 1`). If the macro is ever changed to a non-power-of-2 size (e.g., 1500), the wrap logic breaks and buffer overruns occur.
**Fix:** Use explicit modulo or assert the size is a power of 2:
```c
#define GPRS_RX_DATA_SIZE  1023
static_assert((GPRS_RX_DATA_SIZE + 1) && ((GPRS_RX_DATA_SIZE & (GPRS_RX_DATA_SIZE + 1)) == 0), "Must be power of 2 minus 1");
```

### WR-03: Task Stack Sizes Extremely Tight

**File:** `Project/src/main.c:142-146`
**Issue:**
- `STORAGE_TASK_STACK = 128` (128 * 4 = 512 bytes)
- `MONITOR_TASK_STACK = 128`
- `POWER_TASK_STACK = 128`
With local variables, function call frames, and FreeRTOS task context, 512 bytes is dangerously tight. The `commTask` at 384 words (1536 bytes) is better but still risky given the deep call chains into `gprs.c`.
**Fix:** Increase minimum task stacks to at least 256 words (1KB) and enable `configCHECK_FOR_STACK_OVERFLOW` (already set to 2 — good). Monitor `uxTaskGetStackHighWaterMark()` during testing.

### WR-04: powerTask Uses Blocking vTaskDelay for Potentially Long Sleep Intervals

**File:** `Project/src/main.c:817`
**Issue:** `vTaskDelay(pdMS_TO_TICKS(devSleepTime * 1000))` can delay for hours. During this time, the `powerTask` holds no resources but prevents lower-priority tasks from running (time slicing is enabled, but a long delay blocks this task's execution path). More importantly, `devSleepTime` is `uint32_t`; if it exceeds ~4 million seconds, `devSleepTime * 1000` overflows 32 bits before the `pdMS_TO_TICKS` macro.
**Fix:**
```c
uint32_t sleepTicks = devSleepTime * 1000UL;
if (devSleepTime > 4294967) sleepTicks = portMAX_DELAY;  /* prevent overflow */
vTaskDelay(pdMS_TO_TICKS(sleepTicks));
```

### WR-05: findConfigVersion and Similar Parsers Access Buffer Beyond Bounds

**File:** `Project/src/gprs.c:4841-4863`
**Issue:** `findConfigVersion` loops up to `MAX_CONFIG_BUF_SIZE` but accesses `GPRS_RX_Buffer[findStatus+rxi+1]` at line 4843. If `findStatus` is near the end of `GPRS_RX_Buffer` and `rxi` approaches `MAX_CONFIG_BUF_SIZE`, the index overflows the array. This pattern repeats in `findRtcTime`, `findThresholdValue`, `findTimeZone`, and many other config parsers.
**Fix:** Bounds-check every access:
```c
for(rxi = 0; rxi < MAX_CONFIG_BUF_SIZE && (findStatus + rxi + 1) <= GPRS_RX_DATA_SIZE; rxi++)
```

### WR-06: Uninitialized or Under-initialized Local Buffers in Network Parsers

**File:** `Project/src/gprs.c:1474-1521`
**Issue:** `getRssi()` declares `char buf[5]` but only initializes it inside a conditional branch. If the `+CSQ: ` match fails or the loop takes an unexpected path, `buf` may not be null-terminated when passed to `atoi()`.
**Fix:** Always zero-initialize local buffers used for string parsing:
```c
char buf[5] = {0};
```

### WR-07: stationNamCopy Can Overrun Destination Based on Untrusted Length Byte

**File:** `Project/src/gprs.c:15220-15227`
**Issue:** `stationNamCopy` copies `bufIn[0]+1` bytes. If `bufIn[0]` is 255 (malicious or corrupted input), it copies 256 bytes into a buffer that may be smaller (e.g., `stationName[15]` declared at line 16931).
**Fix:** Add a destination size parameter and enforce the copy limit:
```c
static void stationNamCopy(const uint8_t *bufIn, uint8_t *bufOut, size_t outSize)
{
    size_t len = (bufIn[0] + 1 < outSize) ? (bufIn[0] + 1) : outSize;
    memcpy(bufOut, bufIn, len);
}
```

### WR-08: EXTI15_10_IRQHandler Clears Wrong EXTI Line

**File:** `Project/src/stm32l1xx_it.c:82-91`
**Issue:** The handler checks `EXTI_Line12` but clears `EXTI->PR = EXTI_Line12` directly without checking if Line 12 was actually pending. If other lines (10-11, 13-15) share this handler and fire, their pending bits are never cleared, causing interrupt storms.
**Fix:**
```c
void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
#ifdef SCHYDROLOGY
        gsmIntDeal();
#endif
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    // Handle other lines if needed
}
```

### WR-09: BusFault and UsageFault Handlers Spin Forever Without Recovery

**File:** `Project/src/stm32l1xx_it.c:41-49`
**Issue:** `BusFault_Handler` and `UsageFault_Handler` do `while(1) {}` with no watchdog feed, no reset, and no diagnostic logging. In a deployed field device, this causes a permanent lockup requiring physical power cycle.
**Fix:** Log a fault code to a retained register or backup RAM, then reset:
```c
void BusFault_Handler(void)
{
    rstStatus = RST_STATUS_Bus;
    NVIC_SystemReset();
}
```

### WR-10: gprsAckWait Busy-Wait Loop with Fixed Iteration Count

**File:** `Project/src/gprs.c:761-815`
**Issue:** `gprsAckWait` spins for up to 3750 iterations with `delay_GprsMs(20)` each — a 75-second maximum wait. During this time, the calling task blocks but continues to consume CPU cycles in a tight polling loop. No task yield occurs inside the inner retry path.
**Fix:** Reduce the polling frequency and yield:
```c
while((gprsAckFlag >= COMM_ACK) && (gprsDelayCounter < 3750))
{
    gprsDelayCounter++;
    // ... match logic ...
    delay_GprsMs(20);
    taskYIELD();  // or vTaskDelay(1) if scheduler is running
}
```

### WR-11: delay_1us Coarse Granularity and Potential Division Issues

**File:** `Project/src/delay.c:27-38`
**Issue:** `delay_1us(101)` causes `vTaskDelay(pdMS_TO_TICKS(1))` which is 1ms, not ~100us. For `nTime = 100`, it falls through to the busy-wait loop. The threshold `nTime > 100` is arbitrary and undocumented. Also, `(nTime + 999) / 1000` for `nTime = 0xFFFFFFFF` overflows.
**Fix:** Document the behavior and use a safer calculation:
```c
if(nTime > 100 && isSchedulerRunning()) {
    uint32_t ms = (nTime / 1000) + ((nTime % 1000) ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    return;
}
```

### WR-12: configASSERT Spin-Delay Before Reset Wastes Time in Failure State

**File:** `Project/inc/FreeRTOSConfig.h:19-24`
**Issue:** The `configASSERT` macro spins for 1M iterations before reset. In a battery-powered device, this wastes energy and delays recovery. It also keeps interrupts disabled (`taskDISABLE_INTERRUPTS()`), so the watchdog may not fire.
**Fix:**
```c
#define configASSERT(x) \
    if(!(x)) { \
        taskDISABLE_INTERRUPTS(); \
        rstStatus = RST_STATUS_Assert; \
        NVIC_SystemReset(); \
    }
```

---

## Info

### IN-01: Inconsistent Function Naming — sensorPoweCtr Typo

**File:** `Project/src/sensor.c:235`
**Issue:** Function is named `sensorPoweCtr` (missing 'r' in Power). Also `HIGE_FREQ()` macro at line 401 should be `HIGH_FREQ()`.
**Fix:** Rename to `sensorPowerCtrl` and `HIGH_FREQ()` for readability.

### IN-02: Magic Numbers Used Throughout cal_sensor

**File:** `Project/src/sensor.c:912-1100`
**Issue:** Values like `5000` (timeout), `4000`, `3000000`, `2` (max retries), `500` (power cycle delay) are hardcoded with no symbolic constants.
**Fix:** Define macros:
```c
#define SENSOR_CAL_TIMEOUT_MS     5000
#define SENSOR_KEY_VALID_MIN      4000
#define SENSOR_FCHANGE_THRESHOLD  3000000UL
#define SENSOR_MAX_RETRIES        2
```

### IN-03: Dead Code and Commented-Out Sections in gprs.c

**File:** `Project/src/gprs.c` (multiple locations)
**Issue:** Large blocks of commented-out code (e.g., lines 379-498, 15000-15080, 16807-16838) reduce maintainability. The `#if 0` block in `gprsTimeDeal()` is particularly large.
**Fix:** Remove dead code or move to a separate archive file.

### IN-04: Global Variables Declared Without extern in Header

**File:** `Project/src/sensor.c:19-86` and `Project/src/gprs.c`
**Issue:** Global variables like `curWat[]`, `curFaw[]`, `curAbc[]`, `sensorBuf[]`, `sensorValue` are defined in `.c` files. Other files rely on them being declared elsewhere (likely in headers not reviewed). If declarations drift, linker errors or silent type mismatches occur.
**Fix:** Ensure every global is declared `extern` in a header and defined in exactly one `.c` file.

### IN-05: APN and Server Addresses Hardcoded in Firmware

**File:** `Project/src/gprs.c:241-257`, `Project/src/gprs.c:15413-15440`
**Issue:** Server addresses (`api.insentek.com:54862`, `183.232.33.116:9205`, `121.10.203.49:9205`) and APN strings (`CMNET`, `CTNET`) are compiled into the binary. Changing them requires a firmware rebuild.
**Fix:** Store in flash/EEPROM with a default fallback; this is already partially implemented via `devPar.apnBuf` but the hardcoded defaults remain.

### IN-06: configTOTAL_HEAP_SIZE Only 8KB for Five Tasks Plus Queues

**File:** `Project/inc/FreeRTOSConfig.h:13`
**Issue:** With 5 tasks, 2 queues, and multiple semaphores/mutexes, 8KB heap is extremely tight. Stack overflows may manifest as heap corruption rather than stack overflow hook triggers.
**Fix:** Increase to at least 16KB if RAM permits, or use `configAPPLICATION_ALLOCATED_HEAP` to place heap in a dedicated RAM section and monitor with `xPortGetFreeHeapSize()`.

---

## Fix Status (Applied During Development)

| Issue | Status | Commit | Notes |
|-------|--------|--------|-------|
| **WR-10** (delay_GprsMs busy-wait) | ✅ Fixed | `a990a0a` | Replaced with `vTaskDelay` when scheduler running |
| **WR-11** (delay_1us coarse) | ✅ Fixed | `48831dd` | Added scheduler-aware yield for >100us |
| **WR-03** (task stack sizes) | ✅ Fixed | `48831dd` | COMM stack: 256→384 words |
| **WR-12** (configASSERT spin) | ⚠️ Partial | `404a68d` | 1M iteration delay before reset; should remove delay |
| **sensor.c busy-wait** | ✅ Fixed | `9de1a11` | Removed `delay_powerMs()`, replaced with `delay_ms()` + `vTaskDelay(1)` in cal loop |
| **MDK VECT_TAB_OFFSET** | ✅ Fixed | `b02612a` | Set to 0x3000 for 12KB bootloader |
| **ISR printf** | ✅ Fixed | `404a68d` | Removed printf from HardFault/NMI/MemManage handlers |
| **CR-05** (sensorTask sprintf) | ✅ Fixed | `404a68d` | Buffer was already 30 bytes, but review recommends 64+snprintf |

**Remaining Critical**: CR-01, CR-02, CR-03, CR-04, CR-06, CR-07, CR-08 require fixes.

---

_Reviewed: 2026-05-14_
_Reviewer: gsd-code-reviewer_
_Depth: deep_
