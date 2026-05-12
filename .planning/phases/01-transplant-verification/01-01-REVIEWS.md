---
phase: 1
reviewers: [opencode]
reviewed_at: 2026-05-12
plans_reviewed: [01-01-PLAN.md]
---

# Cross-AI Plan Review — Phase 1

## OpenCode Review

### 1. Summary

Phase 1 (移植验证与修复) is a well-structured 4-wave plan covering file-level audit, FreeRTOS architecture compliance, functional verification, and bug fixes. The plan correctly identifies the critical issues: untaken mutexes, bare-metal polling patterns in large source files (gprs.c, mode.c), and missing RTOS primitives in key drivers. The scope boundary is reasonable — audit + critical fixes now, full RTOS refactoring deferred. However, several architectural risks and scope gaps need attention before this phase can be considered complete.

### 2. Strengths

- **Comprehensive audit structure**: 4-wave decomposition (files → architecture → functionality → fixes) covers the full verification surface
- **Correct priority ranking**: F1-F3 (mutex protection for SPI Flash, USART1, USART3) correctly identified as HIGH — these are genuine concurrency bugs
- **Honest scope boundaries**: Explicitly acknowledges gprs.c and mode.c as "too large to RTOS-ify in this phase" — prevents scope creep
- **Interrupt mapping verified**: SVC/PendSV/SysTick → FreeRTOS port macros confirmed correct, ADXL345 ISR uses `xSemaphoreGiveFromISR` correctly
- **Verification report quality**: VERIFICATION-1.md provides concrete per-file delta analysis (29 .c files checked, 3 with documented differences)
- **SUMMARY-1.md honesty**: Accurately reports "partially compliant" and lists remaining work

### 3. Concerns

- **HIGH — gprs.c architecture gap**: 17,968 lines with zero FreeRTOS primitives, containing 15+ `while` polling loops and 18 AT command timeout loops. The plan defers this to Phase 2, but Phase 2 scope is "period scheduling", not "AT engine event-driven refactoring". This is a scope gap — gprs.c will preempt the entire Comm task during AT polling, making task priority meaningless. A separate gprs.c RTOS-ification task must be added to the roadmap, either in Phase 1 or as a new sub-phase.

- **HIGH — mode.c architectural contamination**: 5,787 lines of bare-metal state machine (`mcuRunStatus`) with zero FreeRTOS primitives, called from multiple task contexts. `mcuRunStatus` is declared `__IO uint8_t` (volatile) but single-byte atomicity on Cortex-M3 only guarantees correctness for aligned reads/writes — compound operations like `if(status == X) { status = Y; }` are not atomic. The plan's F5 assessment that "volatile is sufficient" is incorrect for non-atomic read-modify-write sequences. This needs either a mutex or atomic compare-and-swap.

- **MEDIUM — F4 contradiction between PLAN and SUMMARY**: PLAN Wave 2.4 says `delay_ms()` in task context needs replacement by `vTaskDelay`, but SUMMARY-1.md claims "delay_ms() automatically calls vTaskDelay after scheduler start". This is only true if `delay.c` was modified to detect scheduler state and redirect — but VERIFICATION-1.md shows `delay.c` went from 4 functions to 3, removing the SysTick blocking delay. The remaining `delay_ms()` may still be busy-waiting. This needs clarification and verification.

- **MEDIUM — Heap risk assessment missing**: 8KB FreeRTOS heap with 5 tasks (total ~7KB stacks), 2 queues, 3 mutexes, 1 semaphore. No fragmentation analysis or `heap_4` suitability check. If any task creates/destroys objects at runtime (unlikely but possible in MQTT reconnection paths), fragmentation could cause allocation failure. Recommend documenting worst-case heap usage.

- **MEDIUM — Stack sizing assumptions unverified**: Sensor=256 words (1KB), Comm=256 words (1KB). gprs.c alone has call chains exceeding 10 function levels with local buffers (e.g., `sprintf` into stack arrays). 1KB for Comm task may overflow. Stack watermark analysis (`uxTaskGetStackHighWaterMark()`) should be a Phase 1 task.

- **LOW — DS18B20 function count discrepancy**: PLAN says "17 functions → 13 functions (4 missing)", VERIFICATION says "15 → 13". The count inconsistency (17 vs 15 in bare-metal) suggests the baseline was not precisely established. Should re-verify by reading the actual bare-metal source rather than estimating.

- **LOW — No explicit FreeRTOSConfig.h audit**: `configUSE_PREEMPTION`, `configUSE_TIME_SLICING`, `configTICK_RATE_HZ`, `configMINIMAL_STACK_SIZE` settings are not reviewed. These affect scheduling behavior and portability of `vTaskDelay` timeout values.

- **LOW — Sensor data correctness vs existence**: Wave 3 checks that sensor drivers "exist" but not that they produce correct values. For example, does the FreeRTOS version of soil moisture calculation match the bare-metal version's temperature compensation polynomial? A regression test comparing raw sensor → processed value between the two versions is needed.

### 4. Suggestions

1. **Add gprs.c refactoring as Phase 1.5 or restructure Phase 2**: Clearly scope AT command engine restructuring separate from period scheduling. The current deferral to "Phase 2" is misleading.

2. **Replace `volatile` with mutex for `mcuRunStatus`**: Even for single-byte access, protect compound operations with `xMCUStatusMutex` (or a new mutex). Alternatively, use `__atomic` built-ins for compare-and-swap.

3. **Add stack watermark monitoring task**: A low-priority task that periodically logs `uxTaskGetStackHighWaterMark()` for all 5 tasks during development. This finds stack overflows before they manifest as hard faults in the field.

4. **Audit FreeRTOSConfig.h explicitly**: Add a checklist item for `configTICK_RATE_HZ`, `configUSE_PREEMPTION`, `configMAX_SYSCALL_INTERRUPT_PRIORITY`, and `configTOTAL_HEAP_SIZE` against hardware constraints.

5. **Record critical baseline sensor values**: For each sensor type, capture a known-good output from the bare-metal version (e.g., temperature = 25.3°C → DS18B20 raw = 0x019A) and verify the FreeRTOS version produces identical results.

6. **Add heap fragmentation stress test**: Write a test that repeatedly connects/disconnects MQTT 100 times (simulating field network instability) while monitoring `xPortGetFreeHeapSize()` for monotonic decrease.

### 5. Risk Assessment

**Overall: MEDIUM**

| Risk | Severity | Likelihood | Mitigation |
|------|----------|------------|------------|
| gprs.c preempts task scheduling during AT polling | HIGH | Certain | Must be addressed before production |
| mode.c mcuRunStatus race condition | HIGH | Medium | Add mutex or atomic access |
| Comm task stack overflow | HIGH | Medium | Add watermark monitoring and increase to 512 words |
| delay_ms busy-wait in mode.c blocks scheduling | MEDIUM | High | Verify delay.c implementation |
| DS18B20 function gap causes missing temperature reads | MEDIUM | Low | Re-verify against bare-metal file |
| Heap exhaustion after repeated MQTT reconnects | MEDIUM | Low | Add stress test and monitor |

The plan is fundamentally sound but has two critical blind spots: **gprs.c's bare-metal polling will defeat FreeRTOS scheduling**, and **mode.c's unprotected state machine can corrupt shared state**. Both are fixable but must be explicitly task- scoped, not lost between phases.

---

## Consensus Summary

### Agreed Strengths

(Only 1 reviewer — OpenCode — see Strengths section above.)

### Agreed Concerns

(Only 1 reviewer — see Concerns section above.)

### Divergent Views

N/A — single reviewer.
