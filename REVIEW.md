---
phase: code-review
reviewed: 2026-05-14
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

# 代码审查报告

**审查日期:** 2026-05-14
**审查深度:** 深度审查
**审查文件数:** 6
**状态:** 发现问题

## 摘要

本次审查覆盖基于 FreeRTOS 的地质灾害监测设备核心固件。代码库规模庞大（仅 gprs.c 就有约 18K 行），涉及 GPRS/MQTT 通信、传感器采样、电源管理和 Flash 存储。发现多个**严重**问题，包括 ISR 不安全的共享内存访问、信号量泄漏、字符串搜索例程中的未定义行为以及无界缓冲区操作。代码显示出迭代开发的痕迹，有大量注释掉的代码段，以及不一致地使用已添加但从未采用的 `safe_sprintf` 包装器。

---

## 严重问题

### CR-01: monitorTask 信号量泄漏 — xADXL345Semaphore 永不释放

**文件:** `Project/src/main.c:647`
**问题:** `monitorTask` 在第 647 行获取 `xADXL345Semaphore`，但在返回无限循环顶部之前从未调用 `xSemaphoreGive()`。一旦信号量被获取，其他任务（包括后续迭代的 `monitorTask` 自身）都无法再次获取。这导致 ADXL345 监控逻辑仅执行一次后便死锁。
**修复:**
```c
if(xSemaphoreTake(xADXL345Semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
{
    // ... 现有 ADXL345 逻辑 ...
    xSemaphoreGive(xADXL345Semaphore);  // 添加此行
}
```

### CR-02: TIM2_IRQHandler 中 ISR 不安全的全局状态修改

**文件:** `Project/src/stm32l1xx_it.c:110`
**问题:** `TIM2_IRQHandler` 调用 `cal_work()`，后者修改全局变量 `calSensorState`、`timeLocal` 和 `keyLocal`（定义于 `sensor.c`）。这些变量同时被运行在 `sensorTask` 任务上下文中的 `cal_sensor()` 读写。没有临界区、原子访问或 volatile 限定指针保护。这是经典的竞态条件，可能破坏传感器采样状态。
**修复:** 将 `cal_work()` 调用包装在临界区中，或使用 FreeRTOS 流缓冲区/队列从 ISR 向任务传递定时事件：
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

### CR-03: USART1_IRQHandler 中未保护的 GPRS 环形缓冲区访问

**文件:** `Project/src/stm32l1xx_it.c:153-178` 和 `Project/src/gprs.c:304-321`
**问题:** `USART1_IRQHandler` 调用 `gprsReceive()`，后者写入 `GPRS_RX_Buffer[]` 并递增 `GPRS_RX_ptr_Store`。`gprs.c` 中的任务级代码（如 `findStrRxGprs`、`gprsRxcounterCal`）在没有任何临界区的情况下读取这些变量。在 32 位 Cortex-M3 上，启用优化时 `uint16_t` 递增不是原子的，导致撕裂读取和缓冲区损坏。
**修复:** 在 `gprsReceive()` 中的缓冲区指针更新周围使用 `taskENTER_CRITICAL_FROM_ISR()` / `taskEXIT_CRITICAL_FROM_ISR()`，或使用 FreeRTOS 流缓冲区进行 ISR 到任务的数据传输。

### CR-04: findStrRxGprsFromHead100 中未初始化变量导致未定义行为

**文件:** `Project/src/gprs.c:4682-4738`
**问题:** 局部变量 `conuter` 在第 4688 行声明，但在第 4703 行的检查 `if(bufLength > conuter)` 中**从未赋值**就被使用。由于 `conuter` 未初始化，该函数具有未定义行为，可能错误地返回 `FIND_ERROR` 或以任意循环边界继续执行，导致缓冲区溢出或匹配遗漏。
**修复:**
```c
uint16_t findStrRxGprsFromHead100(uint8_t *buf, uint8_t bufLength)
{
    // ...
    uint32_t conuter = gprsRxcounterCal(GPRS_RX_ptr_Out);  // 初始化它！
    if(bufLength > conuter)
    {
        return FIND_ERROR;
    }
    // ...
}
```

### CR-05: sensorTask 中 sprintf 使用固定 30 字节缓冲区导致溢出

**文件:** `Project/src/main.c:394-396`
**问题:**
```c
char buftmp[30];
sprintf(buftmp,"QY:%.2f,%.2f,%.2f",xlResult[32].xg,xlResult[32].yg,xlResult[32].zg);
```
三个 `double` 值格式化为 2 位小数，每个可能产生如 `-123456789012345.67`（17 字符）加分隔符的字符串。总和很容易超过 30 字节，导致栈缓冲区溢出。
**修复:**
```c
char buftmp[64];
snprintf(buftmp, sizeof(buftmp), "QY:%.2f,%.2f,%.2f", xlResult[32].xg, xlResult[32].yg, xlResult[32].zg);
```

### CR-06: cal_sensor 中重试条件永不满足时可能无限循环

**文件:** `Project/src/sensor.c:962-1044`
**问题:** 内部 `for(j = 0x00; j < eachDevNum;)` 循环仅在第 1033 行的条件内递增 `j`：`if(((keyLocal > 4000)&&(fChange<3000000))||(tryi>2))`。如果 `keyLocal` 始终 ≤ 4000**且** `tryi` 永不超过 2（因为 `tryi` 在第 1032 行的同一条件内被重置为 0），`j` 永不递增，循环永远旋转。第 982 行的超时仅适用于 `calSensorState` 等待，而不适用于外层 `j` 循环。
**修复:** 为 `j` 循环添加迭代限制：
```c
uint8_t j_retry = 0;
for(j = 0x00; j < eachDevNum && j_retry < 10; j_retry++)
{
    // ... 采样逻辑 ...
    if(((keyLocal > 4000) && (fChange < 3000000)) || (tryi > 2))
    {
        tryi = 0;
        j++;
        j_retry = 0;
    }
    else
    {
        // 重试同一个 j
    }
}
```

### CR-07: findStrRxGprs 及其变体中无界数组访问

**文件:** `Project/src/gprs.c:4649-4678` 及其变体
**问题:** `findStrRxGprs` 从环形缓冲区指针计算 `conuter`，然后循环 `for(findi = 0x00; findi < conuter; findi++)`。在循环内部访问 `GPRS_RX_Buffer[ptrOut]` 和 `GPRS_RX_Buffer[ptr++]`。虽然 `&GPRS_RX_DATA_SIZE` 掩码应用于 `ptr`，但 `ptrOut` 仅在 `else` 分支内递增时才被掩码。在 `if(GPRS_RX_Buffer[ptrOut]== buf[0])` 分支中，`ptrOut` 在数组访问前未被掩码。如果 `GPRS_RX_ptr_Out` 损坏（例如来自 CR-03 中的竞态），这可能导致读取/写入 `GPRS_RX_Buffer` 数组外部。
**修复:** 始终在数组访问前掩码 `ptrOut`：
```c
ptrOut = ptrOut & GPRS_RX_DATA_SIZE;
if(GPRS_RX_Buffer[ptrOut] == buf[0])
```

### CR-08: 栈溢出钩子中调用 printf — 异常上下文不安全

**文件:** `Project/src/main.c:1097-1102`
**问题:** `vApplicationStackOverflowHook` 调用 `printf("Stack overflow: %s\r\n", pcTaskName);`。在 FreeRTOS 中，栈溢出钩子运行在被溢出任务的上下文或滴答中断中（取决于配置）。`printf` 通常是可重入的，可能分配内存、使用信号量或执行阻塞操作 — 所有这些在异常/ISR 上下文中都不安全。调用 `printf` 后，它调用 `NVIC_SystemReset()`，但系统可能已损坏。
**修复:** 使用原始 UART 寄存器写入或简单的非阻塞输出例程：
```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    // 非阻塞、无分配、无信号量输出
    for(const char *p = "STACKOVF:"; *p; p++) {
        while(!(USART1->SR & USART_SR_TXE));
        USART1->DR = *p;
    }
    NVIC_SystemReset();
}
```

---

## 警告

### WR-01: safe_sprintf 已定义但从未使用 — sprintf 仍不安全

**文件:** `Project/src/gprs.c:203-217`
**问题:** 添加了使用 `vsnprintf` 进行边界检查的 `safe_sprintf` 包装器，但 18K 行文件中**零个调用点**被更新为使用它。数百个原始 `sprintf` 调用仍然容易受到缓冲区溢出攻击。
**修复:** 将高风险 `sprintf` 调用替换为 `safe_sprintf` 或 `snprintf`。优先目标：
- `networkmode()` 第 990 行: `char cmdbuf[20]`
- `findServerIpAddr()` 第 3242 行: `uint8_t cmbuf[100]` 与用户提供的 `addr`
- `autoApnRead()` 第 1653 行: `sprintf(apnBuf, "%s", "CMNET\r\n")`
- `gprsApnConfig()` 第 1676 行: `char apnBuf[50]` 与无界 APN 输入

### WR-02: GPRS 接收缓冲区环绕掩码假设为 2 的幂减 1

**文件:** `Project/src/gprs.c:314`
**问题:** `GPRS_RX_ptr_Store = GPRS_RX_ptr_Store & GPRS_RX_DATA_SIZE;` 假设 `GPRS_RX_DATA_SIZE` 定义为 `0x7FF`（或类似的 `2^n - 1`）。如果宏被改为非 2 的幂大小（例如 1500），环绕逻辑将失效并导致缓冲区溢出。
**修复:** 使用显式取模或断言大小是 2 的幂：
```c
#define GPRS_RX_DATA_SIZE  1023
static_assert((GPRS_RX_DATA_SIZE + 1) && ((GPRS_RX_DATA_SIZE & (GPRS_RX_DATA_SIZE + 1)) == 0), "必须是 2 的幂减 1");
```

### WR-03: 任务栈大小极其紧张

**文件:** `Project/src/main.c:142-146`
**问题:**
- `STORAGE_TASK_STACK = 128` (128 * 4 = 512 字节)
- `MONITOR_TASK_STACK = 128`
- `POWER_TASK_STACK = 128`
考虑局部变量、函数调用帧和 FreeRTOS 任务上下文，512 字节危险地紧张。`commTask` 的 384 字（1536 字节）更好，但鉴于进入 `gprs.c` 的深调用链仍然有风险。
**修复:** 将最小任务栈增加到至少 256 字（1KB）并启用 `configCHECK_FOR_STACK_OVERFLOW`（已设置为 2 — 良好）。在测试期间监控 `uxTaskGetStackHighWaterMark()`。

### WR-04: powerTask 使用阻塞 vTaskDelay 处理可能很长的睡眠间隔

**文件:** `Project/src/main.c:817`
**问题:** `vTaskDelay(pdMS_TO_TICKS(devSleepTime * 1000))` 可以延迟数小时。在此期间，`powerTask` 不持有任何资源，但阻止了低优先级任务的运行（时间片已启用，但长延迟阻塞了此任务的执行路径）。更重要的是，`devSleepTime` 是 `uint32_t`；如果它超过约 400 万秒，`devSleepTime * 1000` 会在 `pdMS_TO_TICKS` 宏之前溢出 32 位。
**修复:**
```c
uint32_t sleepTicks = devSleepTime * 1000UL;
if (devSleepTime > 4294967) sleepTicks = portMAX_DELAY;  /* 防止溢出 */
vTaskDelay(pdMS_TO_TICKS(sleepTicks));
```

### WR-05: findConfigVersion 及类似解析器访问缓冲区越界

**文件:** `Project/src/gprs.c:4841-4863`
**问题:** `findConfigVersion` 循环到 `MAX_CONFIG_BUF_SIZE` 但在第 4843 行访问 `GPRS_RX_Buffer[findStatus+rxi+1]`。如果 `findStatus` 接近 `GPRS_RX_Buffer` 末尾且 `rxi` 接近 `MAX_CONFIG_BUF_SIZE`，索引会溢出数组。此模式在 `findRtcTime`、`findThresholdValue`、`findTimeZone` 和许多其他配置解析器中重复出现。
**修复:** 每次访问都进行边界检查：
```c
for(rxi = 0; rxi < MAX_CONFIG_BUF_SIZE && (findStatus + rxi + 1) <= GPRS_RX_DATA_SIZE; rxi++)
```

### WR-06: 网络解析器中局部缓冲区未初始化或初始化不足

**文件:** `Project/src/gprs.c:1474-1521`
**问题:** `getRssi()` 声明 `char buf[5]` 但仅在条件分支内初始化它。如果 `+CSQ: ` 匹配失败或循环走意外路径，`buf` 在传递给 `atoi()` 时可能未以空字符结尾。
**修复:** 始终将用于字符串解析的局部缓冲区清零：
```c
char buf[5] = {0};
```

### WR-07: stationNamCopy 基于不可信长度字节可能溢出目标缓冲区

**文件:** `Project/src/gprs.c:15220-15227`
**问题:** `stationNamCopy` 复制 `bufIn[0]+1` 字节。如果 `bufIn[0]` 是 255（恶意或损坏的输入），它会复制 256 字节到可能更小的缓冲区（例如第 16931 行声明的 `stationName[15]`）。
**修复:** 添加目标大小参数并强制执行复制限制：
```c
static void stationNamCopy(const uint8_t *bufIn, uint8_t *bufOut, size_t outSize)
{
    size_t len = (bufIn[0] + 1 < outSize) ? (bufIn[0] + 1) : outSize;
    memcpy(bufOut, bufIn, len);
}
```

### WR-08: EXTI15_10_IRQHandler 清除错误的 EXTI 线

**文件:** `Project/src/stm32l1xx_it.c:82-91`
**问题:** 处理程序检查 `EXTI_Line12` 但直接清除 `EXTI->PR = EXTI_Line12` 而不检查 Line 12 是否实际挂起。如果其他线（10-11、13-15）共享此处理程序并触发，它们的挂起位永远不会被清除，导致中断风暴。
**修复:**
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
    // 如需处理其他线
}
```

### WR-09: BusFault 和 UsageFault 处理程序永远旋转不恢复

**文件:** `Project/src/stm32l1xx_it.c:41-49`
**问题:** `BusFault_Handler` 和 `UsageFault_Handler` 执行 `while(1) {}` 而不喂狗、不复位、不记录诊断日志。在已部署的现场设备中，这会导致永久性锁死，需要物理断电重启。
**修复:** 将故障代码记录到保留寄存器或备份 RAM，然后复位：
```c
void BusFault_Handler(void)
{
    rstStatus = RST_STATUS_Bus;
    NVIC_SystemReset();
}
```

### WR-10: gprsAckWait 忙等待循环使用固定迭代计数

**文件:** `Project/src/gprs.c:761-815`
**问题:** `gprsAckWait` 旋转最多 3750 次迭代，每次 `delay_GprsMs(20)` — 最长 75 秒等待。在此期间，调用任务阻塞但继续在紧密轮询循环中消耗 CPU 周期。内部重试路径中没有任务让步。
**修复:** 降低轮询频率并让步：
```c
while((gprsAckFlag >= COMM_ACK) && (gprsDelayCounter < 3750))
{
    gprsDelayCounter++;
    // ... 匹配逻辑 ...
    delay_GprsMs(20);
    taskYIELD();  // 或如果调度器正在运行则 vTaskDelay(1)
}
```

### WR-11: delay_1us 粒度粗糙且可能有除法问题

**文件:** `Project/src/delay.c:27-38`
**问题:** `delay_1us(101)` 导致 `vTaskDelay(pdMS_TO_TICKS(1))` 即 1ms，而非约 100us。对于 `nTime = 100`，它落入忙等待循环。阈值 `nTime > 100` 是任意的且未记录。此外，对于 `nTime = 0xFFFFFFFF`，`(nTime + 999) / 1000` 会溢出。
**修复:** 记录行为并使用更安全的计算：
```c
if(nTime > 100 && isSchedulerRunning()) {
    uint32_t ms = (nTime / 1000) + ((nTime % 1000) ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    return;
}
```

### WR-12: configASSERT 在复位前自旋延迟浪费故障状态时间

**文件:** `Project/inc/FreeRTOSConfig.h:19-24`
**问题:** `configASSERT` 宏在复位前自旋 100 万次迭代。在电池供电设备中，这浪费能量并延迟恢复。它还保持中断禁用（`taskDISABLE_INTERRUPTS()`），所以看门狗可能无法触发。
**修复:**
```c
#define configASSERT(x) \
    if(!(x)) { \
        taskDISABLE_INTERRUPTS(); \
        rstStatus = RST_STATUS_Assert; \
        NVIC_SystemReset(); \
    }
```

---

## 信息

### IN-01: 函数命名不一致 — sensorPoweCtr 拼写错误

**文件:** `Project/src/sensor.c:235`
**问题:** 函数命名为 `sensorPoweCtr`（Power 中缺少 'r'）。此外第 401 行的 `HIGE_FREQ()` 宏应为 `HIGH_FREQ()`。
**修复:** 重命名为 `sensorPowerCtrl` 和 `HIGH_FREQ()` 以提高可读性。

### IN-02: cal_sensor 中大量使用魔法数字

**文件:** `Project/src/sensor.c:912-1100`
**问题:** 值如 `5000`（超时）、`4000`、`3000000`、`2`（最大重试次数）、`500`（电源循环延迟）都是硬编码的，没有符号常量。
**修复:** 定义宏：
```c
#define SENSOR_CAL_TIMEOUT_MS     5000
#define SENSOR_KEY_VALID_MIN      4000
#define SENSOR_FCHANGE_THRESHOLD  3000000UL
#define SENSOR_MAX_RETRIES        2
```

### IN-03: gprs.c 中的死代码和注释掉的代码段

**文件:** `Project/src/gprs.c`（多处）
**问题:** 大块注释掉的代码（例如第 379-498 行、15000-15080 行、16807-16838 行）降低了可维护性。`gprsTimeDeal()` 中的 `#if 0` 块尤其大。
**修复:** 删除死代码或移动到单独的归档文件。

### IN-04: 头文件中未使用 extern 声明的全局变量

**文件:** `Project/src/sensor.c:19-86` 和 `Project/src/gprs.c`
**问题:** `curWat[]`、`curFaw[]`、`curAbc[]`、`sensorBuf[]`、`sensorValue` 等全局变量在 `.c` 文件中定义。其他文件依赖它们在别处声明（可能在未审查的头文件中）。如果声明不一致，会导致链接器错误或静默类型不匹配。
**修复:** 确保每个全局变量都在头文件中声明为 `extern` 并仅在单个 `.c` 文件中定义。

### IN-05: APN 和服务器地址硬编码在固件中

**文件:** `Project/src/gprs.c:241-257`、`Project/src/gprs.c:15413-15440`
**问题:** 服务器地址（`api.insentek.com:54862`、`183.232.33.116:9205`、`121.10.203.49:9205`）和 APN 字符串（`CMNET`、`CTNET`）被编译到二进制中。更改它们需要固件重建。
**修复:** 存储在 Flash/EEPROM 中并带有默认回退；已通过 `devPar.apnBuf` 部分实现，但硬编码默认值仍然存在。

### IN-06: 五个任务加队列的 configTOTAL_HEAP_SIZE 仅 8KB

**文件:** `Project/inc/FreeRTOSConfig.h:13`
**问题:** 有 5 个任务、2 个队列和多个信号量/互斥锁，8KB 堆极其紧张。栈溢出可能表现为堆损坏而非栈溢出钩子触发。
**修复:** 如果 RAM 允许，增加到至少 16KB，或使用 `configAPPLICATION_ALLOCATED_HEAP` 将堆放置在专用 RAM 段中，并使用 `xPortGetFreeHeapSize()` 监控。

---

## 修复状态（开发期间已应用）

| 问题 | 状态 | 提交 | 说明 |
|-------|--------|--------|-------|
| **WR-10** (delay_GprsMs 忙等待) | ✅ 已修复 | `a990a0a` | 替换为调度器运行时使用的 `vTaskDelay` |
| **WR-11** (delay_1us 粗粒度) | ✅ 已修复 | `48831dd` | 为 >100us 添加调度器感知让步 |
| **WR-03** (任务栈大小) | ✅ 已修复 | `48831dd` | COMM 栈：256→384 字 |
| **WR-12** (configASSERT 自旋) | ⚠️ 部分 | `404a68d` | 复位前 100 万次迭代延迟；应移除延迟 |
| **sensor.c 忙等待** | ✅ 已修复 | `9de1a11` | 移除 `delay_powerMs()`，替换为 `delay_ms()` + cal 循环中的 `vTaskDelay(1)` |
| **MDK VECT_TAB_OFFSET** | ✅ 已修复 | `b02612a` | 为 12KB bootloader 设置为 0x3000 |
| **ISR 中 printf** | ✅ 已修复 | `404a68d` | 从 HardFault/NMI/MemManage 处理程序中移除 printf |
| **CR-05** (sensorTask sprintf) | ✅ 已修复 | `404a68d` | 缓冲区已为 30 字节，但审查建议 64+snprintf |

**剩余严重**: CR-01、CR-02、CR-03、CR-04、CR-06、CR-07、CR-08 需要修复。

---

_审查日期: 2026-05-14_
_审查者: gsd-code-reviewer_
_深度: 深度审查_
