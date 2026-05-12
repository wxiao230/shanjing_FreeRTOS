# shanjing_FreeRTOS

基于 FreeRTOS 的 STM32L151RCT6 地质灾害监测设备固件。

## 架构

裸机状态机重构为 FreeRTOS 多任务模型。原始 3200 行 `main.c` 中的状态机（`mcuRunStatus`）被分解为独立任务，通过队列和信号量同步。

## 任务

| 任务 | 优先级 | 栈大小 | 功能 |
|---|---|---|---|
| **Sensor** | 2 | 512 | 传感器采样（温度、水分、盐分、倾角、激光） |
| **Comm** | 3 | 512 | GPRS/MQTT 通信（连接、发布、接收） |
| **Storage** | 1 | 256 | 定时存储数据到 SPI Flash |
| **Monitor** | 3 | 256 | ADXL345 中断监控、报警检测 |
| **Power** | 1 | 256 | 电源管理、电池检测、休眠/唤醒调度 |

## 通信

- **xSensorQueue** (`uint8_t`) — 触发采样的命令
- **xCommQueue** (`uint8_t`) — 触发通信的命令（发送、MQTT 初始化等）
- **xSPIFlashMutex** — SPI Flash 互斥
- **xUSART1Mutex** — USART1 互斥（4G 与 SK60 分时复用）
- **xUSART3Mutex** — USART3 互斥（倾角与 RS485 分时复用）

## 工程结构

```
Project/
├── src/            应用源代码 (*.c)
├── inc/            头文件 (*.h)
├── lib/            依赖库
│   ├── CMSIS/      ARM CMSIS 核心
│   ├── STM32L1xx_StdPeriph_Driver/  STM32 标准外设库
│   └── STM32_EVAL/  EVAL 板支持
├── build/          编译输出
└── README.md
```

## 依赖

需将 FreeRTOS 内核源码 (v10.x) 放入 `lib/FreeRTOS/` 目录。

FreeRTOS 配置在 `inc/FreeRTOSConfig.h`。处理器为 Cortex-M3，使用 Heap_4，时钟 16MHz，Tick 频率 1000Hz。

## 与原项目的关键差异

1. **延时** — `delay_ms()` 重定义为 `vTaskDelay()`（裸机编译时使用 SysTick 忙等待，ARMCC 模式下保留原实现）
2. **SysTick** — 由 FreeRTOS 内核管理，不再由 `delay.c` 重配置
3. **中断** — SVC/PendSV/SysTick 由 FreeRTOS port 层接管，`stm32l1xx_it.c` 中移除相应处理函数
4. **状态机** — 原始 `main.c` 中 `switch(mcuRunStatus)` 的每个分支对应一个独立任务
5. **阻塞等待** — `portMAX_DELAY` 替代 `while(1)` 轮询
6. **共享资源** — SPI Flash、USART1、USART3 由 Mutex 保护
