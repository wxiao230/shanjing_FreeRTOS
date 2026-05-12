# 地质灾害监测设备固件 - 使用说明文档

**版本**: v1.0  
**日期**: 2026-05-12  
**适用硬件**: STM32L151RCT6 (Cortex-M3, 256KB Flash, 32KB RAM)  
**RTOS**: FreeRTOS v10.x  

---

## 目录

1. [项目概述](#1-项目概述)
2. [系统架构](#2-系统架构)
3. [目录结构](#3-目录结构)
4. [构建系统](#4-构建系统)
5. [功能配置](#5-功能配置)
6. [工作流程](#6-工作流程)
7. [文件说明](#7-文件说明)
8. [API 参考](#8-api-参考)
9. [烧录与调试](#9-烧录与调试)
10. [故障排查](#10-故障排查)

---

## 1. 项目概述

本项目是基于 FreeRTOS 的地质灾害监测设备固件，运行于 STM32L151RCT6 微控制器。设备通过多种传感器采集环境数据，通过 4G 模块（SIM7600CE）使用 MQTT 协议将数据上报到云端服务器。

### 1.1 支持的传感器

| 传感器 | 接口 | 用途 | 可选 |
|--------|------|------|------|
| DS18B20 | 1-Wire | 温度测量 | 否 |
| 水分/盐分传感器 | 频率计数 | 土壤水分、盐分 | 否 |
| 倾角传感器 (QX) | USART3 | 倾斜角度测量 | 是 |
| ADXL345 | I2C (PB6/PB7) | 三轴加速度、振动监测 | 否 |
| 激光测距 (SK60) | USART1 | 激光测距（与4G分时复用） | 是 |
| ADS1015 | I2C (PB8/PB9) | 电压/电流 ADC 转换 | 否 |
| 孔隙水压力 | - | 仅 MDP 型号支持 | 是 |

### 1.2 通信方式

- **调试串口**: USART2 (PA2/PA3), 115200 baud
- **4G 模块**: USART1 (PA9/PA10), SIM7600CE
- **倾角传感器**: USART3 (PC10/PC11)
- **MQTT 协议**: 通过 4G TCP 连接
- **私有协议**: 二进制数据包格式

---

## 2. 系统架构

### 2.1 软件架构

```
┌─────────────────────────────────────────────────────────────┐
│                        应用层 (Application)                   │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
│  │ Sensor   │  │ Comm     │  │ Storage  │  │ Monitor      │ │
│  │ Task     │  │ Task     │  │ Task     │  │ Task         │ │
│  │ (Prio 2) │  │ (Prio 3) │  │ (Prio 1) │  │ (Prio 3)     │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────┬───────┘ │
│       │             │             │               │         │
│       └─────────────┴─────────────┴───────────────┘         │
│                         队列/信号量同步                        │
├─────────────────────────────────────────────────────────────┤
│                      中间件 (Middleware)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ FreeRTOS     │  │ MQTT Client  │  │ Sensor Drivers   │   │
│  │ Kernel       │  │ (paho)       │  │ (DS18B20, ADXL...)│  │
│  └──────────────┘  └──────────────┘  └──────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│                       硬件抽象层 (HAL)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ STM32L1xx    │  │ CMSIS        │  │ USART/I2C/SPI    │   │
│  │ StdPeriph    │  │ Core         │  │ Drivers          │   │
│  └──────────────┘  └──────────────┘  └──────────────────┘   │
├─────────────────────────────────────────────────────────────┤
│                        硬件层 (Hardware)                      │
│  STM32L151RCT6 │  SIM7600CE │  SPI Flash │  Sensors        │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 FreeRTOS 任务

| 任务名 | 优先级 | 栈大小 | 周期 | 功能描述 |
|--------|--------|--------|------|----------|
| **Sensor** | 2 | 256 words | 事件驱动 | 执行传感器采样（温度、水分、盐分、倾角） |
| **Comm** | 3 | 384 words | 事件驱动 | GPRS/MQTT 通信、数据发送、固件升级 |
| **Storage** | 1 | 128 words | 60s | 定时将数据存储到 SPI Flash |
| **Monitor** | 3 | 128 words | 事件+1s | ADXL345 中断监控、报警检测 |
| **Power** | 1 | 128 words | 5s | 电池检测、休眠/唤醒调度、周期管理 |

**优先级说明**: 
- 数值越大优先级越高（FreeRTOS 默认）
- Comm (3) 和 Monitor (3) 为最高优先级
- Sensor (2) 为中等优先级
- Storage (1) 和 Power (1) 为最低优先级

### 2.3 任务间通信

| 机制 | 名称 | 用途 |
|------|------|------|
| 队列 | `xSensorQueue` | Power → Sensor: 触发采样命令 |
| 队列 | `xCommQueue` | Power/Sensor → Comm: 触发发送命令 |
| 互斥锁 | `xSPIFlashMutex` | 保护 SPI Flash 读写 |
| 互斥锁 | `xUSART1Mutex` | 保护 USART1 (4G模块) |
| 互斥锁 | `xUSART3Mutex` | 保护 USART3 (倾角传感器) |
| 互斥锁 | `xSensorBufMutex` | 保护传感器数据缓冲区 |
| 二值信号量 | `xADXL345Semaphore` | ADXL345 中断 → Monitor 任务 |

---

## 3. 目录结构

```
shanjing_FreeRTOS/
├── .planning/                          # 项目规划文档
│   ├── ROADMAP.md                      # 路线图
│   ├── phases/                         # 阶段执行计划
│   │   ├── 01-transplant-verification/ # Phase 1: 移植验证
│   │   ├── 02-period-scheduling/       # Phase 2: 周期调度
│   │   └── 03-cache-retry/             # Phase 3: 断网缓存
│   └── REVIEW.md                       # 代码审查报告
│
├── Project/                            # 工程主目录
│   ├── inc/                            # 头文件目录 (25个)
│   │   ├── main.h                      # 主头文件：宏定义、全局变量、FreeRTOS声明
│   │   ├── FreeRTOSConfig.h            # FreeRTOS 配置文件
│   │   ├── sensor.h                    # 传感器驱动接口
│   │   ├── gprs.h                      # GPRS/MQTT 接口
│   │   ├── usart.h                     # 串口驱动接口
│   │   ├── spi_flash.h                 # SPI Flash 接口
│   │   ├── power.h                     # 电源管理接口
│   │   ├── rtc.h                       # RTC 实时时钟接口
│   │   ├── ds18b20.h                   # DS18B20 温度传感器
│   │   ├── w_ads1x15.h                 # ADS1015 ADC 驱动
│   │   ├── stm32_mems_adapter.h        # ADXL345 加速度计
│   │   ├── mqttClient.h                # MQTT 客户端
│   │   ├── mqttTransport.h             # MQTT 传输层
│   │   ├── cigemPacket.h               # 数据包封装协议
│   │   ├── mode.h                      # 工作模式管理
│   │   ├── hydrology.h                 # 水文计算
│   │   ├── qingxie.h                   # 倾角传感器协议
│   │   ├── rs485.h                     # RS485/Modbus 接口
│   │   ├── sk60.h                      # 激光测距传感器
│   │   ├── ds1302.h                    # DS1302 RTC 芯片
│   │   ├── deviceset.h                 # 设备配置
│   │   ├── time.h                      # 时间管理
│   │   ├── key.h                       # 按键处理
│   │   ├── led.h                       # LED 控制
│   │   ├── crc16.h                     # CRC16 校验
│   │   ├── common.h                    # 通用工具函数
│   │   ├── stm32l1xx_conf.h            # 标准外设库配置
│   │   ├── stm32l1xx_it.h              # 中断处理声明
│   │   └── ymodem.h                    # Ymodem 升级协议
│   │
│   ├── src/                            # 源文件目录 (28个)
│   │   ├── main.c                      # 主程序：任务创建、初始化、调度
│   │   ├── sensor.c                    # 传感器采样核心逻辑
│   │   ├── gprs.c                      # GPRS/MQTT 通信（17974行，主通信模块）
│   │   ├── mode.c                      # 工作模式管理、Flash存储地址管理
│   │   ├── usart.c                     # USART 驱动实现
│   │   ├── spi_flash.c                 # SPI Flash 驱动
│   │   ├── power.c                     # 电源管理、GPIO控制
│   │   ├── rtc.c                       # RTC 配置、唤醒管理
│   │   ├── ds18b20.c                   # DS18B20 1-Wire 驱动
│   │   ├── w_ads1x15.c                 # ADS1015 I2C 驱动
│   │   ├── stm32_mems_adapter.c        # ADXL345 I2C 驱动
│   │   ├── mqttClient.c                # MQTT 协议实现
│   │   ├── mqttTransport.c             # MQTT TCP 传输层
│   │   ├── cigemPacket.c               # 数据包封装/解析
│   │   ├── hydrology.c                 # 土壤水分/盐分计算
│   │   ├── qingxie.c                   # 倾角传感器通信
│   │   ├── rs485.c                     # RS485/Modbus 实现
│   │   ├── sk60.c                      # 激光测距传感器驱动
│   │   ├── ds1302.c                    # DS1302 RTC 驱动
│   │   ├── deviceset.c                 # 设备参数读取/存储
│   │   ├── time.c                      # 时间计算、看门狗
│   │   ├── key.c                       # 按键扫描、中断
│   │   ├── led.c                       # LED 控制
│   │   ├── crc16.c                     # CRC16 计算
│   │   ├── common.c                    # 通用工具函数
│   │   ├── delay.c                     # 延时函数（支持RTOS调度）
│   │   ├── system_stm32l1xx.c          # 系统时钟配置
│   │   ├── stm32l1xx_it.c              # 中断服务程序
│   │   ├── startup_stm32l1xx_md.s      # 启动文件（中等容量）
│   │   └── startup_stm32l1xx_mdp.s     # 启动文件（中等+容量）
│   │
│   ├── lib/                            # 库文件
│   │   ├── FreeRTOS/                   # FreeRTOS 内核源码
│   │   └── Libraries/                  # STM32 标准外设库 + CMSIS
│   │
│   └── build/gcc/                      # GCC 构建目录
│       ├── CMakeLists.txt              # CMake 构建配置
│       ├── toolchain.cmake             # 工具链配置
│       ├── stm32l151rc.ld              # 链接脚本
│       ├── openocd.cfg                 # OpenOCD 调试配置
│       └── build/                      # 构建输出目录
│
└── bin/                                # 预编译固件
    └── sj321021.bin                    # 预编译二进制文件
```

---

## 4. 构建系统

### 4.1 环境要求

| 工具 | 版本 | 用途 |
|------|------|------|
| ARM GCC | 15.2.1+ | 交叉编译器 |
| CMake | 3.16+ | 构建系统 |
| Ninja | 1.10+ | 构建工具（推荐） |
| OpenOCD | 0.12+ | 烧录/调试（可选） |
| Python | 3.8+ | 辅助脚本 |

### 4.2 首次构建

```bash
# 进入构建目录
cd Project/build/gcc

# 配置构建（Release模式）
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build

# 输出文件
# build/sj321021.bin  - 烧录用二进制文件
# build/sj321021.hex  - 烧录用 HEX 文件
# build/sj321021.elf  - 调试用 ELF 文件
```

### 4.3 构建配置

#### 4.3.1 CMakeLists.txt 关键配置

```cmake
# 预处理器宏定义
add_compile_definitions(
    STM32L1XX_MDP           # 使用中等+容量启动文件
    USE_STDPERIPH_DRIVER    # 使用标准外设库
    HSE_VALUE=8000000       # 外部晶振 8MHz
    STM32L151RC             # 目标芯片型号
    VECT_TAB_OFFSET=0x3000  # 中断向量表偏移（Bootloader后）
)
```

#### 4.3.2 编译选项

| 选项 | 说明 |
|------|------|
| `-mcpu=cortex-m3` | 目标 Cortex-M3 |
| `-mthumb` | Thumb 指令集 |
| `-O2` | 优化级别 2（平衡大小和速度）|
| `-g3` | 最大调试信息 |
| `-ffunction-sections` | 函数级链接优化 |
| `-fdata-sections` | 数据级链接优化 |
| `--gc-sections` | 移除未使用段 |
| `-specs=nano.specs` | 使用 newlib-nano（小体积）|
| `-specs=nosys.specs` | 无系统调用（裸机）|

#### 4.3.3 内存布局

```
FLASH (256KB):
  0x0800_0000 - 0x0800_2FFF  Bootloader (12KB)
  0x0800_3000 - 0x0803_FFFF  Application (244KB)

RAM (32KB):
  0x2000_0000 - 0x2000_7FFF  全部可用
  
当前固件大小:
  text: 139,068 bytes (54.3% Flash)
  data: 648 bytes
  bss:  20,616 bytes (64.9% RAM)
```

---

## 5. 功能配置

### 5.1 核心功能开关

所有功能开关位于 `Project/inc/main.h` 文件中，通过修改宏定义值来启用/禁用功能。

#### 5.1.1 GPRS 通信配置

```c
#define GPRS_USE_CTR    2   // GPRS功能控制
```

| 值 | 说明 | 适用场景 |
|----|------|----------|
| 0 | 禁用 GPRS，不上传数据 | 纯本地采集/调试 |
| 1 | 按定时周期发送 GPRS 数据 | 省电模式 |
| 2 | 每次采集完成后都发送 | 实时监测模式（默认）|

```c
#define MQTT_USE_CTR    1   // MQTT功能控制（仅在 GPRS_USE_CTR>0 时有效）
```

| 值 | 说明 |
|----|------|
| 0 | 禁用 MQTT，使用私有协议 |
| 1 | 启用 MQTT 协议（默认）|

#### 5.1.2 传感器配置

```c
#define QX_CTR              1   // 倾角传感器
#define SK60_CTR            0   // 激光测距传感器
#define KXYL_CTR            0   // 孔隙水压力（仅MDP型号）
#define ACCE_CTR            1   // 加速度计功能
```

| 宏 | 值=0 | 值=1 | 值=2 |
|----|------|------|------|
| QX_CTR | 禁用倾角 | 仅采集 | 采集+支持校准 |
| SK60_CTR | 禁用激光 | 启用激光 | 启用+倾角复用 |
| KXYL_CTR | 禁用水压 | 启用水压 | - |

#### 5.1.3 通信接口配置

```c
#define USART_USE_CTR       0   // RS485/Modbus 功能
```

| 值 | 说明 |
|----|------|
| 0 | 禁用 RS485 |
| 1 | 每次采集后通过 RS485 发送 |
| 2 | 每次采集后 GPRS+RS485 同时发送 |

#### 5.1.4 高级功能配置（仅MDP型号）

```c
#define GPRS_SLEEP_CTR          1   // GPRS休眠功能
#define ALARM_CTR               1   // 自动预警功能
#define UPGRADE_FRIMWARE_CTR    1   // MQTT远程固件升级
#define MQTT_TEST_SCH_CTR       0   // 测试模式
#define BEEP_ALARM_CTR          0   // 蜂鸣器报警
#define SECOND_MQTT_ADDR        1   // 第二平台MQTT地址
```

### 5.2 配置示例

#### 示例 1: 基础版（仅土壤监测，无倾角，无4G）

```c
#define GPRS_USE_CTR        0   // 无4G
#define MQTT_USE_CTR        0   // 无MQTT
#define QX_CTR              0   // 无倾角
#define SK60_CTR            0   // 无激光
#define USART_USE_CTR       0   // 无RS485
#define KXYL_CTR            0   // 无水压
```

**功能**: 本地采集土壤温度/水分/盐分，通过调试串口输出

#### 示例 2: 标准版（土壤+倾角+4G+MQTT）

```c
#define GPRS_USE_CTR        2   // 每次采集都发4G
#define MQTT_USE_CTR        1   // 启用MQTT
#define QX_CTR              1   // 启用倾角采集
#define SK60_CTR            0   // 无激光
#define USART_USE_CTR       0   // 无RS485
#define KXYL_CTR            0   // 无水压
#define GPRS_SLEEP_CTR      1   // GPRS休眠
#define ALARM_CTR           1   // 预警功能
```

**功能**: 完整的地质灾害监测，含倾角报警、MQTT上报、GPRS休眠省电

#### 示例 3: 高级版（全功能）

```c
#define GPRS_USE_CTR        2
#define MQTT_USE_CTR        1
#define QX_CTR              2       // 倾角+校准
#define SK60_CTR            1       // 激光测距
#define USART_USE_CTR       1       // RS485
#define KXYL_CTR            1       // 孔隙水压力
#define GPRS_SLEEP_CTR      1
#define ALARM_CTR           1
#define UPGRADE_FRIMWARE_CTR 1      // 远程升级
#define SECOND_MQTT_ADDR    2       // 双平台
```

**功能**: 全传感器支持、双平台上传、远程升级、RS485扩展

### 5.3 配置验证

修改 `main.h` 后，必须重新编译：

```bash
cd Project/build/gcc
cmake --build build --clean-first
```

---

## 6. 工作流程

### 6.1 系统启动流程

```
上电/复位
    │
    ▼
┌─────────────────┐
│  systemInit()   │  配置时钟 (32MHz)、GPIO、外设
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  硬件初始化      │  USART2(调试)、LED、DS1302 RTC、SPI Flash
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  电池检测循环    │  低电压时进入休眠，否则继续
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  设备参数加载    │  从 SPI Flash 读取配置、校准参数
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  FreeRTOS 初始化 │  创建队列、信号量、任务
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ vTaskStartScheduler() │ 启动调度器，任务开始运行
└─────────────────┘
```

### 6.2 数据采集周期

```
Power Task (每5秒执行一次)
    │
    ├── 读取电池电压
    ├── 检查休眠条件 ──→ 低电压 → 进入低功耗休眠
    │
    ├── 检查采集周期 ──→ 到达 curDevPeriod ──→ 发送 SENSOR_CMD_SAMPLE
    │                       到 xSensorQueue
    │
    └── 检查发送周期 ──→ 到达 curDevSendPeriod ──→ 发送 COMM_CMD_SEND_LATEST
                            (当 sendPeriod < samplePeriod)

Sensor Task (事件驱动)
    │
    ├── 接收 SENSOR_CMD_SAMPLE
    ├── 执行 cal_sensor() ──→ 采集温度/水分/盐分
    ├── 执行倾角采集 (如果 QX_CTR>0)
    ├── 执行激光测距 (如果 SK60_CTR>0)
    ├── 数据存入 sensorValue 结构体
    ├── mcuRunStatus = MCURUNSTATUS_Save
    └── 发送 COMM_CMD_SEND 到 xCommQueue

Storage Task (每60秒执行一次)
    │
    └── flashStoreTask() ──→ 将未发送数据写入 SPI Flash
```

### 6.3 数据发送流程

```
Comm Task (事件驱动)
    │
    ├── 接收 COMM_CMD_SEND
    │   ├── 判断发送条件 (RTC校准、未发包数、MQTT状态)
    │   ├── 如果满足条件:
    │   │   ├── gsmTaskInit() ──→ 初始化4G模块
    │   │   ├── gprsStart() ──→ 建立TCP连接
    │   │   ├── gprsSendRxTask() ──→ 发送数据+接收响应
    │   │   ├── gprsfrimwareTaskInsentek() ──→ 检查固件升级
    │   │   └── gsmTaskEnd() ──→ 断开连接
    │   └── 如果不满足: 跳过本次发送
    │
    ├── 接收 COMM_CMD_MQTT_INIT (如果 MQTT_USE_CTR>0)
    │   ├── 遍历所有MQTT通道
    │   ├── mqttserverinit() ──→ 初始化MQTT客户端
    │   ├── transport_open() ──→ 建立TCP连接
    │   ├── MQTTSerialize_connect() ──→ 发送CONNECT包
    │   ├── MQTTSerialize_subscribe() ──→ 发送SUBSCRIBE包
    │   ├── 循环发送未发送数据:
    │   │   ├── sensorDataReadToBuf() ──→ 从Flash读取
    │   │   ├── mqttpublishsensordata() ──→ 封装PUBLISH包
    │   │   └── MQTTSerialize_publish() ──→ 发送
    │   └── 断开连接
    │
    └── 接收 COMM_CMD_SEND_LATEST (如果 sendPeriod < samplePeriod)
        └── 轻量级MQTT发布当前最新数据（不读Flash）
```

### 6.4 低功耗休眠流程

```
Power Task
    │
    ├── 计算下次唤醒时间
    │   ├── 如果 curPeriodType != NORMAL: 使用 nextSendTime()
    │   └── 否则: samplePeriodAdj(curDevPeriod, beginSleepTime)
    │
    ├── 设置 RTC 唤醒定时器
    ├── 关闭外设电源:
    │   ├── 关闭 5V 传感器电源 (PA15)
    │   ├── 关闭倾角传感器电源 (PC12)
    │   ├── USART 进入低功耗模式
    │   └── SPI Flash 进入 PowerDown
    │
    ├── 进入 STOP 模式 (vTaskDelay)
    │
    └── RTC 唤醒后:
        ├── 恢复外设电源
        ├── 重新初始化 USART
        └── 继续主循环
```

### 6.5 报警检测流程

```
Monitor Task
    │
    ├── 等待 xADXL345Semaphore (ADXL345中断触发)
    │   ├── 读取 FIFO 数据 (32组加速度值)
    │   ├── 计算平均值
    │   ├── 如果 ay 变化 > 80: 设备被拔起 (DEVICE_UNPLUG)
    │   └── 否则: 振动报警 (DEVICE_VIBRATION)
    │
    ├── 周期性读取 (每秒)
    │   └── 读取10组数据计算平均值
    │
    └── 水分阈值检查 (如果 ALARM_CTR>0)
        ├── 计算水分变化量
        ├── 如果变化 > moistureThreshold: 触发报警
        └── 发送报警数据到服务器
```

### 6.6 数据存储格式

SPI Flash 使用环形缓冲区存储传感器数据，每条记录 512 字节：

```
偏移    内容                    大小
─────────────────────────────────────
0x00    包头标记 (0x2c, 0x2c)   2 bytes
0x02    数据类型                1 byte
0x04    年                      1 byte
0x05    月                      1 byte
0x06    日                      1 byte
0x07    时                      1 byte
0x08    分                      1 byte
0x09    秒                      1 byte
0x0A    传感器数据...           ...
0x200   下一条记录
```

**Flash 地址范围**:
- `SENSOR_DATA_BASE_ADDR`: 数据存储起始地址
- `SENSOR_DATA_END_ADDR`: 数据存储结束地址
- 容量: 约 10,240 条记录 (约 5MB)
- 写满后自动回绕覆盖最旧数据

---

## 7. 文件说明

### 7.1 核心文件

#### main.c
**路径**: `Project/src/main.c`  
**行数**: ~1100  
**功能**: 
- 系统初始化 (`systemInit()`)
- FreeRTOS 资源创建（队列、信号量、任务）
- 5 个任务实现：
  - `sensorTask()`: 传感器采样主逻辑
  - `commTask()`: 通信控制主逻辑
  - `storageTask()`: 定时存储任务
  - `monitorTask()`: 监控报警任务
  - `powerTask()`: 电源管理主循环
- 离线检测逻辑 (`gprsSendAllowed()`, `gprsSendFailed()`, `gprsSendSucceeded()`)
- 栈溢出钩子 (`vApplicationStackOverflowHook()`)

#### main.h
**路径**: `Project/inc/main.h`  
**功能**:
- 功能开关宏定义（GPRS_USE_CTR, MQTT_USE_CTR 等）
- 全局变量声明
- FreeRTOS 对象声明（任务句柄、队列、信号量）
- 状态常量定义（MCURUNSTATUS_*, RST_STATUS_* 等）

### 7.2 传感器模块

#### sensor.c / sensor.h
**功能**: 传感器采样核心逻辑
- `cal_sensor()`: 主采样函数，依次采集所有传感器节点
- `readDs1302Time()`: 读取 RTC 时间
- `ds1302tobuf()`: 时间转缓冲区
- `devWaterOutput()`: 水分输出计算
- 全局变量:
  - `sensorValue`: 当前传感器值结构体
  - `curWat[]`: 各节点水分值
  - `curFaw[]`: 各节点盐分值
  - `unSendDataPacket`: 未发送数据包计数

#### ds18b20.c / ds18b20.h
**功能**: DS18B20 温度传感器驱动
- `DS18B20_Start()`: 启动温度转换
- `DS18B20_ReadTemp()`: 读取温度值
- 使用 1-Wire 协议，GPIO 模拟时序

#### w_ads1x15.c / w_ads1x15.h
**功能**: ADS1015 ADC 驱动
- `Read_Value()`: 读取指定通道 ADC 值
- `ADS_MAXVALUE`: ADC 满量程值
- 使用 I2C 接口 (PB8/PB9)

#### stm32_mems_adapter.c / stm32_mems_adapter.h
**功能**: ADXL345 加速度计驱动
- `ADXL345_Init()`: 初始化加速度计
- `ADXL345_Read()`: 读取三轴加速度
- FIFO 配置、中断配置
- 数据格式: `xlResult[]` 数组

#### qingxie.c / qingxie.h
**功能**: 倾角传感器通信
- `Initiate_A_Talk_sensorData()`: 发送命令读取倾角
- `tiltTask()`: 倾角校准任务
- 使用 USART3 通信
- Ymodem 协议支持（固件升级）

#### sk60.c / sk60.h
**功能**: 激光测距传感器
- `sk60Task()`: 激光测量任务
- 与 4G 模块分时复用 USART1

### 7.3 通信模块

#### gprs.c / gprs.h
**路径**: `Project/src/gprs.c`  
**行数**: ~17974  
**功能**: 4G 通信主模块（项目最大文件）
- `gsmTaskInit()`: GSM/4G 模块初始化（AT命令序列）
- `gprsStart()`: 建立 TCP 连接
- `gprsSendRxTask()`: 发送数据并接收响应
- `gprsfrimwareTaskInsentek()`: 固件升级检查
- `sensorDataPacketHead()`: 构造传感器数据包头
- `devRegDataPacketHead()`: 构造设备注册包
- `gprsConfigTask()`: 服务器配置处理
- 数据缓存: `packetHeadBuf[100]`, `sensorBuf[]`
- 大量 AT 命令处理逻辑

#### mqttClient.c / mqttClient.h
**功能**: MQTT 协议客户端
- `mqttserverinit()`: 初始化 MQTT 连接参数
- `mqttpublishsensordata()`: 封装传感器数据为 MQTT 报文
- 基于 paho-embedded-mqtt 库

#### mqttTransport.c / mqttTransport.h
**功能**: MQTT TCP 传输层
- `transport_open()`: 建立 TCP 连接
- `transport_sendPacketBuffer()`: 发送数据包
- `transport_rxTask()`: 接收并解析响应
- 超时重传机制

#### cigemPacket.c / cigemPacket.h
**功能**: 数据包协议封装
- `cigemperiodrunmodeInit()`: 初始化周期运行模式
- `cigemSensorTimeChange()`: 处理服务器下发的周期配置
- 数据序列化/反序列化

#### usart.c / usart.h
**功能**: USART 驱动
- `USART1_Config()`: 4G模块串口 (115200 baud)
- `USART2_Config()`: 调试串口 (115200 baud)
- `USART3_Config()`: 倾角传感器串口
- `usart1SendBuf()`: 发送数据到4G模块
- `usart1StoreBuffer()`: 存储接收到的数据
- ORE 溢出错误处理

### 7.4 存储模块

#### spi_flash.c / spi_flash.h
**功能**: SPI Flash 驱动
- `SPI_FLASH_Init()`: 初始化 SPI 接口
- `SPI_FLASH_BufferRead()`: 读取数据
- `SPI_FLASH_BufferWrite()`: 写入数据
- `SPI_Flash_WAKEUP()`: 从休眠唤醒
- `SPI_Flash_PowerDown()`: 进入低功耗
- 使用 PA5/PA6/PA7 + PB10(CS)

#### mode.c / mode.h
**功能**: 工作模式管理
- `sensorDataWriteAddrStore()`: 存储写入地址到 Flash
- `gprsSendAddrStore()`: 存储发送地址到 Flash
- Flash 地址管理函数
- 设备参数存储/读取

### 7.5 系统模块

#### rtc.c / rtc.h
**功能**: RTC 实时时钟
- `rtcConfigurationJudge()`: RTC 配置判断
- `RTC_WKUP_IRQHandler()`: RTC 唤醒中断
- 使用 STM32 内部 RTC + 外部 DS1302 备份

#### power.c / power.h
**功能**: 电源管理
- `power3PinInit()`: 3.3V 电源控制 GPIO 初始化
- `power331On()`: 打开存储 Flash 电源
- `qxPower33On()`: 打开倾角传感器电源
- `rtcIoLowPower()`: RTC IO 低功耗配置
- `uart2LowPower()`: USART2 低功耗配置

#### delay.c / delay.h
**功能**: 延时函数
- `delay_ms()`: 毫秒延时（调度器运行时自动使用 vTaskDelay）
- `delay_1us()`: 微秒延时（短延时忙等待，长延时自动切换）
- `isSchedulerRunning()`: 查询调度器状态
- `delaySchedulerReady()`: 标记调度器已启动

#### time.c / time.h
**功能**: 时间管理与看门狗
- `getCurrterTime()`: 获取当前时间戳（秒）
- `castTimeAdj()`: 计算时间差
- `samplePeriodAdj()`: 计算采样周期调整值
- `sofeWareWatchDog()`: 软件看门狗（基于 TIM9）
- `sysSoftReset()`: 软件复位

#### stm32l1xx_it.c / stm32l1xx_it.h
**功能**: 中断服务程序
- `HardFault_Handler()`: 硬件故障处理（复位）
- `EXTI4_IRQHandler()`: ADXL345 中断（信号量通知）
- `USART1_IRQHandler()`: 4G 数据接收
- `RTC_WKUP_IRQHandler()`: RTC 唤醒
- `TIM9_IRQHandler()`: 看门狗定时器

#### system_stm32l1xx.c
**功能**: 系统时钟配置
- 配置为 32MHz (HSI 内部时钟)
- 中断向量表偏移 (VECT_TAB_OFFSET=0x3000)

---

## 8. API 参考

### 8.1 传感器 API

```c
// 执行一次完整采样
void cal_sensor(void);

// 读取 DS18B20 温度
float DS18B20_ReadTemp(void);

// 读取 ADS1015 ADC
uint16_t Read_Value(uint8_t channel);

// 读取倾角数据
COM_StatusTypeDef Initiate_A_Talk_sensorData(uint8_t **data);

// 读取 ADXL345
void ADXL345_Read(ADXL345_Data *data);
```

### 8.2 通信 API

```c
// GPRS 初始化
uint8_t gsmTaskInit(uint8_t flag);

// 建立连接
uint8_t gprsStart(void);

// 发送数据并接收响应
void gprsSendRxTask(httpConfig *config);

// 发送数据包
uint16_t gprsSend(uint8_t *buf, uint16_t len, uint8_t type, uint8_t link);

// MQTT 发布
int mqttpublishsensordata(MQTTMessage *msg, uint8_t *buf, 
                          uint8_t dup, uint16_t packetid, char *clientID);
```

### 8.3 存储 API

```c
// 从 Flash 读取传感器数据
uint8_t sensorDataReadToBuf(uint8_t type, uint32_t addr, 
                            uint8_t *buf, uint16_t *len);

// 存储当前数据到 Flash
void sensorDataSave(void);

// 写入地址到 Flash
void sensorDataWriteAddrStore(uint32_t addr);
```

### 8.4 系统 API

```c
// 获取当前时间（秒）
uint32_t getCurrterTime(void);

// 延时（自动兼容 RTOS）
void delay_ms(uint32_t ms);

// 软件复位
void sysSoftReset(void);

// 进入低功耗
void lowPowerEnter(uint32_t sec);
```

---

## 9. 烧录与调试

### 9.1 烧录方式

#### 方式 1: OpenOCD + J-Link/ST-Link

```bash
cd Project/build/gcc

# 烧录固件（自动擦除+烧录+复位）
cmake --build build --target flash

# 仅擦除芯片
cmake --build build --target flash-erase
```

**OpenOCD 配置** (`openocd.cfg`):
```tcl
source [find interface/jlink.cfg]
source [find target/stm32l1.cfg]
```

#### 方式 2: 直接命令行

```bash
openocd -f interface/jlink.cfg -f target/stm32l1.cfg \
  -c "program build/sj321021.bin 0x08003000 verify reset exit"
```

#### 方式 3: STM32CubeProgrammer

```bash
STM32_Programmer_CLI -c port=SWD -w build/sj321021.bin 0x08003000 -v -g
```

### 9.2 调试串口

| 参数 | 值 |
|------|-----|
| 接口 | USART2 (PA2=TX, PA3=RX) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 流控 | 无 |

**常用调试命令** (通过串口发送):
- 查看当前状态
- 手动触发采样
- 修改配置参数
- 查看未发送数据计数

### 9.3 GDB 调试

```bash
# 启动 OpenOCD 服务器
openocd -f interface/jlink.cfg -f target/stm32l1.cfg

# 另一个终端启动 GDB
arm-none-eabi-gdb build/sj321021

# GDB 命令
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

---

## 10. 故障排查

### 10.1 常见问题

#### 问题 1: 设备无法启动

**排查步骤**:
1. 检查电源电压（正常 3.3V，电池 > 3.5V）
2. 检查调试串口输出（115200 baud）
3. 查看 `rstStatus` 值：
   - 2: 按键复位
   - 4: 软件看门狗复位
   - 6: 定时器看门狗复位
   - 8: HardFault
   - 9: MemManage

#### 问题 2: 4G 无法连接

**排查步骤**:
1. 检查 SIM 卡是否插入、是否有流量
2. 检查天线连接
3. 查看调试输出中的 AT 命令交互
4. 检查 `gprsSendAllowed()` 是否返回 false（离线模式）
5. 手动发送 `AT` 测试模块响应

#### 问题 3: 传感器数据异常

**排查步骤**:
1. 检查 5V 电源是否打开（PA15）
2. 检查传感器节点选通 GPIO
3. 查看 `cal_sensor()` 中的调试输出
4. 检查校准参数是否正确

#### 问题 4: 频繁复位

**排查步骤**:
1. 检查电池电压（低电压会导致复位）
2. 查看复位原因 (`rstStatus`)
3. 检查栈溢出（启用 `configCHECK_FOR_STACK_OVERFLOW`）
4. 检查 HardFault 触发条件

### 10.2 调试信息解读

```
# 启动日志示例
boot fire ware ver[1]            # Bootloader 版本正常
rtc init                           # RTC 初始化
rtc init ok                        # RTC 正常
devPeriod:3600 sec                 # 采集周期 1小时
curDevSendPeriod:3600 sec          # 发送周期 1小时
batV:3.85                          # 电池电压 3.85V
sleep 3600 sec                     # 进入休眠 1小时

# 采样日志示例
sensor node 0 cal begin
f:5120.00                          # 频率值
batV:3.85,chargeV:4.12             # 电池和充电电压
send 1 packet                      # 发送 1 个数据包
```

### 10.3 性能指标

| 指标 | 值 | 说明 |
|------|-----|------|
| 启动时间 | ~2s | 从复位到首次采样 |
| 采样周期 | 1s ~ 12h | 可配置 |
| 发送周期 | 1s ~ 12h | 可独立配置 |
| 单次采样时间 | ~5s | 含所有传感器 |
| 4G 连接时间 | ~10s | 从开机到连接服务器 |
| MQTT 发布耗时 | ~2s | 单包数据 |
| 休眠电流 | ~50uA | 典型值 |
| 工作电流 | ~100mA | 含4G模块 |

---

## 附录 A: 数据包格式

### A.1 传感器数据包（上行）

```
字节偏移    字段            长度    说明
─────────────────────────────────────────
0x00       包头标记         2       0x2c, 0x2c
0x02       数据类型         1       0x01=传感器数据
0x03       设备状态         1       
0x04       年              1       
0x05       月              1       
0x06       日              1       
0x07       时              1       
0x08       分              1       
0x09       秒              1       
0x0A       温度            4       float, ℃
0x0E       水分节点1        4       float, %
0x12       水分节点2        4       float, %
...        ...             ...     
0x1E       盐分节点1        4       float, mS/cm
0x22       盐分节点2        4       float, mS/cm
...        ...             ...     
0x2E       倾角X           4       float, °
0x32       倾角Y           4       float, °
0x36       电池电压         4       float, V
0x3A       充电电压         4       float, V
...        ...             ...     
```

### A.2 MQTT 主题格式

```
发布主题: devices/{clientID}/telemetry
订阅主题: devices/{clientID}/commands

示例:
  发布: devices/SJ321021/telemetry
  订阅: devices/SJ321021/commands
```

---

## 附录 B: 引脚定义

| 功能 | 引脚 | 方向 | 说明 |
|------|------|------|------|
| LED | PA1 | 输出 | 高电平亮 |
| 按键 | PA0 | 输入 | 高电平有效，上拉 |
| 调试 TX | PA2 | 输出 | USART2 TX |
| 调试 RX | PA3 | 输入 | USART2 RX |
| SPI SCK | PA5 | 输出 | SPI Flash |
| SPI MISO | PA6 | 输入 | SPI Flash |
| SPI MOSI | PA7 | 输出 | SPI Flash |
| 4G TX | PA9 | 输出 | USART1 TX |
| 4G RX | PA10 | 输入 | USART1 RX |
| 5V 使能 | PA15 | 输出 | 采样前打开 |
| I2C SCL | PB6 | 开漏 | ADXL345 |
| I2C SDA | PB7 | 开漏 | ADXL345 |
| I2C SCL2 | PB8 | 开漏 | ADS1015 |
| I2C SDA2 | PB9 | 开漏 | ADS1015 |
| Flash CS | PB10 | 输出 | SPI Flash 片选 |
| 4G 电源 | PB11 | 输出 | 4G模块电源控制 |
| 倾角 TX | PC10 | 输出 | USART3 TX |
| 倾角 RX | PC11 | 输入 | USART3 RX |
| 倾角电源 | PC12 | 输出 | 倾角传感器电源 |

---

**文档结束**

*如有问题，请提交 Issue 到项目仓库。*
