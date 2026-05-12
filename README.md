# shanjing_FreeRTOS

**Geological Disaster Monitoring Device Firmware**

基于 FreeRTOS 的 STM32L151RCT6 地质灾害监测设备固件。支持多传感器数据采集、4G 通信、MQTT 上报、低功耗管理。

## 仓库地址

https://github.com/wxiao230/shanjing_FreeRTOS

## 硬件平台

| 参数 | 规格 |
|------|------|
| MCU | STM32L151RCT6 (Cortex-M3, 256KB Flash, 32KB RAM) |
| 时钟 | 32MHz (HSI 内部时钟) |
| 通信模块 | SIM7600CE (4G LTE) |
| 存储 | SPI Flash (W25QXX 系列) |
| 调试接口 | USART2 (PA2/PA3), 115200 baud |

## 支持的传感器

| 传感器 | 接口 | 功能 | 可配置 |
|--------|------|------|--------|
| DS18B20 | 1-Wire | 温度测量 | 必选 |
| 土壤水分/盐分 | 频率计数 | 土壤含水量、电导率 | 必选 |
| 倾角传感器 (QX) | USART3 | 倾斜角度 | 可选 |
| ADXL345 | I2C | 三轴加速度、振动监测 | 必选 |
| 激光测距 (SK60) | USART1 | 激光测距 | 可选 |
| ADS1015 | I2C | 电池电压 ADC 转换 | 必选 |
| 孔隙水压力 | - | 仅 MDP 型号 | 可选 |

## 软件架构

基于 FreeRTOS v10.x 的多任务实时操作系统：

| 任务 | 优先级 | 栈大小 | 功能 |
|------|--------|--------|------|
| **Sensor** | 2 | 256 words | 传感器采样 |
| **Comm** | 3 | 384 words | GPRS/MQTT 通信 |
| **Storage** | 1 | 128 words | SPI Flash 定时存储 |
| **Monitor** | 3 | 128 words | ADXL345 中断监控 |
| **Power** | 1 | 128 words | 电池管理、休眠调度 |

**任务间通信**:
- 队列: `xSensorQueue` (Power → Sensor), `xCommQueue` (Sensor → Comm)
- 互斥锁: `xSPIFlashMutex`, `xUSART1Mutex`, `xUSART3Mutex`, `xSensorBufMutex`
- 信号量: `xADXL345Semaphore` (中断通知)

## 快速开始

### 环境要求

| 工具 | 版本 |
|------|------|
| arm-none-eabi-gcc | 15.2.1+ |
| cmake | 3.16+ |
| ninja | 1.10+ |
| OpenOCD | 0.12+ (可选) |

### 构建

```bash
cd Project/build/gcc
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 输出文件
# build/sj321021.bin  - 烧录用二进制文件
# build/sj321021.hex  - 烧录用 HEX 文件
```

### 烧录

```bash
# OpenOCD + J-Link
cmake --build build --target flash

# 或命令行
openocd -f interface/jlink.cfg -f target/stm32l1.cfg \
  -c "program build/sj321021.bin 0x08003000 verify reset exit"
```

### 功能配置

修改 `Project/inc/main.h` 中的宏定义，重新编译即可生成不同功能的固件：

```c
#define GPRS_USE_CTR    2   // 0=无4G, 1=定时发, 2=每次采集发
#define MQTT_USE_CTR    1   // 0=关, 1=开
#define QX_CTR          1   // 0=关, 1=采集, 2=采集+校准
#define SK60_CTR        0   // 0=关, 1=激光测距
```

**配置示例**:

| 场景 | GPRS | MQTT | QX | SK60 |
|------|------|------|-----|------|
| 基础版（仅本地采集） | 0 | 0 | 0 | 0 |
| 标准版（土壤+倾角+MQTT） | 2 | 1 | 1 | 0 |
| 高级版（全功能+远程升级） | 2 | 1 | 2 | 1 |

## 项目文档

| 文档 | 说明 |
|------|------|
| [USER_GUIDE.md](USER_GUIDE.md) | 完整使用说明：架构、文件说明、API、工作流程、故障排查 |
| [REVIEW.md](REVIEW.md) | 代码审查报告：安全漏洞、修复记录 |
| `.planning/ROADMAP.md` | 项目路线图 |

## 目录结构

```
shanjing_FreeRTOS/
├── Project/
│   ├── inc/           # 头文件 (25个)
│   │   ├── main.h     # 功能配置宏
│   │   ├── FreeRTOSConfig.h
│   │   └── ...
│   ├── src/           # 源文件 (28个)
│   │   ├── main.c     # 主程序、任务创建
│   │   ├── gprs.c     # 4G通信 (17974行)
│   │   ├── sensor.c   # 传感器采样
│   │   └── ...
│   ├── lib/           # FreeRTOS + STM32标准外设库
│   └── build/gcc/     # 构建目录
├── USER_GUIDE.md      # 使用说明
├── REVIEW.md          # 代码审查
└── .planning/         # 项目规划
```

## 内存占用

```
FLASH: 139,068 bytes / 256KB (54.3%)
RAM:   20,616 bytes  / 32KB  (64.9%)
Heap:  8KB (FreeRTOS heap_4)
```

## 通信协议

- **MQTT**: 通过 4G TCP 连接，主题格式 `devices/{clientID}/telemetry`
- **私有协议**: 二进制数据包，支持设备注册、传感器数据、配置请求
- **数据存储**: SPI Flash 环形缓冲区，512字节/条，约 10,240 条容量

## 低功耗管理

- **休眠模式**: STOP 模式，RTC 定时唤醒
- **休眠电流**: ~50uA
- **工作电流**: ~100mA (含 4G 模块)
- **自动休眠**: 电池低电压时自动进入休眠

## 安全特性

- 栈溢出检测 (`configCHECK_FOR_STACK_OVERFLOW=2`)
- `configASSERT` 宏定义
- 缓冲区溢出保护 (safe_sprintf)
- 离线检测与退避机制 (连续3次失败后休眠5分钟)

## 许可证

本项目基于 STM32 标准外设库和 FreeRTOS，遵循相应许可证。

## 作者

wxiao230

## 相关链接

- [FreeRTOS 官网](https://www.freertos.org/)
- [STM32L1xx 参考手册](https://www.st.com/en/microcontrollers-microprocessors/stm32l151rc.html)
