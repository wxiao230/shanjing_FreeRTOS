# Technology Stack

**Last updated:** 2026-05-10
**Focus:** tech

## Languages

- **C (C99)** — All application and library source code
- **ARM Assembly** — Startup files (`startup_stm32l1xx_md.s`, `startup_stm32l1xx_mdp.s`)
- **Python** — FreeRTOS kernel internal tooling only (not used by this project)

## Target Hardware

- **MCU:** STM32L151RCT6 (ARM Cortex-M3, 256KB Flash, 32KB RAM)
- **Board:** Custom PCB for geological hazard monitoring

## Toolchain & Build System

| Component | Details |
|-----------|---------|
| **IDE** | Keil MDK-ARM 5.39 (uVision 5) |
| **Compiler** | ARM Compiler 5.06u7 (build 960) — ARMCC/RVCT, **not** GCC/AC6 |
| **Linker Scatter** | `Project/build/MDK-ARM/Objects/Project.sct` |
| **C Library** | microlib (`--library_type=microlib`) |
| **Flash Algorithm** | `STM32L1xx_256.FLM` |
| **Project File** | `Project/build/MDK-ARM/Project.uvprojx` |
| **VS Code IntelliSense** | `Project/c_cpp_properties.json` (GCC-based, not used for build) |

## Linker Layout

- **ROM:** `0x08003000` – `0x0003D000` (244KB, offset for IAP bootloader at `0x08000000-0x08002FFF`)
- **RAM:** `0x20000000` – `0x00008000` (32KB)

## Frameworks & Libraries

| Library | Version | Location | Purpose |
|---------|---------|----------|---------|
| **CMSIS** | 5.3.0 | `Project/lib/Libraries/CMSIS/CORE/` | ARM Cortex-M core access, `core_cm3.h` |
| **STM32L1xx_StdPeriph_Driver** | V1.1.1 | `Project/lib/Libraries/STM32L1xx_StdPeriph_Driver/` | Full STM32L1xx peripheral HAL (GPIO, I2C, SPI, USART, ADC, RTC, DMA, Flash, etc.) |
| **FreeRTOS Kernel** | v10.x | `Project/lib/FreeRTOS/` | Real-time OS kernel, heap_4 allocator, RVDS/ARM_CM3 port |
| **microlib** | Built-in | ARM Compiler 5 | Minimal C runtime (no full printf, no file I/O) |

## FreeRTOS Configuration

| Parameter | Value |
|-----------|-------|
| **Heap Size** | 8 KB (`configTOTAL_HEAP_SIZE`) |
| **Heap Scheme** | heap_4 |
| **Tick Rate** | 1000 Hz (configTICK_RATE_HZ=1000) |
| **Tickless Idle** | Enabled (`configUSE_TICKLESS_IDLE=1`) |
| **Max Priorities** | 5 |
| **CPU Clock** | 32 MHz |

## Project Statistics

- **Source files:** 29 `.c` + 2 `.s` in `Project/src/`
- **Header files:** 32 `.h` in `Project/inc/`
- **Total lines:** ~49,000 across all source
- **Largest files:** `gprs.c` (~12,000 lines), `mode.c` (~5,800 lines), `cigemPacket.c` (~2,400 lines)

## Build Variants

Two startup files selected by preprocessor define:
- `STM32L1XX_MD` — `startup_stm32l1xx_md.s` (medium density)
- `STM32L1XX_MDP` — `startup_stm32l1xx_mdp.s` (medium+ density, current build)
