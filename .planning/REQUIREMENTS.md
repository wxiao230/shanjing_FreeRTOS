# 需求定义

## v1 需求

### 移植验证 (VERIFY)

- [ ] **VERIFY-01**: 逐源文件对比裸机版和 FreeRTOS 版，确认 29 个 `.c` 文件全部移植
- [ ] **VERIFY-02**: 验证每个传感器驱动 (DS18B20, 土壤水分, 倾角, ADXL345, ADS1015, SK60) 功能完整
- [ ] **VERIFY-03**: 验证 GPRS/MQTT 通信栈 (gprs.c, mqttClient.c, mqttTransport.c) 功能完整
- [ ] **VERIFY-04**: 确认 FreeRTOS 架构合规 — 任务化、队列通信、信号量、互斥锁正确使用
- [ ] **VERIFY-05**: 确认中断处理正确 — ISR 极简、信号量 defer 到任务、临界区保护
- [x] **VERIFY-06**: 确认 SPI Flash 存储/读取逻辑正确 — W25X64 (8MB) 已验证，寻址无溢出
- [ ] **VERIFY-07**: 确认低功耗管理 (tickless idle, RTC 唤醒, 外设关断) 正常

### 采集/发送周期 (PERIOD)

- [x] **PERIOD-01**: 采集周期 1s~12h 可配置（步进 1s）— `curDevPeriod` + MQTT setsensortime
- [x] **PERIOD-02**: 发送周期 1s~12h 可配置，独立于采集周期 — `curDevSendPeriod`
- [x] **PERIOD-03**: 当发送周期 < 采集周期时，按发送周期发送最新的已有数据 — COMM_CMD_SEND_LATEST
- [x] **PERIOD-04**: 发送数据包含最新采集值 + 时间戳，缺失参数可标记为无效 — sensorValue + time
- [x] **PERIOD-05**: 采集周期和发送周期通过 MQTT 远程可配置（现有平台命令兼容）— upload_intv → curDevSendPeriod

### 断网缓存与补发 (CACHE)

- [x] **CACHE-01**: 断网时按发送周期缓存原始数据（传感器值 + 时间戳）— sensorDataSave() 始终写入Flash
- [x] **CACHE-02**: 缓存数据按 FIFO 顺序存储在 SPI Flash 中 — FlashWrite() 环形缓冲区
- [x] **CACHE-03**: 联网成功后自动按 FIFO 顺序补发缓存数据 — curGprsSendAddr / curMqttSendAddr
- [x] **CACHE-04**: 补发完成后恢复正常实时发送 — unSendDataPacket==0 时切换
- [x] **CACHE-05**: 缓存满时丢弃最旧数据，保证最新数据不丢 — FlashWrite() 自动回绕
- [x] **CACHE-06**: 支持 1s 发送周期场景 — 每分钟 60 包上传，不丢包 — Phase2+Phase3

## v2 需求（延迟）

- 动态调整缓存深度（根据可用 Flash 空间自适应）
- 压缩缓存数据以提高存储效率
- 按优先级缓存（报警数据优先发送）

## 超出范围

- 更换 MCU 平台 — 固定在 STM32L151RC
- 增加新传感器类型 — 仅支持现有传感器
- 云端平台开发
- 通信协议更换

---

*更新日期：2026-05-10*
