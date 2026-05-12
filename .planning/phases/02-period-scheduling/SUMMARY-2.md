# Phase 2 完成总结: 采集/发送周期调度

## 状态: ✅ 已完成 (代码已存在)

所有 Wave 1-4 的实现在初始 FreeRTOS 移植中已完成，无需新增代码。

## 验证结果

- **构建**: 通过 (ninja: no work to do)
- **代码位置**: `Project/src/main.c`

## 实现覆盖

### Wave 1: 独立发送周期变量
- `main.h:196` — `extern __IO uint32_t curDevSendPeriod;`
- `main.c:117` — `__IO uint32_t curDevSendPeriod = 3600;` (默认1h)

### Wave 2: powerTask 发送定时逻辑
- `main.c:768-776` — 当 `curDevSendPeriod < curDevPeriod` 时，独立触发发送：
  ```c
  if(curDevSendPeriod < curDevPeriod) {
      uint32_t sendElapsed = castTimeAdj(lastSendTime);
      if((sendElapsed >= curDevSendPeriod) && (mcuPowerStatus == MCU_OFF_STATE)) {
          lastSendTime = getCurrterTime();
          xQueueSend(xCommQueue, &(uint8_t){COMM_CMD_SEND_LATEST}, 0);
      }
  }
  ```

### Wave 3: COMM_CMD_SEND_LATEST handler
- `main.c:559-613` — 轻量级 MQTT publish-only 命令：
  - GPRS 初始化 → MQTT 连接 → publish `sensorValue` → 断开
  - 不复用持久连接，每次发送完即断开（功耗由外部周期配置负责）

### Wave 4: MQTT 远程配置
- `cigemPacket.c:211` — `curDevSendPeriod = cigemsensortime.upload;`
- `cigemPacket.c:237` — `curDevSendPeriod = uploadPeriod;`
- 下行 `setsensortime` 命令的 `upload_intv` 字段直接映射到 `curDevSendPeriod`

## 关键设计决策

1. **发送时同步更新 `lastSendTime`** (line 763, 773) — 避免采样→发送流程中重复触发
2. **COMM_CMD_SEND_LATEST 是轻量 publish-only** — 不读 Flash，只发当前 `sensorValue`
3. **发送后断开 GPRS** — 功耗由周期配置控制，非持久连接
4. **transmissionDensity 逻辑保留** — 原有下行兼容，但 `curDevSendPeriod` 独立控制发送频率

## 已废弃决策

- 无新增内容

## 风险

- **无** — 代码已在初始移植中实现，与 Phase 1 一起验证构建

## 下一步

Phase 3: offline cache retry 机制 — 处理 GPRS 断线时的数据缓存与重传策略。
