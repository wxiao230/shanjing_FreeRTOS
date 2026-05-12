# Phase 3 Summary: 断网缓存与补发 — 完成

## 修复与新增

### Fix 1: GPRS TIME_ERROR 不再跳过记录 (gprs.c:12185)
- 删除 `curGprsSendAddr += SENSOR_DATA_STORE_SIZE` — 不再跳过失败记录
- 删除 `unSendDataPacket--` — 计数器不递减
- 失败记录留在 Flash 缓存中，下次连接时重试

### Fix 2: 离线检测与退避 (main.c)
- `gprsSendAllowed()` — 检查离线标志；退避期满后自动恢复
- `gprsSendFailed()` — 递增连续失败计数；`>=3` 时进入离线模式（退避5分钟）
- `gprsSendSucceeded()` — 重置计数和标志
- `COMM_CMD_SEND` 和 `COMM_CMD_SEND_LATEST` 均集成离线检查

## 缓存机制验证

| 场景 | 行为 | 状态 |
|------|------|------|
| 网络正常 | GPRS连接→发送→断开 | ✅ |
| 网络断连（GPRS连不上） | `gsmTaskInit` 失败→数据留在Flash→退避后重试 | ✅ |
| 网络断连（GPRS连上但发不出） | `gprsSend` 失败→GPRS会话结束→下次重连重试 | ✅ |
| 服务器TIME_ERROR | 记录不跳过→下次重试→直到成功或被覆盖 | ✅ 修复 |
| 缓存满 | 环形缓冲区覆盖最旧数据 | ✅ 已有 |
| MQTT发送失败 | `transport_sendPacketBuffer` 失败→continue重试 | ✅ 已有 |

## 参数
- `GPRS_OFFLINE_MAX_RETRY` = 3
- `GPRS_OFFLINE_BACKOFF_SEC` = 300 (5分钟)

## 文件变更
| 文件 | 变更 |
|------|------|
| `gprs.c:12185-12198` | 删除 TIME_ERROR 中的指针推进和计数器递减 |
| `main.c:74-110` | 新增离线检测变量和三个 helper 函数 |
| `main.c:456-475` | COMM_CMD_SEND 集成离线检测 |
| `main.c:557-605` | COMM_CMD_SEND_LATEST 集成离线检测 |
