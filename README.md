# 项目概述
## 项目名称
工业边缘控制器（冷却泵/风机）
## 项目概述
本项目基于 ARM 架构 RK3506B 搭建冷却泵/风机状态监测边缘网关：通过 Modbus RTU 轮询振动、温度传感器，通过 CAN-FD 接入流量、电压、电流并按报文 ID 做优先级仲裁；边缘侧完成数据清洗、滑窗特征提取、报警判定与控制字生成，再通过 OPC UA 把振动、温度等数据及告警/控制信息发布给上层监控与控制层；配套日志、心跳、断网补传与 systemd 自愈，保证现场可运维性。
## 项目需求背景
1.项目进程崩溃，保证可以自愈。\
2.保证边缘控制器健康，实时输出日志，保证运维安全。\
3.对采集数据处理完整，数据封装（质量，时间，命令）完整，保证上层执行层执行准确。\
4.多线程不干扰采集线程，保证采集实时性。\
5.执行层结果反馈，保证闭环。\
6.数据有一点的优先级，多条命令不干扰。\
7.本地数据链路断线，保证再连接可以立马自启。\
8.采集线程卡死，保证整个系统不崩溃。\
9.断网后，数据要保留重要信息与命令，启动后数据上传。\
10.保证数据安全。
## 系统架构

```
     振动/温度传感器                     流量计 / 电压 / 电流
           │                                    │
     RS485 / Modbus RTU                  CAN-FD（can0/can1）
           │                                    │
           ▼                                    ▼
   ┌────────────────────────────────────────────────────┐
   │             edge-agent（RK3506B / Linux）           │
   │                                                    │
   │  acq 线程 ──► 采集快照 ──► processing 线程          │
   │  (1s 轮询)                 (100ms tick/500ms 评估)  │
   │                                │                   │
   │                                ├─► 特征/报警/控制字 │
   │                                └─► 历史缓存（1Hz）  │
   │                                                    │
   │  opcua_server：地址空间 + HistoryRead + 方法命令    │
   │  monitor：status.json / 心跳 / 日志                 │
   └─────────────────────────┬──────────────────────────┘
                             │ OPC UA（TCP 4840）
                             ▼
                    上层监控 / 控制层（SCADA / MES）
```
## 核心功能流程

1. **采集**：`acq` 线程同时接两路总线——Modbus RTU 每 1s 轮询振动/温度，CAN-FD 按报文 ID 优先级接收流量/电压/电流；做完 CRC、超时重试和帧重同步后，把原始值、质量、时间戳写入快照；采集线程只负责取数，不参与上传。

2. **处理**：`processing` 线程每 100ms 取一次快照，按 10s 滑窗计算振动 RMS/峰值、温度变化率，再经过"阈值 + 回差 + 持续时间"报警状态机，输出健康度、`ControlWord` 和控制建议。

3. **上传**：`opcua_server` 每 100ms 把原始点、处理结果、心跳写入 OPC UA 地址空间，每 1s 写一条历史；断网期间历史照写，重连后用 HistoryRead 补读。

4. **命令闭环**：上层调用 `ExecuteCommand` → 进入命令队列 → 处理线程异步执行 → 结果通过回调写回 `LastCommand*`（成功 / 拒绝 / 失败 / 超时），客户端用 `RequestId` 对账。

5. **断线处理**：Modbus 单次读超时 120ms、重试 2 次（退避 50ms），单组最坏约 460ms，不阻塞其他线程；连续 3s 无新数据（`stale_after_ms=3000`）质量降为 Bad/Uncertain 并立即触发 `data_stale` 报警，恢复后下一轮（1s）回到 Good。CAN 错误帧/BusOff 由控制器自动恢复，恢复后下一帧继续采集。

6. **断网处理**：OPC UA 断开时本地 1Hz 历史照写，每节点 3600 条 ≈ **1 小时**，重连后 HistoryRead 回读；发布连续 3 轮（≈300ms）失败即点亮控制字 bit4 并扣健康度；命令超时普通 5s、`start_measure` 为窗口时长 +10s；`Heartbeat` 每 100ms +1，建议上位机 3s 不变化判链路断开。

7. **异常兜底**：采集心跳 5s 未更新即退出进程，systemd `RestartSec=2` 拉起（实测 2.12s），`StartLimitBurst=5/60s` 防止秒崩死循环。

> 一句话：采集归采集、处理归处理、上传归上传，命令异步执行并闭环反馈。

## 目录结构

```
config/                 配置（串口、RS485、处理、报警、历史、OPC UA）
edge-agent/             C 源码（采集/处理/OPC UA/监控/日志）
BMS/                    CAN-FD 仿真（python-can）
test/                   Modbus/串口测试程序
opcUA_server/open62541/ open62541 源码
```
##  构建与部署
编译代码
```bash
cmake -S edge-agent -B edge-agent/build \
      -DCMAKE_TOOLCHAIN_FILE=<buildroot-sdk>/toolchainfile.cmake
cmake --build edge-agent/build -j$(nproc)
```
systemd配置
```ini
[Unit]
Description=edge agent
After=network.target

[Service]
ExecStart=/userdata/edge/edge-agent /userdata/edge/config_wt.json
Restart=on-failure
RestartSec=2
StartLimitIntervalSec=60
StartLimitBurst=5

[Install]
WantedBy=multi-user.target
```