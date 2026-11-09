#ifndef CAN_ACQ_H
#define CAN_ACQ_H

/*
 * CAN 总线采集（两路从机：冷却泵 / 风机）
 *
 *   请求/响应式采集：每轮先发电压/电流（high）请求，再发流量（normal）请求；
 *   接收侧也先处理 high 帧，电压/电流一收到就立刻做保护判断并下发命令。
 *
 * 线程模型与 acq_thread 一致：
 *   - can_acq 线程：SocketCAN 收发放到后台，快照给 processing / opcua_server
 *   - 打开发失败不退出进程：每 5s 重连，值按 stale_after_ms 置 STALE
 *
 * 命令下发：
 *   - can.tx_enabled=false（默认，硬件未接）时只打印命令帧
 *   - can.tx_enabled=true 时才真正写 SocketCAN
 */

#include "config.h"
#include "acq_thread.h"

#include <stdint.h>

/* 节点故障位，OPC UA 以 UInt32 暴露 */
#define CAN_FAULT_NONE         0u
#define CAN_FAULT_OVERVOLTAGE  (1u << 0)
#define CAN_FAULT_UNDERVOLTAGE (1u << 1)
#define CAN_FAULT_OVERCURRENT  (1u << 2)
#define CAN_FAULT_FLOW_LOW     (1u << 3)
#define CAN_FAULT_COMM_LOST    (1u << 4)
#define CAN_FAULT_SENSOR       (1u << 5)

/* 下发命令码，按 node.cmd_id 组帧：[cmd][param_hi][param_lo] */
typedef enum {
    CAN_CMD_NONE = 0,
    CAN_CMD_REDUCE_LOAD = 1,   /* 降载 */
    CAN_CMD_STOP = 2,          /* 停机保护 */
    CAN_CMD_BOOST = 3,         /* 提速/加冷却 */
    CAN_CMD_CLEAR_FAULT = 4    /* 清故障 */
} can_cmd_t;

typedef struct {
    double   value;
    quality_t quality;
    int      priority;         /* 0=normal 1=high */
    int      node;             /* 所属节点下标 */
    int64_t  ts_mono_us;
    int64_t  ts_utc_us;
    uint32_t seq;              /* 成功解码次数 */
    uint32_t timeouts;         /* 请求发出但本轮没收到响应 */
} can_signal_value_t;

typedef struct {
    char     name[64];
    uint32_t node_id;
    uint32_t fault;            /* CAN_FAULT_* */
    int      alarm_level;      /* 0=正常 1=warning 2=critical */
    int64_t  last_rx_us;
    uint32_t rx_frames;
    uint32_t cmd_seq;
    char     last_cmd[32];
    double   last_cmd_param;
} can_node_value_t;

typedef struct {
    int enabled;
    int link_up;
    int signal_count;
    int node_count;
    can_signal_value_t signals[MAX_CAN_SIGNALS];
    can_node_value_t   nodes[MAX_CAN_NODES];

    uint32_t rx_frames;
    uint32_t rx_high;
    uint32_t rx_normal;
    uint32_t tx_requests;
    uint32_t tx_timeouts;
    uint32_t seq;
    int64_t  last_rx_us;
} can_snapshot_t;

int  can_acq_start(const app_config_t *cfg);
void can_acq_stop(void);
void can_acq_snapshot_get(can_snapshot_t *out);

/* 心跳：距上次 CAN 线程心跳多少毫秒；-1 表示未启动/未启用 */
long long can_acq_heartbeat_age_ms(void);

/*
 * 下发执行命令。
 *   node   : can.nodes[] 下标
 *   cmd    : 命令码
 *   param  : 参数（按 *10 取整后放 2 字节，0~6553.5）
 *   reason : 打印用的触发原因
 * 返回 0 表示已发送或已按 print-only 打印。
 */
int can_cmd_send(int node, can_cmd_t cmd, double param, const char *reason);

const char *can_cmd_name(can_cmd_t cmd);

#endif /* CAN_ACQ_H */
