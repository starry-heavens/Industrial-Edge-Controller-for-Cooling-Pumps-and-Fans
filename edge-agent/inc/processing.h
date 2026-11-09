#ifndef PROCESSING_H
#define PROCESSING_H

/*
 * 中间处理层（数据 -> 控制信息）
 *
 *   采集线程(Modbus RTU) --快照--> processing 线程 --派生量/报警/控制字--> OPC UA
 *                                     ^                              |
 *                                     +------ 传输结果回调 ----------+
 *
 * 这里不做硬实时控制，只做“慢回路”的二次处理：
 *   - 振动速度/加速度滑窗特征（RMS、峰值）
 *   - 温度变化率
 *   - 报警状态机（阈值 + 回差 + 持续时间）
 *   - 振动烈度分级与健康度
 *   - 控制字 / 建议动作（给上位机或 PLC 做非实时联锁）
 *   - 异步处理命令：启动测量窗口、报警确认、自检
 */

#include "config.h"

#include <stdint.h>

/* 控制字位定义，OPC UA 上以 UInt32 暴露 */
#define PROC_CTL_ALARM_ACTIVE   (1u << 0)   /* 有报警 */
#define PROC_CTL_COOLING_DEMAND (1u << 1)   /* 需要加强冷却 */
#define PROC_CTL_SENSOR_FAULT   (1u << 2)   /* 采集链路异常 */
#define PROC_CTL_MEASURE_RUNNING (1u << 3)  /* 测量窗口进行中 */
#define PROC_CTL_UPLOAD_FAULT   (1u << 4)   /* OPC 上传连续失败 */
#define PROC_CTL_CAN_FAULT      (1u << 5)   /* CAN 节点故障（含通信/流量） */
#define PROC_CTL_CAN_PROTECT    (1u << 6)   /* CAN 电压/电流保护动作 */

typedef enum {
    PROC_CMD_NONE = 0,
    PROC_CMD_START_MEASURE,   /* 启动 N 秒振动测量窗口（非实时命令） */
    PROC_CMD_ACK_ALARM,       /* 报警确认，不强制清除报警 */
    PROC_CMD_SELF_TEST        /* 采集链路自检 */
} proc_cmd_t;

typedef enum {
    PROC_CMD_STATE_PENDING = 0,
    PROC_CMD_STATE_RUNNING,   /* 已接受，后台执行中（如测量窗口） */
    PROC_CMD_STATE_DONE,
    PROC_CMD_STATE_REJECTED,
    PROC_CMD_STATE_FAILED
} proc_cmd_state_t;

typedef struct {
    uint32_t         id;
    proc_cmd_t       cmd;
    proc_cmd_state_t state;
    int32_t          code;          /* 0=成功，负值=失败原因 */
    int64_t          ts_utc_us;
    char             message[96];
} proc_cmd_result_t;

/* CAN 从机（冷却泵/风机）的处理结果 */
typedef struct {
    char     name[64];
    double   flow;
    double   voltage;
    double   current;
    int      quality;           /* quality_t，取该节点最差信号 */
    uint32_t fault;             /* CAN_FAULT_* */
    int      alarm_level;       /* 0/1/2 */
    uint32_t cmd_seq;
    char     last_cmd[32];
    double   last_cmd_param;
} proc_can_node_t;

typedef struct {
    /* 振动速度特征（mm/s） */
    double vib_rms[3];
    double vib_peak[3];
    /* 加速度特征（g） */
    double acc_rms[3];
    /* 温度 */
    double temp_c;
    double temp_rate_c_s;
    /* 评估结果 */
    double severity;            /* 三轴速度 RMS 最大值 */
    int    severity_class;      /* 1=A 2=B 3=C 4=D */
    int    health;              /* 0..100 */
    int    bad_points;          /* 质量非 GOOD 的点数 */
    /* 报警 */
    int      alarm_level;       /* 0=无 1=warning 2=critical */
    int      alarm_active_count;
    uint32_t alarm_bits;        /* 按规则序号置位，最多 32 条 */
    char     active_alarms[128];
    /* 控制输出 */
    uint32_t control_word;
    char     control_action[32];
    /* 测量窗口 */
    int      measure_state;     /* 0=idle 1=running 2=done */
    double   measure_remaining_s;
    double   measure_rms;
    double   measure_peak;
    /* 最近一次命令执行结果 */
    proc_cmd_result_t last_cmd;
    /* OPC 传输结果（由 opcua_server 回调写入） */
    uint32_t upload_seq;
    uint32_t upload_fail_total;
    uint32_t upload_fail_streak;
    /* CAN 冷却泵/风机 */
    proc_can_node_t can[MAX_CAN_NODES];
    int      can_node_count;
    uint32_t can_fault_bits;
    int      can_alarm_level;
    int      can_link_up;
    uint32_t can_rx_frames;
    uint32_t can_rx_high;
    uint32_t can_tx_requests;
    uint32_t can_tx_timeouts;
    uint32_t can_cmd_seq;
    char     can_cmd_node[64];
    char     can_cmd_name[32];
    double   can_cmd_param;
    /* 元信息 */
    uint32_t eval_seq;
    int64_t  ts_mono_us;
    int64_t  ts_utc_us;
} proc_snapshot_t;

typedef void (*proc_cmd_result_cb_t)(const proc_cmd_result_t *r, void *user);

int  proc_start(const app_config_t *cfg);
void proc_stop(void);

/* 线程安全地读取处理后快照 */
void proc_snapshot_get(proc_snapshot_t *out);

/* OPC 每次发布完成后回调，用于把“传输结果”并入控制逻辑 */
void proc_note_transfer(uint32_t seq, uint32_t ok, uint32_t failed);

/* 提交一条处理命令，返回请求 id；0 表示队列满/参数非法 */
uint32_t proc_cmd_submit(proc_cmd_t cmd, double param);

/* 命令执行完成时回调（在 processing 线程上下文调用，回调里不要阻塞） */
void proc_set_cmd_result_cb(proc_cmd_result_cb_t cb, void *user);

const char *proc_cmd_name(proc_cmd_t cmd);
const char *proc_cmd_state_name(proc_cmd_state_t s);

#endif /* PROCESSING_H */
