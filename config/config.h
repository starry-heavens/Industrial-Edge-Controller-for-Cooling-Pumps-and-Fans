#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PT_INT16 = 0, PT_UINT16, PT_INT32, PT_UINT32, PT_FLOAT
} point_type_t;

typedef struct {
    char         name[64];
    uint16_t     addr;
    point_type_t type;
    double       scale;
    char         unit[16];
    double       min;
    double       max;
} point_cfg_t;

#define MAX_POINTS 64

/* ===== 中间处理层（采集 -> 处理 -> OPC）配置 ===== */
#define MAX_ALARM_RULES 16

typedef enum {
    ALARM_COND_NONE = 0,
    ALARM_COND_GT,      /* >  */
    ALARM_COND_GE,      /* >= */
    ALARM_COND_LT,      /* <  */
    ALARM_COND_LE,      /* <= */
    ALARM_COND_STALE    /* 数据失效 */
} alarm_cond_t;

typedef struct {
    char         name[32];
    char         point[64];     /* 点名，"*" 表示任意点 */
    alarm_cond_t cond;
    double       threshold;
    double       hysteresis;    /* 回差，防止阈值附近抖动 */
    int          level;         /* 1=warning 2=critical */
    int          duration_ms;   /* 条件持续多久才置位 */
} alarm_rule_cfg_t;

typedef struct {
    int    window_ms;           /* 特征计算滑窗 */
    int    eval_interval_ms;    /* 评估周期 */
    double vib_class_b;         /* ISO 10816 振动烈度分级边界 mm/s */
    double vib_class_c;
    double vib_class_d;
} processing_cfg_t;

/* OPC UA HistoryRead 的历史缓存（内存环形缓冲） */
typedef struct {
    int enabled;
    int sample_interval_ms;     /* 采样周期 */
    int max_values;             /* 每个节点保留多少个样本 */
} history_cfg_t;

/* ===== CAN 总线采集（冷却泵/风机：流量、电压、电流） ===== */
#define MAX_CAN_NODES   8
#define MAX_CAN_SIGNALS 32

typedef enum {
    CAN_ROLE_NONE = 0,
    CAN_ROLE_FLOW,
    CAN_ROLE_VOLTAGE,
    CAN_ROLE_CURRENT
} can_role_t;

typedef struct {
    char         name[64];
    int          node;          /* 所属节点下标 */
    can_role_t   role;          /* flow/voltage/current，用于处理层按角色取值 */
    uint32_t     req_id;        /* 请求帧 CAN ID，0=只监听不请求 */
    uint32_t     resp_id;       /* 响应帧 CAN ID */
    uint8_t      start_byte;    /* 数据在帧内的起始字节 */
    uint8_t      len;           /* 1/2/4 字节 */
    point_type_t type;
    int          little_endian; /* 0=大端（默认） 1=小端 */
    double       scale;
    double       offset;
    char         unit[16];
    double       min;
    double       max;
    int          priority;      /* 0=normal 1=high（电压/电流用 high） */
    double       warn_hi;       /* 上限报警/保护阈值，0=不启用 */
    double       crit_hi;
    double       warn_lo;       /* 下限报警/保护阈值，0=不启用 */
    double       crit_lo;
} can_signal_cfg_t;

typedef struct {
    char     name[64];
    uint32_t node_id;
    uint32_t cmd_id;            /* 下发命令帧 CAN ID */
} can_node_cfg_t;

typedef struct {
    int    enabled;
    char   device[32];
    int    bitrate;             /* 仅记录/打印，接口速率用 ip link 配置 */
    int    poll_interval_ms;
    int    response_timeout_ms;
    int    retries;
    int    stale_after_ms;
    int    tx_enabled;          /* 0=命令只打印（硬件未接），1=真正写 CAN */

    can_node_cfg_t   nodes[MAX_CAN_NODES];
    int              node_count;
    can_signal_cfg_t signals[MAX_CAN_SIGNALS];
    int              signal_count;
} can_cfg_t;

typedef struct {
    char device[128];
    int  baudrate;
    int  databits;
    int  stopbits;
    char parity[8];
    int  timeout_ms;
    int  inter_frame_delay_ms;
    int  retries;
    int  retry_backoff_ms;

    char mode[16];
    int  de_gpio;
    bool de_active_high;
    int  post_tx_delay_us;

    char dev_name[64];
    int  slave_id;
    int  function;
    int  poll_interval_ms;
    int  stale_after_ms;

    int  opcua_port;

    processing_cfg_t proc;
    alarm_rule_cfg_t alarms[MAX_ALARM_RULES];
    int              alarm_rule_count;
    history_cfg_t    hist;
    can_cfg_t        can;

    point_cfg_t points[MAX_POINTS];
    int         point_count;
} app_config_t;

int  config_load(const char *path, app_config_t *cfg);
void config_dump(const app_config_t *cfg);

#endif