#define _GNU_SOURCE

#include "opcua_server.h"
#include "acq_thread.h"
#include "can_acq.h"
#include "processing.h"
#include "log.h"

#include "open62541.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* -------- 处理结果节点（OPC UA 上的“控制信息”） -------- */
enum {
    PN_VIB_RMS_X = 0,
    PN_VIB_RMS_Y,
    PN_VIB_RMS_Z,
    PN_VIB_PEAK_X,
    PN_VIB_PEAK_Y,
    PN_VIB_PEAK_Z,
    PN_ACC_RMS_X,
    PN_ACC_RMS_Y,
    PN_ACC_RMS_Z,
    PN_TEMP_C,
    PN_TEMP_RATE,
    PN_SEVERITY,
    PN_SEVERITY_CLASS,
    PN_HEALTH,
    PN_BAD_POINTS,
    PN_ALARM_LEVEL,
    PN_ALARM_COUNT,
    PN_ALARM_BITS,
    PN_ACTIVE_ALARMS,
    PN_CONTROL_WORD,
    PN_CONTROL_ACTION,
    PN_MEASURE_STATE,
    PN_MEASURE_REMAIN,
    PN_MEASURE_RMS,
    PN_MEASURE_PEAK,
    PN_CMD_ID,
    PN_CMD_NAME,
    PN_CMD_STATE,
    PN_CMD_CODE,
    PN_CMD_SUCCESS,
    PN_CMD_MESSAGE,
    PN_PUB_SEQ,
    PN_PUB_FAILED,
    PN_UPLOAD_FAIL_TOTAL,
    PN_UPLOAD_FAIL_STREAK,
    PN_HEARTBEAT,
    PN_UPTIME,
    PN_ACQ_SEQ,
    PN_EVAL_SEQ,
    PN_HIST_SEQ,
    PN_HIST_MAX,
    PN_HIST_ENABLED,
    PN_CAN_LINK_UP,
    PN_CAN_FAULT_BITS,
    PN_CAN_ALARM_LEVEL,
    PN_CAN_RX_FRAMES,
    PN_CAN_RX_HIGH,
    PN_CAN_TX_REQUESTS,
    PN_CAN_TX_TIMEOUTS,
    PN_CAN_CMD_NODE,
    PN_CAN_CMD_NAME,
    PN_CAN_CMD_PARAM,
    PN_CAN_CMD_SEQ,
    PN_COUNT
};

static const char *const g_proc_names[PN_COUNT] = {
    [PN_VIB_RMS_X]          = "VibRmsX",
    [PN_VIB_RMS_Y]          = "VibRmsY",
    [PN_VIB_RMS_Z]          = "VibRmsZ",
    [PN_VIB_PEAK_X]         = "VibPeakX",
    [PN_VIB_PEAK_Y]         = "VibPeakY",
    [PN_VIB_PEAK_Z]         = "VibPeakZ",
    [PN_ACC_RMS_X]          = "AccRmsX",
    [PN_ACC_RMS_Y]          = "AccRmsY",
    [PN_ACC_RMS_Z]          = "AccRmsZ",
    [PN_TEMP_C]             = "TempC",
    [PN_TEMP_RATE]          = "TempRate",
    [PN_SEVERITY]           = "VibrationSeverity",
    [PN_SEVERITY_CLASS]     = "SeverityClass",
    [PN_HEALTH]             = "HealthIndex",
    [PN_BAD_POINTS]         = "BadPointCount",
    [PN_ALARM_LEVEL]        = "AlarmLevel",
    [PN_ALARM_COUNT]        = "ActiveAlarmCount",
    [PN_ALARM_BITS]         = "AlarmBits",
    [PN_ACTIVE_ALARMS]      = "ActiveAlarms",
    [PN_CONTROL_WORD]       = "ControlWord",
    [PN_CONTROL_ACTION]     = "ControlAction",
    [PN_MEASURE_STATE]      = "MeasureState",
    [PN_MEASURE_REMAIN]     = "MeasureRemainingS",
    [PN_MEASURE_RMS]        = "MeasureRms",
    [PN_MEASURE_PEAK]       = "MeasurePeak",
    [PN_CMD_ID]             = "LastCommandId",
    [PN_CMD_NAME]           = "LastCommandName",
    [PN_CMD_STATE]          = "LastCommandState",
    [PN_CMD_CODE]           = "LastCommandCode",
    [PN_CMD_SUCCESS]        = "LastCommandSuccess",
    [PN_CMD_MESSAGE]        = "LastCommandMessage",
    [PN_PUB_SEQ]            = "PublishSeq",
    [PN_PUB_FAILED]         = "PublishFailed",
    [PN_UPLOAD_FAIL_TOTAL]  = "UploadFailTotal",
    [PN_UPLOAD_FAIL_STREAK] = "UploadFailStreak",
    [PN_HEARTBEAT]          = "Heartbeat",
    [PN_UPTIME]             = "UptimeS",
    [PN_ACQ_SEQ]            = "AcqSeq",
    [PN_EVAL_SEQ]           = "EvalSeq",
    [PN_HIST_SEQ]           = "HistorySampleSeq",
    [PN_HIST_MAX]           = "HistoryMaxValues",
    [PN_HIST_ENABLED]       = "HistoryEnabled",
    [PN_CAN_LINK_UP]        = "CanLinkUp",
    [PN_CAN_FAULT_BITS]     = "CanFaultBits",
    [PN_CAN_ALARM_LEVEL]    = "CanAlarmLevel",
    [PN_CAN_RX_FRAMES]      = "CanRxFrames",
    [PN_CAN_RX_HIGH]        = "CanHighPriorityRx",
    [PN_CAN_TX_REQUESTS]    = "CanTxRequests",
    [PN_CAN_TX_TIMEOUTS]    = "CanTxTimeouts",
    [PN_CAN_CMD_NODE]       = "CanLastCommandNode",
    [PN_CAN_CMD_NAME]       = "CanLastCommandName",
    [PN_CAN_CMD_PARAM]      = "CanLastCommandParam",
    [PN_CAN_CMD_SEQ]        = "CanLastCommandSeq",
};

static UA_Server         *g_server = NULL;
static pthread_t          g_tid;
static volatile int       g_run   = 0;
static const app_config_t *g_cfg  = NULL;

static opcua_transfer_cb_t g_transfer_cb   = NULL;
static void               *g_transfer_user = NULL;

static pthread_mutex_t    g_cmd_mu = PTHREAD_MUTEX_INITIALIZER;
static proc_cmd_result_t  g_cmd_result;
static int                g_cmd_dirty = 0;

static uint32_t           g_pub_seq = 0;
static UA_UInt16          g_ns      = 1;

/* ---- OPC UA HistoryRead 的历史缓存（内存环形缓冲） ---- */
#define HIST_MAX_NODES  32

typedef struct {
    int kind;     /* 0=处理后节点 1=Modbus 原始点 2=CAN 原始信号 */
    int idx;
    int type;     /* 0=Double 1=Int32 2=UInt32，必须与节点 DataType 一致 */
} hist_node_t;

static UA_HistoryDataBackend   g_hist_backend;
static UA_HistoryDataGathering g_hist_gathering;
static hist_node_t             g_hist_nodes[HIST_MAX_NODES];
static int                     g_hist_node_count = 0;
static int                     g_hist_enabled    = 0;
static int64_t                 g_hist_last_us    = 0;
static int64_t                 g_hist_last_ts    = 0;
static uint32_t                g_hist_seq        = 0;
static uint32_t                g_heartbeat       = 0;
static int64_t                 g_start_us        = 0;

/* 已接受、但还没等到最终结果的命令（防止处理线程卡死时客户端永远等不到结果） */
static uint32_t           g_pending_id          = 0;
static proc_cmd_t         g_pending_cmd         = PROC_CMD_NONE;
static int64_t            g_pending_deadline_us = 0;

static int64_t now_mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static int64_t now_utc_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

/* OPC UA DateTime: 1601-01-01 起的 100ns */
static UA_DateTime ua_ts(int64_t utc_us)
{
    if (utc_us <= 0) return UA_DateTime_now();
    return (UA_DateTime)(utc_us * 10LL + 116444736000000000LL);
}

static UA_NodeId node_id(const char *s)
{
    return UA_NODEID_STRING(g_ns, (char *)(uintptr_t)s);
}

static UA_NodeId proc_id(int idx)
{
    return node_id(g_proc_names[idx]);
}

/* 写一个标量值。源数据在栈上，写完后不能再 clear */
static UA_StatusCode write_scalar(UA_NodeId id, void *data,
                                  const UA_DataType *type,
                                  UA_DateTime ts, UA_StatusCode status)
{
    UA_DataValue dv;
    UA_DataValue_init(&dv);

    dv.hasValue = true;
    UA_Variant_setScalar(&dv.value, data, type);
    dv.value.storageType = UA_VARIANT_DATA_NODELETE;

    dv.hasSourceTimestamp = true;
    dv.sourceTimestamp    = ts;

    dv.hasStatus = true;
    dv.status    = status;

    return UA_Server_writeDataValue(g_server, id, dv);
}

/* ---------- 建节点 ---------- */

static void add_folder(const char *id, const char *name, UA_NodeId parent)
{
    UA_ObjectAttributes attr = UA_ObjectAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)name);

    UA_StatusCode st = UA_Server_addObjectNode(
        g_server, node_id(id), parent,
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(g_ns, (char *)name),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        attr, NULL, NULL);

    if (st != UA_STATUSCODE_GOOD)
        fprintf(stderr, "[opcua] add folder %s failed 0x%08x\n", name, st);
}

static void add_var(UA_NodeId parent, const char *id, const char *name,
                    const UA_DataType *type, void *initial)
{
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)name);
    attr.accessLevel = UA_ACCESSLEVELMASK_READ;
    attr.dataType    = type->typeId;
    UA_Variant_setScalar(&attr.value, initial, type);

    UA_StatusCode st = UA_Server_addVariableNode(
        g_server, node_id(id), parent,
        UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
        UA_QUALIFIEDNAME(g_ns, (char *)name),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr, NULL, NULL);

    if (st != UA_STATUSCODE_GOOD)
        fprintf(stderr, "[opcua] add var %s failed 0x%08x\n", name, st);
}

static void add_double(UA_NodeId parent, const char *id, const char *name, double init)
{
    UA_Double v = init;
    add_var(parent, id, name, &UA_TYPES[UA_TYPES_DOUBLE], &v);
}

static void add_int32(UA_NodeId parent, const char *id, const char *name, UA_Int32 init)
{
    UA_Int32 v = init;
    add_var(parent, id, name, &UA_TYPES[UA_TYPES_INT32], &v);
}

static void add_uint32(UA_NodeId parent, const char *id, const char *name, UA_UInt32 init)
{
    UA_UInt32 v = init;
    add_var(parent, id, name, &UA_TYPES[UA_TYPES_UINT32], &v);
}

static void add_bool(UA_NodeId parent, const char *id, const char *name, UA_Boolean init)
{
    UA_Boolean v = init;
    add_var(parent, id, name, &UA_TYPES[UA_TYPES_BOOLEAN], &v);
}

static void add_string(UA_NodeId parent, const char *id, const char *name,
                       const char *init)
{
    UA_String s;
    s.length = strlen(init);
    s.data   = (UA_Byte *)(uintptr_t)init;
    add_var(parent, id, name, &UA_TYPES[UA_TYPES_STRING], &s);
}

/* ---------- 命令方法 ---------- */

static UA_StatusCode method_execute_command(
    UA_Server *server, const UA_NodeId *sessionId, void *sessionContext,
    const UA_NodeId *methodId, void *methodContext,
    const UA_NodeId *objectId, void *objectContext,
    size_t inputSize, const UA_Variant *input,
    size_t outputSize, UA_Variant *output)
{
    (void)server; (void)sessionId; (void)sessionContext;
    (void)methodId; (void)methodContext; (void)objectId; (void)objectContext;

    char     cmd_buf[32] = {0};
    char     msg[160];
    uint32_t req_id  = 0;
    UA_Boolean accepted = false;
    double   param   = 0.0;
    proc_cmd_t cmd   = PROC_CMD_NONE;

    snprintf(msg, sizeof(msg), "invalid command");

    if (inputSize < 1 ||
        !UA_Variant_hasScalarType(&input[0], &UA_TYPES[UA_TYPES_STRING])) {
        snprintf(msg, sizeof(msg), "input[0] 'Command' must be a String");
        goto finish;
    }

    {
        const UA_String *cs = (const UA_String *)input[0].data;
        size_t n = cs->length < sizeof(cmd_buf) - 1 ? cs->length : sizeof(cmd_buf) - 1;
        if (cs->data && n > 0) memcpy(cmd_buf, cs->data, n);
        cmd_buf[n] = '\0';
    }

    if (inputSize >= 2 &&
        UA_Variant_hasScalarType(&input[1], &UA_TYPES[UA_TYPES_DOUBLE]))
        param = *(const UA_Double *)input[1].data;

    if (strcmp(cmd_buf, "start_measure") == 0)      cmd = PROC_CMD_START_MEASURE;
    else if (strcmp(cmd_buf, "ack_alarm") == 0)     cmd = PROC_CMD_ACK_ALARM;
    else if (strcmp(cmd_buf, "self_test") == 0)     cmd = PROC_CMD_SELF_TEST;

    if (cmd == PROC_CMD_NONE) {
        snprintf(msg, sizeof(msg), "unknown command '%s'", cmd_buf);
        goto finish;
    }

    /* 登记看门狗和入队放在同一把锁里，避免执行结果先于登记到达 */
    pthread_mutex_lock(&g_cmd_mu);
    req_id = proc_cmd_submit(cmd, param);
    if (req_id != 0) {
        int64_t timeout_us;
        if (cmd == PROC_CMD_START_MEASURE) {
            double sec = param > 0 ? param : 10.0;
            if (sec < 1)  sec = 1;
            if (sec > 60) sec = 60;
            timeout_us = (int64_t)(sec * 1e6) + 10 * 1000000LL;
        } else {
            timeout_us = 5 * 1000000LL;
        }

        g_pending_id          = req_id;
        g_pending_cmd         = cmd;
        g_pending_deadline_us = now_mono_us() + timeout_us;
    }
    pthread_mutex_unlock(&g_cmd_mu);

    if (req_id == 0) {
        snprintf(msg, sizeof(msg), "command queue full or invalid parameter");
    } else {
        accepted = true;
        snprintf(msg, sizeof(msg), "accepted: %s id=%u", proc_cmd_name(cmd), req_id);
    }

finish:
    if (outputSize >= 1) {
        UA_UInt32 v = req_id;
        UA_Variant_setScalarCopy(&output[0], &v, &UA_TYPES[UA_TYPES_UINT32]);
    }
    if (outputSize >= 2) {
        UA_Boolean v = accepted;
        UA_Variant_setScalarCopy(&output[1], &v, &UA_TYPES[UA_TYPES_BOOLEAN]);
    }
    if (outputSize >= 3) {
        UA_String s;
        s.length = strlen(msg);
        s.data   = (UA_Byte *)(uintptr_t)msg;
        UA_Variant_setScalarCopy(&output[2], &s, &UA_TYPES[UA_TYPES_STRING]);
    }
    return UA_STATUSCODE_GOOD;
}

static void add_command_method(UA_NodeId parent)
{
    UA_Argument in[2];
    UA_Argument_init(&in[0]);
    in[0].description = UA_LOCALIZEDTEXT((char *)"zh-CN",
        (char *)"start_measure | ack_alarm | self_test");
    in[0].name      = UA_STRING((char *)"Command");
    in[0].dataType  = UA_TYPES[UA_TYPES_STRING].typeId;
    in[0].valueRank = UA_VALUERANK_SCALAR;

    UA_Argument_init(&in[1]);
    in[1].description = UA_LOCALIZEDTEXT((char *)"zh-CN",
        (char *)"命令参数，例如测量窗口秒数");
    in[1].name      = UA_STRING((char *)"Param");
    in[1].dataType  = UA_TYPES[UA_TYPES_DOUBLE].typeId;
    in[1].valueRank = UA_VALUERANK_SCALAR;

    UA_Argument out[3];
    UA_Argument_init(&out[0]);
    out[0].description = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)"请求编号");
    out[0].name      = UA_STRING((char *)"RequestId");
    out[0].dataType  = UA_TYPES[UA_TYPES_UINT32].typeId;
    out[0].valueRank = UA_VALUERANK_SCALAR;

    UA_Argument_init(&out[1]);
    out[1].description = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)"是否已接受");
    out[1].name      = UA_STRING((char *)"Accepted");
    out[1].dataType  = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
    out[1].valueRank = UA_VALUERANK_SCALAR;

    UA_Argument_init(&out[2]);
    out[2].description = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)"结果描述");
    out[2].name      = UA_STRING((char *)"Message");
    out[2].dataType  = UA_TYPES[UA_TYPES_STRING].typeId;
    out[2].valueRank = UA_VALUERANK_SCALAR;

    UA_MethodAttributes attr = UA_MethodAttributes_default;
    attr.displayName    = UA_LOCALIZEDTEXT((char *)"zh-CN", (char *)"ExecuteCommand");
    attr.description    = UA_LOCALIZEDTEXT((char *)"zh-CN",
        (char *)"非实时处理命令：测量窗口 / 报警确认 / 自检");
    attr.executable     = true;
    attr.userExecutable = true;

    UA_StatusCode st = UA_Server_addMethodNode(
        g_server, node_id("ExecuteCommand"), parent,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(g_ns, (char *)"ExecuteCommand"),
        attr, method_execute_command,
        2, in, 3, out, NULL, NULL);

    if (st != UA_STATUSCODE_GOOD)
        fprintf(stderr, "[opcua] add method failed 0x%08x\n", st);
}

static void create_nodes(void)
{
    UA_NodeId objects = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId acq     = node_id("AcquisitionData");
    UA_NodeId proc    = node_id("ProcessedData");
    UA_NodeId sys     = node_id("SystemStatus");
    UA_NodeId cmds    = node_id("Commands");

    add_folder("AcquisitionData", "AcquisitionData", objects);
    add_folder("ProcessedData",   "ProcessedData",   objects);
    add_folder("SystemStatus",    "SystemStatus",    objects);
    add_folder("Commands",        "Commands",        objects);

    for (int i = 0; i < g_cfg->point_count; i++)
        add_double(acq, g_cfg->points[i].name, g_cfg->points[i].name, 0.0);

    /* CAN 原始信号 */
    for (int i = 0; i < g_cfg->can.signal_count; i++)
        add_double(acq, g_cfg->can.signals[i].name,
                   g_cfg->can.signals[i].name, 0.0);

    add_double(proc, g_proc_names[PN_VIB_RMS_X],  "VibRmsX (mm/s)",       0.0);
    add_double(proc, g_proc_names[PN_VIB_RMS_Y],  "VibRmsY (mm/s)",       0.0);
    add_double(proc, g_proc_names[PN_VIB_RMS_Z],  "VibRmsZ (mm/s)",       0.0);
    add_double(proc, g_proc_names[PN_VIB_PEAK_X], "VibPeakX (mm/s)",      0.0);
    add_double(proc, g_proc_names[PN_VIB_PEAK_Y], "VibPeakY (mm/s)",      0.0);
    add_double(proc, g_proc_names[PN_VIB_PEAK_Z], "VibPeakZ (mm/s)",      0.0);
    add_double(proc, g_proc_names[PN_ACC_RMS_X],  "AccRmsX (g)",          0.0);
    add_double(proc, g_proc_names[PN_ACC_RMS_Y],  "AccRmsY (g)",          0.0);
    add_double(proc, g_proc_names[PN_ACC_RMS_Z],  "AccRmsZ (g)",          0.0);
    add_double(proc, g_proc_names[PN_TEMP_C],     "TempC (degC)",         0.0);
    add_double(proc, g_proc_names[PN_TEMP_RATE],  "TempRate (degC/s)",    0.0);
    add_double(proc, g_proc_names[PN_SEVERITY],   "VibrationSeverity (mm/s)", 0.0);
    add_int32 (proc, g_proc_names[PN_SEVERITY_CLASS], "SeverityClass",    1);
    add_int32 (proc, g_proc_names[PN_HEALTH],     "HealthIndex",          100);
    add_int32 (proc, g_proc_names[PN_BAD_POINTS], "BadPointCount",        0);
    add_int32 (proc, g_proc_names[PN_ALARM_LEVEL],"AlarmLevel",           0);
    add_int32 (proc, g_proc_names[PN_ALARM_COUNT],"ActiveAlarmCount",     0);
    add_uint32(proc, g_proc_names[PN_ALARM_BITS], "AlarmBits",            0);
    add_string(proc, g_proc_names[PN_ACTIVE_ALARMS], "ActiveAlarms",      "");
    add_uint32(proc, g_proc_names[PN_CONTROL_WORD],  "ControlWord",       0);
    add_string(proc, g_proc_names[PN_CONTROL_ACTION],"ControlAction",     "normal");
    add_int32 (proc, g_proc_names[PN_MEASURE_STATE], "MeasureState",      0);
    add_double(proc, g_proc_names[PN_MEASURE_REMAIN], "MeasureRemainingS", 0.0);
    add_double(proc, g_proc_names[PN_MEASURE_RMS],    "MeasureRms",        0.0);
    add_double(proc, g_proc_names[PN_MEASURE_PEAK],   "MeasurePeak",       0.0);

    add_uint32(sys, g_proc_names[PN_CMD_ID],        "LastCommandId",       0);
    add_string(sys, g_proc_names[PN_CMD_NAME],      "LastCommandName",     "");
    add_int32 (sys, g_proc_names[PN_CMD_STATE],     "LastCommandState",    0);
    add_int32 (sys, g_proc_names[PN_CMD_CODE],      "LastCommandCode",     0);
    add_bool  (sys, g_proc_names[PN_CMD_SUCCESS],   "LastCommandSuccess",  false);
    add_string(sys, g_proc_names[PN_CMD_MESSAGE],   "LastCommandMessage",  "");
    add_uint32(sys, g_proc_names[PN_PUB_SEQ],       "PublishSeq",          0);
    add_uint32(sys, g_proc_names[PN_PUB_FAILED],    "PublishFailed",       0);
    add_uint32(sys, g_proc_names[PN_UPLOAD_FAIL_TOTAL],  "UploadFailTotal",  0);
    add_uint32(sys, g_proc_names[PN_UPLOAD_FAIL_STREAK], "UploadFailStreak", 0);

    add_uint32(sys, g_proc_names[PN_HEARTBEAT],    "Heartbeat",         0);
    add_uint32(sys, g_proc_names[PN_UPTIME],       "UptimeS",           0);
    add_uint32(sys, g_proc_names[PN_ACQ_SEQ],      "AcqSeq",            0);
    add_uint32(sys, g_proc_names[PN_EVAL_SEQ],     "EvalSeq",           0);
    add_uint32(sys, g_proc_names[PN_HIST_SEQ],     "HistorySampleSeq",  0);
    add_uint32(sys, g_proc_names[PN_HIST_MAX],     "HistoryMaxValues",  0);
    add_bool  (sys, g_proc_names[PN_HIST_ENABLED], "HistoryEnabled",    false);

    add_bool  (proc, g_proc_names[PN_CAN_LINK_UP],     "CanLinkUp",           false);
    add_uint32(proc, g_proc_names[PN_CAN_FAULT_BITS],  "CanFaultBits",        0);
    add_int32 (proc, g_proc_names[PN_CAN_ALARM_LEVEL], "CanAlarmLevel",       0);
    add_uint32(proc, g_proc_names[PN_CAN_RX_FRAMES],   "CanRxFrames",         0);
    add_uint32(proc, g_proc_names[PN_CAN_RX_HIGH],     "CanHighPriorityRx",   0);
    add_uint32(proc, g_proc_names[PN_CAN_TX_REQUESTS], "CanTxRequests",       0);
    add_uint32(proc, g_proc_names[PN_CAN_TX_TIMEOUTS], "CanTxTimeouts",       0);
    add_string(proc, g_proc_names[PN_CAN_CMD_NODE],    "CanLastCommandNode",  "");
    add_string(proc, g_proc_names[PN_CAN_CMD_NAME],    "CanLastCommandName",  "");
    add_double(proc, g_proc_names[PN_CAN_CMD_PARAM],   "CanLastCommandParam", 0.0);
    add_uint32(proc, g_proc_names[PN_CAN_CMD_SEQ],     "CanLastCommandSeq",   0);

    for (int ni = 0; ni < g_cfg->can.node_count; ni++) {
        char id[96];
        snprintf(id, sizeof(id), "CanFault_%s", g_cfg->can.nodes[ni].name);
        add_uint32(proc, id, id, 0);

        snprintf(id, sizeof(id), "CanQuality_%s", g_cfg->can.nodes[ni].name);
        add_int32(proc, id, id, 0);
    }

    add_command_method(cmds);
}

/* ---------- 发布 ---------- */

static void put_double(int idx, double v, UA_DateTime ts,
                       uint32_t *total, uint32_t *failed)
{
    UA_Double d = v;
    (*total)++;
    if (write_scalar(proc_id(idx), &d, &UA_TYPES[UA_TYPES_DOUBLE],
                     ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
        (*failed)++;
}

static void put_int32(int idx, UA_Int32 v, UA_DateTime ts,
                      uint32_t *total, uint32_t *failed)
{
    UA_Int32 d = v;
    (*total)++;
    if (write_scalar(proc_id(idx), &d, &UA_TYPES[UA_TYPES_INT32],
                     ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
        (*failed)++;
}

static void put_uint32(int idx, UA_UInt32 v, UA_DateTime ts,
                       uint32_t *total, uint32_t *failed)
{
    UA_UInt32 d = v;
    (*total)++;
    if (write_scalar(proc_id(idx), &d, &UA_TYPES[UA_TYPES_UINT32],
                     ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
        (*failed)++;
}

static void put_bool(int idx, UA_Boolean v, UA_DateTime ts,
                     uint32_t *total, uint32_t *failed)
{
    UA_Boolean d = v;
    (*total)++;
    if (write_scalar(proc_id(idx), &d, &UA_TYPES[UA_TYPES_BOOLEAN],
                     ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
        (*failed)++;
}

static void put_string(int idx, const char *v, UA_DateTime ts,
                       uint32_t *total, uint32_t *failed)
{
    UA_String s;
    s.length = strlen(v);
    s.data   = (UA_Byte *)(uintptr_t)v;

    (*total)++;
    if (write_scalar(proc_id(idx), &s, &UA_TYPES[UA_TYPES_STRING],
                     ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
        (*failed)++;
}

static UA_StatusCode quality_status(quality_t q)
{
    switch (q) {
    case Q_GOOD:      return UA_STATUSCODE_GOOD;
    case Q_STALE:     return UA_STATUSCODE_UNCERTAINLASTUSABLEVALUE;
    case Q_UNCERTAIN: return UA_STATUSCODE_UNCERTAIN;
    default:          return UA_STATUSCODE_BADCOMMUNICATIONERROR;
    }
}

static void publish_raw(const acq_snapshot_t *s, uint32_t *total, uint32_t *failed)
{
    int n = s->point_count;
    if (n > MAX_POINTS) n = MAX_POINTS;

    for (int i = 0; i < n; i++) {
        UA_StatusCode qs = quality_status(s->points[i].quality);
        UA_Double v = s->points[i].value;
        UA_DateTime ts = ua_ts(s->points[i].ts_utc_us);

        (*total)++;
        if (write_scalar(node_id(g_cfg->points[i].name), &v,
                         &UA_TYPES[UA_TYPES_DOUBLE], ts, qs) != UA_STATUSCODE_GOOD)
            (*failed)++;
    }
}

static void publish_can_raw(const can_snapshot_t *c, uint32_t *total, uint32_t *failed)
{
    for (int i = 0; i < c->signal_count; i++) {
        UA_StatusCode qs = quality_status(c->signals[i].quality);
        UA_Double v = c->signals[i].value;
        UA_DateTime ts = ua_ts(c->signals[i].ts_utc_us);

        (*total)++;
        if (write_scalar(node_id(g_cfg->can.signals[i].name), &v,
                         &UA_TYPES[UA_TYPES_DOUBLE], ts, qs) != UA_STATUSCODE_GOOD)
            (*failed)++;
    }
}

static void publish_processed(const proc_snapshot_t *p, uint32_t *total, uint32_t *failed)
{
    UA_DateTime ts = ua_ts(p->ts_utc_us);

    put_double(PN_VIB_RMS_X,  p->vib_rms[0],  ts, total, failed);
    put_double(PN_VIB_RMS_Y,  p->vib_rms[1],  ts, total, failed);
    put_double(PN_VIB_RMS_Z,  p->vib_rms[2],  ts, total, failed);
    put_double(PN_VIB_PEAK_X, p->vib_peak[0], ts, total, failed);
    put_double(PN_VIB_PEAK_Y, p->vib_peak[1], ts, total, failed);
    put_double(PN_VIB_PEAK_Z, p->vib_peak[2], ts, total, failed);
    put_double(PN_ACC_RMS_X,  p->acc_rms[0],  ts, total, failed);
    put_double(PN_ACC_RMS_Y,  p->acc_rms[1],  ts, total, failed);
    put_double(PN_ACC_RMS_Z,  p->acc_rms[2],  ts, total, failed);
    put_double(PN_TEMP_C,     p->temp_c,      ts, total, failed);
    put_double(PN_TEMP_RATE,  p->temp_rate_c_s, ts, total, failed);
    put_double(PN_SEVERITY,   p->severity,    ts, total, failed);

    put_int32(PN_SEVERITY_CLASS, p->severity_class, ts, total, failed);
    put_int32(PN_HEALTH,         p->health,         ts, total, failed);
    put_int32(PN_BAD_POINTS,     p->bad_points,     ts, total, failed);

    put_int32 (PN_ALARM_LEVEL, p->alarm_level,        ts, total, failed);
    put_int32 (PN_ALARM_COUNT, p->alarm_active_count, ts, total, failed);
    put_uint32(PN_ALARM_BITS,  p->alarm_bits,         ts, total, failed);
    put_string(PN_ACTIVE_ALARMS, p->active_alarms,    ts, total, failed);

    put_uint32(PN_CONTROL_WORD,   p->control_word,    ts, total, failed);
    put_string(PN_CONTROL_ACTION, p->control_action,  ts, total, failed);

    put_int32 (PN_MEASURE_STATE,  p->measure_state,        ts, total, failed);
    put_double(PN_MEASURE_REMAIN, p->measure_remaining_s,  ts, total, failed);
    put_double(PN_MEASURE_RMS,    p->measure_rms,          ts, total, failed);
    put_double(PN_MEASURE_PEAK,   p->measure_peak,         ts, total, failed);

    put_bool  (PN_CAN_LINK_UP,     p->can_link_up,      ts, total, failed);
    put_uint32(PN_CAN_FAULT_BITS,  p->can_fault_bits,   ts, total, failed);
    put_int32 (PN_CAN_ALARM_LEVEL, p->can_alarm_level,  ts, total, failed);
    put_uint32(PN_CAN_RX_FRAMES,   p->can_rx_frames,    ts, total, failed);
    put_uint32(PN_CAN_RX_HIGH,     p->can_rx_high,      ts, total, failed);
    put_uint32(PN_CAN_TX_REQUESTS, p->can_tx_requests,  ts, total, failed);
    put_uint32(PN_CAN_TX_TIMEOUTS, p->can_tx_timeouts,  ts, total, failed);
    put_string(PN_CAN_CMD_NODE,    p->can_cmd_node,     ts, total, failed);
    put_string(PN_CAN_CMD_NAME,    p->can_cmd_name,     ts, total, failed);
    put_double(PN_CAN_CMD_PARAM,   p->can_cmd_param,    ts, total, failed);
    put_uint32(PN_CAN_CMD_SEQ,     p->can_cmd_seq,      ts, total, failed);

    for (int ni = 0; ni < p->can_node_count; ni++) {
        char id[96];
        UA_UInt32 fault   = p->can[ni].fault;
        UA_Int32  quality = p->can[ni].quality;

        snprintf(id, sizeof(id), "CanFault_%s", p->can[ni].name);
        (*total)++;
        if (write_scalar(node_id(id), &fault, &UA_TYPES[UA_TYPES_UINT32],
                         ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
            (*failed)++;

        snprintf(id, sizeof(id), "CanQuality_%s", p->can[ni].name);
        (*total)++;
        if (write_scalar(node_id(id), &quality, &UA_TYPES[UA_TYPES_INT32],
                         ts, UA_STATUSCODE_GOOD) != UA_STATUSCODE_GOOD)
            (*failed)++;
    }
}

static void publish_cmd_result(const proc_cmd_result_t *r)
{
    UA_DateTime ts = ua_ts(r->ts_utc_us);
    uint32_t total = 0, failed = 0;
    /* 只有 done + code 0 才算执行成功 */
    UA_Boolean success = (r->state == PROC_CMD_STATE_DONE && r->code == 0);

    put_uint32(PN_CMD_ID,   r->id,                    ts, &total, &failed);
    put_string(PN_CMD_NAME, proc_cmd_name(r->cmd),    ts, &total, &failed);
    put_int32 (PN_CMD_STATE, (UA_Int32)r->state,      ts, &total, &failed);
    put_int32 (PN_CMD_CODE, r->code,                  ts, &total, &failed);
    put_bool  (PN_CMD_SUCCESS, success,               ts, &total, &failed);
    put_string(PN_CMD_MESSAGE, r->message,            ts, &total, &failed);
}

/* ---------- HistoryRead 历史缓存 ---------- */

static UA_NodeId hist_node_id(const hist_node_t *h)
{
    switch (h->kind) {
    case 1:  return node_id(g_cfg->points[h->idx].name);
    case 2:  return node_id(g_cfg->can.signals[h->idx].name);
    default: return proc_id(h->idx);
    }
}

static double proc_history_value(const proc_snapshot_t *p, int idx)
{
    switch (idx) {
    case PN_VIB_RMS_X:      return p->vib_rms[0];
    case PN_VIB_RMS_Y:      return p->vib_rms[1];
    case PN_VIB_RMS_Z:      return p->vib_rms[2];
    case PN_TEMP_C:         return p->temp_c;
    case PN_SEVERITY:       return p->severity;
    case PN_SEVERITY_CLASS: return (double)p->severity_class;
    case PN_HEALTH:         return (double)p->health;
    case PN_ALARM_LEVEL:    return (double)p->alarm_level;
    case PN_CONTROL_WORD:   return (double)p->control_word;
    default:                return 0.0;
    }
}

static void hist_register(int kind, int idx)
{
    if (g_hist_node_count >= HIST_MAX_NODES) return;

    int type = 0;
    if (kind == 0) {
        if (idx == PN_SEVERITY_CLASS || idx == PN_HEALTH || idx == PN_ALARM_LEVEL)
            type = 1;   /* Int32 */
        else if (idx == PN_CONTROL_WORD)
            type = 2;   /* UInt32 */
    }

    hist_node_t h = { .kind = kind, .idx = idx, .type = type };
    UA_NodeId nid = hist_node_id(&h);

    UA_HistorizingNodeIdSettings s;
    memset(&s, 0, sizeof(s));
    s.historizingBackend         = g_hist_backend;
    s.maxHistoryDataResponseSize = 1000;
    s.historizingUpdateStrategy  = UA_HISTORIZINGUPDATESTRATEGY_USER;
    s.pollingInterval            = 0;

    UA_StatusCode st = g_hist_gathering.registerNodeId(
        g_server, g_hist_gathering.context, &nid, s);
    if (st != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] history register failed 0x%08x\n", st);
        return;
    }

    st = UA_Server_writeHistorizing(g_server, nid, true);
    if (st != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] set historizing failed 0x%08x\n", st);
        return;
    }

    g_hist_nodes[g_hist_node_count++] = h;
}

static void setup_history(void)
{
    if (!g_cfg->hist.enabled) {
        fprintf(stderr, "[opcua] history disabled\n");
        return;
    }

    g_hist_gathering = UA_HistoryDataGathering_Circular(HIST_MAX_NODES);
    g_hist_backend = UA_HistoryDataBackend_Memory_Circular(
        HIST_MAX_NODES, (size_t)g_cfg->hist.max_values);

    if (!g_hist_backend.context || !g_hist_gathering.context) {
        fprintf(stderr, "[opcua] history backend init failed\n");
        return;
    }

    UA_ServerConfig *config = UA_Server_getConfig(g_server);
    UA_HistoryDatabase hdb = UA_HistoryDatabase_default(g_hist_gathering);
    if (!hdb.context) {
        fprintf(stderr, "[opcua] history database init failed\n");
        return;
    }
    config->historyDatabase = hdb;

    /* 原始点位 */
    for (int i = 0; i < g_cfg->point_count; i++)
        hist_register(1, i);

    /* 关键处理结果 */
    static const int proc_nodes[] = {
        PN_VIB_RMS_X, PN_VIB_RMS_Y, PN_VIB_RMS_Z,
        PN_TEMP_C, PN_SEVERITY, PN_SEVERITY_CLASS,
        PN_HEALTH, PN_ALARM_LEVEL, PN_CONTROL_WORD
    };
    for (size_t k = 0; k < sizeof(proc_nodes) / sizeof(proc_nodes[0]); k++)
        hist_register(0, proc_nodes[k]);

    /* CAN 原始信号（冷却泵/风机流量、电压、电流） */
    for (int i = 0; i < g_cfg->can.signal_count; i++)
        hist_register(2, i);

    g_hist_enabled = 1;
    fprintf(stderr, "[opcua] history: %d nodes x %d values @%dms (HistoryRead)\n",
            g_hist_node_count, g_cfg->hist.max_values,
            g_cfg->hist.sample_interval_ms);
}

static void history_sample(const acq_snapshot_t *a, const proc_snapshot_t *p,
                           const can_snapshot_t *c)
{
    if (!g_hist_enabled) return;
    if (p->ts_utc_us <= 0) return;   /* 还没有有效采集数据 */

    int64_t now = now_mono_us();
    int64_t interval = (int64_t)g_cfg->hist.sample_interval_ms * 1000;
    if (g_hist_last_us != 0 && now - g_hist_last_us < interval) return;

    /* 采集轮次没更新时不重复写同一个时间戳 */
    if (p->ts_utc_us == g_hist_last_ts) return;

    g_hist_last_us = now;
    g_hist_last_ts = p->ts_utc_us;

    UA_DateTime ts = ua_ts(p->ts_utc_us);

    for (int i = 0; i < g_hist_node_count; i++) {
        const hist_node_t *h = &g_hist_nodes[i];

        double v = 0.0;
        UA_StatusCode st = UA_STATUSCODE_GOOD;

        if (h->kind == 1) {
            if (h->idx >= a->point_count) continue;
            v = a->points[h->idx].value;
            st = quality_status(a->points[h->idx].quality);
        } else if (h->kind == 2) {
            if (h->idx >= c->signal_count) continue;
            v = c->signals[h->idx].value;
            st = quality_status(c->signals[h->idx].quality);
        } else {
            v = proc_history_value(p, h->idx);
        }

        UA_Double d  = 0.0;
        UA_Int32  i32 = 0;
        UA_UInt32 u32 = 0;

        UA_DataValue dv;
        UA_DataValue_init(&dv);
        dv.hasValue = true;

        if (h->type == 1) {
            i32 = (UA_Int32)v;
            UA_Variant_setScalar(&dv.value, &i32, &UA_TYPES[UA_TYPES_INT32]);
        } else if (h->type == 2) {
            u32 = (UA_UInt32)v;
            UA_Variant_setScalar(&dv.value, &u32, &UA_TYPES[UA_TYPES_UINT32]);
        } else {
            d = v;
            UA_Variant_setScalar(&dv.value, &d, &UA_TYPES[UA_TYPES_DOUBLE]);
        }
        dv.value.storageType = UA_VARIANT_DATA_NODELETE;

        dv.hasSourceTimestamp = true;
        dv.sourceTimestamp = ts;
        dv.hasStatus = true;
        dv.status = st;

        UA_NodeId nid = hist_node_id(h);
        g_hist_backend.serverSetHistoryData(g_server, g_hist_backend.context,
                                            NULL, NULL, &nid, true, &dv);
    }

    g_hist_seq++;
}

static void publish_cycle(void)
{
    acq_snapshot_t  a;
    proc_snapshot_t p;
    can_snapshot_t  c;
    uint32_t total = 0, failed = 0;

    acq_snapshot_get(&a);
    proc_snapshot_get(&p);
    can_acq_snapshot_get(&c);

    g_pub_seq++;

    publish_raw(&a, &total, &failed);
    publish_can_raw(&c, &total, &failed);
    publish_processed(&p, &total, &failed);

    {
        UA_DateTime ts = ua_ts(now_utc_us());
        put_uint32(PN_PUB_SEQ,            g_pub_seq,             ts, &total, &failed);
        put_uint32(PN_PUB_FAILED,         failed,                ts, &total, &failed);
        put_uint32(PN_UPLOAD_FAIL_TOTAL,  p.upload_fail_total,   ts, &total, &failed);
        put_uint32(PN_UPLOAD_FAIL_STREAK, p.upload_fail_streak,  ts, &total, &failed);

        uint32_t uptime = g_start_us ? (uint32_t)((now_mono_us() - g_start_us) / 1000000) : 0;
        put_uint32(PN_HEARTBEAT,    ++g_heartbeat,             ts, &total, &failed);
        put_uint32(PN_UPTIME,       uptime,                    ts, &total, &failed);
        put_uint32(PN_ACQ_SEQ,      a.dev_seq,                 ts, &total, &failed);
        put_uint32(PN_EVAL_SEQ,     p.eval_seq,                ts, &total, &failed);
        put_uint32(PN_HIST_SEQ,     g_hist_seq,                ts, &total, &failed);
        put_uint32(PN_HIST_MAX,     g_cfg->hist.enabled ?
                     (UA_UInt32)g_cfg->hist.max_values : 0,    ts, &total, &failed);
        put_bool  (PN_HIST_ENABLED, g_hist_enabled,            ts, &total, &failed);
    }

    history_sample(&a, &p, &c);

    opcua_transfer_result_t r;
    r.seq       = g_pub_seq;
    r.total     = total;
    r.ok        = total - failed;
    r.failed    = failed;
    r.status    = failed ? -1 : 0;
    r.ts_utc_us = now_utc_us();

    if (g_transfer_cb) g_transfer_cb(&r, g_transfer_user);
}

/* ---------- 命令结果 ---------- */

static void on_cmd_result(const proc_cmd_result_t *r, void *user)
{
    (void)user;

    pthread_mutex_lock(&g_cmd_mu);

    if (g_pending_id != 0 && r->id >= g_pending_id &&
        (r->state == PROC_CMD_STATE_DONE ||
         r->state == PROC_CMD_STATE_REJECTED ||
         r->state == PROC_CMD_STATE_FAILED))
        g_pending_id = 0;

    g_cmd_result = *r;
    g_cmd_dirty  = 1;
    pthread_mutex_unlock(&g_cmd_mu);
}

static void drain_cmd_result(void)
{
    proc_cmd_result_t r;
    int have = 0;

    pthread_mutex_lock(&g_cmd_mu);
    if (g_cmd_dirty) {
        r = g_cmd_result;
        g_cmd_dirty = 0;
        have = 1;
    }
    pthread_mutex_unlock(&g_cmd_mu);

    if (have) publish_cmd_result(&r);
}

/* 处理线程卡死/异常时，不能让客户端一直等：超时后主动补一条失败结果 */
static void check_cmd_timeout(void)
{
    uint32_t   id  = 0;
    proc_cmd_t cmd = PROC_CMD_NONE;
    int64_t    now = now_mono_us();

    pthread_mutex_lock(&g_cmd_mu);
    if (g_pending_id != 0 && now >= g_pending_deadline_us) {
        id  = g_pending_id;
        cmd = g_pending_cmd;
        g_pending_id = 0;
    }
    pthread_mutex_unlock(&g_cmd_mu);

    if (id == 0) return;

    proc_cmd_result_t r;
    memset(&r, 0, sizeof(r));
    r.id         = id;
    r.cmd        = cmd;
    r.state      = PROC_CMD_STATE_FAILED;
    r.code       = -2;
    r.ts_utc_us  = now_utc_us();
    snprintf(r.message, sizeof(r.message),
             "no result within timeout (processing thread busy?)");

    publish_cmd_result(&r);
    LOG_E("opcua", "command %u (%s) timeout, no result from processing thread",
          id, proc_cmd_name(cmd));
}

/* ---------- 线程 ---------- */

static void *opcua_thread_main(void *arg)
{
    (void)arg;

    g_server = UA_Server_new();
    if (!g_server) {
        fprintf(stderr, "[opcua] UA_Server_new failed\n");
        return NULL;
    }

    UA_ServerConfig *config = UA_Server_getConfig(g_server);
    UA_StatusCode retval = UA_ServerConfig_setMinimal(
        config, (UA_UInt16)g_cfg->opcua_port, NULL);
    if (retval != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] config failed 0x%08x\n", retval);
        UA_Server_delete(g_server);
        g_server = NULL;
        return NULL;
    }

    g_ns = UA_Server_addNamespace(g_server, "urn:edge-agent:acq");
    if (g_ns == 0) g_ns = 1;

    create_nodes();
    setup_history();

    proc_set_cmd_result_cb(on_cmd_result, NULL);
    retval = UA_Server_run_startup(g_server);
    if (retval != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] startup failed 0x%08x\n", retval);
        proc_set_cmd_result_cb(NULL, NULL);
        UA_Server_delete(g_server);
        g_server = NULL;
        return NULL;
    }

    fprintf(stderr, "[opcua] listening on opc.tcp://0.0.0.0:%d\n", g_cfg->opcua_port);

    g_start_us = now_mono_us();

    while (g_run) {
        UA_Server_run_iterate(g_server, false);
        publish_cycle();
        drain_cmd_result();
        check_cmd_timeout();

        struct timespec pause = { .tv_sec = 0, .tv_nsec = 100 * 1000 * 1000 };
        nanosleep(&pause, NULL);
    }

    UA_Server_run_shutdown(g_server);
    proc_set_cmd_result_cb(NULL, NULL);
    UA_Server_delete(g_server);
    g_server = NULL;

    fprintf(stderr, "[opcua] stopped\n");
    return NULL;
}

int opcua_server_start(const app_config_t *cfg)
{
    g_cfg = cfg;
    g_run = 1;

    if (pthread_create(&g_tid, NULL, opcua_thread_main, NULL) != 0) {
        g_run = 0;
        return -1;
    }
    return 0;
}

void opcua_server_stop(void)
{
    if (!g_run) return;
    g_run = 0;
    pthread_join(g_tid, NULL);
}

void opcua_server_set_transfer_cb(opcua_transfer_cb_t cb, void *user)
{
    g_transfer_cb   = cb;
    g_transfer_user = user;
}
