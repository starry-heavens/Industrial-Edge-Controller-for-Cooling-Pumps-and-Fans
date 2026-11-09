#ifndef OPCUA_SERVER_H
#define OPCUA_SERVER_H

#include "config.h"

#include <stdint.h>

/*
 * 一次发布（本地数据 -> OPC UA 地址空间）的结果。
 * 服务器每轮写入完成后通过回调把结果交给上层，
 * 上层可用它做失败统计、健康度降级等“控制”动作。
 */
typedef struct {
    uint64_t seq;          /* 发布轮次 */
    uint32_t total;        /* 本轮尝试写入的节点数 */
    uint32_t ok;           /* 成功数 */
    uint32_t failed;       /* 失败数 */
    int32_t  status;       /* 0=全部成功，-1=存在失败 */
    int64_t  ts_utc_us;
} opcua_transfer_result_t;

typedef void (*opcua_transfer_cb_t)(const opcua_transfer_result_t *r, void *user);

int  opcua_server_start(const app_config_t *cfg);
void opcua_server_stop(void);

/* 注册“传输结果”回调（在 OPC 线程上下文调用，回调里不要阻塞） */
void opcua_server_set_transfer_cb(opcua_transfer_cb_t cb, void *user);

#endif /* OPCUA_SERVER_H */
