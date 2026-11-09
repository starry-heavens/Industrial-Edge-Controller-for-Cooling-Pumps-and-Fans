#include "config.h"
#include "acq_thread.h"
#include "can_acq.h"
#include "log.h"
#include "monitor.h"
#include "opcua_server.h"
#include "processing.h"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int sig) { (void)sig; g_stop = 1; }

/* OPC UA 每轮发布后的“传输结果”回调：把结果送回处理层参与控制判断 */
static void on_transfer_result(const opcua_transfer_result_t *r, void *user)
{
    (void)user;

    proc_note_transfer((uint32_t)r->seq, r->ok, r->failed);

    if (r->failed)
        LOG_W("main", "opcua transfer seq=%llu ok=%u failed=%u",
              (unsigned long long)r->seq, r->ok, r->failed);
}

static int sleep_interruptible(int seconds)
{
    struct timespec req = { .tv_sec = seconds, .tv_nsec = 0 };
    struct timespec rem;
    while (nanosleep(&req, &rem) != 0) {
        if (g_stop) return -1;
        req = rem;
    }
    return g_stop ? -1 : 0;
}

static const char *qn(quality_t q)
{
    switch (q) {
    case Q_GOOD: return "GOOD";
    case Q_STALE: return "STALE";
    case Q_BAD: return "BAD";
    default: return "UNC";
    }
}

int main(int argc, char **argv)
{
    if (log_init(NULL, LOG_LEVEL_INFO) != 0)
        fprintf(stderr, "log init failed, continue without log\n");
    else
        LOG_I("main", "app start, version=0.1.0");

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    const char *path = (argc > 1) ? argv[1] : "config_wt.json";
    app_config_t cfg;

    if (config_load(path, &cfg) != 0) {
        fprintf(stderr, "load config failed: %s\n", path);
        LOG_E("main", "load config failed");
        log_close();
        return 1;
    }
    config_dump(&cfg);

    if (acq_thread_start(&cfg) != 0) {
        fprintf(stderr, "acq start failed\n");
        LOG_E("main", "acq start failed");
        log_close();
        return 1;
    }

    /* CAN 采集：接口打不开不退出进程，后台每 5s 重连 */
    if (can_acq_start(&cfg) != 0) {
        LOG_E("main", "can acq thread start failed");
    } else {
        LOG_I("main", "can acq started (enabled=%d)", cfg.can.enabled);
    }

    /* 中间处理层：把采集数据变成特征/报警/控制字，并提供异步处理命令 */
    if (proc_start(&cfg) != 0) {
        fprintf(stderr, "processing start failed\n");
        LOG_E("main", "processing start failed");
        acq_thread_stop();
        can_acq_stop();
        log_close();
        return 1;
    }

    /* --- 启动 OPC UA 服务器（数据 + 处理结果 + 命令方法） --- */
    opcua_server_set_transfer_cb(on_transfer_result, NULL);

    if (opcua_server_start(&cfg) != 0) {
        fprintf(stderr, "opcua server start failed\n");
        LOG_E("main", "opcua start failed");
    } else {
        LOG_I("main", "opcua server started");
    }
    if (monitor_start() != 0) {
        fprintf(stderr, "monitor start failed, continue\n");
        LOG_W("main", "monitor start failed");
    } else {
        LOG_I("main", "monitor started");
    }

    for (int i = 0; !g_stop; i++) {
        if (sleep_interruptible(1) != 0) break;

        acq_snapshot_t s;
        acq_snapshot_get(&s);
        proc_snapshot_t ps;
        proc_snapshot_get(&ps);
        can_snapshot_t cs;
        can_acq_snapshot_get(&cs);

        printf("---- cycle %d  dev_seq=%u fail=%u ----\n",
               i + 1, s.dev_seq, s.fail_count);
        for (int k = 0; k < s.point_count; k++) {
            printf("  %-12s = %12.6f %-6s [%s]\n",
                   cfg.points[k].name,
                   s.points[k].value,
                   cfg.points[k].unit,
                   qn(s.points[k].quality));
        }
        if (cs.enabled) {
            printf("  [can] link=%s rx=%u(high=%u) tx_req=%u timeout=%u\n",
                   cs.link_up ? "up" : "down",
                   cs.rx_frames, cs.rx_high, cs.tx_requests, cs.tx_timeouts);
            for (int ni = 0; ni < cs.node_count; ni++) {
                printf("  [can] %-14s", cs.nodes[ni].name);
                for (int si = 0; si < cs.signal_count; si++) {
                    if (cs.signals[si].node != ni) continue;
                    printf(" %s=%.3f%s[%s]",
                           cfg.can.signals[si].name,
                           cs.signals[si].value,
                           cfg.can.signals[si].unit,
                           qn(cs.signals[si].quality));
                }
                printf(" fault=0x%02X level=%d",
                       cs.nodes[ni].fault, cs.nodes[ni].alarm_level);
                if (cs.nodes[ni].last_cmd[0])
                    printf(" last_cmd=%s(%.1f)",
                           cs.nodes[ni].last_cmd, cs.nodes[ni].last_cmd_param);
                printf("\n");
            }
        }
        printf("  [proc] health=%3d alarm=%d(%s) ctl=0x%02X(%s) sev=%.2f class=%d\n",
               ps.health, ps.alarm_level, ps.active_alarms,
               ps.control_word, ps.control_action,
               ps.severity, ps.severity_class);
    }

    monitor_stop();
    opcua_server_stop();
    acq_thread_stop();
    proc_stop();
    can_acq_stop();
    LOG_I("main", "app exit");
    log_close();
    return 0;
}