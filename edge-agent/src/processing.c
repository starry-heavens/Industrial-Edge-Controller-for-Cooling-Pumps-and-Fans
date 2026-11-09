#define _GNU_SOURCE

#include "processing.h"
#include "acq_thread.h"
#include "can_acq.h"
#include "log.h"

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PROC_TICK_MS        100
#define PROC_HIST_MAX       128
#define PROC_CMD_QUEUE_LEN  8
#define PROC_RESULT_MAX     16

typedef struct {
    double  val[PROC_HIST_MAX];
    int64_t ts[PROC_HIST_MAX];
    uint8_t q[PROC_HIST_MAX];
    int     head;       /* 下一个写入位置 */
    int     count;
} point_hist_t;

typedef struct {
    int     active;
    int     acked;
    int64_t pending_since_us;
    int64_t active_since_us;
    int64_t ack_us;
} alarm_rt_t;

typedef struct {
    uint32_t  id;
    proc_cmd_t cmd;
    double    param;
    int64_t   submit_us;
} pending_cmd_t;

typedef struct {
    int      state;      /* 0=idle 1=running 2=done */
    int64_t  end_us;
    uint32_t cmd_id;
    double   seconds;
} measure_rt_t;

typedef struct {
    int     n;
    double  rms;
    double  peak;
    double  first;
    double  last;
    int64_t first_ts;
    int64_t last_ts;
} hist_stat_t;

static app_config_t      g_cfg;
static pthread_mutex_t   g_mu = PTHREAD_MUTEX_INITIALIZER;
static proc_snapshot_t   g_snap;

static pthread_t         g_tid;
static volatile int      g_run;

static point_hist_t      g_hist[MAX_POINTS];
static uint32_t          g_last_seq[MAX_POINTS];
static uint8_t           g_seen[MAX_POINTS];

static int               g_vel_idx[3]  = { -1, -1, -1 };
static int               g_acc_idx[3]  = { -1, -1, -1 };
static int               g_temp_idx    = -1;
static int               g_rule_point[MAX_ALARM_RULES];
static uint8_t           g_rule_enabled[MAX_ALARM_RULES];

/* CAN 按 role 映射到节点，不依赖信号顺序 */
static int               g_can_flow_idx[MAX_CAN_NODES];
static int               g_can_volt_idx[MAX_CAN_NODES];
static int               g_can_curr_idx[MAX_CAN_NODES];
static int64_t           g_can_boost_us[MAX_CAN_NODES];
static uint32_t          g_can_seen_cmd_seq[MAX_CAN_NODES];

static alarm_rt_t        g_alarm[MAX_ALARM_RULES];
static measure_rt_t      g_measure;

static pending_cmd_t     g_queue[PROC_CMD_QUEUE_LEN];
static int               g_q_head, g_q_tail, g_q_len;
static uint32_t          g_next_cmd_id = 1;

static proc_cmd_result_cb_t g_cb;
static void                *g_cb_user;

static int64_t           g_last_eval_us;

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

static int axis_of_name(const char *name)
{
    size_t n = strlen(name);
    if (n < 2 || name[n - 2] != '_') return -1;
    switch (name[n - 1]) {
    case 'x': case 'X': return 0;
    case 'y': case 'Y': return 1;
    case 'z': case 'Z': return 2;
    default:            return -1;
    }
}

static int find_point(const char *name)
{
    for (int i = 0; i < g_cfg.point_count; i++)
        if (strcmp(g_cfg.points[i].name, name) == 0) return i;
    return -1;
}

/* 按单位把点位映射到速度/加速度/温度，避免把轴名写死 */
static void build_map(void)
{
    for (int ax = 0; ax < 3; ax++) { g_vel_idx[ax] = -1; g_acc_idx[ax] = -1; }
    g_temp_idx = -1;

    for (int i = 0; i < g_cfg.point_count; i++) {
        const point_cfg_t *p = &g_cfg.points[i];
        int ax = axis_of_name(p->name);

        if (strcmp(p->unit, "mm/s") == 0 && ax >= 0) {
            g_vel_idx[ax] = i;
        } else if (strcmp(p->unit, "g") == 0 && ax >= 0) {
            g_acc_idx[ax] = i;
        } else if (strstr(p->unit, "degC") || strstr(p->unit, "°C") ||
                   strcmp(p->unit, "C") == 0) {
            if (g_temp_idx < 0) g_temp_idx = i;
        }
    }

    for (int r = 0; r < g_cfg.alarm_rule_count; r++) {
        const alarm_rule_cfg_t *rule = &g_cfg.alarms[r];
        g_rule_enabled[r] = 0;
        g_rule_point[r]   = -1;

        if (rule->cond == ALARM_COND_STALE) {
            g_rule_enabled[r] = 1;
            if (strcmp(rule->point, "*") != 0) {
                int pi = find_point(rule->point);
                if (pi < 0) {
                    g_rule_enabled[r] = 0;
                    LOG_W("proc", "alarm '%s': point '%s' not found, disabled",
                          rule->name, rule->point);
                } else {
                    g_rule_point[r] = pi;
                }
            }
        } else {
            int pi = find_point(rule->point);
            if (pi < 0) {
                LOG_W("proc", "alarm '%s': point '%s' not found, disabled",
                      rule->name, rule->point);
            } else {
                g_rule_point[r]   = pi;
                g_rule_enabled[r] = 1;
            }
        }
    }

    fprintf(stderr, "[proc] vel=(%d,%d,%d) acc=(%d,%d,%d) temp=%d rules=%d\n",
            g_vel_idx[0], g_vel_idx[1], g_vel_idx[2],
            g_acc_idx[0], g_acc_idx[1], g_acc_idx[2],
            g_temp_idx, g_cfg.alarm_rule_count);
}

static void build_can_map(void)
{
    for (int ni = 0; ni < MAX_CAN_NODES; ni++) {
        g_can_flow_idx[ni] = -1;
        g_can_volt_idx[ni] = -1;
        g_can_curr_idx[ni] = -1;
        g_can_boost_us[ni] = 0;
    }

    for (int si = 0; si < g_cfg.can.signal_count; si++) {
        const can_signal_cfg_t *s = &g_cfg.can.signals[si];
        if (s->node < 0 || s->node >= g_cfg.can.node_count) continue;

        switch (s->role) {
        case CAN_ROLE_FLOW:
            if (g_can_flow_idx[s->node] < 0) g_can_flow_idx[s->node] = si;
            break;
        case CAN_ROLE_VOLTAGE:
            if (g_can_volt_idx[s->node] < 0) g_can_volt_idx[s->node] = si;
            break;
        case CAN_ROLE_CURRENT:
            if (g_can_curr_idx[s->node] < 0) g_can_curr_idx[s->node] = si;
            break;
        default:
            break;
        }
    }

    fprintf(stderr, "[proc] can nodes=%d", g_cfg.can.node_count);
    for (int ni = 0; ni < g_cfg.can.node_count; ni++)
        fprintf(stderr, " %s=(flow %d, V %d, A %d)",
                g_cfg.can.nodes[ni].name,
                g_can_flow_idx[ni], g_can_volt_idx[ni], g_can_curr_idx[ni]);
    fprintf(stderr, "\n");
}

static void hist_push(point_hist_t *h, double v, int64_t ts, quality_t q)
{
    h->val[h->head] = v;
    h->ts[h->head]  = ts;
    h->q[h->head]   = (uint8_t)q;
    h->head = (h->head + 1) % PROC_HIST_MAX;
    if (h->count < PROC_HIST_MAX) h->count++;
}

/* 由新到旧遍历滑窗，只统计 GOOD 点 */
static void hist_stats(const point_hist_t *h, int64_t now_us, int64_t win_us,
                       hist_stat_t *out)
{
    memset(out, 0, sizeof(*out));

    double sumsq = 0.0;
    double peak  = 0.0;
    int    n     = 0;

    for (int k = 0; k < h->count; k++) {
        int idx = (h->head - 1 - k + PROC_HIST_MAX * 2) % PROC_HIST_MAX;
        int64_t ts = h->ts[idx];
        if (ts <= 0) continue;
        if (now_us - ts > win_us) break;
        if (h->q[idx] != (uint8_t)Q_GOOD) continue;

        double v  = h->val[idx];
        double av = fabs(v);
        sumsq += v * v;
        if (av > peak) peak = av;

        if (n == 0) {              /* 最新的有效样本 */
            out->last    = v;
            out->last_ts = ts;
        }
        out->first    = v;         /* 一直覆盖到最旧的有效样本 */
        out->first_ts = ts;
        n++;
    }

    out->n    = n;
    out->peak = peak;
    out->rms  = (n > 0) ? sqrt(sumsq / (double)n) : 0.0;
}

static void sample_all(const acq_snapshot_t *s, int64_t now_us)
{
    int n = s->point_count;
    if (n > MAX_POINTS) n = MAX_POINTS;

    for (int i = 0; i < n; i++) {
        const point_value_t *pv = &s->points[i];
        if (g_seen[i] && pv->seq == g_last_seq[i]) continue;

        int64_t ts = pv->ts_mono_us > 0 ? pv->ts_mono_us : now_us;
        hist_push(&g_hist[i], pv->value, ts, pv->quality);
        g_last_seq[i] = pv->seq;
        g_seen[i]     = 1;
    }
}

static int quality_rank(quality_t q)
{
    switch (q) {
    case Q_GOOD:      return 0;
    case Q_STALE:     return 1;
    case Q_UNCERTAIN: return 2;
    case Q_BAD:       return 3;
    default:          return 3;
    }
}

static quality_t rank_quality(int r)
{
    switch (r) {
    case 0:  return Q_GOOD;
    case 1:  return Q_STALE;
    case 2:  return Q_UNCERTAIN;
    default: return Q_BAD;
    }
}

/*
 * 把 CAN 快照并入处理结果。
 * 电压/电流的快速保护在 can_acq 线程里已经做过；这里做：
 *   - 节点 flow/voltage/current 汇总与最差质量
 *   - 流量低（慢回路）判断，按 5s 冷却下发 boost
 *   - CAN 故障位、报警级别汇总
 */
static void can_update(const can_snapshot_t *cs, int64_t now)
{
    if (!g_cfg.can.enabled) {
        g_snap.can_node_count   = 0;
        g_snap.can_fault_bits   = 0;
        g_snap.can_alarm_level  = 0;
        g_snap.can_link_up      = 0;
        return;
    }

    g_snap.can_node_count  = cs->node_count;
    g_snap.can_link_up     = cs->link_up;
    g_snap.can_rx_frames   = cs->rx_frames;
    g_snap.can_rx_high     = cs->rx_high;
    g_snap.can_tx_requests = cs->tx_requests;
    g_snap.can_tx_timeouts = cs->tx_timeouts;

    uint32_t all_fault = 0;
    int      all_level = 0;

    for (int ni = 0; ni < cs->node_count; ni++) {
        const can_node_value_t *cn = &cs->nodes[ni];
        proc_can_node_t        *pn = &g_snap.can[ni];

        snprintf(pn->name, sizeof(pn->name), "%s", cn->name);
        pn->flow    = 0.0;
        pn->voltage = 0.0;
        pn->current = 0.0;

        int rank = quality_rank(Q_GOOD);
        for (int si = 0; si < cs->signal_count; si++) {
            if (cs->signals[si].node != ni) continue;
            int r = quality_rank(cs->signals[si].quality);
            if (r > rank) rank = r;
        }
        pn->quality = (int)rank_quality(rank);

        if (g_can_flow_idx[ni] >= 0)
            pn->flow = cs->signals[g_can_flow_idx[ni]].value;
        if (g_can_volt_idx[ni] >= 0)
            pn->voltage = cs->signals[g_can_volt_idx[ni]].value;
        if (g_can_curr_idx[ni] >= 0)
            pn->current = cs->signals[g_can_curr_idx[ni]].value;

        uint32_t fault = cn->fault;
        int      level = cn->alarm_level;

        /* 流量低是慢回路：在 processing 里判断 */
        int fi = g_can_flow_idx[ni];
        if (fi >= 0 && cs->signals[fi].quality == Q_GOOD) {
            const can_signal_cfg_t *s = &g_cfg.can.signals[fi];
            double v = cs->signals[fi].value;

            if (s->crit_lo > 0 && v <= s->crit_lo) {
                fault |= CAN_FAULT_FLOW_LOW;
                if (level < 2) level = 2;
            } else if (s->warn_lo > 0 && v <= s->warn_lo) {
                fault |= CAN_FAULT_FLOW_LOW;
                if (level < 1) level = 1;
            }
        }

        pn->fault       = fault;
        pn->alarm_level = level;
        pn->cmd_seq     = cn->cmd_seq;
        pn->last_cmd_param = cn->last_cmd_param;
        snprintf(pn->last_cmd, sizeof(pn->last_cmd), "%s", cn->last_cmd);

        all_fault |= fault;
        if (level > all_level) all_level = level;

        /* 流量低且没有电压/电流/通信故障时，下发 boost 加冷却 */
        int protect = (fault & (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE |
                                CAN_FAULT_OVERCURRENT));
        if (cs->link_up && (fault & CAN_FAULT_FLOW_LOW) && !protect &&
            !(fault & CAN_FAULT_COMM_LOST)) {
            if (now - g_can_boost_us[ni] >= 5 * 1000000LL) {
                double param = (level >= 2) ? 100.0 : 60.0;
                can_cmd_send(ni, CAN_CMD_BOOST, param, "flow low");
                g_can_boost_us[ni] = now;
            }
        }

        /* 把 CAN 侧（含快速保护）刚发过的命令同步到全局状态 */
        if (cn->cmd_seq != g_can_seen_cmd_seq[ni]) {
            g_can_seen_cmd_seq[ni] = cn->cmd_seq;
            g_snap.can_cmd_seq++;
            snprintf(g_snap.can_cmd_node, sizeof(g_snap.can_cmd_node), "%s", cn->name);
            snprintf(g_snap.can_cmd_name, sizeof(g_snap.can_cmd_name), "%s", cn->last_cmd);
            g_snap.can_cmd_param = cn->last_cmd_param;
        }
    }

    g_snap.can_fault_bits  = all_fault;
    g_snap.can_alarm_level = all_level;
}

static void compute_metrics(const acq_snapshot_t *s, const can_snapshot_t *cs,
                            int64_t now_us)
{
    int64_t    win = (int64_t)g_cfg.proc.window_ms * 1000;
    hist_stat_t st;

    for (int ax = 0; ax < 3; ax++) {
        int vi = g_vel_idx[ax];
        int ai = g_acc_idx[ax];

        if (vi >= 0) {
            hist_stats(&g_hist[vi], now_us, win, &st);
            g_snap.vib_rms[ax]  = st.rms;
            g_snap.vib_peak[ax] = st.peak;
        } else {
            g_snap.vib_rms[ax]  = 0.0;
            g_snap.vib_peak[ax] = 0.0;
        }

        if (ai >= 0) {
            hist_stats(&g_hist[ai], now_us, win, &st);
            g_snap.acc_rms[ax] = st.rms;
        } else {
            g_snap.acc_rms[ax] = 0.0;
        }
    }

    if (g_temp_idx >= 0 && g_temp_idx < s->point_count) {
        g_snap.temp_c = s->points[g_temp_idx].value;
        hist_stats(&g_hist[g_temp_idx], now_us, win, &st);
        if (st.n >= 2 && st.last_ts > st.first_ts)
            g_snap.temp_rate_c_s =
                (st.last - st.first) / ((double)(st.last_ts - st.first_ts) / 1e6);
        else
            g_snap.temp_rate_c_s = 0.0;
    } else {
        g_snap.temp_c = 0.0;
        g_snap.temp_rate_c_s = 0.0;
    }

    double sev = 0.0;
    for (int ax = 0; ax < 3; ax++)
        if (g_snap.vib_rms[ax] > sev) sev = g_snap.vib_rms[ax];
    g_snap.severity = sev;

    if (sev < g_cfg.proc.vib_class_b)      g_snap.severity_class = 1;
    else if (sev < g_cfg.proc.vib_class_c) g_snap.severity_class = 2;
    else if (sev < g_cfg.proc.vib_class_d) g_snap.severity_class = 3;
    else                                   g_snap.severity_class = 4;

    int bad = 0;
    for (int i = 0; i < s->point_count; i++)
        if (s->points[i].quality != Q_GOOD) bad++;

    if (g_cfg.can.enabled && cs) {
        for (int i = 0; i < cs->signal_count; i++)
            if (cs->signals[i].quality != Q_GOOD) bad++;
        for (int i = 0; i < cs->signal_count; i++)
            if (cs->signals[i].ts_utc_us > g_snap.ts_utc_us)
                g_snap.ts_utc_us = cs->signals[i].ts_utc_us;
    }
    g_snap.bad_points = bad;

    for (int i = 0; i < s->point_count; i++)
        if (s->points[i].ts_utc_us > g_snap.ts_utc_us)
            g_snap.ts_utc_us = s->points[i].ts_utc_us;
}

static int condition_true(alarm_cond_t c, double v, double thr)
{
    switch (c) {
    case ALARM_COND_GT: return v >  thr;
    case ALARM_COND_GE: return v >= thr;
    case ALARM_COND_LT: return v <  thr;
    case ALARM_COND_LE: return v <= thr;
    default:            return 0;
    }
}

/* raw < 0 表示数据质量无效，保持原状态 */
static void alarm_update(alarm_rt_t *rt, const alarm_rule_cfg_t *r,
                         int raw, double val, int64_t now_us)
{
    if (raw < 0) return;

    if (rt->active) {
        int release;

        if (r->cond == ALARM_COND_STALE) {
            release = !raw;
        } else {
            double h = r->hysteresis;
            switch (r->cond) {
            case ALARM_COND_GT: release = (val <= r->threshold - h); break;
            case ALARM_COND_GE: release = (val <  r->threshold - h); break;
            case ALARM_COND_LT: release = (val >= r->threshold + h); break;
            case ALARM_COND_LE: release = (val >  r->threshold + h); break;
            default:            release = !raw; break;
            }
        }

        if (release) {
            rt->active = 0;
            rt->acked  = 0;
            rt->pending_since_us = 0;
            LOG_I("proc", "alarm cleared: %s", r->name);
        }
        return;
    }

    if (!raw) {
        rt->pending_since_us = 0;
        return;
    }

    if (rt->pending_since_us == 0) rt->pending_since_us = now_us;
    if (now_us - rt->pending_since_us >= (int64_t)r->duration_ms * 1000) {
        rt->active = 1;
        rt->active_since_us = now_us;
        rt->acked  = 0;
        rt->pending_since_us = 0;
        LOG_W("proc", "alarm active: %s level=%d value=%.3f", r->name, r->level, val);
    }
}

static void eval_alarms(const acq_snapshot_t *s, int64_t now_us)
{
    uint32_t bits  = 0;
    int      count = 0;
    int      level = 0;
    char     list[128];
    size_t   off   = 0;
    list[0] = '\0';

    for (int r = 0; r < g_cfg.alarm_rule_count; r++) {
        if (!g_rule_enabled[r]) continue;

        const alarm_rule_cfg_t *rule = &g_cfg.alarms[r];
        int    pi  = g_rule_point[r];
        int    raw = -1;
        double val = 0.0;

        if (rule->cond == ALARM_COND_STALE) {
            int bad = 0;
            if (pi >= 0) {
                bad = (s->points[pi].quality != Q_GOOD);
            } else {
                for (int i = 0; i < s->point_count; i++)
                    if (s->points[i].quality != Q_GOOD) { bad = 1; break; }
            }
            raw = bad;
            val = bad ? 1.0 : 0.0;
        } else if (pi >= 0 && s->points[pi].quality == Q_GOOD) {
            val = s->points[pi].value;
            raw = condition_true(rule->cond, val, rule->threshold);
        }

        alarm_update(&g_alarm[r], rule, raw, val, now_us);

        if (g_alarm[r].active) {
            count++;
            if (rule->level > level) level = rule->level;

            if (r < 32) bits |= (uint32_t)1u << r;

            if (off < sizeof(list) - 1) {
                int w = snprintf(list + off, sizeof(list) - off, "%s%s",
                                 off ? "," : "", rule->name);
                if (w > 0) {
                    off += (size_t)w;
                    if (off >= sizeof(list)) off = sizeof(list) - 1;
                }
            }
        }
    }

    g_snap.alarm_bits         = bits;
    g_snap.alarm_active_count = count;
    g_snap.alarm_level        = level;
    snprintf(g_snap.active_alarms, sizeof(g_snap.active_alarms), "%s", list);
}

static void update_health(const acq_snapshot_t *s)
{
    (void)s;

    int h = 100;

    switch (g_snap.severity_class) {
    case 2: h -= 8;  break;
    case 3: h -= 25; break;
    case 4: h -= 50; break;
    default: break;
    }

    if (g_snap.alarm_active_count > 0) {
        int pen = (g_snap.alarm_level >= 2) ? 15 : 6;
        h -= g_snap.alarm_active_count * pen;
    }

    if (g_snap.bad_points > 0)                 h -= 20;
    if (g_snap.upload_fail_streak >= 3)        h -= 10;

    if (g_cfg.can.enabled) {
        if (g_snap.can_fault_bits &
            (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE | CAN_FAULT_OVERCURRENT))
            h -= 20;
        if (g_snap.can_fault_bits & CAN_FAULT_COMM_LOST)
            h -= 20;
        else if (g_snap.can_fault_bits & CAN_FAULT_FLOW_LOW)
            h -= 8;
    }

    if (h < 0)   h = 0;
    if (h > 100) h = 100;
    g_snap.health = h;
}

static void update_control(const acq_snapshot_t *s)
{
    uint32_t cw = 0;
    int temp_alarm = 0;

    if (g_snap.alarm_active_count > 0) cw |= PROC_CTL_ALARM_ACTIVE;

    for (int r = 0; r < g_cfg.alarm_rule_count; r++) {
        if (!g_alarm[r].active) continue;
        int pi = g_rule_point[r];
        if (pi >= 0 && strstr(g_cfg.points[pi].unit, "degC") != NULL)
            temp_alarm = 1;
    }

    if (g_snap.severity_class >= 3 || temp_alarm)
        cw |= PROC_CTL_COOLING_DEMAND;

    int sensor_fault = 0;
    for (int i = 0; i < s->point_count; i++)
        if (s->points[i].quality != Q_GOOD) { sensor_fault = 1; break; }

    if (g_cfg.can.enabled) {
        for (int ni = 0; ni < g_snap.can_node_count; ni++) {
            if (g_snap.can[ni].quality != Q_GOOD) { sensor_fault = 1; break; }
        }
    }

    if (g_snap.can_fault_bits) cw |= PROC_CTL_CAN_FAULT;
    if (g_snap.can_fault_bits &
        (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE | CAN_FAULT_OVERCURRENT))
        cw |= PROC_CTL_CAN_PROTECT;

    if (sensor_fault)                        cw |= PROC_CTL_SENSOR_FAULT;
    if (g_measure.state == 1)                cw |= PROC_CTL_MEASURE_RUNNING;
    if (g_snap.upload_fail_streak >= 3)      cw |= PROC_CTL_UPLOAD_FAULT;

    g_snap.control_word = cw;

    const char *action = "normal";
    if ((g_snap.can_fault_bits &
         (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE | CAN_FAULT_OVERCURRENT)) &&
        g_snap.can_alarm_level >= 2)             action = "stop_and_inspect";
    else if (g_snap.can_fault_bits & CAN_FAULT_COMM_LOST) action = "check_sensor";
    else if (sensor_fault)                       action = "check_sensor";
    else if (g_snap.severity_class >= 4)         action = "stop_and_inspect";
    else if (g_snap.severity_class == 3 ||
             g_snap.alarm_level >= 2)            action = "reduce_load";
    else if (g_snap.can_fault_bits &
             (CAN_FAULT_OVERVOLTAGE | CAN_FAULT_UNDERVOLTAGE | CAN_FAULT_OVERCURRENT))
                                                 action = "reduce_load";
    else if (g_snap.can_fault_bits & CAN_FAULT_FLOW_LOW) action = "boost_cooling";
    else if (cw & PROC_CTL_COOLING_DEMAND)       action = "boost_cooling";
    else if (g_snap.alarm_level >= 1 ||
             g_snap.can_fault_bits)              action = "inspect";

    snprintf(g_snap.control_action, sizeof(g_snap.control_action), "%s", action);
}

static void result_init(proc_cmd_result_t *r, uint32_t id, proc_cmd_t cmd,
                        proc_cmd_state_t state, int32_t code, const char *msg)
{
    memset(r, 0, sizeof(*r));
    r->id       = id;
    r->cmd      = cmd;
    r->state    = state;
    r->code     = code;
    r->ts_utc_us = now_utc_us();
    snprintf(r->message, sizeof(r->message), "%s", msg ? msg : "");
}

static void measure_start(uint32_t id, double sec, int64_t now_us)
{
    if (sec <= 0)  sec = 10.0;
    if (sec < 1)   sec = 1.0;
    if (sec > 60)  sec = 60.0;

    g_measure.state  = 1;
    g_measure.end_us = now_us + (int64_t)(sec * 1e6);
    g_measure.cmd_id = id;
    g_measure.seconds = sec;

    g_snap.measure_state    = 1;
    g_snap.measure_remaining_s = sec;
    g_snap.measure_rms      = 0.0;
    g_snap.measure_peak     = 0.0;

    LOG_I("proc", "measurement window started: id=%u %.1fs", id, sec);
}

static void execute_command(const pending_cmd_t *pc, const acq_snapshot_t *s,
                            int64_t now_us, proc_cmd_result_t *res, int *nres)
{
    if (*nres >= PROC_RESULT_MAX) return;

    switch (pc->cmd) {
    case PROC_CMD_START_MEASURE:
        if (g_measure.state == 1) {
            result_init(&res[(*nres)++], pc->id, pc->cmd, PROC_CMD_STATE_REJECTED,
                        -1, "measurement already running");
        } else {
            measure_start(pc->id, pc->param, now_us);
            result_init(&res[(*nres)++], pc->id, pc->cmd, PROC_CMD_STATE_RUNNING,
                        0, "measurement window started");
        }
        break;

    case PROC_CMD_ACK_ALARM: {
        int n = 0;
        for (int r = 0; r < g_cfg.alarm_rule_count; r++) {
            if (g_alarm[r].active && !g_alarm[r].acked) {
                g_alarm[r].acked  = 1;
                g_alarm[r].ack_us = now_us;
                n++;
            }
        }
        char msg[96];
        if (n > 0)
            snprintf(msg, sizeof(msg), "acknowledged %d alarm(s)", n);
        else
            snprintf(msg, sizeof(msg), "no active alarm");
        result_init(&res[(*nres)++], pc->id, pc->cmd, PROC_CMD_STATE_DONE, 0, msg);
        break;
    }

    case PROC_CMD_SELF_TEST: {
        int bad = 0;
        for (int i = 0; i < s->point_count; i++)
            if (s->points[i].quality != Q_GOOD) bad++;

        char msg[96];
        if (bad == 0)
            snprintf(msg, sizeof(msg), "self-test ok: %d/%d points good",
                     s->point_count, s->point_count);
        else
            snprintf(msg, sizeof(msg), "self-test fault: %d/%d points bad",
                     bad, s->point_count);

        result_init(&res[(*nres)++], pc->id, pc->cmd,
                    bad ? PROC_CMD_STATE_FAILED : PROC_CMD_STATE_DONE,
                    bad ? 2 : 0, msg);
        break;
    }

    default:
        result_init(&res[(*nres)++], pc->id, pc->cmd, PROC_CMD_STATE_REJECTED,
                    -1, "unknown command");
        break;
    }
}

static void drain_commands(const acq_snapshot_t *s, int64_t now_us,
                           proc_cmd_result_t *res, int *nres)
{
    while (g_q_len > 0 && *nres < PROC_RESULT_MAX) {
        pending_cmd_t pc = g_queue[g_q_head];
        g_q_head = (g_q_head + 1) % PROC_CMD_QUEUE_LEN;
        g_q_len--;
        execute_command(&pc, s, now_us, res, nres);
    }
}

static void update_measure(int64_t now_us, proc_cmd_result_t *res, int *nres)
{
    if (g_measure.state != 1) return;

    int64_t remain = g_measure.end_us - now_us;
    if (remain > 0) {
        g_snap.measure_state       = 1;
        g_snap.measure_remaining_s = (double)remain / 1e6;
        return;
    }

    /* 测量窗口结束，用滑窗特征作为本次测量结果 */
    g_snap.measure_rms  = g_snap.severity;
    g_snap.measure_peak = 0.0;
    for (int ax = 0; ax < 3; ax++)
        if (g_snap.vib_peak[ax] > g_snap.measure_peak)
            g_snap.measure_peak = g_snap.vib_peak[ax];

    g_snap.measure_state       = 2;
    g_snap.measure_remaining_s = 0.0;

    if (*nres < PROC_RESULT_MAX) {
        char msg[96];
        snprintf(msg, sizeof(msg),
                 "measurement done: rms=%.2f mm/s peak=%.2f mm/s",
                 g_snap.measure_rms, g_snap.measure_peak);
        result_init(&res[(*nres)++], g_measure.cmd_id, PROC_CMD_START_MEASURE,
                    PROC_CMD_STATE_DONE, 0, msg);
        LOG_I("proc", "measurement done: id=%u rms=%.2f peak=%.2f",
              g_measure.cmd_id, g_snap.measure_rms, g_snap.measure_peak);
    }

    g_measure.state = 2;
}

static void proc_tick(void)
{
    acq_snapshot_t s;
    can_snapshot_t cs;
    int64_t now = now_mono_us();
    acq_snapshot_get(&s);
    can_acq_snapshot_get(&cs);

    proc_cmd_result_t results[PROC_RESULT_MAX];
    int nres = 0;

    pthread_mutex_lock(&g_mu);

    sample_all(&s, now);
    can_update(&cs, now);

    int64_t eval_us = (int64_t)g_cfg.proc.eval_interval_ms * 1000;
    if (g_last_eval_us == 0 || now - g_last_eval_us >= eval_us) {
        compute_metrics(&s, &cs, now);
        eval_alarms(&s, now);
        update_health(&s);
        g_snap.eval_seq++;
        g_snap.ts_mono_us = now;
        g_last_eval_us = now;
    }

    drain_commands(&s, now, results, &nres);
    update_measure(now, results, &nres);
    update_control(&s);

    pthread_mutex_unlock(&g_mu);

    for (int i = 0; i < nres; i++) {
        pthread_mutex_lock(&g_mu);
        g_snap.last_cmd = results[i];
        pthread_mutex_unlock(&g_mu);

        if (g_cb) g_cb(&results[i], g_cb_user);
    }
}

static void *proc_thread_main(void *arg)
{
    (void)arg;

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_run) {
        proc_tick();

        next.tv_nsec += PROC_TICK_MS * 1000000L;
        while (next.tv_nsec >= 1000000000L) {
            next.tv_nsec -= 1000000000L;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

int proc_start(const app_config_t *cfg)
{
    g_cfg = *cfg;

    memset(&g_snap, 0, sizeof(g_snap));
    memset(g_hist, 0, sizeof(g_hist));
    memset(g_last_seq, 0, sizeof(g_last_seq));
    memset(g_seen, 0, sizeof(g_seen));
    memset(g_alarm, 0, sizeof(g_alarm));
    memset(&g_measure, 0, sizeof(g_measure));
    memset(g_queue, 0, sizeof(g_queue));
    memset(g_can_seen_cmd_seq, 0, sizeof(g_can_seen_cmd_seq));

    g_q_head = g_q_tail = g_q_len = 0;
    g_next_cmd_id = 1;
    g_last_eval_us = 0;
    g_snap.health = 100;

    build_map();
    build_can_map();

    g_run = 1;
    if (pthread_create(&g_tid, NULL, proc_thread_main, NULL) != 0) {
        g_run = 0;
        return -1;
    }
    return 0;
}

void proc_stop(void)
{
    if (!g_run) return;
    g_run = 0;
    pthread_join(g_tid, NULL);
}

void proc_snapshot_get(proc_snapshot_t *out)
{
    pthread_mutex_lock(&g_mu);
    *out = g_snap;
    pthread_mutex_unlock(&g_mu);
}

void proc_note_transfer(uint32_t seq, uint32_t ok, uint32_t failed)
{
    (void)ok;

    pthread_mutex_lock(&g_mu);
    g_snap.upload_seq = seq;
    if (failed > 0) {
        g_snap.upload_fail_total += failed;
        if (g_snap.upload_fail_streak < 0xFFFFFFFFu)
            g_snap.upload_fail_streak++;
    } else {
        g_snap.upload_fail_streak = 0;
    }
    pthread_mutex_unlock(&g_mu);
}

uint32_t proc_cmd_submit(proc_cmd_t cmd, double param)
{
    if (cmd != PROC_CMD_START_MEASURE &&
        cmd != PROC_CMD_ACK_ALARM &&
        cmd != PROC_CMD_SELF_TEST)
        return 0;

    pthread_mutex_lock(&g_mu);

    if (g_q_len >= PROC_CMD_QUEUE_LEN) {
        pthread_mutex_unlock(&g_mu);
        return 0;
    }

    uint32_t id = g_next_cmd_id++;
    if (g_next_cmd_id == 0) g_next_cmd_id = 1;

    pending_cmd_t *pc = &g_queue[g_q_tail];
    pc->id        = id;
    pc->cmd       = cmd;
    pc->param     = param;
    pc->submit_us = now_mono_us();

    g_q_tail = (g_q_tail + 1) % PROC_CMD_QUEUE_LEN;
    g_q_len++;

    pthread_mutex_unlock(&g_mu);

    LOG_I("proc", "command queued: id=%u cmd=%s param=%.1f", id, proc_cmd_name(cmd), param);
    return id;
}

void proc_set_cmd_result_cb(proc_cmd_result_cb_t cb, void *user)
{
    g_cb      = cb;
    g_cb_user = user;
}

const char *proc_cmd_name(proc_cmd_t cmd)
{
    switch (cmd) {
    case PROC_CMD_START_MEASURE: return "start_measure";
    case PROC_CMD_ACK_ALARM:     return "ack_alarm";
    case PROC_CMD_SELF_TEST:     return "self_test";
    default:                     return "none";
    }
}

const char *proc_cmd_state_name(proc_cmd_state_t s)
{
    switch (s) {
    case PROC_CMD_STATE_PENDING:  return "pending";
    case PROC_CMD_STATE_RUNNING:  return "running";
    case PROC_CMD_STATE_DONE:     return "done";
    case PROC_CMD_STATE_REJECTED: return "rejected";
    case PROC_CMD_STATE_FAILED:   return "failed";
    default:                      return "unknown";
    }
}
