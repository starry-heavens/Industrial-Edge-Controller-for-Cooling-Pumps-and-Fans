#define _GNU_SOURCE
#include "config.h"
#include "json_lite.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) { fclose(f); return NULL; }
    char *buf = malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = '\0';
    return buf;
}

static point_type_t parse_type(const char *s)
{
    if (!s) return PT_INT16;
    if (!strcmp(s, "uint16")) return PT_UINT16;
    if (!strcmp(s, "int32"))  return PT_INT32;
    if (!strcmp(s, "uint32")) return PT_UINT32;
    if (!strcmp(s, "float"))  return PT_FLOAT;
    return PT_INT16;
}

/* "> 4.5" / ">= 4.5" / "< 1" / "<= 1" / "stale" */
static void parse_alarm_condition(const char *s, alarm_rule_cfg_t *r)
{
    r->cond = ALARM_COND_NONE;
    if (!s) return;

    while (*s == ' ' || *s == '\t') s++;

    if (strncmp(s, "stale", 5) == 0) {
        r->cond = ALARM_COND_STALE;
        return;
    }

    double thr;
    if (s[0] == '>' && s[1] == '=')      { r->cond = ALARM_COND_GE; thr = strtod(s + 2, NULL); }
    else if (s[0] == '<' && s[1] == '=') { r->cond = ALARM_COND_LE; thr = strtod(s + 2, NULL); }
    else if (s[0] == '>')                { r->cond = ALARM_COND_GT; thr = strtod(s + 1, NULL); }
    else if (s[0] == '<')                { r->cond = ALARM_COND_LT; thr = strtod(s + 1, NULL); }
    else return;

    r->threshold = thr;
}

static int parse_alarm_level(const char *s)
{
    if (s && !strcmp(s, "critical")) return 2;
    return 1;
}

static void parse_processing(const json_value_t *root, app_config_t *cfg)
{
    const json_value_t *pr = json_object_get(root, "processing");

    cfg->proc.window_ms        = (int)json_number(json_object_get(pr, "window_ms"), 10000);
    cfg->proc.eval_interval_ms = (int)json_number(json_object_get(pr, "eval_interval_ms"), 500);
    cfg->proc.vib_class_b      = json_number(json_object_get(pr, "vib_class_b"), 1.8);
    cfg->proc.vib_class_c      = json_number(json_object_get(pr, "vib_class_c"), 4.5);
    cfg->proc.vib_class_d      = json_number(json_object_get(pr, "vib_class_d"), 11.2);

    if (cfg->proc.window_ms < 1000)    cfg->proc.window_ms = 1000;
    if (cfg->proc.window_ms > 60000)   cfg->proc.window_ms = 60000;
    if (cfg->proc.eval_interval_ms < 100)  cfg->proc.eval_interval_ms = 100;
    if (cfg->proc.eval_interval_ms > 5000) cfg->proc.eval_interval_ms = 5000;
    if (cfg->proc.vib_class_b <= 0) cfg->proc.vib_class_b = 1.8;
    if (cfg->proc.vib_class_c <= cfg->proc.vib_class_b)
        cfg->proc.vib_class_c = cfg->proc.vib_class_b * 2.5;
    if (cfg->proc.vib_class_d <= cfg->proc.vib_class_c)
        cfg->proc.vib_class_d = cfg->proc.vib_class_c * 2.5;
}

static void parse_history(const json_value_t *root, app_config_t *cfg)
{
    const json_value_t *h = json_object_get(root, "history");

    cfg->hist.enabled            = json_bool(json_object_get(h, "enabled"), true);
    cfg->hist.sample_interval_ms = (int)json_number(json_object_get(h, "sample_interval_ms"), 1000);
    cfg->hist.max_values         = (int)json_number(json_object_get(h, "max_values"), 3600);

    if (cfg->hist.sample_interval_ms < 200)   cfg->hist.sample_interval_ms = 200;
    if (cfg->hist.sample_interval_ms > 60000) cfg->hist.sample_interval_ms = 60000;
    if (cfg->hist.max_values < 60)            cfg->hist.max_values = 60;
    if (cfg->hist.max_values > 86400)         cfg->hist.max_values = 86400;
}

static void parse_alarm_rules(const json_value_t *root, app_config_t *cfg)
{
    const json_value_t *rules = json_object_get(json_object_get(root, "alarm"), "rules");
    size_t n = json_array_size(rules);

    for (size_t i = 0; i < n && cfg->alarm_rule_count < MAX_ALARM_RULES; i++) {
        const json_value_t *rj = json_array_get(rules, i);
        if (json_type(rj) != JSON_OBJECT) continue;

        alarm_rule_cfg_t *r = &cfg->alarms[cfg->alarm_rule_count];
        memset(r, 0, sizeof(*r));
        snprintf(r->name, sizeof(r->name), "%s",
                 json_string(json_object_get(rj, "name"), "alarm"));
        snprintf(r->point, sizeof(r->point), "%s",
                 json_string(json_object_get(rj, "point"), "*"));
        parse_alarm_condition(json_string(json_object_get(rj, "condition"), NULL), r);
        if (r->cond == ALARM_COND_NONE) {
            fprintf(stderr, "config: alarm rule '%s' has invalid condition, skipped\n", r->name);
            continue;
        }
        r->level       = parse_alarm_level(json_string(json_object_get(rj, "level"), "warning"));
        r->hysteresis  = json_number(json_object_get(rj, "hysteresis"), 0.0);
        r->duration_ms = (int)json_number(json_object_get(rj, "duration_ms"), 0);
        if (r->hysteresis < 0)  r->hysteresis = 0;
        if (r->duration_ms < 0) r->duration_ms = 0;
        cfg->alarm_rule_count++;
    }
}

static can_role_t parse_can_role(const char *s)
{
    if (!s) return CAN_ROLE_NONE;
    if (!strcmp(s, "flow"))    return CAN_ROLE_FLOW;
    if (!strcmp(s, "voltage")) return CAN_ROLE_VOLTAGE;
    if (!strcmp(s, "current")) return CAN_ROLE_CURRENT;
    return CAN_ROLE_NONE;
}

static const char *can_role_name(can_role_t r)
{
    switch (r) {
    case CAN_ROLE_FLOW:    return "flow";
    case CAN_ROLE_VOLTAGE: return "voltage";
    case CAN_ROLE_CURRENT: return "current";
    default:               return "none";
    }
}

/*
 * CAN 配置：
 *   "can": {
 *     "device": "can0", "bitrate": 250000, "poll_interval_ms": 200,
 *     "response_timeout_ms": 40, "retries": 2, "stale_after_ms": 1200,
 *     "tx_enabled": false,
 *     "nodes": [ { "name": "cooling_pump", "node_id": "0x10", "cmd_id": "0x210",
 *                  "signals": [ { "name": "pump_voltage", "role": "voltage", ... } ] } ]
 *   }
 */
static void parse_can(const json_value_t *root, app_config_t *cfg)
{
    const json_value_t *c = json_object_get(root, "can");

    memset(&cfg->can, 0, sizeof(cfg->can));
    if (!c) return;

    cfg->can.enabled = json_bool(json_object_get(c, "enabled"), false);
    snprintf(cfg->can.device, sizeof(cfg->can.device), "%s",
             json_string(json_object_get(c, "device"), "can0"));
    cfg->can.bitrate             = (int)json_long(json_object_get(c, "bitrate"), 250000);
    cfg->can.poll_interval_ms    = (int)json_number(json_object_get(c, "poll_interval_ms"), 200);
    cfg->can.response_timeout_ms = (int)json_number(json_object_get(c, "response_timeout_ms"), 40);
    cfg->can.retries             = (int)json_number(json_object_get(c, "retries"), 2);
    cfg->can.stale_after_ms      = (int)json_number(json_object_get(c, "stale_after_ms"), 1200);
    cfg->can.tx_enabled          = json_bool(json_object_get(c, "tx_enabled"), false);

    if (cfg->can.poll_interval_ms < 20)     cfg->can.poll_interval_ms = 20;
    if (cfg->can.poll_interval_ms > 10000)  cfg->can.poll_interval_ms = 10000;
    if (cfg->can.response_timeout_ms < 5)   cfg->can.response_timeout_ms = 5;
    if (cfg->can.response_timeout_ms > 2000) cfg->can.response_timeout_ms = 2000;
    if (cfg->can.retries < 0)               cfg->can.retries = 0;
    if (cfg->can.retries > 5)               cfg->can.retries = 5;
    if (cfg->can.stale_after_ms < 100)      cfg->can.stale_after_ms = 100;
    if (cfg->can.stale_after_ms > 60000)    cfg->can.stale_after_ms = 60000;

    const json_value_t *nodes = json_object_get(c, "nodes");
    size_t nn = json_array_size(nodes);

    for (size_t i = 0; i < nn && cfg->can.node_count < MAX_CAN_NODES; i++) {
        const json_value_t *nj = json_array_get(nodes, i);
        if (json_type(nj) != JSON_OBJECT) continue;

        can_node_cfg_t *n = &cfg->can.nodes[cfg->can.node_count];
        memset(n, 0, sizeof(*n));
        snprintf(n->name, sizeof(n->name), "%s",
                 json_string(json_object_get(nj, "name"), "can_node"));
        n->node_id = (uint32_t)json_long(json_object_get(nj, "node_id"), 0);
        n->cmd_id  = (uint32_t)json_long(json_object_get(nj, "cmd_id"), 0);

        const json_value_t *sigs = json_object_get(nj, "signals");
        size_t sn = json_array_size(sigs);

        for (size_t j = 0; j < sn && cfg->can.signal_count < MAX_CAN_SIGNALS; j++) {
            const json_value_t *sj = json_array_get(sigs, j);
            if (json_type(sj) != JSON_OBJECT) continue;

            can_signal_cfg_t *s = &cfg->can.signals[cfg->can.signal_count];
            memset(s, 0, sizeof(*s));

            s->node = cfg->can.node_count;
            snprintf(s->name, sizeof(s->name), "%s",
                     json_string(json_object_get(sj, "name"), "can_signal"));
            s->role = parse_can_role(json_string(json_object_get(sj, "role"), NULL));

            s->req_id  = (uint32_t)json_long(json_object_get(sj, "req_id"), 0);
            s->resp_id = (uint32_t)json_long(json_object_get(sj, "resp_id"), 0);

            int byte = (int)json_long(json_object_get(sj, "byte"), 0);
            int len  = (int)json_long(json_object_get(sj, "len"), 2);
            if (byte < 0) byte = 0;
            if (byte > 7) byte = 7;
            if (len != 1 && len != 2 && len != 4) len = 2;
            if (byte + len > 8) {
                fprintf(stderr, "config: can signal '%s' byte+len > 8, skipped\n", s->name);
                continue;
            }
            s->start_byte = (uint8_t)byte;
            s->len        = (uint8_t)len;

            s->type   = parse_type(json_string(json_object_get(sj, "type"), "uint16"));
            s->little_endian =
                (strcmp(json_string(json_object_get(sj, "endian"), "big"), "little") == 0);
            s->scale  = json_number(json_object_get(sj, "scale"), 1.0);
            s->offset = json_number(json_object_get(sj, "offset"), 0.0);
            snprintf(s->unit, sizeof(s->unit), "%s",
                     json_string(json_object_get(sj, "unit"), ""));
            s->min = json_number(json_object_get(sj, "min"), 0.0);
            s->max = json_number(json_object_get(sj, "max"), 0.0);

            s->priority = (strcmp(json_string(json_object_get(sj, "priority"), "normal"),
                                  "high") == 0) ? 1 : 0;
            s->warn_hi = json_number(json_object_get(sj, "warn_hi"), 0.0);
            s->crit_hi = json_number(json_object_get(sj, "crit_hi"), 0.0);
            s->warn_lo = json_number(json_object_get(sj, "warn_lo"), 0.0);
            s->crit_lo = json_number(json_object_get(sj, "crit_lo"), 0.0);

            cfg->can.signal_count++;
        }
        cfg->can.node_count++;
    }

    if (cfg->can.enabled && cfg->can.signal_count == 0) {
        fprintf(stderr, "config: can enabled but no valid signal, disabled\n");
        cfg->can.enabled = 0;
    }
}

int config_load(const char *path, app_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    char *text = read_file(path);
    if (!text) { perror("read config"); return -1; }

    json_value_t *root = json_parse(text);
    free(text);
    if (!root || json_type(root) != JSON_OBJECT) {
        fprintf(stderr, "config: json parse failed\n");
        json_free(root);
        return -1;
    }

    const json_value_t *serial = json_object_get(root, "serial");
    snprintf(cfg->device, sizeof(cfg->device), "%s",
             json_string(json_object_get(serial, "device"), "/dev/ttyS3"));
    cfg->baudrate             = (int)json_number(json_object_get(serial, "baudrate"), 9600);
    cfg->databits             = (int)json_number(json_object_get(serial, "databits"), 8);
    cfg->stopbits             = (int)json_number(json_object_get(serial, "stopbits"), 1);
    snprintf(cfg->parity, sizeof(cfg->parity), "%s",
             json_string(json_object_get(serial, "parity"), "none"));
    cfg->timeout_ms           = (int)json_number(json_object_get(serial, "timeout_ms"), 120);
    cfg->inter_frame_delay_ms = (int)json_number(json_object_get(serial, "inter_frame_delay_ms"), 4);
    cfg->retries              = (int)json_number(json_object_get(serial, "retries"), 2);
    cfg->retry_backoff_ms     = (int)json_number(json_object_get(serial, "retry_backoff_ms"), 50);

    const json_value_t *rs485 = json_object_get(root, "rs485");
    snprintf(cfg->mode, sizeof(cfg->mode), "%s",
             json_string(json_object_get(rs485, "mode"), "auto"));
    cfg->de_gpio          = (int)json_long(json_object_get(rs485, "de_gpio"), -1);
    cfg->de_active_high   = json_bool(json_object_get(rs485, "de_active_high"), true);
    cfg->post_tx_delay_us = (int)json_number(json_object_get(rs485, "post_tx_delay_us"), 500);

    const json_value_t *dev = json_object_get(root, "device");
    snprintf(cfg->dev_name, sizeof(cfg->dev_name), "%s",
             json_string(json_object_get(dev, "name"), "dev0"));
    cfg->slave_id         = (int)json_long(json_object_get(dev, "slave_id"), 1);
    cfg->function         = (int)json_long(json_object_get(dev, "function"), 0x03);
    cfg->poll_interval_ms = (int)json_number(json_object_get(dev, "poll_interval_ms"), 1000);
    cfg->stale_after_ms   = (int)json_number(json_object_get(dev, "stale_after_ms"), 3000);

    const json_value_t *pts = json_object_get(dev, "points");
    size_t n = json_array_size(pts);
    for (size_t i = 0; i < n && cfg->point_count < MAX_POINTS; i++) {
        const json_value_t *pn = json_array_get(pts, i);
        if (json_type(pn) != JSON_OBJECT) continue;
        point_cfg_t *p = &cfg->points[cfg->point_count++];
        snprintf(p->name, sizeof(p->name), "%s",
                 json_string(json_object_get(pn, "name"), "unnamed"));
        p->addr  = (uint16_t)json_long(json_object_get(pn, "addr"), 0);
        p->type  = parse_type(json_string(json_object_get(pn, "type"), "int16"));
        p->scale = json_number(json_object_get(pn, "scale"), 1.0);
        snprintf(p->unit, sizeof(p->unit), "%s",
                 json_string(json_object_get(pn, "unit"), ""));
        p->min   = json_number(json_object_get(pn, "min"), 0.0);
        p->max   = json_number(json_object_get(pn, "max"), 0.0);
    }

    cfg->opcua_port = (int)json_long(json_object_get(json_object_get(root, "opcua"), "port"), 4840);
    if (cfg->opcua_port <= 0 || cfg->opcua_port > 65535) cfg->opcua_port = 4840;

    parse_processing(root, cfg);
    parse_history(root, cfg);
    parse_alarm_rules(root, cfg);
    parse_can(root, cfg);

    json_free(root);
    if (cfg->point_count == 0) return -1;
    return 0;
}

void config_dump(const app_config_t *cfg)
{
    printf("serial : %s @%d %dN%d parity=%s timeout=%dms retries=%d\n",
           cfg->device, cfg->baudrate, cfg->databits, cfg->stopbits,
           cfg->parity, cfg->timeout_ms, cfg->retries);
    printf("rs485  : mode=%s de_gpio=%d active_high=%d post_tx=%dus\n",
           cfg->mode, cfg->de_gpio, cfg->de_active_high, cfg->post_tx_delay_us);
    printf("device : %s slave=%d func=0x%02X poll=%dms stale=%dms\n",
           cfg->dev_name, cfg->slave_id, cfg->function,
           cfg->poll_interval_ms, cfg->stale_after_ms);
    printf("opcua  : port=%d\n", cfg->opcua_port);
    printf("process: window=%dms eval=%dms class B/C/D=%.1f/%.1f/%.1f mm/s\n",
           cfg->proc.window_ms, cfg->proc.eval_interval_ms,
           cfg->proc.vib_class_b, cfg->proc.vib_class_c, cfg->proc.vib_class_d);
    printf("history: %s sample=%dms max=%d/node\n",
           cfg->hist.enabled ? "on" : "off",
           cfg->hist.sample_interval_ms, cfg->hist.max_values);
    for (int i = 0; i < cfg->point_count; i++) {
        const point_cfg_t *p = &cfg->points[i];
        printf("  point[%02d] %-12s addr=0x%04X type=%d scale=%g unit=%s\n",
               i, p->name, p->addr, p->type, p->scale, p->unit);
    }
    for (int i = 0; i < cfg->alarm_rule_count; i++) {
        const alarm_rule_cfg_t *r = &cfg->alarms[i];
        printf("  alarm[%02d] %-24s point=%-12s cond=%d thr=%.3f hyst=%.3f level=%d dur=%dms\n",
               i, r->name, r->point, r->cond, r->threshold,
               r->hysteresis, r->level, r->duration_ms);
    }

    printf("can    : %s dev=%s bitrate=%d poll=%dms timeout=%dms retries=%d tx=%s nodes=%d signals=%d\n",
           cfg->can.enabled ? "on" : "off", cfg->can.device, cfg->can.bitrate,
           cfg->can.poll_interval_ms, cfg->can.response_timeout_ms, cfg->can.retries,
           cfg->can.tx_enabled ? "send" : "print-only",
           cfg->can.node_count, cfg->can.signal_count);
    for (int i = 0; i < cfg->can.node_count; i++) {
        const can_node_cfg_t *n = &cfg->can.nodes[i];
        printf("  can node[%d] %-14s node_id=0x%03X cmd_id=0x%03X\n",
               i, n->name, n->node_id, n->cmd_id);
    }
    for (int i = 0; i < cfg->can.signal_count; i++) {
        const can_signal_cfg_t *s = &cfg->can.signals[i];
        printf("  can sig[%02d] %-16s node=%d role=%-7s req=0x%03X resp=0x%03X "
               "byte=%u len=%u type=%d endian=%s scale=%g unit=%s prio=%s "
               "thr=%.2f/%.2f/%.2f/%.2f\n",
               i, s->name, s->node, can_role_name(s->role), s->req_id, s->resp_id,
               s->start_byte, s->len, s->type, s->little_endian ? "little" : "big",
               s->scale, s->unit, s->priority ? "high" : "normal",
               s->warn_hi, s->crit_hi, s->warn_lo, s->crit_lo);
    }
}