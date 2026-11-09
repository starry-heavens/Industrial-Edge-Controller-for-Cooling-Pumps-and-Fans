#ifndef ACQ_THREAD_H
#define ACQ_THREAD_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum { Q_GOOD = 0, Q_STALE, Q_BAD, Q_UNCERTAIN } quality_t;

typedef struct {
    double    value;
    quality_t quality;
    int64_t   ts_mono_us;
    int64_t   ts_utc_us;
    uint32_t  seq;
} point_value_t;

typedef struct {
    point_value_t points[MAX_POINTS];
    int           point_count;
    uint32_t      dev_seq;
    uint32_t      fail_count;
    int64_t       last_good_us;
} acq_snapshot_t;

int  acq_thread_start(const app_config_t *cfg);
void acq_thread_stop(void);
void acq_snapshot_get(acq_snapshot_t *out);

/* 心跳：距上次心跳多少毫秒；-1 表示还没启动 */
long long acq_heartbeat_age_ms(void);

#endif