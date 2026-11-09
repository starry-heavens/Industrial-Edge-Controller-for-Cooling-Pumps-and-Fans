#define _POSIX_C_SOURCE 200809L

#include "monitor.h"
#include "acq_thread.h"
#include "log.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>        /* _exit */
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>

#define APP_VERSION                 "0.1.0"
#define DISK_PATH                   "/userdata"
#define SYSLOG_INTERVAL_S           10
#define ACQ_HEARTBEAT_TIMEOUT_MS    5000   /* 采集线程 5 秒没心跳判定卡死 */

static char g_status_dir[64];
static char g_status_file[128];
static char g_status_tmp[128];

static pthread_t    g_tid;
static volatile int g_run = 0;
static unsigned long long g_prev_total = 0;
static unsigned long long g_prev_idle  = 0;

static long long mono_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}

static int pick_status_dir(void)
{
    const char *cands[] = { "/run/edge", "/dev/shm/edge", "/tmp/edge" };
    for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]); i++) {
        if (mkdir(cands[i], 0755) == 0 || errno == EEXIST) {
            snprintf(g_status_dir,  sizeof(g_status_dir),  "%s", cands[i]);
            snprintf(g_status_file, sizeof(g_status_file), "%s/status.json", cands[i]);
            snprintf(g_status_tmp,  sizeof(g_status_tmp),  "%s/status.json.tmp", cands[i]);
            return 0;
        }
    }
    return -1;
}

static int read_cpu(double *usage)
{
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1;

    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    int n = fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu",
                   &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal);
    fclose(f);
    if (n < 4) return -1;

    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;
    unsigned long long idle_all = idle + iowait;

    if (g_prev_total == 0) { g_prev_total = total; g_prev_idle = idle_all; return -1; }

    unsigned long long dt = total - g_prev_total;
    unsigned long long di = idle_all - g_prev_idle;
    g_prev_total = total;
    g_prev_idle  = idle_all;
    if (dt == 0) return -1;
    *usage = (double)(dt - di) * 100.0 / (double)dt;
    return 0;
}

static void read_mem(double *used_mb, double *total_mb, double *pct)
{
    FILE *f = fopen("/proc/meminfo", "r");
    if (!f) return;

    char key[64], unit[16];
    unsigned long val;
    unsigned long mem_total = 0, mem_avail = 0;

    while (fscanf(f, "%63s %lu %15s", key, &val, unit) == 3) {
        if (strcmp(key, "MemTotal:") == 0)     mem_total = val;
        if (strcmp(key, "MemAvailable:") == 0) mem_avail = val;
    }
    fclose(f);
    if (mem_total == 0) return;

    double t = mem_total / 1024.0;
    double a = mem_avail / 1024.0;
    *total_mb = t;
    *used_mb  = t - a;
    *pct      = (t - a) * 100.0 / t;
}

static void read_load(double *l1, double *l5, double *l15)
{
    FILE *f = fopen("/proc/loadavg", "r");
    if (!f) return;
    if (fscanf(f, "%lf %lf %lf", l1, l5, l15) != 3) *l1 = *l5 = *l15 = 0.0;
    fclose(f);
}

static double read_temp(void)
{
    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return -1.0;
    long milli = 0;
    if (fscanf(f, "%ld", &milli) != 1) { fclose(f); return -1.0; }
    fclose(f);
    return milli / 1000.0;
}

static void read_disk(double *used_gb, double *total_gb, double *pct)
{
    struct statvfs st;
    if (statvfs(DISK_PATH, &st) != 0) return;

    unsigned long long total = (unsigned long long)st.f_blocks * st.f_frsize;
    unsigned long long avail = (unsigned long long)st.f_bavail * st.f_frsize;
    if (total == 0) return;

    double t = total / (1024.0 * 1024.0 * 1024.0);
    double a = avail / (1024.0 * 1024.0 * 1024.0);
    *total_gb = t;
    *used_gb  = t - a;
    *pct      = (t - a) * 100.0 / t;
}

static void write_status(void)
{
    acq_snapshot_t snap;
    acq_snapshot_get(&snap);

    long long now_us = mono_us();
    long     uptime_s = (long)(now_us / 1000000LL);

    double cpu = -1.0;
    int have_cpu = (read_cpu(&cpu) == 0);
    double mem_used = 0, mem_total = 0, mem_pct = 0;
    read_mem(&mem_used, &mem_total, &mem_pct);
    double l1 = 0, l5 = 0, l15 = 0;
    read_load(&l1, &l5, &l15);
    double temp = read_temp();
    double disk_used = 0, disk_total = 0, disk_pct = 0;
    read_disk(&disk_used, &disk_total, &disk_pct);

    FILE *fp = fopen(g_status_tmp, "w");
    if (!fp) return;

    time_t now_wall = time(NULL);
    struct tm tm;
    gmtime_r(&now_wall, &tm);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &tm);

    fprintf(fp,
        "{\n"
        "  \"version\": \"%s\",\n"
        "  \"uptime_s\": %ld,\n"
        "  \"ts\": \"%s\",\n"
        "  \"system\": {\n"
        "    \"cpu_pct\": %.1f,\n"
        "    \"mem_used_mb\": %.1f,\n"
        "    \"mem_total_mb\": %.1f,\n"
        "    \"mem_pct\": %.1f,\n"
        "    \"load1\": %.2f,\n"
        "    \"load5\": %.2f,\n"
        "    \"load15\": %.2f,\n"
        "    \"temp_c\": %.1f,\n"
        "    \"disk_used_gb\": %.2f,\n"
        "    \"disk_total_gb\": %.2f,\n"
        "    \"disk_pct\": %.1f\n"
        "  },\n"
        "  \"acq\": {\n"
        "    \"dev_seq\": %u,\n"
        "    \"fail_count\": %u,\n"
        "    \"last_good_ago_s\": %.1f\n"
        "  }\n"
        "}\n",
        APP_VERSION, uptime_s, ts,
        have_cpu ? cpu : -1.0,
        mem_used, mem_total, mem_pct,
        l1, l5, l15, temp,
        disk_used, disk_total, disk_pct,
        snap.dev_seq, snap.fail_count,
        snap.last_good_us ? (now_us - snap.last_good_us) / 1e6 : -1.0);

    fclose(fp);
    if (rename(g_status_tmp, g_status_file) != 0) remove(g_status_tmp);
}

static void log_system(void)
{
    double cpu = -1.0;
    int have_cpu = (read_cpu(&cpu) == 0);
    double mem_used = 0, mem_total = 0, mem_pct = 0;
    read_mem(&mem_used, &mem_total, &mem_pct);
    double l1 = 0, l5 = 0, l15 = 0;
    read_load(&l1, &l5, &l15);
    double temp = read_temp();
    double disk_used = 0, disk_total = 0, disk_pct = 0;
    read_disk(&disk_used, &disk_total, &disk_pct);

    char cpu_buf[16];
    if (have_cpu) snprintf(cpu_buf, sizeof(cpu_buf), "%.1f%%", cpu);
    else          snprintf(cpu_buf, sizeof(cpu_buf), "n/a");

    acq_snapshot_t s;
    acq_snapshot_get(&s);

    LOG_I("sys",
          "cpu=%s mem=%.1f/%.0fMB(%.1f%%) load=%.2f/%.2f/%.2f temp=%.1fC disk=%.2f/%.2fGB(%.1f%%) dev_seq=%u fail=%u",
          cpu_buf, mem_used, mem_total, mem_pct,
          l1, l5, l15, temp,
          disk_used, disk_total, disk_pct,
          s.dev_seq, s.fail_count);
}

static void *monitor_thread(void *arg)
{
    (void)arg;
    int elapsed = 0;

    while (g_run) {
        sleep(1);
        if (!g_run) break;

        /* 采集线程心跳检查：超过阈值判定卡死，整个进程退出让 systemd 拉起 */
        long long age = acq_heartbeat_age_ms();
        if (age >= 0 && age > ACQ_HEARTBEAT_TIMEOUT_MS) {
            LOG_E("monitor", "acq heartbeat lost: %lld ms, exiting for restart", age);
            log_flush();
            _exit(1);
        }

        write_status();

        elapsed++;
        if (elapsed >= SYSLOG_INTERVAL_S) {
            elapsed = 0;
            log_system();
        }
    }
    return NULL;
}

int monitor_start(void)
{
    if (pick_status_dir() != 0) return -1;

    double dummy;
    read_cpu(&dummy);

    g_run = 1;
    if (pthread_create(&g_tid, NULL, monitor_thread, NULL) != 0) {
        g_run = 0;
        return -1;
    }
    return 0;
}

void monitor_stop(void)
{
    if (!g_run) return;
    g_run = 0;
    pthread_join(g_tid, NULL);
}