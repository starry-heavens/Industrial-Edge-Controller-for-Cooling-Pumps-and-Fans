#define _POSIX_C_SOURCE 200809L

#include "log.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#define LOG_DIR   "/userdata/linux_application_project/edge/log"
#define LOG_FILE  "/userdata/linux_application_project/edge/log/app.log"
#define MAX_LINE  512

static FILE           *g_fp        = NULL;
static pthread_mutex_t g_lock      = PTHREAD_MUTEX_INITIALIZER;
static pthread_t       g_tid;
static volatile int    g_run       = 0;
static log_level_t     g_min_level = LOG_LEVEL_INFO;

static const char *level_str(log_level_t level)
{
    switch (level) {
    case LOG_LEVEL_DEBUG: return "DEBUG";
    case LOG_LEVEL_INFO:  return "INFO";
    case LOG_LEVEL_WARN:  return "WARN";
    case LOG_LEVEL_ERROR: return "ERROR";
    default:              return "?????";
    }
}

static void fmt_time(char *buf, size_t len)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             ts.tv_nsec / 1000000);
}

static void *log_thread(void *arg)
{
    (void)arg;
    while (g_run) {
        sleep(1);
        pthread_mutex_lock(&g_lock);
        if (g_fp) { fflush(g_fp); fsync(fileno(g_fp)); }
        pthread_mutex_unlock(&g_lock);
    }
    return NULL;
}

int log_init(const char *path, log_level_t min_level)
{
    if (mkdir(LOG_DIR, 0755) != 0 && errno != EEXIST) return -1;

    g_fp = fopen(path ? path : LOG_FILE, "a");
    if (!g_fp) return -1;

    g_min_level = min_level;
    g_run = 1;
    if (pthread_create(&g_tid, NULL, log_thread, NULL) != 0) {
        fclose(g_fp); g_fp = NULL; g_run = 0;
        return -1;
    }
    return 0;
}

void log_write(log_level_t level, const char *module, const char *fmt, ...)
{
    if (level < g_min_level || !g_fp) return;

    char timebuf[32];
    fmt_time(timebuf, sizeof(timebuf));

    char line[MAX_LINE];
    int off = snprintf(line, sizeof(line), "%s [%s] [%s] ",
                       timebuf, level_str(level), module ? module : "-");
    if (off < 0 || (size_t)off >= sizeof(line)) return;

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + off, sizeof(line) - (size_t)off, fmt, ap);
    va_end(ap);

    size_t used = strlen(line);
    if (used + 1 < sizeof(line)) { line[used++] = '\n'; line[used] = '\0'; }

    pthread_mutex_lock(&g_lock);
    if (g_fp) fputs(line, g_fp);
    pthread_mutex_unlock(&g_lock);
}

void log_flush(void)
{
    pthread_mutex_lock(&g_lock);
    if (g_fp) { fflush(g_fp); fsync(fileno(g_fp)); }
    pthread_mutex_unlock(&g_lock);
}

void log_close(void)
{
    if (!g_run) return;
    g_run = 0;
    pthread_join(g_tid, NULL);

    pthread_mutex_lock(&g_lock);
    if (g_fp) { fflush(g_fp); fclose(g_fp); g_fp = NULL; }
    pthread_mutex_unlock(&g_lock);
}