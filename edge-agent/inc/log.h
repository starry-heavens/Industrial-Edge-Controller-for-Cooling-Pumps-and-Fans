#ifndef LOG_H
#define LOG_H

#include <stdarg.h>

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

int  log_init(const char *path, log_level_t min_level);
void log_write(log_level_t level, const char *module, const char *fmt, ...);
void log_flush(void);
void log_close(void);

#define LOG_D(mod, ...) log_write(LOG_LEVEL_DEBUG, mod, __VA_ARGS__)
#define LOG_I(mod, ...) log_write(LOG_LEVEL_INFO,  mod, __VA_ARGS__)
#define LOG_W(mod, ...) log_write(LOG_LEVEL_WARN,  mod, __VA_ARGS__)
#define LOG_E(mod, ...) log_write(LOG_LEVEL_ERROR, mod, __VA_ARGS__)

#endif