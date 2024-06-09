#ifndef _LOGGING_H_
#define _LOGGING_H_

#include <stdarg.h>
#include <stdio.h>

typedef enum {
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG,
} log_level_e;

void _fetcher_log(FILE *stream, log_level_e lvl, const char *file, const char *func, int line, const char *fmt_string,
                  ...);

#define fetcher_log(stream, lvl, fmt_string, ...)                                                                      \
    _fetcher_log((stream), (lvl), __FILE__, __FUNCTION__, __LINE__, (fmt_string), ##__VA_ARGS__)

#endif // _LOGGING_H_
