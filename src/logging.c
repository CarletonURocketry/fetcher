#include "logging.h"
#include <stdio.h>
#include <time.h>

#define TIME_STR_BUFSIZE 50
#define TIME_FORMAT "%Y-%m-%dT%H:%M:%S"

const char *LEVEL_STRING[] = {
    [LOG_ERROR] = "ERROR",
    [LOG_WARN] = "WARN",
    [LOG_INFO] = "INFO",
    [LOG_DEBUG] = "DEBUG",
};

/**
 * Logs a message to the file stream.
 * @param stream The file stream to log to.
 * @param lvl The logging level to display.
 * @param file The file name from where this function was called.
 * @param func The function name from where this function was called.
 * @param line The line number of this call.
 * @param fmt_string line The format string for the message being sent.
 * @param ... Any parameters for the format string.
 */
void _fetcher_log(FILE *stream, log_level_e lvl, const char *file, const char *func, int line, const char *fmt_string,
                  ...) {

    va_list args;
    va_start(args, fmt_string);

    // Get time for time stamp
    time_t cur_time = time(NULL);
    struct tm *tm = localtime(&cur_time);
    char time_str[TIME_STR_BUFSIZE];
    strftime(time_str, sizeof(time_str), TIME_FORMAT, tm);

    // Log to stream
    fprintf(stream, "[%s] %s fetcher %s:%d - %s() - \"", time_str, LEVEL_STRING[lvl], file, line, func);
    vfprintf(stream, fmt_string, args);
    fputc('"', stream);
    fputc('\n', stream);
}
