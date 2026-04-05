#pragma once
#include <chrono>

#define LOG_INIT() static int __last_log_time = 0, __last_log_line = 0

#define LOG_MILLIS() std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()

#define LOG_THROTTLE(interval, ...) \
    if (__last_log_line != __LINE__ || __last_log_time + interval >= LOG_MILLIS()) { \
        __VA_ARGS__; \
        __last_log_time = LOG_MILLIS(); \
        __last_log_line = __LINE__; \
    }