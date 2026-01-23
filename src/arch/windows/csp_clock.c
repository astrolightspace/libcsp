#include <csp/csp_hooks.h>
#include "csp_macro.h"

#include <windows.h>

/* Convert Windows FILETIME to Unix timestamp */
static void filetime_to_csp_timestamp(const FILETIME * ft, csp_timestamp_t * time) {
    /* FILETIME is 100-nanosecond intervals since January 1, 1601 UTC */
    /* Unix epoch is January 1, 1970 UTC */
    /* Difference is 11644473600 seconds */
    ULARGE_INTEGER uli;
    uli.LowPart = ft->dwLowDateTime;
    uli.HighPart = ft->dwHighDateTime;

    /* Convert to seconds and nanoseconds since Unix epoch */
    uint64_t total_100ns = uli.QuadPart;
    uint64_t unix_100ns = total_100ns - 116444736000000000ULL;

    time->tv_sec = (uint32_t)(unix_100ns / 10000000ULL);
    time->tv_nsec = (uint32_t)((unix_100ns % 10000000ULL) * 100);
}

__weak void csp_clock_get_time(csp_timestamp_t * time) {
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    filetime_to_csp_timestamp(&ft, time);
}

__weak int csp_clock_set_time(const csp_timestamp_t * time) {
    /* Setting system time requires elevated privileges on Windows */
    /* For now, return error as this is rarely needed */
    (void)time;
    return CSP_ERR_INVAL;
}
