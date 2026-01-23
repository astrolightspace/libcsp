/**
 * Windows compatibility header for CSP
 * Provides POSIX equivalents for Windows builds
 */
#pragma once

#ifdef _WIN32

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Windows doesn't have endian.h - define byte swap macros */
#if defined(_MSC_VER)
#include <intrin.h>
#define htobe16(x) _byteswap_ushort(x)
#define htole16(x) (x)
#define be16toh(x) _byteswap_ushort(x)
#define le16toh(x) (x)
#define htobe32(x) _byteswap_ulong(x)
#define htole32(x) (x)
#define be32toh(x) _byteswap_ulong(x)
#define le32toh(x) (x)
#define htobe64(x) _byteswap_uint64(x)
#define htole64(x) (x)
#define be64toh(x) _byteswap_uint64(x)
#define le64toh(x) (x)
#else
/* MinGW or other Windows compilers */
#define htobe16(x) __builtin_bswap16(x)
#define htole16(x) (x)
#define be16toh(x) __builtin_bswap16(x)
#define le16toh(x) (x)
#define htobe32(x) __builtin_bswap32(x)
#define htole32(x) (x)
#define be32toh(x) __builtin_bswap32(x)
#define le32toh(x) (x)
#define htobe64(x) __builtin_bswap64(x)
#define htole64(x) (x)
#define be64toh(x) __builtin_bswap64(x)
#define le64toh(x) (x)
#endif

/* Windows uses strtok_s instead of strtok_r */
#define strtok_r(str, delim, saveptr) strtok_s(str, delim, saveptr)

/* unistd.h equivalents */
#include <io.h>
#include <process.h>

#ifndef STDIN_FILENO
#define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif

/* usleep equivalent */
#include <windows.h>
static inline void usleep(unsigned int usec) {
    Sleep(usec / 1000);
}

#endif /* _WIN32 */
