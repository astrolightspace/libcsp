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

/*
 * C11 stdatomic.h compatibility for MSVC
 * MSVC's C11 atomics support requires specific flags that may not be available.
 * Provide a minimal implementation using Interlocked functions.
 */
#if defined(_MSC_VER) && !defined(__cplusplus)

typedef volatile long atomic_int;

#define ATOMIC_VAR_INIT(value) (value)

static __inline int atomic_load(const volatile atomic_int* obj) {
    return _InterlockedOr((volatile long*)obj, 0);
}

static __inline void atomic_store(volatile atomic_int* obj, int desired) {
    _InterlockedExchange((volatile long*)obj, desired);
}

static __inline int atomic_exchange(volatile atomic_int* obj, int desired) {
    return _InterlockedExchange((volatile long*)obj, desired);
}

static __inline int atomic_fetch_add(volatile atomic_int* obj, int arg) {
    return _InterlockedExchangeAdd((volatile long*)obj, arg);
}

static __inline int atomic_fetch_sub(volatile atomic_int* obj, int arg) {
    return _InterlockedExchangeAdd((volatile long*)obj, -arg);
}

/* Memory ordering - simplified, full barrier for all */
#define memory_order_relaxed 0
#define memory_order_consume 1
#define memory_order_acquire 2
#define memory_order_release 3
#define memory_order_acq_rel 4
#define memory_order_seq_cst 5

#define atomic_load_explicit(obj, order) atomic_load(obj)
#define atomic_store_explicit(obj, val, order) atomic_store(obj, val)

static __inline int atomic_compare_exchange_strong(volatile atomic_int* obj, int* expected, int desired) {
    int old = _InterlockedCompareExchange((volatile long*)obj, desired, *expected);
    if (old == *expected) {
        return 1;  /* Success */
    } else {
        *expected = old;
        return 0;  /* Failure */
    }
}

#endif /* _MSC_VER && !__cplusplus */

#endif /* _WIN32 */
