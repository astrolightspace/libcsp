#pragma once

#include "csp/autoconfig.h"

#if (CSP_ZEPHYR)
#include <zephyr/kernel.h>
#elif defined(_MSC_VER)
/* MSVC compatibility */
#define __noinit
#ifndef __packed
#define __packed
#endif
#define __maybe_unused
#define __unused
#define __weak

#define CONTAINER_OF(ptr, type, member) \
	((type *)(void *)((char *)(ptr) - offsetof(type, member)))
#else
/* GCC/Clang */
#define __noinit __attribute__((section(".noinit")))
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif
#define __maybe_unused __attribute__((__unused__))
#define __unused __attribute__((__unused__))
#define __weak   __attribute__((__weak__))

#define CONTAINER_OF(ptr, type, member) \
	((type *)(void *)((char *)(ptr) - offsetof(type, member)))

#endif
