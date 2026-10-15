#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#ifdef _WIN32
#include <csp/csp_windows.h>
#endif

#include <csp/csp.h>
#include <csp/csp_debug.h>
#include <csp/csp_id.h>
#include <csp/csp_iflist.h>
#include <csp/interfaces/csp_if_lo.h>
#include "csp/autoconfig.h"

static int csp_rtable_parse(const char * rtable, int dry_run, uint8_t host_bits,
							const char *const *names, size_t name_count) {

	int valid_entries = 0;
	if (rtable == NULL) {
		return CSP_ERR_INVAL;
	}

	char rtable_copy[CSP_RTABLE_STRING_MAX_LEN + 1];
	const size_t str_len = strnlen(rtable, sizeof(rtable_copy));
	if (str_len > CSP_RTABLE_STRING_MAX_LEN) {
		return CSP_ERR_INVAL;
	}
	memcpy(rtable_copy, rtable, str_len);
	rtable_copy[str_len] = 0;

	/* Get first token */
	char * saveptr;
	char * str = strtok_r(rtable_copy, ",", &saveptr);
	while (str != NULL) {
		unsigned int address = 0, via = CSP_NO_VIA_ADDRESS;
		int netmask = host_bits;
		int consumed = 0;
		char name[15];
		if (sscanf(str, "%u/%d %14s %u %n", &address, &netmask, name, &via, &consumed) == 4) {
		} else if (sscanf(str, "%u/%d %14s %n", &address, &netmask, name, &consumed) == 3) {
			via = CSP_NO_VIA_ADDRESS;
		} else if (sscanf(str, "%u %14s %u %n", &address, name, &via, &consumed) == 3) {
			netmask = host_bits;
		} else if (sscanf(str, "%u %14s %n", &address, name, &consumed) == 2) {
			netmask = host_bits;
			via = CSP_NO_VIA_ADDRESS;
		} else {
			return CSP_ERR_INVAL;
		}
		if (str[consumed] != '\0') {
			return CSP_ERR_INVAL;
		}
		if (netmask == -1) {
			netmask = host_bits;
		}
		name[sizeof(name) - 1] = 0;

		csp_iface_t * ifc = NULL;
		bool known = false;
		if (names != NULL) {
			for (size_t i = 0; i < name_count; i++) {
				if (strcmp(name, names[i]) == 0) {
					known = true;
					break;
				}
			}
		} else {
			ifc = csp_iflist_get_by_name(name);
			known = ifc != NULL;
		}
		const unsigned int max_address = (1U << host_bits) - 1U;
		if (address > max_address || netmask < 0 || netmask > host_bits || !known ||
			(via != CSP_NO_VIA_ADDRESS && via > max_address)) {
			csp_dbg_errno = CSP_DBG_ERR_INVALID_RTABLE_ENTRY; 
			return CSP_ERR_INVAL;
		}

		if (dry_run == 0) {
			int res = csp_rtable_set(address, netmask, ifc, via);
			if (res != CSP_ERR_NONE) {
				csp_dbg_errno = CSP_DBG_ERR_INVALID_RTABLE_ENTRY;
				return res;
			}
		}
		valid_entries++;
		if (valid_entries > CSP_RTABLE_SIZE) {
			return CSP_ERR_NOMEM;
		}
		str = strtok_r(NULL, ",", &saveptr);
	}

	return valid_entries;
}

int csp_rtable_load(const char * rtable) {
	int ret = csp_rtable_check(rtable);
	return ret < 0 ? ret : csp_rtable_parse(rtable, 0, csp_id_get_host_bits(), NULL, 0);
}

int csp_rtable_check(const char * rtable) {
	return csp_rtable_parse(rtable, 1, csp_id_get_host_bits(), NULL, 0);
}

int csp_rtable_check_config(const char * rtable, uint8_t version,
							const char *const *names, size_t count) {
	if ((version != 1 && version != 2) || names == NULL) {
		return CSP_ERR_INVAL;
	}
	return csp_rtable_parse(rtable, 1, version == 1 ? 5 : 14, names, count);
}


typedef struct {
	char * buffer;
	size_t len;
	size_t maxlen;
	int error;
} csp_rtable_save_ctx_t;

static bool csp_rtable_save_route(void * vctx, csp_route_t * route) {
	csp_rtable_save_ctx_t * ctx = vctx;

	// Do not save loop back interface
	if (strcmp(route->iface->name, CSP_IF_LOOPBACK_NAME) == 0) {
		return true;
	}

	const char * sep = (ctx->len == 0) ? "" : ",";

	char mask_str[10];
	if (route->netmask != csp_id_get_host_bits()) {
		snprintf(mask_str, sizeof(mask_str), "/%u", route->netmask);
	} else {
		mask_str[0] = 0;
	}
	char via_str[10];
	if (route->via != CSP_NO_VIA_ADDRESS) {
		snprintf(via_str, sizeof(via_str), " %u", route->via);
	} else {
		via_str[0] = 0;
	}
	size_t remain_buf_size = ctx->maxlen - ctx->len;
	int res = snprintf(ctx->buffer + ctx->len, remain_buf_size,
					   "%s%u%s %s%s", sep, route->address, mask_str, route->iface->name, via_str);
	if ((res < 0) || (res >= (int)(remain_buf_size))) {
		ctx->error = CSP_ERR_NOMEM;
		return false;
	}
	ctx->len += res;
	return true;
}

int csp_rtable_save(char * buffer, size_t maxlen) {
	csp_rtable_save_ctx_t ctx = {.len = 0, .buffer = buffer, .maxlen = maxlen, .error = CSP_ERR_NONE};
	buffer[0] = 0;
	csp_rtable_iterate(csp_rtable_save_route, &ctx);
	return ctx.error;
}
