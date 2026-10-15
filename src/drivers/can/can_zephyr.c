#include <csp/interfaces/csp_if_can.h>

#include <csp/csp.h>
#include <csp/drivers/can_zephyr.h>

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(csp_can_driver, CONFIG_LIBCSP_LOG_LEVEL);

#define CSP_CAN_RX_MSGQ_TIME_OUT K_MSEC(1000)
#define CSP_CAN_TX_TIME_OUT      K_MSEC(100)
#define CSP_CAN_RX_THREAD_JOIN_TIME_OUT K_MSEC(1500)
#define CSP_CAN_STOP_EVENT (0x01)

/**
 * CAN interface data
*/
typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_can_interface_data_t ifdata;
	const struct device * device;
	struct k_msgq rx_msgq;
	uint8_t rx_msgq_buf[CONFIG_CSP_CAN_RX_MSGQ_DEPTH * sizeof(struct can_frame)];
	struct k_thread rx_thread;
	int filter_id;
	struct k_event stop_can_event;
	size_t slot;
	bool started;
} can_context_t;

static K_THREAD_STACK_ARRAY_DEFINE(rx_stack,
			CONFIG_CSP_CAN_RX_THREAD_NUM, CONFIG_CSP_CAN_RX_THREAD_STACK_SIZE);
static can_context_t * rx_contexts[CONFIG_CSP_CAN_RX_THREAD_NUM];

/*
 * Decode and log the CSP header carried in a CFP BEGIN frame. The routing header
 * lives in the first data bytes (v1) or in the CAN id + first data bytes (v2);
 * continuation frames carry no header and are skipped.
 */
static void csp_can_log_csp_header(uint32_t id, const uint8_t * data, uint8_t dlc) {

	uint16_t pri, src, dst, dport, sport;
	uint8_t flags;
	uint8_t hdr[6];
	uint8_t hdr_len;

	if (csp_conf.version == 1) {
		/* v1: 4-byte CSP header is data[0..3] (network order) of the BEGIN frame. */
		if ((CFP_TYPE(id) != 0u) || (dlc < 4u)) {
			return;
		}
		hdr[0] = data[0];
		hdr[1] = data[1];
		hdr[2] = data[2];
		hdr[3] = data[3];
		hdr_len = 4u;
		uint32_t h = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
			     ((uint32_t)hdr[2] << 8) | (uint32_t)hdr[3];
		pri   = (h >> 30) & 0x3u;
		src   = (h >> 25) & 0x1Fu;
		dst   = (h >> 20) & 0x1Fu;
		dport = (h >> 14) & 0x3Fu;
		sport = (h >> 8) & 0x3Fu;
		flags = h & 0xFFu;
	} else {
		/* v2: 6-byte header = [pri|dst from CAN id] + 4 data bytes (network order). */
		if (((id & (CFP2_BEGIN_MASK << CFP2_BEGIN_OFFSET)) == 0u) || (dlc < 4u)) {
			return;
		}
		uint16_t hi = (uint16_t)((id >> CFP2_DST_OFFSET) & 0xFFFFu);
		hdr[0] = (uint8_t)(hi >> 8);
		hdr[1] = (uint8_t)(hi & 0xFFu);
		hdr[2] = data[0];
		hdr[3] = data[1];
		hdr[4] = data[2];
		hdr[5] = data[3];
		hdr_len = 6u;
		uint64_t h = ((uint64_t)hdr[0] << 40) | ((uint64_t)hdr[1] << 32) |
			     ((uint64_t)hdr[2] << 24) | ((uint64_t)hdr[3] << 16) |
			     ((uint64_t)hdr[4] << 8) | (uint64_t)hdr[5];
		pri   = (h >> 46) & 0x3u;
		dst   = (h >> 32) & 0x3FFFu;
		src   = (h >> 18) & 0x3FFFu;
		dport = (h >> 12) & 0x3Fu;
		sport = (h >> 6) & 0x3Fu;
		flags = h & 0x3Fu;
	}

	/* Full reconstructed CSP header bytes (v2 includes the 2 bytes from the CAN id). */
	LOG_HEXDUMP_DBG(hdr, hdr_len, "CSP hdr");
	/* XTEA (0x04) is deprecated in libcsp and has no CSP_FXTEA define. */
	LOG_DBG("CSP v%d hdr: pri=%u src=%u dst=%u dport=%u sport=%u flags=0x%02x [%s%s%s%s]",
		csp_conf.version, pri, src, dst, dport, sport, flags,
		(flags & CSP_FHMAC) ? "HMAC " : "",
		(flags & 0x04u) ? "XTEA " : "",
		(flags & CSP_FRDP) ? "RDP " : "",
		(flags & CSP_FCRC32) ? "CRC" : "");
}

static void csp_can_rx_thread(void * arg1, void * arg2, void * arg3) {

	int ret;
	struct can_frame frame;
	struct k_msgq * rx_msgq = arg1;
	csp_iface_t * iface = arg2;
	struct k_event * stop_can_event = arg3;

	while (true) {
		/*
		 * Check if there is a need to stop the RX thread based
		 * on a user's request.
		*/
		if (k_event_wait(stop_can_event, CSP_CAN_STOP_EVENT, false, K_NO_WAIT) != 0) {
			break;
		}

		/* Read CAN frame */
		ret = k_msgq_get(rx_msgq, &frame, CSP_CAN_RX_MSGQ_TIME_OUT);
		if (ret == -EAGAIN) {
			continue;
		} else if (ret < 0) {
			LOG_ERR("[%s] k_msgq_get() failed, errno %d", iface->name, ret);
			break;
		}

		LOG_DBG("[%s] RX id=0x%08x %s%sdlc=%u",
			iface->name, frame.id,
			(frame.flags & CAN_FRAME_IDE) ? "EXT " : "STD ",
			(frame.flags & CAN_FRAME_RTR) ? "RTR " : "",
			frame.dlc);
		LOG_HEXDUMP_DBG(frame.data, MIN(frame.dlc, CAN_MAX_DLC), "CAN RX");

		csp_can_log_csp_header(frame.id, frame.data, frame.dlc);

		/* CSP requires extended frame format, drop it. */
		if (!(frame.flags & CAN_FRAME_IDE)) {
			LOG_WRN("[%s] discarding Standard ID frame", iface->name);
			continue;
		}

		/* CSP uses data frame only, drop it. */
		if (frame.flags & CAN_FRAME_RTR) {
			LOG_WRN("[%s] discarding RTR frame", iface->name);
			continue;
		}

		/* Call the common CSP CAN RX function. */
		csp_can_rx(iface, frame.id, frame.data, frame.dlc, NULL);
	}
}

static int csp_can_tx_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc) {

	int ret = CSP_ERR_NONE;
	struct can_frame frame = {0};
	can_context_t * ctx = driver_data;

	if (dlc > CAN_MAX_DLC) {
		ret = CSP_ERR_INVAL;
		goto end;
	}

	frame.id = id;
	frame.dlc = dlc;
	frame.flags = CAN_FRAME_IDE;
	memcpy(frame.data, data, dlc);

	LOG_DBG("[%s] TX id=0x%08x dlc=%u", ctx->name, id, dlc);
	LOG_HEXDUMP_DBG(data, MIN(dlc, CAN_MAX_DLC), "CAN TX");
	csp_can_log_csp_header(id, data, dlc);

	ret = can_send(ctx->device, &frame, CSP_CAN_TX_TIME_OUT, NULL, NULL);
	if (ret < 0) {
		LOG_ERR("[%s] can_send() failed, errno %d", ctx->name, ret);
	}

end:
	return ret;
}

static void csp_can_remove_rx_filter(can_context_t * ctx) {

	can_remove_rx_filter(ctx->device, ctx->filter_id);
	ctx->filter_id = -1;
}

static int csp_can_finish_rx_thread(can_context_t * ctx) {

	int ret;

	k_event_set(&ctx->stop_can_event, CSP_CAN_STOP_EVENT);

	ret = k_thread_join(&ctx->rx_thread, CSP_CAN_RX_THREAD_JOIN_TIME_OUT);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to finish the RX thread. error: %d",
			 ctx->name, ret);
		return ret;
	}

	k_event_clear(&ctx->stop_can_event, CSP_CAN_STOP_EVENT);

	k_msgq_purge(&ctx->rx_msgq);
	k_msgq_cleanup(&ctx->rx_msgq);

	return ret;
}

int csp_can_prepare_interface(const struct device * device, const char * ifname,
				    uint16_t address, uint32_t bitrate,
				    uint16_t filter_addr, uint16_t filter_mask,
				    csp_iface_t ** return_iface) {

	int ret;
	k_tid_t rx_tid;
	can_context_t * ctx = NULL;
	const char * name;

	if (!device_is_ready(device) || bitrate == 0 || return_iface == NULL) {
		LOG_ERR("Invalid CAN device or configuration");
		ret = CSP_ERR_INVAL;
		goto end;
	}

	name = ifname ? ifname : device->name;
	*return_iface = NULL;

	size_t slot = ARRAY_SIZE(rx_contexts);
	for (size_t i = 0; i < ARRAY_SIZE(rx_contexts); i++) {
		if (rx_contexts[i] != NULL && rx_contexts[i]->device == device) {
			LOG_ERR("[%s] CAN device already owned by CSP", name);
			return CSP_ERR_USED;
		}
		if (rx_contexts[i] == NULL && slot == ARRAY_SIZE(rx_contexts)) {
			slot = i;
		}
	}
	if (strlen(name) > CSP_IFLIST_NAME_MAX || csp_iflist_get_by_name(name) != NULL) {
		LOG_ERR("[%s] Invalid or duplicate CAN interface name", name);
		return CSP_ERR_INVAL;
	}
	if (slot == ARRAY_SIZE(rx_contexts)) {
		LOG_ERR("[%s] No more RX thread can be created. (MAX: %d) Please check CONFIG_CSP_CAN_RX_THREAD_NUM.",
			 name, CONFIG_CSP_CAN_RX_THREAD_NUM);
		ret = CSP_ERR_DRIVER;
		goto end;
	}

	/*
	 * TODO:
	 * In the current implementation, we use k_alloc() to allocate the memory
	 * for CAN context like socketcan implementation.
	 * However, we plan to remove dynamic memory allocations as described in
	 * the issue below.
	 * - https://github.com/libcsp/libcsp/issues/460
	 * And in Zephyr, the default heap memory size is 0, so we need to set a
	 * value using CONFIG_HEAP_MEM_POOL_SIZE.
	 */
	ctx = k_calloc(1, sizeof(can_context_t));
	if (ctx == NULL) {
		LOG_ERR("[%s] Failed to allocate %zu bytes from the system heap", 
			  name, sizeof(can_context_t));
		ret = CSP_ERR_NOMEM;
		goto end;
	}

	/* Set the each parameter to CAN context. */
	strncpy(ctx->name, name, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.addr = address;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->iface.driver_data = ctx;
	ctx->ifdata.tx_func = csp_can_tx_frame;
	ctx->ifdata.pbufs = NULL;
	ctx->device = device;
	ctx->filter_id = -1;
	ctx->slot = slot;
	k_event_init(&ctx->stop_can_event);

	LOG_DBG("INIT %s: device: [%s], local address: %d, bitrate: %d: filter add: %d, filter mask: 0x%04x",
			  ctx->name, device->name, address, bitrate, filter_addr, filter_mask);

	/* Initialize the RX message queue */
	k_msgq_init(&ctx->rx_msgq, ctx->rx_msgq_buf,
						sizeof(struct can_frame), CONFIG_CSP_CAN_RX_MSGQ_DEPTH);

	/* Set Bit rate */
	ret = can_set_bitrate(device, bitrate);
	if (ret < 0) {
		LOG_ERR("[%s] can_set_bitrate() failed, error: %d", ctx->name, ret);
		goto cleanup_heap;
	}

	/* Set RX filter */
	ret = csp_can_set_rx_filter(&ctx->iface, filter_addr, filter_mask);
	if (ret < 0) {
		LOG_ERR("[%s] csp_can_add_rx_filter() failed, error: %d", ctx->name, ret);
		goto cleanup_heap;
	}

	/* Add interface to CSP */
	ret = csp_can_add_interface(&ctx->iface);
	if (ret != CSP_ERR_NONE) {
		LOG_ERR("[%s] csp_can_add_interface() failed, error: %d", ctx->name, ret);
		goto cleanup_filter;
	}

	/* Create receive thread */
	rx_tid = k_thread_create(&ctx->rx_thread, rx_stack[slot],
					 K_THREAD_STACK_SIZEOF(rx_stack[slot]),
					 (k_thread_entry_t)csp_can_rx_thread,
					 &ctx->rx_msgq, &ctx->iface, &ctx->stop_can_event,
					 CONFIG_CSP_CAN_RX_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(rx_tid, "csp_can_rx");
	rx_contexts[slot] = ctx;

	if (return_iface) {
		*return_iface = &ctx->iface;
	}

	return CSP_ERR_NONE;

	/*
	 * The following section is for restoring acquired resources when
	 * something fails. Unfortunately, we can't take any action if the
	 * restoration process fails, so we proceed with the remaining
	 * cleanup. In addtion to this, we've chosen not to restore the
	 * CAN bit rate. If this causes any issues, please open an issue
	 * on GitHub.
	 */
cleanup_filter:
	csp_can_remove_rx_filter(ctx);

cleanup_heap:
	k_free(ctx);

end:
	return ret;
}

int csp_can_start(csp_iface_t * iface) {
	if (iface == NULL || iface->driver_data == NULL) {
		LOG_ERR("Invalid CSP CAN interface");
		return CSP_ERR_INVAL;
	}
	can_context_t * ctx = iface->driver_data;
	if (ctx->started) {
		LOG_ERR("[%s] CAN already started", iface->name);
		return CSP_ERR_ALREADY;
	}
	int ret = can_start(ctx->device);
	if (ret < 0) {
		LOG_ERR("[%s] can_start() failed (%d)", iface->name, ret);
		return ret;
	}
	ctx->started = true;
	k_thread_start(&ctx->rx_thread);
	return CSP_ERR_NONE;
}

int csp_can_open_and_add_interface(const struct device * device, const char * ifname,
								   uint16_t address, uint32_t bitrate,
								   uint16_t filter_addr, uint16_t filter_mask,
								   csp_iface_t ** return_iface) {
	if (return_iface != NULL) {
		*return_iface = NULL;
	}
	csp_iface_t * iface = NULL;
	int ret = csp_can_prepare_interface(device, ifname, address, bitrate,
									   filter_addr, filter_mask, &iface);
	if (ret != CSP_ERR_NONE) {
		return ret;
	}
	ret = csp_can_start(iface);
	if (ret != CSP_ERR_NONE) {
		(void)csp_can_stop(iface);
		return ret;
	}
	if (return_iface != NULL) {
		*return_iface = iface;
	}
	return CSP_ERR_NONE;
}

int csp_can_set_rx_filter(csp_iface_t * iface, uint16_t filter_addr, uint16_t filter_mask) {

	int ret;
	struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
	};
	can_context_t * ctx;

	if ((iface == NULL) || (iface->driver_data == NULL)) {
		ret = CSP_ERR_INVAL;
		goto end;
	}

	ctx = iface->driver_data;

	/* If a filter is already set, delete it. */
	if (ctx->filter_id >= 0) {
		can_remove_rx_filter(ctx->device, ctx->filter_id);
	}

	if (csp_conf.version == 1) {
		filter.id = CFP_MAKE_DST(filter_addr);
		filter.mask = CFP_MAKE_DST(filter_mask);
	} else {
		filter.id = filter_addr << CFP2_DST_OFFSET;
		filter.mask = filter_mask << CFP2_DST_OFFSET;
	}

	ret = can_add_rx_filter_msgq(ctx->device, &ctx->rx_msgq, &filter);
	if (ret < 0) {
		LOG_ERR("[%s] can_add_rx_filter_msgq() failed, error: %d", iface->name, ctx->filter_id);
		goto end;
	}
	ctx->filter_id = ret;

end:
	return ret;
}

int csp_can_stop(csp_iface_t * iface) {

	int ret;
	can_context_t * ctx;

	if ((iface == NULL) || (iface->driver_data == NULL)) {
		ret = CSP_ERR_INVAL;
		goto end;
	}

	ctx = iface->driver_data;

	ret = CSP_ERR_NONE;
	if (ctx->started) {
		ret = can_stop(ctx->device);
		if (ret < 0) {
			LOG_ERR("[%s] can_stop() failed (%d)", iface->name, ret);
			return ret;
		}
		int joined = csp_can_finish_rx_thread(ctx);
		if (joined < 0) {
			return joined;
		}
	} else {
		k_thread_abort(&ctx->rx_thread);
		k_msgq_purge(&ctx->rx_msgq);
		k_msgq_cleanup(&ctx->rx_msgq);
	}

	(void)csp_can_remove_interface(&ctx->iface);

	csp_can_remove_rx_filter(ctx);

	LOG_DBG("Stop CAN interface: %s. ret: %d", iface->name, ret);

	rx_contexts[ctx->slot] = NULL;
	while (ctx->ifdata.pbufs != NULL) {
		csp_packet_t *packet = ctx->ifdata.pbufs;
		ctx->ifdata.pbufs = packet->next;
		csp_buffer_free(packet);
	}
	k_free(ctx);

end:
	return ret;
}
