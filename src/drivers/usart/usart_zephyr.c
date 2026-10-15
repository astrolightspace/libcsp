#include "csp/drivers/usart_zephyr.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(libcsp, CONFIG_LIBCSP_LOG_LEVEL);

typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_usart_callback_t rx_callback;
	void * user_data;
	csp_usart_fd_t fd;
	struct uart_config original_config;
	bool started;
#if !defined(CONFIG_UART_INTERRUPT_DRIVEN)
	struct k_thread rx_thread;
	struct k_sem stop_rx;
	atomic_t stopping;
	int cbuf_len;
	uint8_t cbuf[CONFIG_CSP_UART_RX_BUFFER_LENGTH + 1];
#endif
} usart_context_t;

#if !defined(CONFIG_UART_INTERRUPT_DRIVEN)
static K_THREAD_STACK_ARRAY_DEFINE(uart_rx_stack,
								   CONFIG_CSP_UART_RX_THREAD_NUM, CONFIG_CSP_UART_RX_THREAD_STACK_SIZE);
#endif
static usart_context_t * uart_contexts[CONFIG_CSP_UART_RX_THREAD_NUM];

K_MUTEX_DEFINE(uart_driver_lock);

void csp_usart_lock(void * driver_data) {
	k_mutex_lock(&uart_driver_lock, K_MSEC(CONFIG_CSP_MUTEX_TIMEOUT));
}

void csp_usart_unlock(void * driver_data) {
	k_mutex_unlock(&uart_driver_lock);
}

int csp_usart_write(csp_usart_fd_t fd, const void * data, size_t data_length) {
	unsigned char * buf = (unsigned char *)data;

	for (size_t i = 0; i < data_length; i++) {
		uart_poll_out(fd, buf[i]);
	}

	return data_length;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void csp_usart_rx_isr(const struct device * dev, void * user_data) {
	usart_context_t * ctx = user_data;
	uint8_t buf[CONFIG_CSP_UART_RX_BUFFER_LENGTH];

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!uart_irq_rx_ready(dev)) {
			continue;
		}

		int len = uart_fifo_read(dev, buf, sizeof(buf));
		if (len > 0) {
			LOG_HEXDUMP_DBG(buf, len, ctx->name);
			ctx->rx_callback(ctx->user_data, buf, len, ctx);
		}
	}
}
#else
int csp_usart_zephyr_process_char(usart_context_t * ctx) {
	int ret;
	char recv_char;

	ret = uart_poll_in(ctx->fd, &recv_char);

	/* no more data */
	if (ret < 0) {
		/* if we have any data, hand it over */
		if (ctx->cbuf_len > 0) {
			LOG_HEXDUMP_DBG(ctx->cbuf, ctx->cbuf_len, ctx->name);
			ctx->rx_callback(ctx->user_data, ctx->cbuf, ctx->cbuf_len, NULL);
			ctx->cbuf_len = 0;
		}
		return ret;
	}

	/* got some data */

	/* FIXME: API shouldn't return > 0 but pl1011 does */
	if (ret > 0) LOG_WRN("[%s] uart_poll_in returned %d", ctx->name, ret);

	/* put data in the buf. We are sure we have room for it */
	ctx->cbuf[ctx->cbuf_len] = recv_char;
	ctx->cbuf_len++;

	/* if it's full, hand it over */
	if (ctx->cbuf_len >= CONFIG_CSP_UART_RX_BUFFER_LENGTH) {
		LOG_HEXDUMP_DBG(ctx->cbuf, ctx->cbuf_len, ctx->name);
		ctx->rx_callback(ctx->user_data, ctx->cbuf, ctx->cbuf_len, NULL);
		ctx->cbuf_len = 0;
	}

	return ret;
}

void csp_usart_rx_thread(void * arg1, void * arg2, void * arg3) {
	usart_context_t * ctx = arg1;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("[%s] start uart rx polling thread", ctx->name);

	while (!atomic_get(&ctx->stopping)) {
		int ret = csp_usart_zephyr_process_char(ctx);

		if (ret < 0) {
			k_sem_take(&ctx->stop_rx, K_NSEC(CONFIG_CSP_UART_RX_INTERVAL));
		}
	}
}
#endif

static usart_context_t * find_context(csp_usart_fd_t fd) {
	for (size_t i = 0; i < ARRAY_SIZE(uart_contexts); i++) {
		if (uart_contexts[i] != NULL && uart_contexts[i]->fd == fd) {
			return uart_contexts[i];
		}
	}
	return NULL;
}

int csp_usart_prepare(const struct device * fd, const struct uart_config * config,
					 csp_usart_callback_t rx_callback, void * user_data,
					 csp_usart_fd_t * return_fd) {
	if (config == NULL || config->baudrate == 0 || rx_callback == NULL || return_fd == NULL) {
		LOG_ERR("Invalid CSP UART configuration");
		return CSP_ERR_INVAL;
	}
	*return_fd = NULL;

	size_t slot = ARRAY_SIZE(uart_contexts);
	for (size_t i = 0; i < ARRAY_SIZE(uart_contexts); i++) {
		if (uart_contexts[i] != NULL && uart_contexts[i]->fd == fd) {
			LOG_ERR("UART device already owned by CSP");
			return CSP_ERR_USED;
		}
		if (uart_contexts[i] == NULL && slot == ARRAY_SIZE(uart_contexts)) {
			slot = i;
		}
	}
	if (slot == ARRAY_SIZE(uart_contexts)) {
		LOG_ERR("No CSP UART RX slots available (CONFIG_CSP_UART_RX_THREAD_NUM=%d)",
				CONFIG_CSP_UART_RX_THREAD_NUM);
		return CSP_ERR_DRIVER;
	}

	if (!device_is_ready(fd)) {
		LOG_ERR("CSP UART device not ready");
		return CSP_ERR_DRIVER;
	}

	usart_context_t * ctx = k_calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		LOG_ERR("[%s] Failed to allocate UART context", fd->name);
		return CSP_ERR_NOMEM;
	}
	int ret = uart_config_get(fd, &ctx->original_config);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to get UART configuration (%d)", fd->name, ret);
		k_free(ctx);
		return CSP_ERR_DRIVER;
	}
	ret = uart_configure(fd, config);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to configure UART at %u baud (%d)", fd->name, config->baudrate, ret);
		int restore = uart_configure(fd, &ctx->original_config);
		if (restore < 0) {
			LOG_ERR("[%s] Failed to restore UART configuration (%d)", fd->name, restore);
		}
		k_free(ctx);
		return CSP_ERR_DRIVER;
	}

	strncpy(ctx->name, fd->name, sizeof(ctx->name) - 1);
	ctx->rx_callback = rx_callback;
	ctx->user_data = user_data;
	ctx->fd = fd;
	uart_contexts[slot] = ctx;

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_rx_disable(fd);
	ret = uart_irq_callback_user_data_set(ctx->fd, csp_usart_rx_isr, ctx);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to set UART RX callback (%d)", fd->name, ret);
		int restore = uart_configure(fd, &ctx->original_config);
		if (restore < 0) {
			LOG_ERR("[%s] Failed to restore UART configuration (%d)", fd->name, restore);
		}
		uart_contexts[slot] = NULL;
		k_free(ctx);
		return CSP_ERR_DRIVER;
	}
#else
	k_sem_init(&ctx->stop_rx, 0, 1);
	k_tid_t rx_tid = k_thread_create(&ctx->rx_thread, uart_rx_stack[slot],
									 K_THREAD_STACK_SIZEOF(uart_rx_stack[slot]),
									 (k_thread_entry_t)csp_usart_rx_thread,
									 ctx, NULL, NULL,
									 CONFIG_CSP_UART_RX_THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(rx_tid, "csp_uart_rx");
#endif

	*return_fd = ctx->fd;
	LOG_INF("UART [%s] configured at %u baud (RX paused)", fd->name, config->baudrate);
	return CSP_ERR_NONE;
}

int csp_usart_start(csp_usart_fd_t fd) {
	usart_context_t * ctx = find_context(fd);
	if (ctx == NULL || ctx->started) {
		LOG_ERR("CSP UART is not prepared or already started");
		return CSP_ERR_INVAL;
	}
	ctx->started = true;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_rx_enable(fd);
#else
	k_thread_start(&ctx->rx_thread);
#endif
	return CSP_ERR_NONE;
}

int csp_usart_close(csp_usart_fd_t fd) {
	usart_context_t * ctx = find_context(fd);
	if (ctx == NULL) {
		LOG_ERR("CSP UART is not open");
		return CSP_ERR_INVAL;
	}
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_rx_disable(fd);
	int ret = uart_irq_callback_user_data_set(fd, NULL, NULL);
	if (ret < 0) {
		LOG_ERR("[%s] Failed to clear UART callback (%d)", fd->name, ret);
		return CSP_ERR_DRIVER;
	}
#else
	if (ctx->started) {
		atomic_set(&ctx->stopping, 1);
		k_sem_give(&ctx->stop_rx);
		int ret = k_thread_join(&ctx->rx_thread, K_MSEC(1500));
		if (ret < 0) {
			LOG_ERR("[%s] Failed to join UART RX thread (%d)", fd->name, ret);
			return CSP_ERR_DRIVER;
		}
	} else {
		k_thread_abort(&ctx->rx_thread);
	}
#endif
	int restore = uart_configure(fd, &ctx->original_config);
	if (restore < 0) {
		LOG_ERR("[%s] Failed to restore UART configuration (%d)", fd->name, restore);
		return CSP_ERR_DRIVER;
	}
	for (size_t i = 0; i < ARRAY_SIZE(uart_contexts); i++) {
		if (uart_contexts[i] == ctx) {
			uart_contexts[i] = NULL;
			break;
		}
	}
	k_free(ctx);
	return CSP_ERR_NONE;
}

int csp_usart_open(const csp_usart_conf_t * conf, csp_usart_callback_t callback,
				   void * user_data, csp_usart_fd_t * return_fd) {
	if (conf == NULL || conf->device == NULL || return_fd == NULL) {
		LOG_ERR("Invalid CSP UART open arguments");
		return CSP_ERR_INVAL;
	}
	*return_fd = NULL;
	const struct device * dev = device_get_binding(conf->device);
	struct uart_config config;
	if (!device_is_ready(dev) || uart_config_get(dev, &config) < 0) {
		LOG_ERR("[%s] Cannot read UART configuration", conf->device);
		return CSP_ERR_DRIVER;
	}
	config.baudrate = conf->baudrate;
	int ret = csp_usart_prepare(dev, &config, callback, user_data, return_fd);
	if (ret == CSP_ERR_NONE) {
		ret = csp_usart_start(*return_fd);
	}
	return ret;
}
