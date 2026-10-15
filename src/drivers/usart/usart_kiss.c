#include <csp/interfaces/csp_if_kiss.h>

#include <csp/csp_debug.h>
#include <stdlib.h>

#include <csp/csp.h>
#include <csp/drivers/usart.h>

#if (CSP_ZEPHYR)
#include <csp/drivers/usart_zephyr.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(libcsp, CONFIG_LIBCSP_LOG_LEVEL);
#endif


typedef struct {
	char name[CSP_IFLIST_NAME_MAX + 1];
	csp_iface_t iface;
	csp_kiss_interface_data_t ifdata;
	csp_usart_fd_t fd;
} kiss_context_t;

static int kiss_driver_tx(void * driver_data, const unsigned char * data, size_t data_length) {

	kiss_context_t * ctx = driver_data;
	if (csp_usart_write(ctx->fd, data, data_length) == (int)data_length) {
		return CSP_ERR_NONE;
	}
	return CSP_ERR_TX;
}

static void kiss_driver_rx(void * user_data, uint8_t * data, size_t data_size, void * pxTaskWoken) {

	kiss_context_t * ctx = user_data;
	csp_kiss_rx(&ctx->iface, data, data_size, pxTaskWoken);
}

#if (CSP_ZEPHYR)
int csp_kiss_prepare(const struct device * device, const struct uart_config * config,
					 const char * ifname, uint16_t addr, csp_iface_t ** return_iface) {
	if (ifname == NULL) {
		ifname = CSP_IF_KISS_DEFAULT_NAME;
	}
	if (return_iface == NULL || strlen(ifname) > CSP_IFLIST_NAME_MAX ||
		csp_iflist_get_by_name(ifname) != NULL) {
		LOG_ERR("Invalid or duplicate KISS interface name");
		return CSP_ERR_INVAL;
	}
	*return_iface = NULL;
	kiss_context_t * ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		LOG_ERR("Failed to allocate KISS context");
		return CSP_ERR_NOMEM;
	}
	strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.addr = addr;
	ctx->iface.driver_data = ctx;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->ifdata.tx_func = kiss_driver_tx;

	int ret = csp_usart_prepare(device, config, kiss_driver_rx, ctx, &ctx->fd);
	if (ret != CSP_ERR_NONE) {
		free(ctx);
		return ret;
	}
	ret = csp_kiss_add_interface(&ctx->iface);
	if (ret != CSP_ERR_NONE) {
		(void)csp_usart_close(ctx->fd);
		free(ctx);
		return ret;
	}
	*return_iface = &ctx->iface;
	return CSP_ERR_NONE;
}

int csp_kiss_start(csp_iface_t * iface) {
	if (iface == NULL || iface->driver_data == NULL) {
		LOG_ERR("Invalid KISS interface");
		return CSP_ERR_INVAL;
	}
	kiss_context_t * ctx = iface->driver_data;
	return csp_usart_start(ctx->fd);
}

int csp_kiss_close(csp_iface_t * iface) {
	if (iface == NULL || iface->driver_data == NULL) {
		LOG_ERR("Invalid KISS interface");
		return CSP_ERR_INVAL;
	}
	kiss_context_t * ctx = iface->driver_data;
	int ret = csp_usart_close(ctx->fd);
	/* A failed close may retain the callback; do not free its user_data. */
	if (ret != CSP_ERR_NONE) {
		return ret;
	}
	csp_iflist_remove(iface);
	if (ctx->ifdata.rx_packet != NULL) {
		csp_buffer_free(ctx->ifdata.rx_packet);
	}
	free(ctx);
	return CSP_ERR_NONE;
}

int csp_usart_open_and_add_kiss_interface(const csp_usart_conf_t * conf, const char * ifname,
										 uint16_t addr, csp_iface_t ** return_iface) {
	if (conf == NULL || conf->device == NULL) {
		LOG_ERR("Invalid KISS UART configuration");
		return CSP_ERR_INVAL;
	}
	if (return_iface != NULL) {
		*return_iface = NULL;
	}
	const struct device * device = device_get_binding(conf->device);
	struct uart_config config;
	if (!device_is_ready(device) || uart_config_get(device, &config) < 0) {
		LOG_ERR("[%s] Cannot read KISS UART configuration", conf->device);
		return CSP_ERR_DRIVER;
	}
	config.baudrate = conf->baudrate;
	csp_iface_t * iface = NULL;
	int ret = csp_kiss_prepare(device, &config, ifname, addr, &iface);
	if (ret != CSP_ERR_NONE) {
		return ret;
	}
	ret = csp_kiss_start(iface);
	if (ret != CSP_ERR_NONE) {
		(void)csp_kiss_close(iface);
		return ret;
	}
	if (return_iface != NULL) {
		*return_iface = iface;
	}
	return CSP_ERR_NONE;
}
#else
int csp_usart_open_and_add_kiss_interface(const csp_usart_conf_t * conf, const char * ifname, uint16_t addr, csp_iface_t ** return_iface) {

	if (ifname == NULL) {
		ifname = CSP_IF_KISS_DEFAULT_NAME;
	}

	kiss_context_t * ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		return CSP_ERR_NOMEM;
	}

	strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
	ctx->iface.name = ctx->name;
	ctx->iface.addr = addr;
	ctx->iface.driver_data = ctx;
	ctx->iface.interface_data = &ctx->ifdata;
	ctx->ifdata.tx_func = kiss_driver_tx;

#if (CSP_ZEPHYR)
	ctx->fd = NULL;
#else
	ctx->fd = -1;
#endif

	int res = csp_kiss_add_interface(&ctx->iface);
	if (res == CSP_ERR_NONE) {
		res = csp_usart_open(conf, kiss_driver_rx, ctx, &ctx->fd);
	}

#if (CSP_ZEPHYR)
	LOG_DBG("INIT %s: device %s, local address: %d, baudrate: %d",
		ctx->name, conf->device, addr, conf->baudrate);
#else
	csp_print("INIT %s: device %s, local address: %d, baudrate: %d\n",
		ctx->name, conf->device, addr, conf->baudrate);
#endif

	if (return_iface) {
		*return_iface = &ctx->iface;
	}

	return res;
}
#endif
