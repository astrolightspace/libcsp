/* Copyright (c) 2026 Astrolight
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/ztest.h>

#include <csp/drivers/usart.h>
#include <csp/drivers/usart_zephyr.h>
#include <csp/csp_iflist.h>

static struct uart_config current_config;
static int get_error;
static int configure_error;
static unsigned int configure_calls;
static unsigned int rx_enable_calls;

static int test_uart_config_get(const struct device *dev, struct uart_config *config) {
	ARG_UNUSED(dev);
	if (get_error != 0) {
		return get_error;
	}
	*config = current_config;
	return 0;
}

static int test_uart_configure(const struct device *dev, const struct uart_config *config) {
	ARG_UNUSED(dev);
	configure_calls++;
	if (configure_error != 0) {
		return configure_error;
	}
	current_config = *config;
	return 0;
}

static int test_uart_poll_in(const struct device *dev, unsigned char *c) {
	ARG_UNUSED(dev);
	ARG_UNUSED(c);
	return -1;
}

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
static void test_uart_callback_set(const struct device *dev, uart_irq_callback_user_data_t callback,
								   void *user_data) {
	ARG_UNUSED(dev);
	ARG_UNUSED(callback);
	ARG_UNUSED(user_data);
	if (callback != NULL) {
		zassert_equal(configure_calls, 1, "UART must be configured before RX callback");
	}
}

static void test_uart_rx_enable(const struct device *dev) {
	ARG_UNUSED(dev);
	rx_enable_calls++;
}

static void test_uart_rx_disable(const struct device *dev) {
	ARG_UNUSED(dev);
}
#endif

static DEVICE_API(uart, test_uart_api) = {
	.poll_in = test_uart_poll_in,
	.configure = test_uart_configure,
	.config_get = test_uart_config_get,
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	.irq_callback_set = test_uart_callback_set,
	.irq_rx_enable = test_uart_rx_enable,
	.irq_rx_disable = test_uart_rx_disable,
#endif
};

DEVICE_DEFINE(csp_test_uart, "kiss_uart", NULL, NULL, NULL, NULL, POST_KERNEL,
			  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &test_uart_api);

static void rx_callback(void *user_data, uint8_t *buffer, size_t length, void *isr_data) {
	ARG_UNUSED(user_data);
	ARG_UNUSED(buffer);
	ARG_UNUSED(length);
	ARG_UNUSED(isr_data);
}

static const csp_usart_conf_t usart_config = {
	.device = "kiss_uart",
	.baudrate = 115200,
	.databits = 8,
	.stopbits = 1,
	.paritysetting = 0,
};

static void before_test(void *fixture) {
	ARG_UNUSED(fixture);
	current_config = (struct uart_config){
		.baudrate = 921600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.stop_bits = UART_CFG_STOP_BITS_1,
		.parity = UART_CFG_PARITY_EVEN,
		.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS,
	};
	get_error = 0;
	configure_error = 0;
	configure_calls = 0;
	rx_enable_calls = 0;
}

ZTEST(usart, test_settings_baud_overrides_boot_baud) {
	csp_usart_fd_t fd = NULL;
	zassert_equal(csp_usart_open(&usart_config, rx_callback, NULL, &fd), CSP_ERR_NONE);
	zassert_equal(current_config.baudrate, 115200);
	zassert_equal(current_config.data_bits, UART_CFG_DATA_BITS_8);
	zassert_equal(current_config.stop_bits, UART_CFG_STOP_BITS_1);
	zassert_equal(current_config.parity, UART_CFG_PARITY_EVEN);
	zassert_equal(current_config.flow_ctrl, UART_CFG_FLOW_CTRL_RTS_CTS);
	zassert_equal(configure_calls, 1);
	zassert_equal(fd, DEVICE_GET(csp_test_uart));
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	zassert_equal(rx_enable_calls, 1);
#endif
	zassert_equal(csp_usart_close(fd), CSP_ERR_NONE);
	zassert_equal(current_config.baudrate, 921600);
}

ZTEST(usart, test_default_baud_is_applied) {
	current_config.baudrate = 115200;
	csp_usart_conf_t config = usart_config;
	config.baudrate = 921600;
	csp_usart_fd_t fd = NULL;

	zassert_equal(csp_usart_open(&config, rx_callback, NULL, &fd), CSP_ERR_NONE);
	zassert_equal(current_config.baudrate, 921600);
	zassert_equal(configure_calls, 1);
	zassert_equal(csp_usart_close(fd), CSP_ERR_NONE);
}

ZTEST(usart, test_prepare_defers_rx_and_applies_full_config) {
	struct uart_config config = {
		.baudrate = 115200,
		.data_bits = UART_CFG_DATA_BITS_7,
		.stop_bits = UART_CFG_STOP_BITS_2,
		.parity = UART_CFG_PARITY_ODD,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
	};
	csp_usart_fd_t fd = NULL;
	zassert_equal(csp_usart_prepare(DEVICE_GET(csp_test_uart), &config, rx_callback, NULL, &fd),
				  CSP_ERR_NONE);
	zassert_equal(rx_enable_calls, 0);
	zassert_equal(current_config.data_bits, UART_CFG_DATA_BITS_7);
	zassert_equal(current_config.stop_bits, UART_CFG_STOP_BITS_2);
	zassert_equal(current_config.parity, UART_CFG_PARITY_ODD);
	zassert_equal(current_config.flow_ctrl, UART_CFG_FLOW_CTRL_NONE);
	zassert_equal(csp_usart_close(fd), CSP_ERR_NONE);
	zassert_equal(current_config.baudrate, 921600);
	zassert_equal(current_config.parity, UART_CFG_PARITY_EVEN);
}

ZTEST(usart, test_kiss_close_unregisters_and_reuses_resources) {
	struct uart_config config = current_config;
	config.baudrate = 115200;
	for (size_t i = 0; i < CONFIG_CSP_UART_RX_THREAD_NUM + 1; i++) {
		configure_calls = 0;
		csp_iface_t *iface = NULL;
		zassert_equal(csp_kiss_prepare(DEVICE_GET(csp_test_uart), &config, "KISS", 7, &iface),
					  CSP_ERR_NONE);
		zassert_equal(csp_iflist_get_by_name("KISS"), iface);
		zassert_equal(csp_kiss_start(iface), CSP_ERR_NONE);
		zassert_equal(csp_kiss_close(iface), CSP_ERR_NONE);
		zassert_is_null(csp_iflist_get_by_name("KISS"));
	}
}

ZTEST(usart, test_get_config_failure_prevents_open) {
	get_error = -ENOSYS;
	csp_usart_fd_t fd = NULL;

	zassert_equal(csp_usart_open(&usart_config, rx_callback, NULL, &fd), CSP_ERR_DRIVER);
	zassert_is_null(fd);
	zassert_equal(configure_calls, 0);
	zassert_equal(rx_enable_calls, 0);
}

ZTEST(usart, test_unsupported_configuration_prevents_open) {
	configure_error = -ENOTSUP;
	csp_usart_fd_t fd = NULL;

	zassert_equal(csp_usart_open(&usart_config, rx_callback, NULL, &fd), CSP_ERR_DRIVER);
	zassert_is_null(fd);
	zassert_equal(current_config.baudrate, 921600);
	zassert_equal(rx_enable_calls, 0);
}

ZTEST(usart, test_invalid_configuration_prevents_open) {
	configure_error = -EINVAL;
	csp_usart_fd_t fd = NULL;

	zassert_equal(csp_usart_open(&usart_config, rx_callback, NULL, &fd), CSP_ERR_DRIVER);
	zassert_is_null(fd);
	zassert_equal(current_config.baudrate, 921600);
	zassert_equal(rx_enable_calls, 0);
}

ZTEST_SUITE(usart, NULL, NULL, before_test, NULL, NULL);
