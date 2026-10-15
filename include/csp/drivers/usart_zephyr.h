/* Copyright (c) 2026 Astrolight
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <csp/drivers/usart.h>
#include <zephyr/drivers/uart.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Configure a UART and reserve its RX resources without enabling reception.
 * One CSP owner per device. Configuration is restored when closed.
 */
int csp_usart_prepare(const struct device *device, const struct uart_config *config,
                     csp_usart_callback_t callback, void *user_data, csp_usart_fd_t *fd);
int csp_usart_start(csp_usart_fd_t fd);
int csp_usart_close(csp_usart_fd_t fd);

/** Prepare/register KISS without enabling RX. The caller sets address policy
 * and installs routes before calling csp_kiss_start().
 */
int csp_kiss_prepare(const struct device *device, const struct uart_config *config,
                     const char *name, uint16_t address, csp_iface_t **iface);
int csp_kiss_start(csp_iface_t *iface);
int csp_kiss_close(csp_iface_t *iface);

#ifdef __cplusplus
}
#endif
