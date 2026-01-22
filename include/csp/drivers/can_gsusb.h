/****************************************************************************
 * **File:** csp/drivers/can_gsusb.h
 *
 * **Description:** GS_USB CAN driver for Candlelight/Canable USB-CAN adapters.
 *
 * This driver supports USB CAN adapters using the gs_usb protocol, including:
 * - Candlelight FD (VID=0x1d50, PID=0x606f)
 * - Canable (VID=0x1d50, PID=0x606f)
 * - Other gs_usb compatible devices
 *
 * .. note:: This driver requires the libusb-1.0 library.
 ****************************************************************************/
#pragma once

#include <csp/interfaces/csp_if_can.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * GS_USB configuration structure
 */
typedef struct {
    uint16_t vendor_id;     /**< USB Vendor ID (0 for auto-detect) */
    uint16_t product_id;    /**< USB Product ID (0 for auto-detect) */
    uint8_t channel;        /**< CAN channel on multi-channel devices (typically 0) */
    uint32_t bitrate;       /**< CAN bitrate in bits/second */
    bool fd_mode;           /**< Enable CAN FD mode if supported */
} csp_can_gsusb_conf_t;

/**
 * Default GS_USB configuration for Candlelight FD
 */
#define CSP_CAN_GSUSB_CONF_DEFAULT { \
    .vendor_id = 0,                  \
    .product_id = 0,                 \
    .channel = 0,                    \
    .bitrate = 1000000,              \
    .fd_mode = false                 \
}

/**
 * Open GS_USB CAN device and add CSP interface.
 *
 * Parameters:
 * @param[in] conf GS_USB configuration (use NULL for defaults)
 * @param[in] ifname CSP interface name, use #CSP_IF_CAN_DEFAULT_NAME for default name.
 * @param[in] node_id CSP address of the interface.
 * @param[out] return_iface the added interface.
 * @return CSP_ERR_NONE on success, or error code on failure.
 */
int csp_can_gsusb_open_and_add_interface(const csp_can_gsusb_conf_t *conf,
                                          const char *ifname,
                                          unsigned int node_id,
                                          csp_iface_t **return_iface);

/**
 * Stop the Rx thread and free resources.
 *
 * .. note:: This will invalidate CSP, because an interface can't be removed.
 *           This is primarily for testing.
 *
 * Parameters:
 * @param[in] iface interface to stop.
 * @return #CSP_ERR_NONE on success, otherwise an error code.
 */
int csp_can_gsusb_stop(csp_iface_t *iface);

/**
 * Check if GS_USB devices are available.
 *
 * @return Number of GS_USB compatible devices found, or negative on error.
 */
int csp_can_gsusb_enumerate(void);

#ifdef __cplusplus
}
#endif
