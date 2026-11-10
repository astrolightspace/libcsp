/**
 * GS_USB CAN driver for Candlelight/Canable USB-CAN adapters.
 *
 * This driver implements the gs_usb protocol used by many USB-CAN adapters
 * based on the STM32 microcontroller with candleLight firmware.
 *
 * Protocol reference: Linux kernel drivers/net/can/usb/gs_usb.c
 */

#include <csp/drivers/can_gsusb.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <csp/csp.h>
#include <csp/csp_debug.h>

#ifdef _WIN32
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#ifdef _WIN32
#include <windows.h>
#define usleep(x) Sleep((x) / 1000)
#define pthread_t HANDLE
#define pthread_create(t, a, f, d) ((*t = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)f, d, 0, NULL)) == NULL)
#else
#include <pthread.h>
#include <unistd.h>
#endif

/* GS_USB protocol constants */
#define GS_USB_BREQ_HOST_FORMAT      0
#define GS_USB_BREQ_BITTIMING        1
#define GS_USB_BREQ_MODE             2
#define GS_USB_BREQ_BERR             3
#define GS_USB_BREQ_BT_CONST         4
#define GS_USB_BREQ_DEVICE_CONFIG    5
#define GS_USB_BREQ_TIMESTAMP        6
#define GS_USB_BREQ_IDENTIFY         7
#define GS_USB_BREQ_GET_USER_ID      8
#define GS_USB_BREQ_SET_USER_ID      9
#define GS_USB_BREQ_DATA_BITTIMING   10
#define GS_USB_BREQ_BT_CONST_EXT     11

/* GS_USB mode values */
#define GS_CAN_MODE_RESET            0
#define GS_CAN_MODE_START            1

/* GS_USB mode flags (used when starting the device) */
#define GS_CAN_MODE_LISTEN_ONLY      (1 << 0)
#define GS_CAN_MODE_LOOP_BACK        (1 << 1)
#define GS_CAN_MODE_TRIPLE_SAMPLE    (1 << 2)
#define GS_CAN_MODE_ONE_SHOT         (1 << 3)
#define GS_CAN_MODE_HW_TIMESTAMP     (1 << 4)
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE (1 << 7)
#define GS_CAN_MODE_FD               (1 << 8)

/* GS_USB feature flags */
#define GS_CAN_FEATURE_LISTEN_ONLY   (1 << 0)
#define GS_CAN_FEATURE_LOOP_BACK     (1 << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE (1 << 2)
#define GS_CAN_FEATURE_ONE_SHOT      (1 << 3)
#define GS_CAN_FEATURE_HW_TIMESTAMP  (1 << 4)
#define GS_CAN_FEATURE_IDENTIFY      (1 << 5)
#define GS_CAN_FEATURE_USER_ID       (1 << 6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1 << 7)
#define GS_CAN_FEATURE_FD            (1 << 8)
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX (1 << 9)
#define GS_CAN_FEATURE_BT_CONST_EXT  (1 << 10)
#define GS_CAN_FEATURE_TERMINATION   (1 << 11)

/* CAN frame flags */
#define GS_CAN_FLAG_OVERFLOW         (1 << 0)
#define GS_CAN_FLAG_FD               (1 << 1)
#define GS_CAN_FLAG_BRS              (1 << 2)
#define GS_CAN_FLAG_ESI              (1 << 3)

/* Known GS_USB device identifiers */
#define GS_USB_VID_CANDLELIGHT       0x1d50
#define GS_USB_PID_CANDLELIGHT       0x606f

/* USB endpoints */
#define GS_USB_ENDPOINT_IN           0x81
#define GS_USB_ENDPOINT_OUT          0x02

/* Maximum CAN data length */
#define CAN_MAX_DLEN                 8
#define CANFD_MAX_DLEN               64

/* USB transfer timeout in milliseconds */
#define GS_USB_TIMEOUT               1000

/* Number of IN transfers kept in flight. The Linux kernel gs_usb driver
 * also keeps 10 URBs in flight. */
#define GS_USB_RX_TRANSFERS          10

/* Event loop poll interval in milliseconds. A poll timeout does not cancel
 * the IN transfers, so no data is lost when it expires. */
#define GS_USB_EVENT_POLL_MS         100

/* Time in milliseconds to wait for the cancelled IN transfers at stop */
#define GS_USB_STOP_TIMEOUT_MS       2000

/* Consecutive RX transfer errors before a transfer is not resubmitted */
#define GS_USB_RX_MAX_ERRORS         100

#pragma pack(push, 1)

/* GS_USB host frame structure */
typedef struct {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t data[CANFD_MAX_DLEN];
} gs_host_frame_t;

/* GS_USB device configuration */
typedef struct {
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t icount;
    uint32_t sw_version;
    uint32_t hw_version;
} gs_device_config_t;

/* GS_USB bit timing constants */
typedef struct {
    uint32_t feature;
    uint32_t fclk_can;
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} gs_device_bt_const_t;

/* GS_USB bit timing structure */
typedef struct {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} gs_device_bittiming_t;

/* GS_USB mode structure */
typedef struct {
    uint32_t mode;
    uint32_t flags;
} gs_device_mode_t;

/* GS_USB host config (endianness) */
typedef struct {
    uint32_t byte_order;
} gs_host_config_t;

#pragma pack(pop)

/* Size of a classic CAN host frame (header and 8 data bytes), without timestamp.
 * The device expects frames of exactly this size when FD mode is off. */
#define GS_HOST_FRAME_CLASSIC_SIZE   (offsetof(gs_host_frame_t, data) + CAN_MAX_DLEN)

/* CAN interface context */
typedef struct {
    char name[CSP_IFLIST_NAME_MAX + 1];
    csp_iface_t iface;
    csp_can_interface_data_t ifdata;
    pthread_t rx_thread;
    int rx_thread_started;
    libusb_device_handle *dev_handle;
    libusb_context *usb_ctx;
    uint8_t channel;
    volatile int running;
    gs_device_bt_const_t bt_const;
    /* Asynchronous IN transfers. The callbacks run in the thread that handles
     * libusb events. libusb lets only one thread handle events at a time, so
     * the callbacks do not run concurrently. */
    struct libusb_transfer *rx_transfers[GS_USB_RX_TRANSFERS];
    gs_host_frame_t rx_buffers[GS_USB_RX_TRANSFERS];
    volatile int rx_active;     /* Number of IN transfers in flight */
    int rx_errors;              /* Consecutive IN transfer errors */
} gsusb_context_t;

/* Forward declarations */
static int csp_can_gsusb_tx_frame(void *driver_data, uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * Free GS_USB context resources
 */
static void gsusb_free(gsusb_context_t *ctx) {
    if (ctx) {
        if (ctx->rx_active > 0) {
            /* Transfers are still in flight. Freeing them or the context is
             * not safe, so leak the resources. */
            csp_print("GS_USB[%s]: %d RX transfers still active, resources not freed\n",
                      ctx->name, ctx->rx_active);
            return;
        }
        for (int i = 0; i < GS_USB_RX_TRANSFERS; i++) {
            if (ctx->rx_transfers[i]) {
                libusb_free_transfer(ctx->rx_transfers[i]);
            }
        }
        if (ctx->dev_handle) {
            libusb_release_interface(ctx->dev_handle, 0);
            libusb_close(ctx->dev_handle);
        }
        if (ctx->usb_ctx) {
            libusb_exit(ctx->usb_ctx);
        }
        free(ctx);
    }
}

/**
 * Calculate bit timing parameters for a given bitrate
 * Target sample point: 75-87.5% (CAN 2.0 recommendation)
 */
static int gsusb_calc_bittiming(gsusb_context_t *ctx, uint32_t bitrate, gs_device_bittiming_t *bt) {
    uint32_t fclk = ctx->bt_const.fclk_can;
    uint32_t best_error = UINT32_MAX;
    uint32_t best_brp = 0, best_tseg1 = 0, best_tseg2 = 0;
    int best_sp_error = 1000;  /* Sample point error in 0.1% */

    /* Target sample point: 87.5% (875 in 0.1% units) */
    const int target_sp = 875;

    /* Try different BRP values */
    for (uint32_t brp = ctx->bt_const.brp_min; brp <= ctx->bt_const.brp_max; brp += ctx->bt_const.brp_inc) {
        uint32_t tq_per_bit = fclk / (brp * bitrate);

        if (tq_per_bit < 8 || tq_per_bit > 25) continue;

        /* Try different TSEG1/TSEG2 combinations */
        for (uint32_t tseg1 = ctx->bt_const.tseg1_min; tseg1 <= ctx->bt_const.tseg1_max; tseg1++) {
            uint32_t tseg2 = tq_per_bit - 1 - tseg1;
            if (tseg2 < ctx->bt_const.tseg2_min || tseg2 > ctx->bt_const.tseg2_max) continue;

            /* Calculate actual bitrate */
            uint32_t actual_bitrate = fclk / (brp * (1 + tseg1 + tseg2));
            uint32_t bitrate_error = (actual_bitrate > bitrate) ?
                            (actual_bitrate - bitrate) : (bitrate - actual_bitrate);

            /* Calculate sample point (in 0.1% units) */
            int sample_point = ((1 + tseg1) * 1000) / (1 + tseg1 + tseg2);
            int sp_error = (sample_point > target_sp) ?
                          (sample_point - target_sp) : (target_sp - sample_point);

            /* Prefer configurations with:
             * 1. Exact bitrate match (or close)
             * 2. Sample point close to 87.5%
             */
            if (bitrate_error < best_error ||
                (bitrate_error == best_error && sp_error < best_sp_error)) {
                best_error = bitrate_error;
                best_sp_error = sp_error;
                best_brp = brp;
                best_tseg1 = tseg1;
                best_tseg2 = tseg2;
            }
        }
    }

    if (best_error == UINT32_MAX) {
        csp_print("GS_USB: Cannot calculate bit timing for %u bps\n", bitrate);
        return CSP_ERR_INVAL;
    }

    bt->prop_seg = 0;
    bt->phase_seg1 = best_tseg1;
    bt->phase_seg2 = best_tseg2;
    bt->sjw = (best_tseg2 < ctx->bt_const.sjw_max) ? best_tseg2 : ctx->bt_const.sjw_max;
    bt->brp = best_brp;

    int sample_point = ((1 + best_tseg1) * 1000) / (1 + best_tseg1 + best_tseg2);
    csp_print("GS_USB: Bit timing: brp=%u, tseg1=%u, tseg2=%u, sjw=%u, sample_point=%d.%d%% (error=%u)\n",
              best_brp, best_tseg1, best_tseg2, bt->sjw, sample_point / 10, sample_point % 10, best_error);

    return CSP_ERR_NONE;
}

/**
 * Send control request to GS_USB device
 */
static int gsusb_control_out(gsusb_context_t *ctx, uint8_t request, uint16_t channel,
                             void *data, uint16_t size) {
    int ret = libusb_control_transfer(ctx->dev_handle,
                                      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                      request, channel, 0,
                                      (unsigned char *)data, size,
                                      GS_USB_TIMEOUT);
    return (ret == size) ? CSP_ERR_NONE : CSP_ERR_DRIVER;
}

/**
 * Receive control request from GS_USB device
 */
static int gsusb_control_in(gsusb_context_t *ctx, uint8_t request, uint16_t channel,
                            void *data, uint16_t size) {
    int ret = libusb_control_transfer(ctx->dev_handle,
                                      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                      request, channel, 0,
                                      (unsigned char *)data, size,
                                      GS_USB_TIMEOUT);
    return (ret == size) ? CSP_ERR_NONE : CSP_ERR_DRIVER;
}

/**
 * Decode one host frame from the device and forward it to CSP
 */
static void gsusb_rx_frame(gsusb_context_t *ctx, const gs_host_frame_t *frame, int length) {
    if (length < (int)offsetof(gs_host_frame_t, data)) {  /* Minimum frame size */
        return;
    }

    /* Check channel */
    if (frame->channel != ctx->channel) {
        return;
    }

    /* Skip echo frames (TX confirmation) */
    if (frame->echo_id != 0xFFFFFFFF) {
        return;
    }

    /* Extract CAN ID flags */
    uint32_t can_id = frame->can_id;
    bool is_extended = (can_id & 0x80000000) != 0;  /* IDE flag */
    bool is_rtr = (can_id & 0x40000000) != 0;       /* RTR flag */
    bool is_error = (can_id & 0x20000000) != 0;     /* ERR flag */

    /* Skip error and RTR frames */
    if (is_error || is_rtr) {
        return;
    }

    /* CSP uses extended frames */
    if (!is_extended) {
        return;
    }

    if (frame->flags & GS_CAN_FLAG_OVERFLOW) {
        csp_print("GS_USB[%s]: Device RX overflow, frames lost\n", ctx->name);
    }

    /* Mask out flags */
    can_id &= 0x1FFFFFFF;

    /* Validate DLC */
    uint8_t dlc = frame->can_dlc;
    if (dlc > CAN_MAX_DLEN) {
        dlc = CAN_MAX_DLEN;
    }

    /* Forward to CSP */
    csp_can_rx(&ctx->iface, can_id, frame->data, dlc, NULL);
}

/**
 * Completion callback for the IN transfers.
 *
 * The transfer is resubmitted at once, so the device always has IN transfers
 * to complete. The callback runs in the thread that handles libusb events:
 * the RX thread, or a TX call that waits for its bulk transfer.
 */
static void LIBUSB_CALL gsusb_rx_callback(struct libusb_transfer *transfer) {
    gsusb_context_t *ctx = (gsusb_context_t *)transfer->user_data;

    switch (transfer->status) {
    case LIBUSB_TRANSFER_COMPLETED:
        ctx->rx_errors = 0;
        gsusb_rx_frame(ctx, (const gs_host_frame_t *)transfer->buffer, transfer->actual_length);
        break;

    case LIBUSB_TRANSFER_CANCELLED:
        ctx->rx_active--;
        return;

    case LIBUSB_TRANSFER_NO_DEVICE:
        if (ctx->running) {
            csp_print("GS_USB[%s]: Device disconnected\n", ctx->name);
        }
        ctx->rx_active--;
        return;

    default:
        if (ctx->running) {
            csp_print("GS_USB[%s]: USB read error: %s\n", ctx->name,
                      libusb_error_name(transfer->status));
        }
        if (++ctx->rx_errors >= GS_USB_RX_MAX_ERRORS) {
            csp_print("GS_USB[%s]: Too many USB read errors, RX transfer stopped\n", ctx->name);
            ctx->rx_active--;
            return;
        }
        break;
    }

    if (ctx->running) {
        int ret = libusb_submit_transfer(transfer);
        if (ret == LIBUSB_SUCCESS) {
            return;
        }
        csp_print("GS_USB[%s]: Could not resubmit RX transfer: %s\n", ctx->name, libusb_error_name(ret));
    }
    ctx->rx_active--;
}

/**
 * Allocate and submit the IN transfers.
 *
 * @return Number of submitted transfers.
 */
static int gsusb_rx_submit_all(gsusb_context_t *ctx) {
    for (int i = 0; i < GS_USB_RX_TRANSFERS; i++) {
        struct libusb_transfer *transfer = libusb_alloc_transfer(0);
        if (transfer == NULL) {
            break;
        }
        ctx->rx_transfers[i] = transfer;
        libusb_fill_bulk_transfer(transfer, ctx->dev_handle, GS_USB_ENDPOINT_IN,
                                  (unsigned char *)&ctx->rx_buffers[i], sizeof(ctx->rx_buffers[i]),
                                  gsusb_rx_callback, ctx, 0);  /* No timeout */
        int ret = libusb_submit_transfer(transfer);
        if (ret != LIBUSB_SUCCESS) {
            csp_print("GS_USB[%s]: Could not submit RX transfer: %s\n", ctx->name, libusb_error_name(ret));
            break;
        }
        ctx->rx_active++;
    }
    return ctx->rx_active;
}

/**
 * Cancel all IN transfers. A transfer that is not in flight is ignored.
 */
static void gsusb_rx_cancel_all(gsusb_context_t *ctx) {
    for (int i = 0; i < GS_USB_RX_TRANSFERS; i++) {
        if (ctx->rx_transfers[i]) {
            libusb_cancel_transfer(ctx->rx_transfers[i]);
        }
    }
}

/**
 * RX thread function: handle libusb events until all IN transfers are done.
 */
#ifdef _WIN32
static DWORD WINAPI gsusb_rx_thread(void *arg) {
#else
static void *gsusb_rx_thread(void *arg) {
#endif
    gsusb_context_t *ctx = (gsusb_context_t *)arg;
    int stop_wait_ms = 0;

    while (ctx->rx_active > 0) {
        if (!ctx->running) {
            /* Cancel in each loop: a TX thread can resubmit a transfer while
             * the stop request occurs. */
            gsusb_rx_cancel_all(ctx);
            if (stop_wait_ms >= GS_USB_STOP_TIMEOUT_MS) {
                csp_print("GS_USB[%s]: Timeout while RX transfers stop\n", ctx->name);
                break;
            }
            stop_wait_ms += GS_USB_EVENT_POLL_MS;
        }

        struct timeval tv = { 0, GS_USB_EVENT_POLL_MS * 1000 };
        int ret = libusb_handle_events_timeout_completed(ctx->usb_ctx, &tv, NULL);
        if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_INTERRUPTED) {
            csp_print("GS_USB[%s]: USB event error: %s\n", ctx->name, libusb_error_name(ret));
            usleep(100000);  /* 100ms backoff on error */
        }
    }

    if (ctx->running) {
        csp_print("GS_USB[%s]: All RX transfers stopped, no more frames are received\n", ctx->name);
    }

#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

/**
 * TX function for CSP
 */
static int csp_can_gsusb_tx_frame(void *driver_data, uint32_t id, const uint8_t *data, uint8_t dlc) {
    gsusb_context_t *ctx = (gsusb_context_t *)driver_data;

    if (dlc > CAN_MAX_DLEN) {
        return CSP_ERR_INVAL;
    }

    gs_host_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.echo_id = 0;  /* No echo for TX */
    frame.can_id = id | 0x80000000;  /* Set extended frame flag */
    frame.can_dlc = dlc;
    frame.channel = ctx->channel;
    frame.flags = 0;
    memcpy(frame.data, data, dlc);

    /* Send only the classic frame size. The full structure has room for
     * 64 data bytes and spans two USB packets, which the device does not
     * expect when FD mode is off. */
    int transferred;
    int ret = libusb_bulk_transfer(ctx->dev_handle, GS_USB_ENDPOINT_OUT,
                                   (unsigned char *)&frame, GS_HOST_FRAME_CLASSIC_SIZE,
                                   &transferred, GS_USB_TIMEOUT);

    if (ret != LIBUSB_SUCCESS) {
        csp_print("GS_USB[%s]: TX error: %s\n", ctx->name, libusb_error_name(ret));
        return CSP_ERR_TX;
    }

    return CSP_ERR_NONE;
}

int csp_can_gsusb_enumerate(void) {
    libusb_context *ctx = NULL;
    libusb_device **list = NULL;
    int count = 0;

    if (libusb_init(&ctx) != LIBUSB_SUCCESS) {
        return -1;
    }

    ssize_t device_count = libusb_get_device_list(ctx, &list);
    if (device_count < 0) {
        libusb_exit(ctx);
        return -1;
    }

    /* Known GS_USB compatible devices */
    const struct {
        uint16_t vid;
        uint16_t pid;
        const char *name;
    } known_devices[] = {
        { 0x1d50, 0x606f, "Candlelight/Canable" },
        { 0x1d50, 0x606d, "Candlelight" },
        { 0x1209, 0x2323, "cantact" },
        { 0x16d0, 0x0f30, "CANable MKS" },
        { 0, 0, NULL }
    };

    for (ssize_t i = 0; i < device_count; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != LIBUSB_SUCCESS) {
            continue;
        }

        for (int j = 0; known_devices[j].vid != 0; j++) {
            if (desc.idVendor == known_devices[j].vid &&
                desc.idProduct == known_devices[j].pid) {
                csp_print("GS_USB: Found %s at bus %d, device %d\n",
                         known_devices[j].name,
                         libusb_get_bus_number(list[i]),
                         libusb_get_device_address(list[i]));
                count++;
                break;
            }
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(ctx);
    return count;
}

int csp_can_gsusb_open_and_add_interface(const csp_can_gsusb_conf_t *conf,
                                          const char *ifname,
                                          unsigned int node_id,
                                          csp_iface_t **return_iface) {
    csp_can_gsusb_conf_t default_conf = CSP_CAN_GSUSB_CONF_DEFAULT;
    if (conf == NULL) {
        conf = &default_conf;
    }

    if (ifname == NULL) {
        ifname = CSP_IF_CAN_DEFAULT_NAME;
    }

    csp_print("GS_USB INIT %s: vid=0x%04x, pid=0x%04x, channel=%d, bitrate=%u\n",
              ifname, conf->vendor_id, conf->product_id, conf->channel, conf->bitrate);

    /* Allocate context */
    gsusb_context_t *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return CSP_ERR_NOMEM;
    }

    strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
    ctx->channel = conf->channel;
    ctx->running = 0;

    /* Initialize libusb */
    int ret = libusb_init(&ctx->usb_ctx);
    if (ret != LIBUSB_SUCCESS) {
        csp_print("GS_USB[%s]: libusb_init failed: %s\n", ctx->name, libusb_error_name(ret));
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Find and open device */
    uint16_t vid = conf->vendor_id;
    uint16_t pid = conf->product_id;

    /* Auto-detect if VID/PID not specified */
    if (vid == 0 || pid == 0) {
        vid = GS_USB_VID_CANDLELIGHT;
        pid = GS_USB_PID_CANDLELIGHT;
    }

    ctx->dev_handle = libusb_open_device_with_vid_pid(ctx->usb_ctx, vid, pid);
    if (ctx->dev_handle == NULL) {
        csp_print("GS_USB[%s]: Could not find device (VID=0x%04x, PID=0x%04x)\n",
                  ctx->name, vid, pid);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Claim interface */
#ifdef _WIN32
    /* On Windows with WinUSB driver, we may need to set auto-detach */
    libusb_set_auto_detach_kernel_driver(ctx->dev_handle, 1);
#else
    /* Detach kernel driver if attached (Linux) */
    if (libusb_kernel_driver_active(ctx->dev_handle, 0) == 1) {
        ret = libusb_detach_kernel_driver(ctx->dev_handle, 0);
        if (ret != LIBUSB_SUCCESS) {
            csp_print("GS_USB[%s]: Could not detach kernel driver: %s\n",
                      ctx->name, libusb_error_name(ret));
        }
    }
#endif

    ret = libusb_claim_interface(ctx->dev_handle, 0);
    if (ret != LIBUSB_SUCCESS) {
        csp_print("GS_USB[%s]: Could not claim interface: %s\n",
                  ctx->name, libusb_error_name(ret));
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Set host format (byte order) */
    gs_host_config_t host_conf = { .byte_order = 0x0000BEEF };  /* Little endian */
    ret = gsusb_control_out(ctx, GS_USB_BREQ_HOST_FORMAT, 0, &host_conf, sizeof(host_conf));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not set host format\n", ctx->name);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Get device configuration */
    gs_device_config_t dev_conf;
    ret = gsusb_control_in(ctx, GS_USB_BREQ_DEVICE_CONFIG, 0, &dev_conf, sizeof(dev_conf));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not get device config\n", ctx->name);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }
    /* gs_usb firmware uses 0-indexed interface count, convert to 1-indexed */
    uint8_t num_channels = dev_conf.icount + 1;
    csp_print("GS_USB[%s]: Device: SW=0x%08x, HW=0x%08x, channels=%d\n",
              ctx->name, dev_conf.sw_version, dev_conf.hw_version, num_channels);

    if (ctx->channel >= num_channels) {
        csp_print("GS_USB[%s]: Invalid channel %d (device has %d channels)\n",
                  ctx->name, ctx->channel, num_channels);
        gsusb_free(ctx);
        return CSP_ERR_INVAL;
    }

    /* Reset the channel before configuration. A previous session can leave
     * the channel started, and the device ignores bit timing changes then. */
    gs_device_mode_t reset_mode = { .mode = GS_CAN_MODE_RESET, .flags = 0 };
    ret = gsusb_control_out(ctx, GS_USB_BREQ_MODE, ctx->channel, &reset_mode, sizeof(reset_mode));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not reset channel, continuing\n", ctx->name);
    }

    /* Get bit timing constants */
    ret = gsusb_control_in(ctx, GS_USB_BREQ_BT_CONST, ctx->channel,
                           &ctx->bt_const, sizeof(ctx->bt_const));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not get bit timing constants\n", ctx->name);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }
    csp_print("GS_USB[%s]: Clock: %u Hz, features: 0x%08x\n",
              ctx->name, ctx->bt_const.fclk_can, ctx->bt_const.feature);

    /* Calculate and set bit timing */
    gs_device_bittiming_t bt;
    ret = gsusb_calc_bittiming(ctx, conf->bitrate, &bt);
    if (ret != CSP_ERR_NONE) {
        gsusb_free(ctx);
        return ret;
    }

    ret = gsusb_control_out(ctx, GS_USB_BREQ_BITTIMING, ctx->channel, &bt, sizeof(bt));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not set bit timing\n", ctx->name);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Start CAN interface */
    gs_device_mode_t mode;
    mode.mode = GS_CAN_MODE_START;
    mode.flags = 0;
    ret = gsusb_control_out(ctx, GS_USB_BREQ_MODE, ctx->channel, &mode, sizeof(mode));
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: Could not set mode\n", ctx->name);
        gsusb_free(ctx);
        return CSP_ERR_DRIVER;
    }

    /* Setup CSP interface */
    ctx->iface.name = ctx->name;
    ctx->iface.addr = node_id;
    ctx->iface.interface_data = &ctx->ifdata;
    ctx->iface.driver_data = ctx;
    ctx->ifdata.tx_func = csp_can_gsusb_tx_frame;
    ctx->ifdata.pbufs = NULL;

    /* Add interface to CSP */
    ret = csp_can_add_interface(&ctx->iface);
    if (ret != CSP_ERR_NONE) {
        csp_print("GS_USB[%s]: csp_can_add_interface() failed: %d\n", ctx->name, ret);
        gsusb_free(ctx);
        return ret;
    }

    /* Submit the IN transfers, then start the RX thread that handles them */
    ctx->running = 1;
    if (gsusb_rx_submit_all(ctx) == 0) {
        csp_print("GS_USB[%s]: Could not submit any RX transfer\n", ctx->name);
        ctx->running = 0;
        /* Note: Can't remove interface from CSP once added */
        return CSP_ERR_DRIVER;
    }
    if (ctx->rx_active < GS_USB_RX_TRANSFERS) {
        csp_print("GS_USB[%s]: Only %d of %d RX transfers submitted\n",
                  ctx->name, ctx->rx_active, GS_USB_RX_TRANSFERS);
    }

    if (pthread_create(&ctx->rx_thread, NULL, gsusb_rx_thread, ctx) != 0) {
        csp_print("GS_USB[%s]: Could not create RX thread\n", ctx->name);
        ctx->running = 0;
        /* Cancel the submitted transfers and wait for them in this thread */
        gsusb_rx_thread(ctx);
        /* Note: Can't remove interface from CSP once added */
        return CSP_ERR_NOMEM;
    }
    ctx->rx_thread_started = 1;

    csp_print("GS_USB[%s]: Interface started successfully\n", ctx->name);

    if (return_iface) {
        *return_iface = &ctx->iface;
    }

    return CSP_ERR_NONE;
}

int csp_can_gsusb_stop(csp_iface_t *iface) {
    gsusb_context_t *ctx = (gsusb_context_t *)iface->driver_data;

    /* Signal RX thread to stop. The thread cancels the IN transfers and
     * returns when they are done, or after GS_USB_STOP_TIMEOUT_MS. */
    ctx->running = 0;

    /* Wait for thread to finish */
    if (ctx->rx_thread_started) {
#ifdef _WIN32
        WaitForSingleObject(ctx->rx_thread, INFINITE);
        CloseHandle(ctx->rx_thread);
#else
        pthread_join(ctx->rx_thread, NULL);
#endif
        ctx->rx_thread_started = 0;
    }

    /* Stop CAN interface */
    gs_device_mode_t mode;
    mode.mode = GS_CAN_MODE_RESET;
    mode.flags = 0;
    gsusb_control_out(ctx, GS_USB_BREQ_MODE, ctx->channel, &mode, sizeof(mode));

    gsusb_free(ctx);
    return CSP_ERR_NONE;
}
