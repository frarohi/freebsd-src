/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2025 Frank Hilgendorf (frank.hilgendorf@posteo.de)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * 'fgp' supports the older Apple Fountain and Geyser trackpads
 * that are not supported by wsp.
 * It implements the evdev interface for them as a Semi-MT trackpad.
 *
 * This driver reuses some code from both the atp and the wsp trackpad driver.
 * - From atp: USB interface, raw data extraction, span detection,
 *             dev driver interface
 * - From wsp: evdev interface.
 *
 * code prefix naming:
 * usb: USB bus related
 * dev: /dev mouse output interface
 * evd: evdev output interface
 * pad: trackpad hardware related
 * prc: data processing
 */

#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mouse.h>
#include <sys/mutex.h>
#include <sys/stddef.h>
#include <sys/stdint.h>
#include <sys/sysctl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>

#include "usbdevs.h"

#define USB_DEBUG_VAR fgp_debug
#include <dev/usb/usb_debug.h>

#ifdef EVDEV_SUPPORT
#include "opt_evdev.h"

#include <dev/evdev/evdev.h>
#include <dev/evdev/input.h>
#endif /* EVDEV_SUPPORT */

#ifdef USB_DEBUG_VAR
#ifdef USB_DEBUG
#define DPRINTN(n, fmt, ...)			\
	do {								\
		if ((USB_DEBUG_VAR) >= (n)) {	\
			printf(fmt, ##__VA_ARGS__); \
		}								\
	} while (0)
#else /* !USB_DEBUG */
#define DPRINTN(...)	\
	do {				\
	} while (0)
#endif /* USB_DEBUG */
#endif /* USB_DEBUG_VAR */

#define FGP_DRIVER_NAME "fgp"

/*
 * Threshold (in interrupt cycles) of consecutive empty frames.
 * When the threshold is reached, the Pad is reset
 * in order to stop further transmision
 */
#define FGP_PAD_IDLENESS_THRESHOLD 5

/*
 * fgp implements a semi multitouch evdev device
 * Finger 0: Min x,y
 * Finger 1: Max x,y
 */
#define FGP_EVD_MAX_SLOTS 2

/* Fingers reported via evdev interface */
#define FGP_EVD_MAX_FINGERS 3

/* Signal/noise ratios */
#define FGP_EVD_SN_COORD 150
#define FGP_EVD_SN_CPR	 20

/* Maximum contact pressure value */
#define FGP_EVD_CPR_MAX 100

/* Use Button tap+click to emulate Buttons 1-3 */
#define FGP_EVD_BUTTON_TAP_AND_CLICK 1

/* Shift scale factor for touch center of gravity */
#define FGP_PRC_XY_SHIFT_BITS 8

/* Sensor values noise level */
#define FGP_PRC_NOISE_THRESHOLD 2

/*
 * Ignore pressure spans with cumulative pressure
 * below this value.
 */
#define FGP_PRC_TOUCH_MIN_CPR 10

/*
 * Maximum ssumed finger width [mm].
 * Will be converted to maximum (sensors) width of a touch
 */
#define FGP_PRC_FINGER_MAX_WIDTH_MM 15

/*
 * The divisor used to translate
 * location delta to movement mickeys.
 */
#define FGP_DEV_MOVE_SCALE_FACTOR 8

/* The divisor used to translate
 * location delta to scroll steps.
 */
#define FGP_DEV_SCROLL_SCALE_FACTOR 8

/* Hardware related parameters */
#define FGP_PAD_MODE_LENGTH  8
#define FGP_PAD_DATA_BUF_MAX 1024
#define FGP_PAD_MAX_XSENSORS 26
#define FGP_PAD_MAX_YSENSORS 16

/* Tunables */
static struct fgp_sysctl_params {
	int dev_move_scale_factor;
	int dev_scroll_scale_factor;
	int evd_button_tap_and_click;
} fgp_sysctl_params = {
	.dev_move_scale_factor = FGP_DEV_MOVE_SCALE_FACTOR,
	.dev_scroll_scale_factor = FGP_DEV_SCROLL_SCALE_FACTOR,
	.evd_button_tap_and_click = FGP_EVD_BUTTON_TAP_AND_CLICK,
};

static SYSCTL_NODE(_hw_usb, OID_AUTO, fgp, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "USB FGP");

static int fgp_sysctl_dev_move_scale_factor_handler(SYSCTL_HANDLER_ARGS);
SYSCTL_PROC(_hw_usb_fgp, OID_AUTO, dev_move_scale_factor,
    CTLTYPE_UINT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
    &fgp_sysctl_params.dev_move_scale_factor,
    sizeof(fgp_sysctl_params.dev_move_scale_factor),
    fgp_sysctl_dev_move_scale_factor_handler, "IU",
    "fgp device movement scale factor");

static int fgp_sysctl_dev_scroll_scale_factor_handler(SYSCTL_HANDLER_ARGS);
SYSCTL_PROC(_hw_usb_fgp, OID_AUTO, dev_scroll_scale_factor,
    CTLTYPE_UINT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
    &fgp_sysctl_params.dev_scroll_scale_factor,
    sizeof(fgp_sysctl_params.dev_scroll_scale_factor),
    fgp_sysctl_dev_scroll_scale_factor_handler, "IU",
    "fgp device scroll scale factor");

/* evdev interface */
#ifdef EVDEV_SUPPORT
SYSCTL_UINT(_hw_usb_fgp, OID_AUTO, evdev_button_tap_and_click, CTLFLAG_RWTUN,
    &fgp_sysctl_params.evd_button_tap_and_click, FGP_EVD_BUTTON_TAP_AND_CLICK,
    "Use combined tap and click for evdev button 1-3 emulation");
#endif /* EVDEV_SUPPORT */

#ifdef USB_DEBUG
enum fgp_log_level {
	FGP_LLEVEL_DISABLED = 0,
	FGP_LLEVEL_ERROR,
	FGP_LLEVEL_INFO,    /* for diagnostics */
	FGP_LLEVEL_DEBUG,   /* for troubleshooting */
};
static int fgp_debug = FGP_LLEVEL_ERROR; /* the default is to only log errors */
SYSCTL_INT(_hw_usb_fgp, OID_AUTO, debug, CTLFLAG_RWTUN, &fgp_debug,
    FGP_LLEVEL_ERROR, "FGP debug level");
#endif /* USB_DEBUG */

/* product types supported */
enum fgp_pad_product {
	PRODUCT_FOUNTAIN,
	PRODUCT_GEYSER1,
	PRODUCT_GEYSER1_17inch,
	PRODUCT_GEYSER2,
	PRODUCT_GEYSER3,
	PRODUCT_GEYSER4,
	PRODUCT_FOUNTAIN_GEYSER_MAX /* keep this at the end */
};

/* transfer protocol types */
enum fgp_pad_type {
	PROTO_GEYSER1,
	PROTO_GEYSER2,
	PROTO_GEYSER3,
	PROTO_GEYSER4
};

typedef int8_t fgp_pad_sensor;	   /* raw sensor data */
typedef int16_t fgp_pad_axis_data; /* normalized sensor data */
typedef uint16_t fgp_prc_location; /* calculated touch location */

/* Array holds sensor values + one guard value 0 */
typedef struct fgp_pad_xy {
	fgp_pad_axis_data x[FGP_PAD_MAX_XSENSORS + 1];
	fgp_pad_axis_data y[FGP_PAD_MAX_YSENSORS + 1];
} fgp_pad_xy;

typedef struct fgp_pad_axis {
	uint8_t sensors;
	uint8_t size_mm;
} fgp_pad_axis;

/* device-specific configuration */
struct fgp_pad_params {
	struct fgp_pad_axis x;
	struct fgp_pad_axis y;
	uint8_t data_len; /* for sensor data */
	enum fgp_pad_type proto;
	char *name;
};

static const struct fgp_pad_params
    fgp_pad_parameters[PRODUCT_FOUNTAIN_GEYSER_MAX] = {
	    [PRODUCT_FOUNTAIN] = { .data_len = 81,
		.x.sensors = 16,
		.y.sensors = 16,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER1,
		.name = "Apple Trackpad Fountain" },
	    [PRODUCT_GEYSER1] = { .data_len = 81,
		.x.sensors = 16,
		.y.sensors = 16,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER1,
		.name = "Apple Trackpad Geyser-1" },
	    [PRODUCT_GEYSER1_17inch] = { .data_len = 81,
		.x.sensors = 26,
		.y.sensors = 16,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER1,
		.name = "Apple Trackpad Geyser-1-17" },
	    [PRODUCT_GEYSER2] = { .data_len = 64,
		.x.sensors = 15,
		.y.sensors = 9,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER2,
		.name = "Apple Trackpad Geyser-2" },
	    [PRODUCT_GEYSER3] = { .data_len = 64,
		.x.sensors = 16,
		.y.sensors = 16,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER3,
		.name = "Apple Trackpad Geyser-3" },
	    [PRODUCT_GEYSER4] = { .data_len = 64,
		.x.sensors = 20,
		.y.sensors = 10,
		.x.size_mm = 100,
		.y.size_mm = 50,
		.proto = PROTO_GEYSER4,
		.name = "Apple Trackpad Geyser-4" }
    };

static const STRUCT_USB_HOST_ID fgp_usb_models[] = {
	/* PowerBooks Feb 2005, iBooks G4 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x020e, PRODUCT_FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x020f, PRODUCT_FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0210, PRODUCT_FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x030a, PRODUCT_FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x030b, PRODUCT_GEYSER1) },

	/* PowerBooks Oct 2005 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0214, PRODUCT_GEYSER2) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0215, PRODUCT_GEYSER2) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0216, PRODUCT_GEYSER2) },

	/* Core Duo MacBook & MacBook Pro */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0217, PRODUCT_GEYSER3) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0218, PRODUCT_GEYSER3) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0219, PRODUCT_GEYSER3) },

	/* Core2 Duo MacBook & MacBook Pro */
	{ USB_VPI(USB_VENDOR_APPLE, 0x021a, PRODUCT_GEYSER4) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x021b, PRODUCT_GEYSER4) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x021c, PRODUCT_GEYSER4) },

	/* Core2 Duo MacBook3,1 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0229, PRODUCT_GEYSER4) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x022a, PRODUCT_GEYSER4) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x022b, PRODUCT_GEYSER4) },

	/* 17 inch PowerBook */
	{ USB_VPI(USB_VENDOR_APPLE, 0x020d, PRODUCT_GEYSER1_17inch) },
};

#define FGP_DEV_FIFO_BUF_SIZE	  8  /* bytes */
#define FGP_DEV_FIFO_QUEUE_MAXLEN 50 /* units */

enum fgp_usb_xfer_mode {
	FGP_USB_INTR_DT,
	FGP_USB_RESET,
	FGP_USB_N_TRANSFER,
};

struct fgp_softc; /* forward declaration */

/*
 * The following structure captures the touch state of an axis.
 */
typedef struct fgp_prc_pspan {
	fgp_prc_location loc_min;     /* minimum location */
	fgp_prc_location loc_max;     /* maximum location */
	fgp_prc_location loc_min_cpr; /* compression at loc_min */
	fgp_prc_location loc_max_cpr; /* compression at loc_min */
	uint8_t loc_n;		      /* number of touch locations detected */
} fgp_prc_pspan;

/*
 * The following structure captures the touch state of an axis.
 */
typedef struct fgp_evd_slot {
	fgp_prc_location abs_X; /* Finger position X */
	fgp_prc_location abs_Y; /* Finger position Y */
	uint16_t prs;		/* Finger Pressure   */
} fgp_evd_slot;

#define FGP_STATE_DEV_OPENED  0x01
#define FGP_STATE_EVD_OPENED  0x02
#define FGP_STATE_USB_READING 0x04

struct fgp_softc {
	device_t fgp_device;
	struct mtx fgp_mutex; /* for synchronization */
	uint8_t fgp_io_state;
	struct usb_device *usb_device;
	struct usb_xfer *usb_transfer[FGP_USB_N_TRANSFER];
	char pad_mode_bytes[FGP_PAD_MODE_LENGTH]; /* device mode */
	/* data from t-pad in USB INTR packets. */
	fgp_pad_sensor pad_data_frame[FGP_PAD_DATA_BUF_MAX] __aligned(4);
	uint8_t pad_idlecount;
	fgp_pad_xy pad_baseline;
	const struct fgp_pad_params *pad_params; /* device configuration */
	fgp_prc_pspan prc_pspans_x;
	fgp_prc_pspan prc_pspans_y;
	uint8_t prc_phys_button;     /* Physical button state */
	uint8_t prc_phys_button_o;   /* Last Physical button state */
	uint8_t pad_fingers_o;	     /* Last fingers count */
	struct usb_fifo_sc dev_fifo; /* dev output FIFO */
	mousehw_t dev_mouse_hw;
	mousemode_t dev_mouse_mode;
	mousestatus_t dev_mouse_status;
	int dev_pollrate;
	uint8_t dev_fflags;
	uint32_t dev_loc_x;
	uint32_t dev_loc_y;
	int16_t dev_delta_x;
	int16_t dev_delta_y;
#ifdef EVDEV_SUPPORT
	struct evdev_dev *evd_device;
	fgp_evd_slot evd_slots_o[FGP_EVD_MAX_SLOTS];
	u_int evd_click_and_tap;
#endif /* EVDEV_SUPPORT */
};

/*
 * The last byte of the fountain-geyser sensor data contains status bits; the
 * following values define the meanings of these bits.
 * (only Geyser 3/4)
 */
enum fgp_pad_geyser34_status_bits {
	FGP_PAD_STATUS_BUTTON = (uint8_t)0x01, /* The button was pressed */
	FGP_PAD_STATUS_BASE_UPDATE =
	    (uint8_t)0x04, /* Data from an untouched pad */
	FGP_PAD_STATUS_FROM_RESET =
	    (uint8_t)0x10, /* Reset previously performed */
};

typedef enum fgp_pad_device_mode {
	PAD_RAW_SENSOR_MODE = (uint8_t)0x01,
	PAD_HID_MODE = (uint8_t)0x08
} fgp_pad_device_mode;

/*
 * function prototypes
 */

/* forward declarations */
struct fgp_pad_params;
enum fgp_pad_frame_state;

/*
 * Input
 */
/* USB interface. */
static int fgp_usb_probe(device_t self);
static int fgp_usb_attach(device_t dev);
static int fgp_usb_detach(device_t dev);
static void fgp_usb_intr_cb(struct usb_xfer *usb_transfer,
    usb_error_t usb_error);
static void fgp_usb_reset_cb(struct usb_xfer *usb_transfer,
    usb_error_t usb_error);

/* Trackpad interface. */
static usb_error_t fgp_pad_set_device_mode(struct fgp_softc *,
    fgp_pad_device_mode);
static void fgp_pad_read_sensor_data(const fgp_pad_sensor *sensor_data,
    const struct fgp_pad_params *pad_params, fgp_pad_xy *raw_data,
    uint8_t *pad_status);
static void fgp_pad_store_baseline(fgp_pad_xy *pad_data,
    fgp_pad_xy *pad_baseline);

/* data processing */
static void fgp_prc_detect_touches(fgp_pad_axis_data *pad_data,
    fgp_pad_axis_data *pad_baseline, const struct fgp_pad_axis *axis,
    fgp_prc_pspan *spans);
static void fgp_prc_send_data(struct fgp_softc *sc);

/* dev interface */
static int fgp_dev_open(struct usb_fifo *dev_fifo, int dev_flags);
static void fgp_dev_close(struct usb_fifo *dev_fifo, int dev_flags);
static int fgp_dev_ioctl(struct usb_fifo *dev_fifo, u_long cmd, void *addr,
    int dev_flags);
static void fgp_dev_reset_buf(struct fgp_softc *);
static void fgp_dev_start_read(struct usb_fifo *dev_fifo);
static void fgp_dev_stop_read(struct usb_fifo *dev_fifo);

/* evdev interface */
#ifdef EVDEV_SUPPORT
static int fgp_evd_open(struct evdev_dev *evdev);
static int fgp_evd_close(struct evdev_dev *evdev);

static const struct evdev_methods fgp_evd_methods = {
	.ev_open = &fgp_evd_open,
	.ev_close = &fgp_evd_close,
};
#endif /* EVDEV_SUPPORT */

static struct usb_fifo_methods fgp_dev_methods = {
	.f_open = &fgp_dev_open,
	.f_close = &fgp_dev_close,
	.f_ioctl = &fgp_dev_ioctl,
	.f_start_read = &fgp_dev_start_read,
	.f_stop_read = &fgp_dev_stop_read,
	.basename[0] = FGP_DRIVER_NAME,
};

static const struct usb_config fgp_usb_xfer_config[FGP_USB_N_TRANSFER] = {
	[FGP_USB_INTR_DT] = {
		.type      = UE_INTERRUPT,
		.endpoint  = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.pipe_bof      = 1, /* block pipe on failure */
			.short_xfer_ok = 1,
		},
		.bufsize   = FGP_PAD_DATA_BUF_MAX,
		.callback  = &fgp_usb_intr_cb,
	},
	[FGP_USB_RESET] = {
		.type      = UE_CONTROL,
		.endpoint  = 0, /* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize   = sizeof(struct usb_device_request) + FGP_PAD_MODE_LENGTH,
		.callback  = &fgp_usb_reset_cb,
		.interval  = 0,  /* no pre-delay */
	},
};

static usb_error_t
fgp_pad_set_device_mode(struct fgp_softc *sc, fgp_pad_device_mode newMode)
{
	uint8_t mode_value;
	usb_error_t err;

	if ((newMode != PAD_RAW_SENSOR_MODE) && (newMode != PAD_HID_MODE)) {
		return (USB_ERR_INVAL);
	}
	if (newMode == PAD_RAW_SENSOR_MODE) {
		mode_value = (uint8_t)0x04;
	} else {
		mode_value = newMode;
	}

	err = usbd_req_get_report(sc->usb_device, NULL, sc->pad_mode_bytes,
	    sizeof(sc->pad_mode_bytes), 0, 0x03, 0x00);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		DPRINTF("Failed to read device mode (%d)\n", err);
		return (err);
	}

	/* already in requested mode? */
	if (sc->pad_mode_bytes[0] == mode_value) {
		return (err);
	}

	/*
	 * XXX Need to wait at least 250ms for hardware to get
	 * ready. The device mode handling appears to be handled
	 * asynchronously and we should not issue these commands too
	 * quickly.
	 */
	pause("WHW", hz / 4);

	sc->pad_mode_bytes[0] = mode_value;
	err = usbd_req_set_report(sc->usb_device, NULL /* mutex */,
	    sc->pad_mode_bytes, sizeof(sc->pad_mode_bytes),
	    0 /* interface idx */, 0x03 /* type */, 0x00 /* id */);
	return (err);
}

static void
fgp_usb_reset_cb(struct usb_xfer *usb_transfer, usb_error_t usb_error)
{
	usb_device_request_t req;
	struct usb_page_cache *pc;
	struct fgp_softc *sc = usbd_xfer_softc(usb_transfer);
	uint8_t mode_value = 0x04;

	switch (USB_GET_STATE(usb_transfer)) {
	case USB_ST_SETUP:
		sc->pad_mode_bytes[0] = mode_value;
		req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
		req.bRequest = UR_SET_REPORT;
		USETW2(req.wValue, (uint8_t)0x03 /* type */,
		    (uint8_t)0x00 /* id */);
		USETW(req.wIndex, 0);
		USETW(req.wLength, FGP_PAD_MODE_LENGTH);

		pc = usbd_xfer_get_frame(usb_transfer, 0);
		usbd_copy_in(pc, 0, &req, sizeof(req));
		pc = usbd_xfer_get_frame(usb_transfer, 1);
		usbd_copy_in(pc, 0, sc->pad_mode_bytes, FGP_PAD_MODE_LENGTH);

		usbd_xfer_set_frame_len(usb_transfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(usb_transfer, 1, FGP_PAD_MODE_LENGTH);
		usbd_xfer_set_frames(usb_transfer, 2);
		usbd_transfer_submit(usb_transfer);
		break;

	case USB_ST_TRANSFERRED:
	default:
		break;
	}
}

static void
fgp_pad_store_baseline(fgp_pad_xy *pad_data, fgp_pad_xy *pad_baseline)
{
	int i;

	DPRINTFN(FGP_LLEVEL_DEBUG, "[core] Pressure update baseline\n");
	/*
	 * Add noise threshold offset to baseline.
	 * This will reduce the contribution from noisy
	 * lower pressure readings.
	 */
	for (i = 0; i < FGP_PAD_MAX_XSENSORS; i++) {
		pad_baseline->x[i] = pad_data->x[i] + FGP_PRC_NOISE_THRESHOLD;
	}
	for (i = 0; i < FGP_PAD_MAX_YSENSORS; i++) {
		pad_baseline->y[i] = pad_data->y[i] + FGP_PRC_NOISE_THRESHOLD;
	}
};

static void
fgp_prc_send_data(struct fgp_softc *sc)
{
	DPRINTFN(FGP_LLEVEL_DEBUG,
	    "[core] spanX .loc_n: %d, loc_min: %d, loc_min_cpr: %d, loc_max: %d, loc_max_cpr: %d\n",
	    sc->prc_pspans_x.loc_n, sc->prc_pspans_x.loc_min,
	    sc->prc_pspans_x.loc_min_cpr, sc->prc_pspans_x.loc_max,
	    sc->prc_pspans_x.loc_max_cpr);
	DPRINTFN(FGP_LLEVEL_DEBUG,
	    "[core] spanY .loc_n: %d, loc_min: %d, loc_min_cpr: %d, loc_max: %d, loc_max_cpr: %d\n",
	    sc->prc_pspans_y.loc_n, sc->prc_pspans_y.loc_min,
	    sc->prc_pspans_y.loc_min_cpr, sc->prc_pspans_y.loc_max,
	    sc->prc_pspans_y.loc_max_cpr);

	uint16_t pad_fingers = max(sc->prc_pspans_x.loc_n,
	    sc->prc_pspans_y.loc_n);

	enum fgp_pad_button_action {
		FGP_PAD_BUTTON_NONE,
		FGP_PAD_BUTTON_DOWN,
		FGP_PAD_BUTTON_UP,
	};

	enum fgp_pad_button_action pad_button_action = FGP_PAD_BUTTON_NONE;
	/* Phys button changed */
	if (sc->prc_phys_button != sc->prc_phys_button_o) {
		if (sc->prc_phys_button)
			pad_button_action = FGP_PAD_BUTTON_DOWN;
		else
			pad_button_action = FGP_PAD_BUTTON_UP;
		sc->dev_mouse_status.flags |= MOUSE_STDBUTTONSCHANGED;
	}

	uint8_t dev_buf_buttons;
	uint16_t dev_stat_buttons;
	uint8_t dev_buf[8];
	int16_t dev_dx = 0;
	int16_t dev_dy = 0;
	int16_t dev_dv = 0;
	uint32_t dev_loc_x;
	uint32_t dev_loc_y;

/* evdev interface */
#ifdef EVDEV_SUPPORT
	/* Event device opened and requested? -> Use event channel */
	if ((sc->fgp_io_state & FGP_STATE_EVD_OPENED) &&
	    (evdev_rcpt_mask & EVDEV_RCPT_HW_MOUSE)) {
		uint8_t do_sync = 0;
		fgp_evd_slot pad_slots[FGP_EVD_MAX_SLOTS] = {
			[0] = {
				.abs_X = 0,
				.abs_Y = 0,
				.prs = 0,
			},
			[1] = {
				.abs_X = 0,
				.abs_Y = 0,
				.prs = 0,
			}
		};

		/* touches to MT slots */
		if (pad_fingers) {
			pad_slots[0].abs_X = sc->prc_pspans_x.loc_min;
			pad_slots[0].abs_Y = sc->prc_pspans_y.loc_min;
			/*
			 * If X has not less touches than Y take highest cpr
			 * from X , otherwise from Y
			 */
			pad_slots[0].prs = sc->prc_pspans_x.loc_n >=
				sc->prc_pspans_y.loc_n ?
			    max(sc->prc_pspans_x.loc_min_cpr,
				sc->prc_pspans_x.loc_max_cpr) :
			    max(sc->prc_pspans_y.loc_min_cpr,
				sc->prc_pspans_y.loc_max_cpr);
			if (pad_fingers > 1) {
				pad_slots[1].abs_X = sc->prc_pspans_x.loc_n >
					1 ?
				    sc->prc_pspans_x.loc_max :
				    sc->prc_pspans_x.loc_min;
				pad_slots[1].abs_Y = sc->prc_pspans_y.loc_n >
					1 ?
				    sc->prc_pspans_y.loc_max :
				    sc->prc_pspans_y.loc_min;
				pad_slots[1].prs = pad_slots[0].prs;
			}
		};

		for (int sl = 0; sl < FGP_EVD_MAX_SLOTS; sl++) {
			if (memcmp(&pad_slots[sl], &sc->evd_slots_o[sl],
				sizeof(fgp_evd_slot))) {
				if (pad_slots[sl].prs) {
					evdev_push_abs(sc->evd_device,
					    ABS_MT_SLOT, sl);
					/* Was Slot inactive? */
					if (sc->evd_slots_o[sl].prs == 0) {
						evdev_push_abs(sc->evd_device,
						    ABS_MT_TRACKING_ID, sl);
					}
					evdev_push_abs(sc->evd_device,
					    ABS_MT_POSITION_X,
					    pad_slots[sl].abs_X);
					evdev_push_abs(sc->evd_device,
					    ABS_MT_POSITION_Y,
					    pad_slots[sl]
						.abs_Y); /* Accumulate finger
							    data */
					evdev_push_abs(sc->evd_device,
					    ABS_MT_PRESSURE, pad_slots[sl].prs);
				} else {
					evdev_push_abs(sc->evd_device,
					    ABS_MT_SLOT, sl);
					evdev_push_abs(sc->evd_device,
					    ABS_MT_TRACKING_ID, -1);
				}
				do_sync = 1;

				sc->evd_slots_o[sl] = pad_slots[sl];
			}
		}

		/* Buttons */
		if (sc->evd_click_and_tap) {
			switch (pad_button_action) {
			case FGP_PAD_BUTTON_DOWN:
				switch (pad_fingers) {
				case 0:
				case 1:
					evdev_push_key(sc->evd_device, BTN_LEFT,
					    1);
					break;
				case 2:
					evdev_push_key(sc->evd_device,
					    BTN_RIGHT, 1);
					break;
				default:
					evdev_push_key(sc->evd_device,
					    BTN_MIDDLE, 1);
				}
				do_sync = 1;
				break;
			case FGP_PAD_BUTTON_UP:
				switch (sc->pad_fingers_o) {
				case 0:
				case 1:
					evdev_push_key(sc->evd_device, BTN_LEFT,
					    0);
					break;
				case 2:
					evdev_push_key(sc->evd_device,
					    BTN_RIGHT, 0);
					break;
				default:
					evdev_push_key(sc->evd_device,
					    BTN_MIDDLE, 0);
				}
				do_sync = 1;
				break;
			default:
				break;
			}
		}
		if (sc->prc_phys_button == 0) {
			sc->evd_click_and_tap =
			    (fgp_sysctl_params.evd_button_tap_and_click != 0);
		}

		if (do_sync) {
			/* BTN_EVENTS are apparently generated by libevdev
			 * Due to EVDEV_FLAG_MT_STCOMPAT ?
			 */
			evdev_sync(sc->evd_device);
		}
	}
#endif /* EVDEV_SUPPORT */
	if (sc->fgp_io_state & FGP_STATE_DEV_OPENED) {
		/* /dev/fgp output */

		/* buttons */
		dev_buf_buttons = MOUSE_MSC_BUTTONS;
		dev_stat_buttons = 0;
		if (sc->prc_phys_button) {
			switch (pad_fingers) {
			case 0:
			case 1:
				dev_buf_buttons &= ~MOUSE_MSC_BUTTON1UP;
				dev_stat_buttons |= MOUSE_BUTTON1DOWN;
				break;
			case 2:
				dev_buf_buttons &= ~MOUSE_MSC_BUTTON3UP;
				dev_stat_buttons |= MOUSE_BUTTON3DOWN;
				break;
			default:
				dev_buf_buttons &= ~MOUSE_MSC_BUTTON2UP;
				dev_stat_buttons |= MOUSE_BUTTON2DOWN;
			}
		}
		sc->dev_mouse_status.button = dev_stat_buttons;

		/* Movement Delta */
		dev_loc_x = sc->prc_pspans_x.loc_n ?
		    (sc->prc_pspans_x.loc_min + sc->prc_pspans_x.loc_max) /
			imax(sc->prc_pspans_x.loc_n, 2) :
		    0;
		dev_loc_y = sc->prc_pspans_y.loc_n ?
		    (sc->prc_pspans_y.loc_min + sc->prc_pspans_y.loc_max) /
			imax(sc->prc_pspans_y.loc_n, 2) :
		    0;

		/* If finger count has not changed, calculate delta */
		if (sc->pad_fingers_o == pad_fingers) {
			sc->dev_delta_x += (dev_loc_x - sc->dev_loc_x) * 4;
			sc->dev_delta_y += (dev_loc_y - sc->dev_loc_y) * 4;
			DPRINTFN(FGP_LLEVEL_DEBUG,
			    "[dev] delta_x: %i, delta_y: %i\n", sc->dev_delta_x,
			    sc->dev_delta_y);
			switch (pad_fingers) {
			case 1:
				/* mouse pointer */
				dev_dx = sc->dev_delta_x /
				    (int)fgp_sysctl_params
					.dev_move_scale_factor;
				dev_dy = sc->dev_delta_y /
				    (int)fgp_sysctl_params
					.dev_move_scale_factor;
				sc->dev_delta_x %=
				    (int)fgp_sysctl_params
					.dev_move_scale_factor;
				sc->dev_delta_y %=
				    (int)fgp_sysctl_params
					.dev_move_scale_factor;
				break;
			case 2:
				/* scroll wheel */
				dev_dv = sc->dev_delta_y /
				    (int)fgp_sysctl_params
					.dev_scroll_scale_factor;
				sc->dev_delta_y %= (int)fgp_sysctl_params
						       .dev_scroll_scale_factor;
				break;
			default:
				/* no action */
				dev_dx = dev_dy = dev_dv = 0;
			}
		} else {
			sc->dev_delta_x = 0;
			sc->dev_delta_y = 0;
		}
		/* Invert y axes */
		dev_dy = -dev_dy;
		dev_dv = -dev_dv;
		/* clip to if range */
		dev_dx = imin(dev_dx, 254);
		dev_dx = imax(dev_dx, -256);
		dev_dy = imin(dev_dy, 254);
		dev_dy = imax(dev_dy, -256);
		dev_dv = imin(dev_dv, 126);
		dev_dv = imax(dev_dv, -128);
		/* update ioctl data */
		sc->dev_mouse_status.dx += dev_dx;
		sc->dev_mouse_status.dy += dev_dy;
		sc->dev_mouse_status.dz += dev_dv;

		DPRINTFN(FGP_LLEVEL_DEBUG, "[dev] dx: %i, dy: %i, dv: %i\n",
		    dev_dx, dev_dy, dev_dv);
		/*
		 * Encode the mouse data in standard format;
		 * refer to mouse(4)
		 */
		dev_buf[0] = sc->dev_mouse_mode.syncmask[1];
		dev_buf[0] |= dev_buf_buttons;
		dev_buf[1] = dev_dx >> 1;
		dev_buf[2] = dev_dy >> 1;
		dev_buf[3] = dev_dx - (dev_dx >> 1);
		dev_buf[4] = dev_dy - (dev_dy >> 1);
		/* Encode extra bytes for level 1 */
		if (sc->dev_mouse_mode.level == 1) {
			dev_buf[5] = dev_dv >> 1;
			dev_buf[6] = dev_dv - (dev_dv >> 1);
			dev_buf[7] = MOUSE_SYS_EXTBUTTONS;
		}

		usb_fifo_put_data_linear(sc->dev_fifo.fp[USB_FIFO_RX], dev_buf,
		    sc->dev_mouse_mode.packetsize, 1);

		/* remember location */
		sc->dev_loc_x = dev_loc_x;
		sc->dev_loc_y = dev_loc_y;
	}
	sc->prc_phys_button_o = sc->prc_phys_button;
	sc->pad_fingers_o = pad_fingers;
}

/*
 * Interpret the data from the X and Y pressure sensors. This function
 * is called separately for the X and Y sensor arrays. The data in the
 * USB packet is laid out in the following manner:
 *
 * sensor_data:
 *            --,--,Y1,Y2,--,Y3,Y4,--,Y5,...,Y10, ... X1,X2,--,X3,X4
 *  indices:   0  1  2  3  4  5  6  7  8 ...  15  ... 20 21 22 23 24
 *
 * '--' (in the above) indicates that the value is unimportant.
 *
 * Information about the above layout was obtained from the
 * implementation of the AppleTouch driver in Linux.
 *
 * parameters:
 *   sensor_data
 *       raw sensor data from the USB packet.
 *   pad_params
 *       The number of elements in X/Y axis
 *       The protocol to use to interpret the data
 *   raw_data
 *       The array to be initialized with the readings.
 *   pad_status
 *       Status byte sent by pad
 */
static void
fgp_pad_read_sensor_data(const fgp_pad_sensor *sensor_data,
    const struct fgp_pad_params *pad_params, fgp_pad_xy *raw_data,
    uint8_t *pad_status)
{
	uint8_t i;
	uint8_t di; /* index into sensor data */

	memset(raw_data, 0, sizeof(fgp_pad_axis_data));

	switch (pad_params->proto) {
	case PROTO_GEYSER1:
		/*
		 * For Geyser 1, the sensors are laid out in pairs
		 * every 5 bytes.
		 */
		/* X */
		for (i = 0, di = 2; i < 8; di += 5, i++) {
			raw_data->x[i] = (fgp_pad_axis_data)sensor_data[di];
			raw_data->x[i + 8] = (fgp_pad_axis_data)
			    sensor_data[di + 2];
			if (pad_params->x.sensors > 16) {
				raw_data->x[i + 16] = (fgp_pad_axis_data)
				    sensor_data[di + 40];
			}
		}
		/* Y */
		for (i = 0, di = 1; i < 8; di += 5, i++) {
			raw_data->y[i] = (fgp_pad_axis_data)sensor_data[di];
			raw_data->y[i + 8] = (fgp_pad_axis_data)
			    sensor_data[di + 2];
		}
		break;
	case PROTO_GEYSER2:
		/* X */
		for (i = 0, di = 19; i < pad_params->x.sensors; /* empty */) {
			raw_data->x[i++] = (fgp_pad_axis_data)sensor_data[di++];
			raw_data->x[i++] = (fgp_pad_axis_data)sensor_data[di++];
			di++;
		}
		/* Y */
		for (i = 0, di = 1; i < pad_params->y.sensors; /* empty */) {
			raw_data->y[i++] = (fgp_pad_axis_data)sensor_data[di++];
			raw_data->y[i++] = (fgp_pad_axis_data)sensor_data[di++];
			di++;
		}
		break;
	case PROTO_GEYSER3:
	case PROTO_GEYSER4:
		/* X */
		for (i = 0, di = 20; i < pad_params->x.sensors; /* empty */) {
			raw_data->x[i++] = (fgp_pad_axis_data)sensor_data[di++];
			raw_data->x[i++] = (fgp_pad_axis_data)sensor_data[di++];
			di++;
		}
		/* Y */
		for (i = 0, di = 2; i < pad_params->y.sensors; /* empty */) {
			raw_data->y[i++] = (fgp_pad_axis_data)sensor_data[di++];
			raw_data->y[i++] = (fgp_pad_axis_data)sensor_data[di++];
			di++;
		}
		break;
	default:
		break;
	}
	/* Guard values */
	raw_data->x[pad_params->x.sensors] = 0;
	raw_data->y[pad_params->y.sensors] = 0;
	/* Pad status bits */
	*pad_status = sensor_data[pad_params->data_len - 1];
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	DPRINTFN(FGP_LLEVEL_DEBUG, "[raw]----------\n");
	if (*pad_status != 0)
		DPRINTFN(FGP_LLEVEL_DEBUG, "[raw] *pad_status: %x\n",
		    *pad_status);
	DPRINTFN(FGP_LLEVEL_DEBUG, "[raw] X: ");
	for (i = 0; i < pad_params->x.sensors; i++) {
		DPRINTN(FGP_LLEVEL_DEBUG, "%.3i ", raw_data->x[i]);
	}
	DPRINTN(FGP_LLEVEL_DEBUG, "\n");
	DPRINTFN(FGP_LLEVEL_DEBUG, "[raw] Y: ");
	for (i = 0; i < pad_params->y.sensors; i++) {
		DPRINTN(FGP_LLEVEL_DEBUG, "%.3i ", raw_data->y[i]);
	}
	DPRINTN(FGP_LLEVEL_DEBUG, "\n");
	DPRINTFN(FGP_LLEVEL_DEBUG, "[raw]----------\n");
}

static void
fgp_prc_detect_touches(fgp_pad_axis_data *pad_data,
    fgp_pad_axis_data *pad_baseline, const struct fgp_pad_axis *axis,
    fgp_prc_pspan *spans)
{
	uint8_t i;
	uint16_t maxp;	      /* max pressure seen within a span */
	uint16_t wcpr;	      /* cumulated pressure working      */
	uint16_t wcog;	      /* cumulated center of gravity working */
	uint8_t wwdt;	      /* width, number of sensors in touch */
	fgp_prc_location loc; /* resulting touch location value */
	bool finger_complete;
	/*
	 * Maximum amount of sensors for a single finger
	 * A finger is completed and a new one started
	 * when this limit is exceeded
	 */
	uint8_t wwdt_max = (FGP_PRC_FINGER_MAX_WIDTH_MM * axis->sensors) /
	    axis->size_mm;

	/* The following is a tiny state machine to track fingers */

	/* state of the pressure span */
	enum fgp_prc_pspan_state {
		FGP_PSPAN_INACTIVE,
		FGP_PSPAN_INCREASING,
		FGP_PSPAN_DECREASING,
	} pspan_state = FGP_PSPAN_INACTIVE;

	spans->loc_min = 0;
	spans->loc_max = 0;
	spans->loc_min_cpr = 0;
	spans->loc_max_cpr = 0;
	spans->loc_n = 0;

	maxp = 0;
	wwdt = 0;
	wcpr = 0;
	wcog = 0;
	finger_complete = false;

	/*
	 * cover sensors + trailing guard
	 * The guard value is 0 making sure that
	 * the last finger will be completed within the loop
	 */
	for (i = 0; i <= axis->sensors; i++) {
		pad_data[i] = pad_data[i] > pad_baseline[i] ?
		    pad_data[i] - pad_baseline[i] :
		    0;
		if (pad_data[i] == 0) {
			if (pspan_state == FGP_PSPAN_INACTIVE) {
				/*
				 * There is no pressure information for this
				 * sensor, and we aren't tracking a finger.
				 */
				continue;
			} else {
				/*
				 * There is a finger in progress
				 * now it is finished.
				 */
				pspan_state = FGP_PSPAN_INACTIVE;
				finger_complete = true;
			}
		} else if (wwdt == wwdt_max) {
			finger_complete = true;
			pspan_state = FGP_PSPAN_INCREASING;
		} else {
			/* pad_data[i] > 0 */
			switch (pspan_state) {
			case FGP_PSPAN_INACTIVE:
				pspan_state = FGP_PSPAN_INCREASING;
				maxp = pad_data[i];
				break;
			case FGP_PSPAN_INCREASING:
				if (pad_data[i] > maxp)
					maxp = pad_data[i];
				else if ((pad_data[i] << 1) <= maxp)
					pspan_state = FGP_PSPAN_DECREASING;
				break;
			case FGP_PSPAN_DECREASING:
				if (pad_data[i] > pad_data[i - 1]) {
					/*
					 * Slope is rising again.
					 * This is the beginning of
					 * the next finger
					 */
					pspan_state = FGP_PSPAN_INCREASING;
					finger_complete = true;
					continue;
				}
				break;
			}
		}
		if (finger_complete) {
			DPRINTFN(FGP_LLEVEL_DEBUG,
			    "[core] (%d) Finger complete.  wwdt: %d, wcum: %d\n",
			    axis->sensors, wwdt, wcpr);
			if (wcpr >= FGP_PRC_TOUCH_MIN_CPR) {
				loc = (wcog << FGP_PRC_XY_SHIFT_BITS) / wcpr;
				DPRINTFN(FGP_LLEVEL_DEBUG,
				    "[core] (%d) New Finger.  loc: %d\n",
				    axis->sensors, loc);

				/* second+ finger? */
				if (spans->loc_n) {
					spans->loc_max = loc;
					spans->loc_max_cpr = wcpr;
				} else {
					spans->loc_min = loc;
					spans->loc_min_cpr = wcpr;
				}
				spans->loc_n++;
			}
			maxp = wwdt = wcpr = wcog = 0;
			finger_complete = false;
		} else {
			/* Accumulate finger data */
			wwdt++;
			wcpr += pad_data[i];
			wcog += pad_data[i] * (i + 1);
		}
	}

	if (spans->loc_n) {
		DPRINTFN(FGP_LLEVEL_DEBUG,
		    "[core] (%d) loc_n: %d, loc_min: %d, loc_min_cpr: %d, loc_max: %d, loc_max_cpr: %d\n",
		    axis->sensors, spans->loc_n, spans->loc_min,
		    spans->loc_min_cpr, spans->loc_max, spans->loc_max_cpr);
	}
}

static int
fgp_usb_probe(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (ENXIO);
	}
	if (uaa->info.bInterfaceClass != UICLASS_HID) {
		return (ENXIO);
	}
	if ((usbd_lookup_id_by_uaa(fgp_usb_models, sizeof(fgp_usb_models), uaa)) == 0) {
		return ((uaa->info.bInterfaceProtocol == UIPROTO_MOUSE) ?
			BUS_PROBE_DEFAULT :
			ENXIO);
	}
	return (ENXIO);
}

static int
fgp_usb_attach(device_t dev)
{
	struct fgp_softc *sc = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_error_t err;
	void *descriptor_ptr = NULL;
	uint16_t descriptor_len;
	int hid_data_len;
	unsigned long di;

	DPRINTFN(FGP_LLEVEL_DEBUG, "sc=%p\n", sc);

	sc->fgp_device = dev;
	sc->usb_device = uaa->device;
	di = USB_GET_DRIVER_INFO(uaa);
	sc->pad_params = &fgp_pad_parameters[di];

	/* Get HID descriptor */
	if (usbd_req_get_hid_desc(uaa->device, NULL, &descriptor_ptr,
		&descriptor_len, M_TEMP,
		uaa->info.bIfaceIndex) != USB_ERR_NORMAL_COMPLETION) {
		return (ENXIO);
	}

	/* Get HID report descriptor length */
	hid_data_len = hid_report_size_max(descriptor_ptr, descriptor_len,
	    hid_input, NULL);
	free(descriptor_ptr, M_TEMP);

	if (hid_data_len != sc->pad_params->data_len) {
		DPRINTF("Trackpad datalengths: param=%d, actual=%d\n",
		    sc->pad_params->data_len, hid_data_len);
		return (ENXIO);
	}

	/*
	 * By default the touchpad behaves like an HID device, sending
	 * packets with reportID = 2. Such reports contain only
	 * limited information--they encode movement deltas and button
	 * events,--but do not include data from the pressure
	 * sensors. The device input mode can be switched from HID
	 * reports to raw sensor data using vendor-specific USB
	 * control commands.
	 * FOUNTAIN devices will give an error when trying to switch
	 * input mode, so we skip this command
	 */
	if (di == PRODUCT_FOUNTAIN) {
		DPRINTF("device mode switch skipped: Fountain device\n");
	} else if ((err = fgp_pad_set_device_mode(sc, PAD_RAW_SENSOR_MODE)) !=
	    0) {
		DPRINTF("failed to set mode to 'RAW_SENSOR' (%d)\n", err);
		return (ENXIO);
	}

	mtx_init(&sc->fgp_mutex, "fgp lock", NULL, MTX_DEF | MTX_RECURSE);

	err = usbd_transfer_setup(uaa->device, &uaa->info.bIfaceIndex,
	    sc->usb_transfer, fgp_usb_xfer_config, FGP_USB_N_TRANSFER, sc,
	    &sc->fgp_mutex);
	if (err) {
		DPRINTF("error=%s\n", usbd_errstr(err));
		goto detach;
	}

	if (usb_fifo_attach(sc->usb_device, sc, &sc->fgp_mutex,
		&fgp_dev_methods, &sc->dev_fifo, device_get_unit(dev), -1,
		uaa->info.bIfaceIndex, UID_ROOT, GID_OPERATOR, 0644)) {
		goto detach;
	}

	device_set_usb_desc(dev);

	sc->dev_mouse_hw.buttons = 3;
	sc->dev_mouse_hw.iftype = MOUSE_IF_USB;
	sc->dev_mouse_hw.type = MOUSE_PAD;
	sc->dev_mouse_hw.model = MOUSE_MODEL_GENERIC;
	sc->dev_mouse_hw.hwid = 0;
	sc->dev_mouse_mode.protocol = MOUSE_PROTO_MSC;
	sc->dev_mouse_mode.rate = -1;
	sc->dev_mouse_mode.resolution = MOUSE_RES_UNKNOWN;
	sc->dev_mouse_mode.packetsize = MOUSE_MSC_PACKETSIZE;
	sc->dev_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
	sc->dev_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
	sc->dev_mouse_mode.accelfactor = 0;
	sc->dev_mouse_mode.level = 0;

	sc->fgp_io_state = 0;

	sc->prc_phys_button = 0;
	sc->prc_phys_button_o = 0;
	sc->pad_idlecount = 0;
	sc->pad_fingers_o = 0;
	sc->dev_loc_x = 0;
	sc->dev_loc_y = 0;
	sc->dev_delta_x = 0;
	sc->dev_delta_y = 0;

#ifdef EVDEV_SUPPORT
	sc->evd_slots_o[0].abs_X = sc->evd_slots_o[0].abs_Y =
	    sc->evd_slots_o[0].prs = 0;
	sc->evd_slots_o[1].abs_X = sc->evd_slots_o[1].abs_Y =
	    sc->evd_slots_o[1].prs = 0;
	sc->evd_click_and_tap = fgp_sysctl_params.evd_button_tap_and_click;

	sc->evd_device = evdev_alloc();
	evdev_set_name(sc->evd_device, sc->pad_params->name);
	evdev_set_phys(sc->evd_device, device_get_nameunit(dev));
	evdev_set_id(sc->evd_device, BUS_USB, uaa->info.idVendor,
	    uaa->info.idProduct, 0);
	evdev_set_serial(sc->evd_device, usb_get_serial(uaa->device));
	evdev_set_methods(sc->evd_device, sc, &fgp_evd_methods);

	evdev_support_prop(sc->evd_device, INPUT_PROP_POINTER);
	evdev_support_prop(sc->evd_device, INPUT_PROP_SEMI_MT);
	evdev_support_event(sc->evd_device, EV_SYN);
	evdev_support_event(sc->evd_device, EV_ABS);
	evdev_support_event(sc->evd_device, EV_KEY);

	evdev_support_nfingers(sc->evd_device, FGP_EVD_MAX_FINGERS);
	/* finger position */
	evdev_support_abs(sc->evd_device, ABS_MT_SLOT, 0, FGP_EVD_MAX_SLOTS, 0,
	    0, 0);
	evdev_support_abs(sc->evd_device, ABS_MT_TRACKING_ID, -1, 100, 0, 0, 0);
	evdev_support_abs(sc->evd_device, ABS_MT_POSITION_X, 0,
	    (sc->pad_params->x.sensors << FGP_PRC_XY_SHIFT_BITS),
	    (sc->pad_params->x.sensors << FGP_PRC_XY_SHIFT_BITS) /
		FGP_EVD_SN_COORD,
	    0,
	    (sc->pad_params->x.sensors << FGP_PRC_XY_SHIFT_BITS) /
		sc->pad_params->x.size_mm);
	evdev_support_abs(sc->evd_device, ABS_MT_POSITION_Y, 0,
	    (sc->pad_params->y.sensors << FGP_PRC_XY_SHIFT_BITS),
	    (sc->pad_params->y.sensors << FGP_PRC_XY_SHIFT_BITS) /
		FGP_EVD_SN_COORD,
	    0,
	    (sc->pad_params->y.sensors << FGP_PRC_XY_SHIFT_BITS) /
		sc->pad_params->y.size_mm);

	/* finger pressure */
	evdev_support_abs(sc->evd_device, ABS_MT_PRESSURE, 0, FGP_EVD_CPR_MAX,
	    FGP_EVD_CPR_MAX / FGP_EVD_SN_CPR, 0, 0);

	evdev_support_key(sc->evd_device, BTN_TOUCH);
	evdev_support_key(sc->evd_device, BTN_TOOL_FINGER);
	evdev_support_key(sc->evd_device, BTN_TOOL_DOUBLETAP);
	evdev_support_key(sc->evd_device, BTN_TOOL_TRIPLETAP);

	evdev_support_key(sc->evd_device, BTN_LEFT);
	evdev_support_key(sc->evd_device, BTN_RIGHT);
	evdev_support_key(sc->evd_device, BTN_MIDDLE);

	evdev_set_flag(sc->evd_device, EVDEV_FLAG_MT_TRACK);
	evdev_set_flag(sc->evd_device, EVDEV_FLAG_MT_AUTOREL);

	/*
	 * Another libinput/udev workaround :
	 * Lacking udev, we don't have a chance to tell libinput that this is a
	 * touchpad. Hence, it is mismatched to be a mouse and expected to send
	 * mouse specific events. Apparently the only way to have it recognized
	 * as touchpad is to set this flag
	 */
	evdev_set_flag(sc->evd_device, EVDEV_FLAG_MT_STCOMPAT);

	err = evdev_register_mtx(sc->evd_device, &sc->fgp_mutex);
	if (err) {
		goto detach;
	}
#endif /* EVDEV_SUPPORT */

	return (0);

detach:
	fgp_usb_detach(dev);
	return (ENOMEM);
}

static int
fgp_usb_detach(device_t dev)
{
	struct fgp_softc *sc;

	sc = device_get_softc(dev);

	/* before leaving, reset trackpad to HID mode */
	fgp_pad_set_device_mode(sc, PAD_HID_MODE);

	usb_fifo_detach(&sc->dev_fifo);

#ifdef EVDEV_SUPPORT
	evdev_free(sc->evd_device);
#endif /* EVDEV_SUPPORT */

	usbd_transfer_unsetup(sc->usb_transfer, FGP_USB_N_TRANSFER);
	mtx_destroy(&sc->fgp_mutex);
	return (0);
}

static void
fgp_usb_intr_cb(struct usb_xfer *usb_transfer, usb_error_t usb_error)
{
	struct fgp_softc *sc = usbd_xfer_softc(usb_transfer);
	struct usb_page_cache *pc;
	int frame_len;

	uint8_t pad_status_bits;
	uint8_t pad_xy_cnt;
	fgp_pad_xy pad_raw_data;

	/* trackpad data frame state */
	enum {
		FGP_PAD_STATE_UNKNOWN, /* State unknown (initial)  */
		FGP_PAD_STATE_ERRAXIS, /* One Axis without reading */
		FGP_PAD_STATE_IDLE,    /* Trackpad is idle         */
		FGP_PAD_STATE_RESET,   /* Trackpad is reset        */
		FGP_PAD_STATE_BASE,    /* Base value data frame    */
		FGP_PAD_STATE_READING, /* Touch reading data frame */
	} pad_fstate = FGP_PAD_STATE_UNKNOWN;

	usbd_xfer_status(usb_transfer, &frame_len, NULL, NULL, NULL);

	switch (USB_GET_STATE(usb_transfer)) {
	case USB_ST_TRANSFERRED:
		pc = usbd_xfer_get_frame(usb_transfer, 0);
		usbd_copy_out(pc, 0, sc->pad_data_frame, frame_len);

		/* Ignore damaged data packets */
		if (frame_len != sc->pad_params->data_len) {
			DPRINTN(FGP_LLEVEL_INFO,
			    "Trackpad datalengths: param=%d, actual=%d\n",
			    sc->pad_params->data_len, frame_len);
		} else {
			sc->dev_mouse_status.flags &= ~(
			    MOUSE_STDBUTTONSCHANGED | MOUSE_POSCHANGED);
			sc->dev_mouse_status.obutton =
			    sc->dev_mouse_status.button;

			fgp_pad_read_sensor_data(sc->pad_data_frame,
			    sc->pad_params, &pad_raw_data, &pad_status_bits);

			/*
			 * If this is a base update (from an untouched
			 * pad), we should set the base values for the sensor
			 * data; deltas with respect to these base values can
			 * be used as pressure readings subsequently.
			 */
			if ((sc->pad_params->proto == PROTO_GEYSER3) ||
			    (sc->pad_params->proto == PROTO_GEYSER4)) {
				if (pad_status_bits &
				    FGP_PAD_STATUS_BASE_UPDATE)
					pad_fstate = FGP_PAD_STATE_BASE;
			}
			if ((pad_fstate != FGP_PAD_STATE_BASE) &&
			    (pad_fstate != FGP_PAD_STATE_RESET)) {
				/* Read physical button state */
				sc->prc_phys_button =
				    ((pad_status_bits & FGP_PAD_STATUS_BUTTON) >
					0) ?
				    MOUSE_BUTTON1DOWN :
				    0;

				/* Get pressure readings and detect touches on
				 * both axes. */
				fgp_prc_detect_touches(pad_raw_data.x,
				    sc->pad_baseline.x, &sc->pad_params->x,
				    &sc->prc_pspans_x);

				fgp_prc_detect_touches(pad_raw_data.y,
				    sc->pad_baseline.y, &sc->pad_params->y,
				    &sc->prc_pspans_y);

				/*
				 * Evaluate frame data
				 */
				pad_xy_cnt = (sc->prc_pspans_x.loc_n > 0) ? 1 :
									    0;
				pad_xy_cnt += (sc->prc_pspans_y.loc_n > 0) ? 1 :
									     0;
				switch (pad_xy_cnt) {
				case 0:
					pad_fstate =
					    FGP_PAD_STATE_IDLE; /* trackpad is
								   (untouched)
								 */
					break;
				case 1:
					pad_fstate =
					    FGP_PAD_STATE_ERRAXIS; /* Data of
								      one Axis
								      missing */
					break;
				case 2:
					pad_fstate =
					    FGP_PAD_STATE_READING; /* Valid
								      reading */
				}
			}

			switch (pad_fstate) {
			case FGP_PAD_STATE_RESET:
				/* Not used */
				break;
			case FGP_PAD_STATE_BASE:
				/* Update pressure base data */
				fgp_pad_store_baseline(&pad_raw_data,
				    &sc->pad_baseline);
				break;
			case FGP_PAD_STATE_IDLE:
				/* calculate touch end
				 * When count of idle frames exceeds threshold,
				 * touch cycles are reset
				 */
				fgp_prc_send_data(sc);

				/* Trackpad reset ? */
				if (!sc->prc_phys_button) {
					/*
					 * The Fountain/Geyser device continues
					 * to trigger interrupts at a fast rate
					 * even after touchpad activity has
					 * stopped. Upon detecting that the
					 * device has remained idle beyond a
					 * threshold, we reinitialize it to
					 * silence the interrupts.
					 */
					sc->pad_idlecount++;
					if (sc->pad_idlecount >=
					    FGP_PAD_IDLENESS_THRESHOLD) {
						/*
						 * Use the last frame before we
						 * go idle for calibration on
						 * pads which do not send
						 * calibration frames.
						 */

						if (sc->pad_params->proto <
						    PROTO_GEYSER3) {
							fgp_pad_store_baseline(
							    &pad_raw_data,
							    &sc->pad_baseline);
						}

						DPRINTFN(FGP_LLEVEL_INFO,
						    "[core] idle reset\n");
						sc->pad_idlecount = 0;
						usbd_transfer_start(
						    sc->usb_transfer
							[FGP_USB_RESET]);
					}
				}
				break;
			case FGP_PAD_STATE_READING:
				fgp_prc_send_data(sc);
				break;
			default:
				break;
			}
		}

	case USB_ST_SETUP:
	tr_setup:
		/* In case /dev/fgp* is opened, check if we can put more data
		 * into the dev FIFO
		 */
		if ((usb_fifo_put_bytes_max(sc->dev_fifo.fp[USB_FIFO_RX]) !=
			0) ||
		    ((sc->fgp_io_state & FGP_STATE_DEV_OPENED) == 0)) {
			usbd_xfer_set_frame_len(usb_transfer, 0,
			    sc->pad_params->data_len);
			usbd_transfer_submit(usb_transfer);
		}
		break;

	default: /* Error */
		if (usb_error != USB_ERR_CANCELLED) {
			/* try clear stall first */
			usbd_xfer_set_stall(usb_transfer);
			goto tr_setup;
		}
		break;
	}
}

static void
fgp_usb_start_read(struct fgp_softc *sc)
{
	int rate;
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	if ((sc->fgp_io_state & FGP_STATE_USB_READING) == 0) {
		/* Check if we should override the default polling interval */
		rate = sc->dev_pollrate;
		/* Range check rate */
		if (rate > 1000) {
			rate = 1000;
		}
		/* Check for set rate */
		if ((rate > 0) && (sc->usb_transfer[FGP_USB_INTR_DT] != NULL)) {
			/* Stop current transfer, if any */
			usbd_transfer_stop(sc->usb_transfer[FGP_USB_INTR_DT]);
			/* Set new interval */
			usbd_xfer_set_interval(
			    sc->usb_transfer[FGP_USB_INTR_DT], 1000 / rate);
			/* Only set pollrate once */
			sc->dev_pollrate = 0;
		}

		usbd_transfer_start(sc->usb_transfer[FGP_USB_INTR_DT]);
		sc->fgp_io_state |= FGP_STATE_USB_READING;
		DPRINTFN(FGP_LLEVEL_DEBUG, "Done\n");
	}
}

static void
fgp_usb_stop_read(struct fgp_softc *sc)
{
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	if ((sc->fgp_io_state & FGP_STATE_USB_READING) != 0) {
		usbd_transfer_stop(sc->usb_transfer[FGP_USB_INTR_DT]);
		sc->fgp_io_state &= ~FGP_STATE_USB_READING;
		DPRINTFN(FGP_LLEVEL_DEBUG, "Done\n");
	}
}

#ifdef EVDEV_SUPPORT
static int
fgp_evd_open(struct evdev_dev *evdev)
{
	struct fgp_softc *sc = evdev_get_softc(evdev);
	int rc = 0;

	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	mtx_assert(&sc->fgp_mutex, MA_OWNED);

	fgp_usb_start_read(sc);

	sc->fgp_io_state |= FGP_STATE_EVD_OPENED;
	return (rc);
}

static int
fgp_evd_close(struct evdev_dev *evdev)
{
	struct fgp_softc *sc = evdev_get_softc(evdev);

	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	mtx_assert(&sc->fgp_mutex, MA_OWNED);
	if ((sc->fgp_io_state & FGP_STATE_DEV_OPENED) == 0) {
		fgp_usb_stop_read(sc);
	}

	sc->fgp_io_state &= ~FGP_STATE_EVD_OPENED;
	return (0);
}
#endif /* EVDEV_SUPPORT */

static int
fgp_dev_open(struct usb_fifo *dev_fifo, int dev_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(dev_fifo);

	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	/* check for duplicate open, should not happen */
	if (sc->dev_fflags & dev_flags) {
		return (EBUSY);
	}
	/* check for first open */
	if (sc->dev_fflags == 0) {
		/* reset ODV status */
		memset(&sc->dev_mouse_status, 0, sizeof(sc->dev_mouse_status));
	}
	if (dev_flags & FREAD) {
		if (usb_fifo_alloc_buffer(dev_fifo, FGP_DEV_FIFO_BUF_SIZE,
			FGP_DEV_FIFO_QUEUE_MAXLEN)) {
			return (ENOMEM);
		}
		sc->fgp_io_state |= FGP_STATE_DEV_OPENED;
	}

	sc->dev_fflags |= (dev_flags & (FREAD | FWRITE));

	return (0);
}

static void
fgp_dev_close(struct usb_fifo *dev_fifo, int dev_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(dev_fifo);
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	if (dev_flags & FREAD) {
		usb_fifo_free_buffer(dev_fifo);
		sc->fgp_io_state &= ~FGP_STATE_DEV_OPENED;
	}

	sc->dev_fflags &= ~(dev_flags & (FREAD | FWRITE));
}

static void
fgp_dev_reset_buf(struct fgp_softc *sc)
{
	/* reset read queue */
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	usb_fifo_reset(sc->dev_fifo.fp[USB_FIFO_RX]);
}

static int
fgp_dev_ioctl(struct usb_fifo *dev_fifo, u_long cmd, void *addr, int dev_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(dev_fifo);
	mousemode_t mode;
	int error = 0;

	mtx_lock(&sc->fgp_mutex);

	switch (cmd) {
	case MOUSE_GETHWINFO:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_GETHWINFO\n");
		*(mousehw_t *)addr = sc->dev_mouse_hw;
		break;
	case MOUSE_GETMODE:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_GETMODE\n");
		*(mousemode_t *)addr = sc->dev_mouse_mode;
		break;
	case MOUSE_SETMODE:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_SETMODE\n");
		mode = *(mousemode_t *)addr;

		if (mode.level == -1) {
			/* Don't change the current setting */
			;
		} else if ((mode.level < 0) || (mode.level > 1)) {
			error = EINVAL;
			break;
		}
		sc->dev_mouse_mode.level = mode.level;
		sc->dev_pollrate = mode.rate;
		sc->dev_mouse_hw.buttons = 3;

		if (sc->dev_mouse_mode.level == 0) {
			sc->dev_mouse_mode.protocol = MOUSE_PROTO_MSC;
			sc->dev_mouse_mode.packetsize = MOUSE_MSC_PACKETSIZE;
			sc->dev_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->dev_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->dev_mouse_mode.level == 1) {
			sc->dev_mouse_mode.protocol = MOUSE_PROTO_SYSMOUSE;
			sc->dev_mouse_mode.packetsize = MOUSE_SYS_PACKETSIZE;
			sc->dev_mouse_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->dev_mouse_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		fgp_dev_reset_buf(sc);
		break;
	case MOUSE_GETLEVEL:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_GETLEVEL\n");
		*(int *)addr = sc->dev_mouse_mode.level;
		break;
	case MOUSE_SETLEVEL:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_SETLEVEL\n");
		if ((*(int *)addr < 0) || (*(int *)addr > 1)) {
			error = EINVAL;
			break;
		}
		sc->dev_mouse_mode.level = *(int *)addr;
		sc->dev_mouse_hw.buttons = 3;

		if (sc->dev_mouse_mode.level == 0) {
			sc->dev_mouse_mode.protocol = MOUSE_PROTO_MSC;
			sc->dev_mouse_mode.packetsize = MOUSE_MSC_PACKETSIZE;
			sc->dev_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->dev_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->dev_mouse_mode.level == 1) {
			sc->dev_mouse_mode.protocol = MOUSE_PROTO_SYSMOUSE;
			sc->dev_mouse_mode.packetsize = MOUSE_SYS_PACKETSIZE;
			sc->dev_mouse_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->dev_mouse_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		fgp_dev_reset_buf(sc);
		break;
	case MOUSE_GETSTATUS:
		DPRINTFN(FGP_LLEVEL_DEBUG, "MOUSE_GETSTATUS\n");
		mousestatus_t *status = (mousestatus_t *)addr;

		*status = sc->dev_mouse_status;
		if (status->dx || status->dy || status->dz) {
			status->flags |= MOUSE_POSCHANGED;
		}
		if (status->button != status->obutton) {
			status->flags |= MOUSE_BUTTONSCHANGED;
		}
		sc->dev_mouse_status.obutton = sc->dev_mouse_status.button;
		sc->dev_mouse_status.button = 0;
		sc->dev_mouse_status.dx = 0;
		sc->dev_mouse_status.dy = 0;
		sc->dev_mouse_status.dz = 0;
		break;
	default:
		DPRINTFN(FGP_LLEVEL_DEBUG, "default: ERR\n");
		error = ENOTTY;
		break;
	}

	mtx_unlock(&sc->fgp_mutex);
	return (error);
}

static void
fgp_dev_start_read(struct usb_fifo *dev_fifo)
{
	struct fgp_softc *sc = usb_fifo_softc(dev_fifo);

	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");
	fgp_usb_start_read(sc);
}

static void
fgp_dev_stop_read(struct usb_fifo *dev_fifo)
{
	struct fgp_softc *sc = usb_fifo_softc(dev_fifo);
	DPRINTFN(FGP_LLEVEL_DEBUG, "\n");

#ifdef EVDEV_SUPPORT
	if (sc->fgp_io_state & FGP_STATE_EVD_OPENED) {
		return;
	}
#endif /* EVDEV_SUPPORT */

	fgp_usb_stop_read(sc);
}

static int
fgp_sysctl_dev_move_scale_factor_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	int tmp;

	tmp = fgp_sysctl_params.dev_move_scale_factor;
	error = sysctl_handle_int(oidp, &tmp, 0, req);
	if (error != 0 || req->newptr == NULL) {
		return (error);
	}
	if (tmp == fgp_sysctl_params.dev_move_scale_factor) {
		return (0); /* no change */
	}
	if ((tmp == 0) || (tmp > (10 * FGP_DEV_MOVE_SCALE_FACTOR))) {
		return (EINVAL);
	}
	fgp_sysctl_params.dev_move_scale_factor = tmp;
	DPRINTFN(FGP_LLEVEL_INFO,
	    "[core] %s: resetting mickeys_scale_factor to %u\n",
	    FGP_DRIVER_NAME, tmp);

	return (0);
}

static int
fgp_sysctl_dev_scroll_scale_factor_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	u_int tmp;

	tmp = fgp_sysctl_params.dev_scroll_scale_factor;
	error = sysctl_handle_int(oidp, &tmp, 0, req);
	if (error != 0 || req->newptr == NULL) {
		return (error);
	}
	if (tmp == fgp_sysctl_params.dev_scroll_scale_factor) {
		return (0); /* no change */
	}
	if ((tmp == 0) || (tmp > (10 * FGP_DEV_SCROLL_SCALE_FACTOR))) {
		return (EINVAL);
	}
	fgp_sysctl_params.dev_scroll_scale_factor = tmp;
	DPRINTFN(FGP_LLEVEL_INFO,
	    "[core] %s: resetting dev_scroll_scale_factor to %u\n",
	    FGP_DRIVER_NAME, tmp);

	return (0);
}

static device_method_t fgp_usb_methods[] = {
	DEVMETHOD(device_probe,	 fgp_usb_probe),
	DEVMETHOD(device_attach, fgp_usb_attach),
	DEVMETHOD(device_detach, fgp_usb_detach),
	DEVMETHOD_END
};

static driver_t fgp_usb_driver = { .name = FGP_DRIVER_NAME,
	.methods = fgp_usb_methods,
	.size = sizeof(struct fgp_softc) };

DRIVER_MODULE(fgp, uhub, fgp_usb_driver, NULL, NULL);
MODULE_DEPEND(fgp, usb, 1, 1, 1);
MODULE_DEPEND(fgp, hid, 1, 1, 1);
#ifdef EVDEV_SUPPORT
MODULE_DEPEND(fgp, evdev, 1, 1, 1);
#endif /* EVDEV_SUPPORT */
MODULE_VERSION(fgp, 1);
USB_PNP_HOST_INFO(fgp_usb_models);
