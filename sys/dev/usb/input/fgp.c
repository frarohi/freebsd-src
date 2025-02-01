/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2014 Rohit Grover
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
 * Some tables, structures, definitions and constant values for the
 * touchpad protocol has been copied from Linux's
 * "drivers/input/mouse/bcm5974.c" which has the following copyright
 * holders under GPLv2. All device specific code in this driver has
 * been written from scratch. The decoding algorithm is based on
 * output from FreeBSD's usbdump.
 *
 * atp:
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2005      Johannes Berg (johannes@sipsolutions.net)
 * Copyright (C) 2005      Stelian Pop (stelian@popies.net)
 * Copyright (C) 2005      Frank Arnold (frank@scirocco-5v-turbo.de)
 * Copyright (C) 2005      Peter Osterlund (petero2@telia.com)
 * Copyright (C) 2005      Michael Hanselmann (linux-kernel@hansmi.ch)
 * Copyright (C) 2006      Nicolas Boichat (nicolas@boichat.ch)
 * Copyright (C) 2008      Henrik Rydberg (rydberg@euromail.se)
 * Copyright (C) 2008      Scott Shawcroft (scott.shawcroft@gmail.com)
 *
 * fgp:
 * Copyright (C) 2022      Frank Hilgendorf (frank.hilgendorf@posteo.de)
 */

/*
 * Author's note: 'fgp' supports the older Fountain/Geyser trackpads AND
 * implements the evdev interface.  
 * The driver merges the driver framework of recent atp driver  
 * with evdev support taken from the wsp driver.
 * The F/G sensor processing code has been rewritten and simplified.
 *
 * The Wellspring code from atp has been stripped off, to gain
 * a small dedicated module. wsp and fgp together provide evdev support 
 * for all versions of Apple trackpads.
 *
 * Naming conventions:
 * fgp: Fountain/Geyser (Track)Pad - This driver's name
 * itp: Input TrackPad             - Trackpad USB input interface
 * odv: Output DeVice              - /dev output interface
 * oev: Output EVent device        - evdev output interface
 * pad: TrackPAD device
 * fg : FountainGeyser             - FG trackpad data conversion logic
 *
 */


#include <sys/cdefs.h>
#include "opt_evdev.h"

#include <sys/types.h>
#include <sys/param.h>
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

#include <dev/hid/hid.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>

#include "usbdevs.h"

#define USB_DEBUG_VAR fgp_debug
#include <dev/usb/usb_debug.h>

#ifdef EVDEV_SUPPORT
#include <dev/evdev/input.h>
#include <dev/evdev/evdev.h>
#endif

#ifdef USB_DEBUG_VAR
#ifdef USB_DEBUG
#define	DPRINTN(n,fmt,...) do {		\
  if ((USB_DEBUG_VAR) >= (n)) {			\
    printf(fmt,	##__VA_ARGS__);	\
  }						\
} while (0)
#else  /* !USB_DEBUG */
#define	DPRINTN(...) do { } while (0)
#endif /* USB_DEBUG */
#endif /* USB_DEBUG_VAR */

#define	FGP_CLAMP(x,low,high) do {	\
	if ((x) < (low))				\
		(x) = (low);				\
	else if ((x) > (high))			\
		(x) = (high);				\
} while (0)

#define FGP_DRIVER_NAME "fgp"

/*
 * Driver specific options: the following options may be set by
 * `options' statements in the kernel configuration file.
 */

/* The divisor used to translate 
 * sensor reported delta to movement mickeys. 
*/
#define FGP_MICKEYS_SCALE_FACTOR   16

/* The divisor used to translate 
 * sensor reported delta to scroll steps. 
*/
#define FGP_SCROLL_SCALE_FACTOR    128

/*
 * The minimal initial touch pressure 
 * to detect the beginning of a touch as a tap 
 */
#define FGP_TAP_PRESSURE_THRESHOLD 35

/*
 * Threshold (in interrupt cycles) before a touch is settled.
 * Thus, fingers have time to settle for multi-touch tap events
 */
#define FGP_TOUCH_SETTLEMENT_CYCLES 5

/*
 * Threshold (in interrupt cycles) of consecutive empty frames.
 * When the threshold is reached, the Pad is reset 
 * in order to stop further transmision
 */
#define	FGP_IDLENESS_THRESHOLD 	    10

/*
 * Pressure values noise level
 */
#define FGP_SENSOR_NOISE_THRESHOLD  2

/* The multiplier used to scale up intermediate positions. */
#define FGP_SPAN_FACTOR            380

/* 
 * Finger detection filter settings 
 */

/* Ignore pressure spans with cumulative press. below this value. */
#define FGP_PSPAN_MIN_CUM_PRESSURE 10

/* Maximum allowed width for pressure-spans.*/
#define FGP_PSPAN_MAX_WIDTH        4

/* end of driver specific options */


/* Tunables */
static SYSCTL_NODE(_hw_usb, OID_AUTO, fgp, CTLFLAG_RW | CTLFLAG_MPSAFE, 0,
    "USB FGP");

#ifdef USB_DEBUG
enum fgp_log_level {
	FGP_LLEVEL_DISABLED = 0,
	FGP_LLEVEL_ERROR,
	FGP_LLEVEL_INFO,        /* for diagnostics */
	FGP_LLEVEL_DEBUG,       /* for troubleshooting */
	FGP_LLEVEL_DEVELOP,     /* for development */
};
static int fgp_debug = FGP_LLEVEL_ERROR; /* the default is to only log errors */
SYSCTL_INT(_hw_usb_fgp, OID_AUTO, debug, CTLFLAG_RWTUN,
    &fgp_debug, FGP_LLEVEL_DEBUG, "FGP debug level");
#endif /* USB_DEBUG */

static struct fgp_sysctl_params {
	uint32_t touch_settlement_cycles;
	uint32_t tap_pressure_threshold;
	uint32_t mickeys_scale_factor;
	uint32_t scroll_scale_factor;
} fgp_sysctl_params = {
	.touch_settlement_cycles = FGP_TOUCH_SETTLEMENT_CYCLES,
	.tap_pressure_threshold  = FGP_TAP_PRESSURE_THRESHOLD,
	.mickeys_scale_factor    = FGP_MICKEYS_SCALE_FACTOR,
	.scroll_scale_factor     = FGP_SCROLL_SCALE_FACTOR
};

SYSCTL_UINT(_hw_usb_fgp, OID_AUTO, touch_settlement_cycles, CTLFLAG_RWTUN,
    &fgp_sysctl_params.touch_settlement_cycles, FGP_TOUCH_SETTLEMENT_CYCLES, 
	"Count of interrupt cycles before start of processing");

SYSCTL_UINT(_hw_usb_fgp, OID_AUTO, tap_pressure_threshold, CTLFLAG_RWTUN,
    &fgp_sysctl_params.tap_pressure_threshold, FGP_TAP_PRESSURE_THRESHOLD,
    "Minimum initial pressure to detect a tap gesture");

static int fgp_sysctl_move_scale_factor_handler(SYSCTL_HANDLER_ARGS);
SYSCTL_PROC(_hw_usb_fgp, OID_AUTO, move_scale_factor,
    CTLTYPE_UINT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
    &fgp_sysctl_params.mickeys_scale_factor, sizeof(fgp_sysctl_params.mickeys_scale_factor),
    fgp_sysctl_move_scale_factor_handler, "IU",
    "movement scale factor");

static int fgp_sysctl_scroll_scale_factor_handler(SYSCTL_HANDLER_ARGS);
SYSCTL_PROC(_hw_usb_fgp, OID_AUTO, scroll_scale_factor,
    CTLTYPE_UINT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE,
    &fgp_sysctl_params.scroll_scale_factor, sizeof(fgp_sysctl_params.scroll_scale_factor),
    fgp_sysctl_scroll_scale_factor_handler, "IU",
    "scroll scale factor");

static void
fgp_sysctl_rangecheck(struct fgp_sysctl_params *sysctl)
{
	FGP_CLAMP(sysctl->touch_settlement_cycles, 1, 255);
	FGP_CLAMP(sysctl->tap_pressure_threshold,  1, 255);
}


/* product types supported */
enum fg_product {
	FOUNTAIN,
	GEYSER1,
	GEYSER1_17inch,
	GEYSER2,
	GEYSER3,
	GEYSER4,
	FOUNTAIN_GEYSER_PRODUCT_MAX /* keep this at the end */
};

/* trackpad header types */
enum fg_trackpad_type {
	FG_TRACKPAD_TYPE_GEYSER1,
	FG_TRACKPAD_TYPE_GEYSER2,
	FG_TRACKPAD_TYPE_GEYSER3,
	FG_TRACKPAD_TYPE_GEYSER4
};

/*
 * The following structure captures the touch state of an axis. 
 */
typedef struct fg_pspan {
	uint32_t location;     /* location (scaled using the FGP_SPAN_FACTOR) */
	uint16_t pressure_max; /* maximum compression found (per finger) */
	uint16_t fingers;	   /* number of fingers detected */
} fg_pspan;

#define FGP_SENSOR_DATA_BUF_MAX	1024
#define FGP_MAX_XSENSORS        26
#define FGP_MAX_YSENSORS        16

/* device-specific configuration */
struct fg_pad_params {
	uint8_t               data_len;   /* for sensor data */
	uint8_t               n_xsensors;
	uint8_t               n_ysensors;
	enum fg_trackpad_type prot;
	char                  *name;
};

// static  char *fg_pad_name_FOUNTAIN = "Apple Trackpad Fountain";
// static  char *fg_pad_name_GEYSER1 = "Apple Trackpad Geyser 1";
// static  char *fg_pad_name_GEYSER1_17inch = "Apple Trackpad Geyser 1 17";
// static  char *fg_pad_name_GEYSER2 = "Apple Trackpad Geyser 2";
// static  char *fg_pad_name_GEYSER3 = "Apple Trackpad Geyser 3";
// static  char *fg_pad_name_GEYSER4 = "Apple Trackpad Geyser 4";

static const struct fg_pad_params fg_dev_params[FOUNTAIN_GEYSER_PRODUCT_MAX] = {
	[FOUNTAIN] = {
		.data_len   = 81,
		.n_xsensors = 16,
		.n_ysensors = 16,
		.prot       = FG_TRACKPAD_TYPE_GEYSER1,
		.name       = "Apple Trackpad Fountain"
	},
	[GEYSER1] = {
		.data_len   = 81,
		.n_xsensors = 16,
		.n_ysensors = 16,
		.prot       = FG_TRACKPAD_TYPE_GEYSER1,
		.name       = "Apple Trackpad Geyser-1"
	},
	[GEYSER1_17inch] = {
		.data_len   = 81,
		.n_xsensors = 26,
		.n_ysensors = 16,
		.prot       = FG_TRACKPAD_TYPE_GEYSER1,
		.name       = "Apple Trackpad Geyser-1-17"
	},
	[GEYSER2] = {
		.data_len   = 64,
		.n_xsensors = 15,
		.n_ysensors = 9,
		.prot       = FG_TRACKPAD_TYPE_GEYSER2,
		.name       = "Apple Trackpad Geyser-2"
	},
	[GEYSER3] = {
		.data_len   = 64,
		.n_xsensors = 20,
		.n_ysensors = 10,
		.prot       = FG_TRACKPAD_TYPE_GEYSER3,
		.name       = "Apple Trackpad Geyser-3"
	},
	[GEYSER4] = {
		.data_len   = 64,
		.n_xsensors = 20,
		.n_ysensors = 10,
		.prot       = FG_TRACKPAD_TYPE_GEYSER4,
		.name       = "Apple Trackpad Geyser-4"
	}
};


static const STRUCT_USB_HOST_ID fg_models[] = {
	/* PowerBooks Feb 2005, iBooks G4 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x020e, FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x020f, FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0210, FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x030a, FOUNTAIN) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x030b, GEYSER1 ) },

	/* PowerBooks Oct 2005 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0214, GEYSER2 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0215, GEYSER2 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0216, GEYSER2 ) },

	/* Core Duo MacBook & MacBook Pro */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0217, GEYSER3 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0218, GEYSER3 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x0219, GEYSER3 ) },

	/* Core2 Duo MacBook & MacBook Pro */
	{ USB_VPI(USB_VENDOR_APPLE, 0x021a, GEYSER4 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x021b, GEYSER4 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x021c, GEYSER4 ) },

	/* Core2 Duo MacBook3,1 */
	{ USB_VPI(USB_VENDOR_APPLE, 0x0229, GEYSER4 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x022a, GEYSER4 ) },
	{ USB_VPI(USB_VENDOR_APPLE, 0x022b, GEYSER4 ) },

	/* 17 inch PowerBook */
	{ USB_VPI(USB_VENDOR_APPLE, 0x020d, GEYSER1_17inch) },
};

typedef struct pad_data_x { int16_t x[FGP_MAX_XSENSORS]; } pad_data_x;
typedef struct pad_data_y { int16_t y[FGP_MAX_YSENSORS]; } pad_data_y;

#define FGP_FIFO_BUF_SIZE     8  /* bytes */
#define FGP_FIFO_QUEUE_MAXLEN 50 /* units */

enum usb_xfer_mode {
	FGP_INTR_DT,
	FGP_RESET,
	FGP_N_TRANSFER,
};

enum pad_delta_mode {
	FGP_DELTA_INIT,
	FGP_DELTA_NONE,
	FGP_DELTA_MOVE,
	FGP_DELTA_SCROLL,
};

struct fgp_softc; /* forward declaration */

struct fgp_softc {
	device_t            sc_dev;
	struct usb_device  *sc_itp_device;
	struct mtx          sc_mutex;    /* for synchronization */
	struct usb_fifo_sc  sc_odv_fifo; /* dev output FIFO */

#define	FG_MODE_LENGTH 8
	char                sc_itp_mode_bytes[FG_MODE_LENGTH]; /* device mode */

	const struct fg_pad_params *sc_pad_params; /* device configuration */

	mousehw_t           sc_odv_mouse_hw;
	mousemode_t         sc_odv_mouse_mode;
	mousestatus_t       sc_odv_mouse_status;
	struct usb_xfer    *sc_itp_xfer[FGP_N_TRANSFER];
	int                 sc_odv_pollrate;
	uint8_t             sc_odv_fflags;
	uint8_t	            sc_pad_idlecount;
#define	FGP_STATE_ODV_OPENED	   0x01
#define	FGP_STATE_OEV_OPENED	   0x02
#define	FGP_STATE_ITP_READING      0x04
	uint8_t             sc_fgp_state;

	/* Regarding the data transferred from t-pad in USB INTR packets. */
	int8_t  sc_pad_data[FGP_SENSOR_DATA_BUF_MAX] __aligned(4);

	/* Core processing variables */
	pad_data_x sc_pad_base_x;
	pad_data_y sc_pad_base_y;
	fg_pspan sc_pspans_x;
	fg_pspan sc_pspans_y;
	uint8_t  sc_pad_touch_cycles;
	uint8_t  sc_pad_no_touch_cycles;
	uint8_t  sc_pad_fingers;
	enum pad_delta_mode sc_pad_delta_mode; 
	uint8_t  sc_pad_phys_button;    /* Physical button state */
	uint8_t  sc_pad_tap_button;    /* The button pressed by a touchpad tap */
	uint32_t sc_pad_loc_x;
	uint32_t sc_pad_loc_y;
	int16_t  sc_pad_delta_x;
	int16_t  sc_pad_delta_y;
#ifdef EVDEV_SUPPORT
	struct evdev_dev *sc_evdev;
#endif
};

/*
 * The last byte of the fountain-geyser sensor data contains status bits; the
 * following values define the meanings of these bits.
 * (only Geyser 3/4)
 */
enum itp_geyser34_status_bits {
	FGP_STATUS_BUTTON      = (uint8_t)0x01, /* The button was pressed */
	FGP_STATUS_BASE_UPDATE = (uint8_t)0x04, /* Data from an untouched pad */
};

typedef enum itp_device_mode {
	RAW_SENSOR_MODE = (uint8_t)0x01,
	HID_MODE        = (uint8_t)0x08
} itp_device_mode;

/*
 * function prototypes
 */
static usb_fifo_cmd_t   fgp_odv_start_read;
static usb_fifo_cmd_t   fgp_odv_stop_read;
static usb_fifo_open_t  fgp_odv_open;
static usb_fifo_close_t fgp_odv_close;
static usb_fifo_ioctl_t fgp_odv_ioctl;

static struct usb_fifo_methods fgp_odv_methods = {
	.f_open       = &fgp_odv_open,
	.f_close      = &fgp_odv_close,
	.f_ioctl      = &fgp_odv_ioctl,
	.f_start_read = &fgp_odv_start_read,
	.f_stop_read  = &fgp_odv_stop_read,
	.basename[0]  = FGP_DRIVER_NAME,
};

#ifdef EVDEV_SUPPORT
static evdev_open_t  fgp_oev_open;
static evdev_close_t fgp_oev_close;
static const struct evdev_methods fgp_oev_methods = {
	.ev_open  = &fgp_oev_open,
	.ev_close = &fgp_oev_close,
};
#endif

/* forward declarations */
struct fg_pad_params; 
enum fg_frame_state;

/* device initialization and shutdown */
static usb_error_t fgp_itp_set_device_mode(struct fgp_softc *, itp_device_mode);
static void        fgp_itp_reset_cb(struct usb_xfer *itp_xfer, usb_error_t itp_error);

/* sensor interpretation */
static void        fg_intepret_sensor_data(struct fgp_softc *sc);
static void	       fg_extract_sensor_data(const int8_t *sensor_data, const struct fg_pad_params *pad_params, 
					pad_data_x *raw_data_x, pad_data_y *raw_data_y, uint8_t *pad_status);

static void	       fg_get_pressures(int16_t *p, const int16_t *cur, const int16_t *base, int8_t n);

static void	       fg_detect_fingers(int16_t *, uint8_t, fg_pspan *);
static void		   fg_reset_delta(struct fgp_softc *sc);

/* evdev interface */
#ifdef EVDEV_SUPPORT
static int         fgp_oev_open(struct evdev_dev *evdev);
static int         fgp_oev_close(struct evdev_dev *evdev);
#endif

/* dev interface */
static int         fgp_odv_open(struct usb_fifo *odv_fifo, int odv_flags);
static void        fgp_odv_close(struct usb_fifo *odv_fifo, int odv_flags);
static void	       fgp_odv_reset_buf(struct fgp_softc *);
static int         fgp_odv_ioctl(struct usb_fifo *odv_fifo, u_long cmd, void *addr, int odv_flags);
static void        fgp_odv_start_read(struct usb_fifo *fifo_usb);
static void        fgp_odv_stop_read(struct usb_fifo *fifo_usb);

/* output (dev + evdev) */
static void	       fgp_oxv_add_to_queue(struct fgp_softc *, int16_t, int16_t, int16_t, int16_t, uint32_t);

/* Device methods. */
static device_probe_t  fgp_itp_probe;
static device_attach_t fgp_itp_attach;
static device_detach_t fgp_itp_detach;
static usb_callback_t  fgp_itp_intr_cb;
static usb_callback_t  fgp_itp_reset_cb;

static const struct usb_config fgp_itp_xfer_config[FGP_N_TRANSFER] = {
	[FGP_INTR_DT] = {
		.type      = UE_INTERRUPT,
		.endpoint  = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.pipe_bof      = 1, /* block pipe on failure */
			.short_xfer_ok = 1,
		},
		.bufsize   = FGP_SENSOR_DATA_BUF_MAX,
		.callback  = &fgp_itp_intr_cb,
	},
	[FGP_RESET] = {
		.type      = UE_CONTROL,
		.endpoint  = 0, /* Control pipe */
		.direction = UE_DIR_ANY,
		.bufsize   = sizeof(struct usb_device_request) + FG_MODE_LENGTH,
		.callback  = &fgp_itp_reset_cb,
		.interval  = 0,  /* no pre-delay */
	},
};

static usb_error_t
fgp_itp_set_device_mode(struct fgp_softc *sc, itp_device_mode newMode)
{
	uint8_t     mode_value;
	usb_error_t err;

	if ((newMode != RAW_SENSOR_MODE) && (newMode != HID_MODE)) {
		return (USB_ERR_INVAL);
	}

	if (newMode == RAW_SENSOR_MODE)
		mode_value = (uint8_t)0x04;
	else
		mode_value = newMode;

	err = usbd_req_get_report(sc->sc_itp_device, NULL /* mutex */,
	    sc->sc_itp_mode_bytes, sizeof(sc->sc_itp_mode_bytes), 
		0 /* interface idx */,
	    0x03 /* type */, 
		0x00 /* id */);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		DPRINTF("Failed to read device mode (%d)\n", err);
		return (err);
	}

	/* already in requested mode? */
	if (sc->sc_itp_mode_bytes[0] == mode_value) {
		return (err);
	}

	/*
	 * XXX Need to wait at least 250ms for hardware to get
	 * ready. The device mode handling appears to be handled
	 * asynchronously and we should not issue these commands too
	 * quickly.
	 */
	pause("WHW", hz / 4);

	sc->sc_itp_mode_bytes[0] = mode_value;
	err = usbd_req_set_report(sc->sc_itp_device, NULL /* mutex */,
	    sc->sc_itp_mode_bytes, sizeof(sc->sc_itp_mode_bytes), 
		0 /* interface idx */,
	    0x03 /* type */, 
		0x00 /* id */);
	return (err);
}

static void
fgp_itp_reset_cb(struct usb_xfer *itp_xfer, usb_error_t itp_error)
{
	usb_device_request_t   req;
	struct usb_page_cache *pc;
	struct fgp_softc      *sc = usbd_xfer_softc(itp_xfer);

	uint8_t mode_value = 0x04;

	switch (USB_GET_STATE(itp_xfer)) {
	case USB_ST_SETUP:
		sc->sc_itp_mode_bytes[0] = mode_value;
		req.bmRequestType = UT_WRITE_CLASS_INTERFACE;
		req.bRequest = UR_SET_REPORT;
		USETW2(req.wValue,
		    (uint8_t)0x03 /* type */, (uint8_t)0x00 /* id */);
		USETW(req.wIndex, 0);
		USETW(req.wLength, FG_MODE_LENGTH);

		pc = usbd_xfer_get_frame(itp_xfer, 0);
		usbd_copy_in(pc, 0, &req, sizeof(req));
		pc = usbd_xfer_get_frame(itp_xfer, 1);
		usbd_copy_in(pc, 0, sc->sc_itp_mode_bytes, FG_MODE_LENGTH);

		usbd_xfer_set_frame_len(itp_xfer, 0, sizeof(req));
		usbd_xfer_set_frame_len(itp_xfer, 1, FG_MODE_LENGTH);
		usbd_xfer_set_frames(itp_xfer, 2);
		usbd_transfer_submit(itp_xfer);
		break;

	case USB_ST_TRANSFERRED:
	default:
		break;
	}
}

/* trackpad data frame state */
enum fg_frame_state {
	FG_TPSTATE_UNKNOWN,	/* State unknown (initial)  */
	FG_TPSTATE_ERRAXIS, /* One Axis without reading */
	FG_TPSTATE_IDLE,    /* Trackpad is idle         */
	FG_TPSTATE_BASE,    /* Base value data frame    */
	FG_TPSTATE_READING, /* Touch reading data frame */
};

static void
fg_reset_delta(struct fgp_softc *sc) {
	sc->sc_pad_loc_x   = sc->sc_pspans_x.location;
	sc->sc_pad_loc_y   = sc->sc_pspans_y.location;
	sc->sc_pad_delta_x = 0;
	sc->sc_pad_delta_y = 0;
}

static void
fg_intepret_sensor_data(struct fgp_softc *sc){

	uint16_t pad_fingers  = max(sc->sc_pspans_x.fingers, sc->sc_pspans_y.fingers);
	uint16_t pad_pressure = min(sc->sc_pspans_x.pressure_max, sc->sc_pspans_y.pressure_max);

	/* 
	 *
	 * Process buttons 
	 *
	*/
	if ((! sc->sc_pad_phys_button) && (sc->sc_pad_touch_cycles == fgp_sysctl_params.touch_settlement_cycles)) {
		DPRINTFN(FGP_LLEVEL_DEBUG, "[core] First Mature... ");
		if (pad_pressure > fgp_sysctl_params.tap_pressure_threshold) {
			DPRINTN(FGP_LLEVEL_DEBUG, "TAP detected (%d)\n", pad_pressure);
			switch (pad_fingers) {
			case 1: /* Left Button */
				sc->sc_pad_tap_button           = MOUSE_BUTTON1DOWN;
				break;
			case 2: /* Right Button */
				sc->sc_pad_tap_button           = MOUSE_BUTTON3DOWN;
				break;
			case 3: /* Middle Buton */
				sc->sc_pad_tap_button           = MOUSE_BUTTON2DOWN;
				break;
			default:
				break;
			}
		} else {
			DPRINTN(FGP_LLEVEL_DEBUG, "no tap (%d)\n", pad_pressure);
		}
	} 
	/* After touch, TAP button goes up */
	if (pad_fingers == 0) {
		sc->sc_pad_tap_button = 0;
	}

	/* 
	 *
	 * Process moves 
	 *
	*/
	int16_t mickeys_x = 0;
	int16_t mickeys_y = 0;
	int16_t scroll_h  = 0;
	int16_t scroll_v  = 0;

	/* Reset delta mode */
	if (sc->sc_pad_touch_cycles < fgp_sysctl_params.touch_settlement_cycles) {
		sc->sc_pad_delta_mode  = FGP_DELTA_INIT;
	}

	if (sc->sc_pad_delta_mode == FGP_DELTA_INIT) {
		if (sc->sc_pad_touch_cycles >= fgp_sysctl_params.touch_settlement_cycles) {
			fg_reset_delta(sc);
			sc->sc_pad_fingers = pad_fingers;
			switch (pad_fingers) {
			case 1:
				sc->sc_pad_delta_mode = FGP_DELTA_MOVE;
				break;
			case 2:
				sc->sc_pad_delta_mode = FGP_DELTA_SCROLL;
				break;
			default:
				sc->sc_pad_delta_mode = FGP_DELTA_NONE;
			}
			DPRINTFN(FGP_LLEVEL_DEBUG, "[delta] initialize\n");
		} 
	} else if (pad_fingers != sc->sc_pad_fingers) {
		/* Reset delta data, when gesture (finger count) changes */
		fg_reset_delta(sc);
		sc->sc_pad_fingers = pad_fingers;
	} else if (pad_fingers > 0) {
		sc->sc_pad_delta_x += (sc->sc_pspans_x.location - sc->sc_pad_loc_x);
		sc->sc_pad_delta_y += (sc->sc_pspans_y.location - sc->sc_pad_loc_y);

		DPRINTFN(FGP_LLEVEL_DEBUG, "[delta] ploc X: %d, oloc X: %d, ploc Y: %d, oloc Y: %d, delta X: %d, delta Y: %d\n", 
			sc->sc_pspans_x.location, sc->sc_pad_loc_x, sc->sc_pspans_y.location, sc->sc_pad_loc_y, sc->sc_pad_delta_x, sc->sc_pad_delta_y);

		sc->sc_pad_loc_x   = sc->sc_pspans_x.location;
		sc->sc_pad_loc_y   = sc->sc_pspans_y.location;

		switch (sc->sc_pad_delta_mode) {
		case FGP_DELTA_MOVE: /* Move */
			DPRINTFN(FGP_LLEVEL_DEBUG, "[delta] Move\n");
			mickeys_x = sc->sc_pad_delta_x / (int)fgp_sysctl_params.mickeys_scale_factor;
			mickeys_y = sc->sc_pad_delta_y / (int)fgp_sysctl_params.mickeys_scale_factor;
			sc->sc_pad_delta_x %= (int)fgp_sysctl_params.mickeys_scale_factor;
			sc->sc_pad_delta_y %= (int)fgp_sysctl_params.mickeys_scale_factor;

			sc->sc_odv_mouse_status.dx += mickeys_x;
			sc->sc_odv_mouse_status.dy += mickeys_y;
			if ((mickeys_x != 0) || (mickeys_y != 0)) {
				sc->sc_odv_mouse_status.flags |= MOUSE_POSCHANGED;
			}
			break;
		case FGP_DELTA_SCROLL: /* Scroll */
			DPRINTFN(FGP_LLEVEL_DEBUG, "[delta] Scroll\n");
			scroll_h = sc->sc_pad_delta_x / (int)fgp_sysctl_params.scroll_scale_factor;
			scroll_v = sc->sc_pad_delta_y / (int)fgp_sysctl_params.scroll_scale_factor;
			sc->sc_pad_delta_x %= (int)fgp_sysctl_params.scroll_scale_factor;
			sc->sc_pad_delta_y %= (int)fgp_sysctl_params.scroll_scale_factor;

			sc->sc_odv_mouse_status.dz += scroll_v;
			if ((scroll_h != 0) || (scroll_v != 0)) {
				sc->sc_odv_mouse_status.flags |= MOUSE_POSCHANGED;
			}
			break;
		case FGP_DELTA_NONE: /* more than 2 fingers */
			break;
		default:
			break;
		}
	}

	/* Evaluate button state */
	sc->sc_odv_mouse_status.button = (sc->sc_pad_tap_button != 0) ? sc->sc_pad_tap_button : sc->sc_pad_phys_button;
	/* Flag changed button state */
	DPRINTFN(FGP_LLEVEL_DEBUG, "[core] Button: %d, obutton: %d\n", sc->sc_odv_mouse_status.button, sc->sc_odv_mouse_status.obutton);
	if (sc->sc_odv_mouse_status.button != sc->sc_odv_mouse_status.obutton) {
		sc->sc_odv_mouse_status.flags |= MOUSE_STDBUTTONSCHANGED;
	}
	if (sc->sc_odv_mouse_status.flags & (MOUSE_POSCHANGED | MOUSE_STDBUTTONSCHANGED)) {
		fgp_oxv_add_to_queue(sc, mickeys_x, mickeys_y, scroll_v, scroll_h, sc->sc_odv_mouse_status.button);
	}
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
 *   num
 *       The number of elements in the array 'arr'.
 *   axis
 *       Axis of data to fetch
 *   arr
 *       The array to be initialized with the readings.
 *   prot
 *       The protocol to use to interpret the data
 */
static void
fg_extract_sensor_data(const int8_t *sensor_data, const struct fg_pad_params *pad_params, 
	pad_data_x *raw_data_x, pad_data_y *raw_data_y, uint8_t *pad_status)
{
	uint8_t i;
	uint8_t di;   /* index into sensor data */

	switch (pad_params->prot) {
	case FG_TRACKPAD_TYPE_GEYSER1:
		/*
		 * For Geyser 1, the sensors are laid out in pairs
		 * every 5 bytes.
		 */
		 /* X */
		for (i = 0, di = 2; i < 8; di += 5, i++) {
			raw_data_x->x[i  ] = (int16_t)sensor_data[di  ];
			raw_data_x->x[i+8] = (int16_t)sensor_data[di+2];
			if (pad_params->n_xsensors > 16) {
				raw_data_x->x[i+16] = (int16_t)sensor_data[di+40];
			}
		}
		/* Y */
		for (i = 0, di = 1; i < 8; di += 5, i++) {
			raw_data_y->y[i  ] = (int16_t)sensor_data[di  ];
			raw_data_y->y[i+8] = (int16_t)sensor_data[di+2];
		}
		break;
	case FG_TRACKPAD_TYPE_GEYSER2:
		 /* X */
		for (i = 0, di = 19; i < pad_params->n_xsensors; /* empty */ ) {
			raw_data_x->x[i++] = (int16_t)sensor_data[di++];
			raw_data_x->x[i++] = (int16_t)sensor_data[di++];
			di++;
		}
		 /* Y */
		for (i = 0, di =  1; i < pad_params->n_ysensors; /* empty */ ) {
			raw_data_y->y[i++] = (int16_t)sensor_data[di++];
			raw_data_y->y[i++] = (int16_t)sensor_data[di++];
			di++;
		}
		break;
	case FG_TRACKPAD_TYPE_GEYSER3:
	case FG_TRACKPAD_TYPE_GEYSER4:
		 /* X */
		for (i = 0, di = 20; i < pad_params->n_xsensors; /* empty */ ) {
			raw_data_x->x[i++] = (int16_t)sensor_data[di++];
			raw_data_x->x[i++] = (int16_t)sensor_data[di++];
			di++;
		}
		 /* Y */
		for (i = 0, di =  2; i < pad_params->n_ysensors; /* empty */ ) {
			raw_data_y->y[i++] = (int16_t)sensor_data[di++];
			raw_data_y->y[i++] = (int16_t)sensor_data[di++];
			di++;
		}
		break;
	default:
		break;
	}
	*pad_status = sensor_data[pad_params->data_len - 1];
}

static void
fg_get_pressures(int16_t *p, const int16_t *cur, const int16_t *base, int8_t n)
{
	int8_t i;
	DPRINTN(FGP_LLEVEL_DEBUG, "Pressures (%d): ", n );
	for (i = 0; i < n; i++) {
		p[i] = cur[i] - base[i];

		/*
		 * Shave off pressures below the noise-pressure
		 * threshold; this will reduce the contribution from
		 * lower pressure readings.
		 */
		if (p[i] <= FGP_SENSOR_NOISE_THRESHOLD) {
			p[i] = 0; /* filter away noise */
		} else {
			p[i] -= FGP_SENSOR_NOISE_THRESHOLD;
		}
		DPRINTN(FGP_LLEVEL_DEBUG, "%d ", p[i]);
	}
	DPRINTN(FGP_LLEVEL_DEBUG, "\n" );
}

static void
fg_detect_fingers(int16_t *p, uint8_t num_sensors, fg_pspan  *spans)
{
	uint8_t  i;
	int16_t  maxp; /* max pressure seen within a span */
	uint32_t wcum; /* cumulated pressure working      */
	uint32_t wcog; /* cumulated center of gravity working */
	uint8_t  wwdt; /* width working */
	bool     finger_complete; 

	enum fg_pspan_state {
		FGP_PSPAN_INACTIVE,
		FGP_PSPAN_INCREASING,
		FGP_PSPAN_DECREASING,
	} pspan_state; /* state of the pressure span */

	spans->pressure_max	= 0;
	spans->fingers  	= 0;
	spans->location 	= 0;

	/*
	 * The following is a simple state machine to track fingers
	*/
	maxp = 0;
	wwdt = 0;
	wcum = 0;
	wcog = 0;
	finger_complete = false;
	for (i = 0; i < num_sensors; i++) {
		if (p[i] == 0) {
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
		} else {
			switch (pspan_state) {
			case FGP_PSPAN_INACTIVE:
				pspan_state = FGP_PSPAN_INCREASING;
				maxp  = p[i];
				break;

			case FGP_PSPAN_INCREASING:
				if (p[i] > maxp) {
					maxp = p[i];
				}
				else if (p[i] <= (maxp >> 1))
					pspan_state = FGP_PSPAN_DECREASING;
				break;

			case FGP_PSPAN_DECREASING:
				if (p[i] > p[i - 1]) {
					/*
					 * This is the beginning of
					 * another finger
					 */
					finger_complete = true;
					continue;
				}
				break;
			}
			if ((i+1)==num_sensors){
				/* Last sensor.
				 * Update the working span with this reading
				 * and finish the finger. 
				*/
				wwdt++;
				wcum += p[i];
				wcog += p[i] * (i + 1);
				finger_complete = true;
			}
		}
		if (finger_complete) {
			if (		
				(wwdt <= FGP_PSPAN_MAX_WIDTH       ) &&
				(wcum >= FGP_PSPAN_MIN_CUM_PRESSURE)
			   ) 
			{
				DPRINTFN(FGP_LLEVEL_DEBUG, "[core] (%d) New Finger.  wwdt: %d, wcum: %d\n", 
					num_sensors, wwdt, wcum );
				spans->fingers++;
				spans->location += wcog * FGP_SPAN_FACTOR / wcum;
				spans->pressure_max = max(wcum, spans->pressure_max);
			}
			maxp = 0;
			wwdt = 0;
			wcum = 0;
			wcog = 0;
			finger_complete = false;
		}
		/* Accumulate finger data */
		wwdt++;
		wcum += p[i];
		wcog += p[i] * (i + 1);
	}

	if (spans->fingers)
		DPRINTFN(FGP_LLEVEL_DEBUG, "[core] (%d) fingers: %d, location: %d, pressure: %d\n", 
			num_sensors, spans->fingers, spans->location, spans->pressure);
}

static void
fgp_oxv_add_to_queue(struct fgp_softc *sc, int16_t dx, int16_t dy, int16_t dv, int16_t dh,
    uint32_t buttons_in)
{
	uint32_t buttons_out;
	uint8_t  buf[8];

#ifdef EVDEV_SUPPORT
	DPRINTFN(FGP_LLEVEL_INFO, "[core] dx=%d, dy=%d, dv=%d, dh=%d, buttons=%x, sc_state=%x, evdev_rcpt_mask=%x\n",
	    dx, dy, dv, dh, buttons_in, sc->sc_fgp_state, evdev_rcpt_mask);

	/* Event device opened and requested? -> Use event channel */
	if ((sc->sc_fgp_state & FGP_STATE_OEV_OPENED) && (evdev_rcpt_mask & EVDEV_RCPT_HW_MOUSE)) {

		DPRINTFN(FGP_LLEVEL_INFO, "[core] evdev_push...\n");

		evdev_push_key(sc->sc_evdev, BTN_LEFT,   (buttons_in & MOUSE_BUTTON1DOWN));
		evdev_push_key(sc->sc_evdev, BTN_MIDDLE, (buttons_in & MOUSE_BUTTON2DOWN));
		evdev_push_key(sc->sc_evdev, BTN_RIGHT,  (buttons_in & MOUSE_BUTTON3DOWN));
 		if (dx) evdev_push_rel(sc->sc_evdev, REL_X,      dx);
		if (dy) evdev_push_rel(sc->sc_evdev, REL_Y,      dy);
	//	if (dv) evdev_push_rel(sc->sc_evdev, REL_WHEEL_HI_RES,  dv);
	//	if (dh) evdev_push_rel(sc->sc_evdev, REL_HWHEEL_HI_RES, dh);
		if (dv) evdev_push_rel(sc->sc_evdev, REL_WHEEL,  dv);
		if (dh) evdev_push_rel(sc->sc_evdev, REL_HWHEEL, dh);
 
		evdev_sync(sc->sc_evdev);
		return;
	}
#endif
	/* Without available event channel, use usb dev output */

	dx = imin( dx,  254); dx = imax(dx, -256);
	dy = imin(-dy,  254); dy = imax(dy, -256);
	dv = imin(-dv,  126); dv = imax(dv, -128);

	buttons_out = MOUSE_MSC_BUTTONS;
	if (buttons_in & MOUSE_BUTTON1DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON1UP;
	else if (buttons_in & MOUSE_BUTTON2DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON2UP;
	else if (buttons_in & MOUSE_BUTTON3DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON3UP;

	DPRINTFN(FGP_LLEVEL_INFO, "dx=%d, dy=%d, buttons=%x\n",
	    dx, dy, buttons_out);

	/* Encode the mouse data in standard format; refer to mouse(4) */
	buf[0] = sc->sc_odv_mouse_mode.syncmask[1];
	buf[0] |= buttons_out;
	buf[1] = dx >> 1;
	buf[2] = dy >> 1;
	buf[3] = dx - (dx >> 1);
	buf[4] = dy - (dy >> 1);
	/* Encode extra bytes for level 1 */
	if (sc->sc_odv_mouse_mode.level == 1) {
		buf[5] = dv >> 1;
		buf[6] = dv - (dv >> 1);
		buf[7] = (((~buttons_in) >> 3) & MOUSE_SYS_EXTBUTTONS);
	}

	usb_fifo_put_data_linear(sc->sc_odv_fifo.fp[USB_FIFO_RX], buf,
	    sc->sc_odv_mouse_mode.packetsize, 1);
}

static int
fgp_itp_probe(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST) {
		return (ENXIO);
	}
	if (uaa->info.bInterfaceClass != UICLASS_HID) {
		return (ENXIO);
	}
	if ((usbd_lookup_id_by_uaa(fg_models, sizeof(fg_models), uaa)) == 0) {
		return ((uaa->info.bInterfaceProtocol == UIPROTO_MOUSE) ?
			BUS_PROBE_DEFAULT : ENXIO);
	}
	return (ENXIO);
}

static int
fgp_itp_attach(device_t dev)
{
	struct fgp_softc		 *sc  = device_get_softc(dev);
	struct usb_attach_arg	 *uaa = device_get_ivars(dev);
	usb_error_t				 err;
	void					 *descriptor_ptr = NULL;
	uint16_t descriptor_len;
	int hid_data_len;
	unsigned long di;

	DPRINTFN(FGP_LLEVEL_DEBUG, "sc=%p\n", sc);

	sc->sc_dev        = dev;
	sc->sc_itp_device = uaa->device;
	di = USB_GET_DRIVER_INFO(uaa);
	sc->sc_pad_params = &fg_dev_params[di];

	/* Get HID descriptor */
	if (usbd_req_get_hid_desc(uaa->device, NULL, &descriptor_ptr,
	    &descriptor_len, M_TEMP, uaa->info.bIfaceIndex) !=
	    USB_ERR_NORMAL_COMPLETION) {
			return (ENXIO);
		}

	/* Get HID report descriptor length */
	hid_data_len= hid_report_size_max(descriptor_ptr,
	    descriptor_len, hid_input, NULL);
	free(descriptor_ptr, M_TEMP);

	if (hid_data_len != sc->sc_pad_params->data_len) {
		DPRINTF( "Trackpad datalengths: param=%d, actual=%d\n",
		  sc->sc_pad_params->data_len, hid_data_len);
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
	if (di == FOUNTAIN)
		DPRINTF("device mode switch skipped: Fountain device\n");
	else if ((err = fgp_itp_set_device_mode(sc, RAW_SENSOR_MODE)) != 0) {
		DPRINTF("failed to set mode to 'RAW_SENSOR' (%d)\n", err);
		return (ENXIO);
	}

	mtx_init(&sc->sc_mutex, "fgp lock", NULL, MTX_DEF | MTX_RECURSE);

	err = usbd_transfer_setup(uaa->device,
	    &uaa->info.bIfaceIndex, sc->sc_itp_xfer, fgp_itp_xfer_config,
	    FGP_N_TRANSFER, sc, &sc->sc_mutex);
	if (err) {
		DPRINTF("error=%s\n", usbd_errstr(err));
		goto detach;
	}

	if (usb_fifo_attach(sc->sc_itp_device, sc, &sc->sc_mutex,
	    &fgp_odv_methods, &sc->sc_odv_fifo,
	    device_get_unit(dev), -1, uaa->info.bIfaceIndex,
	    UID_ROOT, GID_OPERATOR, 0644)) {
		goto detach;
	}

	device_set_usb_desc(dev);

	sc->sc_odv_mouse_hw.buttons       = 3;
	sc->sc_odv_mouse_hw.iftype        = MOUSE_IF_USB;
	sc->sc_odv_mouse_hw.type          = MOUSE_PAD;
	sc->sc_odv_mouse_hw.model         = MOUSE_MODEL_GENERIC;
	sc->sc_odv_mouse_hw.hwid          = 0;
	sc->sc_odv_mouse_mode.protocol    = MOUSE_PROTO_MSC;
	sc->sc_odv_mouse_mode.rate        = -1;
	sc->sc_odv_mouse_mode.resolution  = MOUSE_RES_UNKNOWN;
	sc->sc_odv_mouse_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
	sc->sc_odv_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
	sc->sc_odv_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
	sc->sc_odv_mouse_mode.accelfactor = 0;
	sc->sc_odv_mouse_mode.level       = 0;

	sc->sc_fgp_state                  = 0;

	sc->sc_pad_touch_cycles           = 0;
	sc->sc_pad_no_touch_cycles		  = 0;
	sc->sc_pad_fingers				  = 0;
	sc->sc_pad_delta_mode			  = FGP_DELTA_NONE;
	sc->sc_pad_phys_button			  = 0;
	sc->sc_pad_tap_button			  = 0;
	sc->sc_pad_loc_x				  = 0;
	sc->sc_pad_loc_y				  = 0;
	sc->sc_pad_delta_x				  = 0;
	sc->sc_pad_delta_y				  = 0;

#ifdef EVDEV_SUPPORT
	sc->sc_evdev = evdev_alloc();
	evdev_set_name(sc->sc_evdev, sc->sc_pad_params->name);
	evdev_set_phys(sc->sc_evdev, device_get_nameunit(dev));
	evdev_set_id(sc->sc_evdev, BUS_USB, uaa->info.idVendor,
	    uaa->info.idProduct, 0);
	evdev_set_serial(sc->sc_evdev, usb_get_serial(uaa->device));
	evdev_set_methods(sc->sc_evdev, sc, &fgp_oev_methods);
//	evdev_support_prop(sc->sc_evdev, INPUT_PROP_POINTER);
//	evdev_support_prop(sc->sc_evdev, INPUT_PROP_BUTTONPAD);
	evdev_support_event(sc->sc_evdev, EV_SYN);
	evdev_support_event(sc->sc_evdev, EV_REL);
	evdev_support_event(sc->sc_evdev, EV_KEY);
	evdev_support_rel(sc->sc_evdev, REL_X     );
	evdev_support_rel(sc->sc_evdev, REL_Y     );
//	evdev_support_rel(sc->sc_evdev, REL_WHEEL_HI_RES ); /* V scroll */
//	evdev_support_rel(sc->sc_evdev, REL_HWHEEL_HI_RES); /* H scroll */
//	evdev_support_rel(sc->sc_evdev, REL_WHEEL ); /* V scroll */
//	evdev_support_rel(sc->sc_evdev, REL_HWHEEL); /* H scroll */
	evdev_support_key(sc->sc_evdev, BTN_LEFT  );
//	evdev_support_key(sc->sc_evdev, BTN_MIDDLE);
//	evdev_support_key(sc->sc_evdev, BTN_RIGHT );

	err = evdev_register_mtx(sc->sc_evdev, &sc->sc_mutex);
	if (err)
		goto detach;
#endif

	return (0);

detach:
	fgp_itp_detach(dev);
	return (ENOMEM);
}

static int
fgp_itp_detach(device_t dev)
{
	struct fgp_softc *sc;

	sc = device_get_softc(dev);

	/* before leaving, reset trackpad to HID mode */
	fgp_itp_set_device_mode(sc, HID_MODE);

	usb_fifo_detach(&sc->sc_odv_fifo);

#ifdef EVDEV_SUPPORT
	evdev_free(sc->sc_evdev);
#endif

	usbd_transfer_unsetup(sc->sc_itp_xfer, FGP_N_TRANSFER);
	mtx_destroy(&sc->sc_mutex);
	return (0);
}

static void
fgp_itp_intr_cb(struct usb_xfer *xfer, usb_error_t error)
{
	struct fgp_softc      *sc = usbd_xfer_softc(xfer);
	struct usb_page_cache *pc;
	int    frame_len;

	uint8_t    pad_status_bits;
	uint8_t	   pad_xy_cnt;
	pad_data_x pad_raw_data_x;
	pad_data_y pad_raw_data_y;
	pad_data_x pad_pressures_x;
	pad_data_y pad_pressures_y;
	enum fg_frame_state pad_fstate = FG_TPSTATE_UNKNOWN;

	usbd_xfer_status(xfer, &frame_len, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, sc->sc_pad_data, frame_len);

		/* Ignore damaged data packets */
		if (frame_len != sc->sc_pad_params->data_len) {
			DPRINTN(FGP_LLEVEL_INFO, "Trackpad datalengths: param=%d, actual=%d\n",
				sc->sc_pad_params->data_len, frame_len);
		} else {
			fgp_sysctl_rangecheck(&fgp_sysctl_params);

			sc->sc_odv_mouse_status.flags &= ~(MOUSE_STDBUTTONSCHANGED | MOUSE_POSCHANGED);
			sc->sc_odv_mouse_status.obutton = sc->sc_odv_mouse_status.button;

			fg_extract_sensor_data(sc->sc_pad_data, sc->sc_pad_params, &pad_raw_data_x, &pad_raw_data_y, &pad_status_bits);

			/*
			* If this is a base update (from an untouched
			* pad), we should set the base values for the sensor
			* data; deltas with respect to these base values can
			* be used as pressure readings subsequently.
			*/
			if (((sc->sc_pad_params->prot == FG_TRACKPAD_TYPE_GEYSER3) ||
				(sc->sc_pad_params->prot == FG_TRACKPAD_TYPE_GEYSER4))  &&
				(pad_status_bits & FGP_STATUS_BASE_UPDATE)) {
					pad_fstate = FG_TPSTATE_BASE;
			} else {
				/* Read physical button state */
				sc->sc_pad_phys_button = ((pad_status_bits & FGP_STATUS_BUTTON) > 0) ? MOUSE_BUTTON1DOWN : 0;

				/* Get pressure readings and detect fingers on both axes. */
				fg_get_pressures((int16_t *)&pad_pressures_x.x, (int16_t *)pad_raw_data_x.x, sc->sc_pad_base_x.x,
					sc->sc_pad_params->n_xsensors);
				fg_detect_fingers((int16_t *)&pad_pressures_x.x, sc->sc_pad_params->n_xsensors, 
					&sc->sc_pspans_x);
				
				fg_get_pressures((int16_t *)&pad_pressures_y.y, (int16_t *)pad_raw_data_y.y, sc->sc_pad_base_y.y,
					sc->sc_pad_params->n_ysensors);
				fg_detect_fingers((int16_t *)&pad_pressures_y.y, sc->sc_pad_params->n_ysensors, 
					&sc->sc_pspans_y);

				/*
				 * Evaluate frame data quality
				*/
				pad_xy_cnt  = (sc->sc_pspans_x.fingers > 0) ? 1 : 0;
				pad_xy_cnt += (sc->sc_pspans_y.fingers > 0) ? 1 : 0;
				switch (pad_xy_cnt) {
				case 0:
					pad_fstate = FG_TPSTATE_IDLE;	 /* trackpad is (untouched) */
					break;
				case 1:
					pad_fstate = FG_TPSTATE_ERRAXIS; /* Data of one Axis missing */
					break;
				case 2:
					pad_fstate = FG_TPSTATE_READING; /* Valid reading */
				}
			} 

			switch (pad_fstate) {
			case FG_TPSTATE_BASE:
				/* Update pressure base data */
				DPRINTFN(FGP_LLEVEL_DEBUG, "[core] Pressure Update Base\n");
				memcpy(&sc->sc_pad_base_x, &pad_raw_data_x, sizeof(sc->sc_pad_base_x));
				memcpy(&sc->sc_pad_base_y, &pad_raw_data_y, sizeof(sc->sc_pad_base_y));
				break;
			case FG_TPSTATE_IDLE:
				/* calculate touch end 
				* When count of idle frames exceeds threshold,
				* touch cycles are reset
				*/
				if (sc->sc_pad_no_touch_cycles < FGP_IDLENESS_THRESHOLD) {
					sc->sc_pad_no_touch_cycles ++;
					if (sc->sc_pad_no_touch_cycles == FGP_IDLENESS_THRESHOLD) {
						sc->sc_pad_touch_cycles = 0;
					}
				}
				fg_intepret_sensor_data(sc);

				/* Trackpad reset ? */
				if (! sc->sc_pad_phys_button) {
					/*
					* The Fountain/Geyser device continues to trigger interrupts
					* at a fast rate even after touchpad activity has
					* stopped. Upon detecting that the device has remained idle
					* beyond a threshold, we reinitialize it to silence the
					* interrupts.
					*/
					sc->sc_pad_idlecount++;
					if (sc->sc_pad_idlecount >= FGP_IDLENESS_THRESHOLD) {
						/*
						* Use the last frame before we go idle for
						* calibration on pads which do not send
						* calibration frames.
						*/
						DPRINTFN(FGP_LLEVEL_INFO, "[core] idle\n");

						if (sc->sc_pad_params->prot < FG_TRACKPAD_TYPE_GEYSER3) {
							memcpy(&sc->sc_pad_base_x, &pad_raw_data_x, sizeof(sc->sc_pad_base_x));
							memcpy(&sc->sc_pad_base_y, &pad_raw_data_y, sizeof(sc->sc_pad_base_y));
						}

						sc->sc_pad_idlecount = 0;
						usbd_transfer_start(sc->sc_itp_xfer[FGP_RESET]);
					}
				}
				break;
			case FG_TPSTATE_READING:
				/* calculate settlement 
				* Range: 0..touch_settlement_cycles+1
				*/
				sc->sc_pad_no_touch_cycles = 0;
				if (sc->sc_pad_touch_cycles <= fgp_sysctl_params.touch_settlement_cycles) {
					sc->sc_pad_touch_cycles++;
				}
				fg_intepret_sensor_data(sc);
				break;
			default:
				break;
			}




		}

	case USB_ST_SETUP:
	tr_setup:
		/* check if we can put more data into the FIFO */
		if (usb_fifo_put_bytes_max(sc->sc_odv_fifo.fp[USB_FIFO_RX]) != 0) {
			usbd_xfer_set_frame_len(xfer, 0,
			    sc->sc_pad_params->data_len);
			usbd_transfer_submit(xfer);
		}
		break;

	default:                        /* Error */
		if (error != USB_ERR_CANCELLED) {
			/* try clear stall first */
			usbd_xfer_set_stall(xfer);
			goto tr_setup;
		}
		break;
	}
}

static void
fgp_itp_start_read(struct fgp_softc *sc)
{
	int rate;
	if ((sc->sc_fgp_state & FGP_STATE_ITP_READING) == 0) {
		/* Check if we should override the default polling interval */
		rate = sc->sc_odv_pollrate;
		/* Range check rate */
		if (rate > 1000) {
			rate = 1000;
		}
		/* Check for set rate */
		if ((rate > 0) && (sc->sc_itp_xfer[FGP_INTR_DT] != NULL)) {
			/* Stop current transfer, if any */
			usbd_transfer_stop(sc->sc_itp_xfer[FGP_INTR_DT]);
			/* Set new interval */
			usbd_xfer_set_interval(sc->sc_itp_xfer[FGP_INTR_DT], 1000 / rate);
			/* Only set pollrate once */
			sc->sc_odv_pollrate = 0;
		}

		sc->sc_pad_touch_cycles = 0;
		usbd_transfer_start(sc->sc_itp_xfer[FGP_INTR_DT]);
		sc->sc_fgp_state |= FGP_STATE_ITP_READING;
	}
}

static void
fgp_itp_stop_read(struct fgp_softc *sc)
{
	if ((sc->sc_fgp_state & FGP_STATE_ITP_READING) != 0) {
		usbd_transfer_stop(sc->sc_itp_xfer[FGP_INTR_DT]);
		sc->sc_fgp_state &= ~FGP_STATE_ITP_READING;
	}
}

#ifdef EVDEV_SUPPORT
static int
fgp_oev_open(struct evdev_dev *evdev)
{
	struct fgp_softc *sc = evdev_get_softc(evdev);
	int rc = 0;

	mtx_assert(&sc->sc_mutex, MA_OWNED);

	fgp_itp_start_read(sc);

	sc->sc_fgp_state |= FGP_STATE_OEV_OPENED;
	return (rc);
}

static int
fgp_oev_close(struct evdev_dev *evdev)
{
	struct fgp_softc *sc = evdev_get_softc(evdev);

	mtx_assert(&sc->sc_mutex, MA_OWNED);
	if ((sc->sc_fgp_state & FGP_STATE_ODV_OPENED) == 0) {
		fgp_itp_stop_read(sc);
	}

	sc->sc_fgp_state &= ~FGP_STATE_OEV_OPENED;
	return (0);
}
#endif


static int
fgp_odv_open(struct usb_fifo *odv_fifo, int odv_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(odv_fifo);

	/* check for duplicate open, should not happen */
	if (sc->sc_odv_fflags & odv_flags)
		return (EBUSY);

	/* check for first open */
	if (sc->sc_odv_fflags == 0) {
		/* reset ODV status */
		memset(&sc->sc_odv_mouse_status, 0, sizeof(sc->sc_odv_mouse_status));
		sc->sc_pad_touch_cycles = 0;
	}

	if (odv_flags & FREAD) {
		if (usb_fifo_alloc_buffer(odv_fifo,
		    FGP_FIFO_BUF_SIZE, FGP_FIFO_QUEUE_MAXLEN)) {
			return (ENOMEM);
		}
		sc->sc_fgp_state |= FGP_STATE_ODV_OPENED;
	}

	sc->sc_odv_fflags |= (odv_flags & (FREAD | FWRITE));

	return (0);
}

static void
fgp_odv_close(struct usb_fifo *odv_fifo, int odv_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(odv_fifo);
	if (odv_flags & FREAD) {
		usb_fifo_free_buffer(odv_fifo);
		sc->sc_fgp_state &= ~FGP_STATE_ODV_OPENED;
	}

	sc->sc_odv_fflags &= ~(odv_flags & (FREAD | FWRITE));
}

static void
fgp_odv_reset_buf(struct fgp_softc *sc)
{
	/* reset read queue */
	usb_fifo_reset(sc->sc_odv_fifo.fp[USB_FIFO_RX]);
}

static int
fgp_odv_ioctl(struct usb_fifo *odv_fifo, u_long cmd, void *addr, int odv_flags)
{
	struct fgp_softc *sc = usb_fifo_softc(odv_fifo);
	mousemode_t      mode;
	int error = 0;

	mtx_lock(&sc->sc_mutex);

	switch(cmd) {
	case MOUSE_GETHWINFO:
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_GETHWINFO\n");
		*(mousehw_t *)addr = sc->sc_odv_mouse_hw;
		break;
	case MOUSE_GETMODE:
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_GETMODE\n");
		*(mousemode_t *)addr = sc->sc_odv_mouse_mode;
		break;
	case MOUSE_SETMODE:
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_SETMODE\n");
		mode = *(mousemode_t *)addr;

		if (mode.level == -1)
			/* Don't change the current setting */
			;
		else if ((mode.level < 0) || (mode.level > 1)) {
			error = EINVAL;
			break;
		}
		sc->sc_odv_mouse_mode.level = mode.level;
		sc->sc_odv_pollrate         = mode.rate;
		sc->sc_odv_mouse_hw.buttons = 3;

		if (sc->sc_odv_mouse_mode.level == 0) {
			sc->sc_odv_mouse_mode.protocol    = MOUSE_PROTO_MSC;
			sc->sc_odv_mouse_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
			sc->sc_odv_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_odv_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_odv_mouse_mode.level == 1) {
			sc->sc_odv_mouse_mode.protocol    = MOUSE_PROTO_SYSMOUSE;
			sc->sc_odv_mouse_mode.packetsize  = MOUSE_SYS_PACKETSIZE;
			sc->sc_odv_mouse_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_odv_mouse_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		fgp_odv_reset_buf(sc);
		break;
	case MOUSE_GETLEVEL:
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_GETLEVEL\n");
		*(int *)addr = sc->sc_odv_mouse_mode.level;
		break;
	case MOUSE_SETLEVEL:
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_SETLEVEL\n");
		if ((*(int *)addr < 0) || (*(int *)addr > 1)) {
			error = EINVAL;
			break;
		}
		sc->sc_odv_mouse_mode.level = *(int *)addr;
		sc->sc_odv_mouse_hw.buttons = 3;

		if (sc->sc_odv_mouse_mode.level == 0) {
			sc->sc_odv_mouse_mode.protocol    = MOUSE_PROTO_MSC;
			sc->sc_odv_mouse_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
			sc->sc_odv_mouse_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_odv_mouse_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_odv_mouse_mode.level == 1) {
			sc->sc_odv_mouse_mode.protocol    = MOUSE_PROTO_SYSMOUSE;
			sc->sc_odv_mouse_mode.packetsize  = MOUSE_SYS_PACKETSIZE;
			sc->sc_odv_mouse_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_odv_mouse_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		fgp_odv_reset_buf(sc);
		break;
	case MOUSE_GETSTATUS: {
		DPRINTFN(FGP_LLEVEL_DEBUG,"MOUSE_GETSTATUS\n");
		mousestatus_t *status = (mousestatus_t *)addr;

		*status = sc->sc_odv_mouse_status;
		sc->sc_odv_mouse_status.obutton = sc->sc_odv_mouse_status.button;
		sc->sc_odv_mouse_status.button  = 0;
		sc->sc_odv_mouse_status.dx      = 0;
		sc->sc_odv_mouse_status.dy      = 0;
		sc->sc_odv_mouse_status.dz      = 0;

		if (status->dx || status->dy || status->dz)
			status->flags |= MOUSE_POSCHANGED;
		if (status->button != status->obutton)
			status->flags |= MOUSE_BUTTONSCHANGED;
		break;
	}

	default:
		DPRINTFN(FGP_LLEVEL_DEBUG,"default: ERR\n");
		error = ENOTTY;
		break;
	}

	mtx_unlock(&sc->sc_mutex);
	return (error);
}

static void
fgp_odv_start_read(struct usb_fifo *fifo_usb)
{
	struct fgp_softc *sc = usb_fifo_softc(fifo_usb);

	fgp_itp_start_read(sc);
}

static void
fgp_odv_stop_read(struct usb_fifo *fifo_usb)
{
	struct fgp_softc *sc = usb_fifo_softc(fifo_usb);

#ifdef EVDEV_SUPPORT
	if (sc->sc_fgp_state & FGP_STATE_OEV_OPENED) {
		return;
	}
#endif
	fgp_itp_stop_read(sc);
}

static int
fgp_sysctl_move_scale_factor_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	u_int tmp;

	tmp = fgp_sysctl_params.mickeys_scale_factor;
	error = sysctl_handle_int(oidp, &tmp, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (tmp == fgp_sysctl_params.mickeys_scale_factor)
		return (0);     /* no change */
	if ((tmp == 0) || (tmp > (10 * FGP_MICKEYS_SCALE_FACTOR)))
		return (EINVAL);

	fgp_sysctl_params.mickeys_scale_factor = tmp;
	DPRINTFN(FGP_LLEVEL_INFO, "[core] %s: resetting mickeys_scale_factor to %u\n",
	    FGP_DRIVER_NAME, tmp);

	return (0);
}

static int
fgp_sysctl_scroll_scale_factor_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	u_int tmp;

	tmp = fgp_sysctl_params.scroll_scale_factor;
	error = sysctl_handle_int(oidp, &tmp, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (tmp == fgp_sysctl_params.scroll_scale_factor)
		return (0);     /* no change */
	if ((tmp == 0) || (tmp > (10 * FGP_SCROLL_SCALE_FACTOR)))
		return (EINVAL);

	fgp_sysctl_params.scroll_scale_factor = tmp;
	DPRINTFN(FGP_LLEVEL_INFO, "[core] %s: resetting scroll_scale_factor to %u\n",
	    FGP_DRIVER_NAME, tmp);

	return (0);
}

static device_method_t fgp_itp_methods[] = {
	DEVMETHOD(device_probe,  fgp_itp_probe),
	DEVMETHOD(device_attach, fgp_itp_attach),
	DEVMETHOD(device_detach, fgp_itp_detach),

	DEVMETHOD_END
};

static driver_t fgp_itp_driver = {
	.name    = FGP_DRIVER_NAME,
	.methods = fgp_itp_methods,
	.size    = sizeof(struct fgp_softc)
};

DRIVER_MODULE(fgp, uhub, fgp_itp_driver, NULL, NULL);
MODULE_DEPEND(fgp, usb, 1, 1, 1);
MODULE_DEPEND(fgp, hid, 1, 1, 1);
#ifdef EVDEV_SUPPORT
MODULE_DEPEND(fgp, evdev, 1, 1, 1);
#endif
MODULE_VERSION(fgp, 1);
USB_PNP_HOST_INFO(fg_models);
