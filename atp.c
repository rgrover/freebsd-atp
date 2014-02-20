/*-
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
 * TODO:
 *  - replace constants in top-level settings with MACROS
 *  - ensure sanity of variable settings.
 *  - verify well-spring dev params.
 *  - update old atp params
 *  - update man page.
 *  - borrow code for geyser4
 *  - atp_slide_min_movement
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <sys/selinfo.h>
#include <sys/poll.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>

#include "usbdevs.h"

#define USB_DEBUG_VAR atp_debug
#include <dev/usb/usb_debug.h>

#include <sys/mouse.h>

#define ATP_DRIVER_NAME "atp"

/*
 * Driver specific options: the following options may be set by
 * `options' statements in the kernel configuration file.
 */

/* The divisor used to translate sensor reported positions to mickeys. */
#ifndef ATP_SCALE_FACTOR
#define ATP_SCALE_FACTOR 8
#endif

/* Threshold of instantaneous deltas beyond which movement is considered fast.*/
#ifndef ATP_FAST_MOVEMENT_TRESHOLD
#define ATP_FAST_MOVEMENT_TRESHOLD 150
#endif

/*
 * This is the age (in microseconds) beyond which a touch is
 * considered to be a slide; and therefore a tap event isn't registered.
 */
#ifndef ATP_TOUCH_TIMEOUT
#define ATP_TOUCH_TIMEOUT 125000
#endif

/*
 * A double-tap followed by a single-finger slide is treated as a
 * special gesture. The driver responds to this gesture by assuming a
 * virtual button-press for the lifetime of the slide. The following
 * threshold is the maximum time gap (in microseconds) between the two
 * tap events preceding the slide for such a gesture.
 */
#ifndef ATP_DOUBLE_TAP_N_DRAG_THRESHOLD
#define ATP_DOUBLE_TAP_N_DRAG_THRESHOLD 200000
#endif

/*
 * The wait duration (in ticks) after losing a touch contact
 * before zombied strokes are reaped and turned into button events.
 */
#define ATP_ZOMBIE_STROKE_REAP_WINDOW   50
#if ATP_ZOMBIE_STROKE_REAP_WINDOW > 100
#error "ATP_ZOMBIE_STROKE_REAP_WINDOW too large"
#endif

/* Tunables */
static SYSCTL_NODE(_hw_usb, OID_AUTO, atp, CTLFLAG_RW, 0, "USB atp");

#ifdef USB_DEBUG
enum atp_log_level {
	ATP_LLEVEL_DISABLED = 0,
	ATP_LLEVEL_ERROR,
	ATP_LLEVEL_DEBUG,       /* for troubleshooting */
	ATP_LLEVEL_INFO,        /* for diagnostics */
};
static int atp_debug = ATP_LLEVEL_ERROR; /* the default is to only log errors */
SYSCTL_INT(_hw_usb_atp, OID_AUTO, debug, CTLFLAG_RW,
    &atp_debug, ATP_LLEVEL_ERROR, "ATP debug level");
#endif /* USB_DEBUG */

static u_int atp_touch_timeout = ATP_TOUCH_TIMEOUT;
SYSCTL_UINT(_hw_usb_atp, OID_AUTO, touch_timeout, CTLFLAG_RW,
    &atp_touch_timeout, 125000, "age threshold (in micros) for a touch");

static u_int atp_double_tap_threshold = ATP_DOUBLE_TAP_N_DRAG_THRESHOLD;
SYSCTL_UINT(_hw_usb_atp, OID_AUTO, double_tap_threshold, CTLFLAG_RW,
    &atp_double_tap_threshold, ATP_DOUBLE_TAP_N_DRAG_THRESHOLD,
    "maximum time (in micros) to allow association between a double-tap and "
    "drag gesture");

static u_int atp_mickeys_scale_factor = ATP_SCALE_FACTOR;
static int atp_sysctl_scale_factor_handler(SYSCTL_HANDLER_ARGS);
SYSCTL_PROC(_hw_usb_atp, OID_AUTO, scale_factor, CTLTYPE_UINT | CTLFLAG_RW,
    &atp_mickeys_scale_factor, sizeof(atp_mickeys_scale_factor),
    atp_sysctl_scale_factor_handler, "IU", "movement scale factor");

static u_int atp_small_movement_threshold = 30;
SYSCTL_UINT(_hw_usb_atp, OID_AUTO, small_movement, CTLFLAG_RW,
    &atp_small_movement_threshold, 30,
    "the small movement black-hole for filtering noise");

/*
 * The minimum age of a stroke for it to be considered mature; this
 * helps filter movements (noise) from immature strokes. Units: interrupts.
 */
static u_int atp_stroke_maturity_threshold = 4;
SYSCTL_UINT(_hw_usb_atp, OID_AUTO, stroke_maturity_threshold, CTLFLAG_RW,
    &atp_stroke_maturity_threshold, 4,
    "the minimum age of a stroke for it to be considered mature");


#define WELLSPRING_INTERFACE_INDEX 1

typedef enum interface_mode {
	RAW_SENSOR_MODE = (uint8_t)0x01,
	HID_MODE        = (uint8_t)0x08
} interface_mode;

enum {
	ATP_INTR_DT,
	ATP_N_TRANSFER,
};

#define ATP_FIFO_BUF_SIZE        8 /* bytes */
#define ATP_FIFO_QUEUE_MAXLEN   50 /* units */

/* trackpad header types */
enum wellspring_trackpad_type {
	WSP_TRACKPAD_TYPE1,          /* plain trackpad */
	WSP_TRACKPAD_TYPE2,          /* button integrated in trackpad */
	WSP_TRACKPAD_TYPE3           /* additional header fields since June 2013 */
};

/* trackpad finger structure - little endian */
struct wsp_finger_sensor_data {
	int16_t origin;         /* zero when switching track finger */
	int16_t abs_x;          /* absolute x coodinate */
	int16_t abs_y;          /* absolute y coodinate */
	int16_t rel_x;          /* relative x coodinate */
	int16_t rel_y;          /* relative y coodinate */
	int16_t tool_major;     /* tool area, major axis */
	int16_t tool_minor;     /* tool area, minor axis */
	int16_t orientation;    /* 16384 when point, else 15 bit angle */
	int16_t touch_major;    /* touch area, major axis */
	int16_t touch_minor;    /* touch area, minor axis */
	int16_t unused[3];      /* zeros */
	int16_t multi;          /* one finger: varies, more fingers: constant */
} __packed;

typedef struct wsp_finger_to_match {
	boolean_t matched; /* to track fingers as they match against strokes. */
	int       x,y;     /* location (scaled using the mickeys factor) */
} wsp_finger_t;

/* trackpad finger data offsets, le16-aligned */
#define WSP_TYPE1_FINGER_DATA_OFFSET  (13 * 2)
#define WSP_TYPE2_FINGER_DATA_OFFSET  (15 * 2)
#define WSP_TYPE3_FINGER_DATA_OFFSET  (19 * 2)

/* trackpad button data offsets */
#define WSP_TYPE2_BUTTON_DATA_OFFSET   15
#define WSP_TYPE3_BUTTON_DATA_OFFSET   23

/* list of device capability bits */
#define HAS_INTEGRATED_BUTTON   1

/* trackpad finger data size, empirically at least ten fingers */
#define WSP_MAX_FINGERS               16
#define WSP_SIZEOF_FINGER_SENSOR_DATA sizeof(struct wsp_finger_sensor_data)
#define WSP_SIZEOF_ALL_FINGER_DATA    (WSP_MAX_FINGERS * WSP_SIZEOF_FINGER_SENSOR_DATA)
#define WSP_MAX_FINGER_ORIENTATION    16384

#define N_PROD_BITS 8
#if (N_PROD_BITS < 8) || (N_PROD_BITS > 24)
#error "invalid value for N_PROD_BITS"
#endif
#define ENCODE_DRIVER_INFO(FAMILY, PROD)      \
    (((FAMILY) << N_PROD_BITS) | (PROD))
#define WELLSPRING_DRIVER_INFO(PRODUCT)       \
    ENCODE_DRIVER_INFO(TRACKPAD_FAMILY_WELLSPRING, PRODUCT)

#define DECODE_FAMILY_FROM_DRIVER_INFO(INFO)  ((INFO) >> N_PROD_BITS)
#define DECODE_PRODUCT_FROM_DRIVER_INFO(INFO) \
    ((INFO) & ((1 << N_PROD_BITS) - 1))

enum atp_trackpad_family {
	TRACKPAD_FAMILY_GEYSER,
	TRACKPAD_FAMILY_WELLSPRING,
	TRACKPAD_FAMILY_MAX /* keep this at the tail end of the enumeration */
};

enum wellspring_product {
	WELLSPRING1,
	WELLSPRING2,
	WELLSPRING3,
	WELLSPRING4,
	WELLSPRING4A,
	WELLSPRING5,
	WELLSPRING6A,
	WELLSPRING6,
	WELLSPRING5A,
	WELLSPRING7,
	WELLSPRING7A,
	WELLSPRING8,
	WELLSPRING_PRODUCT_MAX /* keep this at the end of the enumeration */
};

/* logical signal quality */
#define WSP_SN_PRESSURE 45      /* pressure signal-to-noise ratio */
#define WSP_SN_WIDTH    25      /* width signal-to-noise ratio */
#define WSP_SN_COORD    250     /* coordinate signal-to-noise ratio */
#define WSP_SN_ORIENT   10      /* orientation signal-to-noise ratio */

/* device-specific parameters */
struct wsp_param {
	int snratio;        /* signal-to-noise ratio */
	int min;            /* device minimum reading */
	int max;            /* device maximum reading */
};

/* device-specific configuration */
struct wsp_dev_params {
	uint8_t  caps;               /* device capability bitmask */
	uint8_t  tp_type;            /* type of trackpad interface */
	uint8_t  finger_data_offset; /* offset to trackpad finger data */
	uint16_t data_len;           /* data length of the trackpad interface */
	struct wsp_param p;          /* finger pressure limits */
	struct wsp_param w;          /* finger width limits */
	struct wsp_param x;          /* horizontal limits */
	struct wsp_param y;          /* vertical limits */
	struct wsp_param o;          /* orientation limits */
};

static const struct wsp_dev_params wsp_dev_params[WELLSPRING_PRODUCT_MAX] = {
	[WELLSPRING1] = {
		.caps       = 0,
		.tp_type    = WSP_TRACKPAD_TYPE1,
		.finger_data_offset  = WSP_TYPE1_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE1_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 256
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4824, 5342
		},
		.y = {
			WSP_SN_COORD, -172, 5820
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING2] = {
		.caps       = 0,
		.tp_type    = WSP_TRACKPAD_TYPE1,
		.finger_data_offset  = WSP_TYPE1_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE1_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 256
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4824, 4824
		},
		.y = {
			WSP_SN_COORD, -172, 4290
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING3] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4460, 5166
		},
		.y = {
			WSP_SN_COORD, -75, 6700
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING4] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4620, 5140
		},
		.y = {
			WSP_SN_COORD, -150, 6600
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING4A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4616, 5112
		},
		.y = {
			WSP_SN_COORD, -142, 5234
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING5] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4415, 5050
		},
		.y = {
			WSP_SN_COORD, -55, 6680
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING6] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4620, 5140
		},
		.y = {
			WSP_SN_COORD, -150, 6600
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING5A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4750, 5280
		},
		.y = {
			WSP_SN_COORD, -150, 6730
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING6A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4620, 5140
		},
		.y = {
			WSP_SN_COORD, -150, 6600
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING7] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4750, 5280
		},
		.y = {
			WSP_SN_COORD, -150, 6730
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING7A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4750, 5280
		},
		.y = {
			WSP_SN_COORD, -150, 6730
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING8] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.tp_type    = WSP_TRACKPAD_TYPE3,
		.finger_data_offset  = WSP_TYPE3_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE3_FINGER_DATA_OFFSET + WSP_SIZEOF_ALL_FINGER_DATA,
		.p = {
			WSP_SN_PRESSURE, 0, 300
		},
		.w = {
			WSP_SN_WIDTH, 0, 2048
		},
		.x = {
			WSP_SN_COORD, -4620, 5140
		},
		.y = {
			WSP_SN_COORD, -150, 6600
		},
		.o = {
			WSP_SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
};

#define ATP_DEV(v,p,i) { USB_VPI(USB_VENDOR_##v, USB_PRODUCT_##v##_##p, i) }

static const STRUCT_USB_HOST_ID wsp_devs[] = {
	/* MacbookAir1.1 */
	ATP_DEV(APPLE, WELLSPRING_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING1)),
	ATP_DEV(APPLE, WELLSPRING_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING1)),
	ATP_DEV(APPLE, WELLSPRING_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING1)),

	/* MacbookProPenryn, aka wellspring2 */
	ATP_DEV(APPLE, WELLSPRING2_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING2)),
	ATP_DEV(APPLE, WELLSPRING2_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING2)),
	ATP_DEV(APPLE, WELLSPRING2_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING2)),

	/* Macbook5,1 (unibody), aka wellspring3 */
	ATP_DEV(APPLE, WELLSPRING3_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING3)),
	ATP_DEV(APPLE, WELLSPRING3_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING3)),
	ATP_DEV(APPLE, WELLSPRING3_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING3)),

	/* MacbookAir3,2 (unibody), aka wellspring4 */
	ATP_DEV(APPLE, WELLSPRING4_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING4)),
	ATP_DEV(APPLE, WELLSPRING4_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING4)),
	ATP_DEV(APPLE, WELLSPRING4_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING4)),

	/* MacbookAir3,1 (unibody), aka wellspring4 */
	ATP_DEV(APPLE, WELLSPRING4A_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING4A)),
	ATP_DEV(APPLE, WELLSPRING4A_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING4A)),
	ATP_DEV(APPLE, WELLSPRING4A_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING4A)),

	/* Macbook8 (unibody, March 2011) */
	ATP_DEV(APPLE, WELLSPRING5_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING5)),
	ATP_DEV(APPLE, WELLSPRING5_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING5)),
	ATP_DEV(APPLE, WELLSPRING5_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING5)),

	/* MacbookAir4,1 (unibody, July 2011) */
	ATP_DEV(APPLE, WELLSPRING6A_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING6A)),
	ATP_DEV(APPLE, WELLSPRING6A_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING6A)),
	ATP_DEV(APPLE, WELLSPRING6A_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING6A)),

	/* MacbookAir4,2 (unibody, July 2011) */
	ATP_DEV(APPLE, WELLSPRING6_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING6)),
	ATP_DEV(APPLE, WELLSPRING6_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING6)),
	ATP_DEV(APPLE, WELLSPRING6_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING6)),

	/* Macbook8,2 (unibody) */
	ATP_DEV(APPLE, WELLSPRING5A_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING5A)),
	ATP_DEV(APPLE, WELLSPRING5A_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING5A)),
	ATP_DEV(APPLE, WELLSPRING5A_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING5A)),

	/* MacbookPro10,1 (unibody, June 2012) */
	/* MacbookPro11,? (unibody, June 2013) */
	ATP_DEV(APPLE, WELLSPRING7_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING7)),
	ATP_DEV(APPLE, WELLSPRING7_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING7)),
	ATP_DEV(APPLE, WELLSPRING7_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING7)),

	/* MacbookPro10,2 (unibody, October 2012) */
	ATP_DEV(APPLE, WELLSPRING7A_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING7A)),
	ATP_DEV(APPLE, WELLSPRING7A_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING7A)),
	ATP_DEV(APPLE, WELLSPRING7A_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING7A)),

	/* MacbookAir6,2 (unibody, June 2013) */
	ATP_DEV(APPLE, WELLSPRING8_ANSI, WELLSPRING_DRIVER_INFO(WELLSPRING8)),
	ATP_DEV(APPLE, WELLSPRING8_ISO,  WELLSPRING_DRIVER_INFO(WELLSPRING8)),
	ATP_DEV(APPLE, WELLSPRING8_JIS,  WELLSPRING_DRIVER_INFO(WELLSPRING8)),
};


struct atp_softc; /* forward declaration */
typedef void (*sensor_data_interpreter_t)(struct atp_softc *sc, unsigned len);

typedef enum atp_stroke_type {
	ATP_STROKE_TOUCH,
	ATP_STROKE_SLIDE,
} atp_stroke_type;

#define ATP_MAX_STROKES         (WSP_MAX_FINGERS)
/*
 * The following structure captures a finger contact with the
 * touchpad. A stroke comprises two p-span components and some state.
 */
typedef struct atp_stroke {
	atp_stroke_type      type;
	uint32_t             flags; /* the state of this stroke */
#define ATSF_ZOMBIE          0x1
	boolean_t matched;          /* to track match against fingers.*/

	struct timeval       ctime; /* create time; for coincident siblings. */
	u_int                age;   /*
				     * Unit: interrupts; we maintain
				     * this value in addition to
				     * 'ctime' in order to avoid the
				     * expensive call to microtime()
				     * at every interrupt.
				     */

	int x, y;                   /* location */

	/* Fields containing information about movement. */
	int   instantaneous_dx; /* curr. change in X location (un-smoothened) */
	int   instantaneous_dy; /* curr. change in Y location (un-smoothened) */
	int   pending_dx;       /* cum. of pending short movements */
	int   pending_dy;       /* cum. of pending short movements */
	int   movement_dx;      /* interpreted smoothened movement */
	int   movement_dy;      /* interpreted smoothened movement */
	u_int cum_movement;     /* cum. absolute movement so far */

	u_int velocity_squared;/* Avg. magnitude (squared) of recent velocity.*/
} atp_stroke_t;

struct atp_softc {
	device_t             sc_dev;
	struct usb_device   *sc_usb_device;
	struct mtx           sc_mutex; /* for synchronization */
	struct usb_fifo_sc   sc_fifo;

	const struct wsp_dev_params *sc_params; /* device configuration */

	mousehw_t            sc_hw;
	mousemode_t          sc_mode;
	mousestatus_t        sc_status;

	u_int                sc_state;
#define ATP_ENABLED          0x01
#define ATP_ZOMBIES_EXIST    0x02
#define ATP_DOUBLE_TAP_DRAG  0x04

	struct usb_xfer     *sc_xfer[ATP_N_TRANSFER];

	u_int                sc_pollrate;
	int                  sc_fflags;

	int8_t              *sensor_data; /* from interrupt packet */
	sensor_data_interpreter_t sensor_data_interpreter;

	atp_stroke_t         sc_strokes[ATP_MAX_STROKES];
	u_int                sc_n_strokes;

	struct callout	     sc_callout;

	uint8_t              sc_ibtn; /*
				       * button status. Set to non-zero if the
				       * mouse-button is physically pressed.
				       * This state variable is exposed through
				       * softc to allow reap_sibling_zombies
				       * to avoid registering taps while the
				       * trackpad button is pressed.
				       */

	struct timeval       sc_reap_time; /* time when zombies were reaped */
};

/*
 * function prototypes
 */
static usb_fifo_cmd_t   atp_start_read;
static usb_fifo_cmd_t   atp_stop_read;
static usb_fifo_open_t  atp_open;
static usb_fifo_close_t atp_close;
static usb_fifo_ioctl_t atp_ioctl;

static struct usb_fifo_methods atp_fifo_methods = {
	.f_open       = &atp_open,
	.f_close      = &atp_close,
	.f_ioctl      = &atp_ioctl,
	.f_start_read = &atp_start_read,
	.f_stop_read  = &atp_stop_read,
	.basename[0]  = ATP_DRIVER_NAME,
};

/* device initialization and shutdown */
static int  atp_set_device_mode(struct atp_softc *sc, interface_mode mode);
static int  atp_enable(struct atp_softc *sc);
static void atp_disable(struct atp_softc *sc);
static int  atp_softc_populate(struct atp_softc *);
static void atp_softc_unpopulate(struct atp_softc *);

static void atp_interpret_wellspring_data(struct atp_softc *, unsigned);
static boolean_t atp_update_wellspring_strokes(struct atp_softc *,
    wsp_finger_t [WSP_MAX_FINGERS], u_int);

/* movement detection */
static __inline void atp_add_stroke(struct atp_softc *, const wsp_finger_t *);
static void          atp_terminate_stroke(struct atp_softc *, u_int);
static boolean_t     wsp_match_strokes_against_fingers(struct atp_softc *,
    wsp_finger_t *, u_int);
static void          atp_advance_stroke_state(struct atp_softc *,
    atp_stroke_t *, boolean_t *);
static __inline boolean_t atp_stroke_has_small_movement(const atp_stroke_t *);
static __inline void atp_update_pending_mickeys(atp_stroke_t *);
static boolean_t     atp_compute_stroke_movement(atp_stroke_t *);

/* tap detection */
static void          atp_reap_sibling_zombies(void *);
static void          atp_convert_to_slide(struct atp_softc *, atp_stroke_t *);

/* updating fifo */
static void          atp_add_to_queue(struct atp_softc *, int, int, int,
    uint32_t);

sensor_data_interpreter_t atp_sensor_data_interpreters[TRACKPAD_FAMILY_MAX] = {
	[TRACKPAD_FAMILY_WELLSPRING] = atp_interpret_wellspring_data,
};

#define MODE_LENGTH 8 /* num bytes holding the device mode */

/* Device methods. */
static device_probe_t  atp_probe;
static device_attach_t atp_attach;
static device_detach_t atp_detach;
static usb_callback_t  atp_intr;

static struct usb_config atp_xfer_config[ATP_N_TRANSFER] = {
	[ATP_INTR_DT] = {
		.type      = UE_INTERRUPT,
		.endpoint  = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.pipe_bof = 1, /* block pipe on failure */
			.short_xfer_ok = 1,
		},
		.bufsize   = 0, /* use wMaxPacketSize */
		.callback  = &atp_intr,
	},
};

static int
atp_set_device_mode(struct atp_softc *sc, interface_mode newMode)
{
	usb_error_t err;
	uint8_t     mode_bytes[MODE_LENGTH];

	if ((newMode != RAW_SENSOR_MODE) && (newMode != HID_MODE))
		return (ENXIO);

	/*
	 * Read the mode; perhaps this could be cached in a static variable
	 * or in the softc, but reading it live from the device may not cause
	 * much overhead either.
	 */
	err = usbd_req_get_report(sc->sc_usb_device, NULL /* mutex */,
	    mode_bytes, sizeof(mode_bytes), 0 /* interface index */,
	    0x03 /* type */, 0x00 /* id */);
	if (err != USB_ERR_NORMAL_COMPLETION) {
		DPRINTF("Failed to read device mode (%d)\n", err);
		return (ENXIO);
	}

	if (mode_bytes[0] == newMode) {
		return (0);
	}
	mode_bytes[0] = newMode;

	return (usbd_req_set_report(sc->sc_usb_device, NULL /* mutex */,
	    mode_bytes, sizeof(mode_bytes), 0 /* interface index */,
	    0x03 /* type */, 0x00 /* id */));
}

static int
atp_enable(struct atp_softc *sc)
{
	if (sc->sc_state & ATP_ENABLED)
		return (0);

	const struct wsp_dev_params *params = sc->sc_params;
	if ((params == NULL) || (params->data_len == 0)) {
		DPRINTF("params uninitialized!\n");
		return (ENXIO);
	}

	/* Allocate the dynamic buffers */
	if (atp_softc_populate(sc) != 0) {
		atp_softc_unpopulate(sc);
		return (ENOMEM);
	}

	/* reset status */
	memset(&sc->sc_status, 0, sizeof(sc->sc_status));

	memset(sc->sc_strokes, 0, sizeof(sc->sc_strokes));
	sc->sc_n_strokes = 0;
	// sc->sc_idlecount = 0;
	sc->sc_state |= ATP_ENABLED;

	DPRINTFN(ATP_LLEVEL_INFO, "enabled atp\n");
	return (0);
}

static void
atp_disable(struct atp_softc *sc)
{
	atp_softc_unpopulate(sc);

	sc->sc_state &= ~(ATP_ENABLED);
	DPRINTFN(ATP_LLEVEL_INFO, "disabled atp\n");
}

/* Allocate dynamic memory for some fields in softc. */
static int
atp_softc_populate(struct atp_softc *sc)
{
	const struct wsp_dev_params *params = sc->sc_params;

	if (params == NULL) {
		DPRINTF("params uninitialized!\n");
		return (ENXIO);
	}
	if (params->data_len) {
		sc->sensor_data = malloc(params->data_len * sizeof(int8_t),
		    M_USB, M_WAITOK | M_ZERO);
		if (sc->sensor_data == NULL) {
			DPRINTF("mem for sensor_data\n");
			return (ENXIO);
		}
	}

	return (0);
}

/* Free dynamic memory allocated for some fields in softc. */
static void
atp_softc_unpopulate(struct atp_softc *sc)
{
	const struct wsp_dev_params *params = sc->sc_params;

	if (params == NULL) {
		return;
	}

	if (sc->sensor_data != NULL) {
		free(sc->sensor_data, M_USB);
		sc->sensor_data = NULL;
	}
}

static int
atp_probe(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	if (uaa->info.bInterfaceClass != UICLASS_HID)
		return (ENXIO);
	/*
	 * Note: for some reason, the check
	 * (uaa->info.bInterfaceProtocol == UIPROTO_MOUSE) doesn't hold true
	 * for wellspring trackpads, so we've removed it from the common path.
	 */

	if ((usbd_lookup_id_by_uaa(wsp_devs, sizeof(wsp_devs), uaa)) == 0) {
		if (uaa->info.bIfaceIndex == WELLSPRING_INTERFACE_INDEX)
			return (0);
	}

	return (ENXIO);
}

static int
atp_attach(device_t dev)
{
	struct atp_softc      *sc  = device_get_softc(dev);
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	usb_error_t            err;

	DPRINTFN(ATP_LLEVEL_INFO, "sc=%p\n", sc);

	sc->sc_dev        = dev;
	sc->sc_usb_device = uaa->device;

	/*
	 * By default the touchpad behaves like an HID device, sending
	 * packets with reportID = 2. Such reports contain only
	 * limited information--they encode movement deltas and button
	 * events,--but do not include data from the pressure
	 * sensors. The device input mode can be switched from HID
	 * reports to raw sensor data using vendor-specific USB
	 * control commands.
	 */
	if (atp_set_device_mode(sc, RAW_SENSOR_MODE) != 0) {
		DPRINTF("failed to set mode to 'RAW_SENSOR' (%d)\n", err);
		return (ENXIO);
	}

	mtx_init(&sc->sc_mutex, "atpmtx", NULL, MTX_DEF | MTX_RECURSE);

	unsigned long di = USB_GET_DRIVER_INFO(uaa);
	if (DECODE_FAMILY_FROM_DRIVER_INFO(di) == TRACKPAD_FAMILY_WELLSPRING) {
		sc->sc_params =
		    &wsp_dev_params[DECODE_PRODUCT_FROM_DRIVER_INFO(di)];
		sc->sensor_data_interpreter = atp_interpret_wellspring_data;
	}
	atp_xfer_config[ATP_INTR_DT].bufsize = sc->sc_params->data_len;

	err = usbd_transfer_setup(uaa->device,
	    &uaa->info.bIfaceIndex, sc->sc_xfer, atp_xfer_config,
	    ATP_N_TRANSFER, sc, &sc->sc_mutex);
	if (err) {
		DPRINTF("error=%s\n", usbd_errstr(err));
		goto detach;
	}

	if (usb_fifo_attach(sc->sc_usb_device, sc, &sc->sc_mutex,
	    &atp_fifo_methods, &sc->sc_fifo,
	    device_get_unit(dev), -1, uaa->info.bIfaceIndex,
	    UID_ROOT, GID_OPERATOR, 0644)) {
		goto detach;
	}

	device_set_usb_desc(dev);

	sc->sc_hw.buttons       = 3;
	sc->sc_hw.iftype        = MOUSE_IF_USB;
	sc->sc_hw.type          = MOUSE_PAD;
	sc->sc_hw.model         = MOUSE_MODEL_GENERIC;
	sc->sc_hw.hwid          = 0;
	sc->sc_mode.protocol    = MOUSE_PROTO_MSC;
	sc->sc_mode.rate        = -1;
	sc->sc_mode.resolution  = MOUSE_RES_UNKNOWN;
	sc->sc_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
	sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
	sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
	sc->sc_mode.accelfactor = 0;
	sc->sc_mode.level       = 0;

	sc->sc_state            = 0;
	sc->sc_ibtn             = 0;

	callout_init_mtx(&sc->sc_callout, &sc->sc_mutex, 0);

	return (0);

detach:
	atp_detach(dev);
	return (ENOMEM);
}

static int
atp_detach(device_t dev)
{
	struct atp_softc *sc;

	sc = device_get_softc(dev);
	atp_set_device_mode(sc, HID_MODE);

	mtx_lock(&sc->sc_mutex);
	callout_drain(&sc->sc_callout);
	if (sc->sc_state & ATP_ENABLED)
		atp_disable(sc);
	mtx_unlock(&sc->sc_mutex);

	usb_fifo_detach(&sc->sc_fifo);

	usbd_transfer_unsetup(sc->sc_xfer, ATP_N_TRANSFER);

	mtx_destroy(&sc->sc_mutex);

	return (0);
}

void
atp_intr(struct usb_xfer *xfer, usb_error_t error)
{
	struct atp_softc            *sc     = usbd_xfer_softc(xfer);
	const struct wsp_dev_params *params = sc->sc_params;
	int                          len;
	struct usb_page_cache       *pc;

	usbd_xfer_status(xfer, &len, NULL, NULL, NULL);

	switch (USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		if (len > (int)params->data_len) {
			DPRINTFN(WSP_LLEVEL_ERROR,
			    "truncating large packet from %u to %u bytes\n",
			    len, params->data_len);
			len = params->data_len;
		} else if (len < params->data_len) {
			/* zero-out any previous sensor-data state */
			memset(sc->sensor_data + len, 0,
			    params->data_len - len);
		}

		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, sc->sensor_data, len);

		sc->sc_status.flags &= ~(MOUSE_STDBUTTONSCHANGED |
		    MOUSE_POSCHANGED);
		sc->sc_status.obutton = sc->sc_status.button;

		(sc->sensor_data_interpreter)(sc, len);

		if (sc->sc_status.button != 0) {
			/* Reset DOUBLE_TAP_N_DRAG if the button is pressed. */
			sc->sc_state &= ~ATP_DOUBLE_TAP_DRAG;
		} else if (sc->sc_state & ATP_DOUBLE_TAP_DRAG) {
			/* Assume a button-press with DOUBLE_TAP_N_DRAG. */
			sc->sc_status.button = MOUSE_BUTTON1DOWN;
		}

		sc->sc_status.flags |=
		    sc->sc_status.button ^ sc->sc_status.obutton;
		if (sc->sc_status.flags & MOUSE_STDBUTTONSCHANGED) {
		    DPRINTFN(ATP_LLEVEL_INFO, "button %s\n",
			((sc->sc_status.button & MOUSE_BUTTON1DOWN) ?
			"pressed" : "released"));
		}

		if (sc->sc_status.flags &
		    (MOUSE_POSCHANGED | MOUSE_STDBUTTONSCHANGED)) {
			int   dx, dy;
			u_int n_movements;
			u_int i;

			dx = 0, dy = 0, n_movements = 0;
			atp_stroke_t *strokep = sc->sc_strokes;
			for (i = 0; i < sc->sc_n_strokes; i++, strokep++) {
				dx += strokep->movement_dx;
				dy += strokep->movement_dy;
				if (strokep->movement_dx ||
				    strokep->movement_dy)
					n_movements++;
			}
			/* average movement if multiple strokes record motion.*/
			if (n_movements > 1) {
				dx /= (int)n_movements;
				dy /= (int)n_movements;
			}

			sc->sc_status.dx += dx;
			sc->sc_status.dy += dy;
			atp_add_to_queue(sc, dx, -dy, 0, sc->sc_status.button);
		}

	case USB_ST_SETUP:
	tr_setup:
		/* check if we can put more data into the FIFO */
		if (usb_fifo_put_bytes_max(sc->sc_fifo.fp[USB_FIFO_RX]) != 0) {
			usbd_xfer_set_frame_len(xfer, 0,
			    sc->sc_params->data_len);
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

void
atp_interpret_wellspring_data(struct atp_softc *sc, unsigned data_len)
{
	const struct wsp_dev_params *params = sc->sc_params;

	/* validate sensor data length */
	if ((data_len < params->finger_data_offset) ||
	    ((data_len - params->finger_data_offset) %
	     WSP_SIZEOF_FINGER_SENSOR_DATA) != 0)
		return;

	unsigned n_source_fingers = (data_len - params->finger_data_offset) /
	    WSP_SIZEOF_FINGER_SENSOR_DATA;
	n_source_fingers = min(n_source_fingers, WSP_MAX_FINGERS);

	/* iterate over the source data collecting useful fingers */
	wsp_finger_t fingers[WSP_MAX_FINGERS];
	unsigned i, n_fingers = 0;
	struct wsp_finger_sensor_data *source_fingerp =
	    (struct wsp_finger_sensor_data *)(sc->sensor_data +
	     params->finger_data_offset);
	for (i = 0; i < n_source_fingers; i++, source_fingerp++) {
		if (le16toh(source_fingerp->touch_major) == 0)
			continue;

		fingers[n_fingers].matched = false;
		fingers[n_fingers].x       =
		    imax(params->x.min, source_fingerp->abs_x) - params->x.min;
		fingers[n_fingers].y       = -params->y.min +
		    params->y.max - source_fingerp->abs_y;

		++n_fingers;
	}

	if ((sc->sc_n_strokes == 0) && (n_fingers == 0))
		return;

	if (atp_update_wellspring_strokes(sc, fingers, n_fingers))
		sc->sc_status.flags |= MOUSE_POSCHANGED;

	switch(params->tp_type) {
	case WSP_TRACKPAD_TYPE2:
		sc->sc_ibtn = sc->sensor_data[WSP_TYPE2_BUTTON_DATA_OFFSET];
		break;
	case WSP_TRACKPAD_TYPE3:
		sc->sc_ibtn = sc->sensor_data[WSP_TYPE3_BUTTON_DATA_OFFSET];
		break;
	default:
		break;
	}
	sc->sc_status.button = sc->sc_ibtn ? MOUSE_BUTTON1DOWN : 0;
}

/* Initialize a stroke from an unmatched finger. */
static __inline void
atp_add_stroke(struct atp_softc *sc, const wsp_finger_t *fingerp)
{
	atp_stroke_t *strokep;

	if (sc->sc_n_strokes >= ATP_MAX_STROKES)
		return;
	strokep = &sc->sc_strokes[sc->sc_n_strokes];

	memset(strokep, 0, sizeof(atp_stroke_t));

	/*
	 * Strokes begin as potential touches. If a stroke survives
	 * longer than a threshold, or if it records significant
	 * cumulative movement, then it is considered a 'slide'.
	 */
	strokep->type    = ATP_STROKE_TOUCH;
	strokep->matched = true;
	strokep->x       = fingerp->x;
	strokep->y       = fingerp->y;

	microtime(&strokep->ctime);
	strokep->age     = 1;       /* Unit: interrupts */

	sc->sc_n_strokes++;

	/* Reset double-tap-n-drag if we have more than one strokes. */
	if (sc->sc_n_strokes > 1)
		sc->sc_state &= ~ATP_DOUBLE_TAP_DRAG;

	DPRINTFN(ATP_LLEVEL_INFO, "[%d,%d]\n", strokep->x, strokep->y);
}

/*
 * Terminate a stroke. While SLIDE strokes are dropped, TOUCH strokes
 * are retained as zombies so as to reap all their siblings together;
 * this helps establish the number of fingers involved in the tap.
 */
static void
atp_terminate_stroke(struct atp_softc *sc, u_int index)
{
	atp_stroke_t *strokep = &sc->sc_strokes[index];

	if (strokep->flags & ATSF_ZOMBIE)
		return;

	if ((strokep->type == ATP_STROKE_TOUCH) &&
	    (strokep->age > atp_stroke_maturity_threshold)) {
		strokep->flags |= ATSF_ZOMBIE;
		sc->sc_state   |= ATP_ZOMBIES_EXIST;
		callout_reset(&sc->sc_callout, ATP_ZOMBIE_STROKE_REAP_WINDOW,
		    atp_reap_sibling_zombies, sc);
	} else {
		/* Drop this stroke. */
		sc->sc_n_strokes--;
		if (index < sc->sc_n_strokes)
			memcpy(strokep, strokep + 1,
			    (sc->sc_n_strokes - index) * sizeof(atp_stroke_t));

		/*
		 * Reset the double-click-n-drag at the termination of
		 * any slide stroke.
		 */
		sc->sc_state &= ~ATP_DOUBLE_TAP_DRAG;
	}
}

static __inline boolean_t
atp_stroke_has_small_movement(const atp_stroke_t *strokep)
{
	return (((u_int)abs(strokep->instantaneous_dx) <=
		 atp_small_movement_threshold) &&
		((u_int)abs(strokep->instantaneous_dy) <=
		 atp_small_movement_threshold));
}

/*
 * Accumulate instantaneous changes into the stroke's 'pending' bucket; if
 * the aggregate exceeds the small_movement_threshold, then retain
 * instantaneous changes for later.
 */
static __inline void
atp_update_pending_mickeys(atp_stroke_t *strokep)
{
	/* accumulate instantaneous movement */
	strokep->pending_dx += strokep->instantaneous_dx;
	strokep->pending_dy += strokep->instantaneous_dy;

#define UPDATE_INSTANTANEOUS_AND_PENDING(I, P)                          \
	if (abs((P)) <= atp_small_movement_threshold)                   \
		(I) = 0; /* clobber small movement */                   \
	else {                                                          \
		if ((I) > 0) {                                          \
			/*                                              \
			 * Round up instantaneous movement to the nearest \
			 * ceiling. This helps preserve small mickey    \
			 * movements from being lost in following scaling \
			 * operation.                                   \
			 */                                             \
			(I) = (((I) + (atp_mickeys_scale_factor - 1)) / \
			       atp_mickeys_scale_factor) *              \
			      atp_mickeys_scale_factor;                 \
									\
			/*                                              \
			 * Deduct the rounded mickeys from pending mickeys. \
			 * Note: we multiply by 2 to offset the previous \
			 * accumulation of instantaneous movement into  \
			 * pending.                                     \
			 */                                             \
			(P) -= ((I) << 1);                              \
									\
			/* truncate pending to 0 if it becomes negative. */ \
			(P) = imax((P), 0);                             \
		} else {                                                \
			/*                                              \
			 * Round down instantaneous movement to the nearest \
			 * ceiling. This helps preserve small mickey    \
			 * movements from being lost in following scaling \
			 * operation.                                   \
			 */                                             \
			(I) = (((I) - (atp_mickeys_scale_factor - 1)) / \
			       atp_mickeys_scale_factor) *              \
			      atp_mickeys_scale_factor;                 \
									\
			/*                                              \
			 * Deduct the rounded mickeys from pending mickeys. \
			 * Note: we multiply by 2 to offset the previous \
			 * accumulation of instantaneous movement into  \
			 * pending.                                     \
			 */                                             \
			(P) -= ((I) << 1);                              \
									\
			/* truncate pending to 0 if it becomes positive. */ \
			(P) = imin((P), 0);                             \
		}                                                       \
	}

	UPDATE_INSTANTANEOUS_AND_PENDING(strokep->instantaneous_dx,
	    strokep->pending_dx);
	UPDATE_INSTANTANEOUS_AND_PENDING(strokep->instantaneous_dy,
	    strokep->pending_dy);
}

/*
 * Compute a smoothened value for the stroke's movement from
 * instantaneous changes in the X and Y components.
 */
static boolean_t
atp_compute_stroke_movement(atp_stroke_t *strokep)
{
	/*
	 * Short movements are added first to the 'pending' bucket,
	 * and then acted upon only when their aggregate exceeds a
	 * threshold. This has the effect of filtering away movement
	 * noise.
	 */
	if (atp_stroke_has_small_movement(strokep))
		atp_update_pending_mickeys(strokep);
	else {                /* large movement */
		/* clear away any pending mickeys if there are large movements*/
		strokep->pending_dx = 0;
		strokep->pending_dy = 0;
	}

	/* scale movement */
	strokep->movement_dx = (strokep->instantaneous_dx) /
	    (int)atp_mickeys_scale_factor;
	strokep->movement_dy = (strokep->instantaneous_dy) /
	    (int)atp_mickeys_scale_factor;

	if ((abs(strokep->instantaneous_dx) >= ATP_FAST_MOVEMENT_TRESHOLD) ||
	    (abs(strokep->instantaneous_dy) >= ATP_FAST_MOVEMENT_TRESHOLD)) {
		strokep->movement_dx <<= 1;
		strokep->movement_dy <<= 1;
	}

	strokep->cum_movement +=
	    abs(strokep->movement_dx) + abs(strokep->movement_dy);

	return ((strokep->movement_dx != 0) || (strokep->movement_dy != 0));
}

void
atp_advance_stroke_state(struct atp_softc *sc, atp_stroke_t *strokep,
    boolean_t *movementp)
{
	/* Revitalize stroke if it had previously been marked as a zombie. */
	if (strokep->flags & ATSF_ZOMBIE)
		strokep->flags &= ~ATSF_ZOMBIE;

	strokep->age++;
	if (strokep->age <= atp_stroke_maturity_threshold) {
		/* Avoid noise from immature strokes. */
		strokep->instantaneous_dx = 0;
		strokep->instantaneous_dy = 0;
	}

	if (atp_compute_stroke_movement(strokep))
		*movementp = TRUE;

	/* Compute the stroke's age. */
	struct timeval tdiff;
	getmicrotime(&tdiff);
	if (timevalcmp(&tdiff, &strokep->ctime, >))
		timevalsub(&tdiff, &strokep->ctime);
	else {
		/*
		 * If we are here, it is because getmicrotime
		 * reported the current time as being behind
		 * the stroke's start time; getmicrotime can
		 * be imprecise.
		 */
		tdiff.tv_sec  = 0;
		tdiff.tv_usec = 0;
	}

	if ((tdiff.tv_sec > (atp_touch_timeout / 1000000)) ||
	    ((tdiff.tv_sec == (atp_touch_timeout / 1000000)) &&
		(tdiff.tv_usec >= (atp_touch_timeout % 1000000))))
		atp_convert_to_slide(sc, strokep);
}

/* Switch a given touch stroke to being a slide. */
void
atp_convert_to_slide(struct atp_softc *sc, atp_stroke_t *strokep)
{
	strokep->type = ATP_STROKE_SLIDE;

	/* Are we at the beginning of a double-click-n-drag? */
	if ((sc->sc_n_strokes == 1) &&
	    ((sc->sc_state & ATP_ZOMBIES_EXIST) == 0) &&
	    timevalcmp(&strokep->ctime, &sc->sc_reap_time, >)) {
		struct timeval delta;
		struct timeval window = {
			atp_double_tap_threshold / 1000000,
			atp_double_tap_threshold % 1000000
		};

		delta = strokep->ctime;
		timevalsub(&delta, &sc->sc_reap_time);
		if (timevalcmp(&delta, &window, <=))
			sc->sc_state |= ATP_DOUBLE_TAP_DRAG;
	}
}

boolean_t
wsp_match_strokes_against_fingers(struct atp_softc *sc,
    wsp_finger_t *fingers, u_int n_fingers)
{
	boolean_t movement = false;
	const static unsigned MAX_ALLOWED_FINGER_DISTANCE = 1000000;
	unsigned si, fi;

	/* reset the matched status for all strokes */
	atp_stroke_t *strokep = sc->sc_strokes;
	for (si = 0; si < sc->sc_n_strokes; si++, strokep++) {
		strokep->matched = false;
	}

	wsp_finger_t *fingerp;
	fingerp = fingers;
	for (fi = 0; fi < n_fingers; fi++, fingerp++) {
		unsigned least_distance = MAX_ALLOWED_FINGER_DISTANCE;
		int best_stroke_index   = -1;

		strokep = sc->sc_strokes;
		for (si = 0; si < sc->sc_n_strokes; si++, strokep++) {
			if (strokep->matched)
				continue;

			int instantaneous_dx = fingerp->x - strokep->x;
			int instantaneous_dy = fingerp->y - strokep->y;

			/* skip strokes which are far away */
			unsigned d_squared =
			    (instantaneous_dx * instantaneous_dx) +
			    (instantaneous_dy * instantaneous_dy);
			if (d_squared > MAX_ALLOWED_FINGER_DISTANCE)
				continue;

			if (d_squared < least_distance) {
				least_distance    = d_squared;
				best_stroke_index = si;
			}
		}

		if (best_stroke_index != -1) {
			fingerp->matched = true;

			strokep = &sc->sc_strokes[best_stroke_index];
			strokep->matched          = true;
			strokep->instantaneous_dx = fingerp->x - strokep->x;
			strokep->instantaneous_dy = fingerp->y - strokep->y;
			strokep->x                = fingerp->x;
			strokep->y                = fingerp->y;

			atp_advance_stroke_state(sc, strokep, &movement);
		}
	}

	return (movement);
}

/*
 * Update strokes by matching against current pressure-spans.
 * Return TRUE if any movement is detected.
 */
boolean_t
atp_update_wellspring_strokes(struct atp_softc *sc,
    wsp_finger_t *fingers, u_int n_fingers)
{
	boolean_t movement = false;
	unsigned si, fi;

	if (sc->sc_n_strokes > 0) {
		movement = wsp_match_strokes_against_fingers(sc, fingers,
		    n_fingers);

		/* handle zombie strokes */
		atp_stroke_t *strokep = sc->sc_strokes;
		for (si = 0; si < sc->sc_n_strokes; si++, strokep++) {
			if (strokep->matched)
				continue;

			atp_terminate_stroke(sc, si);
		}
	}

	/* initialize unmatched fingers as strokes */
	wsp_finger_t *fingerp;
	fingerp = fingers;
	for (fi = 0; fi < n_fingers; fi++, fingerp++) {
		if (fingerp->matched)
			continue;

		atp_add_stroke(sc, fingerp);
	}

	return (movement);
}

static void
atp_add_to_queue(struct atp_softc *sc, int dx, int dy, int dz,
    uint32_t buttons_in)
{
	uint32_t buttons_out;
	uint8_t  buf[8];

	dx = imin(dx,  254); dx = imax(dx, -256);
	dy = imin(dy,  254); dy = imax(dy, -256);
	dz = imin(dz,  126); dz = imax(dz, -128);

	buttons_out = MOUSE_MSC_BUTTONS;
	if (buttons_in & MOUSE_BUTTON1DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON1UP;
	else if (buttons_in & MOUSE_BUTTON2DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON2UP;
	else if (buttons_in & MOUSE_BUTTON3DOWN)
		buttons_out &= ~MOUSE_MSC_BUTTON3UP;

	DPRINTFN(ATP_LLEVEL_INFO, "dx=%d, dy=%d, buttons=%x\n",
	    dx, dy, buttons_out);

	/* Encode the mouse data in standard format; refer to mouse(4) */
	buf[0] = sc->sc_mode.syncmask[1];
	buf[0] |= buttons_out;
	buf[1] = dx >> 1;
	buf[2] = dy >> 1;
	buf[3] = dx - (dx >> 1);
	buf[4] = dy - (dy >> 1);
	/* Encode extra bytes for level 1 */
	if (sc->sc_mode.level == 1) {
		buf[5] = dz >> 1;
		buf[6] = dz - (dz >> 1);
		buf[7] = (((~buttons_in) >> 3) & MOUSE_SYS_EXTBUTTONS);
	}

	usb_fifo_put_data_linear(sc->sc_fifo.fp[USB_FIFO_RX], buf,
	    sc->sc_mode.packetsize, 1);
}

static void
atp_reap_sibling_zombies(void *arg)
{
	struct atp_softc *sc = (struct atp_softc *)arg;
	if (sc->sc_n_strokes == 0)
		return;

	unsigned n_reaped = 0;

	int i;
	atp_stroke_t *strokep;
	for (i = 0; i < sc->sc_n_strokes; i++) {
		strokep = &sc->sc_strokes[i];
		if ((strokep->flags & ATSF_ZOMBIE) == 0)
			continue;

		/* Erase the stroke from the sc. */
		sc->sc_n_strokes--;
		if (i < sc->sc_n_strokes)
			memcpy(strokep, strokep + 1,
			    (sc->sc_n_strokes - i) * sizeof(atp_stroke_t));

		n_reaped += 1;
		--i; /* Decr. i to keep it unchanged for the next iteration */
	}

	DPRINTFN(ATP_LLEVEL_INFO, "reaped %u zombies\n", n_reaped);
	sc->sc_state &= ~ATP_ZOMBIES_EXIST;

	if (n_reaped != 0) {
		microtime(&sc->sc_reap_time); /* remember this time */

		/* Add a pair of events (button-down and button-up). */
		switch (n_reaped) {
		case 1: atp_add_to_queue(sc, 0, 0, 0, MOUSE_BUTTON1DOWN);
			break;
		case 2: atp_add_to_queue(sc, 0, 0, 0, MOUSE_BUTTON2DOWN);
			break;
		case 3: atp_add_to_queue(sc, 0, 0, 0, MOUSE_BUTTON3DOWN);
			break;
		default:
			break;/* handle taps of only up to 3 fingers */
		}
		atp_add_to_queue(sc, 0, 0, 0, 0); /* button release */
	}
}

static void
atp_reset_buf(struct atp_softc *sc)
{
	/* reset read queue */
	usb_fifo_reset(sc->sc_fifo.fp[USB_FIFO_RX]);
}

static void
atp_start_read(struct usb_fifo *fifo)
{
	struct atp_softc *sc = usb_fifo_softc(fifo);
	int rate;

	/* Check if we should override the default polling interval */
	rate = sc->sc_pollrate;
	/* Range check rate */
	if (rate > 1000)
		rate = 1000;
	/* Check for set rate */
	if ((rate > 0) && (sc->sc_xfer[ATP_INTR_DT] != NULL)) {
		/* Stop current transfer, if any */
		usbd_transfer_stop(sc->sc_xfer[ATP_INTR_DT]);
		/* Set new interval */
		usbd_xfer_set_interval(sc->sc_xfer[ATP_INTR_DT], 1000 / rate);
		/* Only set pollrate once */
		sc->sc_pollrate = 0;
	}

	usbd_transfer_start(sc->sc_xfer[ATP_INTR_DT]);
}

static void
atp_stop_read(struct usb_fifo *fifo)
{
	struct atp_softc *sc = usb_fifo_softc(fifo);
	usbd_transfer_stop(sc->sc_xfer[ATP_INTR_DT]);
}

static int
atp_open(struct usb_fifo *fifo, int fflags)
{
	struct atp_softc *sc = usb_fifo_softc(fifo);

	/* check for duplicate open, should not happen */
	if (sc->sc_fflags & fflags)
		return (EBUSY);

	/* check for first open */
	if (sc->sc_fflags == 0) {
		int rc;
		if ((rc = atp_enable(sc)) != 0)
			return (rc);
	}

	if (fflags & FREAD) {
		if (usb_fifo_alloc_buffer(fifo,
		    ATP_FIFO_BUF_SIZE, ATP_FIFO_QUEUE_MAXLEN)) {
			return (ENOMEM);
		}
	}

	sc->sc_fflags |= (fflags & (FREAD | FWRITE));
	return (0);
}

static void
atp_close(struct usb_fifo *fifo, int fflags)
{
	struct atp_softc *sc = usb_fifo_softc(fifo);
	if (fflags & FREAD)
		usb_fifo_free_buffer(fifo);

	sc->sc_fflags &= ~(fflags & (FREAD | FWRITE));
	if (sc->sc_fflags == 0) {
		atp_disable(sc);
	}
}

int
atp_ioctl(struct usb_fifo *fifo, u_long cmd, void *addr, int fflags)
{
	struct atp_softc *sc = usb_fifo_softc(fifo);
	mousemode_t mode;
	int error = 0;

	mtx_lock(&sc->sc_mutex);

	switch(cmd) {
	case MOUSE_GETHWINFO:
		*(mousehw_t *)addr = sc->sc_hw;
		break;
	case MOUSE_GETMODE:
		*(mousemode_t *)addr = sc->sc_mode;
		break;
	case MOUSE_SETMODE:
		mode = *(mousemode_t *)addr;

		if (mode.level == -1)
			/* Don't change the current setting */
			;
		else if ((mode.level < 0) || (mode.level > 1)) {
			error = EINVAL;
			break;
		}
		sc->sc_mode.level = mode.level;
		sc->sc_pollrate   = mode.rate;
		sc->sc_hw.buttons = 3;

		if (sc->sc_mode.level == 0) {
			sc->sc_mode.protocol    = MOUSE_PROTO_MSC;
			sc->sc_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
			sc->sc_mode.protocol    = MOUSE_PROTO_SYSMOUSE;
			sc->sc_mode.packetsize  = MOUSE_SYS_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		atp_reset_buf(sc);
		break;
	case MOUSE_GETLEVEL:
		*(int *)addr = sc->sc_mode.level;
		break;
	case MOUSE_SETLEVEL:
		if ((*(int *)addr < 0) || (*(int *)addr > 1)) {
			error = EINVAL;
			break;
		}
		sc->sc_mode.level = *(int *)addr;
		sc->sc_hw.buttons = 3;

		if (sc->sc_mode.level == 0) {
			sc->sc_mode.protocol    = MOUSE_PROTO_MSC;
			sc->sc_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;
		} else if (sc->sc_mode.level == 1) {
			sc->sc_mode.protocol    = MOUSE_PROTO_SYSMOUSE;
			sc->sc_mode.packetsize  = MOUSE_SYS_PACKETSIZE;
			sc->sc_mode.syncmask[0] = MOUSE_SYS_SYNCMASK;
			sc->sc_mode.syncmask[1] = MOUSE_SYS_SYNC;
		}
		atp_reset_buf(sc);
		break;
	case MOUSE_GETSTATUS: {
		mousestatus_t *status = (mousestatus_t *)addr;

		*status = sc->sc_status;
		sc->sc_status.obutton = sc->sc_status.button;
		sc->sc_status.button  = 0;
		sc->sc_status.dx      = 0;
		sc->sc_status.dy      = 0;
		sc->sc_status.dz      = 0;

		if (status->dx || status->dy || status->dz)
			status->flags |= MOUSE_POSCHANGED;
		if (status->button != status->obutton)
			status->flags |= MOUSE_BUTTONSCHANGED;
		break;
	}

	default:
		error = ENOTTY;
		break;
	}

	mtx_unlock(&sc->sc_mutex);
	return (error);
}

static int
atp_sysctl_scale_factor_handler(SYSCTL_HANDLER_ARGS)
{
	int error;
	u_int tmp;

	tmp = atp_mickeys_scale_factor;
	error = sysctl_handle_int(oidp, &tmp, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (tmp == atp_mickeys_scale_factor)
		return (0);     /* no change */

	atp_mickeys_scale_factor = tmp;
	DPRINTFN(ATP_LLEVEL_INFO, "%s: resetting mickeys_scale_factor to %u\n",
	    ATP_DRIVER_NAME, tmp);

	return (0);
}

static devclass_t atp_devclass;

static device_method_t atp_methods[] = {
	DEVMETHOD(device_probe,  atp_probe),
	DEVMETHOD(device_attach, atp_attach),
	DEVMETHOD(device_detach, atp_detach),

	DEVMETHOD_END
};

static driver_t atp_driver = {
	.name    = ATP_DRIVER_NAME,
	.methods = atp_methods,
	.size    = sizeof(struct atp_softc)
};

DRIVER_MODULE(atp, uhub, atp_driver, atp_devclass, NULL /* evh */, 0);
MODULE_DEPEND(atp, usb, 1, 1, 1);
MODULE_VERSION(atp, 1);
