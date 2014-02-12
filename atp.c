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
 *  - verify well-spring dev params.
 *  - update old atp params
 *  - add comment about not checking mouse subclass in probe()
 *  - update man page.
 *  - rename wsp_dev_params as wellspring_product_params
 *  - borrow code for geyser4
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

#ifdef USB_DEBUG
enum atp_log_level {
    ATP_LLEVEL_DISABLED = 0,
    ATP_LLEVEL_ERROR,
    ATP_LLEVEL_DEBUG,       /* for troubleshooting */
    ATP_LLEVEL_INFO,        /* for diagnostics */
};
static int atp_debug = ATP_LLEVEL_ERROR;/* the default is to only log errors */

SYSCTL_INT(_hw_usb_atp, OID_AUTO, debug, CTLFLAG_RW,
    &atp_debug, ATP_LLEVEL_ERROR, "ATP debug level");
#endif                  /* USB_DEBUG */

#define WELLSPRING_INTERFACE_INDEX 1

typedef enum interface_mode {
	RAW_SENSOR_MODE = (uint8_t)0x01,
	HID_MODE        = (uint8_t)0x08
} interface_mode;


/* Device methods. */
static device_probe_t  atp_probe;
static device_attach_t atp_attach;
static device_detach_t atp_detach;
static usb_callback_t  atp_intr;

enum {
	ATP_INTR_DT,
	ATP_N_TRANSFER,
};

#define ATP_FIFO_BUF_SIZE        8 /* bytes */
#define ATP_FIFO_QUEUE_MAXLEN   50 /* units */

/* button data structure */
struct bt_data {
	uint8_t unknown1;       /* constant */
	uint8_t button;         /* left button */
	uint8_t rel_x;          /* relative x coordinate */
	uint8_t rel_y;          /* relative y coordinate */
} __packed;

/* trackpad header types */
enum wellspring_trackpad_type {
	TYPE1,          /* plain trackpad */
	TYPE2,          /* button integrated in trackpad */
	TYPE3           /* additional header fields since June 2013 */
};

/* trackpad finger data offsets, le16-aligned */
#define WSP_TYPE1_FINGER_DATA_OFFSET  (13 * 2)
#define WSP_TYPE2_FINGER_DATA_OFFSET  (15 * 2)
#define WSP_TYPE3_FINGER_DATA_OFFSET  (19 * 2)

/* trackpad button data offsets */
#define BUTTON_TYPE2        15
#define BUTTON_TYPE3        23

/* list of device capability bits */
#define HAS_INTEGRATED_BUTTON   1

/* trackpad finger structure - little endian */
struct wsp_finger {
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

/* trackpad finger data size, empirically at least ten fingers */
#define WSP_MAX_FINGERS            16
#define WSP_SIZEOF_FINGER_STRUCT   sizeof(struct wsp_finger)
#define WSP_SIZEOF_FINGER_DATA     (WSP_MAX_FINGERS * WSP_SIZEOF_FINGER_STRUCT)
#define WSP_MAX_FINGER_ORIENTATION 16384

/* logical signal quality */
#define SN_PRESSURE 45      /* pressure signal-to-noise ratio */
#define SN_WIDTH    25      /* width signal-to-noise ratio */
#define SN_COORD    250     /* coordinate signal-to-noise ratio */
#define SN_ORIENT   10      /* orientation signal-to-noise ratio */

/* device-specific parameters */
struct wsp_param {
	int snratio;        /* signal-to-noise ratio */
	int min;            /* device minimum reading */
	int max;            /* device maximum reading */
};

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

/* device-specific configuration */
struct wsp_dev_params {
	uint8_t  caps;               /* device capability bitmask */
	uint16_t bt_datalen;         /* data length of the button interface */
	uint8_t  tp_type;            /* type of trackpad interface */
	uint8_t  finger_data_offset; /* offset to trackpad finger data */
	uint16_t data_len;           /* data length of the trackpad interface */
	struct wsp_param p;          /* finger pressure limits */
	struct wsp_param w;          /* finger width limits */
	struct wsp_param x;          /* horizontal limits */
	struct wsp_param y;          /* vertical limits */
	struct wsp_param o;          /* orientation limits */
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
// 	struct timeval       ctime; /* create time; for coincident siblings. */
// 	u_int                age;   /*
// 				     * Unit: interrupts; we maintain
// 				     * this value in addition to
// 				     * 'ctime' in order to avoid the
// 				     * expensive call to microtime()
// 				     * at every interrupt.
// 				     */

// 	atp_stroke_component components[2];
// 	u_int                velocity_squared; /*
// 						* Average magnitude (squared)
// 						* of recent velocity.
// 						*/
// 	u_int                cum_movement; /* cum. absolute movement so far */

// 	uint32_t             flags;  /* the state of this stroke */
// #define ATSF_ZOMBIE          0x1
} atp_stroke;

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
#define ATP_VALID            0x08

	struct usb_xfer     *sc_xfer[ATP_N_TRANSFER];

	u_int                sc_pollrate;
	int                  sc_fflags;

	int8_t              *sensor_data; /* from interrupt packet */
	sensor_data_interpreter_t sensor_data_interpreter;

	atp_stroke           sc_strokes[ATP_MAX_STROKES];
	u_int                sc_n_strokes;

//         u_int                  sc_left_margin;
//         u_int                  sc_right_margin;
//         int                   *base_x;      /* base sensor readings */
//         int                   *base_y;
//         int                   *cur_x;       /* current sensor readings */
//         int                   *cur_y;
//         int                   *pressure_x;  /* computed pressures */
//         int                   *pressure_y;
//         struct timeval         sc_reap_time;
//         struct timeval         sc_reap_ctime; /*ctime of siblings to be reaped*/

//         u_int                  sc_idlecount; /* preceding idle interrupts */
// #define ATP_IDLENESS_THRESHOLD 10
};

static const struct wsp_dev_params wsp_dev_params[WELLSPRING_PRODUCT_MAX] = {
	[WELLSPRING1] = {
		.caps       = 0,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE1,
		.finger_data_offset  = WSP_TYPE1_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE1_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 256
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4824, 5342
		},
		.y = {
			SN_COORD, -172, 5820
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING2] = {
		.caps       = 0,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE1,
		.finger_data_offset  = WSP_TYPE1_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE1_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 256
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4824, 4824
		},
		.y = {
			SN_COORD, -172, 4290
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING3] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4460, 5166
		},
		.y = {
			SN_COORD, -75, 6700
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING4] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4620, 5140
		},
		.y = {
			SN_COORD, -150, 6600
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING4A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4616, 5112
		},
		.y = {
			SN_COORD, -142, 5234
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING5] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4415, 5050
		},
		.y = {
			SN_COORD, -55, 6680
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING6] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4620, 5140
		},
		.y = {
			SN_COORD, -150, 6600
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING5A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4750, 5280
		},
		.y = {
			SN_COORD, -150, 6730
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING6A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4620, 5140
		},
		.y = {
			SN_COORD, -150, 6600
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING7] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4750, 5280
		},
		.y = {
			SN_COORD, -150, 6730
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING7A] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE2,
		.finger_data_offset  = WSP_TYPE2_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE2_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4750, 5280
		},
		.y = {
			SN_COORD, -150, 6730
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
		},
	},
	[WELLSPRING8] = {
		.caps       = HAS_INTEGRATED_BUTTON,
		.bt_datalen = sizeof(struct bt_data),
		.tp_type    = TYPE3,
		.finger_data_offset  = WSP_TYPE3_FINGER_DATA_OFFSET,
		.data_len   = WSP_TYPE3_FINGER_DATA_OFFSET + WSP_SIZEOF_FINGER_DATA,
		.p = {
			SN_PRESSURE, 0, 300
		},
		.w = {
			SN_WIDTH, 0, 2048
		},
		.x = {
			SN_COORD, -4620, 5140
		},
		.y = {
			SN_COORD, -150, 6600
		},
		.o = {
			SN_ORIENT, -WSP_MAX_FINGER_ORIENTATION, WSP_MAX_FINGER_ORIENTATION
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

static void atp_interpret_wellspring_data(struct atp_softc *sc, unsigned len);
static boolean_t atp_update_wellspring_strokes(struct atp_softc *sc,
    const struct wsp_finger *fingerp, u_int n_fingers);

sensor_data_interpreter_t atp_sensor_data_interpreters[TRACKPAD_FAMILY_MAX] = {
	[TRACKPAD_FAMILY_WELLSPRING] = atp_interpret_wellspring_data,
};


#define MODE_LENGTH 8 /* num bytes holding the device mode */

/* TODO: rename to atp_xfer_config */
static struct usb_config atp_config[ATP_N_TRANSFER] = {
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

	sc->sc_state &= ~(ATP_ENABLED | ATP_VALID);
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

	// if (params->n_xsensors != 0) {
	//         sc->base_x = malloc(params->n_xsensors * sizeof(*(sc->base_x)),
	//             M_USB, M_WAITOK);
	//         if (sc->base_x == NULL) {
	//                 DPRINTF("mem for sc->base_x\n");
	//                 return (ENXIO);
	//         }

	//         sc->cur_x = malloc(params->n_xsensors * sizeof(*(sc->cur_x)),
	//             M_USB, M_WAITOK);
	//         if (sc->cur_x == NULL) {
	//                 DPRINTF("mem for sc->cur_x\n");
	//                 return (ENXIO);
	//         }

	//         sc->pressure_x =
	//                 malloc(params->n_xsensors * sizeof(*(sc->pressure_x)),
	//                     M_USB, M_WAITOK);
	//         if (sc->pressure_x == NULL) {
	//                 DPRINTF("mem. for pressure_x\n");
	//                 return (ENXIO);
	//         }
	// }

	// if (params->n_ysensors != 0) {
	//         sc->base_y = malloc(params->n_ysensors * sizeof(*(sc->base_y)),
	//             M_USB, M_WAITOK);
	//         if (sc->base_y == NULL) {
	//                 DPRINTF("mem for base_y\n");
	//                 return (ENXIO);
	//         }

	//         sc->cur_y = malloc(params->n_ysensors * sizeof(*(sc->cur_y)),
	//             M_USB, M_WAITOK);
	//         if (sc->cur_y == NULL) {
	//                 DPRINTF("mem for cur_y\n");
	//                 return (ENXIO);
	//         }

	//         sc->pressure_y =
	//                 malloc(params->n_ysensors * sizeof(*(sc->pressure_y)),
	//                     M_USB, M_WAITOK);
	//         if (sc->pressure_y == NULL) {
	//                 DPRINTF("mem. for pressure_y\n");
	//                 return (ENXIO);
	//         }
	// }

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
	// if (params->n_xsensors != 0) {
	//         if (sc->base_x != NULL) {
	//                 free(sc->base_x, M_USB);
	//                 sc->base_x = NULL;
	//         }

	//         if (sc->cur_x != NULL) {
	//                 free(sc->cur_x, M_USB);
	//                 sc->cur_x = NULL;
	//         }

	//         if (sc->pressure_x != NULL) {
	//                 free(sc->pressure_x, M_USB);
	//                 sc->pressure_x = NULL;
	//         }
	// }
	// if (params->n_ysensors != 0) {
	//         if (sc->base_y != NULL) {
	//                 free(sc->base_y, M_USB);
	//                 sc->base_y = NULL;
	//         }

	//         if (sc->cur_y != NULL) {
	//                 free(sc->cur_y, M_USB);
	//                 sc->cur_y = NULL;
	//         }

	//         if (sc->pressure_y != NULL) {
	//                 free(sc->pressure_y, M_USB);
	//                 sc->pressure_y = NULL;
	//         }
	// }
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
	atp_config[ATP_INTR_DT].bufsize = sc->sc_params->data_len;

	err = usbd_transfer_setup(uaa->device,
	    &uaa->info.bIfaceIndex, sc->sc_xfer, atp_config,
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

//         sc->sc_left_margin  = atp_mickeys_scale_factor;
//         sc->sc_right_margin = (sc->sc_params->n_xsensors - 1) *
//                 atp_mickeys_scale_factor;

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
    // uint8_t                status_bits;
    // atp_pspan  pspans_x[ATP_MAX_PSPANS_PER_AXIS];
    // atp_pspan  pspans_y[ATP_MAX_PSPANS_PER_AXIS];
    // u_int      n_xpspans = 0, n_ypspans = 0;
    // u_int      reaped_xlocs[ATP_MAX_STROKES];
    // u_int      tap_fingers = 0;

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

		(sc->sensor_data_interpreter)(sc, len);

    //     /* Interpret sensor data */
    //     atp_interpret_sensor_data(sc->sensor_data,
    //         sc->sc_params->n_xsensors, X, sc->cur_x,
    //         sc->sc_params->prot);
    //     atp_interpret_sensor_data(sc->sensor_data,
    //         sc->sc_params->n_ysensors, Y,  sc->cur_y,
    //         sc->sc_params->prot);

    //     /*
    //      * If this is the initial update (from an untouched
    //      * pad), we should set the base values for the sensor
    //      * data; deltas with respect to these base values can
    //      * be used as pressure readings subsequently.
    //      */
    //     status_bits = sc->sensor_data[sc->sc_params->data_len - 1];
    //     if ((sc->sc_params->prot == ATP_PROT_GEYSER3 &&
    //         (status_bits & ATP_STATUS_BASE_UPDATE)) ||
    //         !(sc->sc_state & ATP_VALID)) {
    //         memcpy(sc->base_x, sc->cur_x,
    //             sc->sc_params->n_xsensors * sizeof(*(sc->base_x)));
    //         memcpy(sc->base_y, sc->cur_y,
    //             sc->sc_params->n_ysensors * sizeof(*(sc->base_y)));
    //         sc->sc_state |= ATP_VALID;
    //         goto tr_setup;
    //     }

    //     /* Get pressure readings and detect p-spans for both axes. */
    //     atp_get_pressures(sc->pressure_x, sc->cur_x, sc->base_x,
    //         sc->sc_params->n_xsensors);
    //     atp_detect_pspans(sc->pressure_x, sc->sc_params->n_xsensors,
    //         ATP_MAX_PSPANS_PER_AXIS,
    //         pspans_x, &n_xpspans);
    //     atp_get_pressures(sc->pressure_y, sc->cur_y, sc->base_y,
    //         sc->sc_params->n_ysensors);
    //     atp_detect_pspans(sc->pressure_y, sc->sc_params->n_ysensors,
    //         ATP_MAX_PSPANS_PER_AXIS,
    //         pspans_y, &n_ypspans);

    //     /* Update strokes with new pspans to detect movements. */
    //     sc->sc_status.flags &= ~MOUSE_POSCHANGED;
    //     if (atp_update_strokes(sc,
    //         pspans_x, n_xpspans,
    //         pspans_y, n_ypspans))
    //         sc->sc_status.flags |= MOUSE_POSCHANGED;

    //     /* Reap zombies if it is time. */
    //     if (sc->sc_state & ATP_ZOMBIES_EXIST) {
    //         struct timeval now;

    //         getmicrotime(&now);
    //         if (timevalcmp(&now, &sc->sc_reap_time, >=))
    //             atp_reap_zombies(sc, &tap_fingers,
    //                 reaped_xlocs);
    //     }

    //     sc->sc_status.flags &= ~MOUSE_STDBUTTONSCHANGED;
    //     sc->sc_status.obutton = sc->sc_status.button;

    //     /* Get the state of the physical buttton. */
    //     sc->sc_status.button = (status_bits & ATP_STATUS_BUTTON) ?
    //         MOUSE_BUTTON1DOWN : 0;
    //     if (sc->sc_status.button != 0) {
    //         /* Reset DOUBLE_TAP_N_DRAG if the button is pressed. */
    //         sc->sc_state &= ~ATP_DOUBLE_TAP_DRAG;
    //     } else if (sc->sc_state & ATP_DOUBLE_TAP_DRAG) {
    //         /* Assume a button-press with DOUBLE_TAP_N_DRAG. */
    //         sc->sc_status.button = MOUSE_BUTTON1DOWN;
    //     }

    //     sc->sc_status.flags |=
    //         sc->sc_status.button ^ sc->sc_status.obutton;
    //     if (sc->sc_status.flags & MOUSE_STDBUTTONSCHANGED) {
    //         DPRINTFN(ATP_LLEVEL_INFO, "button %s\n",
    //             ((sc->sc_status.button & MOUSE_BUTTON1DOWN) ?
    //             "pressed" : "released"));
    //     } else if ((sc->sc_status.obutton == 0) &&
    //         (sc->sc_status.button == 0) &&
    //         (tap_fingers != 0)) {
    //         /* Ignore single-finger taps at the edges. */
    //         if ((tap_fingers == 1) &&
    //             ((reaped_xlocs[0] <= sc->sc_left_margin) ||
    //             (reaped_xlocs[0] > sc->sc_right_margin))) {
    //             tap_fingers = 0;
    //         }
    //         DPRINTFN(ATP_LLEVEL_INFO,
    //             "tap_fingers: %u\n", tap_fingers);
    //     }

    //     if (sc->sc_status.flags &
    //         (MOUSE_POSCHANGED | MOUSE_STDBUTTONSCHANGED)) {
    //         int   dx, dy;
    //         u_int n_movements;

    //         dx = 0, dy = 0, n_movements = 0;
    //         for (u_int i = 0; i < sc->sc_n_strokes; i++) {
    //             atp_stroke *stroke = &sc->sc_strokes[i];

    //             if ((stroke->components[X].movement) ||
    //                 (stroke->components[Y].movement)) {
    //                 dx += stroke->components[X].movement;
    //                 dy += stroke->components[Y].movement;
    //                 n_movements++;
    //             }
    //         }
    //         /*
    //          * Disregard movement if multiple
    //          * strokes record motion.
    //          */
    //         if (n_movements != 1)
    //             dx = 0, dy = 0;

    //         sc->sc_status.dx += dx;
    //         sc->sc_status.dy += dy;
    //         atp_add_to_queue(sc, dx, -dy, sc->sc_status.button);
    //     }

    //     if (tap_fingers != 0) {
    //         /* Add a pair of events (button-down and button-up). */
    //         switch (tap_fingers) {
    //         case 1: atp_add_to_queue(sc, 0, 0, MOUSE_BUTTON1DOWN);
    //             break;
    //         case 2: atp_add_to_queue(sc, 0, 0, MOUSE_BUTTON2DOWN);
    //             break;
    //         case 3: atp_add_to_queue(sc, 0, 0, MOUSE_BUTTON3DOWN);
    //             break;
    //         default: break;/* handle taps of only up to 3 fingers */
    //         }
    //         atp_add_to_queue(sc, 0, 0, 0); /* button release */
    //     }

    //     /*
    //      * The device continues to trigger interrupts at a
    //      * fast rate even after touchpad activity has
    //      * stopped. Upon detecting that the device has
    //      * remained idle beyond a threshold, we reinitialize
    //      * it to silence the interrupts.
    //      */
    //     if ((sc->sc_status.flags  == 0) &&
    //         (sc->sc_n_strokes     == 0) &&
    //         (sc->sc_status.button == 0)) {
    //         sc->sc_idlecount++;
    //         if (sc->sc_idlecount >= ATP_IDLENESS_THRESHOLD) {
    //             DPRINTFN(ATP_LLEVEL_INFO, "idle\n");

    //             /*
    //              * Use the last frame before we go idle for
    //              * calibration on pads which do not send
    //              * calibration frames.
    //              */
    //             if (sc->sc_params->prot < ATP_PROT_GEYSER3) {
    //                 memcpy(sc->base_x, sc->cur_x,
    //                     sc->sc_params->n_xsensors *
    //                     sizeof(*(sc->base_x)));
    //                 memcpy(sc->base_y, sc->cur_y,
    //                     sc->sc_params->n_ysensors *
    //                     sizeof(*(sc->base_y)));
    //             }

    //             sc->sc_idlecount = 0;
    //             usbd_transfer_start(sc->sc_xfer[ATP_RESET]);
    //         }
    //     } else {
    //         sc->sc_idlecount = 0;
    //     }

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
	// const struct wsp_sensor_data_header *hdr;
	const struct wsp_dev_params          *params;

	// hdr = (const struct wsp_sensor_data_header *)(sc->sensor_data);
	params = sc->sc_params;

	if ((data_len < params->finger_data_offset) ||
	    ((data_len - params->finger_data_offset) % WSP_SIZEOF_FINGER_STRUCT) != 0)
		return;

	unsigned n_fingers = (data_len - params->finger_data_offset) /
	    WSP_SIZEOF_FINGER_STRUCT;
	struct wsp_finger *fingerp =
	    (struct wsp_finger *)(sc->sensor_data + params->finger_data_offset);

	printf("%u\n", n_fingers);

	unsigned i;
	for (i = 0; i != n_fingers; i++) {
		/* swap endianness, if any */
		fingerp[i].origin = le16toh((uint16_t)fingerp[i].origin);
		fingerp[i].abs_x = le16toh((uint16_t)fingerp[i].abs_x);
		fingerp[i].abs_y = le16toh((uint16_t)fingerp[i].abs_y);
	//         f[i].rel_x = le16toh((uint16_t)f[i].rel_x);
	//         f[i].rel_y = le16toh((uint16_t)f[i].rel_y);
	//         f[i].tool_major = le16toh((uint16_t)f[i].tool_major);
	//         f[i].tool_minor = le16toh((uint16_t)f[i].tool_minor);
	//         f[i].orientation = le16toh((uint16_t)f[i].orientation);
	//         f[i].touch_major = le16toh((uint16_t)f[i].touch_major);
	//         f[i].touch_minor = le16toh((uint16_t)f[i].touch_minor);
	//         f[i].multi = le16toh((uint16_t)f[i].multi);
		// DPRINTFN(WSP_LLEVEL_INFO, "[%d]ibt=%d, taps=%d, u=%x, o=%4d, ax=%5d, ay=%5d, "
		// "rx=%5d, ry=%5d, tlmaj=%4d, tlmin=%4d, ot=%5d, tchmaj=%4d, tchmin=%4d, m=%4x\n",
		// i, ibt, ntouch, h->q2,
		// f[i].origin, f[i].abs_x, f[i].abs_y, f[i].rel_x, f[i].rel_y,
		// f[i].tool_major, f[i].tool_minor, f[i].orientation,
		// f[i].touch_major, f[i].touch_minor, f[i].multi);
		printf("[%d]o=%4d, ax=%5d, ay=%5d\n", i,
			fingerp[i].origin, fingerp[i].abs_x, fingerp[i].abs_y);

	//     sc->pos_x[i] = f[i].abs_x;
	//     sc->pos_y[i] = params->y.min + params->y.max - f[i].abs_y;
	//     sc->index[i] = &f[i];
	}

	if (atp_update_wellspring_strokes(sc, fingerp, n_fingers)) {
		printf("movement\n");
	}
}

/*
 * Update strokes by matching against current pressure-spans.
 * Return TRUE if any movement is detected.
 */
boolean_t
atp_update_wellspring_strokes(struct atp_softc *sc,
    const struct wsp_finger *fingerp, u_int n_fingers)
{
	return (false);
}

// static void
// atp_add_to_queue(struct atp_softc *sc, int dx, int dy, uint32_t buttons_in)
// {
// 	uint32_t buttons_out;
// 	uint8_t  buf[8];

// 	dx = imin(dx,  254); dx = imax(dx, -256);
// 	dy = imin(dy,  254); dy = imax(dy, -256);

// 	buttons_out = MOUSE_MSC_BUTTONS;
// 	if (buttons_in & MOUSE_BUTTON1DOWN)
// 		buttons_out &= ~MOUSE_MSC_BUTTON1UP;
// 	else if (buttons_in & MOUSE_BUTTON2DOWN)
// 		buttons_out &= ~MOUSE_MSC_BUTTON2UP;
// 	else if (buttons_in & MOUSE_BUTTON3DOWN)
// 		buttons_out &= ~MOUSE_MSC_BUTTON3UP;

// 	DPRINTFN(ATP_LLEVEL_INFO, "dx=%d, dy=%d, buttons=%x\n",
// 	    dx, dy, buttons_out);

// 	/* Encode the mouse data in standard format; refer to mouse(4) */
// 	buf[0] = sc->sc_mode.syncmask[1];
// 	buf[0] |= buttons_out;
// 	buf[1] = dx >> 1;
// 	buf[2] = dy >> 1;
// 	buf[3] = dx - (dx >> 1);
// 	buf[4] = dy - (dy >> 1);
// 	/* Encode extra bytes for level 1 */
// 	if (sc->sc_mode.level == 1) {
// 		buf[5] = 0;                    /* dz */
// 		buf[6] = 0;                    /* dz - (dz / 2) */
// 		buf[7] = MOUSE_SYS_EXTBUTTONS; /* Extra buttons all up. */
// 	}

// 	usb_fifo_put_data_linear(sc->sc_fifo.fp[USB_FIFO_RX], buf,
// 	    sc->sc_mode.packetsize, 1);
// }

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
