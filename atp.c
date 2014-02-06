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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <sys/selinfo.h>
#include <sys/poll.h>
#include <sys/sysctl.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbhid.h>

#include "usbdevs.h"

#define ATP_DRIVER_NAME "atp"

/* Device methods. */
static device_probe_t  atp_probe;
static device_attach_t atp_attach;
static device_detach_t atp_detach;
// static usb_callback_t  atp_intr;

struct atp_softc {
//         device_t               sc_dev;
//         struct usb_device     *sc_usb_device;
// #define MODE_LENGTH 8
//         char                   sc_mode_bytes[MODE_LENGTH]; /* device mode */
//         struct mtx             sc_mutex; /* for synchronization */
//         struct usb_xfer       *sc_xfer[ATP_N_TRANSFER];
//         struct usb_fifo_sc     sc_fifo;

//         struct atp_dev_params *sc_params;

//         mousehw_t              sc_hw;
//         mousemode_t            sc_mode;
//         u_int                  sc_pollrate;
//         mousestatus_t          sc_status;
//         u_int                  sc_state;
// #define ATP_ENABLED            0x01
// #define ATP_ZOMBIES_EXIST      0x02
// #define ATP_DOUBLE_TAP_DRAG    0x04
// #define ATP_VALID              0x08

//         u_int                  sc_left_margin;
//         u_int                  sc_right_margin;

//         atp_stroke             sc_strokes[ATP_MAX_STROKES];
//         u_int                  sc_n_strokes;

//         int8_t                *sensor_data; /* from interrupt packet */
//         int                   *base_x;      /* base sensor readings */
//         int                   *base_y;
//         int                   *cur_x;       /* current sensor readings */
//         int                   *cur_y;
//         int                   *pressure_x;  /* computed pressures */
//         int                   *pressure_y;

//         u_int                  sc_idlecount; /* preceding idle interrupts */
// #define ATP_IDLENESS_THRESHOLD 10

//         struct timeval         sc_reap_time;
//         struct timeval         sc_reap_ctime; /*ctime of siblings to be reaped*/
};

/* button data structure */
struct bt_data {
        uint8_t unknown1;       /* constant */
        uint8_t button;         /* left button */
        uint8_t rel_x;          /* relative x coordinate */
        uint8_t rel_y;          /* relative y coordinate */
} __packed;

/* trackpad header types */
enum tp_type {
        TYPE1,          /* plain trackpad */
        TYPE2,          /* button integrated in trackpad */
        TYPE3           /* additional header fields since June 2013 */
};

/* trackpad finger data offsets, le16-aligned */
#define FINGER_TYPE1        (13 * 2)
#define FINGER_TYPE2        (15 * 2)
#define FINGER_TYPE3        (19 * 2)

/* trackpad button data offsets */
#define BUTTON_TYPE2        15
#define BUTTON_TYPE3        23

/* list of device capability bits */
#define HAS_INTEGRATED_BUTTON   1

/* trackpad finger header - little endian */
struct tp_header {
        uint8_t  flag;
        uint8_t  sn0;
        uint16_t wFixed0;
        uint32_t dwSn1;
        uint32_t dwFixed1;
        uint16_t wLength;
        uint8_t  nfinger;
        uint8_t  ibt;
        int16_t  wUnknown[6];
        uint8_t  q1;
        uint8_t  q2;
} __packed;

/* trackpad finger structure - little endian */
struct tp_finger {
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
#define MAX_FINGERS             16
#define SIZEOF_FINGER           sizeof(struct tp_finger)
#define SIZEOF_ALL_FINGERS      (MAX_FINGERS * SIZEOF_FINGER)
#define MAX_FINGER_ORIENTATION  16384

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

/* TODO: name this enumeration */
enum {
    ATP_FLAG_WELLSPRING1,
    ATP_FLAG_WELLSPRING2,
    ATP_FLAG_WELLSPRING3,
    ATP_FLAG_WELLSPRING4,
    ATP_FLAG_WELLSPRING4A,
    ATP_FLAG_WELLSPRING5,
    ATP_FLAG_WELLSPRING6A,
    ATP_FLAG_WELLSPRING6,
    ATP_FLAG_WELLSPRING5A,
    ATP_FLAG_WELLSPRING7,
    ATP_FLAG_WELLSPRING7A,
    ATP_FLAG_WELLSPRING8,
    ATP_FLAG_MAX,
};

/* device-specific configuration */
struct atp_dev_params {
        uint8_t  caps;          /* device capability bitmask */
        uint16_t bt_datalen;    /* data length of the button interface */
        uint8_t  tp_type;       /* type of trackpad interface */
        uint8_t  tp_offset;     /* offset to trackpad finger data */
        uint16_t tp_datalen;    /* data length of the trackpad interface */
        struct wsp_param p;     /* finger pressure limits */
        struct wsp_param w;     /* finger width limits */
        struct wsp_param x;     /* horizontal limits */
        struct wsp_param y;     /* vertical limits */
        struct wsp_param o;     /* orientation limits */
};

static const struct atp_dev_params atp_dev_params[ATP_FLAG_MAX] = {
    [ATP_FLAG_WELLSPRING1] = {
        .caps       = 0,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE1,
        .tp_offset  = FINGER_TYPE1,
        .tp_datalen = FINGER_TYPE1 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING2] = {
        .caps       = 0,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE1,
        .tp_offset  = FINGER_TYPE1,
        .tp_datalen = FINGER_TYPE1 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING3] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING4] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING4A] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING5] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING6] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING5A] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING6A] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING7] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING7A] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE2,
        .tp_offset  = FINGER_TYPE2,
        .tp_datalen = FINGER_TYPE2 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
    [ATP_FLAG_WELLSPRING8] = {
        .caps       = HAS_INTEGRATED_BUTTON,
        .bt_datalen = sizeof(struct bt_data),
        .tp_type    = TYPE3,
        .tp_offset  = FINGER_TYPE3,
        .tp_datalen = FINGER_TYPE3 + SIZEOF_ALL_FINGERS,
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
            SN_ORIENT, -MAX_FINGER_ORIENTATION, MAX_FINGER_ORIENTATION
        },
    },
};

#define ATP_DEV(v,p,i) { USB_VPI(USB_VENDOR_##v, USB_PRODUCT_##v##_##p, i) }

static const STRUCT_USB_HOST_ID atp_devs[] = {
        /* MacbookAir1.1 */
        ATP_DEV(APPLE, WELLSPRING_ANSI, ATP_FLAG_WELLSPRING1),
        ATP_DEV(APPLE, WELLSPRING_ISO,  ATP_FLAG_WELLSPRING1),
        ATP_DEV(APPLE, WELLSPRING_JIS,  ATP_FLAG_WELLSPRING1),

        /* MacbookProPenryn, aka wellspring2 */
        ATP_DEV(APPLE, WELLSPRING2_ANSI, ATP_FLAG_WELLSPRING2),
        ATP_DEV(APPLE, WELLSPRING2_ISO,  ATP_FLAG_WELLSPRING2),
        ATP_DEV(APPLE, WELLSPRING2_JIS,  ATP_FLAG_WELLSPRING2),

        /* Macbook5,1 (unibody), aka wellspring3 */
        ATP_DEV(APPLE, WELLSPRING3_ANSI, ATP_FLAG_WELLSPRING3),
        ATP_DEV(APPLE, WELLSPRING3_ISO,  ATP_FLAG_WELLSPRING3),
        ATP_DEV(APPLE, WELLSPRING3_JIS,  ATP_FLAG_WELLSPRING3),

        /* MacbookAir3,2 (unibody), aka wellspring4 */
        ATP_DEV(APPLE, WELLSPRING4_ANSI, ATP_FLAG_WELLSPRING4),
        ATP_DEV(APPLE, WELLSPRING4_ISO,  ATP_FLAG_WELLSPRING4),
        ATP_DEV(APPLE, WELLSPRING4_JIS,  ATP_FLAG_WELLSPRING4),

        /* MacbookAir3,1 (unibody), aka wellspring4 */
        ATP_DEV(APPLE, WELLSPRING4A_ANSI, ATP_FLAG_WELLSPRING4A),
        ATP_DEV(APPLE, WELLSPRING4A_ISO,  ATP_FLAG_WELLSPRING4A),
        ATP_DEV(APPLE, WELLSPRING4A_JIS,  ATP_FLAG_WELLSPRING4A),

        /* Macbook8 (unibody, March 2011) */
        ATP_DEV(APPLE, WELLSPRING5_ANSI, ATP_FLAG_WELLSPRING5),
        ATP_DEV(APPLE, WELLSPRING5_ISO,  ATP_FLAG_WELLSPRING5),
        ATP_DEV(APPLE, WELLSPRING5_JIS,  ATP_FLAG_WELLSPRING5),

        /* MacbookAir4,1 (unibody, July 2011) */
        ATP_DEV(APPLE, WELLSPRING6A_ANSI, ATP_FLAG_WELLSPRING6A),
        ATP_DEV(APPLE, WELLSPRING6A_ISO,  ATP_FLAG_WELLSPRING6A),
        ATP_DEV(APPLE, WELLSPRING6A_JIS,  ATP_FLAG_WELLSPRING6A),

        /* MacbookAir4,2 (unibody, July 2011) */
        ATP_DEV(APPLE, WELLSPRING6_ANSI, ATP_FLAG_WELLSPRING6),
        ATP_DEV(APPLE, WELLSPRING6_ISO,  ATP_FLAG_WELLSPRING6),
        ATP_DEV(APPLE, WELLSPRING6_JIS,  ATP_FLAG_WELLSPRING6),

        /* Macbook8,2 (unibody) */
        ATP_DEV(APPLE, WELLSPRING5A_ANSI, ATP_FLAG_WELLSPRING5A),
        ATP_DEV(APPLE, WELLSPRING5A_ISO,  ATP_FLAG_WELLSPRING5A),
        ATP_DEV(APPLE, WELLSPRING5A_JIS,  ATP_FLAG_WELLSPRING5A),

        /* MacbookPro10,1 (unibody, June 2012) */
        /* MacbookPro11,? (unibody, June 2013) */
        ATP_DEV(APPLE, WELLSPRING7_ANSI, ATP_FLAG_WELLSPRING7),
        ATP_DEV(APPLE, WELLSPRING7_ISO,  ATP_FLAG_WELLSPRING7),
        ATP_DEV(APPLE, WELLSPRING7_JIS,  ATP_FLAG_WELLSPRING7),

        /* MacbookPro10,2 (unibody, October 2012) */
        ATP_DEV(APPLE, WELLSPRING7A_ANSI, ATP_FLAG_WELLSPRING7A),
        ATP_DEV(APPLE, WELLSPRING7A_ISO,  ATP_FLAG_WELLSPRING7A),
        ATP_DEV(APPLE, WELLSPRING7A_JIS,  ATP_FLAG_WELLSPRING7A),

        /* MacbookAir6,2 (unibody, June 2013) */
        ATP_DEV(APPLE, WELLSPRING8_ANSI, ATP_FLAG_WELLSPRING8),
        ATP_DEV(APPLE, WELLSPRING8_ISO,  ATP_FLAG_WELLSPRING8),
        ATP_DEV(APPLE, WELLSPRING8_JIS,  ATP_FLAG_WELLSPRING8),
};

static int
atp_probe(device_t self)
{
        struct usb_attach_arg *uaa = device_get_ivars(self);

        if (uaa->usb_mode != USB_MODE_HOST)
                return (ENXIO);

        if ((uaa->info.bInterfaceClass    != UICLASS_HID) ||
            (uaa->info.bInterfaceProtocol != UIPROTO_MOUSE))
                return (ENXIO);
        // if (uaa->info.bIfaceIndex != ATP_IFACE_INDEX)
        //         return (ENXIO);

        printf("passed initial checks\n");

        return (usbd_lookup_id_by_uaa(atp_devs, sizeof(atp_devs), uaa));
}

static int
atp_attach(device_t dev)
{
        printf("in atp_attach\n");
        return (ENXIO);
//         struct atp_softc      *sc = device_get_softc(dev);
//         struct usb_attach_arg *uaa = device_get_ivars(dev);
//         usb_error_t            err;

//         DPRINTFN(ATP_LLEVEL_INFO, "sc=%p\n", sc);

//         sc->sc_dev        = dev;
//         sc->sc_usb_device = uaa->device;

//         /*
//          * By default the touchpad behaves like an HID device, sending
//          * packets with reportID = 2. Such reports contain only
//          * limited information--they encode movement deltas and button
//          * events,--but do not include data from the pressure
//          * sensors. The device input mode can be switched from HID
//          * reports to raw sensor data using vendor-specific USB
//          * control commands; but first the mode must be read.
//          */
//         err = atp_req_get_report(sc->sc_usb_device, sc->sc_mode_bytes);
//         if (err != USB_ERR_NORMAL_COMPLETION) {
//                 DPRINTF("failed to read device mode (%d)\n", err);
//                 return (ENXIO);
//         }

//         if (atp_set_device_mode(dev, RAW_SENSOR_MODE) != 0) {
//                 DPRINTF("failed to set mode to 'RAW_SENSOR' (%d)\n", err);
//                 return (ENXIO);
//         }

//         mtx_init(&sc->sc_mutex, "atpmtx", NULL, MTX_DEF | MTX_RECURSE);

//         err = usbd_transfer_setup(uaa->device,
//             &uaa->info.bIfaceIndex, sc->sc_xfer, atp_config,
//             ATP_N_TRANSFER, sc, &sc->sc_mutex);

//         if (err) {
//                 DPRINTF("error=%s\n", usbd_errstr(err));
//                 goto detach;
//         }

//         if (usb_fifo_attach(sc->sc_usb_device, sc, &sc->sc_mutex,
//                 &atp_fifo_methods, &sc->sc_fifo,
//                 device_get_unit(dev), -1, uaa->info.bIfaceIndex,
//                 UID_ROOT, GID_OPERATOR, 0644)) {
//                 goto detach;
//         }

//         device_set_usb_desc(dev);

//         sc->sc_params           = &atp_dev_params[uaa->driver_info];

//         sc->sc_hw.buttons       = 3;
//         sc->sc_hw.iftype        = MOUSE_IF_USB;
//         sc->sc_hw.type          = MOUSE_PAD;
//         sc->sc_hw.model         = MOUSE_MODEL_GENERIC;
//         sc->sc_hw.hwid          = 0;
//         sc->sc_mode.protocol    = MOUSE_PROTO_MSC;
//         sc->sc_mode.rate        = -1;
//         sc->sc_mode.resolution  = MOUSE_RES_UNKNOWN;
//         sc->sc_mode.accelfactor = 0;
//         sc->sc_mode.level       = 0;
//         sc->sc_mode.packetsize  = MOUSE_MSC_PACKETSIZE;
//         sc->sc_mode.syncmask[0] = MOUSE_MSC_SYNCMASK;
//         sc->sc_mode.syncmask[1] = MOUSE_MSC_SYNC;

//         sc->sc_state            = 0;

//         sc->sc_left_margin  = atp_mickeys_scale_factor;
//         sc->sc_right_margin = (sc->sc_params->n_xsensors - 1) *
//                 atp_mickeys_scale_factor;

//         return (0);

// detach:
//         atp_detach(dev);
        return (ENOMEM);
}

static int
atp_detach(device_t dev)
{
        printf("in atp_detach\n");
        // struct atp_softc *sc;

        // sc = device_get_softc(dev);
        // if (sc->sc_state & ATP_ENABLED) {
        //         mtx_lock(&sc->sc_mutex);
        //         atp_disable(sc);
        //         mtx_unlock(&sc->sc_mutex);
        // }

        // usb_fifo_detach(&sc->sc_fifo);

        // usbd_transfer_unsetup(sc->sc_xfer, ATP_N_TRANSFER);

        // mtx_destroy(&sc->sc_mutex);

        return (0);
}

// void
// atp_intr(struct usb_xfer *xfer, usb_error_t error)
// {
// }


/*
 * Load handler that deals with the loading and unloading of a KLD.
 */

// static int
// atp_loader(struct module *m, int what, void *arg)
// {
//         int err = 0;

//         switch (what) {
//         case MOD_LOAD:                /* kldload */
//                 uprintf("Skeleton KLD loaded.\n");
//                 break;
//         case MOD_UNLOAD:
//                 uprintf("Skeleton KLD unloaded.\n");
//                 break;
//         default:
//                 err = EOPNOTSUPP;
//                 break;
//         }
//         return(err);
// }

static device_method_t atp_methods[] = {
        /* Device interface */
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

static devclass_t atp_devclass;

DRIVER_MODULE(atp, uhub, atp_driver, atp_devclass, NULL /*atp_loader*/ /* evh */, 0);
MODULE_DEPEND(atp, usb, 1, 1, 1);
MODULE_VERSION(atp, 1);
