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
// static device_probe_t  atp_probe;
// static device_attach_t atp_attach;
// static device_detach_t atp_detach;
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


// static int
// atp_probe(device_t self)
// {
        // struct usb_attach_arg *uaa = device_get_ivars(self);

        // if (uaa->usb_mode != USB_MODE_HOST)
        //         return (ENXIO);

        // if ((uaa->info.bInterfaceClass != UICLASS_HID) ||
        //     (uaa->info.bInterfaceProtocol != UIPROTO_MOUSE))
        //         return (ENXIO);

        // return (usbd_lookup_id_by_uaa(atp_devs, sizeof(atp_devs), uaa));
//         return (0);
// }

// static int
// atp_attach(device_t dev)
// {
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
        // return (ENOMEM);
// }

// static int
// atp_detach(device_t dev)
// {
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

//         return (0);
// }

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
        // DEVMETHOD(device_probe,  atp_probe),
        // DEVMETHOD(device_attach, atp_attach),
        // DEVMETHOD(device_detach, atp_detach),
        DEVMETHOD_END
};

static driver_t atp_driver = {
        .name = ATP_DRIVER_NAME,
        .methods = atp_methods,
        .size = sizeof(struct atp_softc)
};

static devclass_t atp_devclass;

DRIVER_MODULE(atp, uhub, atp_driver, atp_devclass, NULL, 0);
MODULE_DEPEND(atp, usb, 1, 1, 1);
MODULE_VERSION(atp, 1);
