/*
 * Rohde & Schwarz USB NRP power meter series kernel module driver
 *
 * Version: 4.2.3
 *
 * Copyright (c) 2001 - 2017 by Rohde & Scharz GmbH & Co. KG
 *
 * This driver supports the RF Power Meter R&S NRP Sensors. These
 * sensors are intelligent standalone instruments that communicate via USB
 * and support the following measurements:
 *
 * - Dynamic range: up to 90 dB (sensor dependent)
 * - Level range: -67 dBm to +45 dBm (sensor dependent)
 * - Frequency range: DC to 67 GHz (sensor dependent)
 * - Measurement speed: 1500 measurements per second in the buffered mode
 * - Linearity uncertainty: 0.007 dB
 * - Precise average power measurements irrespective of modulation and bandwidth
 * - Flexible measurements on up to 128 time slots per power sensor
 * - S parameter correction of components between sensor and test object
 *
 * History:
 *
 * 2014-11-07 - v4.2 added support for new NRPxxS series
 * 2012-05-18 - v4.0 revamp for kernel inclusion
 * 2007-12-07 - v3.0 Rewritten, multi urbs for read and write
 * 2007-05-15 - v2.0 Ported the driver to kernel 2.6, mostly rewritten
 * 2001-05-01 - v0.1 first version
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */

/*
 * Internal format of the NRP USB messages:
 *
 *  Byte 0:	Signature
 *  Byte 1:	Error Code
 *  Byte 2/3:	Array Index
 *		or State (Byte 2)
 *		or Group Number (Byte 2) / Param Number (Byte 3)
 *  Byte 4-15:	Data depening on signature type (Byte 0):
 *		floats, bit fields and integer are 32 bit
 *
 * Signature types:
 *  'E':	single float value
 *  'e':	single value
 *  'A':	float array
 *  'P':	auxiliary or peak float array
 *  'a':	interim float array
 *  'p':	interim auxiliary or peak float array
 *  'F':	feature bit field
 *  'U':	float parameter (32 bit)
 *  'V':	bit field parameter (32 bit)
 *  'W':	integer parameter (32 bit)
 *  'T':	string data
 *  'R':	receipt data (string or binary data)
 *  'X':	internal error
 *  'Z':	current state (Byte 2)
 *  'z':	life sign package
 *  'L':	float value (32 bit)
 *  'M':	bit field value (32 bit)
 *  'N':	long value (32 bit)
 *  'B':	binary data
 *
 * State values:
 *   0:		trigger idle
 *   1:		trigger reserved
 *   2:		wait for trigger
 *   3:		trigger measuring
 *
 * Communication in direction from host to the device are mostly like SCPI.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/kref.h>
#include <linux/bitops.h>
#include <linux/mutex.h>

#ifdef __i386__
#include <linux/dmi.h>
#endif

#include "nrpzmodule.h"

/* Version Information */
#define DRIVER_VERSION	"4.2.3"

/* Get a minor range for your devices from the usb maintainer */
#define NRPZ_MINOR_BASE		0

#define USB_RS_VENDOR_ID		0x0aad
#define USB_NRP_PRODUCT_ID		0x0002
#define USB_NRPZ21_PRODUCT_ID		0x0003
#define USB_NRPFU_PRODUCT_ID		0x0004
#define USB_FSHZ1_PRODUCT_ID		0x000b
#define USB_NRPZ11_PRODUCT_ID		0x000c
#define USB_NRPZ22_PRODUCT_ID		0x0013
#define USB_NRPZ23_PRODUCT_ID		0x0014
#define USB_NRPZ24_PRODUCT_ID		0x0015
#define USB_NRPZ51_PRODUCT_ID		0x0016
#define USB_NRPZ52_PRODUCT_ID		0x0017
#define USB_NRPZ55_PRODUCT_ID		0x0018
#define USB_NRPZ56_PRODUCT_ID		0x0019
#define USB_FSHZ18_PRODUCT_ID		0x001a
#define USB_NRPZ91_PRODUCT_ID		0x0021
#define USB_NRPZ81_PRODUCT_ID		0x0023
#define USB_NRPZ31_PRODUCT_ID		0x002c
#define USB_NRPZ37_PRODUCT_ID		0x002d
#define USB_NRPZ96_PRODUCT_ID		0x002e
#define USB_NRPZ27_PRODUCT_ID		0x002f
#define USB_NRPZ28_PRODUCT_ID		0x0051
#define USB_NRPZ98_PRODUCT_ID		0x0052
#define USB_NRPZ92_PRODUCT_ID		0x0062
#define USB_NRPZ57_PRODUCT_ID		0x0070
#define USB_NRPZ85_PRODUCT_ID		0x0083
#define USB_NRPC40_PRODUCT_ID		0x008f
#define USB_NRPC50_PRODUCT_ID		0x0090
#define USB_NRPZ86_PRODUCT_ID		0x0095
#define USB_NRPZ41_PRODUCT_ID		0x0096
#define USB_NRPZ61_PRODUCT_ID		0x0097
#define USB_NRPZ71_PRODUCT_ID		0x0098
#define USB_NRPZ32_PRODUCT_ID		0x009a
#define USB_NRPZ211_PRODUCT_ID		0x00a6
#define USB_NRPZ221_PRODUCT_ID		0x00a7
#define USB_NRPZ58_PRODUCT_ID		0x00a8
#define USB_NRPC33_PRODUCT_ID		0x00b6
#define USB_NRPC18_PRODUCT_ID		0x00bf

#define USB_NRP8S_PRODUCT_ID		0x00e2
#define USB_NRPC18_B1_PRODUCT_ID	0x00c1
#define USB_NRPC33_B1_PRODUCT_ID	0x00c2
#define USB_NRPC40_B1_PRODUCT_ID	0x00c3
#define USB_NRPC50_B1_PRODUCT_ID	0x00c4

#define USB_NRP8SN_PRODUCT_ID     0x0137
#define USB_NRP18S_PRODUCT_ID     0x0138
#define USB_NRP18SN_PRODUCT_ID    0x0139
#define USB_NRP18W_PRODUCT_ID     0x0143
#define USB_NRP18WN_PRODUCT_ID    0x0144
#define USB_NRP33S_PRODUCT_ID     0x0145
#define USB_NRP33SN_PRODUCT_ID    0x0146
#define USB_NRP18S10_PRODUCT_ID   0x0148
#define USB_NRP18SN10_PRODUCT_ID  0x0149
#define USB_NRP18S20_PRODUCT_ID   0x014A
#define USB_NRP18SN20_PRODUCT_ID  0x014B
#define USB_NRP18S25_PRODUCT_ID   0x014C
#define USB_NRP18SN25_PRODUCT_ID  0x014D
#define USB_NRP18A_PRODUCT_ID     0x014E
#define USB_NRP18AN_PRODUCT_ID    0x014F
#define USB_NRP18T_PRODUCT_ID     0x0150
#define USB_NRP18TN_PRODUCT_ID    0x0151
#define USB_NRP33T_PRODUCT_ID     0x0152
#define USB_NRP33TN_PRODUCT_ID    0x0153
#define USB_NRP40T_PRODUCT_ID     0x0154
#define USB_NRP40TN_PRODUCT_ID    0x0155
#define USB_NRP50T_PRODUCT_ID     0x0156
#define USB_NRP50TN_PRODUCT_ID    0x0157
#define USB_NRP67T_PRODUCT_ID     0x0158
#define USB_NRP67TN_PRODUCT_ID    0x0159
#define USB_NRP110T_PRODUCT_ID    0x015A
#define USB_NRQ6_PRODUCT_ID       0x015B

/* table of devices that work with this driver */
static struct usb_device_id nrpz_table[] = {
	{
		.idVendor = USB_RS_VENDOR_ID,
		.bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
		.bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
		.bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
		.match_flags = USB_DEVICE_ID_MATCH_VENDOR|USB_DEVICE_ID_MATCH_DEV_CLASS|USB_DEVICE_ID_MATCH_INT_CLASS|USB_DEVICE_ID_MATCH_INT_SUBCLASS,
	},
	{
		.idVendor = USB_RS_VENDOR_ID,
		.bDeviceClass = USB_CLASS_VENDOR_SPEC,
		.bInterfaceClass    = USB_CLASS_VENDOR_SPEC,
		.bInterfaceSubClass = USB_CLASS_VENDOR_SPEC,
		.match_flags = USB_DEVICE_ID_MATCH_VENDOR|USB_DEVICE_ID_MATCH_DEV_CLASS|USB_DEVICE_ID_MATCH_INT_CLASS|USB_DEVICE_ID_MATCH_INT_SUBCLASS,
	},
	{ },
};
MODULE_DEVICE_TABLE(usb, nrpz_table);

static u16 known_products[] = {
	USB_NRP_PRODUCT_ID,
	USB_NRPZ21_PRODUCT_ID,
	USB_NRPFU_PRODUCT_ID,
	USB_FSHZ1_PRODUCT_ID,
	USB_NRPZ11_PRODUCT_ID,
	USB_NRPZ22_PRODUCT_ID,
	USB_NRPZ23_PRODUCT_ID,
	USB_NRPZ24_PRODUCT_ID,
	USB_NRPZ51_PRODUCT_ID,
	USB_NRPZ52_PRODUCT_ID,
	USB_NRPZ55_PRODUCT_ID,
	USB_NRPZ56_PRODUCT_ID,
	USB_NRPZ57_PRODUCT_ID,
	USB_FSHZ18_PRODUCT_ID,
	USB_NRPZ91_PRODUCT_ID,
	USB_NRPZ81_PRODUCT_ID,
	USB_NRPZ31_PRODUCT_ID,
	USB_NRPZ37_PRODUCT_ID,
	USB_NRPZ96_PRODUCT_ID,
	USB_NRPZ27_PRODUCT_ID,
	USB_NRPZ28_PRODUCT_ID,
	USB_NRPZ98_PRODUCT_ID,
	USB_NRPZ92_PRODUCT_ID,
	USB_NRPZ85_PRODUCT_ID,
	USB_NRPC40_PRODUCT_ID,
	USB_NRPC50_PRODUCT_ID,
	USB_NRPZ86_PRODUCT_ID,
	USB_NRPZ41_PRODUCT_ID,
	USB_NRPZ61_PRODUCT_ID,
	USB_NRPZ71_PRODUCT_ID,
	USB_NRPZ32_PRODUCT_ID,
	USB_NRPZ211_PRODUCT_ID,
	USB_NRPZ221_PRODUCT_ID,
	USB_NRPZ58_PRODUCT_ID,
	USB_NRPC33_PRODUCT_ID,
	USB_NRPC18_PRODUCT_ID,
	USB_NRP8S_PRODUCT_ID,
	USB_NRPC18_B1_PRODUCT_ID,
	USB_NRPC33_B1_PRODUCT_ID,
	USB_NRPC40_B1_PRODUCT_ID,
	USB_NRPC50_B1_PRODUCT_ID,
	USB_NRP8SN_PRODUCT_ID,
	USB_NRP18S_PRODUCT_ID,
	USB_NRP18SN_PRODUCT_ID,
	USB_NRP18W_PRODUCT_ID,
	USB_NRP18WN_PRODUCT_ID,
	USB_NRP33S_PRODUCT_ID,
	USB_NRP33SN_PRODUCT_ID,
	USB_NRP18S10_PRODUCT_ID,
	USB_NRP18SN10_PRODUCT_ID,
	USB_NRP18S20_PRODUCT_ID,
	USB_NRP18SN20_PRODUCT_ID,
	USB_NRP18S25_PRODUCT_ID,
	USB_NRP18SN25_PRODUCT_ID,
	USB_NRP18A_PRODUCT_ID,
	USB_NRP18AN_PRODUCT_ID,
	USB_NRP18T_PRODUCT_ID,
	USB_NRP18TN_PRODUCT_ID,
	USB_NRP33T_PRODUCT_ID,
	USB_NRP33TN_PRODUCT_ID,
	USB_NRP40T_PRODUCT_ID,
	USB_NRP40TN_PRODUCT_ID,
	USB_NRP50T_PRODUCT_ID,
	USB_NRP50TN_PRODUCT_ID,
	USB_NRP67T_PRODUCT_ID,
	USB_NRP67TN_PRODUCT_ID,
	USB_NRP110T_PRODUCT_ID,
	USB_NRQ6_PRODUCT_ID,
};

struct usb_nrpz {
	struct usb_device *udev;	/* usb device pointer */
	struct usb_interface *intf;	/* usb interface */

	size_t in_size;			/* size of the receive buffer */
	size_t out_size;		/* size of the send buffer */
	__u8 in_epAddr;			/* address of the bulk in endpoint */
	__u8 out_epAddr;		/* address of the bulk out endpoint */
	bool in_use;			/* in use flag */

	struct kref kref;
	wait_queue_head_t wq;

	struct usb_anchor out_running;	/* list of in use output buffers */
	struct list_head out_avail;	/* list of available output buffers */
	spinlock_t write_lock;		/* spinlock for transmit list */
	struct mutex write_mutex;	/* exclusive write data semaphore */

	struct usb_anchor in_running;	/* list of in use input buffers */
	struct list_head in_avail;	/* list of available input buffers */
	spinlock_t read_lock;		/* spinlock for receive list */
	struct mutex read_mutex;	/* exclusive read data semaphore */

	struct urb out_urbs[64];	/* array of urb's for output */
	struct urb in_urbs[64];		/* array of urb's for input */
#ifdef SMA_K
	u8 id;				/* sensor id number */
#endif
};

/* forward declaration */
static struct usb_driver nrpz_driver;

#ifdef SMA_K
#define	UPDATE_DELAY	100
#define	BLINK_DELAY	200
#define	MS2HZ(x)	((x+(1000/HZ)-1)/(1000/HZ))	/* calculate the hz value from millisec. */

spinlock_t gpo_lock;	/* spinlock for gpo access */

static int do_superio;
static u8 gpo_action;
static u8 gpo_val;
static u8 gpo_running;
static const u8 gpo_bit[5] = { 0x00, 0x20, 0x10, 0x80, 0x40 };

static struct delayed_work	deferred_work;

static void __init superio_probe(unsigned short cfg_base)
{
	if (inb(cfg_base) != 0xff)
		return;

	/* Leave configuration */
	outb(0x87, cfg_base);
	if (inb(cfg_base) != 0xff)
		return;

	outb(0x87, cfg_base);

	outb(0x20, cfg_base);
	if (inb(cfg_base + 1) != 0x88)
		return;

	outb(0x21, cfg_base);
	if (inb(cfg_base + 1) != 0x68)
		return;

	do_superio = 1;
}

static void set_gpo(u8 val)
{
	u8 i;

	if (!do_superio)
		return;

	/* Enter Extended Function Mode */
	outb(0x87, 0x4e);
	outb(0x87, 0x4e);

	/* Select logic device 7 */
	outb(0x7, 0x4e);
	outb(0x7, 0x4f);

	/* Assign pin 121-128 bit(7...2) to be 1 */
	outb(0x2f, 0x4e);
	i = inb(0x4f);
	i |= 0xfc;
	outb(0x2f, 0x4e);
	outb(i, 0x4f);

	/* Active logic device 7 */
	outb(0x30, 0x4e);
	outb(0x1, 0x4f);

	/* Select I/O mode */
	outb(0xf0, 0x4e);
	i = inb(0x4f);
	i &= 0x3;
	outb(0xf0, 0x4e);
	outb(i, 0x4f);

	/* Select inversion mode */
	outb(0xf2, 0x4e);
	i = inb(0x4f);
	i &= 0x3;
	outb(0xf2, 0x4e);
	outb(i, 0x4f);

	/* Value of Inversion Register:
	   Only high nibble is available for this function.
	   When set to a 1, the incoming/outgoing port value is inverted.
	   When set to a 0, the incoming/outgoing port value is the same as in Data Register.

	    Value of  I/O Selection Register:
	    Only high nibble is available for this function.
	    When set to a 1, respective GPIO port is programmed as an input port.
	    When set to a 0, respective GPIO port is programmed as an output port.

	    Value of  Output Data / Input Data:
	    Only high nibble is available for this function.
	    If a port is assigned to be an output port, then its respective bit can be read/written.
	    If a port is assigned to be an input port, then its respective bit can be read only.
	*/

 	outb(0xf1, 0x4e); /* GP10, LEDs */
	outb(val, 0x4f);
	outb(0xaa, 0x4e); /* Exit extended function mode */

}

static void led_deferred_work(struct work_struct *work)
{
	spin_lock_irq(&gpo_lock);
	if (!gpo_running) {
		if (gpo_action) {
			gpo_val &= ~gpo_action;
			gpo_running = gpo_action;
			set_gpo(gpo_val);
			schedule_delayed_work(&deferred_work, MS2HZ(BLINK_DELAY));
		}
	}
	else {
		gpo_action &= ~gpo_running;
		gpo_val |= gpo_running;
		gpo_running = 0;
		set_gpo(gpo_val);
		if (gpo_action)
			schedule_delayed_work(&deferred_work, MS2HZ(UPDATE_DELAY));
	}
	spin_unlock_irq(&gpo_lock);
}

void set_action(struct usb_nrpz *dev)
{
	if (!gpo_action)
		schedule_delayed_work(&deferred_work, MS2HZ(UPDATE_DELAY));
	gpo_action |= gpo_bit[dev->id];
}
#endif

static inline void urb_list_add(spinlock_t *lock, struct urb *urb,
				struct list_head *head)
{
	spin_lock_irq(lock);
	list_add(&urb->urb_list, head);
	spin_unlock_irq(lock);
}

static inline void urb_list_add_tail(spinlock_t *lock, struct urb *urb,
				struct list_head *head)
{
	spin_lock_irq(lock);
	list_add_tail(&urb->urb_list, head);
	spin_unlock_irq(lock);
}

static struct urb *urb_list_get(spinlock_t *lock, struct list_head *head)
{
	struct list_head *p;

	spin_lock_irq(lock);

	if (list_empty(head)) {
		spin_unlock_irq(lock);
		return NULL;
	}

	p = head->next;
	list_del(p);
	spin_unlock_irq(lock);

	return list_entry(p, struct urb, urb_list);
}

/*
 * bulks_release
 *
 * release all allocated urb's and and usb buffers
 */
static void bulks_release(struct urb *urb, unsigned n, int buf_size)
{
	while (n--) {
		if (urb->transfer_buffer)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
			usb_buffer_free(
#else
            usb_free_coherent(
#endif
                urb->dev,
				buf_size,
				urb->transfer_buffer,
				urb->transfer_dma);
		++urb;
	}
}

/*
 * bulks_init
 *
 * preallocate urb's and and usb buffers
 */
static int bulks_init(struct usb_nrpz *dev,
		struct urb *urb,
		unsigned n,
		unsigned int pipe,
		int buf_size,
		usb_complete_t complete_fn)
{
	void *buffer;

	while (n--) {
		usb_init_urb(urb);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
		buffer = usb_buffer_alloc(
#else
		buffer = usb_alloc_coherent(
#endif
                dev->udev,
				buf_size,
				GFP_KERNEL,
				&urb->transfer_dma);
		if (!buffer)
			return -ENOMEM;

		/* set up our read urb */
		usb_fill_bulk_urb(urb,
			dev->udev,
			pipe,
			buffer,
			buf_size,
			complete_fn,
			dev);

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		++urb;
	}
	return 0;
}

static int bulks_in_submit(struct usb_nrpz *dev, gfp_t gfp)
{
	int ret;
	unsigned i;

	for (i = 0; i != ARRAY_SIZE(dev->in_urbs); ++i) {
		usb_anchor_urb(&dev->in_urbs[i], &dev->in_running);

		ret = usb_submit_urb(&dev->in_urbs[i], gfp);
		if (ret) {
			usb_kill_anchored_urbs(&dev->in_running);
			return ret;
		}
	}
	return 0;
}

static void nrpz_read_callback(struct urb *urb)
{
	struct usb_nrpz *dev = (struct usb_nrpz *)urb->context;

#ifdef SMA_K
	spin_lock(&gpo_lock);
	set_action(dev);
	spin_unlock(&gpo_lock);
#endif
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -EPIPE ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN))
			dev_err(&dev->intf->dev,
				"Nonzero read bulk status: %d", urb->status);
	}

	spin_lock(&dev->read_lock);
	list_add_tail(&urb->urb_list, &dev->in_avail);
	spin_unlock(&dev->read_lock);
	wake_up_all(&dev->wq);
}

static void nrpz_write_callback(struct urb *urb)
{
	struct usb_nrpz *dev = (struct usb_nrpz *)urb->context;

#ifdef SMA_K
	spin_lock(&gpo_lock);
	set_action(dev);
	spin_unlock(&gpo_lock);
#endif
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		      urb->status == -EPIPE ||
		      urb->status == -ECONNRESET ||
		      urb->status == -ESHUTDOWN))
			dev_err(&dev->intf->dev,
				"Nonzero write bulk status: %d", urb->status);
	}

	spin_lock(&dev->write_lock);
	list_add_tail(&urb->urb_list, &dev->out_avail);
	spin_unlock(&dev->write_lock);
	wake_up_all(&dev->wq);
}

static void nrpz_delete(struct kref *kref)
{
	struct usb_nrpz *dev = container_of(kref, struct usb_nrpz, kref);

	usb_put_dev(dev->udev);
	kfree(dev);
}

static ssize_t nrpz_read(struct file *file, char __user *buffer, size_t count,
				loff_t *ppos)
{
	struct usb_nrpz *dev = file->private_data;
	int ret;
	struct urb *urb;
	size_t n;

	/* verify that we actually have some data to read */
	if (!count)
		return 0;

	/* lock the read data */
	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&dev->read_mutex))
			return -EAGAIN;
	} else {
		ret = mutex_lock_interruptible(&dev->read_mutex);
		if (ret)
			return ret;
	}

	for (;;) {
		urb = urb_list_get(&dev->read_lock, &dev->in_avail);
		if (urb)
			break;

		/* verify that the device wasn't unplugged */
		if (!dev->intf) {
			ret = -ENODEV;
			goto exit;
		}

		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto exit;
		}

		ret = wait_event_interruptible(dev->wq,
			!list_empty(&dev->in_avail) || !dev->intf);
		if (ret) {
			ret = -ERESTARTSYS;
			goto exit;
		}
	}

	if (!urb->status) {
		n = min(count, (size_t)urb->actual_length);

		if (copy_to_user(buffer, urb->transfer_buffer, n)) {
			urb_list_add(&dev->read_lock, urb, &dev->in_avail);
			ret = -EFAULT;
			goto exit;
		}
	} else {
		n = -EPIPE;
	}

	usb_anchor_urb(urb, &dev->in_running);

	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		usb_unanchor_urb(urb);

		if (!dev->intf) {
			urb_list_add_tail(&dev->read_lock, urb, &dev->in_avail);
			urb->status = ret;
#if 0
		dev_err(&dev->intf->dev,
			"Failed submitting read urb (error %d)", ret);
#endif
		}
	}

	ret = n;
exit:
	mutex_unlock(&dev->read_mutex);
	return ret;
}

static ssize_t nrpz_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *ppos)
{
	struct usb_nrpz *dev = file->private_data;
	int ret;
	size_t len = 0;
	struct urb *urb;
	size_t n;

	/* lock the write data */
	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&dev->write_mutex))
			return -EAGAIN;
	} else {
		ret = mutex_lock_interruptible(&dev->write_mutex);
		if (ret)
			return ret;
	}

	do {
		for (;;) {
			/* verify that the device wasn't unplugged */
			if (!dev->intf) {
				ret = -ENODEV;
				goto exit;
			}

			urb = urb_list_get(&dev->write_lock, &dev->out_avail);
			if (urb)
				break;

			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				goto exit;
			}

			ret = wait_event_interruptible(dev->wq,
				!list_empty(&dev->out_avail) || !dev->intf);
			if (ret) {
				ret = -ERESTARTSYS;
				goto exit;
			}
		}

		n = min(count, dev->out_size);

		if (copy_from_user(urb->transfer_buffer, buffer, n)) {
			urb_list_add(&dev->write_lock, urb, &dev->out_avail);
			ret = -EFAULT;
			break;
		}

		urb->transfer_buffer_length = n;

		usb_anchor_urb(urb, &dev->out_running);

		ret = usb_submit_urb(urb, GFP_KERNEL);
		if (ret) {
			usb_unanchor_urb(urb);
			urb_list_add_tail(&dev->write_lock, urb, &dev->out_avail);
#if 0
			dev_err(&dev->intf->dev,
				"Failed submitting write urb (error %d)", ret);
#endif
			break;
		}

		count -= n;
		buffer += n;
		len += n;
	} while (count);
exit:
	if (len)
		ret = len;

	mutex_unlock(&dev->write_mutex);
	return ret;
}

static long nrpz_vendor_out(struct usb_device *udev, struct nrpz_control_req *ncr)
{
	int ret;
	u8 *data;
	u16 size;

	if (ncr->data) {
		size = ncr->size;

		if (!access_ok(VERIFY_READ, (void __user *)ncr->data, size))
			return -EFAULT;

		data = kzalloc(size, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		memcpy(data, ncr->data, size);
	} else {
		size = 0;
		data = NULL;
	}

	ret = usb_control_msg(udev,
			usb_sndctrlpipe(udev, 0),
			ncr->request,
			ncr->type,
			ncr->value,
			ncr->index,
			data,
			size,
			0);

	kfree(data);

	return ret;
}

static long nrpz_vendor_in(struct usb_device *udev, struct nrpz_control_req *ncr)
{
	int ret;
	u8 *data;
	u16 size;

	if (ncr->data) {
		size = ncr->size;

		if (!access_ok(VERIFY_WRITE, (void __user *)ncr->data, size))
			return -EFAULT;

		data = kzalloc(size, GFP_KERNEL);
		if (!data)
			return -ENOMEM;
	} else {
		size = 0;
		data = NULL;
	}

	ret = usb_control_msg(udev,
			usb_rcvctrlpipe(udev, 0),
			ncr->request,
			ncr->type,
			ncr->value,
			ncr->index,
			data,
			size,
			0);

	if (data) {
		memcpy(ncr->data, data, size);
		kfree(data);
	}

	return ret;
}

#define	VRT_RESET_ALL		 1
#define	VRT_GET_DEVICE_INFO	 6
#define VRT_SET_LEGACY_OPEN  9 
#define VRT_SET_LEGACY_CLOSE 10
#define	VRI_DEVICE_NAME		 5
#define	VRI_DEVICE_READY	 6
#define	DEVICE_STATE_SIZE	 128

static long nrpz_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct usb_nrpz *dev = file->private_data;
	struct usb_interface *intf = dev->intf;
	struct usb_device *udev;
	int ret = -EINVAL;

	/* verify that the device wasn't unplugged */
	if (!intf)
		return -ENODEV;

	udev = interface_to_usbdev(intf);

	switch (cmd) {
	case NRPZ_GETSENSORINFO:
	 {
		struct nrpz_sensor_info __user *sensor_info =
			(struct nrpz_sensor_info __user *)arg;

		if (!access_ok(VERIFY_WRITE, sensor_info, sizeof(*sensor_info)))
			return -EFAULT;

		__put_user(udev->descriptor.bcdDevice,
				&sensor_info->bcdDevice);
		__put_user(udev->descriptor.bcdUSB,
				&sensor_info->bcdUSB);
		__put_user(udev->descriptor.bDescriptorType,
				&sensor_info->bDescriptorType);
		__put_user(udev->descriptor.bDeviceClass,
				&sensor_info->bDeviceClass);
		__put_user(udev->descriptor.bDeviceSubClass,
				&sensor_info->bDeviceSubClass);
		__put_user(udev->descriptor.bDeviceProtocol,
				&sensor_info->bDeviceProtocol);
		__put_user(udev->descriptor.bMaxPacketSize0,
				&sensor_info->bMaxPacketSize0);
		__put_user(udev->descriptor.bNumConfigurations,
				&sensor_info->bNumConfigurations);
		__put_user(udev->descriptor.iManufacturer,
				&sensor_info->iManufacturer);
		__put_user(udev->descriptor.iProduct,
				&sensor_info->iProduct);
		__put_user(udev->descriptor.iSerialNumber,
				&sensor_info->iSerialNumber);
		__put_user(le16_to_cpu(udev->descriptor.idVendor),
				&sensor_info->vendorId);
		__put_user(le16_to_cpu(udev->descriptor.idProduct),
				&sensor_info->productId);
		usb_string(udev, udev->descriptor.iManufacturer,
				(char __force *)sensor_info->manufacturer,
				sizeof(sensor_info->manufacturer));
		usb_string(udev, udev->descriptor.iProduct,
				(char __force *)sensor_info->productName,
				sizeof(sensor_info->productName));
		usb_string(udev, udev->descriptor.iSerialNumber,
				(char __force *)sensor_info->serialNumber,
				sizeof(sensor_info->serialNumber));

		return 0;
	 }
    case NRPZ_GETDEVICEREADY:
     {
        u8 *device_ready;

		if (le16_to_cpu(udev->descriptor.idProduct) < 0x00e2)
			return 1;

        device_ready = kmalloc(1, GFP_KERNEL);

        ret = usb_control_msg(udev,
            usb_rcvctrlpipe(udev, 0),
            VRT_GET_DEVICE_INFO,
            USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            0,
            VRI_DEVICE_READY,
            device_ready,
            1,
            5000);

        if (ret > 0)
            ret = *device_ready;

        kfree(device_ready);
        return ret;            
     }
    case NRPZ_LEGACYOPEN:
     {
        u8 *legacy_open;

		if (le16_to_cpu(udev->descriptor.idProduct) < 0x00e2)
			return 0;

        legacy_open = kzalloc(10, GFP_KERNEL);

        ret = usb_control_msg(udev,
            usb_sndctrlpipe(udev, 0),
            VRT_SET_LEGACY_OPEN,
            USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            1,
            1,
            legacy_open,
            0,
            5000);

        kfree(legacy_open);
        return ret;            
     }
    case NRPZ_LEGACYCLOSE:
     {
        u8 *legacy_close;

		if (le16_to_cpu(udev->descriptor.idProduct) < 0x00e2)
			return 0;

        legacy_close = kzalloc(10, GFP_KERNEL);

        ret = usb_control_msg(udev,
            usb_sndctrlpipe(udev, 0),
            VRT_SET_LEGACY_CLOSE,
            USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
            1,
            1,
            legacy_close,
            0,
            5000);

        kfree(legacy_close);
        return ret;            
     }
	case NRPZ_START:
	 {
		u8 *device_state;

		device_state = kzalloc(DEVICE_STATE_SIZE, GFP_KERNEL);

		ret = usb_control_msg(udev,
			usb_rcvctrlpipe(udev, 0),
			VRT_GET_DEVICE_INFO,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			VRI_DEVICE_NAME,
			device_state,
			DEVICE_STATE_SIZE,
			5000);

		if (ret < 0)
			goto done;

		dev_dbg(&intf->dev,
			"device state:%s", device_state);

		if (strncmp(device_state, "Boot ", 5)) {
			ret = 0;
			goto done;
		}

		ret = usb_control_msg(udev,
			usb_sndctrlpipe(udev, 0),
			VRT_RESET_ALL,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			1,
			1,
			device_state,
			sizeof(device_state),
			5000);
done:
		kfree(device_state);
		return ret;
	 }
	case NRPZ_WRITE_DONE:
		if (arg) {
			ret = wait_event_interruptible_timeout(
				dev->out_running.wait,
				list_empty(&dev->out_running.urb_list),
				msecs_to_jiffies(arg));
			if (!ret)
				return -ETIMEDOUT;
			if (ret < 0)
				return ret;
			return 0;
		} else {
			return wait_event_interruptible(
				dev->out_running.wait,
				list_empty(&dev->out_running.urb_list));
		}
		break;
	case NRPZ_VENDOR_CONTROL_MSG_OUT:
	 {
		struct nrpz_control_req ncr;

		if (copy_from_user(&ncr, (struct nrpz_control_req __user *)arg, sizeof(ncr)))
			return -EFAULT;

		return nrpz_vendor_out(udev, &ncr);
	 }
	case NRPZ_VENDOR_CONTROL_MSG_IN:
	 {
		struct nrpz_control_req ncr;

		if (copy_from_user(&ncr, (struct nrpz_control_req __user *)arg, sizeof(ncr)))
			return -EFAULT;

		return nrpz_vendor_in(udev, &ncr);
	 }
	default:
		dev_dbg(&intf->dev,
			"Invalid ioctl call (%08x)", cmd);
		return -ENOTTY;
	}

	return ret;
}

#ifdef CONFIG_64BIT
struct nrpz_control_req_32 {
	unsigned char request;
	unsigned char type;
	unsigned short value;
	unsigned short index;
	uint32_t data;
	unsigned short size;
};

static long nrpz_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct usb_nrpz *dev = file->private_data;
	struct usb_interface *intf = dev->intf;
	struct usb_device *udev;
	int ret = -EINVAL;

	/* verify that the device wasn't unplugged */
	if (!intf)
		return -ENODEV;

	udev = interface_to_usbdev(intf);

	switch (cmd) {
	case NRPZ_VENDOR_CONTROL_MSG_OUT:
	 {
		struct nrpz_control_req_32 ncr32;
		struct nrpz_control_req ncr;

		if (copy_from_user(&ncr32, (struct nrpz_control_req __user *)arg, sizeof(ncr32)))
			return -EFAULT;

		ncr.request = ncr32.request;
		ncr.type = ncr32.type;
		ncr.value = ncr32.value;
		ncr.index = ncr32.index;
		ncr.data = (void *)(unsigned long)ncr32.data;
		ncr.size = ncr32.size;

		return nrpz_vendor_out(udev, &ncr);
	 }
	case NRPZ_VENDOR_CONTROL_MSG_IN:
	 {
		struct nrpz_control_req ncr32;
		struct nrpz_control_req ncr;

		if (copy_from_user(&ncr32, (struct nrpz_control_req __user *)arg, sizeof(ncr32)))
			return -EFAULT;

		ncr.request = ncr32.request;
		ncr.type = ncr32.type;
		ncr.value = ncr32.value;
		ncr.index = ncr32.index;
		ncr.data = (void *)(unsigned long)ncr32.data;
		ncr.size = ncr32.size;

		return nrpz_vendor_in(udev, &ncr);
	 }
	default:
		return nrpz_ioctl(file, cmd, arg);
	}

	return ret;
}
#endif

static void nrpz_cleanup(struct usb_nrpz *dev)
{
	usb_kill_anchored_urbs(&dev->in_running);
	usb_kill_anchored_urbs(&dev->out_running);

	bulks_release(dev->out_urbs, ARRAY_SIZE(dev->out_urbs), dev->out_size);
	bulks_release(dev->in_urbs, ARRAY_SIZE(dev->in_urbs), dev->in_size);

	spin_lock_irq(&dev->read_lock);
	dev->in_use = false;
	spin_unlock_irq(&dev->read_lock);

	kref_put(&dev->kref, nrpz_delete);
}

static int nrpz_open(struct inode *inode, struct file *file)
{
	struct usb_nrpz *dev;
	int minor;
	struct usb_interface *intf;
	int ret;
	unsigned i;

	minor = iminor(inode);

	intf = usb_find_interface(&nrpz_driver, minor);
	if (!intf)
		return -ENODEV;

	dev = usb_get_intfdata(intf);
	if (!dev)
		return -ENODEV;

	spin_lock_irq(&dev->read_lock);
	if (dev->in_use) {
		spin_unlock_irq(&dev->read_lock);
		return -EBUSY;
	}
	dev->in_use = true;
	spin_unlock_irq(&dev->read_lock);

#ifdef SMA_K
	spin_lock_irq(&gpo_lock);
	set_action(dev);
	spin_unlock_irq(&gpo_lock);
#endif

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

	INIT_LIST_HEAD(&dev->in_avail);
	INIT_LIST_HEAD(&dev->out_avail);

	ret = bulks_init(dev,
			 dev->in_urbs,
			 ARRAY_SIZE(dev->in_urbs),
			 usb_rcvbulkpipe(dev->udev, dev->in_epAddr),
			 dev->in_size,
			 nrpz_read_callback);
	if (ret)
		goto error;

	ret = bulks_init(dev,
			 dev->out_urbs,
			 ARRAY_SIZE(dev->out_urbs),
			 usb_sndbulkpipe(dev->udev, dev->out_epAddr),
			 dev->out_size,
			 nrpz_write_callback);
	if (ret)
		goto error;

	ret = bulks_in_submit(dev, GFP_KERNEL);
	if (ret)
		goto error;

	for (i = 0; i != ARRAY_SIZE(dev->out_urbs); ++i)
		list_add(&dev->out_urbs[i].urb_list, &dev->out_avail);

	return 0;
error:
	nrpz_cleanup(dev);
	return ret;
}

static int nrpz_release(struct inode *inode, struct file *file)
{
	struct usb_nrpz *dev = file->private_data;

	nrpz_cleanup(dev);
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
static int nrpz_fsync(struct file *file, loff_t start, loff_t end, int datasync)
#else
static int nrpz_fsync(struct file *file, int datasync)
#endif
{
	struct usb_nrpz *dev = file->private_data;
	int ret;

	/* lock the write data */
	ret = mutex_lock_interruptible(&dev->write_mutex);
	if (ret)
		return ret;

	/* verify that the device wasn't unplugged */
	if (!dev->intf) {
		ret = -ENODEV;
		goto exit;
	}

	ret = wait_event_interruptible(dev->out_running.wait,
				list_empty(&dev->out_running.urb_list));
	if (ret) {
		ret = -ERESTARTSYS;
		goto exit;
	}
exit:
	mutex_unlock(&dev->write_mutex);

	return ret;
}

static int nrpz_flush(struct file *file, fl_owner_t id)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
	return nrpz_fsync(file, 0, LLONG_MAX, 0);
#else
	return nrpz_fsync(file, 0);
#endif
}

static unsigned int nrpz_poll(struct file *file, poll_table *wait)
{
	struct usb_nrpz *dev = file->private_data;
	int ret = 0;

	poll_wait(file, &dev->wq, wait);

	if (!dev->intf)
		ret = POLLIN | POLLOUT | POLLPRI | POLLERR | POLLHUP;
	else {
		if (!list_empty(&dev->in_avail))
			ret |= POLLIN;

		if (!list_empty(&dev->out_avail))
			ret |= POLLOUT;
	}

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
loff_t noop_llseek(struct file *file, loff_t offset, int whence)
{
    return file->f_pos;
}
#endif

static const struct file_operations nrpz_fops = {
	.owner = THIS_MODULE,
	.read = nrpz_read,
	.write = nrpz_write,
	.unlocked_ioctl = nrpz_ioctl,
#ifdef CONFIG_64BIT
	.compat_ioctl = nrpz_compat_ioctl,
#endif
	.open = nrpz_open,
	.release = nrpz_release,
	.fsync = nrpz_fsync,
	.flush = nrpz_flush,
	.poll = nrpz_poll,
	.llseek = noop_llseek,
};

static struct usb_class_driver nrpz_class = {
	.name = "nrpz%d",
	.fops = &nrpz_fops,
	.minor_base = NRPZ_MINOR_BASE,
};

static int nrpz_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	int i;
	int ret;
	struct usb_device *udev;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *iface_desc;
	struct usb_nrpz *dev;
	const char *name;

	udev = usb_get_dev(interface_to_usbdev(intf));
	if (!udev)
		return -ENODEV;

	iface_desc = intf->cur_altsetting;
	if(!iface_desc)
	  return -ENODEV;

	if((iface_desc->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC) || (iface_desc->desc.bInterfaceSubClass != USB_CLASS_VENDOR_SPEC))
	  return -ENODEV;

	for(i = 0; i != ARRAY_SIZE(known_products); ++i) {
		if (known_products[i] == le16_to_cpu(udev->descriptor.idProduct))
			break;
	}

	if (i == ARRAY_SIZE(known_products)) {
		char buf[64] = "";

		usb_string(udev, udev->descriptor.iProduct, buf, sizeof(buf));

		if (strncmp(buf, "NRP", 3)) {
			ret = -ENODEV;
			goto err1;
		}
	}

	/* allocate memory for our device state and intialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&intf->dev, "Out of memory");
		ret = -ENOMEM;
		goto err2;
	}

	ret = -EIO;

	init_waitqueue_head(&dev->wq);
	kref_init(&dev->kref);

	mutex_init(&dev->read_mutex);
	mutex_init(&dev->write_mutex);

	spin_lock_init(&dev->read_lock);
	spin_lock_init(&dev->write_lock);

	init_usb_anchor(&dev->in_running);
	init_usb_anchor(&dev->out_running);

	dev->udev = udev;
	dev->in_use = false;
	dev->intf = intf;

	/* set up the endpoint information */
  /* check out the endpoints */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->in_epAddr && usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			dev->in_size = le16_to_cpu(endpoint->wMaxPacketSize);
			dev->in_epAddr = endpoint->bEndpointAddress;
		} else
		if (!dev->out_epAddr && usb_endpoint_is_bulk_out(endpoint)) {
			/* we found a bulk out endpoint */
			dev->out_size = le16_to_cpu(endpoint->wMaxPacketSize);
			dev->out_epAddr = endpoint->bEndpointAddress;
		}
	}
	if (!(dev->in_epAddr && dev->out_epAddr)) {
		dev_err(&intf->dev,
			"Could not find both bulk in and out endpoints");
		ret = -EIO;
		goto err2;
	}

	usb_set_intfdata(intf, dev);

	ret = usb_register_dev(intf, &nrpz_class);
	if (ret) {
		dev_err(&intf->dev,
			"Not able to get a minor for this device\n");
		goto err3;
	}

	name = dev_name(&dev->udev->dev);

	/* let the user know what node this device is now attached to */
	dev_info(&intf->dev,
		"Device now attached to USB %s on port %s", intf->usb_dev->kobj.name, name);

#ifdef SMA_K
	if (!strcmp(name, "4-2"))
		dev->id = 1;
	else
	if (!strcmp(name, "4-1"))
		dev->id = 2;
	else
	if (!strcmp(name, "3-2"))
		dev->id = 3;
	else
	if (!strcmp(name, "3-1"))
		dev->id = 4;

	dev_info(&intf->dev, "Device id %u", dev->id);

	spin_lock_irq(&gpo_lock);
	gpo_val |= gpo_bit[dev->id];
	set_gpo(gpo_val);
	spin_unlock_irq(&gpo_lock);
#endif
	return 0;
err3:
	usb_set_intfdata(intf, NULL);
err2:
	kfree(dev);
err1:
	usb_put_dev(udev);
	return ret;
}

static void nrpz_disconnect(struct usb_interface *intf)
{
	struct usb_nrpz *dev;

	dev_info(&intf->dev,
		"%s disconnected", intf->usb_dev->kobj.name);

	dev = usb_get_intfdata(intf);
#ifdef SMA_K
	spin_lock_irq(&gpo_lock);
	gpo_val &= ~gpo_bit[dev->id];
	gpo_action &= ~gpo_bit[dev->id];
	gpo_running &= ~gpo_bit[dev->id];
	set_gpo(gpo_val);
	spin_unlock_irq(&gpo_lock);
#endif
	usb_set_intfdata(intf, NULL);

	/* give back our minor */
	usb_deregister_dev(intf, &nrpz_class);

	/* prevent more I/O from starting */
	dev->intf = NULL;

	usb_kill_anchored_urbs(&dev->in_running);
	usb_kill_anchored_urbs(&dev->out_running);

	wake_up_all(&dev->wq);

	/* decrement our usage count */
	kref_put(&dev->kref, nrpz_delete);
}

static void nrpz_draw_down(struct usb_nrpz *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->out_running, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->out_running);

	usb_kill_anchored_urbs(&dev->in_running);
}

static int nrpz_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_nrpz *dev = usb_get_intfdata(intf);

	if (dev)
		nrpz_draw_down(dev);
	return 0;
}

static int nrpz_resume(struct usb_interface *intf)
{
	struct usb_nrpz *dev = usb_get_intfdata(intf);

	if (dev)
		return bulks_in_submit(dev, GFP_NOIO);
	return 0;
}

static struct usb_driver nrpz_driver = {
	.name = "nrpz",
	.probe = nrpz_probe,
	.disconnect = nrpz_disconnect,
	.suspend = nrpz_suspend,
	.resume = nrpz_resume,
	.id_table = nrpz_table,
};

static int __init nrpz_init(void)
{
	int ret;

#ifdef SMA_K
	spin_lock_init(&gpo_lock);

	superio_probe(0x4e);
	if (!do_superio)
		pr_info("no super io chip found\n");
 
	set_gpo(0xf0);
	schedule_timeout_interruptible(HZ/10);
	set_gpo(0x00);

	INIT_DELAYED_WORK(&deferred_work, led_deferred_work);
#endif
	/* register this driver with the USB subsystem */
	ret = usb_register(&nrpz_driver);
	if (ret) {
		pr_err("%s usb_register failed (%d)", nrpz_driver.name, ret);
		return ret;
	}

	pr_info("R&S NRP power meter driver v" DRIVER_VERSION);

	return 0;
}

static void __exit nrpz_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&nrpz_driver);
#ifdef SMA_K
	cancel_delayed_work(&deferred_work);
	flush_scheduled_work();

	set_gpo(0x00);
#endif
}

module_init(nrpz_init);
module_exit(nrpz_exit);

MODULE_AUTHOR("Patrick Geltinger <Patrick.Geltinger@rohde-schwarz.com>");
MODULE_DESCRIPTION("Rohde & Schwarz NRP USB power meter series driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
