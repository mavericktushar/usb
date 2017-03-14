/*
 *
 * USB Demo driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 *
 * This is a port of usb driver from the book "Linux Device Drivers" by
 * Jonathan Corbet, Alessandro Rubini and Greg Kroah-Hartman, published by 
 * O'Reilly & Associates.
 *  
 * Port from 2.6 Linux kernel to 3.19 Linux Kernel.
 *
 * Linux Device Driver 3, usb-skeleton.c code for kerenel 2.6.3 
 * has been written for 3.19
 *
 * This was done purely for my own practice and understanding.
 * No warranty is attached. I cannot take responsibility for errors or
 * fitness for use.
 *
 */

#include <linux/mutex.h>

#define USB_DEMO_VENDOR_ID	0x058F
#define USB_DEMO_PRODUCT_ID	0x6387

#define USB_DEMO_MINOR_BASE     100

static struct mutex usb_mutex;

//Structure for all device specific stuff:
struct usb_demo {
	struct usb_device    *udev;
	struct usb_interface *interface;
	unsigned char        *bulk_in_buffer;
	size_t               bulk_in_size;
	__u8                 bulk_in_endpoint_addr;
	__u8                 bulk_out_endpoint_addr;
	struct kref          kref;
};
