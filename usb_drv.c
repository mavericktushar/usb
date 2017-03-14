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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "usb_drv.h"

static struct usb_device_id usb_demo_table[] = {
	{ USB_DEVICE( USB_DEMO_VENDOR_ID, USB_DEMO_PRODUCT_ID )  },
	{ }
};

MODULE_DEVICE_TABLE( usb, usb_demo_table );

static void demo_delete( struct kref *krf ) {
	struct usb_demo *dev = container_of( krf , struct usb_demo, kref );

	usb_put_dev( dev->udev );
	kfree( dev->bulk_in_buffer );
	kfree( dev );

	return ;
}

static struct usb_driver usb_demo_driver;

//Open Callback ( file operation ):
static int demo_open( struct inode *inode, struct file *filp ) {
	struct usb_demo      *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor( inode );

	interface = usb_find_interface( &usb_demo_driver, subminor );
	if( !interface ) {
		printk(KERN_ALERT "Cannot find interface information for minor: %d\n", subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata( interface );
	if( !dev ) {
		printk(KERN_ALERT "Cannot find device for given interface\n");
		retval = -ENODEV;
		goto exit;
	}

	//Increment usage count:
	kref_get( &dev->kref );

	filp->private_data = dev;

exit:

	return retval;
}

//Close Callback ( file operation ):
static int demo_close( struct inode *inode, struct file *filp ) {
	struct usb_demo *dev = ( struct usb_demo * ) filp->private_data;

	if( dev == NULL ) {
		return -ENODEV;
	}

	kref_put( &dev->kref, demo_delete );

	return 0;
}

//Read Callback ( file operation ):
//usb_bulk_msg:
static ssize_t demo_read( struct file *filp, char __user *buffer, size_t count, loff_t *ppos ) {
	struct usb_demo *dev = ( struct usb_demo * ) filp->private_data;
	int retval = 0;

	printk(KERN_INFO "usb read method called\n");

	//Blocking bulk read to get data from the device:
	retval = usb_bulk_msg( 	dev->udev,
				usb_rcvbulkpipe( dev->udev, dev->bulk_in_endpoint_addr ),
				dev->bulk_in_buffer,
				min( dev->bulk_in_size, count ),
				(int *)&count,
				( HZ * 10 ) );

	//if read successfu:
	//copy the data to userspace:
	if( !retval ) {
		if( copy_to_user( buffer, dev->bulk_in_buffer, count) ) {
			retval = -EFAULT;
		} else {
			retval = count;
		}
	}

	return retval;
}

//After the urb is succefully transmitted to the USB Device:
//Or something happens in transmission:
//urb Callback is called by the USB Core:
//static void demo_write_callback( struct urb *urb, struct pt_regs *regs ) {
static void demo_write_callback( struct urb *urb ) {
	if( urb->status &&
	    !( urb->status == -ENOENT     ||
	       urb->status == -ECONNRESET ||
	       urb->status == -ESHUTDOWN
	     )
	  ) {
		printk(KERN_ALERT "nonzero write bulk status received");
	    }

	usb_free_coherent( urb->dev,
			   urb->transfer_buffer_length,
			   urb->transfer_buffer,
			   urb->transfer_dma );

	return ;
}

//Write Callback ( file operation ):
static ssize_t demo_write( struct file *filp, const char __user * buffer, size_t count, loff_t *ppos ) {
	struct usb_demo *dev = filp->private_data;
	int             retval = 0;
	struct urb      *urb = NULL;
	char            *buf = NULL;

	printk(KERN_INFO "usb write method called\n");

	//if no data to write, exit:
	if( !count ) {
		printk( KERN_ALERT "no data to write\n");
		goto exit;
	}

	//Create a urb:
	//create buffer for it:
	//copy data to the urb:
	urb = usb_alloc_urb( 0, GFP_KERNEL );
	if( !urb ) {
		printk( KERN_ALERT "Could not allocate urb\n");
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent( dev->udev, count, GFP_KERNEL, &urb->transfer_dma );
	if( !buf ) {
		printk( KERN_ALERT "Could not allocate buffer\n");
		retval = -ENOMEM;
		goto error;
	}

	if( copy_from_user( buf, buffer, count ) ) {
		printk(KERN_ALERT "copy_from_user failed in demo_write\n");
		retval = -EFAULT;
		goto error;
	}

	//Initialize the urb properly:
	usb_fill_bulk_urb( urb,
			   dev->udev,
			   usb_sndbulkpipe( dev->udev, dev->bulk_out_endpoint_addr ),
			   buf,
			   count,
			   demo_write_callback,
			   dev );

	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	//Send the data out the bulk port:
	retval = usb_submit_urb( urb, GFP_KERNEL );
	if( retval ) {
		printk(KERN_ALERT "usb_submit_urb failed\n");
		goto error;
	}

	usb_free_urb( urb );

exit:
	return count;

error:
	usb_free_coherent( dev->udev, count, buf, urb->transfer_dma );
	usb_free_urb( urb );
	kfree( buf );

	return retval;
}

static struct file_operations demo_fops = {
	.owner = THIS_MODULE,
	.open  = demo_open,
	.release = demo_close,
	.read  = demo_read,
	.write = demo_write,
};

static struct usb_class_driver demo_class = {
	.name = "usb/demo%d",
	.fops = &demo_fops,
	.minor_base = USB_DEMO_MINOR_BASE,
};

static int usb_demo_probe( struct usb_interface *interface,
			   const struct usb_device_id *id) {
	struct usb_demo                *dev = NULL;
	struct usb_host_interface      *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int    i;
	int    retval = -ENOMEM;

	//Allocate memory for our device and initialize:
	dev = kmalloc( sizeof( struct usb_demo ), GFP_KERNEL );

	if( dev == NULL ) {
		printk(KERN_ALERT "Out Of Memory. Cannot allocate memory\n");
		goto error;
	}

	memset( dev, 0x00, sizeof( *dev ) );
	kref_init( &dev->kref );

	dev->udev = usb_get_dev( interface_to_usbdev( interface ) );
	dev->interface = interface;

	//Setup endpoint information:
	//first bulk-in, bulk-oou endpoints:
	iface_desc = interface->cur_altsetting;

	//Print some Interface information:
	printk(KERN_INFO "USB interface %d probed\n", iface_desc->desc.bInterfaceNumber);
	printk(KERN_INFO "bNumEndpoints 0x%02x\n", iface_desc->desc.bNumEndpoints);
	printk(KERN_INFO "bInterfaceClass 0x%02x\n", iface_desc->desc.bInterfaceClass);

	for( i = 0; i < iface_desc->desc.bNumEndpoints; i++ ) {
		endpoint = &iface_desc->endpoint[i].desc;

		//Print some Endpoint information:
		printk(KERN_INFO "Endpoint[%d]: bEndpointAddress 0x%02x\n", i, endpoint->bEndpointAddress);
		printk(KERN_INFO "Endpoint[%d]: bmAttributes     0x%02x\n", i, endpoint->bmAttributes);
		printk(KERN_INFO "Endpoint[%d]: wMaxPacketSize   0x%04x ( %d )\n",
		       		  i, endpoint->wMaxPacketSize, endpoint->wMaxPacketSize);

		if( !dev->bulk_in_endpoint_addr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN ) &&
		    ( (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		      USB_ENDPOINT_XFER_BULK ) ) {
			//This case satisfies all conditions of
			//bulk in endpoint:

			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpoint_addr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc( buffer_size, GFP_KERNEL );

			if( !dev->bulk_in_buffer ) {
				printk(KERN_ALERT "Could not allocate bulk in buffer\n");
				goto error;
			}
			
		}

		if( !dev->bulk_out_endpoint_addr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ( (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		    USB_ENDPOINT_XFER_BULK ) ) {
			//This case satisfies all conditions of
			//bulk out endpoint:

			dev->bulk_out_endpoint_addr = endpoint->bEndpointAddress;

		}
	}

	if( !dev->bulk_in_endpoint_addr && !dev->bulk_out_endpoint_addr ) {
		printk(KERN_ALERT "Could not find either bulk in endpoint or"
				  " bulk out endpoint\n");

		goto error;
	}

	//Save pointer to our data to this device:
	//Very good feature provided by USB Core;
	usb_set_intfdata( interface, dev );

	//Register the device to the USB Core:
	//Ready to be used:
	retval = usb_register_dev( interface, &demo_class );

	if( retval ) {
		printk(KERN_ALERT "Error registering Driver to the USB Core\n");
		printk(KERN_ALERT "Not able to get minor for the Device\n");
		//Unset the pointer to our data on the device:
		usb_set_intfdata( interface, NULL );
		goto error;
	}

	printk(KERN_ALERT "USB Device now attached to USBDemo-%d", interface->minor);
	printk(KERN_INFO "USB Device: Vendor ID: %04x, Product ID: %04x\n",
			 id->idVendor, id->idProduct);
	return 0;

error:

	if( dev ) {
		kref_put( &dev->kref, demo_delete );
	}

	return retval;
}

static void usb_demo_disconnect( struct usb_interface *interface ) {
	struct usb_demo *dev;
	int minor = interface->minor;

	mutex_lock( &usb_mutex );

	dev = usb_get_intfdata( interface );
	usb_set_intfdata( interface, NULL );

	//give back minor:
	usb_deregister_dev( interface, &demo_class );

	mutex_unlock( &usb_mutex );

	kref_put( &dev->kref, demo_delete );

	printk( KERN_ALERT "USB Demo #%d now disconnected", minor );
	
	return ;
}

static struct usb_driver usb_demo_driver = {
	.name = "usb_demo_driver",
	.id_table = usb_demo_table,
	.probe = usb_demo_probe,
	.disconnect = usb_demo_disconnect,
};

static int __init usb_drv_init( void ) {
	int result;

	//Register driver with the USB Subsystem:
	result = usb_register( &usb_demo_driver );

	//Init the usb_lock:
	mutex_init( &usb_mutex );

	if( result ) {
		printk(KERN_ALERT "[%s:%d]: usb_register failed\n",
		       __FUNCTION__, __LINE__);
	}

	return result;
}

static void __exit usb_drv_exit( void ) {
	//Deregisters this driver from the USB Subsystem:
	usb_deregister( &usb_demo_driver );
}

module_init( usb_drv_init );
module_exit( usb_drv_exit );

MODULE_LICENSE("GPL");
