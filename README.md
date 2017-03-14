# usb

This is a port of usb driver from the book "Linux Device Drivers" by
Jonathan Corbet, Alessandro Rubini and Greg Kroah-Hartman, published by 
O'Reilly & Associates which is based on 2.6 Linux kernel to 3.19 Linux
Kernel

This was done purely for my own practice and understanding. No warranty
is attached. I cannot take responsibility for errors or fitness for use.

Functionality:

To test the driver, I used a usb drive with:
Vendor ID: 0x058f
Product ID: 0x6387

Following are the steps to see init module and exit module being called at
module load/unload time and probe and disconnect being called at usb drive
insertion/removal time.

A) Remove any existing driver for this usb drive:
   sudo rmmod uas
   sudo rmmod usb-storage

B) Insert the Demo driver:
   sudo insmod usb_drv.ko

   Check dmesg:
   usbcore: registered new interface driver usb_demo_driver

C) Insert the usb drive:
   
   Check dmesg:
   USB interface 0 probed
   bNumEndpoints 0x02
   bInterfaceClass 0x08
   Endpoint[0]: bEndpointAddress 0x01
   Endpoint[0]: bmAttributes     0x02
   Endpoint[0]: wMaxPacketSize   0x0200 ( 512 )
   Endpoint[1]: bEndpointAddress 0x82
   Endpoint[1]: bmAttributes     0x02
   Endpoint[1]: wMaxPacketSize   0x0200 ( 512 )
   USB Device now attached to USBDemo-2
   USB Device: Vendor ID: 058f, Product ID: 6387

D) Try reading from the device:
   sudo cat /dev/demo2

   Check dmesg:
   usb read method called
 
E) Disconnect usb drive:

   Check dmesg:
   usb 1-2: USB disconnect, device number 12
   USB Demo #2 now disconnected

F) Remove the Demo Driver:
   sudo rmmod usb_drv
   usbcore: deregistering interface driver usb_demo_driver

Changes required to move from 2.6 to 3.19 kernel:

1) struct usb_driver change:

   struct usb_driver in 3.19 kernel has no owner field, therefore
   cannot initialize it:

   usb_demo_driver looks like this:

   static struct usb_driver usb_demo_driver = {
        .name = "usb_demo_driver",
        .id_table = usb_demo_table,
        .probe = usb_demo_probe,
        .disconnect = usb_demo_disconnect,
   };

2) struct usb_class_driver change:

   struct usb_class_driver in 3.19 kernel has no mode field, therefore
   cannot initialize it:

   demo_class looks like this:

   static struct usb_class_driver demo_class = {
        .name = "usb/demo%d",
        .fops = &demo_fops,
        .minor_base = USB_DEMO_MINOR_BASE,
   };
				    
3) No Big Kernel Lock:
   
   2.6 kernel:
   lock_kernel();

   unlock_kernel();

   3.19 kernel:
   static struct mutex usb_mutex;

   mutex_init( &usb_mutex );

   mutex_lock( &usb_mutex );

   mutex_unlock( &usb_mutex );

4) No usb_buffer_alloc() api:

   2.6 kernel:
   usb_buffer_alloc( struct usb_device *dev, size_t size, gfp_t mem_flags,
                     dma_addr_t *dma );
   usb_buffer_free( struct usb_device *dev, size_t size, gfp_t mem_flags,
                    dma_addr_t *dma );

   3.19 kernel:
   usb_alloc_coherent( struct usb_device *dev, size_t size, gfp_t mem_flags,
                       dma_addr_t *dma );

   usb_free_coherent( struct usb_device *dev, size_t size, gfp_t mem_flags,
                      dma_addr_t *dma );
   
5) URB write callback api change:
   
   2.6 kernel:
   typedef void (*callback)(struct urb *urb, struct pt_regs *regs);

   3.19 kernel:
   usb_complete_t:
  
   which is:
   typedef void (*usb_complete_t)(struct urb *);

   this does not have struct pt_regs * argument
