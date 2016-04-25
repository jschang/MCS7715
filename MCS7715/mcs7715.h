//
//  mcs7715.h
//  MCS7715
//
//  Created by Jonathan Schang on 2/28/15.
//  Copyright (c) 2015 Jon Schang. All rights reserved.
//

#ifndef __MCS7715__mcs7715__
#define __MCS7715__mcs7715__

#include <libusb-1.0/libusb.h>

void mos7715_change_mode(libusb_device_handle *device, int m);
int read_status(libusb_device_handle *usbdev, unsigned char* data);

void mos7715_write_dcr(libusb_device_handle* device, unsigned char data);
unsigned char mos7715_read_dcr(libusb_device_handle* device);

void mos7715_write_dsr(libusb_device_handle* device, unsigned char data);
unsigned char mos7715_read_dsr(libusb_device_handle* device);

void mos7715_write_dpr(libusb_device_handle* device, unsigned char data);
unsigned char mos7715_read_dpr(libusb_device_handle* device);


libusb_device_handle* mcs7715_find_device();

unsigned char parport_mos7715_read_status(libusb_device_handle *dev);

unsigned char parport_mos7715_read_data(libusb_device_handle *dev);
void parport_mos7715_write_data(libusb_device_handle *dev, unsigned char d);

unsigned char parport_mos7715_read_control(libusb_device_handle *dev);
void parport_mos7715_write_control(libusb_device_handle *dev, unsigned char d);
unsigned char parport_mos7715_frob_control(libusb_device_handle* dev, unsigned char mask, unsigned char val);

size_t parport_mos7715_epp_write_data(libusb_device_handle *dev, const void *buf, size_t length, int flags);

void parport_mos7715_data_forward (libusb_device_handle *dev);
void parport_mos7715_data_reverse (libusb_device_handle *dev);

size_t parport_mos7715_write_compat(libusb_device_handle *dev, const void *buffer, size_t len, int flags);

#endif /* defined(__MCS7715__mcs7715__) */
