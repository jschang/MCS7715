//
//  mcs7715.c
//  MCS7715
//
//  Created by Jonathan Schang on 2/28/15.
//  Copyright (c) 2015 Jon Schang. All rights reserved.
//

/*
 1 configs
	config 1 has 1 ifaces
	config 0 identified by 1
 config 0, iface 0 has 1 1 altsetting
 iface 0, alt setting 0, has 5 endpoints
 endpoint 0 address is 10000001
 direction is in
 endpoint 0 descriptor type is 00000101
 endpoint 0 attributes are 00000010
 transfer type is: bulk
 
 endpoint 1 address is 00000010
 direction is out
 endpoint 1 descriptor type is 00000101
 endpoint 1 attributes are 00000010
 transfer type is: bulk
 
 endpoint 2 address is 10000011
 direction is in
 endpoint 2 descriptor type is 00000101
 endpoint 2 attributes are 00000010
 transfer type is: bulk
 
 endpoint 3 address is 00000100
 direction is out
 endpoint 3 descriptor type is 00000101
 endpoint 3 attributes are 00000010
 transfer type is: bulk
 
 endpoint 4 address is 10000111
 direction is in
 endpoint 4 descriptor type is 00000101
 endpoint 4 attributes are 00000011
 transfer type is: interrupt
 */

#include "mcs7715.h"
#include <libusb-1.0/libusb.h>

#define DEBUG_OUT 0
#if DEBUG_OUT==1
#define DBG(...) printf(__VA_ARGS__);
#else
#define DBG(...) 
#endif

#define HZ 1000 // 1000 ms per second

#define MCS7715_VENDOR_ID       0x9710
#define MCS7715_PRODUCT_ID      0x7715

#define MOS_EP0_IN  0b10000001 /* parallel port bulk-in */
#define MOS_EP0_OUT 0b00000010 /* parallel port bulk-out */
#define MOS_EP1_IN  0b10000011 /* serial port bulk-in */
#define MOS_EP1_OUT 0b00000100 /* serial port bulk-out */
#define MOS_EP_INT  0b10000111 /* status endpoint */

/*****************************************************************************
 * ECR modes
 *****************************************************************************/

#define ECR_SPP 00 // spp mode (original
#define ECR_PS2 01 // nibble mode
#define ECR_PPF 02 // cb_fifo mode
#define ECR_ECP 03 // unused, per datasheet
#define ECR_EPP 04 // unused, per datasheet

/*
 * Defines used for sending commands to port
 */

#define WAIT_FOR_EVER   (HZ*0)    /* timeout urb is wait for ever*/
#define MOS_WDR_TIMEOUT (HZ*5)    /* default urb timeout */

#define MOS_PPORT       0x0100    /* Parallel port */
#define MOS_PORT1       0x0200    /* Serial Port 1 */
#define MOS_PORT2       0x0300    /* Serial Port 2 */
#define MOS_VENREG      0x0000    /* Vendor Specific Cmd */
#define MOS_VEN_REG	    0x02      /* Number of Serial Ports */
#define MOS_WRITE       0x0E      /* Write Request */
#define MOS_READ        0x0D      /* Read Request */

void mcs7715_startup(libusb_device_handle* usbdevice);

unsigned char reg[7];

typedef uint8_t __u8;
typedef uint16_t __u16;

static const char *byte_to_binary(int x) {
    static char b[9];
    b[0] = '\0';
    int z;
    for (z = 128; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

int read_status(libusb_device_handle *usbdev, unsigned char* data) {
    int read;
    int ret = libusb_interrupt_transfer(usbdev, MOS_EP_INT, data, 4, &read, HZ*5);
    DBG("libusb_interrupt_transfer(endpoint:0x%X, data:0x%X, length:4, actual_read:0x%X, timeout:HZ*5) ret:%u\n",MOS_EP_INT,(uint32_t)*data,read,ret);
    DBG("read: %u : %s %s %s %s\n",read,byte_to_binary(data[3]),byte_to_binary(data[2]),byte_to_binary(data[1]),byte_to_binary(data[0]));
    return read;
}

int SendMosCmd(libusb_device_handle* usbdev, __u8 request, __u16 value, __u16 index, void *data)
{
    int timeout;
    int status;
    __u8 requesttype;
    __u16 size;
    
    size = 0x00;
    timeout = MOS_WDR_TIMEOUT;
    
    if(value == MOS_VEN_REG) {
        value = 0x0000;
    } else {
        value = (value+1)*0x100;
    }
    value = 0x100;
    
    if(request==MOS_WRITE) {
        request = (__u8)MOS_WRITE;
        requesttype = (__u8)0x40;
        value  = value + (__u16)*((unsigned char *)data);
        data = NULL;
    } else {
        request = (__u8)MOS_READ;
        requesttype = (__u8)0xC0;
        size = 0x01;
    }
    status = libusb_control_transfer(usbdev,requesttype,request,value,index,data,size,timeout);
    DBG("%s - SendMosCmd(req_type:0x%X,request:0x%X,value:0x%X,index:0x%X,.) size:%u, status:%u\n",request==MOS_WRITE?"write":"read",requesttype,request,value,index,size,status);
    return status;
}

void mos7715_change_mode(libusb_device_handle *device, int m)
{
    unsigned char data;
    data = 0x00;
    DBG("change mode intended: %s\n",byte_to_binary(m));
    SendMosCmd(device,MOS_READ,0,0x0A, &data);
    DBG("change mode original mode: %s\n",byte_to_binary(data));
    data = (data & 0b00011111) | (m<<5);
    SendMosCmd(device,MOS_WRITE,0,0x0A, &data);
    DBG("change mode new mode: %s\n",byte_to_binary(data));
}

unsigned char mos7715_read_dcr(libusb_device_handle* device) {
    unsigned char data = 0x00;
    SendMosCmd(device,MOS_READ,0,0x02, &data);
    reg[1] = data;
    DBG( "read dcr: %s\n", byte_to_binary(data) );
    DBG( "\tdirection: %s\n", (data & 0b00100000) ? "input mode" : "output mode" );
    DBG( "\tselect in: %s\n", (data & 0b00001000) ? "selected, pin low" : "not selected, pin high" );
    DBG( "\tinit:      %s\n", (data & 0b00000100) ? "normal, pin low" : "initializing, pin high" );
    DBG( "\tautofeed:  %s\n", (data & 0b00000010) ? "autofeed on, pin low" : "none, pin high" );
    DBG( "\tstrobe:    %s\n", (data & 0b00000001) ? "dpr latched, pin low" : "dpr unlatched, pin high" );
    return data;
}

void mos7715_write_dcr(libusb_device_handle* device, unsigned char data) {
    SendMosCmd(device,MOS_WRITE,0,0x02, &data);
    DBG( "wrote dcr: %s\n", byte_to_binary(data) );
    DBG( "\tdirection: %s\n", (data & 0b00100000) ? "input mode" : "output mode" );
    DBG( "\tselect in: %s\n", (data & 0b00001000) ? "selected, pin low" : "not selected, pin high" );
    DBG( "\tinit:      %s\n", (data & 0b00000100) ? "normal, pin low" : "initializing, pin high" );
    DBG( "\tautofeed:  %s\n", (data & 0b00000010) ? "autofeed on, pin low" : "none, pin high" );
    DBG( "\tstrobe:    %s\n", (data & 0b00000001) ? "dpr latched, pin low" : "dpr unlatched, pin high" );
}

unsigned char mos7715_read_dsr(libusb_device_handle* device) {
    unsigned char data = 0x00;
    SendMosCmd(device,MOS_READ,0,0x01, &data);
    reg[1] = data;
    DBG( "read dsr: %s\n", byte_to_binary(data) );
    DBG( "\tbusy:  %s\n", (data & 0b10000000) ? "ready for data, pin low" : "not ready for data, pin high" );
    DBG( "\tack:   %s\n", (data & 0b01000000) ? "ack high" : "ack low" );
    DBG( "\tpe:    %s\n", (data & 0b00100000) ? "normal, pin low" : "initializing, pin high" );
    DBG( "\tslct:  %s\n", (data & 0b00010000) ? "printer online" : "printer offline" );
    DBG( "\tfault: %s\n", (data & 0b00001000) ? "normal op" : "printer reports error" );
    return data;
}

void mos7715_write_dsr(libusb_device_handle* device, unsigned char data) {
    SendMosCmd(device,MOS_WRITE,0,0x01, &data);
    DBG( "wrote dsr: %s\n", byte_to_binary(data) );
    DBG( "\tbusy:  %s\n", (data & 0b10000000) ? "ready for data, pin low" : "not ready for data, pin high" );
    DBG( "\tack:   %s\n", (data & 0b01000000) ? "ack high" : "ack low" );
    DBG( "\tpe:    %s\n", (data & 0b00100000) ? "normal, pin low" : "initializing, pin high" );
    DBG( "\tslct:  %s\n", (data & 0b00010000) ? "printer online" : "printer offline" );
    DBG( "\tfault: %s\n", (data & 0b00001000) ? "normal op" : "printer reports error" );
}

unsigned char mos7715_read_dpr(libusb_device_handle* device) {
    unsigned char data = 0x00;
    SendMosCmd(device,MOS_READ,0,0x00, &data);
    reg[1] = data;
    DBG( "read dpr: %s\n", byte_to_binary(data) );
    return data;
}

void mos7715_write_dpr(libusb_device_handle* device, unsigned char data) {
    SendMosCmd(device,MOS_WRITE,0,0x00, &data);
    DBG( "wrote dpr: %s\n", byte_to_binary(data) );
}

void mcs7715_save_state(libusb_device_handle *device) {
    
    unsigned char data = 0x00;
    SendMosCmd(device,MOS_READ,0,0x02, &data);
    reg[1] = data;
    
    data = 0x00;
    SendMosCmd(device,MOS_READ,0,0x0A, &data);
    reg[2] = data;
}

libusb_device_handle* mcs7715_find_device() {
    if(libusb_init(0)!=0) {
        return 0;
    }
    // discover devices
    libusb_device **list;
    libusb_device *found = NULL;
    ssize_t cnt = libusb_get_device_list(NULL, &list);
    ssize_t i = 0;
    int err = 0;
    if (cnt < 0) {
        return 0;
    }
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];
        struct libusb_device_descriptor descriptor;
        if (libusb_get_device_descriptor(device,&descriptor)!=0) {
            continue;
        }
        if (descriptor.idVendor==MCS7715_VENDOR_ID
                && descriptor.idProduct==MCS7715_PRODUCT_ID) {
            found = device;
            break;
        }
    }
    libusb_device_handle *handle = 0;
    if (found) {
        err = libusb_open(found, &handle);
        if (err) {
            return 0;
        }
        mcs7715_startup(handle);
    }
    libusb_free_device_list(list, 1);
    return handle;
}

void mcs7715_startup(libusb_device_handle* usbdevice) {
    
    // MCS7715 only has 1 configuration
    int ret = libusb_set_configuration(usbdevice,1);
    DBG("libusb_set_configuration ret: %u\n",ret);
    ret = libusb_claim_interface(usbdevice, 0);
    DBG("libusb_claim_interface ret: %u\n",ret);
    
    /*unsigned char data;
    
    data = 0x30;
	SendMosCmd(usbdevice,MOS_READ,MOS_VEN_REG,0x04, &data);
	data = 0x80;
	SendMosCmd(usbdevice,MOS_WRITE,MOS_VEN_REG,0x04, &data);
	data = 0x00;
	SendMosCmd(usbdevice,MOS_WRITE,MOS_VEN_REG,0x04, &data);
    
    data = 0x0cc;
    SendMosCmd(usbdevice,MOS_READ,0,0x02, &data);
    data = 0x0cc;
    SendMosCmd(usbdevice,MOS_WRITE,0,0x02, &data);
    reg[1] = data; // DCR
    
    data = 0x045;
    SendMosCmd(usbdevice,MOS_READ,0,0x0A, &data);
    data = 0x045;
    SendMosCmd(usbdevice,MOS_WRITE,0,0x0A, &data);
    reg[2] = data; // ECR */
}

/*****************************************************************************
 * parport_mos7715_read_status
 *	this function will be used to read DSR Reg
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

unsigned char parport_mos7715_read_status(libusb_device_handle *dev)
{
    unsigned char data;
    SendMosCmd(dev,MOS_READ,0,0x01,&data);
    return data & 0xf8;
}

/*****************************************************************************
 * parport_mos7715_read_data
 *	this function will be used to read data from Reg0 - Data Reg
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

unsigned char parport_mos7715_read_data(libusb_device_handle *dev)
{
    unsigned char ret;
    SendMosCmd(dev,MOS_READ,0,0x00, &ret);
    return ret;
}

/*****************************************************************************
 * parport_mos7715_write_data
 *	this function will be used to write data to Reg0 - Data Reg
 * Input : 2 Input
 *			pointer to the parport,
 *			character to be written
 *****************************************************************************/

void parport_mos7715_write_data(libusb_device_handle *dev, unsigned char d)
{
    SendMosCmd(dev,MOS_WRITE,0,0x00, &d); 
}

/*****************************************************************************
 * parport_mos7715_read_control
 *	this function will be used to read DCR Reg
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

unsigned char parport_mos7715_read_control(libusb_device_handle *dev)
{
    return reg[1] & 0xf; /* Use sft cpy */
}

/*****************************************************************************
 * parport_mos7715_write_control
 *	this function will be used to write DCR Reg
 * Input : 2 Input
 *			pointer to the parport,
 *			value to be written
 *****************************************************************************/

void parport_mos7715_write_control(libusb_device_handle *dev, unsigned char d)
{
    d = (d & 0xf) | (reg[1] & 0xf0);
    SendMosCmd(dev,MOS_WRITE,0,0x02, &d);
    reg[1] = d;
}

/*****************************************************************************
 * parport_mos7715_epp_write_data
 *	this function will be used to write data in EPP mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/

size_t parport_mos7715_epp_write_data(libusb_device_handle *dev, const void *buf, size_t length, int flags)
{
	int rlen = 0;
	if (!dev)
		return 0;
    
	mos7715_change_mode(dev, ECR_EPP);
    //libusb_bulk_transfer(libusb_device_handle *dev_handle, unsigned char endpoint, unsigned char *data, int length, int *actual_length, unsigned int timeout);	
    libusb_bulk_transfer(dev, MOS_EP0_OUT, (void *)buf, length, &rlen, HZ*20);
    DBG("written %i\n",rlen);

	mos7715_change_mode(dev, ECR_PS2); 

	return rlen;
}

/*****************************************************************************
 * parport_mos7715_data_forward
 *	this function will be used to set P[0-7] to O/P mode
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/

void parport_mos7715_data_forward (libusb_device_handle *dev)
{
	unsigned char d;
	d = reg[1] & ~0x20;
	SendMosCmd(dev,MOS_WRITE,0,0x02,&d);
	reg[1] = d;
}

/*****************************************************************************
 * parport_mos7715_data_reverse
 *	this function will be used to set P[0-7] to I/P mode
 * Input : 1 Input
 *			pointer to the parport
 *****************************************************************************/
void parport_mos7715_data_reverse (libusb_device_handle *dev)
{
	unsigned char d;
	d = reg[1] | 0x20;
	SendMosCmd(dev,MOS_WRITE,0,0x02, &d);
	reg[1] = d;
}

/*****************************************************************************
 * parport_mos7715_frob_control 
 *	this function will be used to alter ECR Reg 
 * Input : 3 Input
 *			pointer to the parport,
 *			mask value,
 *			value to be written		
 *****************************************************************************/

unsigned char parport_mos7715_frob_control(libusb_device_handle* dev, unsigned char mask, unsigned char val)
{
	unsigned char d;
	mask &= 0x0f;
	val &= 0x0f;
	d = (reg[1] & (~mask)) ^ val;
	SendMosCmd(dev,MOS_WRITE,0,0x02, &d);
	reg[1] = d;
	return d & 0xf;
}

/*****************************************************************************
 * parport_mos7715_write_compat
 *	this function will be used to write data in PPF/CB-FIFO mode
 * Input : 4 Input
 *			pointer to the parport
 *			buffer
 *			length
 *			flags
 *****************************************************************************/
size_t parport_mos7715_write_compat(libusb_device_handle *dev, const void *buffer, size_t len, int flags)
{
	int i;
	int rlen;
	if (!dev)
		return 0;
	mos7715_change_mode(dev, ECR_PPF);
	i = libusb_bulk_transfer(dev, MOS_EP0_OUT, (void *)buffer, len, &rlen, HZ*20);
	mos7715_change_mode(dev, ECR_PS2);
	return rlen;
}