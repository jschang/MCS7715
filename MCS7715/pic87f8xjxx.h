//
//  pic87f8xjxx.h
//  MCS7715
//
//  Created by Jonathan Schang on 3/20/15.
//  Copyright (c) 2015 Jon Schang. All rights reserved.
//

#ifndef __MCS7715__pic87f8xjxx__
#define __MCS7715__pic87f8xjxx__

#include "mcs7715.h"

typedef struct {
    libusb_device_handle *lpt_usb_device_handle;
    unsigned char settings;
    unsigned char ack;
} icsp_port;

int icsp_get(icsp_port *port);
void icsp_enter_mode(icsp_port* port);
void icsp_exit_mode(icsp_port* port);
unsigned int icsp_read_memory(unsigned long addr, unsigned char* buffer, int len, icsp_port* port);

#endif /* defined(__MCS7715__pic87f8xjxx__) */
