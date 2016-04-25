//
//  main.c
//  MCS7715
//
//  Created by Jonathan Schang on 2/28/15.
//  Copyright (c) 2015 Jon Schang. All rights reserved.
//

#include <stdio.h>
#include <string.h>     /* strcat */
#include <stdlib.h>     /* strtol */
#include <unistd.h>     /* sleep */
#include <libusb-1.0/libusb.h>
#include "mcs7715.h"
#include "pic87f8xjxx.h"

void print_configurations(libusb_device_handle* devHandle);
const char *byte_to_binary(int x);

void validate_pins() {
    icsp_port port;
    icsp_get(&port);
    for(int i=0; i<4; i++) {
        printf("sending %s\n",byte_to_binary(1<<i));
        mos7715_write_dpr(port.lpt_usb_device_handle,1<<i);
        printf("enter to continue");getchar();
    }
}

void validate_ack() {
    icsp_port port;
    icsp_get(&port);
    mos7715_write_dpr(port.lpt_usb_device_handle,0);
    int dsr = mos7715_read_dsr(port.lpt_usb_device_handle);
    printf("%s",byte_to_binary(dsr));
}

void test_icsp() {

    icsp_port port;
    icsp_get(&port);
    
    icsp_enter_mode(&port);
    
    unsigned char buffer[256];
    bzero(buffer,256);
    
    // device id
    icsp_read_memory(0x3FFFFE, buffer, 2, &port);
    
    // configuration words
    //icsp_read_memory(0x300000, buffer, 8, &port);
    
    icsp_exit_mode(&port);
}

void test_tlvp() {
    libusb_device_handle *devHandle = mcs7715_find_device();
    if(!devHandle) {
        printf("device not plugged in\n");
        exit(1);
    }
    mos7715_change_mode(devHandle,0);
    unsigned char data[4];
    read_status(devHandle, data);
    do {
        for(int i=0;i<5;i++) {
            if(i==4) {
                mos7715_write_dpr(devHandle,0b11111111);
            } else {
                mos7715_write_dpr(devHandle,1<<i);
                mos7715_read_dsr(devHandle);
                mos7715_read_dcr(devHandle);
            }
            usleep(150000);
        }
    } while(1);
}

const char *byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';
    
    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    
    return b;
}

void print_configurations(libusb_device_handle* devHandle) {

    libusb_device *dev = libusb_get_device(devHandle);
    struct libusb_device_descriptor desc;
    
    libusb_get_device_descriptor(dev, &desc);
    printf("%u configs\n",desc.bNumConfigurations);
    for(int i=0;i<desc.bNumConfigurations;i++) {
        
        struct libusb_config_descriptor *config;
        libusb_get_config_descriptor(dev, i, &config);
        printf("\tconfig %u has %u ifaces\n",desc.bNumConfigurations,config->bNumInterfaces);
        printf("\tconfig %u identified by %u\n",i,config->bConfigurationValue);
        
        for(int j=0; j<config->bNumInterfaces; j++) {
            
            struct libusb_interface_descriptor* iface;
            printf("\t\tconfig %u, iface %u has %u 1 altsetting\n",i,j,config->interface[j].num_altsetting);
            
            for(int k=0; k<config->interface[j].num_altsetting; k++) {
                
                iface = &config->interface[j].altsetting[k];
                printf("\t\t\tiface %u, alt setting %u, has %u endpoints\n",j,k,iface->bNumEndpoints);
                
                for(int t=0; t<iface->bNumEndpoints; t++) {
                    
                    printf("\t\t\t\tendpoint %u address is %s\n",t,byte_to_binary(iface->endpoint[t].bEndpointAddress));
                    printf("\t\t\t\t\tdirection is ");
                    uint16_t direction = iface->endpoint[t].bEndpointAddress & 0b10000000;
                    switch(direction) {
                        case LIBUSB_ENDPOINT_IN:  printf("in\n");  break;
                        case LIBUSB_ENDPOINT_OUT: printf("out\n"); break;
                    }
                    printf("\t\t\t\tendpoint %u descriptor type is %s\n",t,byte_to_binary(iface->endpoint[t].bDescriptorType));
                    //Bits 0:1 determine the transfer type and correspond to libusb_transfer_type.
                    //Bits 2:3 are only used for isochronous endpoints and correspond to libusb_iso_sync_type.
                    //Bits 4:5 are also only used for isochronous endpoints and correspond to libusb_iso_usage_type.
                    //Bits 6:7 are reserved.
                    printf("\t\t\t\tendpoint %u attributes are %s\n",t,byte_to_binary(iface->endpoint[t].bmAttributes));
                    printf("\t\t\t\t\ttransfer type is: ");
                    uint16_t transferType = iface->endpoint[t].bmAttributes & 0b11;
                    switch(transferType) {
                        case LIBUSB_TRANSFER_TYPE_CONTROL: printf("control\n"); break;
                        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: printf("isochronous\n"); break;
                        case LIBUSB_TRANSFER_TYPE_BULK: printf("bulk\n"); break;
                        case LIBUSB_TRANSFER_TYPE_INTERRUPT: printf("interrupt\n"); break;
                        default: printf("unknown\n");
                    }
                    printf("\n");
                }
            }
        }
        libusb_free_config_descriptor(config);
    }
}

int main(int argc, const char * argv[]) {
    //validate_pins();
    test_icsp();
}