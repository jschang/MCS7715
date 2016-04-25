//
//  pic87f8xjxx.c
//  MCS7715
//
//  Created by Jonathan Schang on 3/20/15.
//  Copyright (c) 2015 Jon Schang. All rights reserved.
//

#include "pic87f8xjxx.h"
#include <time.h>   // nanosleep
#include <unistd.h> // usleep
#include <math.h>   // floor
#include <stdio.h>  // printf
#include <string.h> // strcat 

#define DEBUG_OUT 1
#if DEBUG_OUT==1
    #define DBG(...) printf(__VA_ARGS__)
#else
    #define DBG(...) 
#endif

#define PGD  1<<0
#define PGC  1<<1
#define MCLR 1<<2
#define VDD  1<<3

#define PGC_ON(p)   p->settings = PGC  | p->settings
#define PGC_OFF(p)  p->settings = (0xff ^ PGC) & p->settings
#define PGD_ON(p)   p->settings = PGD  | p->settings
#define PGD_OFF(p)  p->settings = (0xff ^ PGD) & p->settings
#define MCLR_ON(p)  p->settings = MCLR | p->settings
#define MCLR_OFF(p) p->settings = (0xff ^ MCLR) & p->settings
#define VDD_ON(p)   p->settings = VDD  | p->settings
#define VDD_OFF(p)  p->settings = (0xff ^ VDD)  & p->settings

#define NS_PER_MS 1000000
#define NS_PER_US 1000

#define TIME_PAD  15

#define TIME_P2   100 // serial clock period (pgc)
#define TIME_P2A  40  // pgc low time
#define TIME_P2B  40  // pgc high time
#define TIME_P3   15
#define TIME_p4   15
#define TIME_P5   40
#define TIME_P5A  40  // delay between 4-bit cmd op and next 4-bit cmd op
#define TIME_P6   20  // delay between last pgc low of command byte 
                      // and first pgc high of read of data word
#define TIME_P9   3.4 * NS_PER_MS // delay to allow block programming to occur
                                  // value is 1.2 for some processors, so 
                                  // we'll just use the lowest common 
                                  // denominator
#define TIME_P10  49  * NS_PER_MS // delay to allow row erase to occur
#define TIME_P11  475 * NS_PER_MS // delay to allow bulk erase to occur
#define TIME_P12  400 * NS_PER_US // input data hold time from mclr high
#define TIME_P13  100 * NS_PER_US // cdd high setup time to mclr
#define TIME_P14  10              // data out valid from pgc high
#define TIME_P16  20              // delay between last pgc low and mclr low
#define TIME_P19  1 * NS_PER_MS   // delay from first mclr low to first pgc high 
                                  // for key seq on pgd
#define TIME_P20  40              // delay from last pgc low for key seq on pgd
                                  // to second mclr
                                  
inline static const char * byte_to_binary(unsigned char x)
{
    static char b[9];
    bzero(b,9);
    int z;
    for (z = 128; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

static inline void __sleep(long nanos) {
    struct timespec req, ret;
    req.tv_nsec = nanos + TIME_PAD;
    req.tv_sec = 0;
    nanosleep(&req, &ret);
    //TODO: protect this...actually care about the result
}

static inline void __update_icsp(icsp_port* port) {
    mos7715_write_dpr(port->lpt_usb_device_handle, port->settings);
    unsigned char dsr = mos7715_read_dsr(port->lpt_usb_device_handle);
    port->ack = (dsr & 0b01000000) == 0b01000000;
}

static inline unsigned char __read_bit(icsp_port *port) {
    // clock high, wait
    PGC_ON(port);
    PGD_OFF(port);
    __update_icsp(port);
    __sleep(TIME_P2B);
    
    // clock low and wait
    PGC_OFF(port);
    __update_icsp(port);
    
    // read bit, latched on the falling edge of pgc
    unsigned char bit = port->ack & 1;
    DBG("%i ",bit);
    __update_icsp(port);
    __sleep(TIME_P2A);
    
    return bit;
}

static inline unsigned char __read_byte(icsp_port *port) {
    unsigned char byte = 0;
    DBG("\tread ");
    for( int i=0; i<8; i++ ) {
        byte = (__read_bit(port) << i) | byte;
    }
    DBG("\n");
    return byte;
}

static inline void __send_bit(unsigned char bit, icsp_port* port) {
    PGC_ON(port);
    DBG("%i ",bit);
    if(bit) {
        PGD_ON(port);
    } else {
        PGD_OFF(port);
    }
    //DBG("(%s)->",byte_to_binary(port->settings));
    __update_icsp(port);
    //DBG("(%s) ",byte_to_binary(port->settings));
    __sleep(TIME_P2B);
    PGC_OFF(port);__update_icsp(port);
    __sleep(TIME_P2A);
}

static inline void __send_bits(unsigned long long bits, int count, icsp_port* port) {
    DBG("\tsending lsb 0x%llX ",bits);
    for(int i=0; i<count; i++) {
        // send lsb to msb
        __send_bit(bits >> i & 1, port);
    }
    DBG("\n");
}

static inline void __send_bits_msb(unsigned long long bits, int count, icsp_port* port) {
    DBG("\tsending msb 0x%llX ",bits);
    for(int i=(count-1); i>=0; i--) {
        // send msb to lsb
        __send_bit(bits >> i & 1, port);
    }
    DBG("\n");
}

static inline unsigned char __send_cmd(unsigned char cmd, unsigned int payload, icsp_port* port) {
    //DBG("sending cmd 0x%X 0x%X\n",cmd,payload);
    unsigned char ret = 0;
    __send_bits_msb((unsigned long long)cmd,4,port);
    __sleep(TIME_P5);
    if(cmd==0) {
        for(int i=0; i<4; i++) {
            // send a nibble at a time, lsn to msn
            __send_bits((unsigned long long)(payload >> (i*4)) & 0xf, 4, port);
        }
    } else if(cmd==0b1001) {
        __send_bits(0, 4, port);
        __send_bits(0, 4, port);
        __sleep(TIME_P6);
        ret = __read_byte(port);
    }
    __sleep(TIME_P5A);
    return ret;
}

static inline void __set_tblptr(unsigned long addr, icsp_port* port) {
    // movlw addr[21:16]
    unsigned long long cmd = ((0x0e<<8) | ((addr >> 16) & 0b111111));
    DBG("movlw addr[21:16] = %s\n",byte_to_binary(cmd));
    __send_cmd( 0, cmd, port);
    __send_cmd( 0, 0x6ef8, port); // movwf tblptru
    
    // movlw addr[15:8]
    cmd = (unsigned long long)((0x0e<<8) | ((addr >> 8) & 0b11111111));
    DBG("movlw addr[15:8] = %s\n",byte_to_binary(cmd));
    __send_cmd( 0, cmd, port);
    DBG("movwf tblptrh\n");
    __send_cmd( 0, 0x6ef7, port);
    
    // movlw addr[7:0]
    cmd = (unsigned long long)((0x0e<<8) | (addr & 0b11111111));
    DBG("movlw addr[7:0] = %s\n",byte_to_binary(cmd));
    __send_cmd( 0, cmd, port);
    DBG("movwf tblptrl\n");
    __send_cmd( 0, 0x6ef6, port); // movwf tblptrl
}

int icsp_get(icsp_port *port) {
    port->lpt_usb_device_handle = 0;
    port->ack = 0;
    port->settings = 0;
    libusb_device_handle *devHandle = mcs7715_find_device();
    if(!devHandle) {
        DBG("device not plugged in\n");
        return 0;
    }
    mos7715_change_mode(devHandle,0);
    port->lpt_usb_device_handle = devHandle;
    __update_icsp(port);
    DBG("got icsp port\n");
    return 1;
}

void icsp_enter_mode(icsp_port* port) {
    port->settings = 0;
    PGD_OFF(port);
    PGC_OFF(port);
    VDD_ON(port);
    __update_icsp(port);__sleep(TIME_P13);
    MCLR_ON(port);__update_icsp(port);__sleep(TIME_P19);
    MCLR_OFF(port);__update_icsp(port);__sleep(TIME_P19);
    unsigned long long bits = 0b01001101010000110100100001010000;
    int count = 32;
    DBG("sending ");
    for(int i=(count-1); i>=0; i--) {
        unsigned char bit = bits >> i & 1;
        DBG("%i",bit);
        if(bit == 1) {
            PGD_ON(port);
        } else {
            PGD_OFF(port);
        }
        PGC_OFF(port);__update_icsp(port);__sleep(TIME_P2B);
        PGC_ON(port);__update_icsp(port);__sleep(TIME_P2A);
    }
    PGC_OFF(port);__update_icsp(port);
    DBG("\n");
    __sleep(TIME_P20);
    MCLR_ON(port);__update_icsp(port);__sleep(TIME_P12);
}

void icsp_exit_mode(icsp_port* port) {
    PGD_OFF(port);
    PGC_OFF(port);__update_icsp(port);__sleep(TIME_P16);
    MCLR_OFF(port);__update_icsp(port);__sleep(TIME_P16);
    VDD_OFF(port);__update_icsp(port);
}

unsigned int icsp_read_memory(unsigned long addr, unsigned char* buffer, int len, icsp_port* port) {

    __set_tblptr(addr, port);
    // tblrd *+
    for(int i=0; i<len; i++) {
        DBG("tblrd *+\n");
        unsigned char ret = __send_cmd( 0b1001, 0, port); 
        DBG("read 0x%X %s ",ret,byte_to_binary(ret));
        DBG("at addr 0x%lX\n",addr);
        buffer[i] = ret;
        addr++;
    }
    return 0;
}

void icsp_bulk_erase(icsp_port* port) {
    
    __set_tblptr(0x3c0005,port);
    DBG("Write 01h to TBLPTR @ 0x3C0005h");
    __send_cmd(0b1100,0x0101,port);
    __set_tblptr(0x3c0004,port);
    DBG("Write 80h TO 3C0004h to erase entire device.\n");
    __send_cmd(0b1100,0x8080,port);
    DBG("NOP\n");
    __send_cmd(0b0000,0x0000,port);
    DBG("NOP\n");
    __send_cmd(0b0000,0x0000,port);
    __sleep(TIME_P11);
}

unsigned int icsp_write_memory(unsigned long addr, unsigned char* buffer, int len, icsp_port* port) {
    return 0;
}