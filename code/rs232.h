//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      rs232.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2011-03-10 V0.02 rs232_is_break dazu
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS232
//            see http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
//-----------------------------------------------------------------


#ifndef __RS232_H__
#define __RS232_H__


/// This enum corresponds to LENZ V.3.0 (Lenz uses 19200(default) ... 115200)
typedef enum {BAUD_9600 = 0,
              BAUD_19200 = 1,
              BAUD_38400 = 2,
              BAUD_57600 = 3,
              BAUD_115200 = 4,
              BAUD_2400 = 5,        // used for Intellibox
              BAUD_4800 = 6        // used for Intellibox
              } t_baud;

// this gives a translation to integer              

extern const unsigned long baudrate[] PROGMEM;

extern t_baud actual_baudrate;

extern volatile unsigned char rs232_break_detected;    // gets true, if a break condition on rs232
                                     // is detected.
//=============================================================================
// Upstream Interface
//----------------------------------------------------------------------------
//
void init_rs232(t_baud baudrate);

bool tx_fifo_write (const unsigned char c);

bool tx_fifo_ready (void);

bool tx_all_sent (void);  // 1 if fifo is empty and all data are sent

bool rx_fifo_ready (void);

unsigned char rx_fifo_read (void);

unsigned char rs232_is_break(void);





// debug only:

#define RxBuffer_Size  64              // mind. 16
extern unsigned char RxBuffer[RxBuffer_Size];


#define TxBuffer_Size  64
extern unsigned char TxBuffer[TxBuffer_Size];

extern unsigned char rx_read_ptr;        // point to next read
extern unsigned char rx_write_ptr;       // point to next write
extern unsigned char rx_fill;
extern unsigned char tx_read_ptr;
extern unsigned char tx_write_ptr;
extern unsigned char tx_fill;

#endif  // __RS232_H__

