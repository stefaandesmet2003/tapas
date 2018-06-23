//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2008 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      rs232.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2006-04-28 V0.02 added set_led*** and timeout
//            2006-05-15 V0.03 removed early sei();
//            2006-10-17 V0.04 baudrate types for intellibox mode
//            2006-11-16 V0.05 added push_to_rx for easier simulation
//            2007-01-27 V0.06 changed to 2 stop bits to clear some 
//                             trouble with USB-to-Serial Converter
//            2007-06-09 V0.07 new function tx_all_sent
//                             reset txc on data transmission 
//            2008-04-04 V0.08 used for sniffer - runs on Atmega162,
//                             UART0; 
//            2008-04-17 V0.09 fixed a bug in init U2X was cleared
//                             unintentionally; added better cast!
//            2008-07-09 V0.10 back port; UART is now generic
//            2011-03-10 V0.11 rs232_is_break dazu
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS232
//            see also:
//            http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
//
//
//-----------------------------------------------------------------

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>

#include "config.h"               // general structures and definitions
#include "hardware.h"
#include "status.h"
#include "rs232.h"

#ifndef FALSE 
  #define FALSE  (1==0)
  #define TRUE   (1==1)
#endif

__interrupt void uartRx_isr(void);
__interrupt void uartTx_isr(void);
//=====================================================================
//
// RS232
//
// purpose:   send and receive messages from pc
//
// how:       uart acts with interrupt on fifos.
//            ohter programs access only the fifos.
//            hardware handshake with RTS and CTS.
//            display of status with one LED (see status.c)
//
// uses:      set_led_rs232
//            no_timeout.rs232 (see status.c)
// interface: see rs232.h
//
// 2do:       zur Zeit wird RTS nur als connected Erkennung benutzt,
//            das Senden wird NICHT angehalten.
//
//-----------------------------------------------------------------


// FIFO-Objekte und Puffer fï¿½r die Ein- und Ausgabe
// max. Size: 255

#define RxBuffer_Size  64              // mind. 16
unsigned char RxBuffer[RxBuffer_Size];


#define TxBuffer_Size  64              // on sniffer: 128
unsigned char TxBuffer[TxBuffer_Size];

unsigned char rx_read_ptr = 0;        // point to next read
unsigned char rx_write_ptr = 0;       // point to next write
unsigned char rx_fill = 0;
unsigned char tx_read_ptr = 0;
unsigned char tx_write_ptr = 0;
unsigned char tx_fill = 0;

// sds : wordt dit gebruikt??
const unsigned long baudrate[]  =    // ordered like in Lenz Interface!
    {9600L,   //  = 0,
     19200L,  //  = 1,
     38400L,  //  = 2,
     57600L,  //  = 3,
     115200L, //  = 4,
     2400L   //  = 5,  // used for Intellibox
	};

t_baud actual_baudrate;                      // index to field above

volatile unsigned char rs232_break_detected = 0;      // flag, that a break was detected
                                                    // volatile, weil aus ISR bearbeitet wird.

//-------------------------------------------------------------------
// this goes to usart i

void init_rs232(t_baud new_baud)
  {
    uint8_t dummy;

    // FIFOs fï¿½r Ein- und Ausgabe initialisieren
    rx_read_ptr = 0;      
    rx_write_ptr = 0;
    rx_fill = 0;      
    tx_read_ptr = 0;
    tx_write_ptr = 0;
    tx_fill = 0;

    DINT;

    // This function is found in the F2806x_Sci.c file.
    InitSciaGpio();

    //
    // 1 stop bit,  No loopback, No parity,8 char bits, async mode,
    // idle-line protocol
    //
    // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
    // Data mode 8N1, asynchron
    SciaRegs.SCICCR.all =0x0007;

    //
    // enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    //
    SciaRegs.SCICTL1.all =0x0003;

    SciaRegs.SCICTL2.bit.TXINTENA = 0; // Tx interrupts on -- voorlopig starten met off, en on als we moeten starten met zenden
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1; // Rx interrupts on

    actual_baudrate = new_baud;

    switch(new_baud)
	  {
	    default:
		  case BAUD_9600:
          // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
          SciaRegs.SCIHBAUD    =0x0001;
          SciaRegs.SCILBAUD    =0x0024;
	  	    break;
		  case BAUD_19200:
          // 19200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
          SciaRegs.SCIHBAUD    =0x0000;
          SciaRegs.SCILBAUD    =0x0091;
	  	    break;
	  }

    // interrupt configuration
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIRXINTA = &uartRx_isr;
    PieVectTable.SCITXINTA = &uartTx_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Enable CPU INT9 which is connected to SCI-A TX & RX interrupts
    IER |= M_INT9;
    // Enable SCIRXINTA & SCITXINTA in the PIE: Group 9 interrupt 1 & 2
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx2 = 1;

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset    

    // nodig?
    //SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    //SciaRegs.SCIFFRX.bit.RXFIFORESET=1;

    // Flush Receive-Buffer
    while (SciaRegs.SCIRXST.bit.RXRDY == 1)
    {
      dummy = SciaRegs.SCIRXBUF.bit.RXDT;
    }

    rs232_break_detected = 0;
    
    EINT;

} // init_rs232

// sds : moet nog weg!!
unsigned char rs232_is_break(void)
{
  return (FALSE);
}

//---------------------------------------------------------------------------
// Empfangene Zeichen werden in die Eingabgs-FIFO gespeichert und warten dort
// Wenn bis auf einen Rest von 10 gefï¿½llt ist: CTS senden.
//

__interrupt void uartRx_isr(void)
{
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  if (SciaRegs.SCIRXST.bit.FE == 1)
  {
    // sw reset nodig
    SciaRegs.SCICTL1.bit.SWRESET = 0;
    SciaRegs.SCICTL1.bit.SWRESET = 1;
    // en waarschijnlijk ook nog de RxBuffer wissen?
     rs232_break_detected = 1;      // set flag for parser and discard
     if (SciaRegs.SCIRXBUF.bit.RXDT == 0)    // this is a break sent!
     {
       rs232_break_detected = 1;      // set flag for parser and discard
     }
  }
  else
    {
      if (SciaRegs.SCIRXST.bit.OE == 1)
        { // DATA Overrun -> Fatal
          // !!!
        }
      RxBuffer[rx_write_ptr] = SciaRegs.SCIRXBUF.bit.RXDT;

      rx_write_ptr++;
      if (rx_write_ptr == RxBuffer_Size) rx_write_ptr=0;
      rx_fill++;
      if (rx_fill > (RxBuffer_Size - 10))
        {
          // we are full, stop remote Tx -> set CTS off !!!!
          // sds : not possible -> do nothing
        }
    }
} // uartRX_isr

//----------------------------------------------------------------------------
// Ein Zeichen aus der Ausgabe-FIFO lesen und ausgeben
// Ist das Zeichen fertig ausgegeben, wird ein neuer SIG_UART_DATA-IRQ getriggert
// Ist das FIFO leer, deaktiviert die ISR ihren eigenen IRQ.

// Bei 9 Bit kï¿½nnte noch ein Stopbit erzeugt werden: UCSRB |= (1<<TXB8);

//vervangt atmega328p USART_UDRE_vect
__interrupt void  uartTx_isr(void)
{
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  if (tx_read_ptr != tx_write_ptr)
    {
      SciaRegs.SCITXBUF = TxBuffer[tx_read_ptr];
      tx_read_ptr++;
      if (tx_read_ptr == TxBuffer_Size) tx_read_ptr=0;
      tx_fill--;
    }
  else
  {
    SciaRegs.SCICTL2.bit.TXINTENA = 0; // disable further TxINT
    //digitalWrite(RS485_DERE,RS485Receive); // dit is eigenlijk te vroeg, want de data zijn nog niet naar buiten geshift!! moet op de TXC int gebeuren
  }
} // uartTx_isr

//=============================================================================
// Upstream Interface
//-----------------------------------------------------------------------------
// TX:
bool tx_fifo_ready (void)
{
  if (tx_fill < (TxBuffer_Size-16))     // keep space for one complete message (16)
  {
    return(1);                        // true if enough room
  }
  else
  {
    return(0);
  }
} // tx_fifo_ready


// ret 1 if full
// slightly different from atmega328 implementation
// UDRE interrupt works as long as empty flag is set, on tms320, interrupt is triggered on writing to the SCITXBUF
// so on tms320 first write has to happen outside tx isr, rest can be done within tx isr
bool tx_fifo_write (const unsigned char c)
{
  bool retval = false;
  // no other solution for tms320 than to run the complete func with ints disabled
  // if interrupted in the else part, we might be writing in the TxBuffer, while the isr disables TXINTENA, and data will never be sent.
  
  // alt : laat TXINTENA = 1 altijd
  // if (SciaRegs.SCICTL2.bit.TXRDY == 1) --> schrijf naar SCITXBUF
  // else idem als hieronder
  // ook onder DINT; dan wordt een tx_fifo_write niet onderbroken door een tx-isr
  // na fifo_write zal de isr dan nieuwe data in de ringbuffer zien, en ze in SCITXBUF schrijven

  DINT;
  if (SciaRegs.SCICTL2.bit.TXINTENA == 0)
  {
    // rechtstreeks naar SCITXBUF schrijven
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCITXBUF = (Uint16) (c & 0xFF); // this write will trigger the tx isr, and subsequent writes can occur from within the isr
  }
  else {
    TxBuffer[tx_write_ptr] = c;

    tx_write_ptr++;
    if (tx_write_ptr == TxBuffer_Size) tx_write_ptr=0;

    tx_fill++;

    //sds not used for tms320 digitalWrite(RS485_DERE,RS485Transmit);

    //if (tx_fill < (TxBuffer_Size-16)) //sds : dit is toch niet juist??? enfin, retval wordt toch niet gebruikt
    if (tx_fill > (TxBuffer_Size-16))
      {
        retval = true;
      }
  }
  EINT;
  return (retval);
} // tx_fifo_write

// ret 1 if all is sent
bool tx_all_sent (void)
{
  if (tx_fill == 0)
    {
      if (SciaRegs.SCICTL2.bit.TXRDY == 0) return (0); // SCITXBUF not empty
      if (SciaRegs.SCICTL2.bit.TXEMPTY == 0) return (0); // data not completely shifted out on the pin
      return(1);
    }
  else
    {
      return(0);
    }
} // tx_all_sent

void uart_puts (const char *s)
{
    do
      {
        tx_fifo_write (*s);
      }
    while (*s++);
}


//------------------------------------------------------------------------------
// RX:
bool rx_fifo_ready (void)
{
    if (rx_read_ptr != rx_write_ptr)
      {
        return(1);     // there is something
      }
    else
      {
        return(0);  
      }
}

//-------------------------------------------------------------------
// rx_fifo_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();

unsigned char rx_fifo_read (void)
{
  unsigned char retval;

  retval = RxBuffer[rx_read_ptr];
  rx_read_ptr++;
  if (rx_read_ptr == RxBuffer_Size) rx_read_ptr=0;

  DINT;
  rx_fill--;
  EINT;
  
  // forget CTS control in this implementation

  return(retval);
} // rx_fifo_read

