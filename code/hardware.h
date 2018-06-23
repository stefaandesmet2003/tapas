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
// file:      hardware.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2007-03-27 V0.01 copied from config.h,
//                             all removed but hardware accesses.
//            2008-07-09 V0.02 added ATmega644P
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   central definitions used in the project
//            all hardware project settings are done here!!
//            
//            1.   Prozessor, Timing
//                 (change here, if OpenDCC is run on a different uP)
//            2.   IOs
//                 (change here, if OpenDCC is run on a different board)
//
//-----------------------------------------------------------------
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#define PROGMEM
//========================================================================
// 2. Port Definitions
//========================================================================
//SDS port A is niet aanwezig op arduino platform!! keuze maken in I/O!!
//voorlopig alle port A defines commenten

//SDS 201610: definities vertaald naar arduino pins, ipv 0..7 op een poort

#define ROTENC_CLK      2     // D2 (INT0),in, draaiknop CLK
#define ACK_DETECTED    3     // D3 (INT1), in    this is only a short pulse -> use int!
#define RS485_DERE      4     // D4, OUT (RS485 CTRL)
#define NDCC_OK         5     // SDS --> dit signaal bestaat nog niet in OpenDCC!!
// D5 vrij
#define ROTENC_DT       6     // D6,in, draaiknop DT
#define ROTENC_SW       7     // D7,in, drukknop op de rotary enc
#define BUTTON_GREEN    8     // input, SDS D8 
#define DCC             9     // out,sds D9
#define NDCC            10     // out,sds D10
#define BUTTON_RED      11     // input, SDS D11
//D12 vrij --> button 3 & 4 voorzien!!
//D13 vrij

#define NSHORT_PROG     14     // in,sds A0
#define NSHORT_MAIN     15     // in,sds A1
#define SW_ENABLE_MAIN  16     // out,sds A2  high = enable main
#define SW_ENABLE_PROG  17     // out,sds A3  high = enable programming
//PC4 = A4 = SDA
//PC5 = A5 = SCL
#define EXT_STOP        20     // in,sds A6
// SDS, A7 is voorlopig vrij

#define RS485Transmit    HIGH
#define RS485Receive     LOW

//SDS #define MY_CTS_HIGH      PORTB |= (1<<MY_CTS)
//SDS #define MY_CTS_LOW       PORTB &= ~(1<<MY_CTS)
//SDS #define MY_CTS_STATE     (PORTB & (1<<MY_CTS))
#define MY_CTS_HIGH //SDS, nog opkuisen in rs232.cpp
#define MY_CTS_LOW //SDS, nog opkuisen in rs232.cpp
#define MY_CTS_STATE 0 //SDS, nog opkuisen in rs232.cpp

/*
#define MAIN_TRACK_ON    digitalWrite(SW_ENABLE_MAIN,HIGH)
#define MAIN_TRACK_OFF   digitalWrite(SW_ENABLE_MAIN,LOW)
#define MAIN_TRACK_STATE (digitalRead(SW_ENABLE_MAIN))
#define PROG_TRACK_ON    digitalWrite(SW_ENABLE_PROG,HIGH)
#define PROG_TRACK_OFF   digitalWrite(SW_ENABLE_PROG,LOW)
#define PROG_TRACK_STATE (digitalRead(SW_ENABLE_PROG))
*/
// DO is inverting on TAPAS
#define MAIN_TRACK_ON     (GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1)
#define MAIN_TRACK_OFF    (GpioDataRegs.GPBSET.bit.GPIO40 = 1)
#define MAIN_TRACK_STATE  (!GpioDataRegs.GPBDAT.bit.GPIO40)
#define PROG_TRACK_ON    
#define PROG_TRACK_OFF   
#define PROG_TRACK_STATE  (0)

//sds 201611 : NMAIN_SHORT, NPROG_SHORT, zijn active low
//sds 201611 : ACK_DETECTED active high
//sds 201610 : de keys zijn active low aangesloten

/*
#define MAIN_IS_SHORT    (digitalRead(NSHORT_MAIN)==LOW)
#define PROG_IS_SHORT    (digitalRead(NSHORT_PROG)==LOW)
#define ACK_IS_DETECTED  (digitalRead(ACK_DETECTED)==HIGH)
#define EXT_STOP_ACTIVE  (analogRead(EXT_STOP)<512)  // A6 is analog-only pin, digitalRead always returns 0!!
*/

#define MAIN_IS_SHORT    (GpioDataRegs.GPBDAT.bit.GPIO54 == 1)
// TAPAS : not implemented in demo
#define PROG_IS_SHORT    0
#define ACK_IS_DETECTED  0
#define EXT_STOP_ACTIVE  0

#endif   // hardware.h

