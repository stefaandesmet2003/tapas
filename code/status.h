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
// file:      status.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2007-10-15 V0.02 key_go_at_start_pressed neu dazu.
//            2009-06-23 V0.03 added clock to status event
//            2009-06-23 V0.04 added read_occ to status event
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   control of LED
//            timertick
//            check for key and output short
//
// 2do:       
//-----------------------------------------------------------------
#ifndef __STATUS_H__
#define __STATUS_H__

void init_state(void);          // start up isr and timer0


typedef struct       // each member will be decremented on every tick until 0
  {
    unsigned int parser;       // high level parser timeout
  } t_no_timeout;

extern t_no_timeout no_timeout;
extern t_opendcc_state opendcc_state;            // this is the current state of the box

// Interface for OpenDCC-Parser

typedef struct 
  {
    unsigned char changed: 1;       // if != 0: there was (could be) a state change
                                    //        set by set_opendcc_state - cleared by host parser
    unsigned char changed_xp: 1;    // mirror for Xpressnet
                                    //        set by set_opendcc_state - cleared by xpressnet master
    unsigned char power_off: 1;     // there was a power off event, power has been turned off (not *is* off)
                                    //        set by set_opendcc_state - cleared by host parser (ib)
    unsigned char clock: 1;         // there was a minute tick for DCC Layout time
                                    //        set by fast_clock - cleared by xpressnet master
    unsigned char read_occ: 1;      // occupancy detector should resend data
                                    //        set by host parser - cleared by xpressnet master
  } t_status_event;

extern t_status_event status_event;         // Message flag: there was a change in status

// ---------------------------------------------------------------------------------
// fast clock record

extern t_fast_clock fast_clock;



//===================================================================================
// Interface-Routines to other programs
// Alle Zeiten in us

void set_opendcc_state(t_opendcc_state next);
void run_state(void);
unsigned char is_prog_state(void);
unsigned char is_power_on(void);

extern unsigned char ext_stop_enabled;
typedef enum {HWEVENT_MAINSHORT, HWEVENT_PROGSHORT, HWEVENT_EXTSTOP, HWEVENT_NODCC} hwEvent_t;
extern void hwEvent_Handler( hwEvent_t event ) __attribute__ ((weak));

#endif // __STATUS_H__

