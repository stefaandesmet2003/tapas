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
// file:      dccout.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-06-10 V0.2 added volatile, damit der gnu nicht auf
//                            register optimiert.
//            2008-08-29 V0.3 railcom
//            2009-06-23 V0.4 MAX_DCC_SIZE
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   ISR for DCC-OUT
//
//-----------------------------------------------------------------


struct next_message_s
  {
    unsigned char size;
    t_msg_type    type;
    unsigned char dcc[MAX_DCC_SIZE];
  };

extern struct next_message_s next_message;

extern unsigned char next_message_count;     // load message and set count
                                               // if > 1 -> output next_message
											   // if = 0 -> ready for next_message

void init_dccout(void);                      // call once at boot up
void dccout_enable_cutout(void);             // create railcom cutout
void dccout_disable_cutout(void);
unsigned char dccout_query_cutout(void);

void dcc_main_track_on(void);
void dcc_main_track_off(void);
void dcc_prog_track_on(void);
void dcc_prog_track_off(void);
unsigned char dcc_main_track_state(void);
unsigned char dcc_prog_track_state(void);


