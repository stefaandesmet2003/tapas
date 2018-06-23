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
// file:      ibox_programmer.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-12-04 V0.2 change to multitasking

// 2do:       not completed
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   builds service mode

// extern t_lenz_result lenz_result;        // This stores the last programming result
extern unsigned int prog_loco;           // last loco (for pom);
extern unsigned int prog_cv;             // last cv;
extern unsigned char prog_data;          // last data;

extern unsigned char ps_bitpos;        // Bitpos bzw. lokaler Programmschritt 

extern unsigned char prog_result_size;

typedef struct 
  {
    unsigned char bidi_pending: 1; // if 1: bidi sent, but not read -> set by parser, cleared by XevtPT
    
    unsigned char result_bidi: 1;  // if 1: result generated -> set by xpressnet, cleared by parser
    
    unsigned char result: 1;       // if 1: result generated -> set by programmer, cleared by parser
    
    unsigned char busy: 1;         // if 1: we are running (set by enter_progmode, cleared by PS_idle)
  } t_prog_event;

extern t_prog_event prog_event; 

typedef enum
  {
    PQ_REGMODE      = 0x10,     // register mode
    PQ_CVMODE_B0    = 0x14     // cv mode 1-255
  } t_prog_qualifier;

extern t_prog_qualifier prog_qualifier;

typedef enum
  {
    PT_OKAY     = 0x00,     // Command completed, no errors
    PT_TIMEOUT  = 0xFF,     // Timeout
    PT_NOACK    = 0xFE,     // No acknowledge from decoder (but a write maybe was successful)
    PT_SHORT    = 0xFD,     // Short! (on the PT)
    PT_NODEC    = 0xFC,     // No decoder detected
    PT_ERR      = 0xFB,     // Generic Error
    PT_BITERR   = 0xFA,     // Error during DCC direct bit mode operation
    PT_PAGERR   = 0xF9,     // No acknowledge to paged operation (paged r/w not supported?)
    PT_SELX     = 0xF8,     // Error during Selectrix read
    PT_DCCQD_Y  = 0xF7,     // XPT_DCCQD: Ok (direct bit read mode is (probably) supported)
    PT_DCCQD_N  = 0xF6,     // XPT_DCCQD: Not Ok (direct bit read mode is (probably) not supported)
    PT_TERM     = 0xF4,     // Task terminated (see XPT_Term cmd)
    PT_NOTASK   = 0xF3,     // No task to terminate (see XPT_Term cmd)
    PT_NOTERM   = 0xF2     // Cannot terminate task (see XPT_Term cmd)
  } t_prog_result;

extern t_prog_result prog_result;


void init_programmer(void);
void run_programmer(void);
void enter_progmode(void);
void leave_progmode(void);      // switch to run_off
void reset_programmer(void);    // reset any running programmer task

unsigned char programmer_busy(void);

unsigned char my_XPT_DCCRR (unsigned int cv);                                           // ec, registered
unsigned char my_XPT_DCCWR (unsigned int cv, unsigned char data);                       // ed, registered
unsigned char my_XPT_DCCRP (unsigned int cv);                                           // ee, paged
unsigned char my_XPT_DCCWP (unsigned int cv, unsigned char data);                       // ef, paged
unsigned char my_XPT_DCCRD (unsigned int cv);                                           // f0, direct
unsigned char my_XPT_DCCWD (unsigned int cv, unsigned char data);                       // f1, direct
unsigned char my_XPT_DCCRB (unsigned int cv);                                           // f2, bit
unsigned char my_XPT_DCCWB (unsigned int cv, unsigned char bitpos, unsigned char data); // f3, bit
unsigned char my_XPT_DCCQD (void);                                                      // f4
unsigned char my_XPT_DCCRL (void);                                                      // f5, read long addr
unsigned char my_XPT_DCCWL (unsigned int cv);                                           // f6, write long

unsigned char my_XPT_Term (void);


