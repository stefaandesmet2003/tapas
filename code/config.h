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
// file:      config.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//-------------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   central definitions used in the project
//            all major project settings are done here!!
//            
//            1.   Prozessor, Timing
//                 (change here, if OpenDCC is run on a different uP)
//            2.   IOs
//                 (change here, if OpenDCC is run on a different board)
//            3.   System Defintions and default baudrate
//                 (select here, what system to build (Intellibox, Lenz, S88, DMX)
//            4.   DCC Definitions (do not change)
//            4.a) defines for handling of DCC
//                 (user definitions like loco formats and buffer sizes)
//            4.b) defines for handling of S88
//            4.c) variable definitions
//            5.   Usage of Memory, EEROM and Registers
//                 (change here for number of locos, turnouts, virtual decoders...)
//
//--------------------------------------------------------------------------------
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

typedef unsigned char   uint8_t;
//typedef unsigned char   bool;

#define OPENDCC_SUBVERSION   9
#define OPENDCC_VERSION     23
#define OPENDCC_SERIAL       1     // This is a serial number used to identify different boxes.
#define BUILD_YEAR          12
#define BUILD_MONTH         01
#define BUILD_DAY           27
 

#define DEBUG  0            // Debugswitches:  0: no debugging, interrupts running -> alive!
                            //                 1: no ints, some lowlevel test
                            //                 2: command_organizer tests
                            //                 3: IB or Lenz parser tests, see main.c




#define EEPROM_FIXED_ADDR 0         // if 1: use indivual EEPROM section for Config
                                    // Locos and DMX
                                    // Note: keep at 0, there is bug in AVR Studio
                                    // Atmel is still working on the fix 

#include "hardware.h"               // right now, this includes the definition
                                    // for the hardware on www.opendcc.de


// SDS-TAPAS
Uint32 millis(void);
Uint32 micros(void);
void millis_init();

//------------------------------------------------------------------------
// Timing Definitions (all values in us)
//------------------------------------------------------------------------
//
// same macro as in util/delay.h, but use up to some ms.
// The maximal possible delay is 262.14 ms / F_CPU in MHz.
// This is 16ms for 16MHz; longest used delay: 1000us

// sds-tapas; us-delay needed for programmer
#define _mydelay_us DSP28x_usDelay
/*
#ifndef _UTIL_DELAY_H_
  #include <util/delay.h>
#endif

static inline void _mydelay_us(double __us) __attribute__((always_inline));
void
_mydelay_us(double __us)
{
    uint16_t __ticks;
    double __tmp = ((F_CPU) / 4e6) * __us;
    if (__tmp < 1.0)
        __ticks = 1;
    else if (__tmp > 65535)
        __ticks = 0;    // i.e. 65536
    else
        __ticks = (uint16_t)__tmp;
    _delay_loop_2(__ticks);
}

*/

/// This is the timer tick for timeouts, LEDs, key-debounce ...

//#define TICK_PERIOD                 5000L   // 5ms = 5000us (valid range: 2000-8000)
                                            // if changed: see all .c files and
                                            // check possible range overflows in definitions!


#define TIMER2_TICK_PERIOD          4L      // 4us

#define MAIN_SHORT_DEAD_TIME        8L      // wait 15ms before turning off power after
                                            // a short is detected on outputs
                                            // this is the default - it goes to CV34.

#define PROG_SHORT_DEAD_TIME        40L     // wait 40ms before turning off power after
                                            // a short is detected on outputs
                                            // this is the default - it goes to CV35.

#define POM_TIMEOUT                 500L    // Time until we consider a PoM read as failed (SDS: waar wordt dit gebruikt?nergens??)
#define EXT_STOP_DEAD_TIME          30L     // sds, default value for eadr_ext_stop_deadtime (CV37)

//==========================================================================================
// 3. System Definitions
//==========================================================================================
// Here You find the defines what system to build
//      (may be altered by user)
//------------------------------------------------------------------------------------------

#define LENZ                1
#define INTELLIBOX          2
#define SDS_BOX             3 //sds, serial.print versie

#define NAMED               1
#define UNNAMED             2
#define TRAINCTRLR          3


#define PARSER                  LENZ // INTELLIBOX  // LENZ: behave like a LI101
                                            // INTELLIBOX: behave like a IBOX
                                            // note: not all commands are supported
                                            //       see ibox_parser.c for a detailed
                                            //       list.

#define BIDI_SUPPORT            0           // 0: no special BIDI-support
                                            // 2: add parameter in locobuffer for location, speed, and bidi status

#define BIDI_TEST               0           // 0: standard command station
                                            // 1: special test commands (not for production)
                                            //    bei DCC-Schalten auf Adresse 1000 wird eine CS-Unique-ID Nachricht rausgesendet 

#define RAILCOMPLUS_SUPPORT     0           // 0: no special code
                                            // 1: add test code

#define LOCO_DATABASE           UNNAMED       // NAMED    containing Addr, Format, Names, PictureID
                                            // UNNAMED  containing Addr, Format, (no Names)

#define LOCO_DATABASE_MSG       TRAINCTRLR  // set p50xb answers according to traincontroller

#define LOCO_DATABASE_XMT       TRAINCTRLR  // use accessory command 'on' at virtual decoder to initiate 
                                            // loco data transfer 

#define LOCO_DATABASE_ACC_ADDR  2040        // in terms of IB: counting from 1


#define LOK_NAME_LENGTH         10          // no of char; multimaus uses 5
                                            // XP has a range of 1 to 10; more than 10 would break XP size.
                                            // these are the characters without any trailing 0

//SDS : voorlopig zonder xpressnet, want we hebben geen 2de uart
#define XPRESSNET_ENABLED       0           // 0: classical OpenDCC


#if (PARSER == LENZ)
  #define DEFAULT_BAUD      BAUD_19200      // supported: 2400, 4800, 9600, 19200, 38400, 57600, 115200         
#endif

#define DCC_FAST_CLOCK              1       // 0: standard DCC
                                            // 1: add commands for DCC fast clock

//=========================================================================================
// 4. DCC Definitions
//=========================================================================================
// 4.a) defines for handling of DCC
//      (may be altered by user)
//------------------------------------------------------------------------

#define DCC_DEFAULT_FORMAT     DCC28    // may be DCC14, DCC28, DCC128               (--> CV)
                                        // This Format is reported if loco was never used
                                        // before; is loco was used once, the previous
                                        // format is reported.
                                        // In case of IB-Control:
                                        // This Format is used, when a speed command is given
                                        // to a loco never used before.

#define DCC_F13_F28            1        // 1: add code for functions F13 up to F28

#define RAILCOM_ENABLED        0        // 1: add code to enable RailCom, 

#define DCC_SHORT_ADDR_LIMIT   112      // This is the maximum number for short addressing mode on DCC
#define XP_SHORT_ADDR_LIMIT    99       // This is the maximum number for short addressing mode on Xpressnet


#define NUM_DCC_SPEED_REPEAT   3        // Speed commands are repeated this number   (--> CV)
                                        // + one extra repetition if speed is decreased

#define NUM_DCC_ACC_REPEAT     2        // Accessory Commands are repeated this number (--> CV)

#define NUM_DCC_FUNC_REPEAT    0        // Function Commands are repeated this number (--> CV)

#define NUM_DCC_POM_REPEAT     3        // Program on the main are repeated this number (--> CV)

// note: in addition, there is the locobuffer, where all commands are refreshed
//       this locobuffer does not apply to accessory commands nor pom-commands

//-----------------------------------------------------------------------
// Sizes of Queues and Buffers -> see section 5, memory usage
//
//------------------------------------------------------------------------
// 4.b) variable defines for the state engine
//------------------------------------------------------------------------

typedef enum {DISCONNECTED, CONNECTED, RUNNING, STOPPED} t_rs232_led_state;

typedef enum {INIT,
              RUN_OKAY,     // DCC Running
              RUN_STOP,     // DCC Running, all Engines Emergency Stop
              RUN_OFF,      // Output disabled (2*Taste, PC)
              RUN_SHORT,    // Kurzschluss;
              RUN_PAUSE,    // DCC Running, all Engines Speed 0
              PROG_OKAY,    // Prog-Mode running 
              PROG_SHORT,   // Prog-Mode + Kurzschluss
              PROG_OFF,     // Prog-Mode, abgeschaltet
              PROG_ERROR    // Prog-Mode, Fehler beim Programmieren
              } t_opendcc_state;

//------------------------------------------------------------------------
// define a structure for DCC messages inside OpenDCC
//------------------------------------------------------------------------

#define   MAX_DCC_SIZE  6

// This enum defines the type of message put to the tracks.

typedef enum {is_void,      // message with no special handling (like functions)
              is_stop,      // broadcast
              is_loco,      // standard dcc speed command
              is_acc,       // accessory command
              is_feedback,  // accessory command with feedback
              is_prog,      // service mode - longer preambles
              is_prog_ack}  t_msg_type;


typedef struct
  {
    unsigned char repeat;             // counter for repeat or refresh (depending)
    union
     {
       struct
        {
         unsigned char size: 4;            // 2 .. 5
         t_msg_type    type: 4;            // enum: isvoid, isloco, accessory, ...
        } ;
       unsigned char qualifier;
     } ;
    unsigned char dcc[MAX_DCC_SIZE];  // the dcc content
  } t_message;


// define a structure for the loco memeory (6 bytes)

//SDS sick of compiler complaints
//SDS typedef enum {DCC14 = 0, DCC27 = 1, DCC28 = 2, DCC128 = 3} t_format;
#define DCC14   0
#define DCC27   1
#define DCC28   2
#define DCC128  3

typedef unsigned char t_format;

struct locomem
  {
    unsigned int address;               // address (either 7 or 14 bits)
    unsigned char speed;                // this is in effect a bitfield:
                                        // msb = direction (1 = forward, 0=revers)
                                        // else used as integer, speed 1 ist NOTHALT
                                        // this is i.e. for 28 speed steps:
                                        //       0: stop
                                        //       1: emergency stop
                                        //  2..127: speed steps 1..126
                                        // speed is always stored as 128 speed steps
                                        // and only converted to the according format
                                        // when put on the rails or to xpressnet
    t_format format: 2;                 // 00 = 14, 01=27, 10=28, 11=128 speed steps.
                                        // DCC27 is not supported
    unsigned char active: 1;            // 1: lok is in refresh, 0: lok is not refreshed
    unsigned char fl: 1;                // function light
    unsigned char f4_f1: 4;             // function 4 downto 1
    unsigned char f8_f5: 4;             // function 8 downto 5
    unsigned char f12_f9: 4;            // function 12 downto 9
    #if (DCC_F13_F28)
    #define SIZE_LOCOBUFFER_ENTRY_D 1
    unsigned char f20_f13: 8;           // function 20 downto 13
    unsigned char f28_f21: 8;           // function 28 downto 21
    #endif
    unsigned char refresh;              // refresh is used as level: 0 -> refreshed often
  };

#define SIZE_LOCOBUFFER_ENTRY (6 + SIZE_LOCOBUFFER_ENTRY_B+SIZE_LOCOBUFFER_ENTRY_D+SIZE_LOCOBUFFER_ENTRY_X)

// Note on speed coding (downstream):
//
// Speed is always stored as 0..127.
// Speedentries from IBOX are handled directly.
// Speedentries from LENZ are converted (speed_from_rail) when they are put to
//                                      (speed_to_rail) or read from locobuffer.
// When a message is put on the rails, speed is converted according to format.


// define a structure for programming results

typedef enum 
  {
    PR_VOID     = 0x00, 
    PR_READY    = 0x01,     // 0x11, command station is ready
    PR_BUSY     = 0x02,     // 0x1f, busy
    PR_REGMODE  = 0x03,     // 0x10, register + values
    PR_CVMODE   = 0x04,     // 0x14, Last command was CV + data
    PR_SHORT    = 0x05,     // 0x12, short detected
    PR_NOTFOUND = 0x06     // 0x13, no found
  } t_prog_summary;

typedef struct
  {
    unsigned char minute; 
    unsigned char hour;
    unsigned char day_of_week;
    unsigned char ratio;
  } t_fast_clock;


//========================================================================
// 5. Usage of Memory, EEROM and Registers
//========================================================================
// Globals
//
extern const unsigned char opendcc_version;
extern unsigned char invert_accessory;
extern unsigned char bidi_messages_enabled; 
extern unsigned char xpressnet_feedback_mode;   // filled from CV29



//------------------------------------------------------------------------
// 5.3. Memory Usage - RAM
//------------------------------------------------------------------------
//
// RAM:   Size:     Usage
//         170      General
//          64      RS232 Rx
//          64      RS232 Tx
//         600      Locobuffer (Size * 6)

#define SIZE_QUEUE_PROG       6       // programming queue (7 bytes each entry)
#define SIZE_QUEUE_LP        16       // low priority queue (7 bytes each entry)
#define SIZE_QUEUE_HP         8       // high priority queue (7 bytes each entry)
#define SIZE_REPEATBUFFER    32       // immediate repeat (7 bytes each entry)
//SDS#define SIZE_LOCOBUFFER      64       // no of simult. active locos (6 bytes each entry)
#define SIZE_LOCOBUFFER      5 //SDS, meer dan genoeg nu!! (gebruik ram voor een display)



// see config.c for valid ranges and explanation
// used by TC or railware: -----------------------|
#define   eadr_OpenDCC_Version          0x000  // r  / 
#define   eadr_baudrate                 0x001  // t  /
#define   eadr_OpenDCC_Mode             0x002  //    / reserved (HSI88, Xpressnet, ...)
#define   eadr_virtual_decoder_l        0x003  //    /
#define   eadr_virtual_decoder_h        0x004  //    /
#define   eadr_VersionMirror            0x005  //    / cant read #0- so we mirror the version here again
#define   eadr_CTS_usage                0x006  // r  / corresponds to IB SO 006
#define   eadr_s88_mode                 0x007  //    / s88 via hw or feedback
#define   eadr_s88_autoread             0x008  // r  / no of s88 bytes to be read automatically (total)
#define   eadr_s88_size1                0x009  //    / given in bytes
#define   eadr_s88_size2                0x00a  // 
#define   eadr_s88_size3                0x00b  //
#define   eadr_invert_accessory         0x00c  //    / bit 0: invert Lenz, bit 1: invert IB
#define   eadr_dcc_acc_repeat           0x00d  //
#define   eadr_dcc_acc_time             0x00e  // r  / turn on time of acc (used by IB, default 100)
#define   eadr_startmode_ibox           0x00f  //    / 0=Normal Mode, 1=fixed to P50X
#define   eadr_feedback_s88_offset      0x010  //    / offset for feedback numbers in s88 array, given in bytes
#define   eadr_feedback_s88_type        0x011  //    / status, okay, or error feedback
#define   eadr_extend_prog_resets       0x012  //    / add this number the number of resets command during programming
#define   eadr_extend_prog_command      0x013  //    / use this number to extend program command to a longer time  
#define   eadr_dcc_pom_repeat           0x014  //    / number of repeats for PoM
#define   eadr_dcc_speed_repeat         0x015  //    / 21: speed repeat
#define   eadr_dcc_func_repeat          0x016  
#define   eadr_reserved023              0x017  
#define   eadr_dcc_default_format       0x018  //    / 24: default format: 0 =DCC14, 1=DCC27, 2=DCC28, 3=DCC128 
#define   eadr_railcom_enabled          0x019  //    / 25: 1: railcom enabled
#define   eadr_fast_clock_ratio         0x01a  //    / 26: 0: disabled, 1: ratio of fast clock 1..31
#define   eadr_reserved027              0x01b 
#define   eadr_reserved028              0x01c  
#define   eadr_xpressnet_feedback       0x01d  //    / 29: Xpressnet Feedback mode: 0=256trnt,512feedb. 1; only feedback, 2only trnt
#define   eadr_s88_clk_timing           0x01e  //    / 30: S88-CLK Timing
#define   eadr_feedback_s88_size        0x01f  //    / 31: no. of turnout feedback, given in bytes
#define   eadr_s88_total_from_pc        0x020  //    / 32: invisible: s88_total_from_pc
#define   eadr_I2C_present              0x021  //  r / I2C present - return 0  
#define   eadr_short_turnoff_time       0x022  //    / CV34: Short turnoff time - in ticks
#define   eadr_prog_short_toff_time     0x023  //    / CV35: short turnoff time programming track - in ticks
#define   eadr_ext_stop_enabled         0x024  //    / CV36: enable external Stop Input (not usable together with feedback)
#define   eadr_ext_stop_deadtime        0x025  //    / CV37: external stop deadtime after status RUN 
#define   eadr_reserved038              0x026  
#define   eadr_serial_id                0x027  //    / CV39: serial number, must be > 1

extern unsigned char eemem[] __attribute__((section("EECV")));    // EEMEM

#define EADR_LOCO_FORMAT     0x810080L  // base addr in mem
#define ESIZE_LOCO_FORMAT    64         // no of locos with different format (max. 250)
                                        // each entry requires 2 bytes


// This union allows to access 16 bits as word or as two bytes.
// This approach is (probably) more efficient than shifting.
typedef union
{
    uint16_t as_uint16;
    uint8_t  as_uint8[2];
} t_data16;


typedef struct
  {
    unsigned char  vendor;
    union
      {
        struct
          {
            unsigned char  byte0;
            unsigned char  byte1;
            unsigned char  byte2;
            unsigned char  byte3;
          };
        struct
          {
            unsigned int   product_id;
            unsigned int   product_serial;
          };
        unsigned long vendor32;
      };
  } t_unique_id;

#endif   // config.h

