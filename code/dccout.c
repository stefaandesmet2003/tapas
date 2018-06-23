//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2010 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      dccout.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-05-19 V0.2 long preambles if PROG_TRACK_STATE = ON
//            2006-06-13 V0.3 Bugfix: TC1R no usable as state
//            2006-09-30 V0.4 added check for F_CPU, added code proposal
//                            for Mï¿½rklin MM1 and MM2 (not tested)
//            2007-03-19 V0.5 added code for FEEDBACK
//            2007-03-27 V0.6 added check for hardware.h
//            2008-07-09 V0.7 interrupt processor dependant
//                            (Atmega32, Atmega644P)
//            2008-08-18 V0.8 railcom cutout added - see CUTOUT_GAP
//            2008-09-11      railcom cutout shifted by one bit (bugfix)
//            2009-06-23 V0.9 DCC message size imported from config.h (MAX_DCC_SIZE)
//            2009-07-21 V0.10 mm bug fix speed 11
//            2010-05-29 V0.11 change in cutout_gap from 30 to 38us.
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   ISR for DCC-OUT
//
//-----------------------------------------------------------------

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include "config.h"                 // general structures and definitions
#include <string.h>
#include "dccout.h"                 // import own header

void InitEPwm3(void);
__interrupt void epwm_isr(void);

//-----------------------------------------------------------------
//------ message formats
// DCC Baseline Packet 3 bytes (address data xor) form 42 bits
// 11111111111111 0 0AAAAAAA 0 01DCSSSS 0 EEEEEEEE 1
// built as follows:
// bit:   value     defines
// 0-14   1         preamble
// 15     0         packet start bit  = > allways 0xFF, 0xFE at start
// 16-23  address   dcc address
// 24     0         data start bit
// 25-32  data      data
// 33-40  xor       error checking
// 41     1         packet end bit

// DCC Extended Packets (see NMRA 9.2.1):
// Addr:            used for:
// 0       00000000 broadcast
// 1-127   0AAAAAAA 7-bit addressed loco decoder
// 128-191 10AAAAAA accessory decoder (either 9 bit or 11 bit)
// 192-231 11000000-11100111 14-bit addressed loco decoders
// 232-254 11101000-11111110 reserved, future use
// 255     11111111 idle
//
// following one to three data bytes
// Data:   CCCDDDDD [0 DDDDDDDD 0 DDDDDDDD]
// CCC defines type:
// 000     Funktionen des Dekoders steuern
//         00000000           decoder reset
//         0001CCCC 0AAAAAAA  consist control
// 001     Erweiterte Funktionen des Dekoders steuern
//         00111111 DDDDDDDD  speed step control
// 010     Geschwindigkeit fï¿½r Rï¿½ckwï¿½rtsfahrt
// 011     Geschwindigkeit fï¿½ur Vorwï¿½artsfahrt
// 100     Hilfsfunktion aus Gruppe 1 steuern
//         100DDDDD           DDDDD = FL, F4, F3, F2, F1
// 101     Hilfsfunktion aus Gruppe 2 steuern
//         two modes:
//         1011DDDD           DDDD = F8, F7, F6, F5
//         1010DDDD           DDDD = F12, F11, F10, F9
// 110     Reserviert fï¿½r zukï¿½nftige Erweiterungen
// 111     Zugriff auf Konfguration-Variablen
//         1110CCAA AAAAAAAA DDDDDDDD
//         CC = 00 (reserved), 01=Verify, 11=write, 10 bit set
//         AA AAAAAAAA 10 bit cv-address, msb first (cv = address+1)
//         bit set: DDDDDDDD = 111CDAAA; C=1 -> write, C=0 verify, D=Bit, AAA=bit pos.
//         write requires two packets !
//
// accesory messages
//  9-bit addresses: 10AAAAAA  1AAACDDD
// 11-bit addresses: 10AAAAAA  0AAA0AA1 000XXXXX
//
//=====================================================================
//
// DCC_OUT
//
// purpose:   receives the next message and puts on the DCC-Pins,
//            using the PWM engine of the ATmega.
//
// how:       the pwm timer is programmed to clear on Compare value;
//            each compare hit of the timer generates a INT,
//            Output is toggled in the Timer and the Timer
//            is reprogrammed for the next bit duration.
//
//            Every second INT checks for the settings of next bit and
//            advances the state engine. Both compares run in parallel
//            to generate both DCC and nDCC for the output driver.
//
//            The ISR uses an state engine (doi.state) to track the
//            actual position inside a DCC message. preamble and
//            XOR checksum are managed by the ISR.
//
// interface: new messages (only the payload) are copied from
//            global char array next_message. After copying, the
//            global int next_message_count is decremented. The
//            interfacing routine has to stop interrupts when writing
//            to next_message.
//            next_message: first char = size of message (valid 2...5),
//                          following chars = message (payload);
//
//            if no next_message is given, dccout will keep alive
//            and will send all 1;
//
//-----------------------------------------------------------------
//------ message formats
// DCC Baseline Packet 3 bytes (address data xor) form 42 bits
// 11111111111111 0 0AAAAAAA 0 01DCSSSS 0 EEEEEEEE 1
// built as follows:
// bit:   value     defines
// 0-14   1         preamble
// 15     0         packet start bit  = > allways 0xFF, 0xFE at start
// 16-23  address   dcc address
// 24     0         data start bit
// 25-32  data      data
// 33-40  xor       error checking
// 41     1         packet end bit (may be next preamble)


//-------------------------------------------------------------------------------
// generate bit sequence through PWM timing
// prescaler = 1
// f(clk) = 16 MHz
// t(period_0) = 232 us, t(period_1) = 116 us
//
// calculate: TOP     = f(clk) * t(period) / prescaler - 1
//            COMPARE = f(clk) * t(period) / 2 * prescaler - 1
//
// at 16MHz it lasts aprox. 4us (wc) until new timervals are set;
// Interrupts must not be blocked more then 50us.
// DCCOUT-ISR must have the highest priority in the system.
//
// If an interrupt is missed, the following occurs:
// The counter will wrap at 0xffff, thus putting out a stretched bit of
// 4,096ms duration; annoying, but not dead ;-)
//
//-------------------------------------------------------------------------------


/// This are timing definitions from NMRA
#define PERIOD_1   116L                  // 116us for DCC 1 pulse - do not change
#define PERIOD_0   232L                  // 232us for DCC 0 pulse - do not change
#define CUTOUT_GAP  38L                  // 38us gap after last bit of xor
// #define CUTOUT_GAP  30L                  // 30us gap after last bit of xor

//-----------------------------------------------------------------
//
// Timer 1 overflow interrupt service routine
//
// purpose:  - create dcc activity
//           - set new CTC (clear timer on compare) values
//             depending on next bit.
//           - advance state engine
//-----------------------------------------------------------------
//

// upstream interface for messages:

struct next_message_s next_message;    // see dccout.h

unsigned char next_message_count;

//----------------------------------------------------------------------------------------
// Optimierter Code
// folgende Tricks:
//  a) doi.phase wird direkt von PIND zurï¿½ckgelesen, damit entfï¿½llt der Memoryzugriff
//  b) doi.state wird in MY_STATE_REG[7..5] abgelegt - schnell abzufragen
//  c) kein switch fï¿½r doi.state, sondern if else
//  d) doi.bits_in_state wird in MY_STATE_REG[4...0] abgelegt. MY_STATE_REG ist
//     damit *die* zentrale globale Variable, die den Zustand der ISR abbildet.
//  e) do_send nicht als call, sondern direkt als inline call
//  f) Mit einem #define DCCOUT_STATE_REG auf ein IO-Register des Prozessors
//     kann die ISR noch weiter beschleunigt werden -> dieses IO-Register
//     wird dann als globale Variable fï¿½r MY_STATE_REG misbraucht. 
//
// Optimizimg code
// following tricks
// a) read back of doi.phase from PIND - shorter then memory access
// b) doi.state is allocated at MY_STATE_REG[7..5] - fast access
// c) use of if-then-else instead of switch().
// d) doi.bits_in_state are allocated at MY_STATE_REG[4..0] - fast access
//    MY_STATE_REG is *the* central variable!
// e) do_send() is not called, but compiled as inline code
// f) MY_STATE_REG is mapped to an unused IO port of the atmel
//    -> #define DCCOUT_STATE_REG 
//----------------------------------------------------------------------------------------

void do_send(bool myout)
{
  if (myout == 0)
    {                                     // 0 - make a long pwm pulse
      EPwm3Regs.TBPRD = 5260;       // Set timer period (116us)
    }
  else
    {                                     // 1 - make a short pwm puls
      EPwm3Regs.TBPRD = 2630;       // Set timer period (58us)
    }
} // do_send

// define some handy names for the states of the ISR
#define DOI_IDLE     (0 << 5)
#define DOI_PREAMBLE (1 << 5)
#define DOI_BSTART   (2 << 5)
#define DOI_BYTE     (3 << 5)
#define DOI_XOR      (4 << 5)
#define DOI_END_BIT  (5 << 5)
#define DOI_CUTOUT_1 (6 << 5)
#define DOI_CUTOUT_2 (7 << 5)
#define DOI_CNTMASK  0x1F

struct
{
  unsigned char state;                            // current state
  unsigned char ibyte;                            // current index of byte in message
  unsigned char cur_byte;                         // current byte
  unsigned char xor_byte;                         // actual check
  unsigned char current_dcc[MAX_DCC_SIZE];        // current message in output processing
  unsigned char bytes_in_message;                 // current size of message (decremented)
  unsigned char railcom_enabled;                  // if true: create cutout
  t_msg_type type;                                // type (for feedback)
} doi;

#define MY_STATE_REG doi.state

__interrupt void epwm_isr(void)
{
  unsigned char state;


  // Clear INT flag for this timer
  EPwm3Regs.ETCLR.bit.INT = 1;

  // Acknowledge this interrupt to receive more interrupts from group 3
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  state = MY_STATE_REG & ~DOI_CNTMASK;    // take only 3 upper bits
  if (state == DOI_IDLE)
  {
    do_send(1);

    if (next_message_count > 0)
    {
      memcpy(doi.current_dcc, next_message.dcc, sizeof(doi.current_dcc));
      doi.bytes_in_message = next_message.size;
      // no size checking - if (doi.cur_size > MAX_DCC_SIZE) doi.cur_size = MAX_DCC_SIZE;
      doi.ibyte = 0;
      doi.xor_byte = 0;
      doi.type = next_message.type;   // remember type in case feedback is required

      next_message_count--;

      if (PROG_TRACK_STATE) MY_STATE_REG = DOI_PREAMBLE+(20-3);   // long preamble if service mode
      else 				MY_STATE_REG = DOI_PREAMBLE+(14-3);     // 14 preamble bits
                                                                  // doi.bits_in_state = 14;  doi.state = dos_send_preamble;
    }
    return;
  }

  if (state == DOI_PREAMBLE)
  {
    do_send(1);
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0)
    {
      MY_STATE_REG = DOI_BSTART;          // doi.state = dos_send_bstart;
    }
    return;
  }

  if (state == DOI_BSTART)
  {
    do_send(0);     // trennende 0
    if (doi.bytes_in_message == 0)
    { // message done, goto xor
      doi.cur_byte = doi.xor_byte;
      MY_STATE_REG = DOI_XOR+8;  // doi.state = dos_send_xor; doi.bits_in_state = 8;
    }
    else
    { // get next addr or data
      doi.bytes_in_message--;
      doi.cur_byte = doi.current_dcc[doi.ibyte++];
      doi.xor_byte ^= doi.cur_byte;
      MY_STATE_REG = DOI_BYTE+8;  // doi.state = dos_send_byte; doi.bits_in_state = 8;
    }
    return;
  }

  if (state == DOI_BYTE)
  {
    if (doi.cur_byte & 0x80) {do_send(1);}
    else                     {do_send(0);}
    doi.cur_byte <<= 1;
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0)
    {
      MY_STATE_REG = DOI_BSTART+8;  // doi.state = dos_send_bstart;
    }
    return;
  }

  if (state == DOI_XOR)    // ev. else absichern
  {
    if (doi.cur_byte & 0x80) {do_send(1);}
    else                     {do_send(0);}
    doi.cur_byte <<= 1;
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0)                  // bitcounter lower 5 bits
    {
      MY_STATE_REG = DOI_END_BIT;  // doi.state = dos_idle;
    }
    return;
  }

  if (state == DOI_END_BIT)
  {
    do_send(1);
    MY_STATE_REG = DOI_CUTOUT_1;
    return;
  }

  if (state == DOI_CUTOUT_1)
  {
    do_send(1);
    MY_STATE_REG = DOI_CUTOUT_2;
    return;
  }

  if (state == DOI_CUTOUT_2)
  {
    do_send(1);
    MY_STATE_REG = DOI_IDLE;
    return;
  }

} // epwm_isr

void init_dccout(void)
{
  MY_STATE_REG = DOI_IDLE; // doi.state = dos_idle;
  next_message_count = 0;
  next_message.size = 2;
  next_message.dcc[0] = 0;
  next_message.dcc[1] = 0;

  doi.railcom_enabled = 0; // voorlopig, want nog geen code voor cutout op tapas

  InitEPwm3();

  do_send(1);           // init COMP regs.

} // init_dccout



//-------------------------------------------------------------------------------------
// RailCom Interface
//-------------------------------------------------------------------------------------

void dccout_enable_cutout(void)
  {
    // eigentlich sollte man doi.state abfragen, um auch beim ersten mal synchron die cutout zu erzeugen
    // aber DCC wirds ï¿½berleben.
    doi.railcom_enabled = 1;
  }

void dccout_disable_cutout(void)
  {
    doi.railcom_enabled = 0;
  }

unsigned char dccout_query_cutout(void)
  {
    return(doi.railcom_enabled);
  }

void InitEPwm3(void)
{
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
  EDIS;

  // configure the corresponding gpio's as epwm
  InitEPwm3Gpio();

  // Setup TBCLK
  EPwm3Regs.TBCTL.bit.PRDLD = 1; // immediate load
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
  EPwm3Regs.TBPRD = 2630;       // Set timer period (58us)
  EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading (pwm7 = master module)
  EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
  EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync downstream module
  EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
  EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
  EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;

  // Set Actions
  EPwm3Regs.AQCTLA.all = 0;
  EPwm3Regs.AQCTLB.all = 0;
  EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;
  EPwm3Regs.AQCTLA.bit.PRD = AQ_CLEAR;
  EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
  EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR;
  
  // deadband setup (AHC)
  EPwm3Regs.DBCTL.bit.OUT_MODE  = 3;          // Dead Band Output Mode Control
  EPwm3Regs.DBCTL.bit.IN_MODE   = 0;          // Dead Band Input Select Mode Control
  EPwm3Regs.DBCTL.bit.POLSEL    = 2;          // Polarity Select Control
  EPwm3Regs.DBCTL.bit.HALFCYCLE = 0;          // Half Cycle Clocking Enable
  EPwm3Regs.DBRED = 20;              // Dead-Band Generator Rising Edge Delay Count Register
  EPwm3Regs.DBFED = 20;              // Dead-Band Generator Falling Edge Delay Count Register

  // Interrupt on CTR=ZRO
  EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO ; // ET_CTR_PRDZERO;     // Select INT on Zero event
  EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
  EPwm3Regs.ETPS.bit.INTPRD = ET_1ST; // ET_3RD; // Generate INT on 3rd event

  EALLOW;  // This is needed to write to EALLOW protected registers
  PieVectTable.EPWM3_INT = &epwm_isr;
  EDIS;    // This is needed to disable write to EALLOW protected registers

  // Enable CPU INT3 which is connected to EPWM1-3 INT: (alle epwm ints zitten op cpu int3)
  IER |= M_INT3;

  // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
  PieCtrlRegs.PIEIER3.bit.INTx3 = 1; //epwm3

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;

} // InitEPwm3
