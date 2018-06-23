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
//--------------------------------------------------------------------------
//
// file:      programmer.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//
//---------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   genereates the dcc servicemode messages and checks for
//            acknowledge.
//
// interface upstream:
//            init_programmer(void)     // 
//            run_programmer(void)      // multitask replacement
//
// interface downstream:
//            put_in_queue_prog         // send command to organizer
//            queue_prog_is_empty       // ask organizer about queue
//
// 2do:       
//--------------------------------------------------------------------------

// LINK:      http://ruppweb.dyndns.org/xray/comp/decoder.htm
//            (calculate long addresses)

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>

#include "config.h"                // general structures and definitions
#include "status.h"                // timeout engine, set_state
#include "dccout.h"                // next message
#include "organizer.h"
#include "programmer.h"
//
// Es gibt drei "switch-Schleifen", damit auch umfangreichere Kommandos
// im quasi Multitasking durchgebracht werden kï¿½nnen. Jede Schleife
// hat einen State, wenn dieser IDLE ist, dann ist diese Schleife fertig;
// 
// Gestartet wird eine Aufgabe durch setzen der Variablen und anschlieï¿½end
// umstellen des State auf "START".
//
//   PI: Programmer Inner Loop (die Kommandos am Gleis)
//       Commands der Inner Loop:
//          CV-Mode, write byte
//          CV-Mode, verify byte
//          CV-Mode, verify single bit
//          CV-Mode, write single bit
//          address_mode_verify
//          address_mode_write
//          register_mode_verify
//          register_mode_write
//
//   PB: Byte Operationen, z.B. beim Lesen einer CV
//       Commands der Byte Loop:
//          CV-Mode, read byte (scan methode)
//          CV-Mode, read byte (bit methode)
//          test_direct_mode
//          register_mode_read_byte
//          paged_mode_write
//          paged_mode_verify
//          paged_mode_read_byte
//
//   PS: Sequenzen
//       Commands der Sequenzer Loop:
//          read long adr
//          write long adr
//          (set curve)
//
// Besonderheit beim direkten Lesen eines Bytes per DCC:
// OpenDCC prï¿½ft vorher nach, ob der Decoder Bitoperationen beherrscht.
// Falls ja, wird das Byte mit den Bitbefehlen gelesen und dann noch
// gegengeprï¿½ft. Fall nein, wird konventionell mit einer Suchschleife
// ï¿½ber alle 256 mï¿½glichen Zustï¿½nde gesucht.
// OpenDCC merkt sich fï¿½r 500ms, ob der Decoder Bitoperationen kann, damit
// entfï¿½llt automatisch die erneute Prï¿½fung bei mehreren Lesebefehlen.
//
//=====================================================================
//
// Data Structures
//
//--------------------------------------------------------------------
// internal, but static:

enum prog_inner_states
  {                            // actual state for this Operation
     PI_IDLE,
     PI_START,
     DO_1ST_RESET,
     DO_PAGE_PRESET,
     DO_2ND_RESET,
     DO_PROG_MESSAGE,
     SETUP_3RD_RESET,
     DO_3RD_RESET
  } prog_inner_state;


enum prog_byte_states
  {
     PB_IDLE,                   // fertig, Ergebnis kann abgeholt werden
     PB_START,                  // Call 
     PB_RUNNING,                // Single fall throu command
     PB_RD_LOOP,                // test content of cv in loop
     PB_RD_BIT,                 // read 8 bits
     PB_RD_BIT_VERIFY,          // verify result
     PB_DCCQD,                  // test decoder for bit operation
     PB_DCCQD2                  // test decoder for bit operation, phase 2
  } prog_byte_state;


enum prog_seq_states            // prog sequencer
  {
     PS_IDLE,                   // fertig, Ergebnis kann abgeholt werden
     PS_START,                  // Call 
     PS_RUNNING,                // Standard command or last command.
     PS_WRITE_PAGE_ADR,         // Pageadresse schreiben
     PS_CHECK_BITOP,            // leadin check, whether decoder can bitop.
     PS_DCCQD,                  // test decoder for bit operation
     PS_DCCRL,                  // read long
     PS_DCCRL2,                 // read long 2
     PS_DCCWL,                  // write long
     PS_DCCWL2,                 // write long phase 2
     PS_DCCWL3                 // write long phase 3
     
  } prog_seq_state;

// -->>> Variablen zur Steuerung der Schleifen -> PI = prog inner
enum
  {
    PIC_CVM_W_BYTE,             // CV-Mode, write byte
    PIC_CVM_V_BYTE,             // CV-Mode, verify byte
    PIC_CVM_W_BIT,              // CV-Mode, write single bit
    PIC_CVM_V_BIT,              // CV-Mode, verify single bit
    //PIC_AM_W_BYTE,              // address_mode_write     
    //PIC_AM_V_BYTE,              // address_mode_verify
    PIC_RM_W_BYTE,              // register_mode_write         
    PIC_RM_V_BYTE              // register_mode_verify
  } pi_command;

unsigned int pi_cv;
unsigned char pi_data;
unsigned char pi_bitpos;
unsigned char pi_result;        // 0 = okay, ack received
                                // !0 = no ack

// -->>> Variablen zur Steuerung der Schleifen -> PB = prog byte
enum
  {
    PBC_RM_W_BYTE,              // Reg-Mode, write byte
    PBC_RM_R_BYTE,              // Reg-Mode, read byte (in loop)
    PBC_CVM_W_BYTE,             // CV-Mode, write byte
    PBC_CVM_R_BYTE,             // CV-Mode, read byte (in loop)
    PBC_CVM_R_BIT,              // CV-Mode, read byte (bit commands)
    PBC_CVM_W_BIT,              // CV-Mode, write byte (bit commands)
    PBC_DCCQD                   // does decoder support single bit?
  } pb_command;

unsigned int pb_cv;
unsigned char pb_data;
unsigned char pb_test;
unsigned char pb_result;        // 0: success, 1: failed


// -->>> Variablen zur Steuerung der Schleifen -> PS
enum
  {
    PSC_DCCRR,                  // read register
    PSC_DCCWR,                  // write register
    PSC_DCCRP,                  // read byte paged mode
    PSC_DCCWP,                  // write byte paged mode
    PSC_DCCRD,                  // read mit byte verify in loop
    PSC_DCCWD,                  // write byte (direct mode)
    PSC_DCCRB,                  // read mit bit verify in loop
    PSC_DCCWB,                  // write single bit
    PSC_DCCQD,                  // does decoder support single bit?
    PSC_DCCRL,                  // read long adr
    PSC_DCCWL                  // write long adr
  } ps_command;                 // hier wird das Command hinterlegt.

unsigned char ps_bitpos;        // Bitpos bzw. lokaler Programmschritt 

unsigned char decoder_can_bit_operations;   // if !0: the current connected decoder can
                                            // do bit programming; this speeds up reading of cv.

#define TIME_REMEMBER_BIT_OP  500L          // after 500ms OpenDCC forgets about the abilitiy
                                            // of the decoder to do bit operations
uint32_t last_bit_check;    // sds type veranderd compatibel met millis()

int page_loaded_in_decoder = -1;            // 

#define TIME_REMEMBER_PAGE    500L          // after 500ms OpenDCC forgets the loaded page addr
                                            
uint32_t last_page_loaded;  // sds type veranderd compatibel met millis()


//--------------------------------------------------------------------------------------------
// Interface Variablen, Global
//
unsigned int prog_loco;           // last loco (for pom);
unsigned int prog_cv;             // aktual cv; this is the 'normal' number, range from 1 ... 1024
unsigned char prog_data;
unsigned char prog_result_size;   // size of data (additional to prog_result)

t_prog_event prog_event;          // whenever the state of the prog_task changes fundamentally
                                  // prog_event.result = 1; is cleared by parser
t_prog_qualifier prog_qualifier;
  
t_prog_result prog_result;


//--------------------------------------------------------------------------------------------
// Overview of Programming Modes:
//
// Mode:        Leading PagePre Mid     Command Post
//              resets          resets          resets (r/w)
//-------------------------------------------------------------------
// Direct         3       -       -       5       1 / 6
// Addr, only     3       5       9       7       1 / 10
// Reg. Mode      3       5       9       7       1 / 10
// Paged Mode     same as Reg. Mode, but page access in advance        
//

typedef enum {P_IDLE, P_WRITE, P_READ}  t_prog_mode;

typedef struct                      // this is the structure where we handle different modes
  {
    t_prog_mode mode;
    unsigned char cycles[5];        // we have 5 different cycles - see table above
  } t_prog_ctrl;

t_prog_ctrl prog_ctrl;           
 
// default array, used during init
//SDS added const to PROGMEM variables
const t_prog_ctrl direct_ctrl_default PROGMEM       = {P_READ, {3, 0, 0, 5,  6}};
const t_prog_ctrl registermode_ctrl_default PROGMEM = {P_READ, {3, 5, 9, 7, 10}};

// working array
// t_prog_ctrl direct_ctrl       = {P_READ, {3, 0, 0, 7,  6}};
// t_prog_ctrl registermode_ctrl = {P_READ, {3, 5, 9, 7, 10}};
t_prog_ctrl direct_ctrl; 
t_prog_ctrl registermode_ctrl;


//--------------------------------------------------------------------
//                            rep, size, type, DCC
t_message pDCC_Reset     = {1, {{ 2, is_void}}, {0x00, 0x00}};    // DCC-Reset-Paket
t_message *dcc_reset_ptr = &pDCC_Reset;
t_message prog_message   = {1, {{ 2, is_prog}}, {0x00, 0x00, 0x00, 0x00, 0x00}};    // DCC-Programming
t_message *prog_message_ptr = &prog_message;

t_message page_preset    = {1, {{ 2, is_void}}, {0b01111101, 0b00000001}};
t_message *page_preset_ptr = &page_preset;




//-----------------------------------------------------------------------------------------------
/// switch opendcc to progmode
/// does nothing if already in progmode, else puts a power cycle on the track

t_opendcc_state opendcc_state_before_prog = RUN_STOP;

void enter_progmode(void)
  {
    switch(opendcc_state)
      {
        case RUN_OKAY:
        case RUN_STOP:                  // DCC Running, all Engines Emergency Stop
        case RUN_OFF:                   // Output disabled (2*Taste, PC)
        case RUN_SHORT:                 // Kurzschluï¿½
        case RUN_PAUSE:                 // DCC Running, all Engines Speed 0

            opendcc_state_before_prog = opendcc_state;
            prog_event.bidi_pending = 0;        // clear any bidi result queues - we do real prog

            while (next_message_count != 0);    // busy wait for current message to terminate
                                                // do not allow organizer to load next command!
	        set_opendcc_state(PROG_OKAY);
            pDCC_Reset.repeat = 20;             // 20 reset packets -> power on cycle	 
	        put_in_queue_prog(dcc_reset_ptr);
            break;

        case PROG_OKAY:                 // nothing to change, we are already in prog_mode
            break;
        case PROG_SHORT:                //
        case PROG_OFF:
        case PROG_ERROR:
            while (next_message_count != 0);    // busy wait for current message to terminate


	        set_opendcc_state(PROG_OKAY);
            pDCC_Reset.repeat = 20;             // 20 reset packets -> power on cycle	 
	        put_in_queue_prog(dcc_reset_ptr);
            break;
      }
  }


void leave_progmode(void)
  {
    while (next_message_count != 0);    // busy wait for current message to terminate
                                        // do not allow organizer to load next command!
    switch(opendcc_state_before_prog)
      {
        case RUN_OKAY:
        case RUN_STOP:             // DCC Running, all Engines Emergency Stop
        case RUN_OFF:              // Output disabled (2*Taste, PC)
        case RUN_SHORT:            // Kurzschluï¿½
        case RUN_PAUSE:            // DCC Running, all Engines Speed 0
            set_opendcc_state(opendcc_state_before_prog);
            break;

        case PROG_OKAY:            // Oops, old state was Progmode - lets default to RUN_OFF
        case PROG_SHORT:           //
        case PROG_OFF:
        case PROG_ERROR:
        default:
        	set_opendcc_state(RUN_OFF);
            break;
      }
  }

void reset_programmer(void)
  {
    prog_event.busy = 0;            // stop any running task
    prog_inner_state = PI_IDLE;
    prog_byte_state = PB_IDLE;
    prog_seq_state = PS_IDLE;
  }

void show_prog_error(void)
  {
    prog_result_size = 0;          // 12.01.2008: in case of Error, we have no data
    set_opendcc_state(PROG_ERROR);
  }

//===================================================================================
//
// message builders of inner loop
//
//===================================================================================


/// CV-Mode, write
//
//  Write Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1 mit CC=11

void build_direct_mode_write(unsigned int cv, unsigned char data)
  {
     unsigned int cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_void;
     prog_message.dcc[0] = 0b01110000 | 0b00001100 | (unsigned char)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (unsigned char)(cv_adr & 0xFF);
     prog_message.dcc[2] = data;

     memcpy(&prog_ctrl, &direct_ctrl, sizeof(prog_ctrl));
     prog_ctrl.mode = P_WRITE;
  }
  
/// CV-Mode, verify
//
//  Verify Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1 mit CC=01

void build_direct_mode_verify(unsigned int cv, unsigned char data)
  {
     unsigned int cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00000100 | (unsigned char)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (unsigned char)(cv_adr & 0xFF);
     prog_message.dcc[2] = data;

     memcpy(&prog_ctrl, &direct_ctrl, sizeof(prog_ctrl));
  }

/// CV-Mode, verify single bit
//
//  Verify Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1 mit CC=10
//  K = (1=write, 0=verify) D = Bitvalue, BBB = bitpos
//  return: true if verified

void build_direct_mode_bit_verify(unsigned int cv, unsigned char bitpos, unsigned char mybit)
  {
     unsigned int cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00001000 | (unsigned char)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (unsigned char)(cv_adr & 0xFF);
     prog_message.dcc[2] = 0b11100000 | ((mybit &0b1) << 3) | (bitpos & 0b111);
     
     memcpy(&prog_ctrl, &direct_ctrl, sizeof(prog_ctrl));
  }

/// CV-Mode, write single bit
//
//  Write Paket
//  preamble 0 0111 CCAA 0 AAAAAAAA 0 111KDBBB 0 EEEEEEEE 1 mit CC=10
//  K = (1=write, 0=verify) D = Bitvalue, BBB = bitpos

void build_direct_mode_bit_write(unsigned int cv, unsigned char bitpos, unsigned char mybit)
  {
     unsigned int cv_adr;

     cv_adr = cv-1;
     prog_message.repeat = 1;
     prog_message.size   = 3;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | 0b00001000 | (unsigned char)((cv_adr >> 8) & 0b11);
     prog_message.dcc[1] = (unsigned char)(cv_adr & 0xFF);
     prog_message.dcc[2] = 0b11110000 | ((mybit &0b1) << 3) | (bitpos & 0b111);
     
     memcpy(&prog_ctrl, &direct_ctrl, sizeof(prog_ctrl));
     prog_ctrl.mode = P_WRITE;
  }



// long-preamble 0 0111CRRR 0 DDDDDDDD 0 EEEEEEEE 1
// RRR=Register address (range 0...7)
// C=0 Verify, C=1 Write register
// RRR=000 ist wie Adress only!
void build_register_mode_verify(unsigned char regadr, unsigned data)
  {
     prog_message.repeat = 1;
     prog_message.size   = 2;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01110000 | (regadr & 0b111);
     prog_message.dcc[1] = data;

     memcpy(&prog_ctrl, &registermode_ctrl, sizeof(prog_ctrl));
  }
  
void build_register_mode_write(unsigned char regadr, unsigned data)
  {
     prog_message.repeat = 1;
     prog_message.size   = 2;
     prog_message.type   = is_prog;
     prog_message.dcc[0] = 0b01111000 | (regadr & 0b111);
     prog_message.dcc[1] = data;

     memcpy(&prog_ctrl, &registermode_ctrl, sizeof(prog_ctrl));
     prog_ctrl.mode = P_WRITE;
  }


//===================================================================================
//
//   Multitask Routinen
//
//--------------------------------------------------------------------------------
//

/// run_prog_inner_task: multitask replacement, must be called in loop
// howto:
// input:   prog_ctrl must be loaded with the desired operation
//          (mode and no. of cycles for each state)
// output:  pi_result is updated
// requires

static uint32_t ackDetectTimeout;
void run_prog_inner_task(void)
  {
    
    switch (prog_inner_state)
      {
        case PI_IDLE:                                       // ready
            break;

        case PI_START:
            switch(pi_command)
              {
                case PIC_CVM_W_BYTE:             // CV-Mode, write byte
                    build_direct_mode_write(pi_cv, pi_data);
                    break;
                case PIC_CVM_V_BYTE:             // CV-Mode, verify byte
                    build_direct_mode_verify(pi_cv, pi_data);
                    break;
                case PIC_CVM_W_BIT:              // CV-Mode, write single bit
                    build_direct_mode_bit_write(pi_cv, pi_bitpos, pi_data & 0x01);
                    break;
                case PIC_CVM_V_BIT:              // CV-Mode, verify single bit
                    build_direct_mode_bit_verify(pi_cv, pi_bitpos, pi_data & 0x01);
                    break;
                case PIC_RM_W_BYTE:              // register_mode_write         
                    build_register_mode_write(pi_cv, pi_data);
                    break;
                case PIC_RM_V_BYTE:              // register_mode_verify
                    build_register_mode_verify(pi_cv, pi_data);
                    break;
              }

            pi_result = PT_NOACK;                       // default: - no acknowledge
            pDCC_Reset.repeat = prog_ctrl.cycles[0];    // send reset packets
            put_in_queue_prog(dcc_reset_ptr);
            prog_inner_state = DO_1ST_RESET;
            
            break;

        case DO_1ST_RESET:
            if (!queue_prog_is_empty()) return;

            if (prog_ctrl.cycles[1] > 0)
              {                                             // page presets required
                page_preset.repeat = prog_ctrl.cycles[1];     
                put_in_queue_prog(page_preset_ptr);                                            
              }
            prog_inner_state = DO_PAGE_PRESET;
            break;

        case DO_PAGE_PRESET:
            if (!queue_prog_is_empty()) return;             

            if (prog_ctrl.cycles[2] > 0)
              {          
                pDCC_Reset.repeat = prog_ctrl.cycles[2];    // send reset packets
                put_in_queue_prog(dcc_reset_ptr);
              }
            prog_inner_state = DO_2ND_RESET;
            break;

        case DO_2ND_RESET:
            if (!queue_prog_is_empty()) return;

            prog_message.repeat = prog_ctrl.cycles[3];
            put_in_queue_prog(&prog_message);               // send actual programming command                                        
            prog_inner_state = DO_PROG_MESSAGE;
            break;

        case DO_PROG_MESSAGE:                           // now wait for ack or timeout
            if (!queue_prog_is_empty()) return;
            // command has been taken, now check for ACK
            
            if (ACK_IS_DETECTED) 
              {
                unsigned char i,j, partsum;
                for (j=0; j<5; j++)                         // ACK must be present for 6ms
                  {                                         // we check only for approx 1ms
                                                            // and we use a filter: small
                                                            // dropouts are ignored
                    partsum = 0;
                    for (i=0; i<20; i++)
                      {
                        _mydelay_us(10);                   
                        if (ACK_IS_DETECTED) partsum++;
                      }
                    if (partsum < 17) return;               // exits here, seems not to be a real ACK
                  }

                DINT;
                // skip further repetitions -> fool dccout - this is dirty! 
                if (next_message_count>1) next_message_count=1;     
                EINT;
                
                pi_result = PT_OKAY;              // 0 = we got a result
                prog_inner_state = SETUP_3RD_RESET;
                //sds LED_CTRL_ON;
                return;
              }

            //new 15.10.2007
            //sds LED_CTRL_OFF;


            if (next_message_count > 1) return;   // again dirty: we ask the communication flag
                                                  // our message goes with rep 5, so 5...1
                                                  // is our time to wait for ACK;                                               

            //sds LED_CTRL_ON;

            prog_inner_state = SETUP_3RD_RESET;  // state change - timeout reached
            break;

        case SETUP_3RD_RESET:
            if (prog_ctrl.mode == P_READ)
                pDCC_Reset.repeat = 2;
            else
                pDCC_Reset.repeat = prog_ctrl.cycles[4];    // if WRITE: send more reset packets
            put_in_queue_prog(dcc_reset_ptr);
            prog_inner_state = DO_3RD_RESET;
            break;

        case DO_3RD_RESET:
            if (queue_prog_is_empty()) return;

            prog_ctrl.mode = P_IDLE;                         // clear request
            prog_inner_state = PI_IDLE;
            break;

     }
  }


// note: prog_inner_state zu oft abgefragt - verbessern !!!

void run_prog_byte_task(void)
  {
    if (prog_inner_state != PI_IDLE)
      {
        run_prog_inner_task();                               // zuerst mal inner loop fertig.
        return;
      }

    switch (prog_byte_state)
      {
        case PB_IDLE:                                       // all done
            break;

        case PB_START:
            pb_result = PT_NOACK;                            //default: no acknowledge
            pi_data = pb_data;
            pi_cv = pb_cv;
            prog_byte_state = PB_RUNNING;               // default
            switch (pb_command)
              {
                case PBC_RM_R_BYTE:             // read register
                    pi_command = PIC_RM_V_BYTE;
                    pb_test = 0;
                    prog_byte_state = PB_RD_LOOP;
                    pi_data = pb_test;
                    break;
                case PBC_RM_W_BYTE:             // write register
                    pi_command = PIC_RM_W_BYTE;
                    break;
                case PBC_CVM_W_BYTE:            // write byte
                    pi_command = PIC_CVM_W_BYTE;
                    break;
                case PBC_CVM_R_BYTE:            // read byte (scan)
                    pb_test = 0;
                    prog_byte_state = PB_RD_LOOP;
                    pi_command = PIC_CVM_V_BYTE;
                    pi_data = pb_test;
                    break;
                case PBC_CVM_R_BIT:             // read byte (with bit commands)
                    pb_test = 0;
                    prog_byte_state = PB_RD_BIT;
                    pi_command = PIC_CVM_V_BIT;
                    pi_bitpos = 0;
                    pi_data = 1;
                    pb_data = 0;  // init
                    break;
                case PBC_CVM_W_BIT:             // write bit
                    pi_command = PIC_CVM_W_BIT;
                    pi_bitpos = ps_bitpos;      // durchreichen
                    break;
                case PBC_DCCQD:                 // does decoder support single bit
                    pb_test = 0;
                    prog_byte_state = PB_DCCQD;
                    pi_command = PIC_CVM_V_BIT;
                    pi_bitpos = 7;
                    pi_data = 1;                // make a bit verify to cv8, MSB
                    pi_cv = 8;
                    break;
              }
            
            // perform
            prog_inner_state = PI_START;                    // dcc task aufrufen
            break;

        case PB_RUNNING:
            pb_result = pi_result;                    // einfach durchreichen
            pb_data = pi_data;
            prog_byte_state = PB_IDLE;                // we are done
            break;

        case PB_RD_LOOP: 
            // scan loop - repeat inner command until match (same cv)
            if (pi_result == 0)
              {
                // match found
                pb_result = PT_OKAY;
                pb_data = pb_test;
                prog_byte_state = PB_IDLE;          // done, report result
              }
            else
              {
                if (pb_test == 255)
                  {
                    pb_result = PT_TIMEOUT;
                    prog_byte_state = PB_IDLE;      // done, no result
                  }
                else
                  { // try next
                    pb_test++;
                    pi_data = pb_test;
                    prog_inner_state = PI_START;    
                  }
              }
            break;
        case PB_RD_BIT: 
            // scan loop - repeat inner bit command and shift
            if (pi_result == 0)
              {
                pb_data |= (1<<pi_bitpos);   // Bit gelesen, drauf odern
                decoder_can_bit_operations = 1;
                last_bit_check = millis();
              }
            pi_bitpos++;
            if (pi_bitpos == 8)
              {
                // all bits completed - verify pattern
                prog_byte_state = PB_RD_BIT_VERIFY;
                pi_command = PIC_CVM_V_BYTE;
                pi_data = pb_data;
                prog_inner_state = PI_START;
                break;
              }
            else
              { // try next
                pi_data = 1;
                prog_inner_state = PI_START;    
              }
            break;
        case PB_RD_BIT_VERIFY:
            if (pi_result == PT_OKAY)
              {
                pb_result = PT_OKAY;
              }
            else pb_result = PT_BITERR;
            prog_byte_state = PB_IDLE;          // done, report result
            break;
        case PB_DCCQD:
            if (pi_result == PT_OKAY)
              {
                pb_result = PT_OKAY;
                decoder_can_bit_operations = 1;
                last_bit_check = millis();
                prog_byte_state = PB_IDLE;          // done, report result
              }
            else
              {
                pi_data = 0;                        // gleiche Position, aber mit 0
                prog_inner_state = PI_START;
                prog_byte_state = PB_DCCQD2;
              }
            break;
        case PB_DCCQD2:
            if (pi_result == PT_OKAY)
              {
                pb_result = PT_OKAY;
                decoder_can_bit_operations = 1;
                last_bit_check = millis();
                prog_byte_state = PB_IDLE;          // done, report result
              }
            else
              {
                decoder_can_bit_operations = 0;      // no bit operations
                last_bit_check = millis();
                prog_byte_state = PB_IDLE;
              }
            break;
      }
  }


//--------------------------------------------------------------------------------------
/// run_programmer: multitask replacement, must be called in loop
//
// howto:   runs the outer loop of programmer (sequence loop)
//          first checks byte loop - if ready then advance sequence.
//
// input:   prog_ctrl must be loaded with the desired operation
//          (mode and no. of cycles for each state)
//
// output:  prog_result is updated, dcc commands are feed to prog_queue
//          
// requires: organizer is running!

void run_programmer(void)
  {
    if ((millis() - last_bit_check) > TIME_REMEMBER_BIT_OP )
      {
        decoder_can_bit_operations = 0;                     // ist ab jetzt void
        last_bit_check = millis();
      }

    if ((millis() - last_page_loaded) > TIME_REMEMBER_PAGE)
      {
        page_loaded_in_decoder = -1;                     // ist ab jetzt void
        last_page_loaded = millis();
      }


    if (prog_byte_state != PB_IDLE)
      {
        run_prog_byte_task();                               // zuerst mal byte loop fertig.
        return;
      }

    switch (prog_seq_state)
      {
        case PS_IDLE:
            prog_event.busy = 0;            // we are done - no more busy
            break;
        case PS_START:
            prog_event.busy = 1;
            prog_result = PT_ERR;           // !!!???
            pb_cv = prog_cv;
            pb_data = prog_data;
            prog_byte_state = PB_START;      // byte task aufrufen
            switch (ps_command)
              {
                case PSC_DCCRR:                     // read register
                    pb_command = PBC_RM_R_BYTE;
                    prog_result_size = 1;
                    prog_seq_state = PS_RUNNING;
                    break;
                case PSC_DCCWR:                     // write register
                    pb_command = PBC_RM_W_BYTE;
                    prog_result_size = 0;
                    prog_seq_state = PS_RUNNING;
                    break;

                case PSC_DCCWP:                     // write dcc byte, paged mode 
                case PSC_DCCRP:                     // read dcc byte, paged mode (byteweise)
                    // paged mode ist wie register mode, nur wird voher in Register 6 die Seite
                    // geschrieben, und ï¿½ber Register 1..4 darauf zugegriffen;
                    //
                    // CV = 1 + (RRR + 4 * (mypage-1))
                    //
                    // This results in rollover at page 256!
                    // see http://www.users.bigpond.com/pbhandary/dcc/service.html
                    pb_data = (unsigned char)(((prog_cv-1) >> 2) + 1);   // = my_page
                    if (pb_data == page_loaded_in_decoder)
                      {
                        // correct page already loaded, skip page write
                        prog_byte_state = PB_IDLE;
                        pb_result = PT_OKAY;
                      }
                    pb_cv = 6-1; // set register 6 (RRR=101) with my_page
                    pb_command = PBC_RM_W_BYTE;
                    prog_seq_state = PS_WRITE_PAGE_ADR;
                    break;
                case PSC_DCCRD:                     // read dcc byte, direct mode (scan)
                    if (decoder_can_bit_operations)
                      {
                        pb_command = PBC_CVM_R_BIT;
                        prog_result_size = 1;
                        prog_seq_state = PS_RUNNING;
                      }
                    else
                      {
                        // pre check for bit programming 
                        prog_seq_state = PS_CHECK_BITOP;
                        pb_command = PBC_DCCQD;
                        prog_result_size = 1;
                      }
                    break;
                case PSC_DCCRB:                     // read dcc byte, direct mode (bit mode)
                    pb_command = PBC_CVM_R_BIT;
                    prog_result_size = 1;
                    prog_seq_state = PS_RUNNING;
                    break;
                case PSC_DCCWD:                     // write dcc byte, direct mode 
                    pb_command = PBC_CVM_W_BYTE;
                    prog_result_size = 0;
                    prog_seq_state = PS_RUNNING;
                    break;
                case PSC_DCCWB:                     // write dcc bit, direct mode 
                    pb_command = PBC_CVM_W_BIT;
                    prog_result_size = 0;
                    prog_seq_state = PS_RUNNING;
                    break;
                case PSC_DCCQD:                     // can decoder do bit operations? 
                    pb_command = PBC_DCCQD;
                    prog_result_size = 0;
                    prog_seq_state = PS_DCCQD;
                    break;
                case PSC_DCCRL:                     // read long adr
                    pb_command = PBC_CVM_R_BIT;     // start with cv17
                    prog_result_size = 2;
                    prog_seq_state = PS_DCCRL;
                    break;
                case PSC_DCCWL:                     // write long adr
                    pb_command = PBC_CVM_W_BYTE;
                    prog_result_size = 0;
                    prog_seq_state = PS_DCCWL;
                    break;
              }
            break;
        case PS_RUNNING:
          //SDS added cast
            prog_result = (t_prog_result) pb_result;            // einfach durchreichen !!!!
                                                // fallweise feiner unterscheiden !!!
            if (prog_result != PT_OKAY)
              {
                show_prog_error();                  
              }
            prog_data = pb_data;
            prog_event.result = 1;
            prog_seq_state = PS_IDLE;               // we are done -> exit
            break;
        case PS_DCCQD:
            if (pb_result == PT_OKAY)
              {                                     // decoder can do bit 
                prog_result = PT_DCCQD_Y;
              }
            else
              {
                prog_result = PT_DCCQD_N;
              }
            prog_event.result = 1;
            prog_seq_state = PS_IDLE;   // we are done
            break;
        case PS_WRITE_PAGE_ADR:
            if (pb_result == PT_OKAY)
              {
                // page is written - now read or write data
                last_page_loaded = millis();        // save timestamp and page;
                page_loaded_in_decoder = pb_data;
                prog_byte_state = PB_START;
                switch (ps_command)
                  {
                    case PSC_DCCWP:             // write dcc byte, paged mode 
                        pb_data = prog_data;
                        pb_cv = (unsigned char)((prog_cv-1) & 0b11);   // = my_register
                        pb_command = PBC_RM_W_BYTE;
                        prog_byte_state = PB_START;
                        prog_seq_state = PS_RUNNING;
                        prog_result_size = 0;
                        break;
                    case PSC_DCCRP:             // read dcc byte, paged mode (byteweise)
                        pb_data = prog_data;
                        pb_cv = (unsigned char)((prog_cv-1) & 0b11);   // = my_register
                        pb_command = PBC_RM_R_BYTE;
                        prog_byte_state = PB_START;
                        prog_seq_state = PS_RUNNING;
                        prog_result_size = 1;
                        break;
                    default:
                        break;
                  }
              }
            else
              {
                // page write failed
                page_loaded_in_decoder = -1;                     // is void
                show_prog_error();
                prog_result = PT_PAGERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }                  
            break;
        case PS_CHECK_BITOP:                 // bit-Mode check now done, lets call the real read
            prog_result = PT_ERR;             
            pb_cv = prog_cv;
            pb_data = prog_data;
            prog_byte_state = PB_START;      // byte task aufrufen
            prog_result_size = 1;
            prog_seq_state = PS_RUNNING;
            if (decoder_can_bit_operations)
              {
                pb_command = PBC_CVM_R_BIT;
              }
            else
              {
                pb_command = PBC_CVM_R_BYTE;
              }
            break;
        case PS_DCCRL:                      // DCC read long adr
            if (pb_result == PT_OKAY)
              {
                prog_data = pb_data;        // save cv17
                pb_command = PBC_CVM_R_BIT; // now cv18
                prog_byte_state = PB_START;
                pb_cv = 18;
                prog_seq_state = PS_DCCRL2;
              }
            else
              {
                show_prog_error();
                prog_result = PT_ERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            break;
        case PS_DCCRL2:
            if (pb_result == PT_OKAY)
              {
                prog_cv = (unsigned int)(prog_data - 192) * 256 + pb_data;         // calc adr
                prog_result = PT_OKAY;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            else
              {
                show_prog_error();
                prog_result = PT_ERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            break;
        case PS_DCCWL:
            if (pb_result == PT_OKAY)
              {
                pb_command = PBC_CVM_W_BYTE; // now cv18
                prog_byte_state = PB_START;
                pb_cv = 18;
                pb_data = ps_bitpos;
                prog_seq_state = PS_DCCWL2;
              }
            else
              {
                show_prog_error();
                prog_result = PT_ERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            break;
        case PS_DCCWL2:
            if (pb_result == PT_OKAY)
              {
                pb_command = PBC_CVM_W_BIT; // now cv29, bit 5 auf 1
                prog_byte_state = PB_START;
                pb_cv = 29;
                pb_data = 1;
                ps_bitpos = 5;
                prog_seq_state = PS_DCCWL3;
              }
            else
              {
                show_prog_error();
                prog_result = PT_ERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            break;
        case PS_DCCWL3:
            if (pb_result == PT_OKAY)
              {
                prog_result = PT_OKAY;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            else
              {
                show_prog_error();
                prog_result = PT_ERR;
                prog_event.result = 1;
                prog_seq_state = PS_IDLE;   // we are done
              }
            break;
     }
  }



void init_programmer(void)
  {
    unsigned char i;

    last_bit_check = millis();
    decoder_can_bit_operations = 0;                  // is void

    page_loaded_in_decoder = -1;                     // is void
    last_page_loaded = millis();

    prog_event.result = 0;
    prog_inner_state = PI_IDLE;
    prog_byte_state = PB_IDLE;
    prog_seq_state = PS_IDLE;

    // read timing values from eeprom;
    // we do:  eadr_extend_prog_resets:  add this number the number of resets command during programming
    //         eadr_extend_prog_command: add thsi number to the programm commands to get more time

    memcpy (&direct_ctrl, &direct_ctrl_default, sizeof(prog_ctrl));
    memcpy (&registermode_ctrl, &registermode_ctrl_default, sizeof(prog_ctrl));

    //SDS eeprom problem
    i = 3;
    if (i > 10) i= 10;  // limit
    direct_ctrl.cycles[0] += i;
    registermode_ctrl.cycles[0] += i;
    
    //SDS eeprom problem
    i = 3; // from config.cpp
    if (i > 10) i= 10;  // limit
    direct_ctrl.cycles[3] += i;
    registermode_ctrl.cycles[3] += i;
  }


//===========================================================================================
//
// Uhlenbrock Interface (upstream)
// This Interface is used by Xpressnet and Lenz as well
//
// bei IBOX Emulation wird eine Task gestartet, wenn diese fertig ist, hinterlegt sie
// das Ergebnis global.

//-------------------------------------------------------------------------------
//

//-------------------------------------------------------------------------------
//
// 0xEC; XPT_DCCRR (3)
// Action:      Read using register
// Parameters   1st     Register# (1..8)
//              2nd     00
// 
// Reply:       0x00    Ok, accepted
//              0x80    busy
//              0x02    bad parameters

unsigned char my_XPT_DCCRR (unsigned int cv)
  {
    if ((cv < 1) || (cv > 8)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_REGMODE;
    ps_command = PSC_DCCRR;
    prog_cv = cv-1;                         // interner Register Range 0..7
    // ps_step = 0;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xED; XPT_DCCWR (4)
// Action:      Write using register
// Parameters   1st     reg 1..8
//              2nd     0
//              3rd     write value (0..255)
//
// Reply:       0x00    Ok, accepted

unsigned char my_XPT_DCCWR (unsigned int cv, unsigned char data)
  {
    if ((cv < 1) || (cv > 8)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_REGMODE;
    ps_command = PSC_DCCWR;
    prog_cv = cv-1;                             // interner Register Range 0..7
    prog_data = data;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xEE; XPT_DCCRP (3)
// Action:      Read using page mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
// 
// Reply:       0x00    Ok, accepted
//              0x80    busy
//              0x02    bad parameters

unsigned char my_XPT_DCCRP (unsigned int cv)
  {
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_REGMODE;
    ps_command = PSC_DCCRP;
    prog_cv = cv;
    // ps_step = 0;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xEF; XPT_DCCWP (4)
// Action:      Write using page mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
//              3rd     write value (0..255)
//
// Reply:       0x00    Ok, accepted

unsigned char my_XPT_DCCWP (unsigned int cv, unsigned char data)
{
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_REGMODE;
    ps_command = PSC_DCCWP;                     // page mode
    // ps_command = PSC_DCCWD;                     // remap to direct mode (18.07.2010)
    prog_cv = cv;
    prog_data = data;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
}



// 0xF0; XPT_DCCRD (4)
// Action:      Read using direct (byte) mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
// 
// Reply:       0x00    Ok, accepted
//              0x80    busy
//              0x02    bad parameters

unsigned char my_XPT_DCCRD (unsigned int cv)
{
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_CVMODE_B0;
    ps_command = PSC_DCCRD;
    prog_cv = cv;
    
    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
}

//-------------------------------------------------------------------------------
//
// 0xF1; XPT_DCCWD (4)
// Action:      Write using direct (byte) mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
//              3rd     write value (0..255)
//
// Reply:       0x00    Ok, accepted

unsigned char my_XPT_DCCWD (unsigned int cv, unsigned char data)
{
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_CVMODE_B0;
    ps_command = PSC_DCCWD;
    prog_cv = cv;
    prog_data = data;


    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
}


//-------------------------------------------------------------------------------
//
// 0xF2; XPT_DCCRB (3)
// Action:      Read using direct (bit) mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
//                      N.B. a whole CV (8 bits) are read, not only 1 bit!
//
//              
// Reply:       0x00    Ok, accepted !!! wo geht der Wert hin????


unsigned char my_XPT_DCCRB (unsigned int cv)
  {
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_CVMODE_B0;
    ps_command = PSC_DCCRB;
    prog_cv = cv;
    // ps_step = 0;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }


//-------------------------------------------------------------------------------
//
// 0xF3; XPT_DCCWB (5)
// Action:      Write using direct (bit) mode
// Parameters   1st     CV# low byte
//              2nd     CV# high byte  ->  CV# must be in range 1..1024
//              3rd     bitposition (0..7)
//              4th     write value (0..1)
//
// Reply:       0x00    Ok, accepted

unsigned char my_XPT_DCCWB (unsigned int cv, unsigned char bitpos, unsigned char data)
  {
    if ((cv < 1) || (cv > 1024)) return(2);
    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    prog_qualifier = PQ_CVMODE_B0;
    ps_command = PSC_DCCWB;
    prog_cv = cv;
    ps_bitpos = bitpos;
    prog_data = data;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xF4; XPT_DCCQD (1)
// Action:      does (bit) mode?

unsigned char my_XPT_DCCQD(void)
  {
    if (prog_event.busy) return(0x80);    // is busy

     // test auf MSB von CV8
     // if (direct_mode_bit_verify(8, 7, 1)) return(1);
     // if (direct_mode_bit_verify(8, 7, 0)) return(1);

    if (prog_event.busy) return(0x80);    // is busy

    enter_progmode();
    ps_command = PSC_DCCQD;
    prog_cv = 8;
    ps_bitpos = 7;
    prog_data = 1;

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xF5;  XPT_DCCRL 
// Action:      Reads long (loco) address into CV17/18
//              and automatically set bit #5 of CV29 
// Note         Only compatible with decoders supporting direct bit
//              read and write mode!
// Parameters   1st     low byte of long address
//              2nd     high byte of long address
//              
// Reply:       0x00    Ok, accepted 
unsigned char my_XPT_DCCRL (void)
  {
    if (prog_event.busy) return(0x80);    // is busy
    
    enter_progmode();
    ps_command = PSC_DCCRL;
    prog_cv = 17;
    
    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }

//-------------------------------------------------------------------------------
//
// 0xF6;  XPT_DCCWL (0F6h) - length = 1+2 bytes
// Action:      Write long (loco) address into CV17/18
//              and automatically set bit #5 of CV29 
// Note         Only compatible with decoders supporting direct bit
//              read and write mode!
// Parameters   1st     low byte of long address
//              2nd     high byte of long address
//              
// Reply:       0x00    Ok, accepted 

unsigned char my_XPT_DCCWL(unsigned int cv)
  {
    if (prog_event.busy) return(0x80);    // is busy
    
    // Berechnung CV17 und CV18:
    // CV17 = (Lokadresse / 256) + 192
    // CV18 = (Lokadresse % 256)
    // CV29 Bit 5 setzen
    
    enter_progmode();
    ps_command = PSC_DCCWL;
    prog_cv = 17;
    prog_data = (unsigned char)(cv / 256) + 192;
    ps_bitpos = (unsigned char)(cv % 256);       // mal dort ablegen :-(

    prog_seq_state = PS_START;
    run_programmer();                           // auch gleich mal aufrufen
    return(0);
  }
//-------------------------------------------------------------------------------
//
// 0xFE; XPT_Term (1)
// Action:      Terminate (abort) current PT task
// Parameters   none
//
// Reply:       0x00    Ok, terminated (an XEvtPT cmd would now report: 01h, 0F4h)
//              0xF3    no task is active!
//              0xF2    could not terminate task! 

unsigned char my_XPT_Term (void)
  {
    if (prog_event.busy) return(0xF3);   // no task is active!
    // switch prog off !!!
    init_programmer();

    prog_event.result = 1;
    prog_result_size = 0;
    prog_result = PT_TERM;
    return(0);
  }


