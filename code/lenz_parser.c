//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006, 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      parser.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//-----------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   reads pc-commands from rs232 and generates calls
//            to organizer.c
//
// interface upstream:
//            init_parser(void)         // set up the queue structures
//            run_parser(void)          // multitask replacement, must be called
//                                      // every 20ms (approx)
//            event_send(t_BC_message)
//
// interface downstream:
//            rx_fifo_read()            // to rs232
//            rx_fifo_ready()
//            tx_fifo_write()
//            tx_fifo_full()
//
//            pc_send_single_rm(addr);  // to s88
//
//-----------------------------------------------------------------

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>

#include "config.h"                // general structures and definitions
#include "dccout.h"
#include "status.h"                // timeout engine
#include "organizer.h"
#include "rs232.h"

#if (PARSER == LENZ)

#include "programmer.h"            // service mode
#include "lenz_parser.h"

//------------------------------------------------------------------------------
// internal to this module, but static:
enum parser_states
  {                            // actual state
     IDLE,
     WF_MESSAGE,
     WF_XOR,
     CHECK_STOLEN_LOK,
     CHECK_MANUAL_TURNOUT
  } parser_state;

unsigned char pcc[16];          // pc_message speicher

unsigned char pcc_size, pcc_index;

//------------------------------------------------------------------------------
// predefined pc_messages:

unsigned char *pars_pcm;

//-- communication

unsigned char pcm_timeout[] = {0x01, 0x01};                 // Timeout
unsigned char pcm_overrun[] = {0x01, 0x06};                 // too many commands
unsigned char pcm_ack[] = {0x01, 0x04};                     // ack

// generell fixed messages

unsigned char pcm_datenfehler[] = {0x61, 0x80};             // xor wrong
unsigned char pcm_busy[] = {0x61, 0x81};                    // busy
unsigned char pcm_unknown[] = {0x61, 0x82};                 // unknown command
unsigned char pcm_BC_alles_aus[] = {0x61, 0x00};            // Kurzschluï¿½abschaltung
unsigned char pcm_BC_alles_an[] = {0x61, 0x01};             // DCC wieder einschalten
unsigned char pcm_BC_progmode[] = {0x61, 0x02};             // Progmode
unsigned char pcm_BC_locos_aus[] = {0x81, 0x00};            // Alle Loks gestoppt

unsigned char pcm_version[] = {0x63, 0x21, 0x36, 0x00};     // LZ100 Zentrale in Version 3.6
unsigned char pcm_liversion[] = {0x02, 0x10, 0x01};         // LI101F Version 1.0 Code 01
                             // {0x02, 0x30, 0x01};         // LIUSB 3.0


// variable messages

unsigned char pcm_status[] = {0x62, 0x22, 0x45};                  //wird von pc_send_status gebaut.
unsigned char pcm_build[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // max 6 bytes

//-------------------------------------------------------------------------------
//
//   Support Routines for parser
//
//--------------------------------------------------------------------------------
//
/// pc_send_lenz:   generell purpose send answer to pc
/// pc_send_lenz_P: generell purpose send answer stored in program memory to pc

void pc_send_lenz(unsigned char *str)      // vorlï¿½ufig kein Timeout
  {
    unsigned char n, total, my_xor;
 
    n = 0;
    my_xor = str[0];
    total = str[0] & 0x0F;

    while (!tx_fifo_ready()) ;             // busy waiting!

    tx_fifo_write(str[0]);                   // send header

    while (n != total)
      {
         n++;
         my_xor ^= str[n];
         tx_fifo_write(str[n]);              // send data
      }    

    tx_fifo_write(my_xor);                   // send xor

  }


//-----------------------------------------------------------------------------------
// Interface fï¿½r events, die in der Zentrale passieren und an den PC gemeldet werden.
// Dies besteht aus 2 Typen:
// a) Zustandsï¿½nderungen
// b) Rï¿½ckmeldungen
//
// a) Zustandsï¿½nderungen

void event_send(void)
  {
    switch(opendcc_state)
      {
        case RUN_OKAY:             // DCC running
            pc_send_lenz(pars_pcm = pcm_BC_alles_an);
            pc_send_lenz(pars_pcm = pcm_BC_alles_an);
            break;
        case RUN_STOP:             // DCC Running, all Engines Emergency Stop
            pc_send_lenz(pars_pcm = pcm_BC_locos_aus);  
            pc_send_lenz(pars_pcm = pcm_BC_locos_aus);  
            break;
        case RUN_OFF:              // Output disabled (2*Taste, PC)
            pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  
            pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  
            break;
        case RUN_SHORT:            // Kurzschluss
            pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  
            pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  
            break;
        case RUN_PAUSE:            // DCC Running, all Engines Speed 0
            pc_send_lenz(pars_pcm = pcm_BC_locos_aus);  
            pc_send_lenz(pars_pcm = pcm_BC_locos_aus);  
            break;

        case PROG_OKAY:
            pc_send_lenz(pars_pcm = pcm_BC_progmode);  
            pc_send_lenz(pars_pcm = pcm_BC_progmode);    // 19.07.2010 
            break;
        case PROG_SHORT:           //
            break;
        case PROG_OFF:
            break;
        case PROG_ERROR:
            break;
      }
    status_event.changed = 0;            // broad cast done
  }

void pc_send_status(void)
  {
    // Format: Headerbyte Daten 1 Daten 2 X-Or-Byte
    // Hex : 0x62 0x22 S X-Or-Byte
    // S:
    // Bit 0: wenn 1, Anlage in Notaus
    // Bit 1: wenn 1, Anlage in Nothalt
    // Bit 2: Zentralen-Startmode (0 = manueller Start, 1 = automatischer Start)
    // Bit 3: wenn 1, dann Programmiermode aktiv
    // Bit 4: reserviert
    // Bit 5: reserviert
    // Bit 6: wenn 1, dann Kaltstart in der Zentrale
    // Bit 7: wenn 1, dann RAM-Check-Fehler in der Zentrale
    // Besonderheiten: siehe bei Lenz
    unsigned char my_status = 0;
    //SDS bits 0 & 1 omwisselen (xpnet spec)
    // RUN_STOP = emergency stop, alle locs een noodstop
    // RUN_OFF = track power off (booster disabled)
    //SDS if (opendcc_state == RUN_STOP) my_status |= 0x01;
    //SDS if (opendcc_state == RUN_OFF) my_status |= 0x02;
    if (opendcc_state == RUN_STOP) my_status |= 0x02;
    if (opendcc_state == RUN_OFF) my_status |= 0x01;
    // my_status &= ~0x04;  // manueller Start
    if ( (opendcc_state == PROG_OKAY)
       | (opendcc_state == PROG_SHORT)
       | (opendcc_state == PROG_OFF)
       | (opendcc_state == PROG_ERROR) ) my_status |= 0x08;          // Programmiermode
    //SDS niet nodig, want je bent hier al running my_status |= 0x40;  // wir behaupten mal "Kaltstart"
    pcm_status[2] = my_status;
    pc_send_lenz(pars_pcm = pcm_status);
  }

//SDS added - quasi identiek aan xp_send_loco_addr uit xpnet.c
void pc_send_loco_addr(unsigned int addr)     
  {
        pcm_build[0] = 0xE3;
        pcm_build[1] = 0x30;                   // 0x30 + KKKK; here KKKK=0, normal loco addr
        if (addr == 0) pcm_build[1] |= 0x04;   // KKKK=4 -> no result found
        if (addr > XP_SHORT_ADDR_LIMIT)
          {
            pcm_build[2] = addr / 256;
            pcm_build[2] |= 0xC0;
          }
        else pcm_build[2] = 0;
        pcm_build[3] = (unsigned char)addr;
        pc_send_lenz(pars_pcm = pcm_build);
  }

void pc_send_lokdaten(unsigned int addr)
  {
    register unsigned char i, data;
    register unsigned char speed;
    
    i = scan_locobuffer(addr);

    
    
    pcm_build[0] = 0xE4;               // Headerbyte = 0xE4
    pcm_build[1] = 0x00;               // Byte1 = Kennung = 0000BFFF:  B=0: nicht besetzt
                                       //   FFF=Fahrstufen: 000=14, 001=27, 010=28, 100=128
    if (i==SIZE_LOCOBUFFER)            // not found - was not used yet
      {
        pcm_build[1] |= get_loco_format(addr);  // ask eeprom about speed steps
        pcm_build[2] = 0;                      // no Speed
        pcm_build[3] = 0;                      // no functions
        pcm_build[4] = 0; 
      }
    else
      {
        speed = convert_speed_to_rail(locobuffer[i].speed, locobuffer[i].format);
        switch(locobuffer[i].format)
          {
            case DCC14:
                pcm_build[2] = speed;    //Byte2 = Speed = R000 VVVV;
                break;
            case DCC27:
                pcm_build[1] |= 0b001;
                if (speed < 1)
                  {
                    pcm_build[2] = speed; 
                  }
                else
                  {          
                    data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
                    data = (data>>1) | ((data & 0x01) <<4);
                    pcm_build[2] = data | (speed & 0x80); 
                  }
                break;
            case DCC28:
                pcm_build[1] |= 0b010;           
                if (speed < 1)
                  {
                    pcm_build[2] = speed; 
                  }
                else
                  {          
                    data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
                    data = (data>>1) | ((data & 0x01) <<4);
                    pcm_build[2] = data | (speed & 0x80); 
                  }
                break;
            case DCC128:
                pcm_build[1] |= 0b100;          
                pcm_build[2] = speed;    //Byte2 = Speed = RVVV VVVV;
                break;
          }
        pcm_build[3] = (locobuffer[i].fl << 4) | locobuffer[i].f4_f1;
        pcm_build[4] = (locobuffer[i].f12_f9 << 4) | locobuffer[i].f8_f5;
        //F0: Zustand der Funktionen 0 bis 4. 0 0 0 F0 F4 F3 F2 F1
        //F1: Zustand der Funktionen 5 bis 12 F12 F11 F10 F9 F8 F7 F6 F5
      }
    pc_send_lenz(pars_pcm = pcm_build);
  }

#if (DCC_F13_F28 == 1)
void pc_send_funct_level_f13_f28(unsigned int addr)
  {
    register unsigned char i;
    
    i = scan_locobuffer(addr);

    pcm_build[0] = 0xE3;                       // Headerbyte = 0xE3
    pcm_build[1] = 0x52;                       // Byte1 = Kennung
    if (i==SIZE_LOCOBUFFER)                     // not found - was not used yet
      {
        pcm_build[2] = 0;                          // no functions
        pcm_build[3] = 0;                          // no functions
      }
    else
      {
        pcm_build[2] = locobuffer[i].f20_f13;
        pcm_build[3] = locobuffer[i].f28_f21;
      }
    pc_send_lenz(pars_pcm = pcm_build);
  }

//SDS added
// zelfde soort antwoord als func_status voor de F1-F12 (dummy data)
void pc_send_funct_status_f13_f28(unsigned int addr)
  {
    register unsigned char i;
    
    i = scan_locobuffer(addr);

    pcm_build[0] = 0xE4;                       // Headerbyte = 0xE3
    pcm_build[1] = 0x51;                       // Byte1 = Kennung
    pcm_build[2] = 0;                          // momentary/continuous niet opgeslagen in locobuffer
    pcm_build[3] = 0;                          // momentary/continuous niet opgeslagen in locobuffer
    
    pc_send_lenz(pars_pcm = pcm_build);
  }

#endif

// we answer, but this is not really supported (not stored in locobuffer).
//SDS kopie uit xpnet.c
void pc_send_loco_func_status(unsigned int addr)
  {
    pcm_build[0] = 0xE3;                       // Headerbyte = 0xE3
    //SDS opgelet hier stond 0x80, fout!!! aanpassen in xpnet.c ook!!
    pcm_build[1] = 0x50;                       // Byte1 = Kennung = 10000000
    pcm_build[2] = 0x00;                       // Byte2 = 000sSSSS; s=F0, SSSS=F4...F1
    pcm_build[3] = 0;                          // Byte3 = SSSSSSSS; SSSSSSSS=F12...F5
    pc_send_lenz(pars_pcm = pcm_build);
  }
  

void send_lok_stolen(unsigned char slot, unsigned int lokaddr)
  {
    pcm_build[0] = 0xE3;                       // Headerbyte = 0xE3
    pcm_build[1] = 0x40;                       // Byte1 = Kennung = 01000000
    pcm_build[2] = (unsigned char) (lokaddr / 256);                           
    pcm_build[3] = (unsigned char) (lokaddr);   
    if (lokaddr >  XP_SHORT_ADDR_LIMIT)
      {
        pcm_build[2] |= 0xc0;                                              
      }
    pc_send_lenz(pars_pcm = pcm_build);
  }

void send_stolen_loks(void)
{
}


#if (DCC_FAST_CLOCK == 1)
void pc_send_fast_clock(void)
  {
    // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
    pcm_build[0] = 0x05;
    pcm_build[1] = 0x01;
    pcm_build[2] = 0x00 | fast_clock.minute;
    pcm_build[3] = 0x80 | fast_clock.hour;
    pcm_build[4] = 0x40 | fast_clock.day_of_week;
    pcm_build[5] = 0xC0 | fast_clock.ratio;

    pc_send_lenz(pars_pcm = pcm_build);
  }
#endif

void send_prog_result(void)
  {
    // Messages:
	// 61 11: ready
	// 61 12: short - Kurzschluss
	// 61 13: cant read - Daten nicht gefunden
	// 61 1f: busy
	// 63 10 EE D: EE=adr, D=Daten; nur fï¿½r Register oder Pagemode, wenn bei cv diese Antwort, dann kein cv!
	// 63 14 CV D: CV=cv, D=Daten: nur wenn cv gelesen wurde

    if (prog_event.busy) 
      {
        pcm_build[0] = 0x61;
		pcm_build[1] = 0x1f;
		pc_send_lenz(pars_pcm = pcm_build); 
      }
    else
      {
    	switch (prog_result)
    	  {
            case PT_OKAY:
                switch (prog_qualifier)
                  {
                    case PQ_REGMODE:
                        pcm_build[0] = 0x63;
                        pcm_build[1] = 0x10;
                        pcm_build[2] = prog_cv;
                        pcm_build[3] = prog_data;
                        pc_send_lenz(pars_pcm = pcm_build); 
                        break;
                    case PQ_CVMODE_B0:
                        pcm_build[0] = 0x63;
                        pcm_build[1] = 0x14 | ((prog_cv >> 8) & 0x03);        // code: 0x14 .. 0x17  (R.Killmann)
                        pcm_build[2] = prog_cv;
                        pcm_build[3] = prog_data;
                        pc_send_lenz(pars_pcm = pcm_build); 
                        break;
                    default:
                        pcm_build[0] = 0x61;
            		    pcm_build[1] = 0x11;     // ready
    	        	    pc_send_lenz(pars_pcm = pcm_build); 
    		            break; 
                  }
                break;
            case PT_TIMEOUT:            // here all other stuff as well
            case PT_NOACK:
            case PT_NODEC:               // No decoder detected
            case PT_ERR:      
            case PT_BITERR:  
            case PT_PAGERR: 
            case PT_SELX:    
            case PT_DCCQD_Y:
            case PT_DCCQD_N: 
            case PT_TERM:
            case PT_NOTASK:  
            case PT_NOTERM:
                pcm_build[0] = 0x61;
    		    pcm_build[1] = 0x13;               // not found
    		    pc_send_lenz(pars_pcm = pcm_build); 
    		    break; 

            case PT_SHORT:
                pcm_build[0] = 0x61;
    		    pcm_build[1] = 0x12;
    		    pc_send_lenz(pars_pcm = pcm_build); 
    		    break;
          }
      }
  }

///------------------------------------------------------------------------------
/// parse_command analyzes the command received by run_parser()
/// currently only the most important commands are implemented
//

//-----------------------------------------------------------------------------------------------------
// List of all messages from the client
// i: implemented, s: simulated, t: tested
// i | s | t | V. | Code
// - | - | - | new|0x05 0xF1 TCODE1 TCODE2 TCODE3 TCODE4 [XOR] "DCC FAST CLOCK set"
// - | - | - | new|0x01 0xF2 "DCC FAST CLOCK query"
// i | - | - |    |0x21 0x10 0x31 "Request for Service Mode results"
// - | - | - |    |0x22 0x11 REG [XOR] "Register Mode read request (Register Mode (REG=1..8))"
// - | - | - |    |0x23 0x12 REG DAT [XOR] "Register Mode write request (Register Mode)"
// - | - | - |    |0x22 0x14 CV [XOR] "Paged Mode read request (Paged Mode)"
// - | - | - |    |0x22 0x15 CV [XOR] "Direct Mode CV read request (CV mode)"
// - | - | - |    |0x23 0x16 CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=1..256))"
// - | - | - |    |0x23 0x17 CV DAT [XOR] "Paged Mode write request (Paged mode)"
// - | - | - | 3.6|0x23 0x1C CV DAT [XOR] "Direct Mode CV write request (CV mode (CV1024, CV=1..255))"
// - | - | - | 3.6|0x23 0x1D CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=256 ... 511))"
// - | - | - | 3.6|0x23 0x1E CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=512 ... 767))"
// - | - | - | 3.6|0x23 0x1F CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=768 ... 1023))"
// i | - | - |    |0x21 0x21 0x00 "Command station software-version request"
// n | n | n |    |0x22 0x22 00000M00 [XOR] "Set command station power-up mode"
// i | - | - |    |0x21 0x24 0x05 "Command station status request"
// i | - | - | new|0x23 0x28 AddrH AddrL [XOR] "Command station special option read"
// i | - | - | new|0x21 0x29 AddrH AddrL DAT [XOR] "Command station special option write"
// i | - | - |    |0x21 0x80 0xA1 "Stop operations request (emergency off)"
// i | - | - |    |0x21 0x81 0xA0 "Resume operations request"
// i | - | - |    |0x42 Addr Nibble [XOR] "Accessory Decoder/Feedback information request"
// i | - | - |    |0x52 Addr DAT [XOR] "Accessory Decoder operation request"
// i | - | - |    |0x80 0x80 "Stop all locomotives request (emergency stop)"
// i | - | - | 2  |0x91 loco_addr [XOR] "Emergency stop a locomotive"
// i | - | - | 3  |0x92 AddrH AddrL [XOR] "Emergency stop a locomotive"
// n | - | - | 2  |0x9N loco_addr_1 loco_addr_2 etc. loco_addr N [XOR] "Emergency stop selected locomotives"
// n | - | - |    |0xA1 loco_addr [XOR] "Locomotive information request" (X-Bus V1)
// n | - | - |    |0xA2 loco_addr ModSel [XOR] "Locomotive information request" (X-Bus V2)
// n | - | - |    |0xC3 0x05 loco_addr_1 loco_addr_2 [XOR] "Establish Double Header"
// n | - | - |    |0xC3 0x04 loco_addr_1 loco_addr_2 [XOR] "Dissolve Double Header"
// n | - | - |    |0xB3 loco_addr loco_data_1 loco_data_2 [XOR] "Locomotive operation" (X-Bus V1)
// n | - | - |    |0xB4 loco_addr loco_data_1 loco_data_2 ModSel [XOR] "Locomotive operation" (X-Bus V2)
// i | - | - | 3  |0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
// n | - | - |    |0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
// n | - | - |    |0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
// i | - | - |    |0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
// i | - | - | 3  |0xE3 0x07 AddrH AddrL [XOR] "Function status request"
// - | - | - | 3.6|0xE3 0x08 AddrH AddrL [XOR] "Function status request F13-F28"
// - | - | - | 3.6|0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
// i | - | - | 3  |0xE4 0x10 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 14 speed step"
// i | - | - | 3  |0xE4 0x11 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 27 speed step"
// i | - | - | 3  |0xE4 0x12 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 28 speed step"
// i | - | - | 3  |0xE4 0x13 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 128 speed step"
// i | - | - |    |0xE4 0x20 AddrH AddrL Group [XOR] "Function operation instruction - Group 1 f0f4f3f2f1"
// i | - | - |    |0xE4 0x21 AddrH AddrL Group [XOR] "Function operation instruction - Group 2 f8f7f6f5"
// i | - | - |    |0xE4 0x22 AddrH AddrL Group [XOR] "Function operation instruction - Group 3 f12..f9"
// - | - | - | 3.6|0xE4 0x23 AddrH AddrL Group [XOR] "Function operation instruction - Group 4 f20..f13"
// n | - | - |    |0xE4 0x24 AddrH AddrL Group [XOR] "Set function state - Group 1"
// n | - | - |    |0xE4 0x25 AddrH AddrL Group [XOR] "Set function state - Group 2"
// n | - | - |    |0xE4 0x26 AddrH AddrL Group [XOR] "Set function state - Group 3"
// - | - | - | 3.6|0xE4 0x27 AddrH AddrL Group [XOR] "Set function state - Group 4"
// - | - | - | 3.6|0xE4 0x28 AddrH AddrL Group [XOR] "Function operation instruction - Group 5 f28..f21"
// - | - | - | 3.6|0xE4 0x2C AddrH AddrL Group [XOR] "Set function state - Group 5"
// - | - | - | 3.6|0xE5 0x2F AddrH AddrL RefMode [XOR] "Set function refresh mode"
// i | - | - |    |0xE6 0x30 AddrH AddrL 0xE4+C CV DAT [XOR] "Operations Mode Programming byte mode read request"
// n | - | - |    |0xE6 0x30 AddrH AddrL 0xE8+C CV DAT [XOR] "Operations Mode programming bit mode write request"
// i | - | - |    |0xE6 0x30 AddrH AddrL 0xEC+C CV DAT [XOR] "Operations Mode Programming byte mode write request"
// i | - | - | new|0xE6 0x30 AddrH AddrL 0xF0+C CV DAT [XOR] "Operations Mode Programming Accessory write request"
// i | - | - | new|0xE5 0x30 AddrH AddrL 0xF4+C CV [XOR] "Operations Mode Programming Accessory read request"
// i | - | - | new|0xE6 0x30 AddrH AddrL 0xF8+C CV DAT [XOR] "Operations Mode Programming ExtAccessory write request"
// i | - | - | new|0xE5 0x30 AddrH AddrL 0xFC+C CV [XOR] "Operations Mode Programming ExtAccessory read request"
// n | - | - |    |0xE5 0x43 ADR1H ADR1L ADR2H ADR2L [XOR] "Establish Double Header"
// n | - | - |    |0xE5 0x43 ADR1H ADR1L 0x00 0x00 [XOR] "Dissolve Double Header"
// n | - | - |    |0xE4 0x40+R AddrH AddrL MTR [XOR] "Add a locomotive to a multi-unit request"
// n | - | - |    |0xE4 0x42 AddrH AddrL MTR [XOR] "Remove a locomotive from a Multi-unit request"
// i | - | - |    |0xE3 0x44 AddrH AddrL [XOR] "Delete locomotive from command station stack request"
// i | - | - | 3.6|0xF0 [XOR] "Read Version of Interface"
// i | - | - | 3.6|0xF2 0x01 ADR [XOR] "Set Xpressnet ADR"
// i | - | - | 3.6|0xF2 0x02 0x05 [XOR] Baud command




void parse_command(void)
  {
    unsigned int addr;
    unsigned char speed = 0;
    unsigned char activate, coil;
    t_format format;
    
    switch(pcc[0] / 16)   // this is opcode
      {
        default:
            break;
        case 0x0:
            #if (DCC_FAST_CLOCK == 1)
            switch(pcc[1])
              {
                default:
                    break;
                case 0xF1:
                    // set clock
                    for(coil = 2; coil <= (pcc[0] & 0x0F); coil++ )   // use coil as temp
                      {
                        speed = 99;     // use speed as temp
                        switch(pcc[coil] & 0xC0)
                          {
                            case 0x00:  speed = pcc[coil] & 0x3F;
                                        if (speed < 60) fast_clock.minute = speed;
                                        break;
                            case 0x80:  speed = pcc[coil] & 0x3F;
                                        if (speed < 24) fast_clock.hour = speed;
                                        break;
                            case 0x40:  speed = pcc[coil] & 0x3F;
                                        if (speed < 7) fast_clock.day_of_week = speed;
                                        break;
                            case 0xC0:  speed = pcc[coil] & 0x3F;
                                        if (speed < 32) fast_clock.ratio = speed;
                                        break;
                          }
                      }                            
                    do_fast_clock(&fast_clock);

                    // send clock_event to Xpressnet
                    status_event.clock = 1;

                    // now send an answer
                    pc_send_fast_clock();
                    return;
                case 0xF2:
                    // query clock
                    pc_send_fast_clock();
                    return;
              }
            #endif
            break;

        case 0x1: // rudolf killmann
            switch(pcc[0])
              {
                default:
                    break;
                case 0x13:
                    switch(pcc[1])
                      {
                        case 0x01:
                        // 0x13 0x01 B+AddrH AddrL   DCC extended accessory command request
                        //            addr = (unsigned int)((unsigned char)(pcc[2] & 0x07) << 8) + pcc[3];
                        //            uint8_t aspect = (pcc[2] >> 3);
                        //            do_extended_accessory(addr, aspect);
                        addr = (unsigned int)(((unsigned char)(pcc[2] & 0x07)) << 8) + pcc[3];
                        do_extended_accessory(addr, (unsigned char)(pcc[2] >> 3) & 0x1F);
                        pc_send_lenz(pars_pcm = pcm_ack);
                        return;
                      }
                    break;
              }
            break;

        case 0x2:
           switch(pcc[1])
             {
                default:
                    break;
                case 0x10: 
                   // Prog.-Ergebnis anfordern 0x21 0x10 0x31
                   if (opendcc_state >= PROG_OKAY)
                     {
                       //xxxold lprog_send_prog_result_again();  xxxold
                       send_prog_result();
                       return;
                     }
                   // else: Command void -> end of case
                   break;
               case 0x11: 
                   // Prog.-Lesen Register 0x22 0x11 REG X-Or
                   // REG contains the Resister (1...8), this Command has no answer
                   //xxxold lprog_read_register(pcc[2]);
                   my_XPT_DCCRR (pcc[2]);
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack,   // 19.07.2010 
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x12: 
                   // Prog.-Schreiben Register 0x23 0x12 REG DAT X-Or
                   //xxxold lprog_write_register(pcc[2], pcc[3]);
                   my_XPT_DCCWR (pcc[2], pcc[3]); 
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack,   // 19.07.2010 
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x14:
                   // Prog-lesen Pagemode Hex : 0x22 0x14 CV X-Or-Byte
                   //xxxold lprog_read_paged(pcc[2]);
                   if (pcc[2] == 0) addr = 256;
                   else addr = pcc[2];
                   my_XPT_DCCRP (addr);
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x15: 
                   // Prog.-Lesen CV 0x22 0x15 CV X-Or
                   //xxxold lprog_read_cv(pcc[2]);
                   if (pcc[2] == 0) addr = 256;
                   else addr = pcc[2];
                   my_XPT_DCCRD (addr);
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x16: 
                   // Prog.-Schreiben CV 0x23 0x16 CV DAT X-Or
                   //xxxold lprog_write_cv(pcc[2], pcc[3]);
                   if (pcc[2] == 0) addr = 256;
                   else addr = pcc[2];
                   my_XPT_DCCWD (addr,pcc[3]);
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x17: 
                   // Prog.-Schreiben Paging 0x23 0x17 CV DAT X-Or
                   //xxxold lprog_write_paged(pcc[2], pcc[3]);
                   if (pcc[2] == 0) addr = 256;
                   else addr = pcc[2];
                   my_XPT_DCCWP (addr,pcc[3]);
                   if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
                case 0x18:      // Prog.-Lesen CV 0x22 0x18 CV X-Or    // CV 1..255, 1024
                case 0x19:      // Prog.-Lesen CV 0x22 0x19 CV X-Or    // CV 256 .. 511
                case 0x1A:      // Prog.-Lesen CV 0x22 0x1A CV X-Or    // CV 512 .. 767
                case 0x1B:      // Prog.-Lesen CV 0x22 0x1B CV X-Or    // CV 768 .. 1023
                    addr = ((pcc[1] & 0x03) * 256) + pcc[2];
                    if (addr == 0) addr = 1024;
                    my_XPT_DCCRD (addr);
                    if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                   // if not prog_state, we send two broadcasts (from state engine)
                   // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
                case 0x1C:      // Prog.-Schreiben CV 0x23 0x1C CV DAT X-Or; CV: 1..255, 1024
                case 0x1D:      // Prog.-Schreiben CV 0x23 0x1D CV DAT X-Or; CV: 256 .. 511
                case 0x1E:      // Prog.-Schreiben CV 0x23 0x1E CV DAT X-Or; CV: 512 ... 767
                case 0x1F:      // Prog.-Schreiben CV 0x23 0x1F CV DAT X-Or; CV: 768 ... 1024
                    addr = ((pcc[1] & 0x03) * 256) + pcc[2];
                    if (addr == 0) addr = 1024;
                    my_XPT_DCCWD (addr, pcc[3]);  // direct mode
                    if (is_prog_state()) pc_send_lenz(pars_pcm = pcm_ack);  // only ack
                    // if not prog_state, we send two broadcasts (from state engine)
                    // pc_send_lenz(pars_pcm = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                                // the other from state engine
                   return;
               case 0x21:
                    // Softwareversion anfordern 0x21 0x21 0x00
                    // 0 = "LZ 100";
                    // 1 = "LH 200";
                    // 2 = "DPC";
                    // 3 = "Control Plus";
                   pc_send_lenz(pars_pcm = pcm_version);
                   return;
               case 0x24:
                   // Status Zentrale anfordern 0x21 0x24 0x05 ---> wird von TC benutzt!
                   pc_send_status();                          // Statusbyte zurï¿½ckliefern 
                   return;            
               case 0x80:
                   // Alles Aus 0x21 0x80 0xA1 ---> Nothalt
                   pc_send_lenz(pars_pcm = pcm_ack);    // buggy?
                   set_opendcc_state(RUN_OFF);
                   // pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  // passiert mit state
                   return;
               case 0x81:
                   // Alles An 0x21 0x81 0xA0
                   pc_send_lenz(pars_pcm = pcm_ack);    // buggy?
                   set_opendcc_state(RUN_OKAY);    
                   // pc_send_lenz(pars_pcm = pcm_BC_alles_an);    // das wird bereits von set_opendcc_state gemacht
                   return;
             }
           break;

        case 0x4:
            // not yet tested: Schaltinformation anfordern 0x42 ADR Nibble X-Or
            // Hex : 0x42 Adresse 0x80 + N X-Or-Byte
            // fï¿½r Weichen: Adresse = Adr / 4; N=Nibble
            // fï¿½r Rï¿½ckmelder: Adresse = Adr / 8; N=Nibble
            // Antwort:
            // Hex : 0x42 ADR ITTNZZZZ X-Or-Byte
            // ADR = Adresse mod 4
            // I: 1=in work; 0=done -> bei uns immer 0
            // TT = Type: 00=Schaltempf. 01=Schaltempf. mit RM, 10: Rï¿½ckmelder, 11 reserved
            // N: 0=lower Nibble, 1=upper
            // ZZZZ: Zustand; bei Weichen je 2 Bits: 00=not yet; 01=links, 10=rechts, 11:void
            // Bei Rï¿½ckmeldern: Direkt die 4 Bits des Nibbles
            // TC interpretiert das alles als direkt ï¿½bereinanderliegend;
            // also hier und in s88.c folgende Notlï¿½sung:
            //  Adressen 0..63  werden als DCC-Accessory interpretiert, aus dem Turnout-Buffer geladen
            //                  und mit TT 01 oder 00 quittiert. Das bedeutet 256 mï¿½gliche Weichen
            //  Adressen 64-127 werden als Feedback interpretiert, das bedeutet 512 mï¿½gliche Melder

  		      switch(xpressnet_feedback_mode)
  		      {
  		          default:  
  				      case 0:                             // mixed mode    
    		            if (pcc[1] < 64)
    		            {
    		              // Nur fï¿½r Schaltinfo: (range 0..63)
  		                addr = (pcc[1] << 2) + (unsigned char)((pcc[2] & 0x01) << 1);
  		                pcm_build[0] = 0x42;
  		                pc_send_lenz(pars_pcm = pcm_build);           
    		            }
    		            else
    		            { // request feedback info, shift addr locally down (sub 64)
  		                addr = ((pcc[1] - 64) << 3) + (unsigned char)((pcc[2] & 0x01) << 2);
  		                pcm_build[0] = 0x42;
  		                pc_send_lenz(pars_pcm = pcm_build);
    		            }
    		            return;
  		          case 1:                           	   // only feedback
    		            addr = ((pcc[1]) << 3) + (unsigned char)((pcc[2] & 0x01) << 2);
  	                pcm_build[0] = 0x42;
  	                pc_send_lenz(pars_pcm = pcm_build);
    		            return;
		            case 2:                           	   // only schaltinfo
  		              addr = (pcc[1] << 2) + (unsigned char)((pcc[2] & 0x01) << 1);
  	                pcm_build[0] = 0x42;
  	                pc_send_lenz(pars_pcm = pcm_build); 
    		            return;
  		         }           

        case 0x5:
            // Schaltbefehl 0x52 ADR DAT X-Or
            // Hex: 0x52 Adresse 0x80 + SBBO; 
            // Adresse: = Decoder;
            // S: 1=activate, 0=deactivate,
            // BB=local adr,
            // O=Ausgang 0 (red) / Ausgang 1 (grï¿½n)
            // (das wï¿½rde eigentlich schon passend fï¿½r DCC vorliegen, aber lieber sauber ï¿½bergeben)
            addr = (unsigned int) (pcc[1] << 2) + ((pcc[2] >> 1) & 0b011);
            activate = (pcc[2] & 0b01000) >> 3;
            coil = pcc[2] & 0b01;
            if (invert_accessory & 0b01) coil = coil ^ 1;
            do_accessory(0, addr, coil, activate);
            pc_send_lenz(pars_pcm = pcm_ack);
            if (pcc[1] < 0x40)                              // only xpressnet feedback if < 256
              { // 28.07.2008
                pcm_build[0] = 0x42;
                pc_send_lenz(pars_pcm = pcm_build);
              }
            return;
        case 0x7:
            break;

        case 0x8:
            // Alle Loks anhalten 0x80 0x80
            set_opendcc_state(RUN_STOP);                                   // from organizer.c   
            pc_send_lenz(pars_pcm = pcm_ack);
            return;
        case 0x9:
            // Eine Lok anhalten ab V3 0x92 ADR High ADR Low X-Or (Nothalt)
            addr = (pcc[1] & 0x3F) * 256 + pcc[2];
            pc_send_lenz(pars_pcm = pcm_ack);

            // format dieser Lok rausfinden
            register unsigned char i;
            i = scan_locobuffer(addr);
            if (i==SIZE_LOCOBUFFER)     // not found
              { 
                format = get_loco_format(addr);
              }
            else
              {
                format = locobuffer[i].format;
              }
            if (do_loco_speed_f(0, addr, 1, format))   // speed = 1: Nothalt!
              {
                 // pc_send_lenz(wait);
              }    
            // !!! fehlt Frage, ob voll!! und wie starten wir wieder?
            return;
        case 0xE:
            switch(pcc[1] & 0xf0)      // high nibble von pcc[1]:
              {
                //sds : klopt dat hier : default helemaal in 't begin?? gaat die niet alle cases overrulen? NEE
                default:
                    break;
                //case 0x00 gecopieerd uit xpnet.c, want juist geimplementeerd
                case 0x00:
                    // 0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
                    // 0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
                    // 0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
                    // 0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
                    // 0xE3 0x07 AddrH AddrL [XOR] "Function status request"
                    // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
                    addr = ((pcc[2] & 0x3F) * 256) + pcc[3];
                    switch(pcc[1] & 0x0f)
                      {
                        unsigned int result;
                        case 0x00:
                            pc_send_lokdaten(addr);
                            //SDS processed = 1;
                            break;
                        case 0x05:
                            result = addr_inquiry_locobuffer(addr, 1); // forward
                            pc_send_loco_addr(result);
                            //SDS processed = 1;
                            break;
                        case 0x06:
                            result = addr_inquiry_locobuffer(addr, 0); // revers
                            pc_send_loco_addr(result);
                            //SDS processed = 1;
                            break;
                        case 0x07:
                            pc_send_loco_func_status(addr);
                            //SDS processed = 1;
                            break;
                        #if (DCC_F13_F28 == 1)
                            // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
                        case 0x08:
                            pc_send_funct_status_f13_f28(addr);  // !!! Das ist nicht korrekt, wir faken das!!!
                            //SDS processed = 1;
                            break;
                            // 0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
                        case 0x09:
                            pc_send_funct_level_f13_f28(addr);
                            //SDS processed = 1;
                            break;
                        #endif
                      }
                    return;

                case 0x01:            // !!! Adresssuche Lok in Mtr ab V3 0xE4 0x01 + R MTR ADR High ADR Low X-Or
                    break;
                    // !!! Adresssuche MTR ab V3 0xE2 0x03 + R MTR X-Or
                    // !!! Stacksuche Lok ab V3 0xE3 0x05 + R ADR High ADR Low X-Or
                    // !!! Fkt-Status anfordern ab V3 0xE3 0x07 ADR High ADR Low X-Or (tastend-nicht tastend)
                case 0x10:           
                    // !!! Lok Fahrbefehl ab V3 0xE4 Kennung ADR High ADR Low Speed X-Or
                    // if (pcc[1] & 0b00010000 == 0b00010000)
                    format = (pcc[1] & 0x03);   // 0=14, 1=27, 2=28, 3=128 see t_format Definition
                    switch(format)
                      {
                        case DCC14:
                            speed = (pcc[4] & 0x80) | (pcc[4] & 0x0F);   
                            break;
                        case DCC27:
                        case DCC28:
                            if ((pcc[4] & 0x0F) <= 1)               // map 0x?0 to 0 and 0x?1 to 1
                                 speed = pcc[4] & 0x81;             // stop or nothalt
                            else 
                              {
                                speed = ((pcc[4] & 0x0F) << 1) | ((pcc[4] & 0x10) >> 4);
                                speed = speed - 2;                  // map 4..31 to 2..29
                                speed = speed | (pcc[4] & 0x80);    // direction
                              }
                            break;
                        case DCC128:
                            speed = pcc[4];
                            break;
                      }
                    addr = (pcc[2] & 0x3F) * 256 + pcc[3];
                    if (organizer_ready())
                      {
                        unsigned char myspeed;
                        myspeed = convert_speed_from_rail(speed, format); // map lenz to internal 0...127
                        do_loco_speed_f(0, addr, myspeed, format);
                        pc_send_lenz(pars_pcm = pcm_ack);
                      }
                    else
                      {
                        pc_send_lenz(pars_pcm = pcm_busy);
                      }
                    return;
                case 0x20:
                    addr = (pcc[2] & 0x3F) * 256 + pcc[3];
                    switch(pcc[1] & 0x0F)   // !!! Lok Funktionsbefehl ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
                      {
                        case 0:          // Hex : 0xE4 0x20 AH AL Gruppe 1 X-Or-Byte   (Gruppe 1: 000FFFFF) f0, f4...f1
                            if (organizer_ready())
                              {
                                do_loco_func_grp0(0, addr, pcc[4]>>4); // light, f0
                                do_loco_func_grp1(0, addr, pcc[4]);
                                pc_send_lenz(pars_pcm = pcm_ack);
                              }
                            else
                              {
                                pc_send_lenz(pars_pcm = pcm_busy);
                              }
                            return;
                        case 1:          // Hex : 0xE4 0x21 AH AL Gruppe 2 X-Or-Byte   (Gruppe 2: 0000FFFF) f8...f5
                            if (organizer_ready())
                              {
                                do_loco_func_grp2(0, addr, pcc[4]);
                                pc_send_lenz(pars_pcm = pcm_ack);
                              }
                            else
                              {
                                pc_send_lenz(pars_pcm = pcm_busy);
                              }
                            return;
                        case 2:          // Hex : 0xE4 0x22 AH AL Gruppe 3 X-Or-Byte   (Gruppe 3: 0000FFFF) f12...f9
                            if (organizer_ready())
                              {
                                do_loco_func_grp3(0, addr, pcc[4]);
                                pc_send_lenz(pars_pcm = pcm_ack);
                              }
                            else
                              {
                                pc_send_lenz(pars_pcm = pcm_busy);
                              }
                            return;
                        case 3:          // Hex : 0xE4 0x23 AH AL Gruppe 3 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
                            if (organizer_ready())
                              {
                                #if (DCC_F13_F28 == 1)
                                do_loco_func_grp4(0, addr, pcc[4]);
                                pc_send_lenz(pars_pcm = pcm_ack);
                                #endif
                              }
                            else
                              {
                                pc_send_lenz(pars_pcm = pcm_busy);
                              }
                            return;
                        case 4:         // !!! Funktionsstatus setzen ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
                                        // Hex : 0xE4 0x24 AH AL Gruppe 1 (0000SSSS)  S=1: Funktion ist tastend
                                        // Hex : 0xE4 0x25 AH AL Gruppe 2 (0000SSSS)
                                        // Hex : 0xE4 0x26 AH AL Gruppe 3 (0000SSSS)
                        case 8:         // Hex : 0xE4 0x28 AH AL Gruppe 3 X-Or-Byte   (Gruppe 5: FFFFFFFF) f28...f21
                            if (organizer_ready())
                              {
                                #if (DCC_F13_F28 == 1)
                                do_loco_func_grp5(0, addr, pcc[4]);
                                pc_send_lenz(pars_pcm = pcm_ack);
                                #endif
                              }
                            else
                              {
                                pc_send_lenz(pars_pcm = pcm_busy);
                              }
                            return;
                      }
                    break;
                case 0x30:
                    // Prog. on Main Read ab V3.6 0xE6 0x30 AddrH AddrL 0xE4 + C CV DAT [XOR] 
                    // Prog. on Main Bit  ab V3   0xE6 0x30 AddrH AddrL 0xE8 + C CV DAT X-Or
                    // Prog. on Main Byte ab V3   0xE6 0x30 AddrH AddrL 0xEC + C CV DAT X-Or

                    // Note: we ignore DAT for read commands
                    // Note: Xpressnet does only PoM for Loco, no Accessory!
                    addr = ((pcc[2] & 0x3F) * 256) + pcc[3];
                      {
                        unsigned int xp_cv;
                        unsigned char xp_data;
                        xp_cv = (pcc[4] & 0x03) * 256 + pcc[5];       // xp_cv has the range 0..1023!
                        xp_cv++;                                      // internally, we use 1..1024
                        xp_data = pcc[6];
                        if ((pcc[4] & 0xFC) == 0xEC)
                          {
                            do_pom_loco(addr, xp_cv, xp_data);        //  program on the main (byte mode)
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                        else if ((pcc[4] & 0xFC) == 0xE4)  // 02.04.2010
                          {
                            do_pom_loco_cvrd(addr, xp_cv);           //  pom cvrd the main (byte mode)
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                        else if ((pcc[4] & 0xFC) == 0xE8)
                          {
                            // bit mode unsupported
                          }
                        else if ((pcc[4] & 0xFC) == 0xF0)
                          {
                            do_pom_accessory(addr, xp_cv, xp_data);
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                        else if ((pcc[4] & 0xFC) == 0xF4)
                          {
                            do_pom_accessory_cvrd(addr, xp_cv);
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                        else if ((pcc[4] & 0xFC) == 0xF8)
                          {
                            do_pom_ext_accessory(addr, xp_cv, xp_data);
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                        else if ((pcc[4] & 0xFC) == 0xFC)
                          {
                            do_pom_ext_accessory_cvrd(addr, xp_cv);
                            pc_send_lenz(pars_pcm = pcm_ack);
                            return;
                          }
                      }
                    break;
                case 0x40:   //Lokverwaltung
                    // !!! Lok zu MTR hinzufï¿½gen ab V3 0xE4 0x40 + R ADR High ADR Low MTR X-Or
                    // !!! Lok aus MTR entfernen ab V3 0xE4 0x42 ADR High ADR Low MTR X-Or
                    // !!! DTR-Befehle ab V3 0xE5 0x43 ADR1 H ADR1 L ADR2 H ADR2 L X-Or
                    // !!! Lok aus Stack lï¿½schen ab V3 0xE3 0x44 ADR High ADR Low X-Or
                    break;
              }
            break;
        case 0xF:
            if (pcc[0] == 0xF0) // ask LI-Version
              {
                pc_send_lenz(pars_pcm = pcm_liversion);
                return;
              }
            switch(pcc[1])
              {
                default:
                    break;
                case 0x01:   // ask / set slot addr
                    if ((pcc[2] < 1) || (pcc[2] > 31)) pcc[2] = 1;  // if out of range: set to 1
                    pc_send_lenz(&pcc[0]);
                    return;
                case 0x02:    // setze Baud (getestet 19.05.2006)
                             // Antwort: F2 02 Baud, wie Aufruf (Antwort noch in der alten Baudrate, dann umschalten)
                             // BAUD = 1 19200 baud (Standardeinstellung)
                             // BAUD = 2 38400 baud
                             // BAUD = 3 57600 baud
                             // BAUD = 4 115200 baud
                             // nach BREAK (wird als 000) empfangen sollte Interface default auf 19200
                             // schalten
                    if ((pcc[2] < 1) || (pcc[2] > 4)) pcc[2] = 1;
                    pc_send_lenz(&pcc[0]);

                    while(!tx_all_sent());                  // busy waiting until all sent
                        
                    //SDS : added cast!!
                    init_rs232((t_baud)pcc[2]);             // jetzt umschalten und fifos flushen
                    return;
              }
      }
    pc_send_lenz(pars_pcm = pcm_unknown);   // wer bis hier durchfï¿½llt, ist unbekannt!
  }



//====================================================================================================
/// run_parser: multitask replacement, must be called in loop


static bool input_ready(void)
  {
    if(rs232_break_detected == 1)
      {
        init_parser();
        init_rs232(BAUD_19200);  // reinit rs232 and get rid of old connection
      }
    return(rx_fifo_ready());
  }


void run_parser(void)
  {
    unsigned char i, my_check;
        
    if (status_event.changed) 
      {
        event_send();                                   // report any Status Change
      }

    switch (parser_state)
      {
        case IDLE:
            if (!input_ready()) return;
            pcc[0] = rx_fifo_read();                        // read header
            pcc_size = pcc[0] & 0x0F;
            pcc_index = 0;                                  // message counter
            no_timeout.parser = 250;      // 250ms - see status.c; aanpassing voor arduino
            parser_state = WF_MESSAGE;
            break;

       case WF_MESSAGE:
            if (pcc_index == pcc_size)
              {
                parser_state = WF_XOR;
                return;
              }
            if (!input_ready())
              {
                if (no_timeout.parser)  return;
                else
                  {
                    parser_state = IDLE;
                    pc_send_lenz(pars_pcm = pcm_timeout);      // throw exception, if timeout reached
                    return;
                  }
              }
            pcc_index++;
            pcc[pcc_index] = rx_fifo_read();
            break;

       case WF_XOR:
            if (!input_ready())
              {
                if (no_timeout.parser)  return;
                else
                  {
                    parser_state = IDLE;
                    pc_send_lenz(pars_pcm = pcm_timeout);        // throw exception, if timeout reached
                    return;
                  }
              }
            my_check = 0;  
            for (i=0; i<=pcc_size; i++) my_check ^= pcc[i];   
            if (my_check != rx_fifo_read())
              {
                // XOR is wrong!
                pc_send_lenz(pars_pcm = pcm_datenfehler);
                parser_state = IDLE;
                return;
              }
           
            parse_command();       // analyze received message and send code

            parser_state = IDLE;
            break;
        case CHECK_STOLEN_LOK:
        case CHECK_MANUAL_TURNOUT:
            parser_state = IDLE;
            break;
     }
  }



void init_parser(void)
  {
    parser_state = IDLE;
    
  }

#endif // (PARSER == LENZ)

