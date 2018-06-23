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
// file:      status.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   control of LED via state engines
//            timertick
//            check for key strokes and output short
//            shorts are stored and and a status_event.changed is set.
//            It is up to the selected parser, to report this event
//            to the host.
//               Lenz uses a push system (event_send), IB gets polled.           
//
// 2do:       output short for prog mode not yet impl.
// 
//-----------------------------------------------------------------


#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>

#include "config.h"                 // general structures and definitions
#include "status.h" 
#include "organizer.h"               
#include "programmer.h"               

 #include "lenz_parser.h"             


//---------------------------------------------------------------------------
// data for timeout and debounce
// every task loads its  corresponding member with the desired timeout value
// ISR timer_tick decrements this value every tick (when not 0)

t_no_timeout no_timeout;

unsigned char main_short_ignore_time;   // time from detect of a short until disable output. is loaded during init from eeprom
unsigned char prog_short_ignore_time;   // is loaded during init from eeprom

//-------------------------------------------------------------------------------------

t_opendcc_state opendcc_state;            // this is the current running state

t_status_event status_event;              // message flag for other programs

unsigned char ext_stop_enabled = 0;       // if true: external stop input is enabled (CV36) 

uint32_t ext_stop_deadtime = EXT_STOP_DEAD_TIME;      // CV37
uint32_t extStopOkLastMillis;

uint32_t state_timerval; //sds : zelfde type als millis()
// values in ms
#define FAST_RECOVER_ON_TIME        1
#define FAST_RECOVER_OFF_TIME       4
#define SLOW_RECOVER_TIME         1000  // 1s voor we opnieuw NO_SHORT melden na een kortsluiting
//sds #define BLOCK_SHORT_after_PowerOn     25    // Shorts are ignored immediately after power on, voorlopig niet gebruikt (copy from bidi-booster)

typedef enum {NO_SHORT, IGNORE_SHORT, FASTREC_OFF, FASTREC_ON, SHORT} shortState_t;
shortState_t mainShortState,progShortState;
uint32_t shortLastMillis;
uint8_t shortFastRecoverAttemptsLeft;
static bool main_short_check();
static bool prog_short_check();


void init_state(void)
{
    ext_stop_enabled = 0;
    extStopOkLastMillis = 0;


    // load timing values from eeprom -> TAPAS NO
    main_short_ignore_time = 8; // sds : in ms

    prog_short_ignore_time = 40; // sds : in ms

    // clear all timeouts
    no_timeout.parser = 0;  // sds: nog nodig in lenz_parser.cpp

    #if (DCC_FAST_CLOCK==1)
      fast_clock.ratio = 8; //SDS from config.cpp
    #endif
    mainShortState = NO_SHORT;
    progShortState = NO_SHORT;
    
    // TAPAS : configure I/O
    // nSHORTMAIN = io54, input with pullup
    // swEnableMain = io40, output
    EALLOW;
    GpioDataRegs.GPBSET.bit.GPIO54 = 0;         // Load the output latch
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;        // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;         // input
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;         // enable pullup

    GpioDataRegs.GPBSET.bit.GPIO40 = 0;         // Load the output latch -> start with swEnableMain disabled
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;        // GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;         // output

    EDIS;        

} // init_state


void timeout_tick(void)
{
    if (no_timeout.parser) no_timeout.parser--;  
    //if (no_timeout.main_short) no_timeout.main_short--;  
    //if (no_timeout.prog_short) no_timeout.prog_short--;  
} // timeout_tick

//---------------------------------------------------------------------------------
// set_opendcc_state(t_opendcc_state next)
//
// Einstellen des nï¿½chsten Zustand mit:
// - Stellen entsprechender Bits in der Hardware und Rï¿½cksetzen der
//   ï¿½berwachungstasks ->
//   damit wird nach "power on" wieder Strom auf den Ausgang gelegt.
// - Stellen der LEDs
// - Setzen der globalen Variablen, fallweise Nachricht an den Host
//


static void prog_on(void)
{
    PROG_TRACK_ON;
}
static void prog_off(void)
{
    PROG_TRACK_OFF;
    reset_programmer();
}

static void main_on(void)
{
    MAIN_TRACK_ON;
}
static void main_off(void)
{
    MAIN_TRACK_OFF;
}

void set_opendcc_state(t_opendcc_state next)
{
  if (next != RUN_OKAY)
  {
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
  }
  else
  {
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
  }

  if (next == opendcc_state) return;      // no change

  opendcc_state = next;
  status_event.changed = 1;
  status_event.changed_xp = 1;

  switch(next)
  {
      case RUN_OKAY:                  // DCC running
          prog_off();
          main_on();
          organizer_state.halted = 0;   // enable speed commands again
          break;
      case RUN_STOP:                  // DCC Running, all Engines Emergency Stop
          do_all_stop();
          prog_off();
          main_on();
          break;
      case RUN_OFF:                   // Output disabled (2*Taste, PC)
        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
          prog_off();
          main_off();
          status_event.power_off = 1;
          break;
      case RUN_SHORT:                 // Kurzschluï¿½
          prog_off();
          main_off();
          status_event.power_off = 1;
          break;
      case RUN_PAUSE:                 // DCC Running, all Engines Speed 0
          prog_off();
          main_on();
          break;

      case PROG_OKAY:
          prog_on();
          main_off();
          organizer_state.halted = 0;   // enable speed commands again
          break;
      case PROG_SHORT:                //
          prog_off();
          main_off();
          status_event.power_off = 1;
          break;
      case PROG_OFF:
          prog_off();
          main_off();
          status_event.power_off = 1;
          break;
      case PROG_ERROR:
          prog_on();
          main_off();
          break;
  }
} // set_opendcc_state

#if (DCC_FAST_CLOCK==1)

unsigned int fc_value;

t_fast_clock fast_clock =      // we start, Monday, 8:00
  {
    0,    // unsigned char minute;
    8,    // unsigned char hour;
    0,    // unsigned char day_of_week;
    8,    // unsigned char ratio;
  };


void dcc_fast_clock_step(void)     // this occurs every 5ms realtime (sds : overblijfsel van TICK_PERIOD)
  {
    if (fast_clock.ratio)
      {
        fc_value += fast_clock.ratio;
        if (fc_value >= 12000)         // 1min = 60000ms 
          {
            fc_value = 0;
            fast_clock.minute++;
            if (fast_clock.minute >= 60)
              {
                fast_clock.minute = 0;
                fast_clock.hour++;
                if (fast_clock.hour >= 24)
                  {
                    fast_clock.hour = 0;
                    fast_clock.day_of_week++;
                    if (fast_clock.day_of_week >=7) fast_clock.day_of_week = 0;
                  }
              }
            // now send this to DCC (but not during programming or when stopped)
            if (opendcc_state == RUN_OKAY) do_fast_clock(&fast_clock);

            // send clock_event to Xpressnet
            status_event.clock = 1;
          }
      }
  }

#endif // DCC_FAST_CLOCK
//-----------------------------------------------------------------
// run_state is a task, called periodically to check for key and
// output shorts.
//


// Hinweis: RUN_PAUSE wird zur Zeit nicht angesprungen

void run_state(void)
{
    if ((millis() - state_timerval) > 5) 
    {
        state_timerval = millis();
        timeout_tick();
        #if (DCC_FAST_CLOCK==1)
            dcc_fast_clock_step();
        #endif
    }
    // check main short
    if ((opendcc_state != RUN_SHORT) && (opendcc_state != PROG_SHORT))
    {
        if (main_short_check() == true)
        {
            set_opendcc_state(RUN_SHORT);
            if (hwEvent_Handler)
                hwEvent_Handler(HWEVENT_MAINSHORT);
        }
        // check prog short
        if (prog_short_check() == true)
        {
            set_opendcc_state(PROG_SHORT);
            if (hwEvent_Handler)
                hwEvent_Handler(HWEVENT_PROGSHORT);
        }
    }
    // check external stop
    if ((ext_stop_enabled) && (opendcc_state != RUN_OFF))
    {
        if (!EXT_STOP_ACTIVE) extStopOkLastMillis = millis();
        else if ((millis() - extStopOkLastMillis) > ext_stop_deadtime)
        {
            // we hebben een extStop event!
            set_opendcc_state(RUN_OFF);
            if (hwEvent_Handler)
                hwEvent_Handler(HWEVENT_EXTSTOP);
        }
    }

} // run_state

// returns true, if we are in prog state
// sds, houden, gebruikt door lenz_parser
unsigned char is_prog_state(void)
{
    unsigned char retval = 0;
    switch(opendcc_state)
    {
        case RUN_OKAY:             // wir laufen ganz normal:
        case RUN_STOP:             // DCC Running, all Engines Emergency Stop
        case RUN_OFF:              // Output disabled (2*Taste, PC)
        case RUN_SHORT:            // Kurzschluï¿½
        case RUN_PAUSE:            // DCC Running, all Engines Speed 0
            retval = 0;
            break;
    case PROG_OKAY:
    case PROG_SHORT:           //
    case PROG_OFF:
    case PROG_ERROR:
        retval = 1;
        break;
    }
    return(retval);
} // is_prog_state


// return : false = no_short (ook tijdens de fast recovery), true = short
static bool main_short_check()
{
    bool retval = false; // no_short
    switch(mainShortState)
    {
        case NO_SHORT :
            
            if (MAIN_IS_SHORT)
            {
                mainShortState =  IGNORE_SHORT;
                shortLastMillis = millis();
            }
            break;
        case IGNORE_SHORT :
            if (!MAIN_IS_SHORT)
            {
                mainShortState = NO_SHORT;
                MAIN_TRACK_ON;
            }
            else
            {
                if ((millis() - shortLastMillis) > main_short_ignore_time)
                {
                    mainShortState = FASTREC_OFF;
                    shortFastRecoverAttemptsLeft = 3;
                    MAIN_TRACK_OFF;
                    shortLastMillis = millis();
                }
            }
            break;
        case FASTREC_OFF : 
            if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) // 4ms uit, en dan opnieuw aan en zien of de short weg is
            {
                MAIN_TRACK_ON;
                mainShortState = FASTREC_ON;
                shortLastMillis = millis();
            }
            break;
        case FASTREC_ON : 
            if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) // 1ms wachten en dan short checken
            {
                if (MAIN_IS_SHORT)
                {
                    mainShortState = FASTREC_OFF;
                    shortLastMillis = millis();
                    MAIN_TRACK_OFF;
                    shortFastRecoverAttemptsLeft--;
                    if (!shortFastRecoverAttemptsLeft)
                    {
                        mainShortState = SHORT;
                        retval = true;
                    }
                }
                else
                {
                    mainShortState = NO_SHORT;
                    MAIN_TRACK_ON;
                }
            }
            break;
        case SHORT : 
            if (MAIN_IS_SHORT)
            {
                shortLastMillis = millis();
                retval = true;
            }
            else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME)
            {
                mainShortState = NO_SHORT; 
            }
            break;
    }
    return (retval);
    
} // main_short_check

// return : false = no_short (ook tijdens de fast recovery), true = short
static bool prog_short_check()
{
    bool retval = false; // no_short
    switch(progShortState)
    {
        case NO_SHORT :
            
            if (PROG_IS_SHORT)
            {
                progShortState =  IGNORE_SHORT;
                shortLastMillis = millis();
            }
            break;
        case IGNORE_SHORT :
            if (!PROG_IS_SHORT)
            {
                progShortState = NO_SHORT;
                PROG_TRACK_ON;
            }
            else
            {
                if ((millis() - shortLastMillis) > prog_short_ignore_time)
                {
                    progShortState = FASTREC_OFF;
                    shortFastRecoverAttemptsLeft = 3;
                    PROG_TRACK_OFF;
                    shortLastMillis = millis();
                }
            }
            break;
        case FASTREC_OFF : 
            if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) // 4ms uit, en dan opnieuw aan en zien of de short weg is
            {
                PROG_TRACK_ON;
                progShortState = FASTREC_ON;
                shortLastMillis = millis();
            }
            break;
        case FASTREC_ON : 
            if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) // 1ms wachten en dan short checken
            {
                if (PROG_IS_SHORT)
                {
                    progShortState = FASTREC_OFF;
                    shortLastMillis = millis();
                    PROG_TRACK_OFF;
                    shortFastRecoverAttemptsLeft--;
                    if (!shortFastRecoverAttemptsLeft)
                    {
                        progShortState = SHORT;
                        retval = true;
                    }
                }
                else
                {
                    progShortState = NO_SHORT;
                    PROG_TRACK_ON;
                }
            }
            break;
        case SHORT : 
            if (PROG_IS_SHORT)
            {
                shortLastMillis = millis();
                retval = true;
            }
            else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME)
            {
                progShortState = NO_SHORT; 
            }
            break;
    }
    return (retval);
    
} // prog_short_check
