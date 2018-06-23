#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "config.h"                // general structures and definitions - make your changes here
#include "database.h"              // format and names
#include "status.h"                // led, key, state
#include "dccout.h"                // make dcc
#include "organizer.h"             // manage commands
#include "programmer.h"            // DCC service mode
#include "rs232.h"
#include "lenz_parser.h"
#include "keys.h"

static void init_main(void);
static void build_loko_7a28s(unsigned int nr, signed char speed, t_message *new_message);

void main(void)
{
  Uint32 toggleMillis, dccMillis;
  t_message organizerMessage;
  //
  // Step 1. Initialize System Control:
  // PLL, WatchDog, enable Peripheral Clocks
  // This example function is found in the F2806x_SysCtrl.c file.
  //
  InitSysCtrl();

  //
  // Step 2. Initalize GPIO:
  // This example function is found in the F2806x_Gpio.c file and
  // illustrates how to set the GPIO to it's default state.
  //
  // InitGpio();  // Skipped for this example

  //
  // Step 3. Clear all interrupts and initialize PIE vector table:
  // Disable CPU interrupts
  //
  DINT;

  //
  // Initialize the PIE control registers to their default state.
  // The default state is all PIE interrupts disabled and flags
  // are cleared.
  // This function is found in the F2806x_PieCtrl.c file.
  //
  InitPieCtrl();

  //
  // Disable CPU interrupts and clear all CPU interrupt flags
  //
  IER = 0x0000;
  IFR = 0x0000;

  //
  // Initialize the PIE vector table with pointers to the shell Interrupt
  // Service Routines (ISR).
  // This will populate the entire table, even if the interrupt
  // is not used in this example.  This is useful for debug purposes.
  // The shell ISR routines are found in F2806x_DefaultIsr.c.
  // This function is found in F2806x_PieVect.c.
  //
  InitPieVectTable();

  // config de 2 leds op tapas
  // Configure GPIO34 as a GPIO output pin
  EALLOW;
  GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
  GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
  GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
  GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;
  EDIS;
  GpioDataRegs.GPBSET.bit.GPIO34 = 1; //SET = led off, CLEAR = led on
  GpioDataRegs.GPBSET.bit.GPIO39 = 1; //SET = led off, CLEAR = led on

  // setup
  millis_init();

  init_database();              // loco format and names
  init_dccout();                // timing engine for dcc

  init_rs232(BAUD_19200);

  init_state();                 // 5ms timer tick
  init_parser();                // command parser

  init_organizer();             // engine for command repetition,
                                // memory of loco speeds and types
  init_programmer();            // State Engine des Programmers

  set_opendcc_state(RUN_OKAY);  // start up with power enabled

  // dcc_startup_messages();    // issue defined power up sequence on tracks (sds: vreemd dat dit ook in de GOLD uitgecomment is..)
  keys_Init ();

  // Enable global Interrupts and higher priority real-time debug events
  EINT;   // Enable Global interrupt INTM
  ERTM;   // Enable Global realtime interrupt DBGM

  toggleMillis = 0;
  dccMillis = 0;

  while(1)
  {

    if ((millis() - toggleMillis) >= 500)
    {
      GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1;
      toggleMillis = millis();
    }


    run_state();        // check short and keys
    run_organizer();    // run command organizer, depending on state,
                        // it will execute normal track operation
                        // or programming
    run_programmer();
    run_parser();       // check commands from pc
    keys_Update();

    } // while (1)

} // main

static void dcc_startup_messages (void)
{
  t_message testmess;
  t_message *testmessptr;
  testmessptr = &testmess;

  // 20 Reset Pakete

  testmessptr = &DCC_Reset;
  set_next_message(testmessptr);
  next_message_count = 20;

  while (next_message_count > 0)      // wait
  {
    delay(1);
  }
  // 10 Idle Pakete

  testmessptr = &DCC_Idle;
  set_next_message(testmessptr);
  next_message_count = 10;

  while (next_message_count > 0)      // wait
  {
    delay(1);
  }
}

//
// End of File
//

