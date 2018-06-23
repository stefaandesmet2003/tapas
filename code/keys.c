#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include "config.h" 
#include <string.h>
#include "keys.h"

/*
rotary encoder : 
 * als DT achterloopt op CLK -> wijzerzin turn
 * als DT voorloopt op CLK -> tegenwijzerzin turn
 */


typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} debounceState_t;

typedef struct
{
  //uint8_t Pin; // index in deze array is dezelfde als in keys[i].Pin === keypins[i]
  debounceState_t State;
  uint32_t LastMillis;
  //uint8_t Key; // index in deze array is de keycode
} debouncedKey_t;

// de index in deze array komt overeen met de keycode 
debouncedKey_t keys[NUMBER_OF_DEBOUNCED_KEYS] = {0};
volatile int turns; // aantal turns gedetecteerd vooraleer de main loop er iets mee doet

static void detect_keys (void);


// aangeroepen bij elke change van CLK
__interrupt void xint1_isr(void)
{
  // Acknowledge this interrupt to get more from group 1
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  int clk; 
  int dt;
  clk = GpioDataRegs.GPADAT.bit.GPIO20;
  dt = GpioDataRegs.GPADAT.bit.GPIO21;
/*
  clk = digitalRead(PIN_ROT_CLK); 
  dt = digitalRead(PIN_ROT_DT);
  */
  if (clk == dt)
    turns--;
  else
    turns++;
} // xint1_isr

void keys_Init (void)  
{
    EALLOW;	// This is needed to write to EALLOW protected registers
    PieVectTable.XINT1 = &xint1_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

    //
    // Enable XINT1 and XINT2 in the PIE: Group 1 interrupt 4
    // Enable INT1 which is connected to WAKEINT
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4
    IER |= M_INT1;                              // Enable CPU INT1
    EINT;                                       // Enable Global Interrupts
    
    // ROT_CLK = io20, ROT_DT = io21, ROT_SW = io23
    // configure as inputs without pullup (er zit al 10k pullup op de module)
    EALLOW;
    GpioDataRegs.GPASET.bit.GPIO20 = 0;         // Load the output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;         // input
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;         // disable pullup
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;       // XINT Qual using 6 samples

    GpioDataRegs.GPASET.bit.GPIO21 = 0;         // Load the output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;         // input
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;         // disable pullup

    GpioDataRegs.GPASET.bit.GPIO23 = 0;         // Load the output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;        // GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;         // input
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;         // disable pullup

    GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;

    // configure XINT1
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 20;   // XINT1 is GPIO20
    XIntruptRegs.XINT1CR.bit.POLARITY = 3;      // Falling edge interrupt, 1 = rising edge, 3 = rising & falling
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;        // Enable XINT1

    EDIS;    
  /*
  pinMode(PIN_ROT_CLK,INPUT); // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  pinMode(PIN_ROT_DT,INPUT);  // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  attachInterrupt (0,isr,CHANGE);   // interrupt 0 is always connected to pin 2 on Arduino UNO
  */
  
} // keys_Init

void keys_Update (void)
{
  int copyTurns = 0;
  uint8_t key;

  DINT;
  if (copyTurns != turns)
  {
    // isr heeft een beweging gedetecteerd
    copyTurns = turns;
    turns = 0;
    EINT;
    if (copyTurns > 0) key = KEY_ROTUP;
    else if (copyTurns < 0) key = KEY_ROTDOWN;
    if (keys_Handler)
        keys_Handler (key | KEYEVENT_NONE );
  }
  EINT;

  // de button keys pollen en het debouncing state machine laten werken
  detect_keys ();
 
} // keys_Update

key_t keys_GetState (key_t key)
{
    key_t keyCode;
    keyCode = key & KEYCODEFILTER;
    if ( keyCode < NUMBER_OF_DEBOUNCED_KEYS )
    {
        return (keys[keyCode].State);
    }
    else
        return (keyCode | KEYEVENT_NONE);
} // keys_GetState

/****************************************************************************************/
/****************************************************************************************/
/****************************************************************************************/
/* quick & dirty -> only check ROT_SW for this demo */
static void detect_keys (void)
{
  int keycode;
  for (keycode=0;keycode<NUMBER_OF_DEBOUNCED_KEYS;keycode++)
  {
    switch(keys[keycode].State)
    {
      case UP : 
        if (GpioDataRegs.GPADAT.bit.GPIO23 == 0 )
        {
          keys[keycode].LastMillis = millis();
          keys[keycode].State = DEBOUNCING_DOWN;
        }
        break;
      case DEBOUNCING_DOWN :
        if (GpioDataRegs.GPADAT.bit.GPIO23 != 0 )
        {
          keys[keycode].State = UP;
        }
        else if ( (millis() - keys[keycode].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[keycode].State = DOWN;
          if (keys_Handler)
            keys_Handler ( keycode | KEYEVENT_DOWN);
        }
        break;
      case DOWN :
        if (GpioDataRegs.GPADAT.bit.GPIO23 != 0 )
        {
          keys[keycode].State = DEBOUNCING_UP;
          keys[keycode].LastMillis = millis();
        }
        else if ( (millis() - keys[keycode].LastMillis) > LONGPRESS_DELAY )
        {
          keys[keycode].State = LONG_DOWN;
          if (keys_Handler)
            keys_Handler (keycode | KEYEVENT_LONGDOWN);
        }
        break;
      case LONG_DOWN :
        if (GpioDataRegs.GPADAT.bit.GPIO23 != 0 )
        {
          keys[keycode].State = DEBOUNCING_UP;
          keys[keycode].LastMillis = millis();
        }
        break;
      case DEBOUNCING_UP :
        if (GpioDataRegs.GPADAT.bit.GPIO23 == 0 )
        {
          keys[keycode].LastMillis = millis();
        }
        else if ( (millis() - keys[keycode].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[keycode].State = UP;
          if (keys_Handler)
            keys_Handler (keycode | KEYEVENT_UP);
        }
        break;
    }
  }
} // detect_keys



