#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "config.h"
#include "status.h"
#include "keys.h"
#include "organizer.h"

static bool handleSpeedKeys (key_t key);

// alle ui states -> simplified for tapas demo
#define UISTATE_LOC_DO_SPEED    2
#define UISTATE_DEFAULT         UISTATE_LOC_DO_SPEED
#define UISTATE_EVENT_MAINSHORT 100

// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80

#define UI_NUM_SPEED_STEPS 128
#define DCC_MINSPEED 2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED 127
uint32_t speedkeyLastMillis;
uint8_t ui_CurSpeed = 0 | DIRECTION_FORWARD ;
uint8_t ui_State = UISTATE_DEFAULT;

void hwEvent_Handler (hwEvent_t event)
{
  // stub -> TODO
  if (event == HWEVENT_MAINSHORT)
  {
    ui_State = UISTATE_EVENT_MAINSHORT;
  }
  if (event == HWEVENT_PROGSHORT)
  {
    // should not happen on tapas demo
  }
  else if (event == HWEVENT_EXTSTOP)
  {
    // should not happen on tapas demo
  }

} // hwEvent_Handler


void keys_Handler (key_t key)
{
    uint8_t keyCode, keyEvent;
    bool keyHandled = false;

    keyCode = key & KEYCODEFILTER;
    keyEvent = key & KEYEVENTFILTER;
    
    // ignore key up events for now
    if (keyEvent == KEYEVENT_UP)
        return;
    
    
    if (ui_State == UISTATE_EVENT_MAINSHORT)
    {
        if (keyCode == KEY_ENTER)
        {
          set_opendcc_state(RUN_OKAY);
          ui_State = UISTATE_DEFAULT; // DO_LOC_SPEED
        }
        return;
    }

    if ((keyCode == KEY_ROTUP) || (keyCode == KEY_ROTDOWN) || (keyCode == KEY_ENTER))
    {
        keyHandled = handleSpeedKeys (key);
    }
  
} // keys_Handler

static bool handleSpeedKeys (key_t key)
{
    uint8_t keyCode, keyEvent;
    bool keyHandled = false;
    uint8_t speedStep, curSpeed, dirBit;

    keyCode = key & KEYCODEFILTER;
    keyEvent = key & KEYEVENTFILTER;
    
    
    // als we snel aan de knop draaien gaat de speed sneller vooruit
    if ((millis() - speedkeyLastMillis) > 50) speedStep = 1;
    else if ((millis() - speedkeyLastMillis) > 30) speedStep = 3;
    else speedStep = 8;
    speedkeyLastMillis = millis();
        
    curSpeed = ui_CurSpeed & 0x7F; // remove direction bit
    dirBit = ui_CurSpeed & DIRECTION_BIT;
    if (keyCode == KEY_ROTUP)
    {
        curSpeed += speedStep;
        if (curSpeed < DCC_MINSPEED) curSpeed = DCC_MINSPEED; // hiermee ga je van 0 naar 2
        if (curSpeed > DCC_MAXSPEED) curSpeed = DCC_MAXSPEED;
        ui_CurSpeed = curSpeed | dirBit;
        keyHandled = true;
    }
    else if (keyCode == KEY_ROTDOWN)
    {
        curSpeed -= speedStep;
        if ((curSpeed < DCC_MINSPEED) || (curSpeed > DCC_MAXSPEED)) curSpeed = 0;
        ui_CurSpeed = curSpeed | dirBit;
        keyHandled = true;
    }
    else if (keyCode == KEY_ENTER) // de switch op de rotary encoder
    {
        if (curSpeed)
        {
            ui_CurSpeed = dirBit; // zero speed, but leave direction bit
        }
        else
        {
            // enter drukken bij stilstand togglet de rijrichting
            ui_CurSpeed ^= DIRECTION_BIT; 
        }
        keyHandled = true;
    }
    // use fixed loc address = 3 for tapas demo
    if (keyHandled)
    {
      if (organizer_ready())
      {
          do_loco_speed (1,3, ui_CurSpeed);
      }      
    }
    return (keyHandled);
} // handleSpeedKeys

