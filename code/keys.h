#ifndef _keys_h_
#define _keys_h_

// ui keys
typedef uint8_t key_t;
// key bestaat uit de 6-bit key-code en 2-bit event-type

#define NUMBER_OF_DEBOUNCED_KEYS 1
#define KEY_ENTER   0
#define KEY_INVALID 0xFF

#define KEY_ROTUP       NUMBER_OF_DEBOUNCED_KEYS + 1
#define KEY_ROTDOWN     NUMBER_OF_DEBOUNCED_KEYS + 2

#define KEYCODEFILTER       0x3F  // filter de keycode
#define KEYEVENTFILTER      0xC0  // filter de keyevent

#define KEYEVENT_NONE       0x00
#define KEYEVENT_DOWN       0x40
#define KEYEVENT_UP         0x80
#define KEYEVENT_LONGDOWN   0xC0

#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 1000

void keys_Init (void);
void keys_Update (void);
// returns the key_state (UP, DOWN,LONGDOWN)
key_t keys_GetState (key_t key);
extern void keys_Handler( key_t key ) __attribute__ ((weak));

#endif // _keys_h_

