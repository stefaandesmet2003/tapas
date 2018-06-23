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
// file:      organizer.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de

//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   queue for dcc messages
//            memory for actual loco states
//            engines for repeat and refresh of dcc messages
//
// interface upstream: 
//            init_organizer(void)  // set up the queue structures
//            run_organizer(void)   // multitask replacement, must be called
//                                  // every 2ms (approx)
//
//            bool dodcc(message*)  // put this message in the queues, returns
//                                  // error if full
//            bool do_loco_speed(loco, speed)
//                                  // change speed of this loco
//
// interface downstream: 
//            uses "next_message..." flags to interact with dccout
//
//-----------------------------------------------------------------


//----------------------------------------------------------------------------------
// define the structures for DCC messages
//----------------------------------------------------------------------------------
// SIZE_... : see config.h
//-------------------------------------- primary command buffer (fifo)

extern t_message queue_lp[SIZE_QUEUE_LP];          // low priority
 
extern t_message queue_hp[SIZE_QUEUE_HP];          // high priority

extern t_message repeatbuffer[SIZE_REPEATBUFFER];  // instant repeat

extern struct locomem locobuffer[SIZE_LOCOBUFFER];      // refresh

// ---------------------------------------------------------------------------------
// predefined messages
//
// stored in bss, copied at start to sram

extern t_message DCC_Reset;    // DCC-Reset-Paket
extern t_message DCC_Idle    ;    // DCC-Idle-Paket

// extern t_message DCC_BC_Stop ;    // Broadcast Motor off: // 01DC000S :D=x, C=1 (ignore D)
// extern t_message DCC_BC_Brake ;    // Broadcast Slow down // if S=0: slow down


//---------------------------------------------------------------------------------
// Upstream Interface for parser
//---------------------------------------------------------------------------------

void init_organizer(void);                                      // must be called once at program start

void run_organizer(void);                                       // must be called in a loop!

typedef struct 
  {
    unsigned char halted: 1;                    // if != 0: speed commands are locally forced to zero
    unsigned char lok_stolen_by_pc: 1;          // lok has been stolen by pc control
    unsigned char lok_stolen_by_handheld: 1;    // lok has been stolen by handheld (control transfer)
    unsigned char lok_operated_by_handheld: 1;  // lok has been manual operated
    unsigned char turnout_by_handheld: 1;       // there was an turnout operation from handheld
    unsigned char bidi_new: 1;
  } t_organizer_state;

extern t_organizer_state organizer_state; 

// -- routines for command entry
// -- all do_*** routines have the same return values
#define ORGZ_SLOW_DOWN  0    // Bit 0: last entry to locobuffer slowed down
#define ORGZ_STOLEN     1    // Bit 1: locomotive has been stolen
#define ORGZ_NEW        2    // Bit 1: new entry created for locomotive
#define ORGZ_FULL       7    // Bit 7: organizer fully loaded

// Note on stolen locomotives:
// a) stolen by handheld: 
// a1) from other handheld: return flag 'stolen', orginal owner in orgz_old_lok_owner
// a2) from PC: return flag 'stolen', global flag 'lok_stolen_by_handheld'; loco_buffer.owner_changed is set.        
// b) stolen by PC:
//    return flag 'stolen', global flag 'lok_stolen_by_pc'; loco_buffer.'owner_changed' is set, original owner
//    is kept in locobuffer.slot

extern unsigned char orgz_old_lok_owner;

bool organizer_ready(void);                                     // true if command can be accepted

unsigned char convert_speed_to_rail(unsigned char speed128, t_format format);
unsigned char convert_speed_from_rail(unsigned char speed, t_format format);

unsigned char do_loco_speed(unsigned char slot, unsigned int addr, unsigned char speed);     // eine Lok eintragen (speed 1-127), will be converted to format
unsigned char do_loco_speed_f(unsigned char slot, unsigned int addr, unsigned char speed, t_format format);      // eine Lok eintragen (incl. format)
unsigned char do_loco_func_grp0(unsigned char slot, unsigned int addr, unsigned char funct); 
unsigned char do_loco_func_grp1(unsigned char slot, unsigned int addr, unsigned char funct); 
unsigned char do_loco_func_grp2(unsigned char slot, unsigned int addr, unsigned char funct); 
unsigned char do_loco_func_grp3(unsigned char slot, unsigned int addr, unsigned char funct); 
#if (DCC_F13_F28 == 1)
 unsigned char do_loco_func_grp4(unsigned char slot, unsigned int addr, unsigned char funct); 
 unsigned char do_loco_func_grp5(unsigned char slot, unsigned int addr, unsigned char funct); 
#endif
 unsigned char do_loco_binstates(unsigned char slot, unsigned int addr, unsigned int binstate); 


bool do_loco_restricted_speed(unsigned int addr, unsigned char data);

void do_all_stop(void);
bool do_accessory(unsigned char slot, unsigned int addr, unsigned char output, unsigned char activate);      // turnout

bool do_extended_accessory(unsigned int addr, unsigned char aspect);

bool do_pom_loco(unsigned int addr, unsigned int cv, unsigned char data);                     // program on the main; cv: 1...1024
bool do_pom_loco_cvrd(unsigned int addr, unsigned int cv);                                    // cv 1...1024

bool do_pom_accessory(unsigned int addr, unsigned int cv, unsigned char data);                // program on the main
bool do_pom_accessory_cvrd(unsigned int addr, unsigned int cv);
bool do_pom_ext_accessory(unsigned int addr, unsigned int cv, unsigned char data);
bool do_pom_ext_accessory_cvrd(unsigned int addr, unsigned int cv);

//sds, moved here from xpnet.cpp
unsigned char scan_locobuffer(unsigned int addr);         // Hilfsroutine: locobuffer durchsuchen
#if (DCC_FAST_CLOCK == 1)
 bool do_fast_clock(t_fast_clock* my_clock);
#endif

t_format find_format_in_locobuffer(unsigned int addr);                         // only find

t_format get_loco_format(unsigned int addr);                                   // find + create if not found

unsigned char store_loco_format(unsigned int addr, t_format format);

unsigned int addr_inquiry_locobuffer(unsigned int addr, unsigned char dir);    // returns next addr in buffer

void delete_from_locobuffer(unsigned int addr);

#if (BIDI_SUPPORT >= 2)
void set_location_locobuffer(unsigned int addr, unsigned int did);             // assigns a detector ID to loco
void clear_location_locobuffer(unsigned int did);                              // remove assignment of a detector ID from loco
#endif


bool do_searchid(t_unique_id* test_id);


//------------------------------------------------------------------

void save_turnout(unsigned char slot, unsigned int addr, unsigned char output);

unsigned char recall_turnout(unsigned int addr);  // addr 0.. 

unsigned char recall_turnout_group(unsigned int group_addr);

unsigned char get_number_of_manual_turnout_ops(void);           // collect flags in turnoutmanual

unsigned int recall_manual_turnout(void);                       // get one manual entry and clear it

//------------------------------------------------------------------
// Upstream Interface for programmer
//------------------------------------------------------------------


bool queue_prog_is_empty(void);

unsigned char put_in_queue_prog(t_message *new_message);




//------------------------------------------------------------------------------------------------
// Routines for locobuffer
//------------------------------------------------------------------------------------------------
//
// purpose:   creates a flexible refresh of loco speeds
//
// how:       every speed command is entered to locobuffer.
//            search_locobuffer return a message of the loco
//            to be refreshed.
//            "younger" locos are refreshed more often.
//
// interface: set_loco_mode (int addr, format)
//            init_locobuffer ()

unsigned char enter_speed_to_locobuffer(unsigned char slot, unsigned int addr, unsigned char speed);                      // returns result
 
unsigned char enter_speed_f_to_locobuffer(unsigned char slot, unsigned int addr, unsigned char speed, t_format format);   // returns result

unsigned char enter_func_to_locobuffer(unsigned char slot, unsigned int addr, unsigned char funct, unsigned char grp);   // returns result

 
t_message * search_locobuffer(void);   // returns pointer to dcc message

unsigned char put_in_queue_lp(t_message *new_message);
 
void set_next_message (t_message *newmsg);



