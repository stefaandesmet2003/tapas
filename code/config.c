//----------------------------------------------------------------
//
// OpenDCC_XP
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      config.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-10-23 V0.01 started
//            2007-01-27 V0.02 shift with DMX Macro 
//            2007-10-16 V0.03 TURNOUT_FEEDBACK_ACTIVATED
//            2008-01-08 V0.04 serial_id and short_turnoff_time added to eeprom
//                             dcc_default_format added.
//                             num_of_*** added (for each command group) 
//            2008-02-02 V0.05 feedback_size added          
//            2008-06-18 V0.06 external Stop  
//            2008-08-06 V0.07 section .EECV to get eeprom fixed at 0x810000  
//            2008-08-29 V0.08 railcom_enabled 
//            2008-11-15 V0.09 invert accessory als global
//            2009-03-15 V0.10 ext_stop_deadtime added
//
//-----------------------------------------------------------------
//
// purpose:   central station for dcc
// content:   EEPROM configuration
//            
//-----------------------------------------------------------------


#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>                 // using sscanf and sprintf increases prog. by 4k!
#include <inttypes.h>

#include <string.h>

#include "config.h"                // general structures and definitions
#include "rs232.h"                 // tx ready


//======================================================================
// some globals used throughout OpenDCC

unsigned char invert_accessory = 0;  // Uhlenbrock seems to invert red and green
                                     // with accessory commands - Lenz not
                                     // we do the same!
                                     // so we have to decide in a mixed environment
                                     // which side should be inverted
                                     // Bit 1: invert IB, Bit 0: invert Lenz
                                     // read from CV12
                                     

unsigned char xpressnet_feedback_mode;  // defines the address mapping of
										// feedbacks in Xpressnet
                                        // read from CV29

unsigned char bidi_messages_enabled;  // 0: standard OpenDCC
                                      // 1: location messages enabled

const unsigned char opendcc_version PROGMEM = OPENDCC_VERSION;


//======================================================================

// Die folgende (assemblerï¿½hnliche) Defintion der EEPROM-Variablen ist
// erforderlich, um diese compilerunabhï¿½ngig auf feste Adressen zu legen.
// Auf diese Adressen kann von auï¿½en mit den Befehl zum Lesen und Schreiben
// von Sonderoptionen zugegriffen werden.

// The following definitions (quite similar to assembler) are neccesary
// to assign fixed addresses to variables based in EEPROM.
// This is required to allow external access to eeprom with so called
// "special option" commands.

//======================================================================

// this should be linked to beginning of EEPROM!
// Note: AVRStudio doesn't handle memory directives correctly - use custom makefile 'makefile_eecv'
//       for siumlation: normal eeprom
// Offsets are defined in config.h

#if 0 // SDS TAPAS

#if (DEBUG==3)
uint8_t ee_mem[] EEMEM =
#else
   #if (__AVR_ATmega32__)
        uint8_t ee_mem[] EEMEM =
   #elif (__AVR_ATmega644P__)
        uint8_t ee_mem[]  __attribute__((section(".EECV")))=
   #elif (__AVR_ATmega328P__)//SDS : atmega328
        uint8_t ee_mem[] EEMEM =
   #else 
        #warning EEPROM Definition for this AVR missing
   #endif
#endif
{
    [eadr_OpenDCC_Version]          = OPENDCC_VERSION,
    [eadr_baudrate]                 = 0, //sds quick & dirty DEFAULT_BAUD,
    [eadr_OpenDCC_Mode]             =                       // here we code the features of OpenDCC
                                      #if (XPRESSNET_ENABLED ==1)
                                      (1 << 0) |            // Bit 0; 1 = Xpressnet Version
                                      #else
                                      (0 << 0) |            // Bit 0; 0 = Standard Version
                                      #endif  
                                      (0 << 1) |            // Bit 1; reserved
                                      (0 << 2) |            // Bit 2; reserved
                                      (0 << 3) |            // Bit 3; reserved
                                      #if (DCC_FAST_CLOCK==1)
                                      (1 << 4) |            // Bit 4; 1 = FAST CLOCK supported
                                      #else
                                      (0 << 4) |            // Bit 4; 0 = no FAST CLOCK
                                      #endif
                                      #if (LOCO_DATABASE == NAMED)
                                      (1 << 5) |            // Bit 5; 1 = Named Lokdaten
                                      #elif (LOCO_DATABASE == UNNAMED)
                                      (0 << 5) |            // Bit 5; 0 = unnamed Lokdaten
                                      #else
                                      #error
                                      #endif
                                      (0 << 6) |            // Bit 6; reserved
                                      (0 << 7),             // Bit 7; reserved
    
    [eadr_virtual_decoder_l]        = (unsigned char) (LOCO_DATABASE_ACC_ADDR),
    [eadr_virtual_decoder_h]        = (unsigned char) (LOCO_DATABASE_ACC_ADDR >> 8),

    [eadr_VersionMirror]            = OPENDCC_VERSION,      // mirror
    [eadr_CTS_usage]                = 255,                  // corresponds to IB SO 006 (255 never use CTS)
    #if (TURNOUT_FEEDBACK_ACTIVATED == 1)
    [eadr_s88_mode]                 = 0b0010,               // s88 mode = feedback only; see s88.c, S88_FEEDBACK
    #else
    [eadr_s88_mode]                 = 0b0001,               // s88 mode = hw read only; see s88.c, S88_HW_READ
    #endif
                                                            // Bit 0: 0 disabled, 1 hw read enabled
                                                            // Bit 1: 0 feedback disabled, 1 hw enabled 
    [eadr_s88_autoread]             = 0x06,                 // no of s88 bytes to be read automatically (total)
    [eadr_s88_size1]                = 0x02,
    [eadr_s88_size2]                = 0x02,
    [eadr_s88_size3]                = 0x02,
    [eadr_invert_accessory]         = 0x01,                 // bit 0: invert Lenz, bit 1: invert IB
    [eadr_dcc_acc_repeat]           = NUM_DCC_ACC_REPEAT,   // Accessory Command repeat counter
    [eadr_dcc_acc_time]             = 100,                  // turn on time of acc (used by IB, default 100)
    [eadr_startmode_ibox]           = 0x00,                 // 0=Normal Mode,
                                                            // 1=fixed to P50X
    [eadr_feedback_s88_offset]      = 0,                    // offset for feedback numbers in s88 array, given in bytes
    [eadr_feedback_s88_type]        = 2,                    // status, okay, or error feedback
                                                            // 0: positive ack feedback (set s88, if feedback was received)
                                                            // 1: error feedback (set s88, if no feedback was received)
                                                            // 2: coil position
    [eadr_extend_prog_resets]       = 3,                    // add this number the number of resets command during programming
    [eadr_extend_prog_command]      = 3,                    // add this number the number of prog command to releave timing
    [eadr_dcc_pom_repeat]           = NUM_DCC_POM_REPEAT,   // CV20:
    [eadr_dcc_speed_repeat]         = NUM_DCC_SPEED_REPEAT, // CV21
    [eadr_dcc_func_repeat]          = NUM_DCC_FUNC_REPEAT,
    [eadr_reserved023]              = 0,
    [eadr_dcc_default_format]       = DCC_DEFAULT_FORMAT,
    [eadr_railcom_enabled]          = RAILCOM_ENABLED,                    // CV25 - railcom
    [eadr_fast_clock_ratio]         = 8,                    // CV26 - fast clock
    [eadr_reserved027]              = 0,
    [eadr_reserved028]              = 0,
    [eadr_xpressnet_feedback]       = 0,                    // CV29: Xpressnet Feedback mode:
	                                                        //       0: 256trnt,512feedb. 
                                                            //       1: only feedback,
                                                            //       2: only turnouts  
    [eadr_s88_clk_timing]           = 2,                    // CV30: S88 Timing (unit 10us) clk high und clk low; rang 1..16
    #if (TURNOUT_FEEDBACK_ACTIVATED == 1)
    [eadr_feedback_s88_size]        = 64,                   // CV31: default 512 turnouts
    #else
    [eadr_feedback_s88_size]        = 0,                    // CV31: default no feedback
    #endif
    [eadr_s88_total_from_pc]        = 0,
    [eadr_I2C_present]              = 0,                    //  I2C present - return 0  
    [eadr_short_turnoff_time]       = MAIN_SHORT_DEAD_TIME,   // 34: Time until shutdown[eadr_reserved034]              = 0,          
    [eadr_prog_short_toff_time]     = PROG_SHORT_DEAD_TIME,          
    #if (TURNOUT_FEEDBACK_ACTIVATED == 1)
    [eadr_ext_stop_enabled]         = 0,                    // 36: 0=default, 1=enable external Stop Input
    #else
    [eadr_ext_stop_enabled]         = 1,                    // 36: 0=default, 1=enable external Stop Input
    #endif
    [eadr_ext_stop_deadtime]        = EXT_STOP_DEAD_TIME,        // 37: dead time after RUN, in millis(), SDS        
    [eadr_reserved038]              = 0,          
    [eadr_serial_id]                = 1,                    // 39: serial id, must be > 0         
};
#endif // SDS-TAPAS


// SDS-TAPAS
// millis implementation for TAPAS
__interrupt void cpu_timer0_isr(void);

Uint32 millisCounter = 0;
void millis_init()
{
    // sds : config timer 0
    // gebaseerd op InitCpuTimers(), maar enkel voor timer0

    // 1ms period -> 1ms interrupt
    CpuTimer0Regs.PRD.all  = (1000-1);

    // Initialize pre-scale counter to divide by 90 (SYSCLKOUT / 90) -> 1us timer counter
    CpuTimer0Regs.TPR.all  = (90-1);
    CpuTimer0Regs.TPRH.all = 0;

    // Make sure timer is stopped
    CpuTimer0Regs.TCR.bit.TSS = 1;

    CpuTimer0Regs.TCR.bit.TRB = 1; // 1 = reload timer
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    CpuTimer0Regs.TCR.bit.FREE = 0;     // Timer Free Run Disabled
    CpuTimer0Regs.TCR.bit.TIE = 1; // 0 = Disable/ 1 = Enable Timer Interrupt

    // interrupt setup
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Enable CPU INT1 which is connected to CPU-Timer 0
    IER |= M_INT1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Use write-only instruction to set TSS bit = 0
    CpuTimer0Regs.TCR.all = 0x4000; // start the timer0 ! (TIE=1,TSS=0, waarom was dit 0x4001, bits0..3 moeten altijd 0 zijn??)
} // time_init

__interrupt void cpu_timer0_isr(void)
{
    millisCounter++;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

} // cpu_timer0_isr

Uint32 millis()
{
    return millisCounter;
} // millis

Uint32 micros()
{
    return (CpuTimer0Regs.TIM.all + 1000*millisCounter);
} // micros
