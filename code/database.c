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
// file:      database.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-08-25 V0.01 started
//            2008-08-27 V0.02 Transfertask for Database entries
//            2009-03-10 V0.03 added delimiters to write_loco_name
//                             LOK_NAME_LENGTH now per define
//            2009-03-11 V0.04 data base is sent as message and as call
//                             bugfix in total_data_base_size
//
//
//--------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   handling of lok names and speed steps
//            (This is not locobuffer, this is the data base,
//            has nothing to do with the actual dcc content)
//
//            
//--------------------------------------------------------------------------

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <string.h>
#include "config.h"                // general structures and definitions
#include "database.h"


// mega32:   2kByte SRAM, 1kByte EEPROM --> SDS : idem atmega328p
// mega644:  4kByte SRAM, 2kByte EEPROM


t_format dcc_default_format;            // dcc default: 0=DCC14, 2=DCC28, 3=DCC128
                                        // this value is read from eeprom

unsigned char next_search_index;        // was asked continously - this is the index for the parser

unsigned char cur_database_entry;
unsigned char total_database_entry;  // all locos with a name
unsigned char db_message[17]; 
unsigned char db_message_ready;

//------------------------------------------------------------------------
//------------------------------------------------------------------------
// Database only for ADDR and FORMAT
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//
// The default format of OpenDCC is defined in config.h (DCC_DEFAULT_FORMAT)
// and stored in EEPROM (CV24)
//
// Rules: if a loco runs with the default format, it is not stored.
//        if a loco runs at a different format, the address and the format
//        is stored in EEPROM as 16 bit value, which is defined as follows:
//        - upper 2 bits contain the loco format;
//        - lower 14 bits contain the loco address.
//        - if entry == 0x0000, the entry is void.
//  
// 
// Up to 64 locos may have different format (ESIZE_LOCO_FORMAT)

// We allocate this storage to a fixed address to have access from the
// parser with a constant offset to eeprom
// (parser commands: read and write Special Option)
//

t_format get_loco_format(unsigned int addr)
{
  return(dcc_default_format);     // not found - default format
}


unsigned char store_loco_format(unsigned int addr, t_format format)
{
  // SDS-TAPAS : eventueel nog in ram bewaren
  return(1);
}

void init_database(void)
{
  dcc_default_format = DCC_DEFAULT_FORMAT;
}

void rewind_database(void)
{
  next_search_index = 0;
}

unsigned char format_to_uint8(t_format myformat)
{  unsigned char i;
  switch(myformat)
    {
      default:
      case DCC14:
              i = 14; break;
      case DCC27:
              i = 27; break;
      case DCC28:
              i = 28;  break;
      case DCC128:
              i = 126; break;
    }
  return(i);
}
