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
// file:      database.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-08-25 V0.01 started
//           
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   handling of lok names and speed steps
//            (This is not locobuffer, this is the data base,
//            has nothing to do with the actual dcc content)
//
// interface upstream: 
//
//-----------------------------------------------------------------

void init_database(void);
t_format get_loco_format(unsigned int addr);
unsigned char format_to_uint8(t_format myformat);
unsigned char store_loco_format(unsigned int addr, t_format format);



