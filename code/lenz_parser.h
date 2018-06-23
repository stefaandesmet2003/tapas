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
// file:      parser.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2008-07-20 V0.2 t_BC_message moved to status.h
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   reads pc-commands from rs232 and generates calls
//            to organizer.c
//


void init_parser(void);
void run_parser(void);
void pc_send_lenz(unsigned char *str);   // *str is the raw message, no xor; xor is added by pc_send


