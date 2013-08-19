/**********************************************************************
 * Serial.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * RS232 Serial Communications Interface maintenance routine prototypes 
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 **********************************************************************/

#ifndef __SERIAL_H
#define __SERIAL_H

#include "IOdata.h"

#define RIGHT_COLUMN	"43"

extern u16 command_timer;

extern void fill_a_hours(u32);
extern void fill_a_minutes(u32);
extern void fill_terminal_info(void);
extern void fill_pressure_input(void);
extern void fill_a_psw_pol(switch_state);
extern void fill_a_psw_mode(pressure_mode_val);
extern void fill_a_switch(switch_state);
extern void fill_machine_state(void);
extern void fill_autostart_state(void);
extern void check_serial(void);
extern void check_reprogram_state();
extern void set_column(u8 *);

#endif



