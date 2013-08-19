/********************************************************
 * canbus.h
 *
 * Doosan Infracore Portable Power STM32 embedded systems
 * Interface routines for the STM33 Controller Area Network 
 * controller peripherial
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ********************************************************/
 
#ifndef _CANBUS_H
#define _CANBUS_H


// The maximum number of engine alert codes
#define MAXDTCS		5

// AutoStart module commands for a PCMD_STATUS / CAN_CMDSTAT message    //Added from ST9 project
typedef enum 
{
	NO_COMMAND =		0,
	START_ENGINE =		1,
	STOP_ENGINE =		2,
    INIT_COMMAND =      3
}master_command_def;


// When we display our '?' screen via RS232, we access screens from 
//  other devices via CAN and send them via RS232 before sending ours. 
// Define a state machine to manage remote '?' screen access
typedef enum
{
	ACCESS_IDLE,
	ACCESS_TRANSMIT,			// Remote server sending sio_buffer[] over CAN
}remote_access_def;

//Typedef for reprogramming state
typedef enum
{
	NONE,
	REPROGRAM,			// Reprogram status
}reprogram_state_def;

extern u8 controllerMode; //Added  from ST9 source
extern u8 contStatus; //Added  from ST9 source

extern remote_access_def remote_access_state;
extern reprogram_state_def reprogram_state;
extern can_id_def dump_to_address;
extern void setup_command(master_command_def);
extern bool update_delays;

extern bool can_on_line;

extern u8 packets_to_transmit;
extern u16 can_comm_watchdog;

extern u16 dm1_timout_count;
extern u16 master_comm_wd;

extern u32 engine_hours;

extern u16 pressure_range_input;


extern void initialize_can(void);
extern void fill_master_timing(void);
extern void fill_software_identity(void);
extern void can_filter_0_action(CAN_FIFOMailBox_TypeDef *);
extern void access_info_screens(void);
extern void check_can(void);

#endif