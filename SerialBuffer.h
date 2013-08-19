/*******************************************************************
 * SerialBuffer.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * RS232 Serial Communications Interface buffer maintenance routine 
 *  defines and prototypes 
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 *******************************************************************/

#ifndef __SERIALBUFFER_H
#define __SERIALBUFFER_H


#define SERIAL_BUFFER_SIZE	2048	// Foreground serial I/O buffer


enum
{
	NULL		= 0,
	STX 		= 2,
	EOT 		= 4,
	ACK 		= 6,
	LINE_FEED	= 0x0A,
	C_RETURN	= 0x0D,
	NAK 		= 0x15,
	ESCAPE		= 0x1B,	
	PROBLEM		= 0x21,		// Internal problem
	QUESTION 	= 0x3F,		// '?'

//	CMD_READ_COMMAND =		0x43,
	
	CMD_RESET_HEALTH =		0x50,

//	CMD_EEPROM_ERASE_TEST = 0x90,
//	CMD_EEPROM_WRITE_TEST =	0x91,

//	CMD_REQ_DATA_LOG =		0x93,
//	CMD_DIAG_STRING =		0x94,
//	CMD_AUTOSTART_CNTRL =	0x95,
//	CMD_ENGINE_CONTROL =	0x96,
//	CMD_LOAD_CONTROL =		0x97,
	CMD_FUEL_LEVEL =		0x98,
	CMD_AMBIENT_TEMP =		0x99,
	CMD_INTERNAL_TEMP =		0x9A,
	CMD_BATTERY_VOLTS =		0x9B,
	CMD_REQ_MACHINE_ID =	0x9C,
	CMD_REQ_CONTROLLER_ID =	0x9D,
	CMD_MACHINE_STATE =		0x9E,

	CMD_DOWNLOAD_SOFTWARE = 0x9F,

	CMD_PART_NUMBER =		0xA1,	// Request the software part number
	CMD_SOFTWARE_VER =		0xA2,	// Request the software version
//	CMD_MACHINE_ID =		0xA3,
//	CMD_INSTALL_KEYCODE =	0xA5,
//	CMD_FAULT_LOG =			0xA9,
//	CMD_ENGINE_ECM_SN =		0xAA,
//	CMD_NUMBER_FAULTS =		0xAC,
//	CMD_ERASE_FAULT_LOG =	0xAF,
									// Reserve 0xB0 - 0xB7 for use by NHP
	CMD_CURRENT_TIME =		0xBA,	// Set the real time clock
	CMD_CURRENT_DATE =		0xBB,	//  "
	CMD_MASTER_SN =			0xBC,	// Identify the connected VT

	CMD_SET_CONTROLLER_ID =	0xBD,	// Set the unique identifier for this controller
	CMD_SET_MACHINE_ID =	0xBF,	// Set the unique identifier for the machine
};


extern u8 sio_buffer[];
extern u16 sio_index;

extern void initialize_serial(void);
extern bool get_serial_character(u8 *);
extern void fill_a_byte(u8);
extern void fill_an_int(u16);
extern void fill_a_long(u32);
extern void fill_a_number(s32, u8);
extern void fill_a_decimal(u32);
extern void fill_a_string(s8 *);
extern void send_a_string(s8 *);
extern bool buffer_is_busy(void);
extern void wait_for_buffer(void);
extern void start_transmitting(void);
extern void send_sio_buffer(void);


#endif
