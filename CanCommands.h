/*****************************************************************
 * CanCommands.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Controller Area Network peripheral hardware setup and interface 
 *  setup and maintainence routines 
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 *****************************************************************/

#ifndef __CANCOMMANDS_H
#define __CANCOMMANDS_H


// Define the upper byte of  the CAN ID field
// PRIORITY(3) << 29 | PF << 19 | PS << 11 | SA << 3 | IDE << 2 | RTR << 1
// Set the outgoing message priority level = 6
//  Pad bit = 0, Data Page = 0, Reserved = 0, IDE = 1, RTR = 0
#define CAN_XMIT_PRIORITY	0xC0000004


// CAN node addresses
typedef enum
{
	CAN_ECU_ADDR =		0,		// Engine ECU
	CAN_MASTER_ADDR =	0x30,	
	
	CAN_TITAN_ADDR =	0x30,	// TITAN controller on mobile compressors
    CAN_AUTOST_ADDR =	0x33,	// Wedge system AutoStart controller
    CAN_G2_AUTO_ADDR =  0x33,   // Gen2 Cortex Autostart with analog pressure sense
	CAN_OTC_ADDR =		0x35,
	CAN_VIEWPORT_ADDR =	0x36,
	CAN_TITAN_SERVER =	0x37,
	CAN_SOMAT_TEST_AD =	0x3E,
	CAN_IQ_MOD_ADDR =	0x3F,	// IQ system controller
	CAN_COMCTR_ADDR =	0x41,	// Security Platform Communication 
	CAN_OUTPUT_ADDR =	0x42,	//  " Additional
	CAN_STARTER_ADDR =	0x43,	//  " Engine
	CAN_OBP_DIS_ADDR =	0x48,	// On Board Power Display in Cab
	CAN_OBP_IO_ADDR =	0x4B,	//  " I/O expansion under chassis - MOTE
	CAN_TEST_ADDR =		0x8F,	// Used to debug
	CAN_ID_ERROR =		0xFE,	// ID assumed on error
	CAN_ALL_ADDR =		0xFF	// All devices
}can_id_def;


// Propriority commands are used with PF_PROP_A/PF_PROP_B messages
// Used to construct a J1939 compatable proprietary message channel
// These command bytes occupy the zero byte of the packet' data fields,
//  leaving up to 7 bytes of communication data.  If more than 7 bytes 
//  are required, multi-packet protocol is used.
typedef enum
{
	PCMD_STATUS =			1,		// To exchange status
	PCMD_PING =				2,		// To verify on-line
	PCMD_DELAYS =			3,		// To exchange warn & crank time
	PCMD_OIL_TEMP =			4,
	PCMD_PRESSURES =		5,		// To broadcast discharge (separation tank)
									//  and regulation pressure
	PCMD_AIREND =			7,		// To display AE oil pressure and sep tank temp
	PCMD_VERSION =			9,		// To display SW ver, ambient T, fuel level, Battery V
	PCMD_TEST_DATA =		0x0A,
	PCMD_HEARTBEAT =		0x0F,
	PCMD_SET_OUTPUT =		0x10,	// Turn an output on
	PCMD_ANALOG_INPUT =		0x13,	// Report an analog input
	PCMD_DIGITAL_I_O =		0x14,	// Report all digital states
//	PCMD_DRIVER_FAULTS =	0x15,	// Report SPI driver faults
//	PCMD_PWM_PERCENT =		0x16,	// Modify a PWM duty cycle
	PCMD_RESET_NOVRAM =		0x1D,	// Initialize NOVRAM
	PCMD_WRITE_NOVRAM =		0x1F,	// Update NOVRAM value
	PCMD_READ_NOVRAM =		0x20,	// Report NOVRAM value
	PCMD_REPROGRAM =		0x23,	// Begin reprogramming a CAN server
	PCMD_PROGRAM_DATA =		0x24,	// Program space data delivery
	PCMD_GET_ID =			0x26,	// Software part number
	PCMD_KILL_POWER =		0x2A,	// Turn off
	PCMD_RESET_OUTPUT =		0x2F,	// Turn an output off
	PCMD_DEBUG_TEST =		0x3F,

    
	PCMD_QUESTION_SCREEN =	0x40,	// Request/terminate question screen
	PCMD_RS232_DATA =		0x41,	// Data transfer via CAN to RS232

	PCMD_COLD_TEMPS =		0x88,	// Request temperatures from cold weather op

//	PCMD_DIAG_STRING =		0x94,
//	PCMD_AUTOSTART_CNTRL =	0x95,
//	PCMD_ENG_CONTROL =		0x96,
//	PCMD_LOAD_CONTROL =		0x97,
	PCMD_FUEL_LEVEL =		0x98,

	PCMD_BATTERY_VOLTS =	0x9B,
	PCMD_MACHINE_STATE =	0x9E,
	PCMD_INTERNAL_TEMP =	0x9F,

	PCMD_SOFTWARE_VER =		0xA2,	// Exchange software versions
	PCMD_REQ_MACHINE_CODE =	0xA3,	// Request the numeric machine ID code
	PCMD_SET_MACHINE_CODE =	0xA4,	// Set the numeric machine ID code	
    PCMD_SET_TITAN_AUTOST_METHOD  = 0xA5,   // swtich closing, opening transducer (1 byte paylod)
    PCMD_SET_AUTOST_TIME  = 0xA6,   // cooldown time (2 byte paylod) 
    PCMD_SET_AUTOST_THRESHOLD = 0xA7    // pressure threshold (2 byte payload)
    
}proprietary_command_def;


// These are added to some proprietary commands
typedef enum
{
	MSG_REQUEST		= 0x0A,
	MSG_RESPONSE	= 0x50
}req_rsp_type;


enum
{
	PF_SPEED =			0,		// Torque / Speed control
	PF_BROADCAST =		0xDF,	// Broadcast on/off
	PF_CAB_MESSAGE =	0xE0,	// Fan Speed request, DPF Regen Inhib/Force to engine
	PF_REQUEST =		0xEA,	// Request a specific PGN
	PF_TPDT =			0xEB,	// Transport protocol data transfer	
	PF_TPCM =			0xEC,	// Transport protocol connection management	
	PF_PROP_A =			0xEF,	// Proprietary A messages
	PF_EEC =			0xF0,	// Electronic engine controller msgs
	PF_DPF =			0xFD,	// 
	PF_PDU =			0xFE,	// PDU specific messages
	PF_PROP_B =			0xFF	// Proprietary B messages use PDU2 format
};


// PS_DPF_CONTROL			0xFD7C	// DPF engine output status from engine
 

// Define the control byte associated with PF_TPCM
// These go in the first byte of data in a TP.CM message
enum
{
	TPCM_RTS =			0x10,
	TPCM_CTS =			0x11,
	TPCM_ACK =			0x13,
	TPCM_BAM =			0x20,	// Broadast Announce Message
	TPCM_ABORT =		0xFF	
};


// Define the specific PDU messages associated with PF_PDU
enum
{
	PS_DM1 =			0xCA,	// Diagnostic message 1
	PS_EEC3 =			0xDF,	// Throttle position for CAN throttle
	PS_HOURS =			0xE5,	// Engine hours
	PS_TIMEDATE =		0xE6,	// Time and date
	PS_IDENTIFY = 		0xEB,	// Make, Model, Serial Number
	PS_ENG_TEMP =		0xEE,	// Engine temperatures
	PS_FLUIDS =			0xEF,	// Fluid levels and pressures
	PS_VEH_SPEED =		0xF1,
	PS_FUEL_USE =		0xF2,
	PS_AMBIENT =		0xF5,	// Ambient conditions
	PS_INLET_EXHT = 	0xF6,	// Inlet and exhaust conditions
	PS_ELEC_POWER =		0xF7,	// Battery potential
	PS_FUEL_LEVEL =		0xFC,
	PS_WATERNFUEL =		0xFF	// Water in fuel message 1 = YES
};


// Define the specific PDU messages associated with PF_EEC
enum
{
	PS_EEC2 =			0x03,	// Accelerator position
	PS_EEC1 =			0x04	// Engine speed, etc.
};


// Define states that control the reprogramming of a CAN server 
//  from a programmer connected to our RS232 port
typedef enum 
{
	PRGM_NORMAL,		// Normal operation, not reprogramming
	PRGM_WAIT_CTS,		// Waiting for the "go ahead" response
	PRGM_DELAY1,		// Wait some time after first response 
	PRGM_DELAY2,		// Wait some time after first response 
	PRGM_READY,			// Got the "go ahead", printing status			
	PROGRAMMING,		// Doin' it
	PRGM_WAIT_ACK,		// Waiting for the "success" indication
	PRGM_SUCCESS,		// Got the "success", printing status
	PRGM_ERROR
}reprog_state_def;


// Define a structure that defines CAN IDs and states during re-programming
// We send a command to start the re-programming cycle to the .requestee 
//  CAN address - usually CAN_ALL_ADDR.  Then the first responder's address 
//  is stored in .responder so we can qualify a message received from a 
//  re-programmed device as an acknowledge of success
typedef struct
{
	can_id_def requestee;
	can_id_def responder;
	reprog_state_def state;
}reprog_server_def;


/*typdef for the configuration struct used to pass cooldown/pressure switch/threshold
configuration data between the cangateway and the autostart modules*/

typedef union _config
{
        char configurationByte;
        struct
        {
            unsigned int bit7   :1  ;
            unsigned int bit6   :1  ;
            unsigned int bit5   :1  ;
            unsigned int bit4   :1  ;
            unsigned int bit3   :1  ;
            unsigned int bit2   :1  ;
            unsigned int CrankOnContact :1;
            unsigned int PressureTransducer :1;
        }Bits;
}configuration;

     
        


extern u16 remote_dump_index;

extern reprog_server_def programming_server;

extern can_id_def this_can_address;

extern configuration asConfiguration;

extern CAN_TxMailBox_TypeDef *select_transmitter(void);
extern bool transmit_proprietary(proprietary_command_def, can_id_def);
extern void can_start_reprogram(can_id_def);
extern void can_send_binary(void);
extern void dm1_message_action(void);
extern void get_engine_serial_number(void);
#endif