/******************************************************
 * define.h
 *
 * Doosan Infracore Portable Power STM32 embedded systems 
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ******************************************************/
 
#ifndef _DEFINE_H
#define _DEFINE_H


#define spp(page) asm volatile ("spp %0" :: "i" (page))  
#define ENABLE_INTERRUPTS asm volatile ("ei");
#define DISABLE_INTERRUPTS asm volatile ("di");

// Macros for compiling the same source file in
// different programming models: compact and medium
#ifdef MEDIUM
#define NEAR near
#define FAR far
#else
#define NEAR
#define FAR
#endif


// Enable to build vendor test code that runs the hardware 
//  verification and will not work in an application
//#define VERIFICATION_CODE

//#ifdef VERIFICATION_CODE 


#define SOFTWARE_PN			46630821


#define SOFTWARE_VERSION	1


// Define some 16 bit, non-zero bit pattern to indicate validity
// This must match the one the BootLoader uses to qualify a program
#define VALID_KEY		0x5A3C


//#else

// V0.01		Requires bootloader.  Remove parameter table.
//				 Add CAN response for SW PN and version.
//				 Report ? screen via CAN upon request.
//				 Remove startup_message.  Transmit status unsolicited.
//				 Rework update_autostart_state().
//				 Add character commands to toggle AutoStop and set cooldown time
//				 Reverse byte order for CAN communications
// V0.02		Install CAN command to initialize NOVRAM.
//				 Remove 1sec delay at start to send status earlier.
// V0.03		Cooldown time communicated in minutes
// V1.00		Same as V0.03
// V0.04		Fix 'm' screen.  Sent to China 4/10/12
// V0.05		Set debounce time to 500ms, not adjustable.
// V1.01		Add Auto/Manual and Remote Pressure switch states to PCMD_STATUS 
//				 packet transmitted to the TITAN
//				 Shipped with first AutoStart EAGLE 9/26/2012
// V1.02 		Move PCMD_STATUS receipt to the receive interrupt


//#endif



// The Compressor States
typedef enum
{
	STATE_READY =		0,		// Waiting to start
	STATE_CRANKING =	1,		// Starter is engaged, waiting for 600 RPM
	REV_ENGINE1 =		2,		// Engine revving prior to opening the unloader solenoid
	WAIT_PUMP_UP =		3,		// Wait for the separation tank to get to 50 PSI
	WAIT_INLET_CLOSE =	4,		// 5 second delay while the inlet closes
	RUN_UNLOADED =		5,		// Engine running at idle - not making air
	REV_ENGINE2 =		6,		// Engine revving prior to loading
	PRE_LOADED =		7,		// Wait for time delay before loading
	RUN_LOADED_LO =		8,		// Engine running slow - loaded but unused
	MACHINE_DOWN =		9,		// Machine failed
	AUTOSTARTING =		10,		// Auto start sequence engaged
	STATE_STOPPING =	11,		// Stop the engine and ready for another crank 
	RUN_LOADED_HI =		13,		// Engine running fast - making air
	STATE_COOLDOWN =	14,		// Run at idle for a few minutes to cool the turbo
	STATE_PRECOOL =		15,		// A few seconds of delay before idle
	STATE_VERIFY =		33,		// Hardware verification
    ENG_START =    34
}op_state_def;



typedef unsigned char byte;
typedef unsigned int uint;
typedef unsigned long ulong;

extern op_state_def machine_state;


typedef union
{
	int value;
	byte digit[2];
}sint;


typedef union
{
	int value;
	byte digit[4];
}slong;


/*typedef enum      //Defined in IOdata.h, changed from ST9 project
{
	LED_OFF,
	LED_ON,
	LED_BLINK,
	LED_NEG_BLINK
}led_control_def; */

 
#endif