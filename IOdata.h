/****************************************************************
 * IOdata.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * On-chip port initialization prototype
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/

#ifndef __IODATA_H
#define __IODATA_H

#define AS_ENABLE_DEBOUNCE		10 

// States of this AutoStart module
typedef enum
{
	AS_IDLE,			// Manual mode
	AS_STARTUP,
	AS_STARTING,
	AS_RUNNING,
	AS_LOADED,
	AS_COOLDOWN,		// Five minute run after pressure is reached
	AS_STOPPED,			// Waiting for the pressure signal to crank
	AS_ERROR
}autostart_state;



//Autostart functionality - from ST9 project
extern autostart_state asState;
			  
extern bool auto_enable_sw;
extern bool startnow_sw;

extern bool startnow_sw_bool;





// Define indexes into digital_input[]
typedef enum
{
	DIN_FILTER,
	DIN_ENABLE_SWITCH,
	DIN_POWER_SIGNAL,
	DIN_UNIT_START,
	DIN_UNIT_LOAD,
	DIN_UNIT_STOP,
	DIN_MANUAL_REGEN,
	DIN_REGEN_INHIBIT,
    DIN_STARTNOW_SW,
	NUMBER_DIGITAL_INPUTS
}digital_index;


// Define bit band access addresses for input/output bits
// PORTE Output Data Register is at 0x4001180C
// Subtract 0x40000000 = 0x1180C
// Multiply by      32 = 0x230180
// Add 4 x bit#		20 = 0x230194
// Add Bit Band base address 0x42000000 to get 0x42230194
enum
{
	POUT1 =				0x422301B4,	// PortE Output bit13: 0x1180C * 32 + (13 * 4) = 0x2301B4
	POUT2 =				0x422301B8,	// PortE Output bit14: 0x1180C * 32 + (14 * 4) = 0x2301B8
	POUT3 =				0x422181A8,	// PortB Output bit10: 0x10C0C * 32 + (10 * 4) = 0x2181A8
	POUT4 =				0x422181AC,	// PortB Output bit11: 0x10C0C * 32 + (11 * 4) = 0x2181AC
	POUT5 =				0x422281B0,	// PortD Output bit12: 0x1140C * 32 + (12 * 4) = 0x2281B0
	POUT6 =				0x422281B4,	// PortD Output bit13: 0x1140C * 32 + (13 * 4) = 0x2281B4
	POUT7 =				0x422201A0,	// PortC Output bit8:  0x1100C * 32 + (8 * 4) = 0x2201A0
	POUT8 =				0x422201A4,	// PortC Output bit9:  0x1100C * 32 + (9 * 4) = 0x2201A4
    POUT9 =             0x4222019C, // PortC Output bit7:  0x1100C * 32 + (7 * 4) = 0x22019C

	// Low side driver / current sense enable, active low
	LSOUT1 =			0x422301A4,	// PortE Output bit9:  0x1180C * 32 + (9 * 4) = 0x2301A4
	LSOUT2 =			0x422301AC,	// PortE Output bit11: 0x1180C * 32 + (11 * 4) = 0x2301AC

	// Switch to ground for average current at AIN8 (LSOUT1)
	IAVG1 =				0x422301A0,	// PortE Output bit8:  0x1180C * 32 + (8 * 4) = 0x2301A0
	// Switch to ground for average current at AIN9 (LSOUT2)
	IAVG2 =				0x422181B8,	// PortB Output bit14: 0x10C0C * 32 + (14 * 4) = 0x2181B8

	LED_GREEN =			0x42230188,	// PORTE Output bit2:  0x1180C * 32 + (2 * 4) = 0x230188
	LED_YELLOW =		0x4223018C,	// PORTE Output bit3:  0x1180C * 32 + (3 * 4) = 0x23018C
	LED_RED =			0x42230190,	// PORTE Output bit4:  0x1180C * 32 + (4 * 4) = 0x230190

	// Thermister pull up terminator enable at the mux outut
	TERM_MUX_PULL_UP =	0x42230194,	// PortE Output bit5:  0x1180C * 32 + (5 * 4) = 0x230194

	// Dry contact pull down terminator enable at the mux output
	TERM_MUX_PULL_DN =	0x42230198,	// PortE Output bit6:  0x1180C * 32 + (6 * 4) = 0x230198

	// Dry contact pull down terminator enable at AIN8
	TERM_AIN8_PULL_DN =	0x422301A8,	// PortE Output bit10: 0x1180C * 32 + (10 * 4) = 0x2301A8

	// Dry contact pull down terminator enable at AIN9
	TERM_AIN9_PULL_DN =	0x422301BC,	// PortE Output bit15: 0x1180C * 32 + (15 * 4) = 0x2301BC

	// Dry contact pull down terminator enable at DIN1
	TERM_DIN1_PULL_DN =	0x42218198,	// PortB Output bit6:  0x10C0C * 32 + (6 * 4) = 0x218198

	// Dry contact pull down terminator enable at DIN2
	TERM_DIN2_PULL_DN =	0x4221819C,	// PortB Output bit7:  0x10C0C * 32 + (7 * 4) = 0x21819C

	// PORTE bit0 is the dry contact pull down terminator enable at DIN3
	//TERM_DIN3_PULL_DN = 0x42230180,	// PortE Output bit0:  0x1180C * 32 + (0 * 4) = 0x230180
    TERM_DIN3_PULL_DN = 0x42230180,	// PortE Output bit0:  0x1180C * 32 + (0 * 4) = 0x230180
	// Analog output enable at DIN2
	AOUT1_ENABLE =		0x42230184,	// PortE Output bit1:  0x1180C * 32 + (1 * 4) = 0x230184

	// Analog output enable at DIN3
	AOUT2_ENABLE =      0x422181A4,	// PortB Output bit9:  0x10C0C * 32 + (9 * 4) = 0x2181A4

	// Power input signal PDREQ from front panel switch, low to request power down
	POWER_SIGNAL =		0x42228138,	// PortD Input bit14: 0x11408 * 32 + (14 * 4) = 0x228138

	// Power hold output to delay power down after PDREQ goes low
	POWER_HOLD =		0x422281BC,	// PortD Output bit15: 0x1140C * 32 + (15 * 4) = 0x2281BC

	// Current sense #1 variable gain active low CS
	POT1_CHIP_SELECT =	0x4223019C,	// PortE Output bit7:  0x1180C * 32 + (7 * 4) = 0x23019C

	// Current sense #2 variable gain active low CS
	POT2_CHIP_SELECT =	0x422181B0,	// PortB Output bit12:  0x10C0C * 32 + (12 * 4) = 0x2181B0

	// DIN1 is digital input but could be changed to TIMER3 Ch2 in for frequency detection
	DIN1_FREQ_INPUT =	0x42218114,	// PortB Input bit5:  0x10C08 * 32 + (5 * 4) = 0x218114
	
	// A harmless unused output to write when a terminator is not used
	TERM_UNUSED =		0x422101BC	// PortA bit15:	0x1080C * 32 + (15 * 4) = 0x2101BC	
};


#define WARNING_ALARM   POUT5
#define AUTO_MODE_LAMP  POUT7
#define LOW_FUEL_LAMP   POUT3  
#define ENGINE_RUN_LAMP POUT1

#define TURN_ON(pin_address)	*(volatile u8 *)pin_address = 1
#define TURN_OFF(pin_address)	*(volatile u8 *)pin_address = 0


typedef enum
{
	SW_OPEN,
	SW_CLOSING_A,
	SW_CLOSING_B,
	SW_CLOSED,
	SW_OPENING_A,
	SW_OPENING_B
}switch_state;


typedef enum
{
	LED_OFF,
	LED_ON,
	LED_BLINK,
	LED_NEG_BLINK
}led_control_def;

typedef enum
{
    Switched,
    Transducer
}pressure_mode_val;

extern pressure_mode_val pressure_mode_current;

extern led_control_def top_red_led,
					   middle_yellow_led,
					   bottom_green_led,
                       bottom_red_led;


extern switch_state digital_input[NUMBER_DIGITAL_INPUTS];

extern void initialize_ports(void);
extern void update_leds(void);
extern void update_digital_in(void);
extern void update_outputs(void);           //Added from ST9 project
extern void autostart_enable_switch(void);  //Added from ST9 project
extern void startnow_state(void);
extern switch_state STN_ACTSW_STATE;

#endif
