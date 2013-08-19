/****************************************************************
 * eeprom.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * On-chip FLASH memory function prototypes and defines to 
 *  support emulated EEPROM
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/


#ifndef _EEPROM_H
#define _EEPROM_H


#define FLASH_PAGE_SIZE			0x800

// Define the beginning addresses in FLASH memory for NOVRAM structures
#define AVAILABLE_FLASH				0x08010000		// 12 pages
#define FAULT_LOG_ADDRESS			0x0801E000		// reserve 2 pages
#define FLASH_NOVRAM_ADDRESS		0x0801F000		//(FAULT_LOG_ADDRESS + (2 * FLASH_PAGE_SIZE))
#define MACHINE_IDENTITY_ADDRESS	0x0801F800		//(FLASH_NOVRAM_ADDRESS + FLASH_PAGE_SIZE)
#define DATA_TRACE_STORAGE			0x08020000		// End: 0x0807FFFF

// Define the length of an element in the Machine Identity FLASH page
// Make an integral number of identity elements exactly fill a FLASH page
// Maximum string length is two shorter: one for the string identification 
//  and one for a NULL character
#define IDENTITY_LENGTH			32

#define PRESSURE_MAX            175;

// Define a structure for emulated EPROM data
// Make sure there is an integer number of halfwords in this structure
// We write it to FLASH memory and restore it from FLASH memory in 
//  halfwords.
typedef struct
{
	u32 on_time;
	u32 run_time;
    u32 unused_long1;
    u32 unused_long2;

	u16 power_cycles;
    u16 warn_time;
    u16 cooldown_time;
    u16 recrank_delay;
	u16 number_autostarts;
	u16 min_battery;
	u16 min_temperature;
	u16 max_temperature;
	u16 min_ess_temp;
	u16 max_ess_temp;
    u16 pressure_threshold;
    u16 unused_int2;

	u8 autostop_enabled;
	u8 fault_log_index;
    u8 number_cranks;
    u8 pressure_config;
	u8 ad_pressure_mode;
    bool pressure_sw_bool;
	u8 error_count;
	u8 timeout_count;
    u8 comm_loss;
}novram_def;


// The identity is stored as strings with associated type to indicate 
//  the name of this machine, the name of this controller, or the 
//  serial number of the engine ECU.
typedef enum
{
	ID_MACHINE,
	ID_CONTROLLER,
	ID_ENGINE
}identity_type_def;

// Action to take on loss of CAN communication with 
//  the autostart module
enum
{
	ACTION_ALERT =		1,
	ACTION_SHUTDOWN =	2,
	ACTION_RUN =		3
};


extern novram_def ram_novram;
extern u16 fault_transmit_count;
extern u16 health_transmit_count;
extern bool transmit_fault_log;

extern u16 this_on_time;


extern void initialize_novram(void);
extern u16 number_of_novram(void);
extern void read_novram(void);
extern void write_novram(void);
extern void access_novram_log(void);
extern void update_health_timers(void);
extern void write_identity(u8, s8 *);
extern void fill_identity(void);
extern u8 *get_recent_identity(identity_type_def);
extern u8 get_number_identities(identity_type_def);
extern u16 number_of_faults(void);
extern void write_fault_log(void);
extern void fill_fault(u8);
extern void access_fault_log(void);
extern bool compare_serial_number(u8 *);
extern void erase_fault_log(void);
extern void initialize_health(void);

 
#endif