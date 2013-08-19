/****************************************************************
 * E3prom.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors 
 *  
 * Functions to maintain and access on-chip FLASH memory used to 
 *  emulate EEPROM - "EEEPROM"
 *
 * 2/27/2013 CLA V0.01	Initial code taken from TitanShow
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "Interrupt.h"
#include "Timer.h"
#include "Analog.h"
#include "CanPackets.h"
#include "CanCommands.h"
#include "Canbus.h"
#include "IOdata.h"
#include "Serial.h"
#include "SerialBuffer.h"
#include "Flash.h"
#include "E3prom.h"


#define NUMBER_INTS_PER_FAULT		25
#define NUMBER_FAULTS_PER_PAGE		(FLASH_PAGE_SIZE / (NUMBER_INTS_PER_FAULT * 2))


// These RAM timers are incremented every system clock tick - 10ms
u16 this_on_time = 0;

// Count down while transmitting fault log
u16 fault_transmit_count = 0,
	health_transmit_count = 0;


// If TRUE, transmit entire fault log on request
// If FALSE, show only fault code
bool transmit_fault_log;


/*---------------------------------------------------------------------
 * NOVRAM is emulated using FLASH memory which must be erased a page at 
 *  a time.  Page size for this processor is 2k bytes.
 * The Fault Log uses two FLASH pages.  If a page is full, the entire 
 *  other page is erased when the next fault occurs.  A fault log entry 
 *  consists of several operational values captured at the time of the 
 *  fault.  An entry does not span across two pages so a FLASH page may 
 *  not be exactly full.
 * Non-volatile parameters use a single FLASH memory page.  They are 
 *  copied to RAM at boot and maintained there.  At power down, we copy 
 *  them to the next available slot in the Novram FLASH page.  If the 
 *  FLASH page is full, it is erased and the Health Parameters are 
 *  written to the first slot.
 * The Machine Identity uses a single FLASH memory page.  It consists 
 *  of a text string describing the machine, a text string describing 
 *  this controller, and a text string that is the engine controller's
 *  serial number.  The text strings are limited to the same size and 
 *  are identified by their first character which is not part of the 
 *  string.  They can be in any order and can repeat without repeating 
 *  the others.  That is, when one is changed, it is written in the 
 *  next available memory slot without re-writing the other two.  When 
 *  the Machine Identity FLASH memory page becomes full, no additional 
 *  changes are allowed.
 * There are user controls to erase the Fault Log, the Health 
 *  Parameters or the Machine Identity.
 *--------------------------------------------------------------------*/

// Define RAM space for the non-volatile parameters
novram_def ram_novram;


void initialize_novram(void)
{
	// The E3PROM has not been initialized

	send_a_string("\n\rInitializing NOVRAM");
	initialize_health();

	ram_novram.error_count = 0;

	ram_novram.min_ess_temp = 0xF000;
	ram_novram.max_ess_temp = 0;
    ram_novram.pressure_threshold = PRESSURE_MAX;
    ram_novram.comm_loss = ACTION_ALERT;
    ram_novram.autostop_enabled = TRUE;
    ram_novram.cooldown_time = SECONDS(15);

    ram_novram.number_cranks = 5;
    ram_novram.pressure_config = 0x00;
    ram_novram.ad_pressure_mode = FALSE;
    ram_novram.pressure_sw_bool = FALSE;
	ram_novram.warn_time = SECONDS(12);
	ram_novram.recrank_delay = SECONDS(8);


}


/*----------------------------------------------------------------
 * next_novram_address()
 *
 * Search the non-volatile FLASH page for the next available slot
 * Return the address of the beginning of that slot
 * If the page is full, the returned address will be within one 
 *  slot of the end of the page, or in case the page is filled
 *  exactly, the returned address will be in the following page
 *---------------------------------------------------------------*/

u16 *next_novram_address(void)
{
	u16 *search_address;

	for ( search_address = (u16 *)FLASH_NOVRAM_ADDRESS;
				(u32)search_address < (FLASH_NOVRAM_ADDRESS + FLASH_PAGE_SIZE);
				search_address += (sizeof(novram_def) / 2) )
	{
		if (*search_address == 0xFFFF)
			break;
	}

	return search_address;
}


u16 number_of_novram(void)
{
	u16 address;

	// Get the FLASH address of the next available slot for NOVRAM storage
	// Subtract the base address to get the address offset
	address = (u32)next_novram_address() - FLASH_NOVRAM_ADDRESS;

	// Divide by the size of a single NOVRAM save
	return address / sizeof(novram_def);
}


/*--------------------------------------------------------------
 * read_novram()
 *
 * Called at the beginning of each session to copy NOVRAM Health 
 *  Parameters from FLASH memory to RAM.
 * If the Health Parameters are not initialized (never written)
 *  initialize the RAM parameters.
 *-------------------------------------------------------------*/

void read_novram(void)
{
	u8 count;

	u16 *flash_address;
	u16 *ram_address;

	// Get the begin address of the last written Health data
	flash_address = next_novram_address() - (sizeof(novram_def) / 2);

	if (flash_address < (u16 *)FLASH_NOVRAM_ADDRESS)
	{
		// There are no health parameters stored in FLASH
		initialize_novram();
	}
	else
	{
		wait_for_buffer();

		fill_a_string("\n\rReading NOVRAM: 0x");
		fill_a_long((u32)flash_address);
		send_sio_buffer();

		// Get the begin address of the RAM structure to write
		ram_address = (u16 *)&ram_novram;

		for (count = 0; count < sizeof(novram_def) / 2; count++)
		{
			*ram_address = *flash_address;
			ram_address++;
			flash_address++;
		}
	}
}


void write_novram(void)
{
	u8 count;
	u16 *flash_address;
	u16 *ram_address;

	// Get the begin address of the next available Health data slot
	flash_address = next_novram_address();

	// Check to see if there is room
	if ( ((u32)flash_address + (sizeof(novram_def) / 2))
								>= (FLASH_NOVRAM_ADDRESS + FLASH_PAGE_SIZE) )
	{
		// There is not room for this update

		// Erase the entire page 
		erase_flash_page(FLASH_NOVRAM_ADDRESS);

		// ..and write it in the first slot
		flash_address = (u16 *)FLASH_NOVRAM_ADDRESS;
	}

	wait_for_buffer();

	fill_a_string("\n\rWriting NOVRAM 0x");
	fill_a_long((u32)flash_address);
	send_sio_buffer();

	// Get the begin address of the RAM structure to write
	ram_address = (u16 *)&ram_novram.on_time;

	// Copy the RAM structure to FLASH memory
	for (count = 0; count < sizeof(novram_def) / 2; count++)
	{
		write_flash_int(flash_address, *ram_address);
		flash_address++;
		ram_address++;
	}
}


/*-----------------------------------------------------------------
 * access_novram_log()
 *
 * Called from the main loop no more often than once every 10ms to 
 *  send another novram image out the RS232 port, if appropriate.
 * NOVRAM is emulated in FLASH memory by storing the RAM structure 
 *  on each power down.  We can see how the machine was used during 
 *  the most recent few sessions by displaying those individual 
 *  saved images.
 * This process is triggered by setting health_transmit_count to 
 *  non-zero.  Be careful to prevent it from being more than the 
 *  number of health images available.
 * The NOVRAM page is erased and started over when it becomes full 
 *  so the oldest image is in the lowest address.
 *----------------------------------------------------------------*/

void access_novram_log(void)
{
	novram_def *flash_novram;

	if (health_transmit_count)
	{
		if ( (DMA1_Channel4->CCR & 1) == 0 )
		{
			// DMA is not enabled so the serial buffer is available

			fill_a_string("\n\n\r	DII-PP TITAN machine health listing ");
			fill_a_number(health_transmit_count, 0);

			health_transmit_count--;

			flash_novram = (novram_def *)( (health_transmit_count
											* sizeof(novram_def))
											+ FLASH_NOVRAM_ADDRESS );

			fill_a_string("\n\rTime powered: ");
			fill_a_hours(flash_novram->on_time);

			fill_a_string("\n\rRun time: ");
			fill_a_hours(flash_novram->run_time);

			fill_a_string("\n\rPower on cycles: ");
			fill_a_decimal(flash_novram->power_cycles);

			fill_a_string("\n\rMinimum battery voltage: ");
			fill_a_number(flash_novram->min_battery, 1);

			fill_a_string(" degF\n\rMinimum board temperature: ");
			fill_a_decimal(flash_novram->min_temperature >> 4);

			fill_a_string(" degF\n\rMaximum board temperature: ");
			fill_a_decimal(flash_novram->max_temperature >> 4);

			fill_a_string("installed\n\rFLASH write errors: ");
			fill_a_decimal(flash_novram->error_count);

			fill_a_string("\n\rFLASH timeout errors: ");
			fill_a_decimal(flash_novram->timeout_count);

			send_sio_buffer();
		}
	}
}


/*-------------------------------------------------------------
 * update_health_timers()
 *
 * Called from the main loop no more often than once every 10ms 
 *  to update some timers if it is time.  We may be away longer 
 *  if the foreground is busy
 *------------------------------------------------------------*/

void update_health_timers(void)
{
	static u16 previous_time = 0,
				frac_on_time = 0;


	u16 delta_time;

	// Get the number of timer ticks since we were here last 
	delta_time = system_timer - previous_time;

	previous_time = system_timer;

	this_on_time += delta_time;
	frac_on_time += delta_time;

	while (frac_on_time > 10)
	{
		frac_on_time -= 10;
		ram_novram.on_time++;
	}
}


/*--------------------------------------------------------------------
 * write_identity()
 *
 * Copy the passed string to FLASH memory at the next available memory 
 *  slot in the IDENTITY page.
 * Return with no action if there is not enough room for this update
 * FLASH memory is written in halfwords (two bytes) and this processor 
 *  is running little endian.  The string is assembled in memory in 
 *  order such that reading it a byte at a time will be correct.
 * The first character is the identity type indicating whether it is 
 *  the machine description, controller description, or the engine 
 *  serial number.
 *-------------------------------------------------------------------*/

void write_identity(u8 id_type, s8 *id_string)
{
	u8 count;
	u16 *flash_address;
	u16 ram_data;

	// Search the Machine Identity FLASH page for the next available slot
	for ( flash_address = (u16 *)MACHINE_IDENTITY_ADDRESS;
				flash_address < (u16 *)(MACHINE_IDENTITY_ADDRESS + FLASH_PAGE_SIZE);
				flash_address += IDENTITY_LENGTH )
	{
		if (*flash_address == 0xFFFF)
		{
			// There is room for this update and flash_address is pointing to 
			//  the lowest available slot

			wait_for_buffer();

			fill_a_string("\n\rWriting Identity: 0x");
			fill_a_long( (u32)flash_address );
			send_sio_buffer();

			// Assemble the identity type and the first character of the string 
			//  into the first halfword
			// The character destined for the lowest address is in the lower half
			ram_data = id_type;
			ram_data |= (*id_string << 8);
			write_flash_int(flash_address++, ram_data);

			// Get the number of halfwords to move (IDENTITY_LENGTH is number of characters)
			// We have already moved one halfword (type is included in length)
			count = (IDENTITY_LENGTH / 2) - 1;

			// Copy the remaining characters to FLASH memory
			for ( ; count > 0; count--)
			{
				id_string++;
				ram_data = *id_string;

				if (*id_string != NULL)
				{
					id_string++;
					ram_data |= (*id_string << 8);
				}

				write_flash_int(flash_address++, ram_data);

				if (*id_string == NULL)
					break;
			}

			break;
		}
	}
}


/*--------------------------------------------------------------
 * fill_identity()
 *
 * Put all entries from the identity page into the serial buffer 
 *  ready to transmit.
 * Characters indicating identity type are added before each
 *  identity string.
 * They are reported in cronological order; the most recent is 
 *  last.
 *-------------------------------------------------------------*/

void fill_identity(void)
{
	u8 *address;

	for (address = (u8 *)MACHINE_IDENTITY_ADDRESS;
				address < (u8 *)(MACHINE_IDENTITY_ADDRESS + FLASH_PAGE_SIZE);
				address += IDENTITY_LENGTH)
	{
		if (*address == 0xFF)
		{
			// There are no more to fill
			break;
		}
		else
		{
			switch (*address)
			{
				case ID_MACHINE:
					fill_a_string("\n\rMachine: ");
					break;
				case ID_CONTROLLER:
					fill_a_string("\n\rController: ");
					break;
				case ID_ENGINE:
					fill_a_string("\n\rEngine SN: ");
					break;
			}

			// Increment past the type character
			address++;

			// Put the actual string in the serial buffer
			fill_a_string(address);
		}
	}
}


/*-----------------------------------------------------------------
 * get_recent_identity()
 *
 * Search the identity page and return the address to the beginning 
 *  of the most recent entry of the requested type
 *----------------------------------------------------------------*/

u8 *get_recent_identity(identity_type_def identity_type)
{
	u8 *address;

	// Search from the end backward
	for (address = (u8 *)((MACHINE_IDENTITY_ADDRESS + FLASH_PAGE_SIZE) - IDENTITY_LENGTH);
				address >= (u8 *)MACHINE_IDENTITY_ADDRESS;
				address -= IDENTITY_LENGTH)
	{
		if (*address == identity_type)
		{
			// address points to the beginning (type character) of 
			//  the requested string

			// Return the address of the string (skip the type character)
			return address + 1;
		}
	}

	return "NONE                ";
}


/*---------------------------------------------------------
 * get_number_identities()
 *
 * Find the address of the last string in the Identity page 
 *  and return the number of strings stored
 *--------------------------------------------------------*/

u8 get_number_identities(identity_type_def identity_type)
{
	u8 count = 0;
	u8 *address;

	// Search from the end backward
	for (address = (u8 *)MACHINE_IDENTITY_ADDRESS;
				address < (u8 *)(MACHINE_IDENTITY_ADDRESS + FLASH_PAGE_SIZE);
				address += IDENTITY_LENGTH)
	{
		if (*address == identity_type)
		{
			// address points to the beginning (type character) of 
			//  the requested string

			count++;
		}
		else if (*address == 0xFF)
			break;
	}

	return count;
}


/*----------------------------------------------------------/*-----------------------------------------------------------
 * fault_address()
 *
 * Return the address of the beginning of the requested fault
 *----------------------------------------------------------*/

u16 *fault_address(u8 fault_index)
{
	u16 address_offset = 0;

	// The index spans from 0 to NUMBER - 1 on the first page
	if (fault_index >= NUMBER_FAULTS_PER_PAGE)
	{
		// The requested address is in the second page

		fault_index -= NUMBER_FAULTS_PER_PAGE;

		// Set the address offset to the beginning of the second page
		address_offset = FLASH_PAGE_SIZE;
	}

	// Multiply the index by the number of bytes per fault 
	//  to get the address offset of the beginning of the requested fault
	address_offset += ((u16)fault_index * (NUMBER_INTS_PER_FAULT * 2));

	return (u16 *)(address_offset + FAULT_LOG_ADDRESS);
}


/*--------------------------------------------------------
 * number_of_faults()
 *
 * Return the number of faults currently in the fault log.
 *-------------------------------------------------------*/

u16 number_of_faults(void)
{
	u16 faults;

	// Get the index to the next available fault storage
	// It is the number available unless we have previously filled both 
	//  pages and are working on re-writing the first page
	// If we are re-writing the second page, it doesn't matter whether 
	//  we have wrapped since both pages would have been erased as needed
	faults = ram_novram.fault_log_index;

	// The number spans from 0 to NUMBER - 1 in the first page
	if (faults < NUMBER_FAULTS_PER_PAGE)
	{
		// We are presently filling the first page

		if ( *(u16 *)(FAULT_LOG_ADDRESS + FLASH_PAGE_SIZE) != 0xFFFF )
		{
			// The second page has something in it

			// Increment the number of faults available by 
			//  the number that fits in a page
			faults += NUMBER_FAULTS_PER_PAGE;
		}
	}

	return faults;
}


/*----------------------------------------------------------------
 * write_fault_log()
 *
 * Write a fault listing to the Fault Log FLASH memory page
 * FLASH memory is written in halfwords (two bytes)
 * When the fault log is modified, remember to update fill_fault()
 *  and set NUMBER_INTS_PER_FAULT.
 *---------------------------------------------------------------*/

void write_fault_log(void)
{
	u16 *address;
	u16 uint_data;

	// Get the address of the fault element to begin writing
	address = fault_address(ram_novram.fault_log_index);

	wait_for_buffer();

	fill_a_string("\n\rWriting Fault: 0x");
	fill_a_long( (u32)address );
	send_sio_buffer();

	write_flash_int(address++, ram_novram.power_cycles);			// 1

	// The character destined for the lowest address is in the lower half
	uint_data = 0x00;

	if (digital_input[DIN_POWER_SIGNAL] == SW_CLOSED)
		uint_data |= 0x0100;
	if (digital_input[DIN_MANUAL_REGEN] == SW_CLOSED)
		uint_data |= 0x0800;
	if (digital_input[DIN_FILTER] == SW_CLOSED)
		uint_data |= 0x1000;
	if (digital_input[DIN_REGEN_INHIBIT] == SW_CLOSED)
		uint_data |= 0x2000;
	if (digital_input[DIN_UNIT_START] == SW_CLOSED)
		uint_data |= 0x8000;
    if (digital_input[DIN_STARTNOW_SW] == SW_CLOSED)
		uint_data |= 0x9000;

	write_flash_int(address++, uint_data);							// 2

//	write_flash_int(address++, engine rpm);							// 3
//	write_flash_int(address++, regulation pressure / 10);			// 4
//	write_flash_int(address++, output pressure / 10);				// 5
//	write_flash_int(address++, discharge temperature >> 4);			// 6
//	write_flash_int(address++, sep tank temperature >> 4);			// 7
//	write_flash_int(address++, ThrottleToRPM());					// 8

	write_flash_int(address++, uint_data);							// 9

	write_flash_int(address++, 0xFF);								// 10

	/*-----------------------------------------
	 * Save three active faults from the engine
	 *----------------------------------------*/

	// SPN can be larger than an int
	// Put the upper bytes of the spn in the lower half of the second
	//  halfword and put the fmi in the upper half
//	write_flash_int(address++, engActiveList[0].spn);					// 20
//	uint_data = (engActiveList[0].spn >> 16) | ((u16)engActiveList[0].fmi << 8);
	write_flash_int(address++, uint_data);								// 21

//	write_flash_int(address++, engActiveList[1].spn);					// 22
//	uint_data = (engActiveList[1].spn >> 16) | ((u16)engActiveList[1].fmi << 8);
	write_flash_int(address++, uint_data);								// 23

//	write_flash_int(address++, engActiveList[2].spn);					// 24
//	uint_data = (engActiveList[2].spn >> 16) | ((u16)engActiveList[2].fmi << 8);
	write_flash_int(address, uint_data);								// 25

	/*---------------------------------------------------
	 * Increment the index for the next time it is needed
	 *--------------------------------------------------*/

	// If we have just written the last one to the second page...
	if ( ram_novram.fault_log_index >= ((2 * NUMBER_FAULTS_PER_PAGE) - 1) )
	{
		// The second page has just become full

		// Erase the first page
		erase_flash_page(FAULT_LOG_ADDRESS);
		
		// .. and start the index over
		ram_novram.fault_log_index = 0;
	}
	else
	{
		// Index spans from zero to number - 1
		if ( ram_novram.fault_log_index == (NUMBER_FAULTS_PER_PAGE - 1) )
		{
			// The first page has just become full

			// Erase the second page (even if not needed)
			erase_flash_page(FAULT_LOG_ADDRESS + FLASH_PAGE_SIZE);
		}

		ram_novram.fault_log_index++;
	}
}


void fill_a_status(u8 status)
{
	if (status == 0xFF)
		fill_a_string("NA  ");
	else
		fill_a_decimal(status);
}


void fill_engine_code(u16 *code_address)
{
	u32 temp_data;
	
	temp_data = *code_address;
	code_address += 1;
	temp_data |= ((*code_address & 0x07) << 16);

	fill_a_decimal(temp_data);

	fill_a_string("/");
	fill_a_decimal(*code_address >> 8);
}


void fill_a_bool(u8 boolean_state)
{
	if (boolean_state)
		fill_a_string("CLOSED");
	else
		fill_a_string("OPEN  ");
}


/*----------------------------------------------------------
 * fill_fault()
 *
 * Copy a fault listing from the Fault Log FLASH page to the 
 *  serial buffer, ready to transmit
 *---------------------------------------------------------*/
 
void fill_fault(u8 fault_index)
{
	s8 *address;

	// Get the address of the fault element to begin reading
	address = (u8 *)fault_address(fault_index);

	if (*address != 0xFF)
	{
		fill_a_string("\n\r\tPower ON count: ");
		fill_a_decimal(*(u16 *)address);

		address += 2;
		fill_a_string("\n\r\tProblem Code: ");
		fill_a_decimal(*address);

		if (transmit_fault_log)
		{
			address++;

			fill_a_string("\n\r\tE-Stop: ");
			fill_a_bool(*address & 0x40);
			fill_a_string("\t\t\t\tStart: ");
			fill_a_bool(*address & 0x80);

			fill_a_string("\n\r\tLow Fuel Alert: ");
			fill_a_bool(*address & 0x02);
			fill_a_string("\t\t\tLow Fuel Shutdown: ");
			fill_a_bool(*address & 0x04);

			fill_a_string("\n\r\tEngine Air Filter: ");
			fill_a_bool(*address & 0x10);
			fill_a_string("\t\t:Wakeup ");
			fill_a_bool(*address & 0x01);

			fill_a_string("\n\r\t:Manual Regen ");
			fill_a_bool(*address & 0x08);
			fill_a_string("\t\t\t:Regen Inhibit ");
			fill_a_bool(*address & 0x20);

			address++;
			fill_a_string("\n\r\tEngine RPM: ");
			fill_a_decimal(*(u16 *)address);

			address += 2;
			fill_a_string("\t\t\t\tRegulation Pressure: ");
			fill_a_number(*(u16 *)address, 2);

			address += 2;
			fill_a_string(" PSI\n\r\tOutput Pressure: ");
			fill_a_decimal(*(u16 *)address);

			address += 2;
			fill_a_string(" PSI\t\t\tDischarge Temperature: ");
			fill_a_decimal(*(u16 *)address);

			address += 2;
			fill_a_string(" PSI\n\r\tSeparation Tank Temperature: ");
			fill_a_decimal(*(u16 *)address);

			address += 2;
			fill_a_string(" DegF\tThrottle Position: ");
			fill_a_decimal(*(u16 *)address);

			address += 2;
			fill_a_string(" RPM\n\r\tMachine ID Code: ");
			fill_a_decimal(*(u8 *)address);

			address++;
			fill_a_string("\t\t\tAutoStart status: ");
			fill_a_status(*(u8 *)address);

			address++;
			fill_a_string("\n\r\tOTC status: ");
			fill_a_status(*(u8 *)address);

			address++;
			fill_a_string("\t\t\tIQ status: ");
			fill_a_status(*(u8 *)address);

			address++;
			fill_a_string("\n\r\tEngine Fault Codes SPN/FMI: ");
			fill_engine_code((u16 *)address);

			address += 4;
			fill_a_string(", ");
			fill_engine_code((u16 *)address);

			address += 4;
			fill_a_string(", ");
			fill_engine_code((u16 *)address);
		}
	}
}


/*----------------------------------------------------------------
 * access_fault_log()
 *
 * Called from the main loop no more often than once every 10ms to 
 *  send another fault out the RS232 port, if appropriate.
 * This process is triggered by setting fault_transmit_count to 
 *  non-zero.  Be careful to prevent it from being more than the 
 *  number of faults available.
 * If transmit_fault_log is TRUE, we send an entire fault listing, 
 *  most recent first.
 * If transmit_fault_log is FALSE, we only send the fault code.
 *---------------------------------------------------------------*/

void access_fault_log(void)
{
	static u8 next_fault,
			  fault_count;

	if (fault_transmit_count)
	{
		if ( (DMA1_Channel4->CCR & 1) == 0 )
		{
			// DMA is not enabled so the serial buffer is available

			// Decrement to the bottom of the first page
			if (next_fault)
				next_fault--;
			else
			{
				// .. and wrap to the top of the second page
				// Faults are counted from 0 to NUMBER - 1
				next_fault = (NUMBER_FAULTS_PER_PAGE * 2) - 1;
			}

			fault_count++;
			fill_a_string("\n\n\r	DII-PP TITAN fault listing ");
			fill_a_decimal(fault_count);

			if (fault_count == 1)
				fill_a_string(" - most recent");

			fill_fault(next_fault);

			send_sio_buffer();

			fault_transmit_count--;
		}
	}
	else
	{
		// Prepare to start the process over
		next_fault = ram_novram.fault_log_index;
		fault_count = 0;
	}
}


/*----------------------------------------------------------------
 * compare_serial_number()
 *
 * Compare two engine serial number strings up to the terminating 
 *  character.
 * Use the most recent engine serial number as one of the strings, 
 *  the other is at the passed address
 *---------------------------------------------------------------*/

bool compare_serial_number(u8 *questioned_string)
{
	u8 *present_sn;

	// Number of characters to compare is one less than the _LENGTH since 
	//  Identity Type is the first character and it is not compared
	u8 count = IDENTITY_LENGTH - 1;

	// Get the address of the most recent engine ECU SN string in FLASH memory
	present_sn = get_recent_identity(ID_ENGINE);

	while (count)
	{
		if (*questioned_string != *present_sn)
			return FALSE;
		else if (*questioned_string == NULL)
			break;
		else
		{
			questioned_string++;
			present_sn++;
			count--;
		}
	}

	return TRUE;
}


void erase_fault_log(void)
{
	// Erase both pages of fault log entries
	erase_flash_page(FAULT_LOG_ADDRESS);
	erase_flash_page(FAULT_LOG_ADDRESS + FLASH_PAGE_SIZE);
	ram_novram.fault_log_index = 0;
}


void initialize_health(void)
{
	ram_novram.power_cycles = 0;
	ram_novram.min_battery = 500;

	ram_novram.on_time = 0;
	ram_novram.run_time = 0;

	ram_novram.min_temperature = 0xF000;
	ram_novram.max_temperature = 0;

	erase_fault_log();

	// Erase the entire page of Identity strings
	// They include Controller descriptions, Machine descriptions and 
	//  Engine ECU serial numbers
	erase_flash_page(MACHINE_IDENTITY_ADDRESS);
}
#ifdef NEVER
wait_for_buffer();
fill_a_string("\n\rFL->CR: ");
fill_an_int(FLASH->CR);
fill_a_string("\n\raddr: ");
fill_a_long(flash_address);
send_sio_buffer();
wait_for_buffer();
#endif
