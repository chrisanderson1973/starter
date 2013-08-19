/****************************************************************
 * Flash.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors 
 *  
 * FLASH memory interface functions
 *
 * 2/27/2013 CLA V0.01	Initial code taken from TitanShow
 ****************************************************************/

#include "stm32f10x.h"
#include "common.h"
#include "Timer.h"
#include "E3prom.h"
#include "SerialBuffer.h"
#include "Flash.h"

#define RDP_KEY             0x00A5
#define FLASH_KEY1			0x45670123
#define FLASH_KEY2			0xCDEF89AB

bool wait_for_flash(void)
{
	u32 timeout = 0;

	if (FLASH->SR & 0x0014)
	{
		// Write protect or un-erased FLASH encountered on a previous write

		// Reset both status bits
		FLASH->SR = 0x0014;

		ram_novram.error_count++;

		return FALSE;
	}

	// Wait until a previous FLASH operation is compelete without errors
	while( (FLASH->SR & 0x0001) && (timeout < 0xFFFFFF) )
		timeout++;

	if (timeout < 0xFFFFFF)
		return TRUE;
	else
	{
		ram_novram.timeout_count++;
		return FALSE;
	}
}


/*----------------------------------------------------------
 * write_flash_int()
 *
 * Write a halfword to a Flash memory address
 * The FPEC is unlocked
 * The FLASH page to write has had write protection disabled
 *---------------------------------------------------------*/

void write_flash_int(u16 *address, u16 int_data)
{
	if ( wait_for_flash() )
	{
		// Set the Program bit in the FLASH Program and Erase Controller
		FLASH->CR = (u32)0x01;

		// Write the halfword to the destination address
		*address = int_data;

		wait_for_flash();
	}

	FLASH->CR = 0;		
}


/*----------------------------
 * erase_flash_page()
 *
 * Erase a specific FLASH page
 *---------------------------*/

bool erase_flash_page(u32 address)
{
	// Wait for a previous transmit to complete so we can use sio_buffer[]
	wait_for_buffer();

	fill_a_string("\r\nErasing FLASH 0x");
	fill_a_long(address);
	send_sio_buffer();

	// Wait for FLASH to be available
	if ( wait_for_flash() )
	{
		// Set the Page Erase enable bit
		FLASH->CR = 0x02;

		// Set the address of the page to erase
		FLASH->AR = address; 

		// Begin erasing the page
		FLASH->CR = 0x42;

		if ( wait_for_flash() )
        {
			// Reset the Page Erase enable bit
			FLASH->CR = 0;

			return TRUE;
		}
	}

	// Reset the Page Erase enable bit
	FLASH->CR = 0;

	return FALSE;
}


/*-------------------------------------------------------------
 * unlock_FPEC()
 *
 * Called in preparation of erasing or programming FLASH memory
 *------------------------------------------------------------*/

void unlock_FPEC(void)
{
	if (FLASH->CR & 0x0080)
	{
		// Unlock the Flash Program Erase controller
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}

	// Clear the FLASH’s error flags
	FLASH->SR = 0x0034;
}


/*-----------------------------------------------------------------------
 * set_option_bytes()
 *
 * A bit pattern indicating valid program (this application) is stored
 *  in user data bytes of the Option Bytes by this application when it
 *  is first executed.  It is used by the BootLoader to tell whether 
 *  to execute or program ROM after a reset.
 * We set that program valid indication to something other than "valid" 
 *  when we want to reprogram this application so the BootLoader will 
 *  initiate reprogramming.  We use the option bytes to pass the CAN
 *  address of a reprogramming master and the CAN address we assume 
 *  during reprogramming.  That means there is a combination of CAN 
 *  addresses that is the same as the "valid" indication value that 
 *  will not work.  If that occurs, set the master to 0xFF.
 * Erase and reprogram the Option bytes.  This also disables program 
 *  space write protection.
 *----------------------------------------------------------------------*/

void set_option_bytes(u16 option_data)
{
	// Wait for a previous operation to complete
	if ( wait_for_flash() )
	{
		// Enable option block programming
		FLASH->OPTKEYR = FLASH_KEY1;
		FLASH->OPTKEYR = FLASH_KEY2;
    
		// Set the Option Byte erase bit
        // Keep the Option Write Enable bit set
		FLASH->CR = 0x0220;

		// Start the Option Byte erasure
		FLASH->CR = 0x0260;

		// Wait for the option bytes to be erased
		if ( wait_for_flash() )
		{
			// Reset the Option Byte Erase bit
			//  and set the Option Byte Program bit
			// Keep the Option Byte Write Enable bit
			FLASH->CR = 0x0210;

			// Write the Data Byte 0 value
			OB->Data0 = option_data;

			// Wait for the Data Byte 0 write to complete
			if ( wait_for_flash() )
			{
				// Write the Data Byte 1 value
				OB->Data1 = option_data >> 8;

				// Wait for the Data Byte 1 write to complete
				if ( wait_for_flash() )
				{
					// Set program space write protection
					// Each bit protects 4k bytes of FLASH memory
					//  WRP0 protects 0x08000000-0x08007FFF
					//  WRP1 protects 0x08008000-0x0800FFFF
					//  WRP2 protects 0x08010000-0x08017FFF
					//  7 LS bits of WRP3 protects 0x08018000-0x0801EFFF
					//  MS bit of WRP protects 0x0801F000-0x0807FFFF
					// Write a bit to 0 to protect the corresponding pair of pages
					OB->WRP0 = 0x00;

					// Wait for the Write Protect Byte 0 write to complete
					if ( wait_for_flash() )
					{
						// Set program space write protection for 0x8000 - 0xFFFF
						OB->WRP1 = 0x00;

						// Wait for the Write Protect Byte 1 write to complete
						if ( wait_for_flash() )
						{
							// Program the Read Protection Option byte to 
							//  something other than RDP_KEY to keep read protection 
							//  active and prevent it from being deactivated without 
							//  first erasing the Option Bytes
							OB->RDP = 0;

							// Wait for the Read Protection byte to be programmed
							wait_for_flash();

							// Reset the Option Byte Program bit and 
							//  the Option Byte Write Enable bit
							FLASH->CR = 0;

							return;
						}
					}
				}
			}
		}
	}

	// Reset the Option Byte Program bit and the Option Byte Write Enable bit
	FLASH->CR = 0;

	send_a_string("\n\rProblem writing option bytes!");
}