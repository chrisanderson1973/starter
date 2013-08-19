/****************************************************************
 * CanCommands.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * Controller Area Network (CAN) peripheral  
 *
 * 2/27/2013 CLA V0.01	Initial code taken from TitanShow
 ****************************************************************/

#include "stm32f10x.h"
#include "common.h"
#include "Timer.h"
#include "SerialBuffer.h"
#include "Analog.h"
#include "IOdata.h"
#include "E3prom.h"
#include "Flash.h"
#include "CanPackets.h"
#include "CanCommands.h"
#include "canbus.h"

u16 remote_dump_index;

can_id_def this_can_address;

reprog_server_def programming_server;

configuration asConfiguration;

/*-------------------------------------------------------------------------
 * select_transmitter()
 *
 * Check to see if one of the CAN transmitters is available and return its 
 *  address.
 * Return 0 if CAN is off-line or if there is not a transmitter available.
 * CAN transmitters are accessed from inside and outside a timer interrupt
 *  If the interrupt occurs after this routine is called and before 
 *  transmission is requested, the interrupted transmission will not occur.
 *  So we reserve one of the three transmit mailboxes for use by a timer 
 *  interrupt to deliver CAN throttle to the engine.
 *------------------------------------------------------------------------*/

CAN_TxMailBox_TypeDef *select_transmitter(void)
{
	if (can_on_line)
	{
		// Select an empty transmit mailbox
		if (CAN1->TSR & 0x04000000)
			return &(CAN1->sTxMailBox[0]);
		else if (CAN1->TSR & 0x08000000)
			return &(CAN1->sTxMailBox[1]);
		else if (CAN1->TSR & 0x10000000)
			return &(CAN1->sTxMailBox[2]);
	}

	return 0;
}


/*----------------------------------------------------------------
 * transmit_proprietary()
 *
 * Called by the foreground task to transmit a proprietary command 
 *---------------------------------------------------------------*/

bool transmit_proprietary(proprietary_command_def command, can_id_def address)
{
	CAN_TxMailBox_TypeDef *mailbox;

	u16 temp_uint;

	mailbox = select_transmitter();

	if (mailbox)
	{
		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
									| PF_PROP_A << 19
									| address << 11
									| this_can_address << 3;
		switch (command)
		{
#ifdef NEVER
			case EXAMPLE:

				// Data Length Code
				mailbox->TDTR = 3;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = ((uint32_t)Data[0])
									| ((uint32_t)Data[1] << 8)
									| ((uint32_t)Data[2] << 16)
									| ((uint32_t)Data[3] << 24);

				mailbox->TDHR = ((uint32_t)Data[4])
									| ((uint32_t)Data[5] << 8)
									| ((uint32_t)Data[6] << 16)
									| (((uint32_t)Data[7] << 24);
				break;
#endif
			case PCMD_HEARTBEAT:

				// Data Length Code
				mailbox->TDTR = 1;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_HEARTBEAT;

				break;

			case PCMD_RESET_NOVRAM:

				// Data Length Code
				mailbox->TDTR = 2;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_RESET_NOVRAM
									| ((uint32_t)MSG_REQUEST << 8);
 
				break;

			case PCMD_REPROGRAM:

				// We are requesting to reprogram another device via CAN
				// On receipt of this command, it gets the bootloader going
				//  which then sends us a CTS to begin the cycle
	
				// Data Length Code
				mailbox->TDTR = 3;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_REPROGRAM | (0x5AA5 << 8);

				break;

			case PCMD_PROGRAM_DATA:

				// Data Length Code
				mailbox->TDTR = 8;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_PROGRAM_DATA
									| (sio_buffer[0] << 8)
									| (sio_buffer[1] << 16)
									| (sio_buffer[2] << 24);

				mailbox->TDHR = (sio_buffer[3])
									| (sio_buffer[4] << 8)
									| (sio_buffer[5] << 16)
									| (sio_buffer[6] << 24);
				break;

			case PCMD_VERSION:

				// Data Length Code
				mailbox->TDTR = 8;

				temp_uint = SOFTWARE_VERSION;
				temp_uint |= 0x8000;				// BETA version

				// Transmit Mailbox Data Registers
				mailbox->TDLR = (u32)PCMD_VERSION
									| (u32)(temp_uint & 0xFF00)
									| ((u32)(temp_uint & 0x00FF) << 16)
									| ((u32)(ambient_temperature & 0xFF00) << 16);

				mailbox->TDHR = (u32)(ambient_temperature & 0x00FF)
									| (u32)(battery_voltage & 0xFF00)
									| ((u32)(battery_voltage & 0x00FF) << 16);
//									| ((u32)some byte << 24);

				break;

			case PCMD_QUESTION_SCREEN:

				// This is the request side of getting a CAN device to start 
				//  sending its '?' screen to us via CAN

				// Data Length Code
				mailbox->TDTR = 2;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_QUESTION_SCREEN | ( (u16)MSG_REQUEST << 8 );

				break;

  		 case PCMD_RS232_DATA:

				// We are sending data via CAN to be transferred to 
				//  RS232 by another module
    
				// sio_index points to next available element in sio_buffer[]
				// remote_dump_index points to the next one to send

                if (remote_dump_index < sio_index)
				{
					// Get the number of characters left to move
					temp_uint = sio_index - remote_dump_index;

					// We can only move 7 characters at a time
					if (temp_uint > 7)
						temp_uint = 7;

					// Data Length Code
					// Length is one greater since command takes a data byte
					mailbox->TDTR = temp_uint + 1;

					// Transmit Mailbox Data Registers
					// Copy 7 data bytes even though they may not be used
					mailbox->TDLR = PCMD_RS232_DATA
									| (sio_buffer[remote_dump_index] << 8)
									| (sio_buffer[remote_dump_index + 1] << 16)
									| (sio_buffer[remote_dump_index + 2] << 24);

					mailbox->TDHR = sio_buffer[remote_dump_index + 3]
									| (sio_buffer[remote_dump_index + 4] << 8)
									| (sio_buffer[remote_dump_index + 5] << 16)
									| (sio_buffer[remote_dump_index + 6] << 24);

					remote_dump_index += temp_uint;
        
				}
				else
                {
					// There are no more characters to send
					// End the session
              
					sio_index = 0;
					remote_access_state = ACCESS_IDLE;
                                
                    return FALSE;
                 
                }
                break;
				
			case PCMD_PING:

				// Data Length Code
				mailbox->TDTR = 2;

				// Transmit Mailbox Data Registers
				mailbox->TDLR = PCMD_PING | (MSG_REQUEST << 8);

				break;
            
            case PCMD_SET_TITAN_AUTOST_METHOD:          // switch closing, opening transducer (1 byte paylod)
            
                // Data Length Code
				mailbox->TDTR = 2;
            
                //Transmit pressure config back to Ciewport
                mailbox->TDLR = PCMD_SET_TITAN_AUTOST_METHOD | ((u16)(ram_novram.pressure_config << 8));
            
                break;
            case PCMD_SET_AUTOST_TIME:                  // cooldown time (2 byte paylod) 
            
                // Data Length Code
				mailbox->TDTR = 3;
            
                //Transmit cooldown time to Viewport
                mailbox->TDLR = PCMD_SET_AUTOST_TIME 
                                |(u32)(ram_novram.cooldown_time << 8);
            
                break;
            case PCMD_SET_AUTOST_THRESHOLD:       // pressure threshold (2 byte payload)
                 // Data Length Code
				mailbox->TDTR = 3;
            
                //Transmit cooldown time to Viewport
                mailbox->TDLR = PCMD_SET_AUTOST_THRESHOLD 
                                |(u32)(ram_novram.pressure_threshold << 8);
          
                break;
            default:

                return FALSE;
                break;
                
		}

		// Request transmit
		mailbox->TIR |= 1;
   
		return TRUE;
	}
	else
		return FALSE;
}


/*---------------------------------------------------------------------
 * can_start_reprogram()
 *
 * Called by the foreground serial service when prompted by a connected 
 *  test tool to begin reprogramming a connected CAN server.
 * Called with parameter 0xFF to program all connected IR controllers, 
 *  except this one.
 * Send the command to a CAN server to begin reprogramming that server 
 *  via CAN communication.
 *--------------------------------------------------------------------*/

void can_start_reprogram(can_id_def server_node)
{
	// Give the server time to finish processing any 
	//  previous message
	wait_for_tic(2);

	if ( transmit_proprietary(PCMD_REPROGRAM, server_node) )
	{
		programming_server.requestee = server_node;

		// Enable waiting for that server to respond with "all clear"
		programming_server.state = PRGM_WAIT_CTS;
	}
	else
		programming_server.state = PRGM_ERROR;	
}


/*-------------------------------------------------------------------
 * can_send_binary()
 *
 * This is the reprogramming function of a connected CAN server.
 * A device connected to our serial port is delivering an Intel .hex 
 *  file at 9600 baud.
 * This function is called continuously when in the programming state 
 *  to check for a new byte in the serial input stream.
 * The server being reprogrammed unravels the .hex and writes the 
 *  resulting binary data to its program space.
 * The server must be a little faster than we get bytes (9600 baud 
 *  ~ 1ms/byte) and so is waiting for the next packet before we get 
 *  enough for it from the tool.
 *------------------------------------------------------------------*/

void can_send_binary(void)
{
	static u8 search_index = 0;
	u8 binary_character;

	if ( get_serial_character(&binary_character) )
	{
		sio_buffer[sio_index++] = binary_character;

		/*---------------------------------------------------------------------
		* Find the End Of File record in order to set programming_server.state 
		*--------------------------------------------------------------------*/

		// Reset search_index at the beginning of each record	 
		if (binary_character == ':')
		{
			search_index = 0;
		
			// Send a progress character
			USART1->DR = '|';
		}

		// No need to worry about wrapping around since each record 
		//  is 32 bytes (64 characters) plus the record overhead
		search_index++;

		// The EOF record is identified by a '1' in the 9th nybble
		if ( (search_index == 9) && (binary_character == '1') )
		{
			// Set search_index larger than it can otherwise be 
			//  to indicate EOF
			search_index = 0xF0;
		}

		// Send a packet after 7 nybbles are accumulated or 
		//  when the EOF record is complete
		if ( (sio_index == 7) || (search_index == 0xF2) )
		{
			// Keep trying until a transmitter is available
			while ( transmit_proprietary(PCMD_PROGRAM_DATA, programming_server.requestee) == FALSE );

			sio_index = 0;

			// Wait for two checksum nybbles after the EOF record 
			//  is identified
			if (search_index == 0xF2) 
				programming_server.state = PRGM_WAIT_ACK;
		}
	}
}


/*------------------------------------------------------------
 * request_engine_hours()
 *
 * Called from the forground task no more often than every few 
 *  minutes to request engine hours.
 *-----------------------------------------------------------*/

bool request_engine_hours(void)
{
	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	if (mailbox)
	{
		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
									| PF_REQUEST << 19
									| CAN_ECU_ADDR << 11
									| this_can_address << 3;

		// Data Length Code
		mailbox->TDTR = 3;

		// Transmit Mailbox Data Registers
		mailbox->TDLR = PS_HOURS | ( (PF_PDU << 8) | (0 << 16) );

		// Request transmit
		mailbox->TIR |= 1;

		return TRUE;
	}
	else
		return FALSE;
}


/*-----------------------------------------------------------------------
 * dm1_message_action()
 *
 * An error message or messages from the engine has been assembled into 
 *  canMsgBuffer[].
 * These messages do not support destination addresses.
 * We get here on single and multiple packet DM1 messages.  Single packet 
 *  messages can deliver a single SPN.  Multiple packet DM1 messages are 
 *  more common since they deliver a list of SPNs.
 *----------------------------------------------------------------------*/

void dm1_message_action(void)
{
	u8 index,
		number;

	u32 temp_spn;

	/*----------------------------------------
	 * Generate a new engine active fault list
	 *---------------------------------------*/

	// Calculate the number of codes in this message
	number = canMsgBuffer[BUFF_LENGTH] - 2;
	// It takes four communication bytes to get one code
	number /= 4;

	// Skip over the warning lamp status
	can_buffer_index = BUFF_DATA_BEGIN + 2;
	
	for (index = 0; index < MAXDTCS; index++)
	{
		temp_spn = canMsgBuffer[can_buffer_index++];
		temp_spn |= (u16)canMsgBuffer[can_buffer_index++] << 8;
		temp_spn |= (u32)(canMsgBuffer[can_buffer_index] & 0xE0) << (16 - 5);

		if (temp_spn && (temp_spn < 0x7FFFF) && number)
		{
//			engActiveList[index].spn = temp_spn;
//			engActiveList[index].fmi = canMsgBuffer[can_buffer_index] & 0x1F;
		}
		else
		{
//			engActiveList[index].spn = 0;
//			engActiveList[index].fmi = 0;
		}

		number--;

		// Increment away from the last byte used and 
		//  skip the occurrance count byte
		can_buffer_index += 2;
	}

	dm1_timout_count = SECONDS(3);
}


bool request_engine_serial_num(void)
{
	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	if (mailbox)
	{
		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
									| PF_REQUEST << 19
									| CAN_ALL_ADDR << 11
									| this_can_address << 3;

		// Data Length Code
		mailbox->TDTR = 3;

		// Transmit Mailbox Data Registers
		mailbox->TDLR = PS_IDENTIFY	| ( (PF_PDU << 8) | (0 << 16) );

		// Request transmit
		mailbox->TIR |= 1;

		return TRUE;
	}
	else
		return FALSE;
}

/*---------------------------------------------------------------------
 * get_engine_serial_number()
 *
 * We request serial number, via CAN communication, from the connected 
 *  engine ECM when we first power up.  Once we get that response from 
 *  the engine ECM, we call this function to parse through the response 
 *  and extract the serial number.
 * That serial number is then added to our Identity if it is not the 
 *  last one.  A serial number may appear more than once.
 *--------------------------------------------------------------------*/
 
void get_engine_serial_number(void)
{
	u8 count = 0,
		 sn[IDENTITY_LENGTH - 1];

	u8 *can_data;

	/*------------------------------------------
	 * Search through the input string to get to 
	 *  the first character of the serial number
	 *-----------------------------------------*/

	can_data = &canMsgBuffer[BUFF_DATA_BEGIN];

	// Parse through the Engine make
	while (*can_data != '*')
		can_data++;

	// Increment past the delimiter
	can_data++;

	// Parse through the Engine model
	while (*can_data != '*')
		can_data++;

	// Increment past the delimiter
	can_data++;
	
	// Retrieve the Engine serial number
	while ( (*can_data != '*') && (count < (IDENTITY_LENGTH - 2)) )
	{
		sn[count++] = *can_data;

		can_data++;
	}

	// Put the string terminating character at the end
	sn[count] = NULL;

	/*-------------------------------------------------------------
	 * Update the Identity page with this serial number if this 
	 *  engine is not the most recent one we have been connected to
	 *------------------------------------------------------------*/

	wait_for_buffer();
	fill_a_string("\n\rFound ECM: ");
	fill_a_string(sn);
	send_sio_buffer();

	if ( compare_serial_number(sn) == FALSE )
	{
		// The last engine in the Identity list does not match 
		//  the one we're presently connected to

		write_identity(ID_ENGINE, sn);
	}
}

/*--------------------------------------------------------------------
 * timed_can_messages()
 *
 * Called no more often than once every 10ms from the foreground task 
 *  to transmit various CAN packets at different times
 * If a packet is due for transmission and there is a CAN transmitter 
 *  available, load it and request transmit.  If no transmitter is 
 *  available, return without resetting the timer and that packet will 
 *  be tried again on the next pass.
 *-------------------------------------------------------------------*/

void timed_can_messages(void)
{
	/*static can_keypad_led_def previous_key_led;
	static bool keypad_initialized = FALSE;

    if (message_delay[MSGIX_FAN_CTRL])
				message_delay[MSGIX_FAN_CTRL]--;
    else
    {
        // Send Discharge Temperature to the ECU

        if ( transmit_fan_control() )
            message_delay[MSGIX_FAN_CTRL] = (SECONDS(1) / 2) - 1;
    }*/

}

