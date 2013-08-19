/*******************************************************************
 * canbus.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * Controller Area Network (CAN) peripheral hardware setup routines 
 *  and specific functions for portable compressors
 *
 * 2/27/2013 CLA V0.01	Initial code taken from TitanShow
 *******************************************************************/

#include "stm32f10x.h"
#include "define.h"
#include "Interrupt.h"
#include "Timer.h"
#include "IOdata.h"
#include "E3prom.h"
#include "SerialBuffer.h"
#include "Serial.h"
#include "CanCommands.h"
#include "CanPackets.h"
#include "canbus.h"
#include "AStuff.h"

u8 controllerMode;    
u8 contStatus;

bool update_delays = FALSE;
bool can_on_line = TRUE;

// Count down counter to transmit more than one packet per CTS reception 
u8 packets_to_transmit = 0;
u16 can_comm_watchdog;
u16 master_comm_wd = 0;
u16 dm1_timout_count;
u32 engine_hours;

u16 pressure_range_input;

// Define an array to hold the CAN addresses of other modules 
// that support remote screen dump
#define MAXIMUM_REMOTE_LISTS	5
can_id_def remote_lists[MAXIMUM_REMOTE_LISTS];
u16 remote_list_index = 0;

// CAN node address to send data to its RS232 port
can_id_def dump_to_address;

// State counter to access remote screens
remote_access_def remote_access_state = ACCESS_IDLE;

// Reprogram state
reprogram_state_def reprogram_state = NONE;

void initialize_can(void)
{
	u16 poll_timer;

	// Reset the Low Speed APB (APB1) peripheral CAN1
    //    RCC->APB1RSTR |= 0x02000000;
    //    RCC->APB1RSTR &= ~0x02000000;

	// Make sure CAN is not in sleep mode
	// Leave CAN frozen in DEBUG mode, no reset, disable time triggered communication,
	// automatic bus-off management, no automatic wakeup, continuous retry attempts,
	//  receive FIFO not locked (loses oldest on overrun), priority driven by message ID,
	//  no sleep request, request initialization mode
	CAN1->MCR = 0x00010041;

	// Wait for initialization acknowledge
	poll_timer = 0;
	while ( (CAN1->MSR & 1) == 0 )
	{
		if (poll_timer < 0xFFFF)
			poll_timer++;
		else
		{
			send_a_string("\n\rCAN init error!");
			return;
		}
	}

    // Set the Bit Timing register for 250k Baud
	// PCLK1 = 32MHz so prescaler = 8 gives 4MHz CAN clock: Tq = 0.25us
	//  250k Baud is 4us/bit = 16 Tq 
	// Not Silent mode, disable Loop Back mode
	// SJW = 1, T2 = 4, T1 = 11, Prescaler = 8
	CAN1->BTR = 0x003A0007;

    // Leave initialization mode
	CAN1->MCR &= ~1;

	// Wait for the acknowledge
	poll_timer = 0;
	
    while (CAN1->MSR & 1)
	{
		if (poll_timer < 0xFFFF)
			poll_timer++;
		else
		{
			send_a_string("\n\rCAN init mode error!");
			return;
		}
	}

	/*---------------------------
	 * Initialize the CAN filters
	 *--------------------------*/

	// Set filter initialization mode
	CAN1->FMR |= 1;

	// Make sure all filters are deactivated
	CAN1->FA1R = 0;

	// Set all filters for single 32-bit scale configuration
	// This high density device only has 14 32bit filters
	CAN1->FS1R = 0x3FFF;

    // Set all filter banks for Identifier Mask mode
	// If any become Identifier List mode, they take lower indexes 
	//  which would shift existing indexes
    CAN1->FM1R = 0;
	
	/*----------------------------------------------------------------
	 * All filters are 32 bit mask and match
	 * Filter 0 connected to FIFO 0 receives PGN from 0xFEE0 to 0xFEEF
	 * Filter 1 connected to FIFO 1 receives PGN from 0xFEF0 to 0xFEFF
	 * Filter 2 connected to FIFO 0 receives PGN from 0xF000 to 0xF007
	 * Filter 3 connected to FIFO 0 receives PGN 0xFECA (DM1 messages) 
	 * Filter 4 connected to FIFO 1 receives messages having 
	 *  PFs from 0xE8 to 0xEF directed to all nodes
	 * Filter 5 connected to FIFO 1 receives messages having 
	 *  PFs from 0xE8 to 0xEF directed to this node
	 * Filter 6 connected to FIFO 0 receives PGN 0xFEDF (EEC3 messages)
	 *---------------------------------------------------------------*/

	// Assign filters 0, 2, 3, 6, 8, 10 and 12 to FIFO 0
	// Assign filters 1, 4, 5, 7, 9, 11 and 13 to FIFO 1
	CAN1->FFA1R = 0x00002AB2;		// .. 0010 1010 1011 0010

	/*------------------------------------------------------------------------
	 * Identifier register layout:
	 * PRIORITY(3) << 29 | PF << 19 | PS << 11 | SA << 3 | IDE << 2 | RTR << 1
	 *
	 * All filter Match registers set:
	 * Data Page = 0, Extended ID = 1 and ReTransmit Request = 0
	 * All filter Mask registers require them to match and do not require
	 *  Priority, Reserved bit or Source Address to match
	 *-----------------------------------------------------------------------*/

	// Filter Bank 0 is Index 0 from FIFO 0
	CAN1->sFilterRegister[0].FR1 = 0x04 | PF_PDU << 19 | 0xE0 << 11;
	// Match PF and upper nybble of PS
    CAN1->sFilterRegister[0].FR2 = 0x08000006 | 0xFF << 19 | 0xF0 << 11;

	// Filter Bank 1 is Index 0 from FIFO 1
	CAN1->sFilterRegister[1].FR1 = 0x04 | PF_PDU << 19 | 0xF0 << 11;
	// Match PF and upper nybble of PS
    CAN1->sFilterRegister[1].FR2 = 0x08000006 | 0xFF << 19 | 0xF0 << 11;

	// Filter Bank 2 is Index 1 from FIFO 0
	CAN1->sFilterRegister[2].FR1 = 0x04 | PF_EEC << 19 | 0x00 << 11;
	// Match PF and upper 5 bits of PS
	CAN1->sFilterRegister[2].FR2 = 0x08000006 | 0xFF << 19 | 0xF8 << 11;

	// Filter Bank 3 is Index 2 from FIFO 0
	CAN1->sFilterRegister[3].FR1 = 0x04 | PF_PDU << 19 | PS_DM1 << 11;
	// Match entire PDU
    CAN1->sFilterRegister[3].FR2 = 0x08000006 | 0xFF << 19 | 0xFF << 11;

	// Filter Bank 4 is Index 1 from FIFO 1
	CAN1->sFilterRegister[4].FR1 = 0x04 | 0xE8 << 19 | CAN_ALL_ADDR << 11;
	// Match upper 5 bits of PF and all of destination address
    CAN1->sFilterRegister[4].FR2 = 0x08000006 | 0xF8 << 19 | 0xFF << 11;

	// Filter Bank 5 is Index 2 from FIFO 1
	CAN1->sFilterRegister[5].FR1 = 0x04 | 0xE8 << 19 | this_can_address << 11;
	// Match upper 5 bits of PF and all of destination address
    CAN1->sFilterRegister[5].FR2 = 0x08000006 | 0xF8 << 19 | 0xFF << 11;

	// Filter Bank 6 is Index 3 from FIFO 0
	CAN1->sFilterRegister[6].FR1 = 0x04 | PF_PDU << 19 | PS_EEC3 << 11;
	// Match entire PDU
    CAN1->sFilterRegister[6].FR2 = 0x08000006 | 0xFF << 19 | 0xFF << 11;

	// Activate filters 0 - 6
	// Leave the others deactivated
	CAN1->FA1R = 0x0000007F;

	// Leave filter initialization mode
	CAN1->FMR &= ~1;

	// Enable CAN FIFO 0 and FIFO 1 message pending interrupts
	CAN1->IER = 0x0012;

	// Configure the NVIC CAN1 FIFO0 receive interrupt
	enable_interrupt(USB_LP_CAN1_RX0_IRQn, 1);

	// Configure the NVIC CAN1 FIFO1 receive interrupt
	enable_interrupt(CAN1_RX1_IRQn, 1);

	can_buffer_state = BUFFER_IDLE;

	// Initialize the communication watchdog timer
	can_comm_watchdog = SECONDS(10);
}

void fill_software_identity(void)
{
	canMsgBuffer[BUFF_FORMAT] = PF_PROP_A;
 	canMsgBuffer[BUFF_ADDRESS] = transmit_identity;	
	canMsgBuffer[BUFF_COMMAND] = PCMD_GET_ID;
	can_buffer_index = BUFF_DATA_BEGIN;

	canMsgBuffer[BUFF_DATA_BEGIN] = (u8)SOFTWARE_PN;
	canMsgBuffer[BUFF_DATA_BEGIN+1] = (u8)(SOFTWARE_PN >> 8);
    canMsgBuffer[BUFF_DATA_BEGIN+2] = (u8)(SOFTWARE_PN >> 16);
    canMsgBuffer[BUFF_DATA_BEGIN+3] = (u8)(SOFTWARE_PN >> 24);
    
    canMsgBuffer[BUFF_DATA_BEGIN+4] = SOFTWARE_VERSION;

	canMsgBuffer[BUFF_LENGTH] = 6;
	canMsgBuffer[BUFF_SEQUENCE] = 0;
}

void fill_master_timing(void)
{
	canMsgBuffer[BUFF_FORMAT] = PF_PROP_A;
 	canMsgBuffer[BUFF_ADDRESS] = CAN_MASTER_ADDR;
	canMsgBuffer[BUFF_COMMAND] = PCMD_DELAYS;

    canMsgBuffer[BUFF_DATA_BEGIN] = ram_novram.number_cranks;

	// Times are stored in seconds

	canMsgBuffer[BUFF_DATA_BEGIN + 1] = ram_novram.warn_time;
	canMsgBuffer[BUFF_DATA_BEGIN + 2] = ram_novram.warn_time >> 8;		// High byte

	canMsgBuffer[BUFF_DATA_BEGIN + 3] = ram_novram.cooldown_time;
	canMsgBuffer[BUFF_DATA_BEGIN + 4] = ram_novram.cooldown_time >> 8;		// High byte

	canMsgBuffer[BUFF_DATA_BEGIN + 5] = ram_novram.recrank_delay;
	canMsgBuffer[BUFF_DATA_BEGIN + 6] = ram_novram.recrank_delay >> 8;		// High byte

	can_buffer_index = BUFF_DATA_BEGIN + 7;

	canMsgBuffer[BUFF_LENGTH] = 7;
	canMsgBuffer[BUFF_SEQUENCE] = 0;
}

/*----------------------------------------------------
 * setup_command()
 *
 * Setup the CAN message buffer for a command to the 
 *  main compressor controller
 * Assemble our status along with the command into the 
 *  CAN message buffer
 *---------------------------------------------------*/

void setup_command(master_command_def command)
{
	canMsgBuffer[BUFF_FORMAT] = PF_PROP_A;
	canMsgBuffer[BUFF_ADDRESS] = CAN_MASTER_ADDR;
	
	canMsgBuffer[BUFF_COMMAND] = PCMD_STATUS;

	canMsgBuffer[BUFF_DATA_BEGIN] = auto_enable_sw;			// Auto/Manual switch; TRUE if Auto
	canMsgBuffer[BUFF_DATA_BEGIN + 1] = startnow_sw;	// Auto initiate switch; TRUE when CLOSED
	
	canMsgBuffer[BUFF_DATA_BEGIN + 2] = command;	// Command

	canMsgBuffer[BUFF_DATA_BEGIN + 3] = 0;			// Unused - remote P low byte
	canMsgBuffer[BUFF_DATA_BEGIN + 4] = 0;			// Unused - remote P high byte

	if (asState == AS_IDLE)
	{
		// Instruct the compressor to consider loss of CAN 
		//  communication to be an alert, not a shutdown            
		canMsgBuffer[BUFF_DATA_BEGIN + 5] = ACTION_ALERT;
	}
	else
	{
		// Set the action to take if communication is lost
	    canMsgBuffer[BUFF_DATA_BEGIN + 5] = ram_novram.comm_loss;
	} 

	canMsgBuffer[BUFF_DATA_BEGIN + 6] = asState;
	can_buffer_index = BUFF_DATA_BEGIN + 7;
	canMsgBuffer[BUFF_LENGTH] = 7;
	canMsgBuffer[BUFF_SEQUENCE] = 0;
}

/*--------------------------------------------------------------
 * can_filter_0_action()
 *
 * Called from FIFO 0 receive interrupt to process CAN receive 
 *  filter 0 messages: PDU = 0xFEE0 - 0xFEEF.
 * These messages do not support destination addresses.
 * Get data bytes from CAN Receive FIFO 0 Mailbox data registers 
 *  and update the appropriate variables as defined by the 
 *  PDU-specific field.
 * The page register is already set to CAN 0 FIFO 0 registers 
 *  page
 *-------------------------------------------------------------*/

void can_filter_0_action(CAN_FIFOMailBox_TypeDef *mailbox)
{
	switch ( (u8)(mailbox->RIR >> 11) )
	{
 		case PS_HOURS:			// 0xFEE5
			// Engine hours is delivered as 0.05 hours/bit
			//  and stored as 0.1 hr/bit
			engine_hours = (mailbox->RDLR + 1) >> 1;
			break;

		case PS_TIMEDATE:		// 0xFEE6
			break;

		case PS_IDENTIFY:		// 0xFEEB
			// This is a single packet message with some serial number
			// We presently don't support any
			break;

		case PS_ENG_TEMP:		// 0xFEEE

			// Coolant temperature delivered in degrees C + 40
			// Store DegC x 16
            // coolant temp = ((mailbox->RDLR & 0x00FF) - 40) << 4;

			// Engine oil temperature delivered in (degrees C + 273) * 32
			// Store DegC x 16
            // oil temp = ((mailbox->RDLR & 0xFFFF0000) >> 17) - (273 * 16);

			break;

		case PS_FLUIDS:		// 0xFEEF

			// Pressure are delivered as 4 kPa per bit
			// and stored in kPa x 16
            // oil pressure = (mailbox->RDLR & 0xFF000000) >> 18;
            // fuel pressure = (mailbox->RDLR & 0x00FF) * 4;

			break;
	}
}


/*----------------------------------------------------------------------
 * can_message_action()
 *
 * Called by the foreground after receiving a CAN message and assembling
 *  it into canMsgBuffer[].
 * Parse through it and take appropriate action.
 * Proprietary A messages (0xEFxx) can deliver only 7 data bytes in a 
 *  single packet since command occupies #0 data. 
 * The first data byte (from data register #1), not command, is in 
 *  canMsgBuffer[BUFF_DATA_BEGIN]
 * If this message requires a response, assemble it into canMsgBuffer[] 
 *  and return TRUE.  The transmission is handled by the foreground.
 * Return FALSE if no response is desired.
 *---------------------------------------------------------------------*/

bool can_message_action(void)
{

	switch (canMsgBuffer[BUFF_FORMAT])
	{
		
            case PF_PROP_A:

			/*--------------------------------
			 * The received packet is a 
			 *  Proprietary A message (0xEFxx)
			 *-------------------------------*/

			switch (canMsgBuffer[BUFF_COMMAND])
			{
				case PCMD_PING:
                    
					if (canMsgBuffer[BUFF_DATA_BEGIN] == MSG_REQUEST)
					{
						// We just got a PING request
						// Send the response PING back to the sender
						// The PDU format byte, destination address and 
						//  proprietary command are already in canMsgBuffer[]

						canMsgBuffer[BUFF_DATA_BEGIN] = MSG_RESPONSE;

						return TRUE;
					}
					else
					{
						// We just received the response to our request for 
						//  the number of other controllers that have ? screens

						// Add it to the array of device addresses if it fits
						if ( remote_list_index < (MAXIMUM_REMOTE_LISTS - 1) )
						{
							remote_lists[remote_list_index] = canMsgBuffer[BUFF_ADDRESS];
							remote_list_index++;
						}

						// We are waiting for all CAN devices to respond
						// This is a response, do not respond!
					}

					break;
				

				case PCMD_HEARTBEAT:

					// While programming via CAN, receipt of HEARTBEAT signals success
					if (programming_server.state == PRGM_WAIT_ACK)
						programming_server.state = PRGM_SUCCESS;

					break;

				case PCMD_RESET_NOVRAM:

					// We could be either controller

					if (canMsgBuffer[BUFF_DATA_BEGIN] == MSG_REQUEST)
					{
						// Another controller is asking us to reset our NOVRAM 
						// Do it and return a response

						initialize_novram();

						canMsgBuffer[BUFF_DATA_BEGIN] = MSG_RESPONSE;

						// Return the same command back as an acknowledge
						return TRUE;
					}
					else
					{
						// The other controller is responding after 
						//  initializing it's NOVRAM

						send_a_string("Remote NOVRAM initialized\n\r");
					}

					break;

				case PCMD_QUESTION_SCREEN:

					// We could be either controller

                    // Another controller is asking us to send
                    //  our '?' screen across CAN for display

                    // Keep the address of the requester
                    dump_to_address = canMsgBuffer[BUFF_ADDRESS];

                    // Put the data into the RS232 transmit buffer 
                    //  then send it via CAN to this sender
                 
                    // Start sending CAN packets
                    remote_access_state = ACCESS_TRANSMIT;

                    // .. from sio_buffer[] beginning at 0
                    remote_dump_index = 0;
              
					break;

			}

			break;

		case PF_PDU:

			switch (canMsgBuffer[BUFF_COMMAND])
			{
				case PS_DM1:

					// We have just received an error message from the engine

					// Receive DM1 messages, PDU = 0xFECA
					dm1_message_action();

					break;

				case PS_IDENTIFY:

					// We have just received a response to our request 
					//  for serial number from the engine ECM

					// Extract the serial number
					// Update the list of engines connected to
					get_engine_serial_number();

					break;

				default:
					break;
			}

			break;
	}

	return FALSE;
}


/*----------------------------------------------------------------------
 * access_info_screens()
 *
 * Called no more often than once every 10ms to display one of several 
 *  information screens from this from another CAN controller.
 * When '?' is received on the RS232 channel, we set remote_access_state 
 *  to _BEGIN.  Then, when we get here, we PING all controllers and 
 *  accumulate their CAN addresses.  Every device that responds to the 
 *  PING will be sent a "question request" message.  That device then 
 *  sends its '?' screen and ends its participation with a "question 
 *  response" message.
 * If not displaying '?' screens, we call functions that send our fault 
 *  log or machine health data out the RS232 port when triggered.  These 
 *  may take multiple passes to transmit since they all do not fit in 
 *  sio_buffer[] at once.
 *---------------------------------------------------------------------*/

void access_info_screens(void)
{
	switch (remote_access_state)
	{
		case ACCESS_IDLE:

			/*--------------------------------------------------------------
			 * Check to see if it is time to send another screen and send it
			 *-------------------------------------------------------------*/

			access_novram_log();

			access_fault_log();

			break;

		case ACCESS_TRANSMIT:

			// We are sending data via CAN 
			//  to another module to transfer to its RS232
            
			// Move the next seven characters
       
			transmit_proprietary(PCMD_RS232_DATA, dump_to_address);
   
			break;
	}
}


/*----------------------------------------------------------------------
 * maintain_can_comm_wd()
 *
 * Called no more than once every 10ms from the foreground task to 
 *  maintain the watchdog timer for CAN packets that use canMsgBuffer[].
 * A stuck transmission could happen if the CAN bus goes off-line.  
 * A stuck reception could happen if the transmitting module has a 
 *  problem
 * Also maintains the ECU watchdog and reflect that state on the yellow 
 *  led.  The ECU watchdog count is reset on receipt of a packet from 
 *  the engine ECU.
 *--------------------------------------------------------------------*/
 
void maintain_can_comm_wd(void)
{
	// Timeout a message if it does not complete in 1/2 second
	if (can_comm_watchdog)
    {
		can_comm_watchdog--;
   
    }
    else
	{
		// We seem to be stuck
		// Kill a possible in-process and free up canMsgBuffer[]

		can_buffer_state = BUFFER_IDLE;
	}

	if(master_comm_wd)
    {
		master_comm_wd--;
        middle_yellow_led = LED_NEG_BLINK;
    }
	else
    {
        middle_yellow_led = LED_OFF;    
    }
    
}


/*-----------------------------------------------------------------------
 * check_can()
 *
 * Called from the main loop to maintain the can "on-line" flag, to take 
 *  action on received messages and to manage the transmission of an 
 *  outgoing message.
 * Check to see if a CTS is required to continue an incoming message, and 
 *  if so, send it and reset the trigger.
 * Check to see if a message is ready and if so, parse it and prepare any 
 *  response
 *----------------------------------------------------------------------*/
 
void check_can(void)
{
	static u8 indicator_delay;

	// CAN Error Status Register
	// Check the bus off flag
	if (CAN1->ESR & 0x04)
	{
		indicator_delay = system_timer + 100;

		send_a_string("\n\r CAN error! ");

		can_on_line = FALSE;
	}

	// We delay a little after can_on_line is FALSE before allowing 
	//  it to be TRUE because an unterminated line is erratic
	else if ( (u8)system_timer == indicator_delay )
		can_on_line = TRUE;	

	switch (can_buffer_state)
	{
		case BUFFER_SEND_CTS:

			// Send a CTS packet to the sender of a multiple packet 
			//  message to prompt it to transmit another one to us
			if ( continue_mp_input() )
			{
				// Reset the communication watchdog timer
				can_comm_watchdog = SECONDS(10);

				can_buffer_state = BUFFER_ACQUIRING;
			}

			break;

		case BUFFER_SEND_ACK:

			// Send the End of Message Acknowledgement to the sender 
			//  of the multiple packet message we have just received
			if ( end_mp_input() )
				can_buffer_state = BUFFER_READY_TO_READ;

			break;
			
		case BUFFER_READY_TO_READ:

			// Parse the message in canMsgBuffer[]
			if ( can_message_action() )
			{
				canMsgBuffer[BUFF_SEQUENCE] = 0;

				can_buffer_state = BUFFER_READY_TO_SEND;
			}
			else
				can_buffer_state = BUFFER_IDLE;

			break;

		case BUFFER_READY_TO_SEND:

			// Send the message in canMsgBuffer[]
			// This will change the buffer state if successful
			transmit_msg_buffer();

			break;
			
		case BUFFER_TRANSMITTING:

			// Send another packet of a multiple packet message	
			if (packets_to_transmit)
			{
				// Reset the communication watchdog timer
				can_comm_watchdog = SECONDS(10);

				if ( continue_mp_output() )
				{
					// A packet transmission was successful

					// Increment the sequence number to the next packet to send
					canMsgBuffer[BUFF_SEQUENCE]++;

					// Decrement the number of packets left to send
					packets_to_transmit--;
				}
			}

			break;
        
        case BUFFER_IDLE:
            
            if (update_delays)
			{
				// Reserve canMsgBuffer[] ASAP
				can_buffer_state = BUFFER_READY_TO_SEND;
	
				// The master's timing parameters need to be set
			    fill_master_timing();
				update_delays = FALSE;
			}
			else if (transmit_identity)
			{
				// Reserve canMsgBuffer[] ASAP
				can_buffer_state = BUFFER_READY_TO_SEND;
	
				// The master's timing parameters need to be set
				fill_software_identity();     
				transmit_identity = 0;
			}
            
            break;
			
		default:
			break;		 
	}
}