
/****************************************************************
 * CanPackets.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * Controller Area Network (CAN) multiple packet transmit/receive 
 *  protocol and functions
 *
 * 2/27/2013 CLA V0.01	Initial code taken from TitanShow
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "CanCommands.h"
#include "canbus.h"
#include "CanPackets.h"
#include "AStuff.h"
#include "Timer.h"
#include "Serial.h"
#include "E3prom.h"


// Maximum number of bytes and packets allowed in a 
//  multi-packet message
#define MAX_NUMBER_PACKETS		6	
#define MAX_MP_BYTES			(MAX_NUMBER_PACKETS * 7)

// Give the engine controller 2.5 seconds before declaring
//  it's communication to be failed
#define ECU_WATCHDOG_TIMEOUT	250

enum
{
	FILTER_0_INDEX	= 0,	// First to FIFO #0
	FILTER_1_INDEX	= 0,	// First to FIFO #1
	FILTER_2_INDEX	= 1,	// Second to FIFO #0
	FILTER_3_INDEX	= 2,	// Third to FIFO #0
	FILTER_4_INDEX	= 1,	// Second to FIFO #1
	FILTER_5_INDEX	= 2,	// Third to FIFO #1
	FILTER_6_INDEX	= 3,	// Fourth to FIFO #0
	FILTER_7_INDEX	= 3,	// Fourth to FIFO #1
	FILTER_8_INDEX	= 4,	// Fifth to FIFO #0
	FILTER_9_INDEX	= 4,	// Fifth to FIFO #1
	FILTER_10_INDEX	= 5,	// Sixth to FIFO #0
	FILTER_11_INDEX	= 5,	// Sixth to FIFO #1
	FILTER_12_INDEX	= 6,	// Seventh to FIFO #0
	FILTER_13_INDEX	= 6		// Seventh to FIFO #1
};	 


bool received_packet_flags[MAX_NUMBER_PACKETS];

// Define a message buffer for CAN communication 
// An entire received message is assembled in it before we parse
//  it and an entire message to transmit is assembled in it
//  before transmission begins
// We cannot accept messages from another sender until one in 
//  process completes
u8 canMsgBuffer[MAX_MP_BYTES + BUFF_DATA_BEGIN],

// The index into canMsgBuffer[]
	 can_buffer_index;
// The status of the message canMsgBuffer[] contains
buffer_state can_buffer_state;

static bool handshake_exchange;

void start_multiple_packets(CAN_TxMailBox_TypeDef *mailbox)
{
	// Transmit the RTS packet requesting to deliver a multiple 
	//  packet message per J1939-21
	// We transmit the data when we get a CTS from the destination

	// Number of data bytes in this RTS packet 
	//  not the multiple packet message to deliver
	mailbox->TDTR = 8;

	// Transmit Mailbox Message Identifier Register
	mailbox->TIR = CAN_XMIT_PRIORITY
									| PF_TPCM << 19
									| canMsgBuffer[BUFF_ADDRESS] << 11
									| this_can_address << 3;

	/*--------------------------------
	 * Transmit Mailbox Data Registers
	 *-------------------------------*/

	// Transport Protocol Connection Managment command byte		 	
	mailbox->TDLR = TPCM_RTS
	// Number of bytes in the message - we limit to 255
					| ((u16)canMsgBuffer[BUFF_LENGTH] << 8)
	// Number of packets in the message
	// There are 7 bytes per packet max since sequence number 
	//  occupies the first byte of each
					| ((u32)((canMsgBuffer[BUFF_LENGTH] + 6) / 7) << 24);

	// Number of packets allowed per CTS
	mailbox->TDHR = 1
	// PGN of the packeted message
					| ((u16)canMsgBuffer[BUFF_COMMAND] << 8)	// PDU ps
					| ((u32)canMsgBuffer[BUFF_FORMAT] << 16);	// PDU pf
	//				| ((0 << 24);								// Extended MMSB

	// Request transmit
	mailbox->TIR |= 1;

	can_buffer_state = BUFFER_TRANSMITTING;

	// Reset the communication watchdog timer
	can_comm_watchdog = SECONDS(10);
}


/*--------------------------------------------------------------
 * transmit_msg_buffer() 
 *
 * A message to transmit has been assembled into canMsgBuffer[] 
 * If multiple packets are required, send the RTS message and
 *  receipt of a CTS from the destination will trigger the data 
 *  packet transmission.
 * Otherwise send the single packet message.
 *-------------------------------------------------------------*/

void transmit_msg_buffer(void) 
{
	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	// If this packet transmits, set the buffer state to IDLE
	//  else, leave it as is and we will get back here to try it again

	if (mailbox)
	{
 
		// Transmit Mailbox Message Identifier Register
		// Leave ID2 set to zero
		mailbox->TIR = CAN_XMIT_PRIORITY
									| ((u32)canMsgBuffer[BUFF_FORMAT] << 19)
									| ((u16)this_can_address << 3);

 		if ( canMsgBuffer[BUFF_FORMAT] == PF_PROP_A )
		{
			/*---------------------------------------------------
			 * This is a proprietary A message.  It is special in 
			 *  that it has a Group Extension (COMMAND) and is 
			 *  destination specific.
			 * The destination address occupies the PDU specific 
			 *  byte in the CAN header, the COMMAND occupies the 
			 *  first data byte and only seven data bytes are 
			 *  possible in a single packet transmission
			 *--------------------------------------------------*/
            // fill_a_string("PF_PROP_A...\r\n");
			if ( canMsgBuffer[BUFF_LENGTH] > 7)
			{
				// There are more than 7 data bytes to send
				// Multiple packet protocol is required
				start_multiple_packets(mailbox);

				return;
			}

			// Put the destination address in the CAN header
			mailbox->TIR |= ((u32)canMsgBuffer[BUFF_ADDRESS] << 11);

			// Length is one greater since command takes a data byte
			mailbox->TDTR = canMsgBuffer[BUFF_LENGTH] + 1;

			// Command goes into the first (#0) data byte
			mailbox->TDLR = canMsgBuffer[BUFF_COMMAND]

			// Copy 7 data bytes even though they all may not be used
							| ((u16)canMsgBuffer[BUFF_DATA_BEGIN] << 8)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 1] << 16)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 2] << 24);

			mailbox->TDHR = canMsgBuffer[BUFF_DATA_BEGIN + 3]
							| ((u16)canMsgBuffer[BUFF_DATA_BEGIN + 4] << 8)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 5] << 16)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 6] << 24);
		}
		else
		{
			/*----------------------------------------------------
			 * This is not proprietary A message
			 * PDU1 Format (pf < 0xEF) has the destination address 
			 *  in the CAN header where PDU2 Format (pf > 0xEF) 
			 *  uses that byte for the Group Extension (ps)
			 * So for FORMAT < 0xEF, packets are destination 
			 *  specific.  For FORMAT > 0xEF, packets are global.  
			 *  In either case, eight data bytes are possible in 
			 *  a single packet.
			 *---------------------------------------------------*/

			if ( canMsgBuffer[BUFF_LENGTH] > 8)
			{
				// There are more than 8 data bytes to send
				// Multiple packet protocol is required
				start_multiple_packets(mailbox);

				return;
			}

		 	if ( canMsgBuffer[BUFF_FORMAT] < PF_PROP_A )
	 		{
	 			// This is a destination specific transfer

				// Put the ADDRESS in the CAN header
				mailbox->TIR |= ((u32)canMsgBuffer[BUFF_ADDRESS] << 11);

				// There is no Group Extension (COMMAND = 0)
	 		}
		 	else
		 	{
				// This is not a destination specific transfer

				// Since the ADDRESS is not used, the COMMAND occupies 
				//  the PDU specific byte in the CAN arbitration header 

				// Put the COMMAND in the CAN header
				mailbox->TIR |= ((u32)canMsgBuffer[BUFF_COMMAND] << 11);
		 	}

			mailbox->TDTR = canMsgBuffer[BUFF_LENGTH];

			// Copy 8 data bytes even though they all may not be used
			mailbox->TDLR = canMsgBuffer[BUFF_DATA_BEGIN]
							| ((u16)canMsgBuffer[BUFF_DATA_BEGIN + 1] << 8)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 2] << 16)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 3] << 24);

			mailbox->TDHR = canMsgBuffer[BUFF_DATA_BEGIN + 4]
							| ((u16)canMsgBuffer[BUFF_DATA_BEGIN + 5] << 8)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 6] << 16)
							| ((u32)canMsgBuffer[BUFF_DATA_BEGIN + 7] << 24);
		}
		
		// Request transmit
		mailbox->TIR |= 1;

		can_buffer_state = BUFFER_IDLE;
	}		
}


/*-----------------------------------------------------------
 * continue_mp_output()
 *
 * Called to send another packet of a multiple packet message
 * Get the packet from canMsgBuffer[] and send it
 *----------------------------------------------------------*/
 
bool continue_mp_output(void)
{
	u8 buffer_index,
		last_index;

	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	// If this packet does not transmit,
	//  we will get back here to try it again

	if (mailbox)
	{
		// Get the index into canMsgBuffer[] for the packet to send
		// Sequence number to transmit begins with 1
		buffer_index = (canMsgBuffer[BUFF_SEQUENCE] - 1) * 7;

		// Add the offset to get past the header stuff
		buffer_index += BUFF_DATA_BEGIN;

		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
							| ((u32)PF_TPDT << 19)						// This packet's pf
							| ((u32)canMsgBuffer[BUFF_ADDRESS] << 11)	// This packet's destination
							| ((u16)this_can_address << 3);

		// Number of data bytes in this packet
		mailbox->TDTR = 8;

		// Get the index to the last data byte in this message
		last_index = canMsgBuffer[BUFF_LENGTH] + BUFF_DATA_BEGIN - 1;

		/*------------------------------------
		 * CAN Transmit Mailbox Data Registers
		 *-----------------------------------*/

		mailbox->TDLR = canMsgBuffer[BUFF_SEQUENCE]		// This packet's sequence number
		// There must be at least one data byte
							| ((u16)canMsgBuffer[buffer_index++] << 8);

		mailbox->TDHR = 0xFFFFFFFF;

		if (buffer_index > last_index)
			mailbox->TDLR |= 0xFFFF0000;

		else
		{
			mailbox->TDLR |= ((u32)canMsgBuffer[buffer_index++] << 16);

			if (buffer_index > last_index)
				mailbox->TDLR |= 0xFF000000;

			else
			{
				mailbox->TDLR |= ((u32)canMsgBuffer[buffer_index++] << 24);

				if (buffer_index <= last_index)
				{
					mailbox->TDHR = canMsgBuffer[buffer_index++];

					if (buffer_index > last_index)
						mailbox->TDHR |= 0xFFFFFF00;
					else
					{
						mailbox->TDHR |= ((u16)canMsgBuffer[buffer_index++] << 8);

						if (buffer_index > last_index)
							mailbox->TDHR |= 0xFFFF0000;
						else
						{
							mailbox->TDHR |= ((u32)canMsgBuffer[buffer_index++] << 16);

							if (buffer_index > last_index)
								mailbox->TDHR |= 0xFF000000;
							else
								mailbox->TDHR |= ((u32)canMsgBuffer[buffer_index++] << 24);
						}
					}
				}
			}
		}

		// Request transmit
		mailbox->TIR |= 1;

		return TRUE;
	}
	else
		return FALSE;
}


void prepare_for_mp_receive(CAN_FIFOMailBox_TypeDef *mailbox)
{
	// Put the sender ID into the receive buffer
	canMsgBuffer[BUFF_ADDRESS] = mailbox->RIR >> 3;

	// Number of data bytes
	// We only accept the first MAX_MP_BYTES of a long message
	// If the high byte of message length is non-zero or the low byte 
	//  is too big, set the count to maximum
	if ( (u8)(mailbox->RDLR >> 16)
			|| ((u8)(mailbox->RDLR >> 8) >= MAX_MP_BYTES) )
	{
		canMsgBuffer[BUFF_LENGTH] = MAX_MP_BYTES;
	}
	else
		canMsgBuffer[BUFF_LENGTH] = mailbox->RDLR >> 8;

	// PDU
	canMsgBuffer[BUFF_COMMAND] = mailbox->RDHR >> 8;
	canMsgBuffer[BUFF_FORMAT] = mailbox->RDHR >> 16;

	// First requested packet number
	canMsgBuffer[BUFF_SEQUENCE] = 1;

	// Initialize the received packet flags to tell 
	//  if that packet has been received
	for (can_buffer_index = 0; 
			can_buffer_index < MAX_NUMBER_PACKETS; 
			can_buffer_index++)
	{
		received_packet_flags[can_buffer_index] = FALSE;
	}      
}


/*------------------------------------------------------------
 * continue_mp_input()
 *
 * Called after receiving a RTS to take delivery of a multiple 
 *  packet message and after receiving a packet of data
 * Assemble and transmit a CTS packet to the sender
 *-----------------------------------------------------------*/
  
bool continue_mp_input(void)
{
	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	// If this packet does not transmit,
	//  we will get back here to try it again

	if (mailbox)
	{
		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
							| ((u32)PF_TPCM << 19)			// This packet's pf
		// The source address of the multiple packet message is the 
		//  destination of this packet
							| ((u32)canMsgBuffer[BUFF_ADDRESS] << 11)
							| ((u16)this_can_address << 3);

		// CAN Transmit Mailbox Data Registers
		mailbox->TDLR = TPCM_CTS
							| ((u16)1 << 8)								// Number of packets requested
							| ((u32)canMsgBuffer[BUFF_SEQUENCE] << 16)	// Requested packet #
							| 0xFF000000;								// Not used, filled per J1939

		mailbox->TDHR = 0xFF
							| ((u16)canMsgBuffer[BUFF_COMMAND] << 8)	// Message ps
							| ((u32)canMsgBuffer[BUFF_FORMAT] << 16);	// Message pf, (MSB = 0)

		// Set data length
		mailbox->TDTR = 8;

		// Request transmit
		mailbox->TIR |= 1;

		return TRUE;
	}
	else
		return FALSE;
}


bool end_mp_input(void)
{
	CAN_TxMailBox_TypeDef *mailbox;

	mailbox = select_transmitter();

	// If this packet does not transmit,
	//  we will get back here to try it again

	if (mailbox)
	{
		// Transmit Mailbox Message Identifier Register
		mailbox->TIR = CAN_XMIT_PRIORITY
							| ((u32)PF_TPCM << 19)			// This packet's pf
		// The source address of the multiple packet message is the 
		//  destination of this packet
							| ((u32)canMsgBuffer[BUFF_ADDRESS] << 11)
							| ((u16)this_can_address << 3);

		// CAN Transmit Mailbox Data Registers
		mailbox->TDLR = TPCM_ACK
							| ((u16)canMsgBuffer[BUFF_LENGTH] << 8)		// Message size (MSB = 0)
							| ((u32)((canMsgBuffer[BUFF_LENGTH] + 6) / 7) << 24);	// Number of packets

		mailbox->TDHR = 0xFF
							| ((u16)canMsgBuffer[BUFF_COMMAND] << 8)	// Message ps
							| ((u32)canMsgBuffer[BUFF_FORMAT] << 16);	// Message pf (MSB = 0)

		// Set data length
		mailbox->TDTR = 8;

		// Request transmit
		mailbox->TIR |= 1;

		return TRUE;
	}
	else
		return FALSE;
}


void buffer_a_can_packet(CAN_FIFOMailBox_TypeDef *mailbox)
{
	if (can_buffer_state == BUFFER_IDLE)
	{
		// Put the sender ID into the receive buffer
		canMsgBuffer[BUFF_ADDRESS] = mailbox->RIR >> 3;

		// Put the PDU format byte into the receive buffer
		canMsgBuffer[BUFF_FORMAT] = mailbox->RIR >> 19;

		if (canMsgBuffer[BUFF_FORMAT] == PF_PROP_A)
		{							
			// This is a destination specific transfer
			// The PS (command) occupies the first data byte 
			//  so only 7 bytes are available for payload

			// Put the number of data bytes into the receive buffer
			// The number of actual data bytes is one less 
			//  since command occupies the first one
			canMsgBuffer[BUFF_LENGTH] = (mailbox->RDTR & 0x0F) - 1;

			// Get the command from the first data byte 
			canMsgBuffer[BUFF_COMMAND] = mailbox->RDLR;

			// Put the data bytes into the receive buffer even 
			//  though they might not be valid
			canMsgBuffer[BUFF_DATA_BEGIN] = mailbox->RDLR >> 8;
			canMsgBuffer[BUFF_DATA_BEGIN + 1] = mailbox->RDLR >> 16;
			canMsgBuffer[BUFF_DATA_BEGIN + 2] = mailbox->RDLR >> 24;
			canMsgBuffer[BUFF_DATA_BEGIN + 3] = mailbox->RDHR;
			canMsgBuffer[BUFF_DATA_BEGIN + 4] = mailbox->RDHR >> 8;
			canMsgBuffer[BUFF_DATA_BEGIN + 5] = mailbox->RDHR >> 16;
			canMsgBuffer[BUFF_DATA_BEGIN + 6] = mailbox->RDHR >> 24;
		}
		else
		{
			if (canMsgBuffer[BUFF_FORMAT] < PF_PROP_A)
			{							
				// This is a destination specific transfer
				// There is no PS (command) so 8 bytes are available for payload

				// Set the command 
				canMsgBuffer[BUFF_COMMAND] = 0xFF;
			}
			else
			{
				// This is a not destination specific transfer
				// The PS (command) is in the header where the destination 
				//  would have been and 8 data bytes are possible

				// Get the command from the CAN header 
				canMsgBuffer[BUFF_COMMAND] = CAN1->sFIFOMailBox[0].RIR >> 11;
			}

			// Put the number of data bytes into the receive buffer
			canMsgBuffer[BUFF_LENGTH] = mailbox->RDTR & 0x0F;

			// Put the data bytes into the receive buffer even 
			//  though they all might not be valid
			canMsgBuffer[BUFF_DATA_BEGIN] = mailbox->RDLR;
			canMsgBuffer[BUFF_DATA_BEGIN + 1] = mailbox->RDLR >> 8;
			canMsgBuffer[BUFF_DATA_BEGIN + 2] = mailbox->RDLR >> 16;
			canMsgBuffer[BUFF_DATA_BEGIN + 3] = mailbox->RDLR >> 24;
			canMsgBuffer[BUFF_DATA_BEGIN + 4] = mailbox->RDHR;
			canMsgBuffer[BUFF_DATA_BEGIN + 5] = mailbox->RDHR >> 8;
			canMsgBuffer[BUFF_DATA_BEGIN + 6] = mailbox->RDHR >> 16;
			canMsgBuffer[BUFF_DATA_BEGIN + 7] = mailbox->RDHR >> 24;
		}

		can_buffer_state = BUFFER_READY_TO_READ;
	}
}


/*------------------------------------------------------------------
 * USB_LP_CAN1_RX0_IRQHandler()
 *
 * CAN1 FIFO 0 receive message pending interrupt handler overlaps a 
 *  USB interrupt vector and so prevents using both at the same time
 * Filter 0 receives PGN from 0xFEE0 to 0xFEEF
 * Filter 2 receives PGN from 0xF000 to 0xF007
 * Filter 3 receives PGN 0xFECA	(DM1 messages)
 * Filter 6 receives PGN 0xFEDF (EEC3 message)
 *-----------------------------------------------------------------*/

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	u8 temp_byte;

	CAN_FIFOMailBox_TypeDef *mailbox;

	mailbox = &(CAN1->sFIFOMailBox[0]);

	// Get the Filter Match Index
	temp_byte = mailbox->RDTR >> 8;

	switch (temp_byte)
	{
		case FILTER_0_INDEX:
			can_filter_0_action(mailbox);
			break;

		case FILTER_3_INDEX:

			/*----------------------------------------------
			 * Filter 3 messages
			 * PDU = 0xFECA, DM1 message
			 * DM1 can be multi-packet but that would be 
			 *  received by filter 5 through FIFO 1
			 * So this DM1 message has 8 or fewer data bytes
			 *---------------------------------------------*/

			// If the CAN buffer is available, assemble it 
			//  there and signal the foreground to parse it 
			buffer_a_can_packet(mailbox);

			break;
	}

	// Release FIFO 0
	CAN1->RF0R = 0x20;
}


void connection_management_action(CAN_FIFOMailBox_TypeDef *mailbox)
{
    switch ( (u8)(mailbox->RDLR) )
    {
        case TPCM_CTS:
            
            // Verify that we are in process of sending a multiple 
            //  packet message to the source of this packet
            if ( (can_buffer_state == BUFFER_TRANSMITTING)
                    && ((u8)(mailbox->RIR >> 3) == canMsgBuffer[BUFF_ADDRESS]) )
            {
                // Number of data packets to transmit 
                //  triggers the transmit process
                packets_to_transmit = mailbox->RDLR >> 8; 

                // The next packet number to transmit
                canMsgBuffer[BUFF_SEQUENCE] = mailbox->RDLR >> 16;
            }

            // This may be a CTS packet in response to our 
            //  PCMD_REPROGRAM request								
            // Check to see if we should begin reprogramming a
            //  CAN server

            /*-----------------------------------------------------------------
             * When reprogramming all connected CAN servers (that support it), 
             *  we begin on receipt of the first CTS since they should all take 
             *  about the same time to erase program space and be ready
             * We ignore CTS messages from the other servers 
             * programming_server.requestee = 0xFF and each server responds 
             *  with their ID as the CTS packet source
             *----------------------------------------------------------------*/
            
            else if ( (programming_server.state == PRGM_WAIT_CTS)
                        && ( (u8)(mailbox->RDHR >> 8) == PCMD_REPROGRAM )
                        && ( (u8)(mailbox->RDHR >> 16) == PF_PROP_A ) )
            {
                programming_server.state = PRGM_DELAY1;
            }
        
        break;
        
        case TPCM_ACK:
            
        case TPCM_ABORT:
            can_buffer_state = BUFFER_IDLE;
        break;
        
        case TPCM_RTS:
            
              /*-----------------------------------------------
             * We have just received the announcement of an 
             *  in-coming multiple packet message 
             * Allow the reception if our buffer is available 
             *  and it is directed to us
             * We only accept the first MAX_MP_BYTES bytes
             *----------------------------------------------*/

            if ( (can_buffer_state == BUFFER_IDLE)
                    && ( (u8)(mailbox->RIR >> 11) == this_can_address ) )
            {
                // Signal to transmit a CTS to the sender
                can_buffer_state = BUFFER_SEND_CTS;

                handshake_exchange = TRUE;

                prepare_for_mp_receive(mailbox);
            }

        break;                            
        
        case TPCM_BAM:
        
            /*----------------------------------------------
             * We have just received the announcement of an 
             *  in-coming broadcast multiple packet message 
             * Allow the reception if our buffer is available 
             * We only accept the first MAX_MP_BYTES bytes
             *----------------------------------------------*/

            if (can_buffer_state == BUFFER_IDLE)
            {
                can_buffer_state = BUFFER_ACQUIRING;
                // Reset the communication watchdog timer
                can_comm_watchdog = SECONDS(10);
                handshake_exchange = FALSE;
                prepare_for_mp_receive(mailbox);
            }

        break;
        
        default:
            
            /*--------------------------------------------------
             * This is not a legal Connection Management command
             * Skip it with no action
             *-------------------------------------------------*/

        break;
    }
        
}

void data_transport_action(CAN_FIFOMailBox_TypeDef *mailbox)
{
    /*------------------------------------------------
    * This is the multi-packet data delivery protocol
    * It is a receive data packet
    *-----------------------------------------------*/
    u8 temp_byte;
    
    // Verify that we are acquiring from the source of 
    //  this packet
    if ( (can_buffer_state == BUFFER_ACQUIRING)
       && ( (u8)(mailbox->RIR >> 3) == canMsgBuffer[BUFF_ADDRESS] ) )
    {
        // Reset the communication watchdog timer
        can_comm_watchdog = SECONDS(10);

        // Get this sequence number
        // The zero data byte holds the packet sequence number 
        // which begins with 1							
        temp_byte = mailbox->RDLR - 1;

        // Flag this sequence as having been received
        received_packet_flags[temp_byte] = TRUE;

        // Set the index into canMsgBuffer[] for this packet
        // There are 7 bytes of data per packet, the packets are
        //  numbered beginning with 1, and the data starts at 
        //  BUFF_DATA_BEGIN
        can_buffer_index = (temp_byte * 7) + BUFF_DATA_BEGIN;

        // If there is room, copy seven bytes from the hardware
        if ( can_buffer_index <= (MAX_MP_BYTES + BUFF_DATA_BEGIN - 7) )
        {
            // Accumulate the data bytes of a multiple packet message
            canMsgBuffer[can_buffer_index++] = mailbox->RDLR >> 8;
            canMsgBuffer[can_buffer_index++] = mailbox->RDLR >> 16;
            canMsgBuffer[can_buffer_index++] = mailbox->RDLR >> 24;
            canMsgBuffer[can_buffer_index++] = mailbox->RDHR;
            canMsgBuffer[can_buffer_index++] = mailbox->RDHR >> 8;
            canMsgBuffer[can_buffer_index++] = mailbox->RDHR >> 16;
            canMsgBuffer[can_buffer_index++] = mailbox->RDHR >> 24;
        }

        if (handshake_exchange)
        {
            // This is a handshake exchange where we request 
            //  a single packet at a time via CTS packet 
            //  and then end the transmission with an ACK
            // This gives us the opportunity to request a 
            // retransmit of a lost packet

            // Preset buffer ready so we can set it  
            // when we find the first missing packet
            can_buffer_state = BUFFER_SEND_ACK;

            // Get the number of packets in this message
            temp_byte = (canMsgBuffer[BUFF_LENGTH] + 6) / 7;

            // See if all the packets have been received
            for (can_buffer_index = 0;
                 can_buffer_index < temp_byte;
                 can_buffer_index++)
            {
                if ( received_packet_flags[can_buffer_index] == FALSE)
                {
                    // We have encountered a missing packet
                    // Trigger to send a CTS
                    can_buffer_state = BUFFER_SEND_CTS;

                    // Put the missing sequence number in the buffer
                    // Packets are numbered beginning with 1
                    canMsgBuffer[BUFF_SEQUENCE] = can_buffer_index + 1;

                    // Stop looking when we find the first missing one
                break;
                }
            }
        }
        else
        {
            // This is not a handshake exchange

            // If we are done, setup to take action 
            //  if not, just quit and wait for the rest
            if (can_buffer_index >= canMsgBuffer[BUFF_LENGTH])
                can_buffer_state = BUFFER_READY_TO_READ;
        }
    }
                
}

/*--------------------------------------------------------------
 * CAN1_RX1_IRQHandler()
 *
 * CAN1 FIFO 1 receive message pending interrupt handler
 * Filter 1 receives PGN from 0xFEF0 to 0xFEFF
 * Filter 4 receives PFs from 0xE8 to 0xEF directed to all nodes
 * Filter 5 receives PFs from 0xE8 to 0xEF directed to this node	 
 *-------------------------------------------------------------*/

void CAN1_RX1_IRQHandler(void)
{

	u8 temp_byte;

	CAN_FIFOMailBox_TypeDef *mailbox;

    // Repeat until no message is pending
    // remember to wait for the FIFO to release

	mailbox = &(CAN1->sFIFOMailBox[1]);

	// Get the Filter Match Index
	temp_byte = (u8)(mailbox->RDTR >> 8);

	switch (temp_byte)
	{
		case FILTER_4_INDEX:	// PFs from 0xE8 to 0xEF directed to all nodes
            break;
        case FILTER_5_INDEX:	// PFs from 0xE8 to 0xEF directed to this node

        // Get the PDU format to see which message it is 
        switch ( (u8)(mailbox->RIR >> 19) )
        {
            case PF_TPCM:	// Transport protocol connection management (0xECxx)
                connection_management_action(mailbox);

            break;
            case PF_TPDT:	// Transport protocol data transfer (0xEBxx)
                data_transport_action(mailbox);

                
            break;
            
            case PF_PROP_A:
                       
                if ((u8)(mailbox->RIR >> 3) == CAN_MASTER_ADDR)
                {
                    master_comm_wd = SECONDS(15);

                    if ((u8)mailbox->RDLR == PCMD_STATUS)
                    {
                        // The Compressor Master controller status
                        contStatus = (u8)(mailbox->RDLR >> 8);

                    
                        pressure_range_input = mailbox->RDLR >> 16;
                    
                        // The Compressor Master controller operational state
                        controllerMode = mailbox->RDHR >> 24;

                        // Check the "delays have been set" flag
                        if ((u8)(mailbox->RDHR >> 8) == 0)
                            update_delays = TRUE;
                    }
                }
            
                if (((u8)mailbox->RDLR) == PCMD_GET_ID)
                    transmit_identity = ((u8)(mailbox->RIR >> 3));
                
                else if (((u8)mailbox->RDLR) == PCMD_QUESTION_SCREEN)
                {
                    // Another controller is asking us to send
                    //  our '?' screen across CAN for display
                                        
                    fill_terminal_info();
      
                    remote_dump_index = 0;
                    
                    // Keep the address of the requester
                    dump_to_address = canMsgBuffer[BUFF_ADDRESS];
       
                    remote_access_state = ACCESS_TRANSMIT;
                }
                // If a CAN device requests to reprogram us,
                //  verify the data is correct to provide some security
                else if ( ((u8)(mailbox->RDLR) == PCMD_REPROGRAM)
                        && ((u16)(mailbox->RDLR >> 8) == 0x5AA5) 
                        && ((u8)(mailbox->RDTR & 0x0F) == 3) )
                {
                    // Start CAN reprogramming
                    // A CAN device has requested to reprogram us

                    // Change the "program valid" byte to something 
                    //  other than valid
                    // Put the source ID (Programming Master) and our can 
                    //  address in the user option bytes for the BootLoader
                    set_option_bytes((u8)(mailbox->RIR >> 3) << 8 | this_can_address );
                 
                    //Set flag to get the foreground ask to service this reprogram interrupt
                    reprogram_state = REPROGRAM;
                }
                 
                //Use signed ints to check for minus/negative cases
                if ((u8)(mailbox->RIR>>3)==CAN_VIEWPORT_ADDR) // VIEWPORT_ADDDR = 0x36
                {
                    fill_a_string("\r\nIncoming canGateWay message.\r\n");
                
                    if (((u8)(mailbox->RDLR)) == PCMD_SET_TITAN_AUTOST_METHOD)
                    {
                    
                        fill_a_string("\r\nIncoming canGateWay - Autostart method\r\n");
                        if ((((s16)(mailbox->RDLR>>8)) >= ((s16)(0x00))) && (((s16)(mailbox->RDLR>>8)) <= ((s16)(0x02))))
                        {
                            ram_novram.pressure_config = ((s16)(mailbox->RDLR>>8));
                            fill_a_string("\r\nSent Autostart method: ");
                            fill_a_decimal(ram_novram.pressure_config);
                            fill_a_string("\r\n");
                        }
                        transmit_proprietary(PCMD_SET_TITAN_AUTOST_METHOD, CAN_VIEWPORT_ADDR);
                        
                        
                    } 
                    else if (((u8)mailbox->RDLR) == PCMD_SET_AUTOST_TIME)
                    {
                        fill_a_string("\r\nIncoming canGateWay - Autostart time\r\n");
                        if ((((s32)(mailbox->RDLR>>8)) >= 0x00) && (((s32)(mailbox->RDLR>>8)) <= 0x7fff))
                        {
                            ram_novram.cooldown_time = ((s32)(mailbox->RDLR>>8));
                            fill_a_string("\r\nSent cool down time: ");
                            fill_a_decimal(ram_novram.cooldown_time);
                            fill_a_string("\r\n");
                            //Send back data
                        }
                        transmit_proprietary(PCMD_SET_AUTOST_TIME, CAN_VIEWPORT_ADDR);  
                        
                    }              
                    else if (((u8)mailbox->RDLR) == PCMD_SET_AUTOST_THRESHOLD)
                    {
                    
                        fill_a_string("\r\nIncoming canGateWay - Autostart threshold\r\n");
                        if ((((s32)(mailbox->RDLR>>8)) >= 0x00) && (((s32)(mailbox->RDLR>>8)) <= 0x7fff))
                        {
                            ram_novram.pressure_threshold = ((s32)(mailbox->RDLR>>8));
                            fill_a_string("\r\nSent cool autostart threshold: ");
                            fill_a_decimal(ram_novram.pressure_threshold);
                            fill_a_string("\r\n");
                        }
                        transmit_proprietary(PCMD_SET_AUTOST_THRESHOLD, CAN_VIEWPORT_ADDR);
                        
   
                    }
                    send_sio_buffer();
                 }
                    
             break;
         
             default:

                    // A single packet message has been received 
                    // If the CAN buffer is available, assemble it 
                    //  there and trigger the foreground to parse it 
                    if (can_buffer_state == BUFFER_IDLE)
                    {
                        // Put the sender ID into the receive buffer
                        canMsgBuffer[BUFF_ADDRESS] = (u8)(mailbox->RIR >> 24);//CAN0_FIFO1_MIDR3;

                        // Put the PDU format byte into the receive buffer
                        canMsgBuffer[BUFF_FORMAT] = (u8)(mailbox->RIR >> 8);//CAN0_FIFO1_MIDR1;

                        if ((u8)(mailbox->RIR >> 8) <= PF_PROP_A ) //CAN0_FIFO1_MIDR1 
                        {							
                            // This is a destination specific transfer

                            // Get the command from the first data byte 
                            canMsgBuffer[BUFF_COMMAND] = (u8)(mailbox->RDLR);//CAN0_FIFO1_MDAR0;

                            // Put the number of data bytes into the receive buffer
                            // The number of actual data bytes is one less 
                            //  since command occupies the first one
                            //canMsgBuffer[BUFF_LENGTH] = CAN0_FIFO1_MDLC - 1;
                            canMsgBuffer[BUFF_LENGTH] = (mailbox->RDTR & 0x0F) - 1;


                        // Put the data bytes into the receive buffer even 
                            //  though they might not be valid
                            canMsgBuffer[BUFF_DATA_BEGIN] = (u8)(mailbox->RDLR);
                            canMsgBuffer[BUFF_DATA_BEGIN + 1] = (u8)(mailbox->RDLR >> 8);
                            canMsgBuffer[BUFF_DATA_BEGIN + 2] = (u8)(mailbox->RDLR >> 16);
                            canMsgBuffer[BUFF_DATA_BEGIN + 3] = (u8)(mailbox->RDLR >> 24);
                            canMsgBuffer[BUFF_DATA_BEGIN + 4] = (u8)(mailbox->RDHR);
                            canMsgBuffer[BUFF_DATA_BEGIN + 5] = (u8)(mailbox->RDHR >> 8);
                            canMsgBuffer[BUFF_DATA_BEGIN + 6] = (u8)(mailbox->RDHR >> 16);
                        }
                        else
                        {
                            // This is a not destination specific transfer

                            // Get the command from the CAN header 
                            canMsgBuffer[BUFF_COMMAND] = (u8)(mailbox->RIR >> 16);//CAN0_FIFO1_MIDR2;

                            // Put the number of data bytes into the receive buffer
                           // canMsgBuffer[BUFF_LENGTH] = CAN0_FIFO1_MDLC;
                           		canMsgBuffer[BUFF_LENGTH] = (mailbox->RDTR & 0x0F) - 1;

                            // Put the data bytes into the receive buffer even 
                            //  though they all might not be valid
                            canMsgBuffer[BUFF_DATA_BEGIN] = (u8)(mailbox->RDLR);//CAN0_FIFO1_MDAR0;
                            canMsgBuffer[BUFF_DATA_BEGIN + 1] = (u8)(mailbox->RDLR >> 8);//CAN0_FIFO1_MDAR1;
                            canMsgBuffer[BUFF_DATA_BEGIN + 2] = (u8)(mailbox->RDLR >> 16);//CAN0_FIFO1_MDAR2;
                            canMsgBuffer[BUFF_DATA_BEGIN + 3] = (u8)(mailbox->RDLR >> 24);//CAN0_FIFO1_MDAR3;
                            canMsgBuffer[BUFF_DATA_BEGIN + 4] = (u8)(mailbox->RDHR);//CAN0_FIFO1_MDAR4;
                            canMsgBuffer[BUFF_DATA_BEGIN + 5] = (u8)(mailbox->RDHR >> 8);//CAN0_FIFO1_MDAR5;
                            canMsgBuffer[BUFF_DATA_BEGIN + 6] = (u8)(mailbox->RDHR >> 16);//CAN0_FIFO1_MDAR6;
                            canMsgBuffer[BUFF_DATA_BEGIN + 7] = (u8)(mailbox->RDHR >> 24);//CAN0_FIFO1_MDAR7;
                        }

                        can_buffer_state = BUFFER_READY_TO_READ;
                    }
                    break;
   
    
       
        }
    
        break;
#ifdef NEVER    
		case FILTER_7_INDEX:
			break;
		case FILTER_9_INDEX:
			break;
		case FILTER_11_INDEX:
			break;
		case FILTER_13_INDEX:
			break;
#endif
		default:
			// Some internal error has occurred
			// A filter not attached to FIFO 1 has reported a hit
		break;
	}
        
	// Release FIFO 1
	CAN1->RF1R = 0x20;

	clear_interrupt_pending(CAN1_RX1_IRQn);

}