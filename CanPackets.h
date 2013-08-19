/****************************************************************
 * CanPackets.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Controller Area Network (CAN) multiple packet transmit and 
 *  receive protocol variables and function prototypes
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/

#ifndef __CANPACKETS_H
#define __CANPACKETS_H


// CAN buffer states
typedef enum 
{
	BUFFER_IDLE,			// Available
	BUFFER_SEND_CTS,		// Transmit CTS to the sender a multiple packet message
	BUFFER_ACQUIRING,		// Taking input packet(s)
	BUFFER_SEND_ACK,		// Transmit Ack to the sender of a multiple packet message
	BUFFER_READY_TO_READ,	// Input complete, ready to parse
	BUFFER_READY_TO_SEND,	// Parse done, ready to transmit
	BUFFER_TRANSMITTING		// Transmitting packet(s)
}buffer_state;


// Define indexes into canMsgBuffer[]
// For received messages: [BUFF_ADDRESS] holds the source address
//  and [BUFF_SEQUENCE] holds the sequence number to request via CTS 
// For transmitted messages: [BUFF_ADDRESS] holds the destination address 
//  and [BUFF_SEQUENCE] holds the sequence number to transmit next
// For all messages, the data begins at [BUFF_DATA_BEGIN]
// Destination specific packets use the first data byte for 
//  COMMAND and so can only transport 7 bytes of data.
// For Broadcast messages (non-destination specific), the command 
//  is assembled into the CAN arbitration header, [BUFF_ADDRESS] is not 
//  used and eight data bytes are possible in a single packet.
enum
{
	BUFF_ADDRESS =		0,
	BUFF_LENGTH =		1,	// Number of data bytes in the message
	BUFF_COMMAND =		2,	// PDU specific (LS byte of PGN)
	BUFF_FORMAT =		3,	// PDU format byte (MS byte of PGN)
	BUFF_SEQUENCE =		4,
	BUFF_DATA_BEGIN =	5
};


extern u8 canMsgBuffer[],
			can_buffer_index;

extern buffer_state can_buffer_state;


extern void transmit_msg_buffer(void);
extern bool continue_mp_output(void);
extern bool continue_mp_input(void);
extern bool end_mp_input(void);

#endif