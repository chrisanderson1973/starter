/******************************************************************
 * SerialBuffer.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * RS232 Serial Communications Interface buffer management routines 
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 ******************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "Interrupt.h"
#include "Timer.h"
#include "SerialBuffer.h"


// The RS232 serial input structure serial_input is a circular 
//  buffer used to accept characters from the serial port anytime
#define SERIAL_INPUT_SIZE	20		// Receive interrupt's buffer
u8 serial_input[SERIAL_INPUT_SIZE];


// The RS232 serial I/O buffer sio_buffer[] is used to parse an 
//  incoming packet and store an in-process outgoing packet
u8 sio_buffer[SERIAL_BUFFER_SIZE];
u16 sio_index;


/*--------------------------------------------------------
 * initialize_serial()
 *
 * Initilize USART1 for 8 data bits, 1 stop bit, no parity
 *  9600 baud, no hardware flow control, receive and 
 *  transmit enabled, receive DMA enabled, and transmit 
 *  DMA enabled
 *-------------------------------------------------------*/

#define BAUDRATE		9600

void initialize_serial(void)
{
	// Disable the USART, 1 start bit, 8 data bits, Wake on idle,
	//  no parity, disable all interrupts, enable transmitter, 
	//  enable receiver in active mode, do not send break
	USART1->CR1 = 0x0000000C;			// .. 0000 0000 0000 1100

	// Disable LIN mode, one stop bit, disable the clock pin
	//  disable LIN interrupt, multiprocessor node address is 0
	USART1->CR2 = 0x00000000;			// .. 0000 0000 0000 0000

	// Disable hardware flow control, enable DMA for transmit and receive,
	//  disable smartcard mode, no NAK on parity error, no half duplex,
	//  disable IrDA mode, no error interrupt
	USART1->CR3 = 0x000000C0;			// .. 0000 0000 1100 0000

	/*------------------------
	 * Configure the baud rate
	 *-----------------------*/

	// Configure the baud rate
	// SYSTEM_CLOCK_FREQ = 32M
	// BR = fclk / (16 * I.F)  ->  I.F = 32M / (16 * 9600) = 208 + 5/16
	USART1->BRR = ((u16)208 << 4) | 5;

	// Enable USART1
    USART1->CR1 |= 0x2000;

	/*----------------------------------------------------------------------
	 * Configure the transmit DMA
	 * DMA Channel 4 is connected to USART1 transmit buffer empty event.
	 *  We use it to send a message from sio_buffer[] and interrupt us on 
	 *  transfer complete.  That interrupt service will disable the DMA 
	 *  channel and reset sio_index to 0.  So if sio_index = 0, sio_buffer[]
	 *  is available.
	 * DMA Channel 4 control register and bytes remaining register are setup 
	 *  each time a transmission is requested so they are skipped here
	 *---------------------------------------------------------------------*/

	// Disable DMA1 Channel 4
	DMA1_Channel4->CCR = 0;

	// DMA1 Channel 4 peripheral address register
	DMA1_Channel4->CPAR = (u32)&USART1->DR;
  
	// DMA1 Channel 4 memory address register
	DMA1_Channel4->CMAR = (u32)sio_buffer;
  
	// Reset interrupt pending bits for DMA1 Channel 4
	DMA1->IFCR = 0x0000F000;

	/*---------------------------------------------------
	 * Configure the Nested Vectored Interrupt Controller
	 *--------------------------------------------------*/

	// Configure the NVIC DMA1 Channel 4 interrupt priority and
	//  enable its IRQ Channel (only upper four bits used)
	enable_interrupt(DMA1_Channel4_IRQn, 0x10);

	/*-------------------------------------------------------------------
	 * Configure the receive DMA
	 * DMA1 Channel 5 is connected to USART1 receive event.  We use it to  
	 *  put received characters in serial_input[] circular buffer.  It is 
	 *  not disabled.  We use the number of data to transfer register to 
	 *  tell where the most recent character is located.
	 *------------------------------------------------------------------*/

	// Disable DMA1 Channel 5
	DMA1_Channel5->CCR = 0;

	// DMA1 Channel 5 peripheral address register
	DMA1_Channel5->CPAR = (u32)&USART1->DR;
  
	// DMA1 Channel 5 memory address register
	DMA1_Channel5->CMAR = (u32)serial_input;
  
	// DMA1 Channel 5 remaining bytes register
	DMA1_Channel5->CNDTR = SERIAL_INPUT_SIZE;

	// Setup the control register for peripheral to memory transfer,
	//  medium priority, 8 bits to memory, 8 bits from peripheral,
	//  increment memory address, do not increment peripheral address,
	//  enable circular mode, read from peripheral, no interrupts, 
	//  enable the DMA channel
	DMA1_Channel5->CCR = 0x10A1;			// .. 0001 0000 1010 0001

	sio_index = 0;
}


/*------------------------------------------------------------------
 * DMA1_Channel4_IRQHandler()
 *
 * DMA1 Channel interrupt request handler invoked on end of transfer
 *  from sio_buffer[] to USART1.
 *-----------------------------------------------------------------*/

void DMA1_Channel4_IRQHandler(void)
{
	// Disable the DMA channel
	DMA1_Channel4->CCR = 0;

	// Reset interrupt pending bits for DMA1 Channel4
	DMA1->IFCR = 0x0000F000;

	// Free up sio_buffer[]
	sio_index = 0;

DMA1->IFCR = 0;
}


bool get_serial_character(u8 *character)
{
	static u8 read_index = 0;

	u8 write_index;

	// Get the index to the next available input character location
	// It is the place that DMA will put the next received character
	//  which is one past the last one received
	write_index = SERIAL_INPUT_SIZE - DMA1_Channel5->CNDTR;

	if (read_index != write_index)
	{
		// There is a character in the input serial stream 

		*character = serial_input[read_index];

		if ( read_index < (SERIAL_INPUT_SIZE - 1) )
			read_index++;
		else
			read_index = 0;

		return TRUE;
	}
	else
		return FALSE;
}


/*--------------------------------------------------------
 * hex_to_ascii()
 *
 * Convert the lower nybble of the input byte to hex ASCII
 *-------------------------------------------------------*/

u8 hex_to_ascii(u8 number)
{
	number &= 0x0F;

	if (number <= 9)
		return number + '0';
	else
	{
		// The number is A-F
		// Add 55 to 10 to get 'A'
		return number + 0x37;
	}
}

 
/*---------------------------------------------------------
 * fill_a_byte()
 *
 * Convert the input byte into two hex ASCII characters and 
 *  assemble them, high nybble first, into sio_buffer[], 
 *  ready to transmit.
 *--------------------------------------------------------*/

void fill_a_byte(u8 send_data)
{
	sio_buffer[sio_index++] = hex_to_ascii(send_data >> 4);
	sio_buffer[sio_index++] = hex_to_ascii(send_data);
}


/*------------------------------------------------------
 * fill_an_int()
 *
 * Convert the input into four hex ASCII characters and 
 *  assemble them, high nybble first, into sio_buffer[], 
 *  ready to transmit.
 *-----------------------------------------------------*/

void fill_an_int(u16 send_data)
{
	fill_a_byte(((u8 *)&send_data)[1]);	//high byte
	fill_a_byte(((u8 *)&send_data)[0]);	//low byte
}


/*------------------------------------------------------
 * fill_a_long()
 *
 * Convert the input into eight hex ASCII characters and 
 *  assemble them, high nybble first, into sio_buffer,
 *  ready to transmit.
 *-----------------------------------------------------*/

void fill_a_long(u32 send_data)
{
	fill_an_int(((u16 *)&send_data)[1]);	//high int
	fill_an_int(((u16 *)&send_data)[0]);	//low int
}


void fill_a_number(s32 data, u8 fract_digits)
{
	u8 temp_byte,
	   printing = FALSE;

	s8 digit;

	s32 divisor = 1000000000;

	if (data < 0)
	{
		sio_buffer[sio_index++] = '-';
		data = -data;
	}

	for (digit = 9; digit >= 0; digit--)
	{
		if (digit == fract_digits)
			printing = TRUE;
		else if (digit == fract_digits - 1)
			sio_buffer[sio_index++] = '.';
	
		temp_byte = data / divisor;
		if (temp_byte || printing)
		{
			printing = TRUE;
			sio_buffer[sio_index++] = hex_to_ascii(temp_byte);
			data -= (u32)temp_byte * divisor;			
		}

		divisor /= 10;
	}
}


void fill_a_decimal(u32 decimal_data)
{
	fill_a_number(decimal_data, 0);
}


/*---------------------------------------------------------------------
 * fill_a_string()
 *
 * Copy a string from program memory to sio_buffer[], ready to transmit
 *--------------------------------------------------------------------*/

void fill_a_string(s8 *input_string)
{
	u8 index = 0;

	while (input_string[index] != NULL)
	{
		sio_buffer[sio_index++] = input_string[index++];
		
		if (index >= 60)
			break;
	}
}


void send_a_string(s8 *display_string)
{
	// Wait for a previous transmit to complete so we can use sio_buffer[]
	wait_for_buffer();

	fill_a_string(display_string);
	send_sio_buffer();
}


bool buffer_is_busy(void)
{
	return (DMA1_Channel4->CCR & 1);
}


/*-------------------------------------------------------------------------
 * wait_for_buffer()
 *
 * Serial transmission uses DMA to send sio_buffer[]
 * Wait for a previous transmit to free up the DMA channel and sio_buffer[]
 *------------------------------------------------------------------------*/

void wait_for_buffer(void)
{
	while (buffer_is_busy());
}


/*---------------------------------------------------------
 * start_transmitting()
 *
 * Some data to transmit has been assembled in sio_buffer[]
 * Invoke DMA1 Channel4 to transmit that data out USART1
 *--------------------------------------------------------*/

void start_transmitting(void)
{
	// DMA1 Channel4 remaining bytes register
	DMA1_Channel4->CNDTR = sio_index;

	// Setup the control register for memory to peripheral transfer,
	//  medium priority, 8 bits to peripheral, 8 bits from memory,
	//  increment memory address, do not increment peripheral address,
	//  disable circular mode, read from memory, interrupt on transfer 
	//  complete and enable the DMA channel
	DMA1_Channel4->CCR = (u32)0x00001093;			// .. 0001 0000 1001 0011
}


void send_sio_buffer(void)
{
	if (sio_index)
	{
		// Normal RS232 transmission
		start_transmitting();
	}
}