/****************************************************************
 * Interrupt.c 
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * Interrupt Service Routines and template for some exception 
 *  handlers and peripherals interrupt service routines.
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "SerialBuffer.h"
#include "Interrupt.h"


void software_reset(void)
{
	send_a_string("\n\rResetting");

	// Wait for the string to transmit
	USART1->SR = 0;				// Make sure transmission complete bit is FALSE
	while ( (USART1->SR & 0x40) == 0 );

	// Set the software reset request
	SCB->AIRCR = (APP_RESET_CONTROL | 0x04);

	// Ensure completion of memory access
	__asm volatile ("dsb");

	// Wait for reset
	while(1);
}


void enable_interrupt(u8 peripheral, u8 priority)
{
    NVIC->IP[peripheral] = priority;
    
    NVIC->ISER[peripheral >> 0x05] = (u32)1 << (peripheral & 0x1F);
}


void clear_interrupt_pending(u8 peripheral)
{
    NVIC->ICPR[peripheral >> 0x05] = (u32)1 << (peripheral & 0x1F);
}


void NMI_Handler(void)
{
	send_a_string("\r\nNMI!");
}


u32 get_reg(void)
{
    u32 result;
  
   __ASM volatile ("mov %0, R0" : "=r" (result));
   return(result);
}


void send_status(s8 *trigger_name)
{
	wait_for_buffer();

	fill_a_string(trigger_name);

	fill_a_long(get_reg());
	fill_a_long(SCB->SHCSR);		// System Handler Control and State Register
	fill_a_long(SCB->CFSR);			// Configurable Fault Status Register
	fill_a_long(SCB->HFSR);			// Hard Fault Status Register
	fill_a_long(SCB->DFSR);			// Debug Fault Status Register
	fill_a_long(SCB->MMFAR);		// Mem Manage Address Register
	fill_a_long(SCB->BFAR);			// Bus Fault Address Register
	fill_a_long(SCB->AFSR);			// Auxiliary Fault Status Register
	send_sio_buffer();

	while(1);
}


void HardFault_Handler(void)
{
	__ASM volatile ("mov	R0, lr");

	send_status("\r\nHard Fault!");
}


void MemManage_Handler(void)
{
	__ASM volatile ("mov	R0, lr");

	send_status("\r\nMemory Manager!");
}


void BusFault_Handler(void)
{
	__ASM volatile ("mov	R0, lr");

	send_status("\r\nBus Fault!");
}


void UsageFault_Handler(void)
{
	__ASM volatile ("mov	R0, lr");

	send_status("\r\nUsage Fault!");
}


void SVC_Handler(void)
{
	send_a_string("\r\nSVC!");

	while (1);
}


void DebugMon_Handler(void)
{
	send_a_string("\r\nDebug Monitor!");

	while (1);
}


void PendSV_Handler(void)
{
	send_a_string("\r\nPending SV!");

	while (1);	
}