/****************************************************************
 * Interrupt.h 
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Prototypes for some interrupt handlers.
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/

#ifndef __INTERRUPT_H
#define __INTERRUPT_H


// Application Interrupt and Reset Control Register value
//  set for interrupt priority grouping 7: all four priority bits 
//  used for sub-priority, none for pre-emption
#define APP_RESET_CONTROL		0x05FA0700


extern void software_reset(void);
extern void enable_interrupt(u8, u8);
extern void clear_interrupt_pending(u8);
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);
extern void SVC_Handler(void);
extern void DebugMon_Handler(void);
extern void PendSV_Handler(void);

#endif
