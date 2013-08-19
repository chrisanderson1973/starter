/****************************************************************
 * timer.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 * Timer setup and maintainence routines 
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 ****************************************************************/


#include "stm32f10x.h"
#include "Common.h"
#include "IOdata.h"
#include "Timer.h"


volatile u32 system_timer = 0;
volatile u32 previous_timer = 0;


// Define the desired RCC configuration register contents
// Disable clock output, USB prescaler is PLL / 1.5 (slowest)
// PLLCLK = HSE * 4 = 32MHz, HSE not divided, PLL clock is HSE
// PLL multiplier is 4 (if PLL is disabled)
// HSE multiplier to PLL is not divided (fastest) (if PLL is disabled)
// ADC clock is PCLK2 / 4
// Set APB2, APB1, and AHB prescalers to minimum (fastest)
// Select PLL as system clock
#define SYSTEM_CLOCK_CONFIG			0x00094002


// The system clock frequency established by the PLL
#define SYSTEM_CLOCK_FREQ			32000000


/*------------------------------------------------------------------
 * initialize_clocks()
 *
 * Called to setup the microcontroller system clocks, initialize the 
 *  Embedded Flash Interface, and setup the PLL
 *-----------------------------------------------------------------*/

void initialize_clocks(void)
{
	// Make sure the HSI is on
	RCC->CR |= 0x01;

	// Wait for the HSI to be stable
	while( (RCC->CR & 0x02) == 0 );

	// Select HSI as system clock
	RCC->CFGR = SYSTEM_CLOCK_CONFIG & 0xFFFFFFFC;

	// Disable the PLL, clock security system and HSE
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	// Disable HSE bypass (if HSE is disabled)
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable interrupts and clear pending bits for 
	//  the PLL, HSE, HSI, LSE, LSI and Clock Security ready interrupts
	RCC->CIR = 0x009F0000;

	// Enable the HSE clock
	RCC->CR |= (uint32_t)0x00010000;

	// Wait for the HSE crystal to stabilize
	while( (RCC->CR & 0x00020000) == 0 );

	// Enable FLASH Prefetch Buffer, disable FLASH half cycle access
	// Setup for one wait state on FLASH access for 24MHz <= SYSCLK <= 48MHz
	FLASH->ACR = 0x11;

	/*-----------------------------------------------------
	 * HCLK, PCLK2 and PCLK1 are set to their fastest above
	 * This is a good place to slow them down if desired
	 *----------------------------------------------------*/

	// Now that the PLL is disabled, rewrite the configuration register 
	//  to set the parameters that depend on disabled PLL
	// Configure the PLL: PLLCLK = HSE * 4 = 32MHz
	// PLL multiplication factor is 4, HSE is not divided
	// Select HSE as PLL clock, keep HSI as system clock
	RCC->CFGR = SYSTEM_CLOCK_CONFIG & 0xFFFFFFFC;

	// Enable the PLL
	RCC->CR |= 0x01000000;

	// Wait for the PLL to stabilize
	while((RCC->CR & 0x02000000) == 0);

	// Select PLL as system clock source
	RCC->CFGR = SYSTEM_CLOCK_CONFIG;    

	// Wait until the PLL is used as system clock source
	while ( (RCC->CFGR & (uint32_t)0x0000000C) != (uint32_t)0x08 );

	/*--------------------------------------------------------------------------
	 * Enable some peripheral clocks
	 * Leave the HSI clock running as FLASH write and erase functions require it
	 *-------------------------------------------------------------------------*/

	// CAN1 and SPI2
    RCC->APB1ENR |= 0x02004000;

	// USART1, ADC1, GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO
    RCC->APB2ENR = 0x0000427D;

	// DMA1
	RCC->AHBENR |= 0x00000001;

	FSMC_Bank1->BTCR[0] = 0x00000080;		// SRAM/NOR Flash CS control register 1
//	FSMC_Bank1->BTCR[1];					// SRAM/NOR - Flash CS timing register 1
	FSMC_Bank1->BTCR[2] = 0x00000080;		// 		" control 2
//	FSMC_Bank1->BTCR[3];					// 		" timing 2
	FSMC_Bank1->BTCR[4] = 0x00000080;		// 		" control 3
//	FSMC_Bank1->BTCR[5];					// 		" timing 3
	FSMC_Bank1->BTCR[6] = 0x00000080;		// 		" control 4
//	FSMC_Bank1->BTCR[7];					// 		" timing 4

//	FSMC_Bank1E->BWTR[0];					// SRAM/NOR - Flash write timing register 1
//	FSMC_Bank1E->BWTR[1];
//	FSMC_Bank1E->BWTR[2];					// SRAM/NOR - Flash write timing register 2
//	FSMC_Bank1E->BWTR[3];
//	FSMC_Bank1E->BWTR[4];					// SRAM/NOR - Flash write timing register 3
//	FSMC_Bank1E->BWTR[5];
//	FSMC_Bank1E->BWTR[6];					// SRAM/NOR - Flash write timing register 4

//	FSMC_Bank2->PCR2;						// NAND Flash/PC Card controller register 2
//	FSMC_Bank2->SR2;						// FIFO Status and Interrupt register 2
//	FSMC_Bank2->PMEM2;						// Common Memory Space Timing register 2
//	FSMC_Bank2->PATT2;						// Attribute Memory Space Timing register 2
//	FSMC_Bank2->RESERVED0;
//	FSMC_Bank2->ECCR2;						// Error Correction Code Result register 2

//	FSMC_Bank3->PCR3;						// NAND Flash/PC Card controller register 3
//	FSMC_Bank3->SR3;						// FIFO Status and Interrupt register 3
//	FSMC_Bank3->PMEM3;						// Common Memory Space Timing register 3
//	FSMC_Bank3->PATT3;						// Attribute Memory Space Timing register 3
//	FSMC_Bank3->RESERVED0;   
//	FSMC_Bank3->ECCR3;						// Error Correction Code Result register 3

//	FSMC_Bank4->PCR4;						// NAND Flash/PC Card controller register 4
//	FSMC_Bank4->SR4;						// FIFO Status and Interrupt register 4
//	FSMC_Bank4->PMEM4;						// Common Memory Space Timing register 4
//	FSMC_Bank4->PATT4;						// Attribute Memory Space Timing register 4
//	FSMC_Bank4->PIO4;						// IO Space Timing register 4

	// Enable Usage Fault, Bus Fault and Memory Manager Fault exceptions
	SCB->SHCSR = 0x00070000;				// System Handler Control and State register
}


void initialize_watchdog(void)
{
	/*-----------------------------------------------
	 * Configure the WDG to generate an EOC interrupt
	 *----------------------------------------------*/
}


void initialize_timers(void)
{
	/*--------------------------------------------------------------
	 * Configure the core SysTick as the system 10ms timer interrupt 
	 *-------------------------------------------------------------*/

	// Set reload register for 10ms interrupts: 32MHz x 0.005sec
	SysTick->LOAD = (SYSTEM_CLOCK_FREQ / 200) - 1;

	// Set the priority for Cortex-M0 SysTick System Interrupt
	// Priority is the upper nybble, 15 is the lowest
    SCB->SHP[SysTick_IRQn + 12] = 0x60;

	// Load the SysTick Counter Value
	SysTick->VAL = 0;

	// Set the SysTick clock source to the core clock ( > 2.5 x reference clock)
	// Enable SysTick IRQ and start the SysTick timer
	SysTick->CTRL = 0x07;
}


void wait_for_tic(u8 count)
{
	s32 time;

	time = system_timer + count;

	// Wait for system_timer to get to the target
	while (time != system_timer);
}


/*----------------------------------------------------
 * wait_some_seconds()
 *
 * Delay loop sticks here until the system timer ticks 
 *  100 * the input parameter
 *---------------------------------------------------*/
 
void wait_some_seconds(s8 delay)
{
	// Construct a the delay from 0.01sec timer ticks
	while (delay--)	
		wait_for_tic(100);
}


/*-----------------------------------------------------------------
 * SysTick_Handler()
 *
 * Cortex core system tick timer interrupt request called avery 5ms
 * The main system timer gets incremented every other pass - 10ms
 *----------------------------------------------------------------*/

void SysTick_Handler(void)
{
	static u16 local_count;

	local_count++;

	if (local_count & 0x01)
		system_timer++;
}


#ifdef NEVER
/*------------------------------------------------------------------
 * TIM1_IRQHandler()
 *
 * TIM1 interrupt request
 *-----------------------------------------------------------------*/

void TIM1_BRK_IRQHandler(void)
{
	//– Break input
}
void TIM1_UP_IRQHandler(void)
{
	//Update: counter overflow/underflow, counter initialization
	//(by software or internal/external trigger)
}
void TIM1_TRG_COM_IRQHandler(void)
{
	//- Trigger event (counter start, stop, initialization or count by internal/external trigger)
}
void TIM1_CC_IRQHandler(void)
{
	//– Input capture
	//– Output compare
}
/*------------------------------------------------------------------
 * TIM2_IRQHandler()
 *
 * TIM2 interrupt request
 *-----------------------------------------------------------------*/
void TIM2_IRQHandler(void)
{
}
#endif

/*-----------------------------------------------------------------
 * TIM3_IRQHandler()
 *
 * TIM3 interrupt request
 *----------------------------------------------------------------*/
void TIM3_IRQHandler(void)
{
	// Clear the Output Compare 1 flag
	TIM3->SR &= ~0x4000;
  
	// Reset TIM3 Counter
	TIM3->CNT = 0x1234;
}


/*------------------------------------
 * WWDG_IRQHandler()
 *
 * Windowed Watchdog interrupt handler
 *-----------------------------------*/

void WWDG_IRQHandler(void)
{
	// Clear the EOC interrupt pending bit
	WWDG->SR &= ~0x01;
}