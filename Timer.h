/****************************************************************
 * Timer.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Timer setup and maintenance defines and function prototypes 
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/

#ifndef __TIMER_H
#define __TIMER_H


#define TICKLE_THE_DOG		{IWDG->KR = 0x5555; IWDG->KR = 0xAAAA;}


// Define a macro to calculate the number of 10ms clock ticks 
//  in a variable number of seconds
#define SECONDS(num)	num * 100
// ..and in a variable number of minutes
#define MINUTES(num)	SECONDS(num * 60)
// ..and in a variable number of hours
#define HOURS(num)		MINUTES(num * 60)


extern volatile u32 system_timer;
extern volatile u32 previous_timer;


extern void initialize_clocks(void);
extern void initialize_watchdog(void);
extern void initialize_timers(void);
extern void wait_for_tic(u8);
extern void wait_some_seconds(s8);


#endif



