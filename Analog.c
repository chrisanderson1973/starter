/**********************************************************************
 * Analog.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 * 
 * On-chip Analog to Digital Convertor (ADC) and Digital to Analog
 *  Convertor (DAC) interface and maintenance routines
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 **********************************************************************/

#include "stm32f10x.h"
#include "common.h"
#include "Interrupt.h"
#include "Timer.h"
#include "IOdata.h"
#include "CanCommands.h"
#include "Canbus.h"
#include "ThermTables.h"
#include "Analog.h"
#include "e3prom.h"

bool alternator_charge_ok;

u16 cooldown_count;
s16 ambient_temperature;
s16 board_temperature;

u16 startnow_sw_debounce;
u16 pressure_threshold;
u16 transducer_pressure;

u16 battery_voltage;

// Define a RAM array for DMA to put ADC conversions
u16 raw_analog[NUMBER_OF_ADC];


typedef struct
{
	u8 adc_channel;		// The ADC channel to read
	u8 mux_select;		// On-board multiplexer select
	u32 terminator_1;	// Bit Band access address for the desired terminator
	u32 terminator_2;	// Bit Band access address of a second terminator if desired
}mplex_setup_def;


// Define the analog input setup for each of the multiplexed analog_index_def 
//  entries (index less than NUMBER_MULTIPLEXED)
// One is updated every 10ms and there is a four sample average on each
// They can be in any order and one can be repeated to be updated faster
const mplex_setup_def multiplexer_setup[NUMBER_ANALOG_INPUTS] =
{
	// ADC channel, Mux Select, BitBand addresses of terminators enable
	{ADC_MULTIPLEXER,	6,	TERM_MUX_PULL_UP,	TERM_UNUSED},	// AIN_DISCH_T
	{ADC_MULTIPLEXER,	5,	TERM_MUX_PULL_UP,	TERM_UNUSED},	// AIN_SEP_TANK_T
	{ADC_MULTIPLEXER,	2,	TERM_MUX_PULL_UP,	TERM_UNUSED},	// AIN_FUEL_LEVEL
	{ADC_MULTIPLEXER,	3,	TERM_MUX_PULL_UP,	TERM_UNUSED},	// AIN_AMBIENT_T
	{ADC_MULTIPLEXER,	0,	TERM_UNUSED,		TERM_UNUSED},	// AIN_AIREND_OIL_P
	{ADC_MULTIPLEXER,	1,	TERM_MUX_PULL_UP,	TERM_MUX_PULL_DN},	// AIN_START_CONTROL
	{ADC_MULTIPLEXER,	4,	TERM_MUX_PULL_UP,	TERM_MUX_PULL_DN}	// AIN_REGEN_CONTROL
};


analog_input_def analog_input[NUMBER_ANALOG_INPUTS];	

/*------------------------------------------------------------------------
 * initialize_analog_input()
 *
 * Initialize the ADC system to convert all used channels continuously.
 * Seven inputs are multiplexed onto one of our analog input channels and 
 *  must be selected then separated by the foregroung task.
 * The ADC clock is PCLK2 / 4 = 8MHz, set by SYSTEM_CLOCK_CONFIG
 * All sample times are set to 28.5 clocks so a single conversion takes 
 *  5.125us (conversion adds 12.5 clocks)
 * There are ten on-chip analog channels used so each one is converted 
 *  every 51.25us.
 *-----------------------------------------------------------------------*/

void initialize_analog_input(void)
{
	/*-------------------------
	 * Configure DMA1 Channel 1
	 *------------------------*/

	// Disable DMA1 Channel 1
	DMA1_Channel1->CCR = 0;

	// Peripheral address register
	DMA1_Channel1->CPAR = (u32)&ADC1->DR;			// ADC1_DR_Address

	// Memory address register
	DMA1_Channel1->CMAR = (u32)raw_analog;
  
	// Set the number of transfers per cycle
	DMA1_Channel1->CNDTR = 10;

	// Disable memory to memory mode, set high priority
	// Memory size is 16 bits, peripheral size is 16 bits
	// Enable memory address increment, disable peripheral address increment
	// Enable circular mode, transfer from peripheral to memory
	// Enable transfer error interrupt, disable other interrupts
	DMA1_Channel1->CCR = 0x000025A8;				// .. 0010 0101 1010 1000

    // Reset all DMA1 Channel 1 interrupt pending bits
    DMA1->IFCR |= 0x0000000F;

	// Enable DMA1 channel1
	DMA1_Channel1->CCR |= 1;

	// Configure the NVIC DMA1 Channel1 interrupt
	enable_interrupt(DMA1_Channel1_IRQn, 2);

	/*---------------
	 * Configure ADC1
	 *--------------*/

	// Disable analog watchdog mode, set independant conversion mode
	// Disble discontinuous mode, no automatic injected group conversion
	// Enable scan mode, disable interrupts
	ADC1->CR1 = 0x00000100;

	// Disable temperature sensor, do not start conversion
	// Enable regular channel conversion on external event: SWSTART
	// Disable injected channel conversion on external event
	// Set for right data alignment, enable DMA
	// Clear calibration registers, do not start calibration
	// Continuous conversion, enable the ADC
	ADC1->CR2 = 0x001E010B;

    // Set the channel sample time for all channels to 28.5 clocks
    ADC1->SMPR1 = 0x006DB6DB;			// 0000 0000 0110 1101 1011 0110 1101 1011
	ADC1->SMPR2 = 0x1B6DB6DB;			// 0001 1011 0110 1101 1011 0110 1101 1011

	// Set regular conversion sequence length for 10 conversions
	// Set regular conversion for channels 0, 1, 2, 3, 6, 7, 14, 12, 10 and 11
	ADC1->SQR1 = 0x00900000;
	ADC1->SQR2 = 0x0005A98E;			// 00/00 000/0 0000 /0101 1/010 10/01 100/0 1110
    
    ADC1->SQR3 = 0x0E618820;			// 00/00 111/0 0110 /0001 1/000 10/00 001/0 0000

	// Wait for the calibration registers to be reset
	while(ADC1->CR2 & 8);

	// Start ADC1 calibaration
	ADC1->CR2 |= 0x04;

	// Wait for calibration to complete
	while(ADC1->CR2 & 4);
     
	// Start the conversion
    ADC1->CR2 |= 0x00400000;	
}


/*-----------------------------------------------------------------
 * DMA1_Channel1_IRQHandler()
 *
 * DMA1 Channel1 interrupt handler.
 * DMA1 Channel1 moves data from the ADC to a holding array for raw 
 *  conversion results.
 * The transfer error interrupt is the only one enabled
 * If this hits, the ADC stops converting
 *----------------------------------------------------------------*/

void DMA1_Channel1_IRQHandler(void)
{
	send_a_string("ADC DMA error");

	// rewrite setup?

	// Clear all DMA1 Channel1 interrupt flags
	DMA1->IFCR = 0x00000001;

	// Re-enable DMA1 channel1
//	DMA1_Channel1->CCR |= 1;
}


/*--------------------------------------------------------------------
 * update_analog_input()
 *
 * Add a reading to one of the analog input channels' history and sum 
 *  it with the previous three samples from the same channel.
 * The sum is called .average and is 4 times larger than analog input
 * So, for input = 0x0FFF, the .average is 0x3FFC
 *-------------------------------------------------------------------*/

void update_analog_input(analog_index_def which, u16 value)
{
	u16 summation;
	u8 count;

	// Get the index to the previous reading
	// Increment it to the place for the present reading
	count = (analog_input[which].index + 1) & 0x03;

	// Store it back away
	analog_input[which].index = count;

	// Put the present reading there
	analog_input[which].history[count] = value;

	// Sum the history samples
	summation = 0;
	for (count = 0; count < 4; count++)
		summation += analog_input[which].history[count];

	// Store the sum as the average
	// maximum 0x3FFC
	analog_input[which].average = summation;
}


/*-------------------------------------------------------------
 * read_raw_analogs()
 *
 * Called no more often than once avery 10ms to update some 
 *  analog inputs.
 * There are NUMBER_ANALOG_INPUTS channels and each has a four 
 *  sample sum to get .average.
 * The multiplexed inputs are updated one at a time so it takes 
 *  several passes to respond to a step change.
 * The directly connected analog inputs are updated every pass 
 *  so they respond to a step change in 40ms.
 *------------------------------------------------------------*/

void read_raw_analogs(void)
{
	static analog_index_def analog_index = 0;

	/*-----------------------------------------
	 * Update the unmultiplexed analog channels
	 * Read the present value from raw_analog[]
	 *----------------------------------------*/

	update_analog_input(AIN_BATTERY_V, raw_analog[ADC_BATTERY_V]);
	update_analog_input(AIN_TRANS_PRES_P, raw_analog[ADC_REG_PRESSURE]);
	update_analog_input(AIN_SEP_TANK_P, raw_analog[ADC_SEP_PRESSURE]);
	update_analog_input(AIN_BOARD_TEMP, raw_analog[ADC_BOARD_TEMP]);
    update_analog_input(AIN_STARTNOW_SW, raw_analog[ADC_DIN3]);
	/*--------------------------------------------------------
	 * Update the multiplexed channel that was setup last pass
	 *-------------------------------------------------------*/

	update_analog_input(analog_index, raw_analog[multiplexer_setup[analog_index].adc_channel]);

	// Disable the terminator(s) for the analog input we just updated
	TURN_OFF(multiplexer_setup[analog_index].terminator_1);
	TURN_OFF(multiplexer_setup[analog_index].terminator_2);

	if ( analog_index < (NUMBER_MULTIPLEXED - 1) )
		analog_index++;
	else
		analog_index = 0;

	// Select the multiplexed channel for the next pass
	GPIOC->BSRR = 0xE0000000 | (multiplexer_setup[analog_index].mux_select << 13);

	// Enable the terminator for the next analog input we will update
	TURN_ON(multiplexer_setup[analog_index].terminator_1);
	TURN_ON(multiplexer_setup[analog_index].terminator_2);
}


s16 deg_c16_to_f(s16 c_temp)
{
	s16 answer;

	// Convert from C(x16) to F
	// T * ((180/100) / 16) + 32
	//	=> T * 9/80 + 32
	answer = c_temp * 9;

	// Round up, divide by 80 and add 32
	answer += 40;
	return (answer / 80) + 32;
}


/*------------------------------------------------------------------
 * kpa_to_psi()
 *
 * Pressures taken from J1939 communication channel are stored as 
 *  kPa x 16.
 * Convert the passed pressure in kPa x 16 to a pressure in PSI x 10
 *-----------------------------------------------------------------*/

s16 kpa_to_psi(s16 pressure)
{
	s32 answer;

	// 1kPa = 0.1450377PSI
	// Multiply by 0.14 * 10 * 2^18 / 16
	answer = pressure * 23756;
	
	answer += 0x20000;

	return (answer >> 18);
}

/*-----------------------------------------------------------------------
 * scale_pressure()
 *
 * Convert a pressure reading from the hardware (x 4) to the range of the  
 *  transducer used.  The reading and the range are inputs.
 * Pressure transducers deliver 0.5V - 4.5V for zero and full scale.
 * They are divided on-board by 16.9k / (16.9k + 8.39k) = 169/253 = 0.668.
 *  which gives 0.334V - 3V
 * The pressure reading passed in is right justified 12 bit A/D value 
 *  summed four times: 3.3V = 0x3FFD
 * The zero PSIg offset is 1658, full scale is 14921.
 * 		so:
 *			P1: 1658, 0		P2: 14921, Px
 *		where:
 *			Px = pressure_range
 *	 		M = (Px - 0) / (14921 - 1658) = Px / 13263
 *			B = y - Mx = 0 - (Px/13263 * 1658) = -Px * (1658 / 13263)
 *		for any x:
 *			y = Mx + B = ((Px/13263) * x) - Px(1658/13263)
 *			  = Px * (x-1658) / 13263 
 *		and Px cannot be negative
 *----------------------------------------------------------------------*/

u16 scale_pressure(u16 analog_value, u16 pressure_range)
{
	u32 answer;

	if (analog_value <= 1658)
		return 0;
	else
	{
		answer = analog_value - 1658;

		answer *= pressure_range;

		// Round up and divide
		answer += (13263 / 2);
		return (answer / 13263);
	}
}



/*---------------------------------------------------------
 * update_analogs()
 *
 * Called from the foreground task no more often than once 
 *  every 160ms to update the global analog values.
 * Analog inputs are each updated every 100ms and consist 
 *  of a four sample sum so it takes 400ms to fully respond
 *  to a step change.
 *--------------------------------------------------------*/
 
void update_analogs(void)
{
	board_temperature = temp16_lookup[analog_input[AIN_BOARD_TEMP].average >> 5];

	/*-------------------------------------------------------------------
	 * Battery is fed to an ADC channel through a 16.9k then 1.8k divider
	 * The .average is four summed readings and we want our answer to be 
	 *  V x 10.  [ 1.8 / (16.9 + 1.8) ] * 34.283V = 3.3V
	 * 
	 *		P1: 0, 0	P2: 0x3FFC (16380), 343
	 *		M = 343 / 16380 = 0.02094 , B = 0
	 *------------------------------------------------------------------*/

	// Multiply by (0.02094 * 2^16), round up and divide back down by 2^16
	battery_voltage = (((u32)analog_input[AIN_BATTERY_V].average * 1372) + 0x8000) >> 16;

	/*------------------------------------------------------------
	 * Set a flag describing the alternator charge voltage with 
	 *  some hysteresis
	 * This is really just battery voltage when we are not running
	 *-----------------------------------------------------------*/

	if (battery_voltage > 260)
		alternator_charge_ok = TRUE;
	else if (battery_voltage < 255)
		alternator_charge_ok = FALSE;
    
    transducer_pressure = scale_pressure(analog_input[AIN_TRANS_PRES_P].average, pressure_range_input);

    
    
}