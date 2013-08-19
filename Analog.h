/*****************************************************************
 * Analog.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * On-chip Analog to Digital Convertor (ADC) and Digital to Analog
 *  Convertor (DAC) interface defines and prototypes
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 *****************************************************************/

#ifndef __ANALOG_H
#define __ANALOG_H


// Regulation system pressure transducer full scale value
//  in PSI x 10
#define TRANS_PRES_PT_VAL				3000

// Define indexes into analog_input[]
// The ones above (lower numbered) NUMBER_MULTIPLEXED have corresponding 
//  entry in multiplexer_setup[]
// If one of the multiplexed inputs is used for a digital input, 
//  we allow the analog input state machine to read it as an analog 
//  value then compare it to a threshold when filling digital_input[]
typedef enum
{
AIN_DISCH_T,
AIN_SEP_TANK_T,
AIN_FUEL_LEVEL,
AIN_AMBIENT_T,
AIN_AIREND_OIL_P,
AIN_START_CONTROL,
AIN_REGEN_CONTROL,
NUMBER_MULTIPLEXED,						// Placeholder, not a real channel
	AIN_BATTERY_V = NUMBER_MULTIPLEXED,		// To prevent skipping an element
	AIN_TRANS_PRES_P,                       //AIN8 - PRESSURE TRANSDUCER
	AIN_SEP_TANK_P,
	AIN_BOARD_TEMP,
AIN_STARTNOW_SW,
	NUMBER_ANALOG_INPUTS
}analog_index_def;


typedef struct
{
	u8 index;
	s16 history[4];
	s16 average;
}analog_input_def;


// Define indexed into raw_analog[]
// The analog channels are converted continuously and moved into raw_analog[] 
//  by the DMA controller without any filtering or demultiplexing.
// For non-multiplexed analog channels, these can be used and provide better 
//  response
// There is an on-board analog multiplexer that selects one of seven 
//  analog inputs for our on-chip ADC.  They can be read directly 
//  and through a RTD amplifier by two of our ADC channels.
// Only the used ADC channels are converted and stored.
// Define the list of the used ADC channels, knowing that ADC channel6 
//  and channel7 need to be de-multiplexed into their individual signals
enum
{
	ADC_BOARD_TEMP,				// On-board thermister - ADC1 Ch0
	ADC_BATTERY_V,				// Battery voltage monitor - ADC1 Ch1
	ADC_CURRENT_2,				// Current sense at LS2 (AIN9) - ADC1 Ch2
	ADC_CURRENT_1,				// Current sense at LS1 (AIN8) - ADC1 Ch3
	ADC_MULTIPLEXER,			// One of seven analog inputs - ADC Ch6
	ADC_MUX_RTD,				//  " through an RTD amplifier - ADC Ch7
	ADC_AIN9,					// Analog input #9 at LS2 - ADC1 Ch14
	ADC_AIN8,					// Analog input #8 at LS1 - ADC1 Ch15
	ADC_DIN2,					// Input #11 is called DIN2 but can be analog - ADC1 Ch10
	ADC_DIN3,					// Input #12 is called DIN3 but can be analog - ADC1 Ch11
	NUMBER_OF_ADC
};


#define ADC_REG_PRESSURE		ADC_AIN8
#define ADC_SEP_PRESSURE		ADC_AIN9

extern u16 raw_analog[];
extern u16 battery_voltage;
extern u16 cooldown_count;
extern s16 ambient_temperature;
extern s16 deg_c16_to_f(s16);
extern s16 kpa_to_psi(s16);

extern u16 startnow_sw_deounce;
extern u16 pressure_threshold;
extern u16 transducer_pressure;



extern analog_input_def analog_input[];
extern bool alternator_charge_ok;

extern void read_raw_analogs(void);
extern void update_analogs(void);

#endif