
/****************************************************************
 * IOdata.c
 * module
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors 
 * 
 * Port initialization and interface functions
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "define.h"
#include "Timer.h"
#include "CanCommands.h"
#include "canbus.h"
#include "E3prom.h"
#include "Analog.h"
#include "SerialBuffer.h"
#include "IOdata.h"
#include "Serial.h"


led_control_def top_red_led,
				middle_yellow_led,
				bottom_green_led,
                bottom_red_led;

autostart_state asState;

bool auto_enable_sw;
bool startnow_sw;

bool startnow_sw_bool;
switch_state STN_ACTSW_STATE;

// There are seven input channels that are multiplexed onto a single 
//  analog input pin.  They can be used to read a switch input, however 
//  they are read as analog signals and compared to a threshold as needed.
switch_state digital_input[NUMBER_DIGITAL_INPUTS];
pressure_mode_val pressure_mode_current;

/*-----------------------------------------------------------
 * initialize_ports()
 *
 * Configure all unused GPIO port pins as floating input mode 
 *  to reduce power consumption and increase the device 
 *  immunity against EMI/EMC
 *----------------------------------------------------------*/

void initialize_ports(void)
{
	/*------------------------------------------------------------
	 * PORT A
	 * Eight channels of analog input, UART1 Tx/Rx and CAN Tx/Rx
	 *		PA.0 - On-board temperature thermister analog input
	 *		PA.1 - Input power voltage monitor analog input
	 *		PA.2 - Scalable current sense 2 (at AIN9) analog input
	 *		PA.3 - Scalable current sense 1 (at AIN8) analog input
	 *		PA.4 - DAC1 analog output #1 at DIN2
	 *		PA.5 - DAC2 analog output #2 at DIN3
	 *		PA.6 - Multiplexed analog / digital inputs AIN1..7
	 *		PA.7 - Multiplexes RTD inputs from AIN1..7
	 *		PA.8 -
	 *		PA.9 - USART1 RS232 transmit data output
	 *		PA.10 - USART1 RS232 receive data input
	 *		PA.11 - CAN J1939 receive data input
	 *		PA.12 - CAN J1939 transmit data output
	 *		PA.13 -
	 *		PA.14 -
	 *		PA.15 -
	 *-----------------------------------------------------------*/

	// All lower bits are analog inputs.
    // Bits 4 and 5 are push-pull GP outputs 2MHz max.
	// The DAC bits are connected to the DAC outputs
	GPIOA->CRL = 0x44224444;

	// Bits 9 and 12 are alternate function push-pull outputs with 2MHz maximum
	// All other upper bits are general purpose floating digital inputs
	GPIOA->CRH = 0x444A44A4;

	// Write all outputs low
	GPIOA->ODR = 0;

	/*------------------------------------------------------------
	 * PORT B
	 *		PB.0 - 
	 *		PB.1 - 
	 *		PB.2 - BOOT1 input signal tied low
	 *		PB.3 - JTAG TDO transmit data output
	 *		PB.4 - JTAG !JRTS active low restart input
	 *		PB.5 - FREQ frequency/PWM/digital input from DIN1
	 *		PB.6 - Pull down dry contact terminator enable at DIN1
	 *		PB.7 - Pull down dry contact terminator enable at DIN2
	 *		PB.8 - 
	 *		PB.9 - 
	 *		PB.10 - Control output signal to High Side Driver 3
	 *		PB.11 - Control output signal to High Side Driver 4
	 *		PB.12 - AIN9 current sense gain #2 chip select
	 *		PB.13 - SPI2 bus clock output to current sense gain
	 *		PB.14 - AIN9 current sense averaging filter enable
	 *		PB.15 - SPI2 bus serial data out to current sense gain
	 *-----------------------------------------------------------*/

	// Bit 6 and 7 are general purpose push-pull digital outputs with 2MHz maximum
	// All other lower bits are general purpose floating digital inputs
	//GPIOB->CRL = 0x22444444;

	// Bits 8 and 9 are general purpose floating digital input
	// Bit 12 is general purpose open collector digital output with 10MHz maximum
	// Bits 13 and 15 are alternate function open collector outputs with 10MHz maximum
	// All other upper bits are general purpose push-pull digital outputs with 2MHz maximum
	//GPIOB->CRH = 0xD2D52244;

	// Set AIN9 current sense chip select
	// Write all other outputs low
	//GPIOB->ODR = 0x1000;
    
    
    	// Bit 6 is general purpose push-pull digital output with 2MHz maximum
	// All other lower bits are general purpose floating digital inputs
	GPIOB->CRL = 0x42444444;				// 0100 0010 0100 0100 0100 0100 0100 0100

	// Bit 9 is general purpose floating digital input
	// All other upper bits are general purpose push-pull digital outputs with 2MHz maximum
	GPIOB->CRH = 0x22222242;				// 0010 0010 0010 0010 0010 0010 0100 0010

	// Write all outputs low
	GPIOB->ODR = 0;

	/*---------------------------------------------------------
	 * PORT C
	 *		PC.0 - Analog / digital input DIN2
	 *		PC.1 - Analog / digital input DIN3
	 *		PC.2 -
	 *		PC.3 -
	 *		PC.4 - Analog / digital input AIN9
	 *		PC.5 - Analog / digital input AIN8
	 *		PC.6 - 
	 *		PC.7 - 
	 *		PC.8 - Control output signal to High Side Driver 7
	 *		PC.9 - Control output signal to High Side Driver 8
	 *		PC.10 - Active low Force OFF output to RS232 driver
	 *		PC.11 - Force ON output to RS232 driver
	 *		PC.12 - Active low Enable output to RS232 driver
	 *		PC.13 - Analog multiplexer select0 digital output
	 *		PC.14 - Analog multiplexer select1 digital output
	 *		PC.15 - Analog multiplexer select2 digital output
	 *--------------------------------------------------------*/

	// Bits 0, 1, 4 and 5 are analog input
	// All other lower bits are general purpose floating digital inputs
	//GPIOC->CRL = 0x44444444;
    
    // Bits 6 and 7 are general purpose floating digital inputs
	// Bit 5 is general purpose push-pull digital output with 2MHz maximum
	// All other lower bits are analog input
	GPIOC->CRL = 0x44000000;

	// All upper bits are general purpose push-pull digital outputs with 2MHz maximum
	GPIOC->CRH = 0x22222222;

	// Set RS232 Force OFF FALSE, RS232 Force ON TRUE, RS232 Enable TRUE
	// Write all other outputs low
	GPIOC->ODR = 0x00000C00;				// 0000 0000 0000 0000 0000 1100 0000 0000

	/*----------------------------------------------------------------
	 * PORT D
	 *		PD.0 - Active low Invalid input from RS232 driver
	 *		PD.1 -
	 *		PD.2 - 
	 *		PD.3 - 
	 *		PD.4 - 
	 *		PD.5 - 
	 *		PD.6 - 
	 *		PD.7 - 
	 *		PD.8 -
	 *		PD.9 -
	 *		PD.10 -
	 *		PD.11 -
	 *		PD.12 - Control output signal to High Side Driver 6
	 *		PD.13 - Control output signal to High Side Driver 5
	 *		PD.14 - Power Down request input signal from ON switch
	 *		PD.15 - Power Hold output, low to shutdown if PD.14 is low
	 *---------------------------------------------------------------*/

	// All lower bits are general purpose floating digital inputs
	GPIOD->CRL = 0x44444444;

	// Bits 12, 13 and 15 are general purpose push-pull digital outputs with 2MHz maximum
	// All other upper bits are general purpose floating digital inputs
	GPIOD->CRH = 0x24224444;

	// Set all outputs low
	GPIOD->ODR = 0;

	/*---------------------------------------------------------------
	 * PORT E
	 *		PE.0 - Pull down dry contact terminator enable at DIN3
	 *		PE.1 - 
	 *		PE.2 - Green LED control output, on when high
	 *		PE.3 - Yellow LED control output, on when high
	 *		PE.4 - Red LED control output, on when high
	 *		PE.5 - Pull up terminator enable to multiplexed analogs
	 *		PE.6 - Pull down terminator enable to multiplexed analogs
	 *		PE.7 - AIN8 current sense gain #1 chip select
	 *		PE.8 - AIN8 current sense averaging filter enable
	 *		PE.9 - AIN8 active low LS output / I sense input enable
	 *		PE.10 - Pull down dry contact terminator enable at AIN8
	 *		PE.11 - AIN9 active low LS output / I sense input enable
	 *		PE.12 -
	 *		PE.13 - Control output signal to High Side Driver 1
	 *		PE.14 - Control output signal to High Side Driver 2
	 *		PE.15 - Pull down dry contact terminator enable at AIN9
	 *--------------------------------------------------------------*/

	// Bit 1 is a general purpose floating digital input
	// Bit 7 is general purpose open collector digital output with 10MHz maximum
	// All other lower bits are general purpose push-pull digital outputs with 2MHz maximum
	//GPIOE->CRL = 0x22222242;

	// Bit 12 is a general purpose floating digital input
	// All other upper bits are general purpose push-pull digital outputs with 2MHz maximum
	//GPIOE->CRH = 0x22242222;
    
// Bit 1 is a general purpose floating digital input
// All other lower bits are general purpose push-pull digital outputs with 2MHz maximum
GPIOE->CRL = 0x22222242;				// 0010 0010 0010 0010 0010 0010 0100 0010

// Bit 12 is a general purpose floating digital input
// All other upper bits are general purpose push-pull digital outputs with 2MHz maximum
GPIOE->CRH = 0x22242222;				// 0010 0010 0010 0100 0010 0010 0010 0010

	//Set PE9 high, all others low, keeps LS1 FET on, A8TRM FET off
	GPIOE->ODR = 0x00000200;
    

	// Set AIN8 current sense chip selsct
	// Turn off both low side drivers (active low control), turn on all three LEDs
	// Write all other outputs low
	//GPIOE->ODR = 0x00000A9C;				// 0000 0000 0000 0000 0000 1010 1001 1100

	/*----------------------------------------------------------
	 * Remap alternate functions for some peripheral connections
	 *---------------------------------------------------------*/

	// Remap TIM4 to CH1-PD12(POUT4), CH2-PD13(POUT3)
	// Remap TIM3 to CH2-PB5(FREQ)
	// Remap TIM2 to CH3-PB10(POUT8), CH4-PB11(POUT7)
	// Remap TIM1 to CH1-PE9(LS1), CH2-PE11(LS2), CH3-PE13(POUT2), CH4-PE14(POUT1)
	AFIO->MAPR = 0x00001BC0;				// rrrr r000 rrr0 0000 0001 1011 1100 0000

}


/*--------------------------------------------------------
 * update_leds()
 *
 * Called from the foreground task no more often than 10ms 
 *  to manage the three on-board LEDs
 * An LED can be on, off, or blinking.
 * If the foreground task is busy, we may be away longer 
 *  than 10ms.
 *-------------------------------------------------------*/

void update_leds(void)
{
	bool blink_state;

	blink_state	= system_timer & 0x40;

	if ( (top_red_led == LED_ON)
			|| ( (top_red_led == LED_BLINK) && blink_state )
			|| ( (top_red_led == LED_NEG_BLINK) && !blink_state) )
	{
		TURN_ON(LED_RED);
	}
	else
		TURN_OFF(LED_RED);

	if ( (middle_yellow_led == LED_ON)
			|| ( (middle_yellow_led == LED_BLINK) && blink_state )
			|| ( (middle_yellow_led == LED_NEG_BLINK) && !blink_state) )
	{
		TURN_ON(LED_YELLOW);
	}
	else
		TURN_OFF(LED_YELLOW);
	
	if ( (bottom_green_led == LED_ON)
			|| ( (bottom_green_led == LED_BLINK) && blink_state )
			|| ( (bottom_green_led == LED_NEG_BLINK) && !blink_state) )
	{
		TURN_ON(LED_GREEN);
	}
	else
		TURN_OFF(LED_GREEN);
    
    
}


/*--------------------------------------------------
 * get_input_state()
 *
 * Read one of the digital inputs and return TRUE if 
 *  the switch is closed, otherwise return FALSE
 *-------------------------------------------------*/

bool get_input_state(digital_index which_input)
{
	switch (which_input)
	{
		case DIN_FILTER:

			// Filter restriction is inverted on the board
			if(*(u8 *)DIN1_FREQ_INPUT == 0)
				return TRUE;

			break;

		case DIN_ENABLE_SWITCH:

			if (raw_analog[ADC_DIN2] > 0xA00)
				return TRUE;

			break;

		case DIN_POWER_SIGNAL:

			if (*(u8 *)POWER_SIGNAL)
				return TRUE;

			break;

		case DIN_UNIT_START:

			if (analog_input[AIN_START_CONTROL].average > 0x2400)
				return TRUE;

			break;

		case DIN_UNIT_LOAD:

			if (analog_input[AIN_START_CONTROL].average < 0x2F0)
				return TRUE;

			break;

		case DIN_UNIT_STOP:

			//if (raw_analog[ADC_DIN2] > 0xA00)
			//	return TRUE;

			break;

		case DIN_MANUAL_REGEN:

			if (analog_input[AIN_REGEN_CONTROL].average < 0x2F0)
				return TRUE;

			break;

		case DIN_REGEN_INHIBIT:

			if (analog_input[AIN_REGEN_CONTROL].average > 0x3800)
				return TRUE;

			break;
            
        case DIN_STARTNOW_SW:
            
            if (analog_input[AIN_STARTNOW_SW].average > 0x3000)
                return TRUE;
            
            break;
            
	}

	return FALSE;
}


/*---------------------------------------------------------------------------
 * update_input_state()
 *
 * Called repeatedly for each digital input to debounce and set the new state
 *--------------------------------------------------------------------------*/

void update_input_state(digital_index which)
{
	switch_state new_state;
	bool pin_state;

	pin_state = get_input_state(which);

 	switch (digital_input[which])
	{
		case SW_OPEN:

			// If the pin is high, the switch is closing,
			//  if not, leave it open
			if (pin_state)
				new_state = SW_CLOSING_A;
			else
				return;

			break;

		case SW_CLOSING_A:

			// If the pin is high, the switch is still closing,
			//  if not, it is open
			if (pin_state)
				new_state = SW_CLOSING_B;
			else
				new_state = SW_OPEN;

			break;

		case SW_CLOSING_B:

			// If the switch is still closing, it is closed
			if (pin_state)
				new_state = SW_CLOSED;
			else
				new_state = SW_OPEN;

			break;

		case SW_CLOSED:

			// If the pin is low, the switch is opening, 
			//  if not, leave it closed
			if (!pin_state)
				new_state = SW_OPENING_A;
			else
				return;

			break;

		case SW_OPENING_A:

			// If the pin is low, the switch is still opening, 
			//  if not, it is closed
			if (!pin_state)
				new_state = SW_OPENING_B;
			else
				new_state = SW_CLOSED;

			break;

		case SW_OPENING_B:

			// If the switch is still opening, it is open
			if (!pin_state)
				new_state = SW_OPEN;
			else
				new_state = SW_CLOSED;

			break;
			
		default:
			// Just to get us back in range if something is awry
			new_state = SW_OPEN;
			break;
	}
	
	digital_input[which] = new_state;
}

/*-------------------------------------------------------------
 * Get the initial pressure_bool to set up the initial pressure 
 * switch polarity
--------------------------------------------------------------*/

void initialize_psw(void)
{
    if (ram_novram.pressure_sw_bool)
        STN_ACTSW_STATE = SW_OPEN;
    else
        STN_ACTSW_STATE = SW_CLOSED;
        
}


/*-------------------------------------------------------------
 * update_digital_in()
 *
 * Called no more often than once every 80ms from the main loop 
 *  to read and debounce the digital inputs.
 * We may be away for longer if the foreground is busy
 *------------------------------------------------------------*/

void update_digital_in(void)
{
	digital_index input_number;

	for (input_number = 0; input_number < NUMBER_DIGITAL_INPUTS; input_number++)
		update_input_state(input_number);
}

/*----------------------------------------------------
 * update_outputs()
 *
 * Update the indicator lights
 * We start with the status set to zero so all outputs 
 *  would be written low (OFF).
 * So we only need to set the corresponding bits when 
 *  the status dictates.
 *---------------------------------------------------*/

void update_outputs(void)
{
	// The "Alarm" horn
	if (asState == AS_STARTUP)
		TURN_ON(WARNING_ALARM);
    else
        TURN_OFF(WARNING_ALARM);

	// The "AutoStart Enabled" light
	if (auto_enable_sw && master_comm_wd)
        TURN_ON(AUTO_MODE_LAMP);
    else
        TURN_OFF(AUTO_MODE_LAMP);

	// The "Low Fuel" light
	//if (contStatus & 0x02)      //Originally, took a fault from main can controller
    //Using multiplexed analog input AIN2, 0x20 is arbitrary
	if (analog_input[AIN_FUEL_LEVEL].average < 0x20)    
        TURN_ON(LOW_FUEL_LAMP);
    else
        TURN_OFF(LOW_FUEL_LAMP);

	// The "Compressor Operating" light turned ON while loaded
    // Unsure how this read/determined - assume "can online" is close


   
}

void autostart_enable_switch(void)
{
	static u8 count;

	

	if (auto_enable_sw == FALSE)
	{
		// The AutoStart enable switch is OPEN
		// Make sure it stays closed for the debounce time 
		//  before declaring it CLOSED
		
		if (digital_input[DIN_ENABLE_SWITCH] == SW_CLOSED)
			count++;
		else
			count = 0;
			
		if (count >= AS_ENABLE_DEBOUNCE)   
			auto_enable_sw = TRUE;
	}
	else
	{
		// The AutoStart enable switch is CLOSED
		// Make sure it stays open for the debounce time 
		//  before declaring it OPEN
		
		if (digital_input[DIN_ENABLE_SWITCH] == SW_OPEN)
			count++;
		else
			count = 0;
			
		if (count >= AS_ENABLE_DEBOUNCE)
			auto_enable_sw = FALSE;
	}
}

/*Start Now State (startnow_state) determines if the engine startup is based
on a switch open/closed, or looking at the pressure threshold.  This determination
occurs every 160ms.

The open/closed state of the switch can be interchagned thru a settings, as well. */

void startnow_state(void)
{
    static u8 count;


    /*Check for pressure mode, or digital input mode*/
    if (ram_novram.ad_pressure_mode)
    {
    /*Pressure mode*/
    
        if (startnow_sw == FALSE)
        {
            
             if (transducer_pressure < ram_novram.pressure_threshold)
                count++;
             else 
                count = 0;
                
            if (count >= AS_ENABLE_DEBOUNCE)   
                startnow_sw = TRUE;
        }
        else
        {
            
            
            if (transducer_pressure >= ram_novram.pressure_threshold)
                count++;
            else 
                count = 0;
                
            if (count >= AS_ENABLE_DEBOUNCE)
                startnow_sw = FALSE;
        }

    
    }
    else
    {
    /*Digital switch input*/    
    /*Check for digital switch orientation -- Corning option*/
    
    if (ram_novram.pressure_sw_bool==FALSE)
        STN_ACTSW_STATE = SW_CLOSED;
    else if (ram_novram.pressure_sw_bool==TRUE)
        STN_ACTSW_STATE = SW_OPEN;
    
        /*Check for start now*/
        if (startnow_sw == FALSE)
        {
            
             if (digital_input[DIN_STARTNOW_SW] == STN_ACTSW_STATE)
                count++;
             else 
                count = 0;
                
             if (count >= AS_ENABLE_DEBOUNCE)   
                 startnow_sw = TRUE;
        }
        else
        {
         
            if (digital_input[DIN_STARTNOW_SW] != STN_ACTSW_STATE)
                count++;
            else 
                count = 0;
                
            if (count >= AS_ENABLE_DEBOUNCE)
                startnow_sw = FALSE;
        }

         
    }
    

}
