/*******************************************************************
 * AStuff.c
 * 
 * Doosan Infracore Portable Power STM32 embedded systems
 * AutoStart module for TITAN based compressors
 * Common routines used by ST9 AutoStart modules ported 
 * to STM32F10x
 *
 * 6-16-2011	RPE		Initial code taken from Wedge AutoStart
 * 2-28-2013    CLA     V0.01  Edited code for ST9 to STM32F10x port
 *******************************************************************/

#include "stm32f10x.h"  //Replaces <sys\st92f150\st92f150.h> with stm32f10x header
#include "define.h" //define.h copied from ST9 project directory
#include "Timer.h"  //Changed from comTimers.h used in ST9 project
#include "IOdata.h" //Changed from iodata.h used in ST9 project
#include "E3prom.h" //Changed from eeprom.h used in ST9 project
#include "Serial.h" //Changed from serial.h used in ST9 project 
#include "CanCommands.h" //Changed from ComCan.h used in ST9 project
#include "canbus.h" //Changed from "Canbus.h" used in ST9 project
#include "CanPackets.h"
#include "AStuff.h" //Reference to AStuff.h used in ST9 project
#include "Analog.h" //Addrd reference to Analog functions STM32, 3/04/13, CLA

// Trigger holds the destination address of a pending identity
//  packet transmission
u8 transmit_identity;

/*-------------------------------------------------------------
 * update_autostart_state()
 *
 * Called continuously from the main loop to update our state 
 *  variable as needed and report it to the machine controller.
 * The machine controller sends it's status to us every five 
 *  seconds and we update our state accordingly.
 * We have two inputs that might activate asynchronously with 
 *  that status communication.  Check those inputs and take 
 *  appropriate action so we don't wait for the next 
 *  communication to have effect
 *-------------------------------------------------------------*/

void update_autostart_state()
{
	static u32 state_time;
    static u16 comm_delay = SECONDS(2);

	static master_command_def previous_command = NO_COMMAND;

	master_command_def present_command = NO_COMMAND;

	if (auto_enable_sw == FALSE)
	{
		// We are in Manual mode
		asState = AS_IDLE;
	}
	else
	{
		switch (controllerMode)
		{
			case AUTOSTARTING:
				asState = AS_STARTUP;
				break;

			case ENG_START:
			case REV_ENGINE1:
			case WAIT_PUMP_UP:
			case WAIT_INLET_CLOSE:
				asState = AS_STARTING;
				break;

			case RUN_UNLOADED:
				state_time = system_timer;
				asState = AS_RUNNING;
				break;

			case REV_ENGINE2:
				asState = AS_LOADED;
				break;

			case PRE_LOADED:
			case RUN_LOADED_LO:
			case RUN_LOADED_HI:
				if (asState != AS_COOLDOWN)
					asState = AS_LOADED;
				break;

			case MACHINE_DOWN:
				asState = AS_ERROR;
				bottom_red_led = LED_ON;
				break;

//			case POWERUP:
//			case AUTOSTOPPING:
//			case AUTOSTOPPED:
//			case STATE_COOLDOWN:
//			case VERIFY:
			default:

				asState = AS_STOPPED;

			
                if (startnow_sw)
                    present_command = START_ENGINE;
               
				break;
		}
    }

	switch (asState)
	{
		case AS_LOADED:

            //Check for auto init switch state 
            //We want to add logic that checks for use of this switch, or the use
            //of the pressure transducer readout
        
            //The TITAN controller will set the pressure threshold
            //We need to receive this on the CAN bus and place in a local variable
            //for use as the threshold for the pressure transducer to provide a cooldown
            //
        
            //ram_novram.autostop_enabled is to be set by a viewport variable
        
        
            if ((startnow_sw == FALSE) && (ram_novram.autostop_enabled == TRUE))
            {
                // The pressure is up and stop is enabled
                state_time = system_timer;
                asState = AS_COOLDOWN;
            }
            
        
			break;

		case AS_COOLDOWN:
            
            if (startnow_sw)
                asState = AS_LOADED;
            else
            {
                cooldown_count = (u16)(system_timer - state_time);

                //SECONDS is a compile-time macro, only.
                if (cooldown_count >= ram_novram.cooldown_time)
                {
                    present_command = STOP_ENGINE;
                }
            }
			break;

		default:
			break;
	}

	if ( (present_command == previous_command) && comm_delay )
	{
		// There is no new command to issue and it is
		//  not time to send a "communication alive" packet
		if (previous_timer != system_timer)
        {
            previous_timer = system_timer;
            comm_delay--;
        }
    
    
    }
    else if (can_buffer_state == BUFFER_IDLE)
    {
        can_buffer_state = BUFFER_READY_TO_SEND;     
        comm_delay = SECONDS(2);

        // Put the command in the CAN message buffer
        setup_command(present_command);
        previous_command = present_command;
    }
}


/*------------------------------------------------------------
 * update_ambient_temp()
 *
 * Called every 160ms to read the ambient temperature sensor 
 * Update the global ambient_temp
 * Read a new value from the temperature sensor analog input 
 *  channel and average it to smooth the reading
 * An 8 bit sample is read and summed with the most recent 
 *  15 samples to get a 16 sample sum
 * An int magnitude is stored having maximum of 0x0ff0
 *-----------------------------------------------------------*/

void update_ambient_temp(void)
{
	static byte index,
				array[16];

	byte count;

	if (index >= 15)
		index = 0;
	else
		index++;

	// Get the analog value from the hardware A/D channel 14
	//spp(62);		  			// ADC data registers 8-15 page
	//array[index] = AD_D14HR;	// Channel 14 high data register

    array[index] = analog_input[AIN_AMBIENT_T].average;
    
	// Update the EEPROM values for min/max operational temperature
	//min_max_ambient(AD_D14HR);

	ambient_temperature = 0;

	for (count = 0; count < 16; count++)
		ambient_temperature += array[count];
}


/*-----------------------------------------------------
 * update_battery_voltage()
 *
 * Called every 160ms to read the input power (battery) 
 *  voltage and average it to smooth the reading
 * An 8 bit sample is summed with the most recent 15 
 *  samples to get a 16 sample sum
 * An int magnitude is stored having maximum of 0x0ff0
 *----------------------------------------------------*/

void update_battery_voltage(void)
{
	byte count;

	static byte index,
				array[16];

	if (index >= 15)
		index = 0;
	else
		index++;

	// Get the analog value from the hardware A/D - raw analog input
    array[index] =  analog_input[AIN_BATTERY_V].average;
    
	battery_voltage = 0;

	for (count = 0; count < 16; count++)
		battery_voltage += array[count];
}

void turn_leds_off(void)
{
	middle_yellow_led = LED_OFF;
	bottom_green_led = LED_OFF;
    bottom_red_led = LED_OFF;
}

void turn_leds_on(void)
{
	middle_yellow_led = LED_ON;
	bottom_green_led = LED_ON;
	bottom_red_led = LED_ON;	
}

/*----------------------------------------------------------
 * perform_self_check()
 *
 * Run a hardware self check continuously until some 
 *  problem is detected.
 * A passing board turns the on-board LEDs on, on at a time, 
 *  for equal times.
 * If a problem is detected, we return and the LEDs will be
 *  activated as normal, indicating failed hardware (if the
 *  verification cable is connected).
 * If the verification version of this code is installed, 
 *  the calling routine does not modify the PCB state so the
 *  failing configuration persists, to troubleshoot the PCB.
 * Assume power supply is 15V.
 *---------------------------------------------------------*/

void perform_self_check(void)
{
	bool first_pass = TRUE;

	byte state_index = 0,
		 timer_hold;
	
	byte led_count;

	// All available analog inputs are used input for this test
	// They may be initialized differently so we set them 
	//  for this test and then change them back afterward
	// Presently, we only allow analog or TTL digital inputs which 
	//  affect only control register #1 of PORT7
	//spp(3);					// Ports 4, 5, 6, 7 control registers page
	//P7C1R = 0xFF;			// Set all for analog

	while (TRUE)
	{ 
		if ( timer_hold != (system_timer >> 2) )
		{
			timer_hold = system_timer >> 2;

			// A new state is run every 40ms
			switch (state_index)
			{
				case 0:

					// This is the entry state 
					// We may not get a full 40ms here the first time

					break;

				case 1:
				
					// RT1 has a 2k to ground giving 1.66V 
					//spp(62);
					//if ( (AD_D9HR < 0x51) || (AD_D9HR > 0x59) )
					//{
					//	if (first_pass == FALSE)
					//		show_diagnostic_string("Problem with RT1", AD_D9HR);
                    //
					//	return;
					//}

					// Set POUT1 high, reset the other outputs low
					//P2DR = 0x80;

					// Turn the fuel sensor power off
					//P1DR = 0;

					break;

				case 2:

					// The serial port is looped back
					// Send a character out the serial port and
					//  verify that the correct character is received
			 		if ( verify_serial_character() == FALSE )
			 		{
			 			show_diagnostic_string("Problem with TxD or RxD", 0);

			 			return;
			 		}

					break;

				case 3:

					// POUT1 is on
					// SW1 is driven by POUT1 with a 178ohm pull down in the cable
					// ALT-S, SW2 and SW4 should be low

					//  ALT-S is inverted on the board
					//if ( (P2DR & 0x01) == 0 )
					//{
					//	show_diagnostic_string("ALT-S should be low while POUT1 is high", 0);
                    //
					//	return;
					//}

					//if ( (P0DR & 0x80) == 0 )
					//{
					//	show_diagnostic_string("POUT1 should drive SW1 high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x40)
					//{
					//	show_diagnostic_string("SW2 should be low while POUT1 is high", 0);
                    //
					//	return;
					//}

					//if (P2DR & 0x02)
					//{
					//	show_diagnostic_string("SW4 should be low while POUT1 is high", 0);
                    //
					//	return;
					//}

					break;

				case 4:

					// FL1 should be low since the fuel sensor power is off
					//spp(62);					
					//if (AD_D11HR > 4)
					//{
					//	show_diagnostic_string("FL1 should be low while unpowered", AD_D11HR);
                    //
					//	return;
					//}

					// Set POUT2 high, reset the other outputs low
					//P2DR = 0x40;

					break;

				case 5:

					// SW3 is tied to RT2 input giving 0.674V on ADC13
					//  and 1.35V on ADC8
					//spp(62);
					//if ( (AD_D13HR < 0x1F) || (AD_D13HR > 0x27) )
					//{
					//	show_diagnostic_string("SW3 should deliver 0.675V", AD_D13HR);
                    //
					//	return;
					//}
					
					//if ( (AD_D8HR < 0x41) || (AD_D8HR > 0x49) )
					//{
					//	show_diagnostic_string("RT2 should deliver 1.35V", AD_D8HR);
                    //
					//	return;
					//}

					break;

				case 6:

					// POUT2 is on
					// SW4 should be high
					// SW1, SW2, ALT-S should be low

					//  ALT-S is inverted on the board
					//if ( (P2DR & 0x01) == 0 )
					//{
					//	show_diagnostic_string("ALT-S should be low while POUT2 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x80)
					//{
					//	show_diagnostic_string("SW1 should be low while POUT2 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x40)
					//{
					//	show_diagnostic_string("SW2 should be low while POUT2 is high", 0);
                    //
					//	return;
					//}

					//if ( (P2DR & 0x02) == 0 )
					//{
					//	show_diagnostic_string("POUT2 should drive SW4 high", 0);
                    //
					//	return;
					//}

					//P1DR = 0x04;	// Turn the fuel sensor power on

					break;

				case 7:

					// PT1 is connected to +5V through 20k
					//spp(62);
					//if ( (AD_D15HR < 0x81) || (AD_D15HR > 0x89) )
					//{
					//	show_diagnostic_string("PT1 should deliver 2.6V", AD_D15HR);

                    //        return;
					//}

#ifdef NEVER
					// Ambient Temperature
					// Data: 0x20 @ +80C, 0xF0 @ -20C
					//spp(62);
					//if ( (AD_D14HR < 0x18) || (AD_D14HR > 0xF0) )
					//{
					//	show_diagnostic_string("Ambient Temp should deliver 2.5V (70F)", AD_D14HR);

					//	return;
					//}
#endif					
					//spp(62);
					//min_max_ess_temperature(AD_D14HR);

					// Set POUT3 high, reset the other outputs low
					//P2DR = 0x20;

					break;

				case 8:

					// POTENtiometer IN has a 2k to ground
					//spp(62);
					//if ( (AD_D10HR < 0xA7) || (AD_D10HR > 0xAF) )
					//{
					//	show_diagnostic_string("POTEN IN should deliver 3.33V", AD_D10HR);
                    //
                    //    return;
					//}

					// FL1 has a 300 ohm to ground giving 2.37V on ADC11 
					//  when the fuel sensor power is on
					// Assume 15V power input
					//spp(62);
					//if ( (AD_D11HR < 0x68) || (AD_D11HR > 0x8C) )
					//{
					//	show_diagnostic_string("FL1 should be powered", AD_D11HR);
                    //
					//	return;
					//}

					break;

				case 9:

					// POUT3 is on
					// SW4 should be high
					// SW1, SW2, ALT-S should be low

					//  ALT-S is inverted on the board
					//if ((P2DR & 0x01) == 0 )
					//{
					//	show_diagnostic_string("ALT-S should be low while POUT3 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x80)
					//{
					//	show_diagnostic_string("SW1 should be low while POUT3 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x40)
					//{
					//	show_diagnostic_string("SW2 should be low while POUT3 is high", 0);
                    //
					//	return;
					//}

					//if ( (P2DR & 0x02) == 0 )
					//{
					//	show_diagnostic_string("POUT3 should drive SW4 high", 0);
                    //
					//	return;
					//}

					break;

				case 10:

					// Set POUT4 high, reset the other outputs low
					//P2DR = 0x10;

					break;

				case 11:

					// POUT4 is on
					// SW2 is driven by POUT4 with a 178ohm pull down in the cable
					// SW1, SW4, ALT-S should be low

					//  ALT-S is inverted on the board
					//if ( (P2DR & 0x01) == 0 )
					//{
					//	show_diagnostic_string("ALT-S should be low while POUT4 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x80)
					//{
					//	show_diagnostic_string("SW1 should be low while POUT4 is high", 0);
                    //
					//	return;
					//}

					//if ( (P0DR & 0x40) == 0 )
					//{
					//	show_diagnostic_string("POUT4 should drive SW2 high", 0);
                    //
					//	return;
					//}

					//if (P2DR & 0x02)
					//{
					//	show_diagnostic_string("SW4 should be low while POUT4 is high", 0);
                    //
					//	return;
					//}

					break;

				case 12:

					// Battery sense provides 2.58V on ADC12
					//spp(62);
					//if ( (AD_D12HR < 0x73) || (AD_D12HR > 0x97) )
					//{
					//	show_diagnostic_string("Problem with Battery Sense", AD_D12HR);
                    //
					//	return;
					//}

					// Verify the CAN hardware has exited initialization 
					//  mode and is not in sleep mode
					//spp(48); 
					//if ( (CAN0_CTRL_CMSR & 0x03)
					// Check the bus off flag
					//	|| (CAN0_CTRL_CESR & 0x04) )
					//{
					//	show_diagnostic_string("Problem with CAN", 0);
                    //
					//	return;
					//}

					// Set POUT5 high, reset the other outputs low
					//P2DR = 0x08;

					break;

				case 13:

					// POUT5 is on
					// ALT-S should be high
					// SW1, SW2, SW4 should be low

					//  ALT-S is inverted on the board
					//if (P2DR & 0x01)
					//{
					//	show_diagnostic_string("POUT5 should drive ALT-S high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x80)
					//{
					//	show_diagnostic_string("SW1 should be low while POUT5 is high", 0);

					//	return;
					//}

					//if (P0DR & 0x40)
					//{
					//	show_diagnostic_string("SW2 should be low while POUT5 is high", 0);
                    //
					//	return;
					//}

					//if (P2DR & 0x02)
					//{
					//	show_diagnostic_string("SW4 should be low while POUT5 is high", 0);
                    //
					//	return;
					//}

					break;

				case 14:

					/*---------------------------------------------
					 * Light the LEDs in a distinctive pattern that 
					 *  indicates self check is running and has not 
					 *  detected a problem
					 *--------------------------------------------*/

					led_count++;

					switch (led_count)
					{
						case 0:
							// This is never hit
							break;

						case 1:
							// Select port 6 control registers page
							//spp(3);
							//P6DR = 5;		// Turn on the top (yellow) LED
                            TURN_ON(LED_YELLOW);
                            
                        
							break;

						case 2:
							// Select port 6 control registers page
							//spp(3);
							//P6DR = 9;		// Turn on the middle (green) LED
                            TURN_ON(LED_GREEN);
                        
							break;

						case 3:
							// Select port 6 control registers page
							//spp(3);
							//P6DR = 0x0C;	// Turn on the bottom (red) LED
                            TURN_ON(LED_RED);
                        
						default:
							led_count = 0;
							break;
					}			

					// Set POUT6 high, reset the other outputs low
					//P2DR = 0x04;

					break;

				case 15:

					// POUT6 is on
					// ALT-S should be high
					// SW1, SW2, SW4 should be low

					//  ALT-S is inverted on the board
					//if (P2DR & 0x01)
					//{
					//	show_diagnostic_string("POUT6 should drive ALT-S high", 0);
                    //    
					//	return;
					//}

					//if (P0DR & 0x80)
					//{
					//	show_diagnostic_string("SW1 should be low while POUT6 is high", 0);
                    //
					//	return;
					//}

					//if (P0DR & 0x40)
					//{
					//	show_diagnostic_string("SW2 should be low while POUT6 is high", 0);
                    //
					//	return;
					//}

					//if (P2DR & 0x02)
					//{
					//	show_diagnostic_string("SW4 should be low while POUT6 is high", 0);
                    //    
					//	return;
					//}

					first_pass = FALSE;

					break;
                    
                    default:
                    break;
			}

			state_index = (++state_index & 0x0F);
		}

		TICKLE_THE_DOG
	}
    
}
