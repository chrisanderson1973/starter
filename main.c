/****************************************************************
 * main.c
 *
 * Doosan Infracore International Portable Power embedded systems
 * CORTEX TITAN AutoStart module controller for air compressors
 *  
 *
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "define.h"    
#include "Interrupt.h"
#include "IOdata.h"
#include "Serial.h"
#include "SerialBuffer.h"
#include "Timer.h"
#include "CanCommands.h"
#include "canbus.h"
#include "Analog.h"
#include "E3prom.h"
#include "Flash.h"
#include "ThermTables.h"
#include "AStuff.h"     


int main(void)
{
	// Define a trigger to delay using values read from the 
	//  hardware before input filtering stabilizes
	static bool inputs_valid = FALSE;

	static u16 hold_timer = 0,
               pass_counter = 0;

	SCB->AIRCR = APP_RESET_CONTROL;

	initialize_clocks();

	initialize_ports();

	TURN_ON(POWER_HOLD);

	// Setup USART1 for 9600 Baud, 1 stop bit, no parity, no handshake,
	//  and DMA multi-buffered input and output
	initialize_serial();

	initialize_timers();

	this_can_address = CAN_AUTOST_ADDR;   

	initialize_can();

    //Used to set the pressure switch polarity based on ram_novram pressure_sw_bool
    initialize_psw();

	// We keep the Flash Program and Erase Controller unlocked since we 
	//  use FLASH memory to emulate EEPROM
	unlock_FPEC();
    
	if ( (FLASH->OBR >> 10) != VALID_KEY)
	{
		// The Program Valid bytes have not been set
		// This must be the first time running after reprogramming

		// Transmit a packet to a possible programming master 
		//  to acknowledge a successful programming cycle
		transmit_proprietary(PCMD_HEARTBEAT, OB->Data1);

		// Wait for it to transmit (if it does)
		wait_for_tic(2);

		// Write the "Program Valid" flag and set program space write protection
		set_option_bytes(VALID_KEY);

		// Reset to make write protection active and get the BootLoader going
		software_reset();
	}

	initialize_analog_input();

	send_a_string("\r\n\n>>>>Doosan Autostart Module<<<<\r\n");
	send_a_string("\n\rPress ESC then 'm' for menu");

	// Read the block of non-volotile status values into RAM
	//  or initialize them if this is the first execution
	read_novram();

	programming_server.state = PRGM_NORMAL;

	/*--------------------------------------------------------
	 * The three on-board LEDs are turned on at initialization
	 * We set the LED control to OFF here but the state is not 
	 *  changed until update_leds() is called
	 *-------------------------------------------------------*/

	top_red_led= LED_OFF;
	middle_yellow_led = LED_NEG_BLINK;
	bottom_green_led = LED_BLINK;

	TICKLE_THE_DOG

	wait_some_seconds(1);

	while(TRUE)
    {
    	if ( (u16)system_timer != hold_timer )
    	{
	  		// We get here no more often than once every 10ms
			// Reload the present timer value for the next time around
			hold_timer = system_timer;

			// Count the number of times we get here so if we get busy, 
			//  we resume where we left off instead of skipping tasks
			pass_counter++;

			// Allow alerts and shutdowns after input values stabilize
			if (pass_counter > 50)
				inputs_valid = TRUE;

			// Each case is executed no more often than 160ms
	    	switch (pass_counter & 0x0F)
	    	{
            
            	case 0:

					/*---------------------------------------------
					 * Switch 1 evaluation - The Auto/Manual switch
					 *--------------------------------------------*/
	    		
					autostart_enable_switch();   //Complete

	    			break;

				case 1:
					update_outputs();            //Need input on RUN_LAMP logic
					break;

				case 2:
					update_ambient_temp();      
					break;

            
				case 3:
					update_leds();               //In IOdata.c
					break;

				case 4:
					update_health_timers();
					break;

	    		case 5:

					switch (programming_server.state)
					{
						case PRGM_DELAY1:
							programming_server.state = PRGM_DELAY2;
							break;

						case PRGM_DELAY2:
							programming_server.state = PRGM_READY;
							break;
					}

	    			break;

				case 6:

					// Calculate global values for temperatures and pressures 
					//  from the averaged values read from the hardware
					update_analogs();

					break;
                case 7:

                    startnow_state();

					break;
			}

			maintain_can_comm_wd();

			// Read a raw analog input value that the DMA transferred
			//  from the hardware to raw_analog[] and update the four sample 
			//  moving window sum in analog_input[].average
			read_raw_analogs();

			// Read and debounce the on-board digital inputs
			update_digital_in();

			// Serial command timer allows single character commands 
			//  for a limited time after after ESC is pressed
			if (command_timer)
				command_timer--;
            
       
            //Maintain timed can messages
			timed_can_messages();
		
            
            // Manage display of data screens
            access_info_screens();
         
		}

		// Manage receipt and transmission of CAN messages 
		//  and take appropriate action 
		check_can();

		check_serial();
        
        // Reprogram interrupt service
        check_reprogram_state();
 
  
        if (digital_input[DIN_POWER_SIGNAL] == SW_OPEN)
		{
			// Hold power on after power down request
			//  long enough to store NOVRAM

			write_novram();

			send_a_string("\r\n\nShutting down ");

			// Wait for outgoing messages to complete
			wait_for_buffer();

			TURN_OFF(POWER_HOLD);

			while(1);
		}
    
        update_autostart_state();
	
		TICKLE_THE_DOG
	}
}