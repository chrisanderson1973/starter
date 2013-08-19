/****************************************************************
 * Serial.c
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN AutoStart module controller for air compressors 
 *  
 * RS232 Serial Communications Interface maintenance routines 
 *
 * 2/27/2013 CLA V0.01	Initial code taken from Titan Compressor
 ****************************************************************/


#include "stm32f10x.h"
#include "common.h"
#include "Timer.h"
#include "E3prom.h"
#include "Flash.h"

#include "Analog.h"
#include "Interrupt.h"
#include "CanPackets.h"
#include "CanCommands.h"
#include "Canbus.h"
#include "SerialBuffer.h"
#include "Serial.h"
#include "AStuff.h"
#include "define.h"

//remove
#include "ThermTables.h"

u16 command_timer;


u8 test_index = 0;
s32 testVar[12];
void print_test_vars(void)
{
	u8 count;
	
	fill_a_string("\n\rTest Data: ");

	for (count=0; count < 10; count++)
	{
		fill_a_string("\n\r");
		fill_a_decimal(count);
		fill_a_string(": ");
		fill_a_long(testVar[count]);
		testVar[count] = 0;
	}

    fill_a_string("\r\n");
    fill_a_string("\r\n");

	test_index = 0;
}

void fill_a_switch(switch_state switch_status)
{
	switch(switch_status)
	{
		case SW_OPEN:
			fill_a_string("OPEN   ");
			break;		

		case SW_CLOSING_A:
		case SW_CLOSING_B:
			fill_a_string("CLOSING");
			break;		

		case SW_CLOSED:
			fill_a_string("CLOSED ");
			break;

		case SW_OPENING_A:
		case SW_OPENING_B:
			fill_a_string("OPENING");
			break;

		default:
			fill_a_string("ERROR! ");
			break;
	}
}


/*----------------------------------------------------------------
 * fill_a_hours()
 *
 * Called to put a time value into sio_buffer[], ready to transmit
 * Timers are passed as 100ms/bit.
 * We show them as 0.1 hour resolution
 *---------------------------------------------------------------*/

void fill_a_hours(u32 time_value)
{
	fill_a_number(time_value / 3600, 1);
	fill_a_string(" hrs");
}


/*----------------------------------------------------------------
 * fill_a_minutes()
 *
 * Called to put a time value into sio_buffer[], ready to transmit
 * Timers are passed as 100ms/bit.
 * We show them as 0.1 minute resolution
 *---------------------------------------------------------------*/

void fill_a_minutes(u32 time_value)
{
	fill_a_number(time_value / 60, 1);
	fill_a_string(" mins");
}


void fill_a_this_mins(u16 this_time)
{
	fill_a_string("      	This session: ");
	// "This" times are stored in system clock ticks (10ms) 
	fill_a_minutes(this_time / 10);
}

void set_column(u8 *col)
{
	fill_a_string("\x1B\x5B");
	fill_a_string(col);
	fill_a_string("\x47");
}

void display_analogs(void)
{
	// In case we're transmitting, wait for it to complete
	wait_for_buffer();

	fill_a_string("\n\r\nbattery: 0x");
	fill_an_int(analog_input[AIN_BATTERY_V].average);
	fill_a_string("\n\rregulation P: 0x");
	fill_an_int(analog_input[AIN_TRANS_PRES_P].average);
	fill_a_string("\n\rseparation P: 0x");
	fill_an_int(analog_input[AIN_SEP_TANK_P].average);
	fill_a_string("\n\rambient: 0x");
	fill_an_int(analog_input[AIN_AMBIENT_T].average);
	fill_a_string("\n\rdischarge T: 0x");
	fill_an_int(analog_input[AIN_DISCH_T].average);
	fill_a_string("\n\rseparation T: 0x");
	fill_an_int(analog_input[AIN_SEP_TANK_T].average);
	fill_a_string("\n\rfuel: 0x");
	fill_an_int(analog_input[AIN_FUEL_LEVEL].average);
	fill_a_string("\n\rboard T: 0x");
	fill_an_int(analog_input[AIN_BOARD_TEMP].average);
	fill_a_string("\n\rairend oil P: 0x");
	fill_an_int(analog_input[AIN_AIREND_OIL_P].average);
	fill_a_string("\n\rstart control: 0x");
	fill_an_int(analog_input[AIN_START_CONTROL].average);
	fill_a_string("\n\rregen control: 0x");
	fill_an_int(analog_input[AIN_REGEN_CONTROL].average);
//#ifdef NEVER
	fill_a_string("\n\n\rraw board T: 0x");
	fill_an_int(raw_analog[ADC_BOARD_TEMP]);
	fill_a_string("\n\rraw battery: 0x");
	fill_an_int(raw_analog[ADC_BATTERY_V]);
	fill_a_string("\n\rraw ADC I 2: 0x");
	fill_an_int(raw_analog[ADC_CURRENT_2]);
	fill_a_string("\n\rraw ADC I 1: 0x");
	fill_an_int(raw_analog[ADC_CURRENT_1]);
	fill_a_string("\n\rraw MUX: 0x");
	fill_an_int(raw_analog[ADC_MULTIPLEXER]);
	fill_a_string("\n\rraw RTD: 0x");
	fill_an_int(raw_analog[ADC_MUX_RTD]);
	fill_a_string("\n\rraw AIN9: 0x");
	fill_an_int(raw_analog[ADC_AIN9]);
	fill_a_string("\n\rraw AIN8: 0x");
	fill_an_int(raw_analog[ADC_AIN8]);
	fill_a_string("\n\rraw DIN2: 0x");
	fill_an_int(raw_analog[ADC_DIN2]);
	fill_a_string("\n\rraw DIN3: 0x");
	fill_an_int(raw_analog[ADC_DIN3]);
    

    
    
//#endif

	fill_a_string("\n\r");
	send_sio_buffer();
}


/*-------------------------------------------------------------------
 * fill_terminal_info()
 *
 * Serial dump of characters for a terminal to display operational
 *  text.
 * Called in response to receiving a '?'
 * Assemble the characters into sio_buffer[], ready to be transmitted
 *------------------------------------------------------------------*/

void fill_terminal_info(void)
{
	u16 temp_int;

	fill_a_string("\n\n\r");
	set_column("24");
	fill_a_string("Doosan Infracore - Portable Power\n\r");

	set_column("29");
    fill_a_string("Cortex Autostart Module");
	
	fill_a_string("\n\n\rSoftware Part Number ");
	fill_a_decimal(SOFTWARE_PN);
	fill_a_string(" V");
    fill_a_number(SOFTWARE_VERSION, 2);
	fill_a_string("B");

	fill_a_string("	  Built on ");	
	fill_a_string(__DATE__);
	fill_a_string(" at ");
	fill_a_string(__TIME__);

	fill_a_string("\n\n\rController Name: ");
	fill_a_string( get_recent_identity(ID_CONTROLLER) );
	set_column("43");
    fill_a_string("Machine Name: ");
	fill_a_string( get_recent_identity(ID_MACHINE) );

    fill_a_string("\n\n\rBattery voltage: ");
    fill_a_number(battery_voltage, 1);
    fill_a_string(" V");

    fill_a_string("\n\n\rAuto-Manual switch: ");
    fill_a_switch(digital_input[DIN_ENABLE_SWITCH]);
    set_column("43");
    fill_a_string("Remote pressure switch: ");
    fill_a_switch(digital_input[DIN_STARTNOW_SW]);
    
    fill_a_string("\n\n\rPressure transducer reading: ");
    fill_a_number(transducer_pressure, 1);
    fill_a_string(" psi");
    
    fill_a_string("\n\n\rPressure mode: ");

    if (ram_novram.ad_pressure_mode)
    {
        fill_a_string("Analog input");
    }
    else
    {
        fill_a_string("Digital input: ");
        fill_a_psw_pol(STN_ACTSW_STATE);
    }
    
    fill_a_string("\n\n\rTITAN Status: ");
    fill_machine_state();
    set_column("43");
    fill_a_string("Autostart Module Status: ");
    fill_autostart_state();

    fill_a_string("\n\n\rNumber of cranks: ");
    fill_a_decimal(ram_novram.number_cranks);
    
    fill_a_string("\n\rWarn time: ");
    fill_a_decimal(ram_novram.warn_time);
    
    fill_a_string("\n\rCooldown time: ");
    fill_a_decimal(ram_novram.cooldown_time);
       
    fill_a_string("\n\rRecrank delay: ");
    fill_a_decimal(ram_novram.recrank_delay);
    
//print_test_vars();
	fill_a_string("\n\r");
}


void fill_pressure_input(void)
{


    fill_a_string("\n\n\Provide pressure input, starting from default of:	");
	fill_a_decimal(ram_novram.pressure_threshold);
    fill_a_string("\n\rPress 'j' to decrease the pressure threshold, Press 'k' to increase the pressure threshold");
    fill_a_string("\n\rPress 'Esc' to quit and save the new threshold.");

}

void fill_a_psw_pol(switch_state swpol)
{
    
    switch(swpol)
    {
        case SW_OPEN:
            fill_a_string("OPEN");
            break;		
        case SW_CLOSED:
            fill_a_string("CLOSED");
            break;
        default:
            fill_a_string("ERROR! ");
            break;
    }
 
}


void send_health_screen(void) 
{
	fill_a_string("\n\n\rPower on cycles:	");
	fill_a_decimal(ram_novram.power_cycles);

	fill_a_string("\n\rTime powered: 		");
	fill_a_hours(ram_novram.on_time);
	fill_a_this_mins(this_on_time);

	fill_a_string("\n\rRun time:		");
	fill_a_hours(ram_novram.run_time);

	send_sio_buffer();
}

void fill_machine_state(void)
{
	switch (controllerMode)
	{
		case STATE_READY:
			fill_a_string("Ready");
			break;

		case STATE_CRANKING:
			fill_a_string("Cranking");
			break;

		case REV_ENGINE1:
			fill_a_string("Reving 1");
			break;

		case WAIT_PUMP_UP:
			fill_a_string("Not 50PSI");
			break;

		case WAIT_INLET_CLOSE:
			fill_a_string("5 Seconds");
			break;

		case RUN_UNLOADED:
			fill_a_string("Unloaded");
			break;

		case REV_ENGINE2:
			fill_a_string("Reving 2");
			break;

		case PRE_LOADED:
			fill_a_string("Preloaded");
			break;

		case RUN_LOADED_LO:
			fill_a_string("Loaded Lo");
			break;

		case MACHINE_DOWN:
			fill_a_string("Shutdown");
			break;

		case AUTOSTARTING:
			fill_a_string("Autostart ");
			//fill_auto_state();
			//fill_a_string(" ");
			
			break;

		case STATE_STOPPING:
			fill_a_string("Stopping");
			break;

		case RUN_LOADED_HI:
			fill_a_string("Loaded Hi");
			break;

		case STATE_COOLDOWN:
			fill_a_string("Cooldown");
			break;

		case STATE_PRECOOL:
			fill_a_string("Pre-cool");
			break;

		case STATE_VERIFY:
			fill_a_string("Verifying");
			break;
	}
}


void fill_autostart_state(void)
{
    switch(asState)
    {
        case AS_IDLE:
            fill_a_string("Idle");
            break;
        case AS_STARTUP:
            fill_a_string("Startup");
            break;
        case AS_STARTING:
            fill_a_string("Starting");
            break;
        case AS_RUNNING:
            fill_a_string("Compresor Running");
            break;
        case AS_LOADED:
            fill_a_string("Loaded");
            break;
        case AS_COOLDOWN:
            fill_a_string("Cool down");
            break;
        case AS_STOPPED:
            fill_a_string("Stopped");
            break;
        case AS_ERROR:
            fill_a_string("Error");
            break;

    }

}


void serial_receive(void)
{
	// Variable number of faults to transmit on request
	static u16 faults_to_show = 3;			   

	u8 serial_character,
	   temp_byte;

	if ( get_serial_character(&serial_character) )
	{
		if (serial_character == ESCAPE)
		{
			// Enable the next character to be a command

			// Timer for single character commands
			command_timer = SECONDS(3);

			// Reset the buffer in case some other characters were received
			sio_index = 0;

			send_a_string("\n\rSingle key commands enabled\n\r");
		}
		else if (command_timer)
		{
			// The previous character was ESC
			// Allow only one single character command at a time
			command_timer = 0;

			switch (serial_character)
			{
				case 'r':

					// Reprogram this controller

					// Set the "Program Valid" flag to a value that the
					//  BootLoader recognizes as an RS232 reprogram and is 
					//  something other than VALID_KEY 
					set_option_bytes(0xAAAA);

					// .. and reset to run the bootloader

				case 'R':

					write_novram();

					// Reset to get the BootLoader going
					software_reset();

					break;

				case 'c':

					/*-----------------------------------------------
					 * Reprogram CAN servers
					 * A connected test tool is prompting us to begin 
					 *  reprogramming all connected CAN devices.
					 *----------------------------------------------*/

					can_start_reprogram(CAN_ALL_ADDR);

					break;

				case 'a':

					// Transmit all faults
					fault_transmit_count = number_of_faults();

					break;

				case 'e':
					erase_fault_log();
					break;

				case 'f':

					// Start transmitting faults
					fault_transmit_count = faults_to_show;

					break;

				case 'h':
					send_health_screen();
					break;

				case 'I':
					display_analogs();
					break;

				case 'H':
					// Display previous novram stores
					health_transmit_count = number_of_novram();
					break;

				case 'm':

					fill_a_string("\n\n\rESC enables a single key command:");
					fill_a_string("\n\rd/x: Adjust discharge temperature AutoLoad threshold");
					fill_a_string("\n\ry/g: Adjust # faults to display");
					fill_a_string("\n\rt: Toggle Fault List<->Code");
					fill_a_string("\n\rf: Display # of Faults");
					fill_a_string("\n\ra: Display all Faults");
					fill_a_string("\n\re: Erase all Faults");
					fill_a_string("\n\rq: Change machine ID");
					fill_a_string("\n\rh: Display latest machine health data");
					fill_a_string("\n\rH: Display all machine health data");
					fill_a_string("\n\rI: Display raw averaged analog inputs");
					fill_a_string("\n\rR: Reset this module");
                
                    fill_a_string("\n\rp: Enabled/Disable Remote Pressure Switch");

					fill_a_string("\n\n\rr: Reprogram this device");
					fill_a_string("\n\rc: Reprogram other CAN devices");

					fill_a_string("\n\n\r!: Initialize NOVRAM\n\r");
					fill_a_string("\n\r?: Display present operating conditions");
					send_sio_buffer();

					break;

				case 't':

					// Toggle between sending the entire fault listing 
					//  and just sending the fault code for each one

					fill_a_string("\n\rFault Log shows ");
					if (transmit_fault_log)
					{
						fill_a_string("codes only");
						transmit_fault_log = FALSE;
					}
					else
					{
						fill_a_string("all data");
						transmit_fault_log = TRUE;
					}

					send_sio_buffer();

					break;

				case 'y':
				case 'g':

					temp_byte = number_of_faults();

					if (faults_to_show >= temp_byte)
						faults_to_show = temp_byte;
					else if (serial_character == 'y')
						faults_to_show++;
					else
					{
						if (faults_to_show > 0)
							faults_to_show--;
					}

					fill_a_string("f will show ");
					fill_a_decimal(faults_to_show);
					fill_a_string(" recent faults");
					send_sio_buffer();
						
					break;

				case '!':

//					while ( transmit_proprietary(PCMD_RESET_NOVRAM, CAN_ALL_ADDR) == FALSE );

					initialize_novram();

					fill_a_string("\n\rNOVRAM is initialized\n\r");
					send_sio_buffer();

					break;

                case 'j':
                    
                    fill_a_string("\n\rDecrease pressure threshold by 10 psi\n\r");
                    fill_a_string("\n\rNew pressure threshold:\n\r");
                    if (ram_novram.pressure_threshold >= 10)
                        ram_novram.pressure_threshold-=10;
                    fill_a_decimal(ram_novram.pressure_threshold);
                    fill_a_string(" psi.\n\r");
                    send_sio_buffer();
    
                    break;
                
                case 'k':
                    
                    fill_a_string("\n\rIncrease pressure threshold by 10 psi\n\r");
                    fill_a_string("\n\rNew pressure threshold:\n\r");
                    if (ram_novram.pressure_threshold <=215)
                        ram_novram.pressure_threshold+=10;
                         fill_a_decimal(ram_novram.pressure_threshold);
                    fill_a_string(" psi.\n\r");
                    send_sio_buffer();
                    break;
                    
                case 'M':
                     fill_a_string("\n\rChange the cooldown actuation to:\n\r");
                    if (pressure_mode_current==Switched)
                    {
                        pressure_mode_current=Transducer;
                        fill_a_string("\n\rCooldown on transducer pressure threshold\n\r");
                    }
                    else
                    {
                        pressure_mode_current=Switched;
                        fill_a_string("\n\rCooldown on remote pressure switch state\n\r");                
                    }
                    send_sio_buffer();
                    break;
                    
                case 'L':
                    fill_a_string("\n\rChange the pressure switch state polarity to:\n\r");
                    if (ram_novram.pressure_sw_bool)
                    {
                        ram_novram.pressure_sw_bool = FALSE;
                        STN_ACTSW_STATE = SW_CLOSED;
                        fill_a_string("\n\rCooldown on CLOSED state\n\r");
                    }
                    else
                    {
                        ram_novram.pressure_sw_bool = TRUE;
                        STN_ACTSW_STATE = SW_OPEN;
                        fill_a_string("\n\rCooldown on OPEN state\n\r");                
                    }
                    send_sio_buffer();
                    break;
                    
                case 'p':
                    fill_a_string("\n\rSet pressure input to:");
                    
                    if (ram_novram.ad_pressure_mode)
                    {
                        fill_a_string("\n\rDigital input.\n\r");
                        ram_novram.ad_pressure_mode=FALSE;
                    }
                    else
                    {
                        fill_a_string("\n\rAnalog input.\n\r");
                        ram_novram.ad_pressure_mode=TRUE;
            
                    }
                    send_sio_buffer();
                    break;
				case '?':

					// Initiate screen access

					// Put operational screen in sio_buffer
					fill_terminal_info();

					// Start DMA writing it out the serial port
					send_sio_buffer();


					break;
                

				default:
					// Some other character has been received
//					fill_a_string("\n\r");
//					fill_a_char(serial_character);
//					send_sio_buffer();
					break;
			}
		}
	}
}


/*---------------------------------------------------------------
 * check_serial()
 *
 * Called continuously from the main task to look for a character 
 *  in the RS232 input stream and take appropriate action.
 *--------------------------------------------------------------*/

void check_serial(void)
{
	// If a transmission is in process, we are still using sio_buffer[]
	//  and so we just return and try again later
	if ( buffer_is_busy() == FALSE )
	{
		// If we are reprogramming a CAN server, report any 
		//  in-process message to the programmer
		switch (programming_server.state)
		{
			case PRGM_NORMAL:
				// We are not programming a connected CAN device
				serial_receive();
				break;

			case PRGM_READY:

				fill_a_string("\n\rReady to reprogram CAN server ");
				fill_a_byte(programming_server.requestee);
				fill_a_string("\n\r");	
				send_sio_buffer();
			
				programming_server.state = PROGRAMMING;

				break;

			case PROGRAMMING:
				can_send_binary();
				break;

			case PRGM_SUCCESS:

				send_a_string("\n\rSuccessful reprogramming\n\r");
			
				programming_server.state = PRGM_NORMAL;

				break;

			case PRGM_ERROR:

				send_a_string("\n\rCAN error - canbus offline\n\r");
			
				programming_server.state = PRGM_NORMAL;

				break;
		}
	}
}

//Interrupt service routine based on reprogram message from the controller
void check_reprogram_state(void)
{
    
    switch (reprogram_state)
        {
            case REPROGRAM:

                
                write_novram();

                // Reset to get the BootLoader going
                software_reset();
            
                reprogram_state = NONE;

                break;
            
            default:
                
                break;
         }

}
