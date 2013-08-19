/**********************************************************
 * AStuff.h
 *
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 **********************************************************/
 
#ifndef _ASTUFF_H
#define _ASTUFF_H

#define LED_ALL			0x0D

extern u8 transmit_identity;

extern void update_autostart_state(void);
extern void update_ambient_temp(void);
extern void update_battery_voltage(void);
extern void update_leds(void);
extern void turn_leds_off(void);
extern void turn_leds_on(void);
extern void perform_self_check(void);

#endif
