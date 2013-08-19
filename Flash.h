/****************************************************************
 * Flash.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Flash memory interface defines and prototypes
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 ****************************************************************/

#ifndef __FLASH_H
#define __FLASH_H


extern void write_flash_int(u16 *, u16);
extern bool wait_for_flash();
extern bool erase_flash_page(u32);
extern void set_option_bytes(u16);


#endif
