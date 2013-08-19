/**********************************************************************
 * common.h
 *
 * Doosan Infracore International Portable Power imbedded systems
 * CORTEX TITAN controller
 * Common defines and prototypes
 * CORTEX TITAN AutoStart module controller for air compressors
 * 2/27/2013 CLA V0.01    Initial code taken from Titan Compressor
 **********************************************************************/

#ifndef __COMMON_H
#define __COMMON_H


#define SOFTWARE_PN			46630821


#define SOFTWARE_VERSION	1


// Define some 16 bit, non-zero bit pattern to indicate validity
// This must match the one the BootLoader uses to qualify a program
#define VALID_KEY		0x5A3C


#endif
