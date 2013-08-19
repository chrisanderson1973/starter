
/*
!!! Primary linker script file generated automatically !!!
This file contains the input and output files names
It also includes the secondary/main linker script that is specified in the Ride options (default or custom)
*/

SEARCH_DIR(".")
SEARCH_DIR("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart")
SEARCH_DIR("C:\Program Files\Raisonance\Ride\Lib\ARM")
STARTUP("C:\Program Files\Raisonance\Ride\Lib\ARM\crt0_STM32x_HD.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Timer.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Analog.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Canbus.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\CanCommands.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\CanPackets.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\E3prom.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Flash.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Interrupt.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\IOdata.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\main.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\Serial.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\SerialBuffer.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\startup.o")
INPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\ThermTables.o")
EXTERN( __io_putchar )
EXTERN( _write )
GROUP(
"STM32x_io_putchar_thumb.a"
"std_sbrk_thumb.lib"
"smallprintf_thumb.a"
"STM32F10x_thumb.lib"
"libc.a"
"libm.a"

)
INCLUDE "C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\STM32F103_32K_32K_FLASH.ld"
OUTPUT("C:\Documents and Settings\All Users\Desktop\CWork\Cortex\Projects\CortexAutoStart\cortexst.elf")
