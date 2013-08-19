
/*
!!! Primary linker script file generated automatically !!!
This file contains the input and output files names
It also includes the secondary/main linker script that is specified in the Ride options (default or custom)
*/

SEARCH_DIR(".")
SEARCH_DIR("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj")
SEARCH_DIR("C:\Program Files\Raisonance\Ride\Lib\ARM")
STARTUP("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\startup.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\main.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Analog.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\AStuff.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Canbus.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\CanCommands.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\CanPackets.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\E3prom.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Flash.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Interrupt.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\IOdata.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Serial.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\SerialBuffer.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\ThermTables.o")
INPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\Timer.o")
GROUP(
"std_sbrk_thumb.lib"
"smallprintf_thumb.a"
"libc.a"
"libm.a"

)
INCLUDE "C:\Users\Chris\Desktop\Development\Embedded\autostart\cortexst.ld"
OUTPUT("C:\Users\Chris\Desktop\Development\Embedded\autostart\obj\cortexst.elf")
