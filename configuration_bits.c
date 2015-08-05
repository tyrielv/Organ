/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/*#include <plib.h>            /* Include to use PIC32 peripheral libraries   */
#include "user.h"

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* Refer to 'C32 Configuration Settings' under the Help > Contents            */
/* > C32 Toolchain in MPLAB X IDE for available PIC32 Configurations.  For    */
/* additional information about what the hardware configurations mean in      */
/* terms of device operation, refer to the device datasheet 'Special Features'*/
/* chapter.                                                                   */
/*                                                                            */
/******************************************************************************/

/* Fill in your configuration bits here.  The general style is shown below.
The Debug Configuration bit is handline by MPLAB and should not be embedded
in the configuration macro.*/

#ifdef USE_PICKIT3

#pragma config FVBUSONIO = ON       // VBUS_ON Controlled by USB Module
#pragma config FUSBIDIO  = ON       // USBID Controlled by USB Module
#pragma config UPLLEN    = ON       // USB PLL Enabled
#pragma config FPLLMUL   = MUL_20   // PLL Multiplier
#pragma config UPLLIDIV  = DIV_2    // USB PLL Input Divider
#pragma config FPLLIDIV  = DIV_2    // PLL Input Divider
#pragma config FPLLODIV  = DIV_1    // PLL Output Divider
#pragma config FPBDIV    = DIV_2    // Peripheral Clock divisor
// The command SYSTEMConfigPerformance(80000000L) in main() sets FPBDIV = DIV_1.
// DIV_2 eased an oscillator startup issue on one chip -- go figure...
#pragma config FWDTEN    = OFF      // Watchdog Timer
#pragma config WDTPS     = PS1      // Watchdog Timer Postscale
#pragma config FCKSM     = CSECME   // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC  = OFF      // CLKO Enable
#pragma config POSCMOD   = XT       // Primary Oscillator
#pragma config IESO      = OFF      // Internal/External Switch-over
#pragma config FSOSCEN   = OFF      // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC     = PRIPLL   // Oscillator Selection
#pragma config CP        = OFF      // Code Protect
#pragma config BWP       = OFF      // Boot Flash Write Protect
#pragma config PWP       = OFF      // Program Flash Write Protect
#pragma config ICESEL    = ICS_PGx2 // ICE/ICD Comm Channel Select

#endif
