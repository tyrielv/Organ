/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>
#include <peripheral/pmp.h>  /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */
#include "HardwareProfile.h" /* Board LEDs, switches, etc.                    */
#include "lcd.h"            /* MIDI definitions                              */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void initLCD(void) {
    // PMP initialization
    mPMPOpen(PMP_ON | PMP_READ_WRITE_EN | 3,
            PMP_DATA_BUS_8 | PMP_MODE_MASTER1 |
            PMP_WAIT_BEG_4 | PMP_WAIT_MID_15 |
            PMP_WAIT_END_4,
            0x0001,         // only PMA0 enabled
            PMP_INT_OFF);   // no interrupts used
    // wait for >30ms
    delayTimer1(9375);
    
    //initiate the HD44780 display 8-bit init sequence
    PMPSetAddress(LCDCMD);  // select command register
    PMPMasterWrite(0x38);   // 8-bit int, 2 lines, 5x7
    delayTimer1(15);
    
    PMPMasterWrite(0x0c);   // ON, no cursor, no blink
    delayTimer1(15);
    
    PMPMasterWrite(0x01);   // clear display
    delayTimer1(563);
    
    PMPMasterWrite(0x06);   // increment cursor, no shift
    delayTimer1(563);
    
}

char readLCD(int addr) {
    PMPSetAddress(addr);    // select register
    mPMPMasterReadByte();   // init read sequence
    return mPMPMasterReadByte();    // read actual data
}

void writeLCD(int addr, char c) {
    while (busyLCD());      // wait for LCD ready
    PMPSetAddress(addr);    // select register
    PMPMasterWrite(c);      // init write sequence
}

void putsLCD(char *s) {
    char c;
    while (*s) {
        switch (*s) {
            case '\n':      // point to second line
                setLCDC(0x40);
                break;
            case '\r':      // home, point to first line
                setLCDC(0);
                break;
            case '\t':      // advance next tab (8) positions
                c = addrLCD();
                while (c & 7) {
                    putLCD(' ');
                    c++;
                }
                if (c > 15) // if necessary to move to second line
                    setLCDC(0x40);
                break;
            default:        // print character
                putLCD(*s);
                break;
        }
        s++;            // point to next character
        delayTimer1(30);// Increase if characters are dropped
    }
}
