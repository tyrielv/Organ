/******************************************************************************/
/* MatrixEncoderDecoder v1.03                                                 */
/* main.c                                                                     */
/* John Kinkennon                                                             */
/* 11/2/2013                                                                  */
/*                                                                            */
/* This program is free software; you can redistribute it and/or modify       */
/* it under the terms of the GNU General Public License as published by       */
/* the Free Software Foundation; either version 2 of the License, or          */
/* (at your option) any later version.                                        */
/*                                                                            */
/* This program is distributed in the hope that it will be useful,            */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of             */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              */
/* GNU General Public License for more details.                               */
/*                                                                            */
/* You should have received a copy of the GNU General Public License along    */
/* with this program; if not, write to the Free Software Foundation, Inc.,    */
/* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.                */
/*                                                                            */
/* John Kinkennon, 4307 NE 65th Ct, Vancouver, WA 98661 USA                   */
/* email: john@kinkennon.com                                                  */
/*                                                                            */
/* This program uses Microchip USB software.  Refer to the included header    */
/* files for Microchip licensing restrictions.                                */
/******************************************************************************/

/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>              /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */
#include "HardwareProfile.h" /* Board LEDs, switches, etc.                    */
#include "midi.h"            /* MIDI definitions                              */
#include "lcd.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

extern uint8_t keyTable[NUM_SWITCHES];
extern bool keyBit[NUM_SWITCHES];
extern uint8_t midiRxMsg[SIZEOF_MSG_BUF];
extern uint8_t midiTxMsg[SIZEOF_TX_BUF];
extern uint8_t ReceivedDataBuffer[SIZEOF_RX_BUF];
extern int mx;
extern int rx;
extern uint8_t msgFinishedLen;
extern uint8_t msgType;
extern uint8_t msgChannel;
extern uint8_t translateTable[8][64][8];

void InitApp(void)
{
    initPorts();
    initBuffers();
    eraseMidiRxMsg();
    eraseMidiTxMsg();
    initNVM();          // initialize flash memory
    initADC();          // initialize potentiometer inputs
    initTimer2();       // initialize the .5ms timer but don't start
    initTimer3();       // initialize Timer 3 but don't start
    initTimer4();       // initialize Timer 4 but don't start
#ifdef USE_I2C
    initI2C();
#endif
#ifdef USE_LCD
    initLCD();
#endif
}

void initPorts(void) {
    AD1PCFG = 0x80fe;   // AN0 and AN8:14 are analog input for potentiometers
    TRISACLR = 0xc6ff;  // RA0:7, RA9:10, RA14:15 are outputs
    LATASET = 0x06f3;   // Turn on bits that drive matrix columns
    TRISBCLR = 0x801e;  // RB1:4 are unused or column outputs, RB15 is a PMP output
    LATBSET = 0x0018;   // Turn on bits that drive matrix columns
    TRISCSET = 0x6000;  // RC13:14 are inputs
    TRISCCLR = 0x001e;  // RC1:4 are outputs
    LATCSET = 0x001e;   // 1's to turn off status LEDs
    TRISDSET = 0xffcf;  // RD0:3, RD6:11 are inputs (matrix rows)
    TRISDCLR = 0x0030;  // RD4:5 are outputs (PMP)
    TRISECLR = 0x03ff;  // RE0:9 are outputs -- includes LEDs and switches!
    LATESET = 0x000f;   // 1's to turn off UBW32 LEDs, PMP data will control LED's
    TRISFCLR = 0x3137;  // RF0:2, RF4:5, RF8, RF12:13 are outputs
    ODCFSET = 0x0134;   // Open-drain mode for pins that cntl SAMs power supply.
    LATFSET = 0x3137;   // 1's turns off SAMs power for RF2, RF4, RF5, RF8
                        // Also turn on bits that drive matrix columns
    TRISGCLR = 0xf3c3;  // RG0:1, RG6:9, RG12:15 are outputs
    LATGSET = 0x00c0;   // Turn off status LED bits
    LATGSET = 0xf303;   // Turn on bits that drive matrix columns
}

void initBuffers(void) {
    int i;
    for (i = 0; i < NUM_SWITCHES; i++) {
        keyTable[i] = 0;
        keyBit[i] = false;
    }
}

void initTranslateTable(void) {
    int i;
    int j;
    int keyTableIndex;
    int noteNumber;
    // initialize the eight channels with normal defaults
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 64; j++) {
            translateTable[i][j][0] = M_NOTE_ON | i;
            translateTable[i][j][1] = M_KBD_FIRST_KEY + j;
            translateTable[i][j][2] = M_VELOCITY_ON;
            translateTable[i][j][3] = 0;
            translateTable[i][j][4] = M_NOTE_OFF | i;
            translateTable[i][j][5] = M_KBD_FIRST_KEY + j;
            translateTable[i][j][6] = M_VELOCITY_OFF;
            translateTable[i][j][7] = 0;
        }
    }
#ifdef USE_8x4_PEDAL
    // overwrite the first channel for an 8x4 pedal board by
    //   ignoring rows 5 thru 8
    for (i = 0; i < 8; i++) {   // for eight groups of four notes each
        for (j = 0; j < 4; j++) {   // four notes
            keyTableIndex = i * 8 + j;
            noteNumber = i * 4 + j;
            translateTable[0][keyTableIndex][0] = M_NOTE_ON;
            translateTable[0][keyTableIndex][1] = M_KBD_FIRST_KEY + noteNumber;
            translateTable[0][keyTableIndex][2] = M_VELOCITY_ON;
            translateTable[0][keyTableIndex][3] = 0;
            translateTable[0][keyTableIndex][4] = M_NOTE_OFF;
            translateTable[0][keyTableIndex][5] = M_KBD_FIRST_KEY + noteNumber;
            translateTable[0][keyTableIndex][6] = M_VELOCITY_OFF;
            translateTable[0][keyTableIndex][7] = 0;
        }
        for (j = 4; j < 8; j++) {   // four empty locations
            keyTableIndex = i * 8 + j;
            translateTable[0][keyTableIndex][0] = 0;
            translateTable[0][keyTableIndex][1] = 0;
            translateTable[0][keyTableIndex][2] = 0;
            translateTable[0][keyTableIndex][3] = 0;
            translateTable[0][keyTableIndex][4] = 0;
            translateTable[0][keyTableIndex][5] = 0;
            translateTable[0][keyTableIndex][6] = 0;
            translateTable[0][keyTableIndex][7] = 0;
        }
    }
#endif
    // fix for transpose function
    translateTable[7][61][4] = 0x97;
    translateTable[7][61][5] = 0x61;
    translateTable[7][61][6] = 0x60;
    translateTable[7][61][7] = 0;
    translateTable[7][62][4] = 0x97;
    translateTable[7][62][5] = 0x62;
    translateTable[7][62][6] = 0x60;
    translateTable[7][62][7] = 0;
}

void eraseRxBuffer(void) {
    int i;
    for (i = 0; i < SIZEOF_RX_BUF; i++)
        ReceivedDataBuffer[i] = 0;
    rx = 0;
}

void eraseMidiRxMsg(void) {
    int i;
    for (i = 0; i < SIZEOF_MSG_BUF; i++)
        midiRxMsg[i] = 0;
    mx = 0;
    msgType = 0;
    msgChannel = 0;
    msgFinishedLen = 0;
}

void eraseMidiTxMsg(void) {
    int i;
    for (i = 0; i < 4; i++) {
        midiTxMsg[i] = 0;
    }
}

void initADC(void) {
    AD1CON1 = 0x00f0;       // auto conversion after sampling, stop after 8 samples
    AD1CON2 = 0x041c;       // use MUXA, AVss/AVdd as Vref
    AD1CON3 = 0x1f3f;       // ??
    AD1CHS = 0;             // ignored for scanning
    AD1CSSL = 0x7f01;       // scan 8 inputs, RB0 and RB8:14
    AD1CON1bits.ADON = 1;   // turn on the ADC
    AD1CON1bits.ASAM = 1;   // start auto sampling
}

/*
 *  Timer 2 sets keyScanTime true so that the keys are scanned every millisecond.
 *  The variable keyScanCount counts through 32 and is incremented every 1ms.
 *  This allows lower priority tasks to be accomplished every x ms or allows
 *  tasks which run every 32 ms to be staggered so they do not all delay key
 *  scanning at the same time.
 */
void initTimer2(void) {
    T2CON = 0x30;           // stop Timer2, prescaler 1:8, int clk
    TMR2 = 0;               // zero the timer
    PR2 = 10000;            // set period register for 1.0 ms
                            //  (10000 x 0.1usec = 1 ms)
    IPC2bits.T2IP = 2;      // interrupt priority 2
    IPC2bits.T2IS = 2;      // sub-priority 2
    IFS0bits.T2IF = 0;      // clear Timer2 interrupt flag
    IEC0bits.T2IE = 1;      // enable interrupts
    //T2CONbits.ON = 1;       // turn on
}

/*
 *  Timer 3 sets a 50ms delay priot to turning on power to SAMs. Every new
 *  SAMs message resets this timer so that with a group of messages the first
 *  SAMs are activated 50ms after the final message.
 */
void initTimer3(void) {
    T3CON = 0x70;           // stop Timer3, prescaler 1:64, int clk
    TMR3 = 0;               // zero the timer
    PR3 = 15625;            // set period register for 50 ms
                            //  (15625 x 3.2usec = 50ms)
    IPC3bits.T3IP = 1;      // interrupt priority 1
    IPC3bits.T3IS = 2;      // sub-priority 2
    IFS0bits.T3IF = 0;      // clear Timer3 interrupt flag
    IEC0bits.T3IE = 1;      // enable interrupts
    //T3CONbits.ON = 1;       // turn on
}

/*
 *  Timer 4 is reset the first scan after setSAMsTims is set to true. After
 *  50ms the Timer4 interrupt turns the power off to all SAMs groups.
 */
void initTimer4(void) {
    T4CON = 0x70;           // stop Timer4, prescaler 1:256, int clk
    TMR4 = 0;               // zero the timer
    PR4 = 15625;            // set period register for 50 ms
                            //  (15625 x 3.2usec = 50ms)
    IPC4bits.T4IP = 2;      // interrupt priority 2
    IPC4bits.T4IS = 2;      // sub-priority 2
    IFS0bits.T4IF = 0;      // clear Timer4 interrupt flag
    IEC0bits.T4IE = 1;      // enable interrupts
    //T4CONbits.ON = 1;       // turn on
}

void resetTimer2(void) {
    T2CON = 0x30;           // stop Timer2, prescaler 1:64, int clk
    TMR2 = 0;               // zero the timer
    T2CONbits.ON = 1;       // turn on
}

void resetTimer3(void) {
    T3CON = 0x70;           // stop Timer3, prescaler 1:64, int clk
    TMR3 = 0;               // zero the timer
    T3CONbits.ON = 1;       // turn on
}

void resetTimer4(void) {
    T4CON = 0x70;           // stop Timer4, prescaler 1:64, int clk
    TMR4 = 0;               // zero the timer
    T4CONbits.ON = 1;       // turn on
}

void delayTimer1(int preset) {
    T1CON = 0x8070;         // start Timer1, prescaler 1:256, int clk
    TMR1 = 0;               // zero the timer
    //T1CONbits.ON = 1;       // turn on
    while (TMR1 < preset);
}

void setMatrixColumn(int matrixColumn) {
    switch (matrixColumn) {
        case 0:
            LATAbits.LATA0 = 0;
            break;
        case 1:
            LATAbits.LATA1 = 0;
            break;
        case 2:
            LATAbits.LATA4 = 0;
            break;
        case 3:
            LATAbits.LATA5 = 0;
            break;
        case 4:
            LATAbits.LATA6 = 0;
            break;
        case 5:
            LATAbits.LATA7 = 0;
            break;
        case 6:
            LATAbits.LATA9 = 0;
            break;
        case 7:
            LATAbits.LATA10 = 0;
            break;
        case 8:
            LATBbits.LATB3 = 0;
            break;
        case 9:
            LATBbits.LATB4 = 0;
            break;
        case 10:
            LATGbits.LATG8 = 0;
            break;
        case 11:
            LATGbits.LATG9 = 0;
            break;
        case 12:
            LATFbits.LATF0 = 0;
            break;
        case 13:
            LATFbits.LATF1 = 0;
            break;
        case 14:
            LATFbits.LATF12 = 0;
            break;
        case 15:
            LATFbits.LATF13 = 0;
            break;
        case 16:
            LATGbits.LATG0 = 0;
            break;
        case 17:
            LATGbits.LATG1 = 0;
            break;
        case 18:
            LATGbits.LATG12 = 0;
            break;
        case 19:
            LATGbits.LATG13 = 0;
            break;
        case 20:
            LATGbits.LATG14 = 0;
            break;
        case 21:
            LATGbits.LATG15 = 0;
            break;
    }
}

void clrMatrixColumn(int matrixColumn) {
    switch (matrixColumn) {
        case 0:
            LATAbits.LATA0 = 1;
            break;
        case 1:
            LATAbits.LATA1 = 1;
            break;
        case 2:
            LATAbits.LATA4 = 1;
            break;
        case 3:
            LATAbits.LATA5 = 1;
            break;
        case 4:
            LATAbits.LATA6 = 1;
            break;
        case 5:
            LATAbits.LATA7 = 1;
            break;
        case 6:
            LATAbits.LATA9 = 1;
            break;
        case 7:
            LATAbits.LATA10 = 1;
            break;
        case 8:
            LATBbits.LATB3 = 1;
            break;
        case 9:
            LATBbits.LATB4 = 1;
            break;
        case 10:
            LATGbits.LATG8 = 1;
            break;
        case 11:
            LATGbits.LATG9 = 1;
            break;
        case 12:
            LATFbits.LATF0 = 1;
            break;
        case 13:
            LATFbits.LATF1 = 1;
            break;
        case 14:
            LATFbits.LATF12 = 1;
            break;
        case 15:
            LATFbits.LATF13 = 1;
            break;
        case 16:
            LATGbits.LATG0 = 1;
            break;
        case 17:
            LATGbits.LATG1 = 1;
            break;
        case 18:
            LATGbits.LATG12 = 1;
            break;
        case 19:
            LATGbits.LATG13 = 1;
            break;
        case 20:
            LATGbits.LATG14 = 1;
            break;
        case 21:
            LATGbits.LATG15 = 1;
            break;
    }
}

