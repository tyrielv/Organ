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
/*                                                                            */
/* Modified 8/5/2015 by Tyrie Vella to support Allen MDC                      */
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
    AD1PCFG = 0x0000;   // No analogs currently enabled
    
    // Matrix rows - inputs default to high
    TRISASET = 0xc03c; //RA2-5, 15-16 are inputs for transpose and (great?)
    TRISCSET = 0x2000; //RD0, 8-11, and RC13 are inputs for pedals and (swell?)
    TRISDSET = 0x0f01; 
    
    
    // Matrix columns - turned on
    //RF2,4,5,8,12 RD14,15, RB12-15 are columns for swell and great
    LATBSET = 0xf000;
    LATDSET = 0xc000;
    LATFSET = 0x1134;
    //RC14, RD1-7,12-13, RF0-1, RG1 are columns for pedals and transpose
    LATCSET = 0x4000;
    LATDSET = 0x30fe;
    LATFSET = 0x0003;
    LATGSET = 0x0002;
    
    //Outputs (currently none - will have some for preset LED)
    
    
    /* TRISACLR = 0xc6ff;  // RA0:7, RA9:10, RA14:15 are outputs
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
    LATGSET = 0xf303;   // Turn on bits that drive matrix columns */
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
    /*TODO
    AD1CON1 = 0x00f0;       // auto conversion after sampling, stop after 8 samples
    AD1CON2 = 0x041c;       // use MUXA, AVss/AVdd as Vref
    AD1CON3 = 0x1f3f;       // ??
    AD1CHS = 0;             // ignored for scanning
    AD1CSSL = 0x7f01;       // scan 8 inputs, RB0 and RB8:14
    AD1CON1bits.ADON = 1;   // turn on the ADC
    AD1CON1bits.ASAM = 1;   // start auto sampling */
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

void resetTimer2(void) {
    T2CON = 0x30;           // stop Timer2, prescaler 1:64, int clk
    TMR2 = 0;               // zero the timer
    T2CONbits.ON = 1;       // turn on
}

void delayTimer1(int preset) {
    T1CON = 0x8070;         // start Timer1, prescaler 1:256, int clk
    TMR1 = 0;               // zero the timer
    //T1CONbits.ON = 1;       // turn on
    while (TMR1 < preset);
}

void setMatrixColumn(int matrixColumn) {
    setMatrixColumnToValue(matrixColumn, 0);
}

void setMatrixColumnToValue(int matrixColumn, int newValue) {
    switch (matrixColumn) {
        case 0:
            MANUAL_OUTPUT_1 = newValue;
            break;
        case 1:
            MANUAL_OUTPUT_2 = newValue;
            break;
        case 2:
            MANUAL_OUTPUT_3 = newValue;
            break;
        case 3:
            MANUAL_OUTPUT_4 = newValue;
            break;
        case 4:
            MANUAL_OUTPUT_5 = newValue;
            break;
        case 5:
            MANUAL_OUTPUT_6 = newValue;
            break;
        case 6:
            MANUAL_OUTPUT_7 = newValue;
            break;
        case 7:
            MANUAL_OUTPUT_8 = newValue;
            break;
        case 8:
            MANUAL_OUTPUT_9 = newValue;
            break;
        case 9:
            MANUAL_OUTPUT_10 = newValue;
            break;
        case 10:
            MANUAL_OUTPUT_11 = newValue;
            break;
        case 11:
            PEDAL_OUTPUT_1 = newValue;
            TRANPOSE_OUTPUT_1 = newValue;
            break;
        case 12:
            PEDAL_OUTPUT_2 = newValue;
            TRANPOSE_OUTPUT_2 = newValue;
            break;
        case 13:
            PEDAL_OUTPUT_3 = newValue;
            TRANPOSE_OUTPUT_3 = newValue;
            break;
        case 14:
            PEDAL_OUTPUT_4 = newValue;
            TRANPOSE_OUTPUT_4 = newValue;
            break;
        case 15:
            PEDAL_OUTPUT_5 = newValue;
            TRANPOSE_OUTPUT_5 = newValue;
            break;
        case 16:
            PEDAL_OUTPUT_6 = newValue;
            TRANPOSE_OUTPUT_6 = newValue;
            break;
        case 17:
            PEDAL_OUTPUT_7 = newValue;
            break;
    }
}

void clrMatrixColumn(int matrixColumn) {
    setMatrixColumnToValue(matrixColumn, 1);
}

