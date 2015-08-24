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
/*                                                                            */
/* Modified 8/5/2015 by Tyrie Vella to support Allen MDC                      */
/******************************************************************************/

/******************************************************************************/
/*  Files to Include                                                          */
/******************************************************************************/

#include <xc.h>             /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */
#include <plib.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "user.h"           /* User funct/params, such as InitApp             */
#include "midi.h"           /* MIDI definitions                               */
#include "HardwareProfile.h"/* Board LEDs, switches, etc.                     */
#include "i2c_jk.h"         /* Comms to Centipede Shields                     */
#include "sysex.h"          /* Types for System Exclusive messages            */
#include "usb.h"
#include "USB/usb_function_midi.h"
#include "lcd.h"

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/*
 * These types are defined in sysex.h and store System Exclusive MIDI messages.
 * The framework exists in parseMidiMsg() to put these messages to further use.
 */
HW_Sysex_LCD_Msg_t LCD_Msg;
HW_Sysex_Status_String_t HW_Status_String;
HW_Sysex_Status_Float_t HW_Status_Float;
HW_Sysex_Status_Byte_t HW_Status_Byte;

/*
 * NVM Tables -- The .user_nvm_data section it located in the program area of
 * flash memory.  It has been moved well past the end of the present code to
 * allow for program growth without the need to constantly relocate this data.
 */
uint8_t nvmTable1[BYTE_PAGE_SIZE] \
__attribute__((space(prog), address(NVM_ADDRESS_PAGE), \
section(".user_nvm_data"))) = {[0 ...(BYTE_PAGE_SIZE) - 1] = 0xFF};

/*
 * keyTable[key#] is a byte representing recent scans on that key (or switch).
 * The top bit == 1 (0b10000000) indicates that the key is considered to be down
 * (closed or ON). The lower three bits indicate status for recent scans.
 * A key that is ON (0b10000111) might transition to OFF as follows:
 * (0b10000110) on this scan key was no longer closed (key up)
 * (0b10000100) 1ms later it is again open
 * (0b00000000) 2ms and the third scan shows open as well (no key bounce) and
 *              the program considers the key open (up).  Opposite on key down.
 */
uint8_t keyTable[NUM_SWITCHES]; // qty. N channels x 64 keys, current status

/*
 * The inverted input value at each of (64 x N channels) input points. The
 * normal key up voltage is 3.3v giving a value of 1 at the port. This value is
 * inverted as it's easier to think of ON == 1 and OFF == 0.
 * Press key #23 and keyBit[23] = 1.
 */
bool keyBit[NUM_SWITCHES];

/*
 * The translation table handles up to 512 keys or switches.
 */
uint8_t translateTable[8][64][8]; // 8 kybd, 64 keys, 4 bytes per msg x2

/* The SAMs table maps each SAM to an organ division,
 * Centipede Shield chip#, and bitOffset where:
 *      Division
 *          0x00 = no assignment
 *          0x01 = pedal division -- SP1 on the Allen power supply
 *          0x02 = swell division -- SP2
 *          0x03 = great division -- SP3
 *          0x04 = coupler panel  -- SP4 (use SP4, disconnect CP power)
 * Division assignments correspond to the original physical and electrical
 * connections in the Allen console and may not match a given virtual
 * organ's usage. They indicate which power supply output powers that SAM.
 *      chip number (add CENT_CHIP_1 to get chipAddress, chips 0 to 3
 *                   are four MCP23017's on the first Centipede Shield,
 *                   chips 4 to 7 are on a second Centipede Shield)
 *          0 to 7
 *      bitOffset   (0 to 7  is A0 to A7 on the MCP23017 IC,
 *                   8 to 15 is B0 to B7 on the IC)
 *          0 to 15
 */

uint8_t ReceivedDataBuffer[SIZEOF_RX_BUF]; // data from USB
uint8_t midiRxMsg[SIZEOF_MSG_BUF]; // big enough to handle typical Sysex buffer
int rx = 0;
int mx = 0; // midiMsg index, not local to parseMidiMsg()
            //   as a message may not be complete
uint8_t midiTxMsg[SIZEOF_TX_BUF];
USB_AUDIO_MIDI_EVENT_PACKET midiData;
USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

uint32_t i2cActualClock1;
// the following arrays are 2 x 8 where there are two I2C buses with 8 ICs each
uint16_t i2cRxData[4];           // input data from pistons and stops
uint8_t chipAddress = CENT_CHIP_1;

volatile uint32_t *adcBufferPtr[8] = { &ADC1BUF0, &ADC1BUF1, &ADC1BUF2,
                    &ADC1BUF3, &ADC1BUF4, &ADC1BUF5, &ADC1BUF6, &ADC1BUF7 };

uint32_t stalePot[16] = { 0,0,0,0,
                        0,0,0,0,
                        0,0,0,0,
                        0,0,0,0 };  // previous potentiometer values
uint32_t freshPot[16] = { 0,0,0,0,
                        0,0,0,0,
                        0,0,0,0,
                        0,0,0,0 };  // current potentiometer values
uint8_t keyScanCount = 0;           // count reset every 32 kybd scans
bool potScanTime = false;           // set by ADC interrupt
bool keyScanTime = false;           // set by timer2 interrupt
uint8_t msgFinishedLen = 0;         // set to length of MIDI msg
uint8_t msgType = 0;
uint8_t msgChannel = 0;
int8_t transposeHW = 0;
int8_t transposeSW = 0;

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int32_t main(void) {

#ifndef PIC32_STARTER_KIT
    /*The JTAG is on by default on POR.  A PIC32 Starter Kit uses the JTAG, but
    for other debug tool use, like ICD 3 and Real ICE, the JTAG should be off
    to free up the JTAG I/O */
    DDPCONbits.JTAGEN = 0;
#endif

    /*Refer to the C32 peripheral library compiled help file for more
    information on the SYTEMConfig function.

    This function sets the PB divider, the Flash Wait States, and the DRM
    /wait states to the optimum value.  It also enables the cacheability for
    the K0 segment.  It could has side effects of possibly alter the pre-fetch
    buffer and cache.  It sets the RAM wait states to 0.  Other than
    the SYS_FREQ, this takes these parameters.  The top 3 may be '|'ed
    together:

    SYS_CFG_WAIT_STATES (configures flash wait states from system clock)
    SYS_CFG_PB_BUS (configures the PB bus from the system clock)
    SYS_CFG_PCACHE (configures the pCache if used)
    SYS_CFG_ALL (configures the flash wait states, PB bus, and pCache)*/

    SYSTEMConfigPerformance(80000000L);
    //SYSTEMConfig(PB_BUS_MAX_FREQ_HZ, SYS_CFG_ALL);

    /* Initialize I/O and Peripherals for application */
    InitApp();
    InitializeSystem(); // the Microchip USB initialization
    
    /*Configure Multivector Interrupt Mode.  Using Single Vector Mode
    is expensive from a timing perspective, so most applications
    should probably not use a Single Vector Mode*/
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    USBDeviceAttach();  // connect to host computer
    if(USBDeviceState == ATTACHED_STATE)         
        resetTimer2(); // start scanning timer;
    while(1)
    {
        ProcessIO();
    }
}

/*
 * parseMidiMsg() needs to process up to 64 bytes from ReceivedDataBuffer[]
 *      and assemble them into midiMsg[]. SysEx msgs are processed up to 64
 *      bytes and do not span two reads of the RxBuffer at this time.
 * TODO: rewrite entire parseMidiMsg()...
 */
void parseMidiMsg(void) {
    rx = 0; // receive buffer index
    int i;
    unsigned char tempData;

    while (ReceivedDataBuffer[rx]) {
        tempData = ReceivedDataBuffer[rx++];
        switch (tempData) {
            case MIDI_CIN_NOTE_OFF:
                for (i = 0; i < 3; i++) {
                    midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                }
                msgType = midiRxMsg[0] & 0xf0;
                msgChannel = midiRxMsg[0] & 0x0f;
                msgFinishedLen = 3;
                break;
            case MIDI_CIN_NOTE_ON:
                for (i = 0; i < 3; i++) {
                    midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                }
                msgType = midiRxMsg[0] & 0xf0;
                msgChannel = midiRxMsg[0] & 0x0f;
                msgFinishedLen = 3;
                break;
            case MIDI_CIN_POLY_KEY_PRESS:
            case MIDI_CIN_CONTROL_CHANGE:
            case MIDI_CIN_PITCH_BEND_CHANGE:
            case MIDI_CIN_SSP:
                for (i = 0; i < 3; i++) {
                    midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                }
                msgType = midiRxMsg[0] & 0xf0;
                msgChannel = midiRxMsg[0] & 0x0f;
                msgFinishedLen = 3;
                break;
            case MIDI_CIN_PROGRAM_CHANGE:
                //case MIDI_CIN_CHANNEL_PRESSURE:
            case MIDI_CIN_SONG_SELECT:
                //case MIDI_CIN_MTC:
                for (i = 0; i < 2; i++) {
                    midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                }
                msgType = midiRxMsg[0] & 0xf0;
                msgChannel = midiRxMsg[0] & 0x0f;
                msgFinishedLen = 2;
                break;
            case MIDI_CIN_SINGLE_BYTE:
                // Real Time msg
                // don't step on msgType, msgChannel, midiMsg[]
                // advance rx to skip past this byte
                rx++;
                break;
            case MIDI_CIN_SYSEX_START:
                //case MIDI_CIN_SYSEX_CONTINUE:
                for (i = 0; i < 3; i++) {
                    midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                }
                break;
            case MIDI_CIN_SYSEX_ENDS_3:
                midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                // no break; fall through
            case MIDI_CIN_SYSEX_ENDS_2:
                midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                // no break; fall through
            case MIDI_CIN_SYSEX_ENDS_1:
                midiRxMsg[mx++] = ReceivedDataBuffer[rx++];
                if (ReceivedDataBuffer[rx - 1] == M_END_EXCLUSIVE) {
                    msgType = M_SYSTEM_EX;
                    msgFinishedLen = mx;
                }
                break;
            default:
                eraseMidiRxMsg(); // if not handled dump it
        }

        // if we have a complete message (all bytes received)
        if ((msgFinishedLen > 0) && (mx == msgFinishedLen)) { // complete msg
            if ((msgType == M_NOTE_ON) || (msgType == M_NOTE_OFF)) {
                if (msgChannel == 5) {
                    // handle a stop action magnet (SAM)
                    int stop = midiRxMsg[1] - 0x24;
                    if (stop > 80) break;       // sanity check                    
                }
            }
            if ((msgType == M_SYSTEM_EX) && (midiRxMsg[1] == M_SYSEX_ID)) {
                switch (msgFinishedLen) {
                    case 39: // handle Hauptwerk LCD message
                        for (i = 0; i < 39; i++)
                            LCD_Msg.v[i] = midiRxMsg[i];
                        if (LCD_Msg.s1.message_type != MSG_TYPE_LCD) break;
                        break;
                    case 22: // change to translation for one key
                        if (midiRxMsg[2] != 0x70) break; // msg type/length mis-match
                        int j = midiRxMsg[3]; // keybd
                        int k = midiRxMsg[4]; // key
                        if (k < NUM_SWITCHES) {
                            translateTable[j][k][0] = (midiRxMsg[5] << 4) | midiRxMsg[6];
                            translateTable[j][k][1] = (midiRxMsg[7] << 4) | midiRxMsg[8];
                            translateTable[j][k][2] = (midiRxMsg[9] << 4) | midiRxMsg[10];
                            translateTable[j][k][3] = (midiRxMsg[11] << 4) | midiRxMsg[12];
                            translateTable[j][k][4] = (midiRxMsg[13] << 4) | midiRxMsg[14];
                            translateTable[j][k][5] = (midiRxMsg[15] << 4) | midiRxMsg[16];
                            translateTable[j][k][6] = (midiRxMsg[17] << 4) | midiRxMsg[18];
                            translateTable[j][k][7] = (midiRxMsg[19] << 4) | midiRxMsg[20];
                        }
                        break;
                    case 21: // handle Hauptwerk Status, string variable
                        for (i = 0; i < 21; i++)
                            HW_Status_String.v[i] = midiRxMsg[i]; // copy msg to structure
                        if (HW_Status_String.s1.message_type != MSG_TYPE_STATUS_STRING) break;
                        // TODO: handle message
                        break;
                    case 9: // handle Hauptwerk Status, float variable
                        for (i = 0; i < 9; i++)
                            HW_Status_Float.v[i] = midiRxMsg[i]; // copy msg to structure
                        if (HW_Status_Float.s1.message_type != MSG_TYPE_STATUS_FLOAT) break;
                        switch (HW_Status_Float.s1.variable_ID) {
                            uint8_t temp;
                            case TRANSPOSER_SEMITONES:
                                temp = HW_Status_Float.s1.float_4;
                                if (temp & 0x40)
                                    temp |= 0x80;   // if 0bx1xxxxxx, 0b11xxxxxx
                                transposeHW = (int8_t)temp;
                                
                                if (transposeSW > transposeHW) {
                                    midiTxMsg[0] = translateTable[7][0x3d][0];
                                    midiTxMsg[1] = translateTable[7][0x3d][1];
                                    midiTxMsg[2] = translateTable[7][0x3d][2];
                                    sendMidiMsg();
                                }
                                if (transposeHW > transposeSW) {
                                    midiTxMsg[0] = translateTable[7][0x3e][0];
                                    midiTxMsg[1] = translateTable[7][0x3e][1];
                                    midiTxMsg[2] = translateTable[7][0x3e][2];
                                    sendMidiMsg();
                                }
                                /* TODO
                                if (transposeSW == 0) {
                                    LATCbits.LATC1 = 1; // RC1 - First LED - Red (transpose)
                                } else {
                                    LATCbits.LATC1 = 0;
                                } */
                            break;
                            }
                        break;
                    case 7: // load table from flash mem, or write one table
                        if (midiRxMsg[2] != 0x72) break; // msg type/length mis-match
                        if (midiRxMsg[3] == 0) { // 0 is load
                            getNvmTable();
                        } else if (midiRxMsg[3] == 1) { // 1 is write
                            putNvmTable();
                        }
                        break;
                    case 6: // handle Hauptwerk Status, boolean variables
                        for (i = 0; i < 6; i++)
                            HW_Status_Byte.v[i] = midiRxMsg[i]; // copy msg to structure
                        if (HW_Status_Byte.s.message_type != MSG_TYPE_STATUS_BOOLEAN) break;
                        /* TODO
                        switch (HW_Status_Byte.s.variable_ID) {
                            case IS_SETTER_MODE_ON:
                                if (HW_Status_Byte.s.byte_value)
                                    LATGbits.LATG6 = 0; // RG6 - 5th LED - Red
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATGbits.LATG6 = 1; // turn off Indicator
                                break;
                            case IS_SCOPE_MODE_ON:
                                if (HW_Status_Byte.s.byte_value)
                                    LATGbits.LATG6 = 0; // RG6 - 5th LED - Red
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATGbits.LATG6 = 1; // turn off Indicator
                                break;
                            case IS_RECORDING_AUDIO:
                                if (HW_Status_Byte.s.byte_value)
                                    LATCbits.LATC3 = 0; // RC3 - 2nd Grn - Audio Recording
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATCbits.LATC3 = 1; // turn off Recording Indicator
                                break;
                            case IS_RECORDING_MIDI:
                                if (HW_Status_Byte.s.byte_value)
                                    LATCbits.LATC3 = 0; // RC3 - 2nd Grn - MIDI Recording
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATCbits.LATC3 = 1; // turn off Recording Indicator
                                break;
                            case IS_PLAYING_MIDI:
                                if (HW_Status_Byte.s.byte_value)
                                    LATCbits.LATC4 = 0; // RC4 - 3rd Grn - Red (transpose)
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATCbits.LATC4 = 1; // turn off MIDI Indicator
                                break;
                            case IS_ORGAN_READY:
                                if (HW_Status_Byte.s.byte_value) {
                                    LATCbits.LATC2 = 0; // RC2 - Second LED - Audio Active
                                    //initI2C();            // White (power)
                                    //resetTimer2();      // start scanning
                                    for (i=0; i<16; i++) {
                                        stalePot[i] = 0;
                                    }
                                }
                                else if (HW_Status_Byte.s.byte_value == 0) {
                                    //T2CONbits.ON = 0;   // stop scanning
                                    LATCbits.LATC2 = 1; // turn off Audio Active
                                }
                                break;
                            case IS_IN_ERROR_STATE:
                                if (HW_Status_Byte.s.byte_value)
                                    LATGbits.LATG7 = 0; // RG7 - 6th LED - Red - Error
                                else if (HW_Status_Byte.s.byte_value == 0)
                                    LATGbits.LATG7 = 1; // turn off Error
                                break;
                        } */
                } // end of handled sysex messages
            } // end of complete sysex msg
                        eraseMidiRxMsg();
        } // end of complete msg
        if (rx > SIZEOF_RX_BUF) eraseRxBuffer();
    } // end of going through RxBuffer
    eraseMidiRxMsg();
    eraseRxBuffer();
}

void initNVM(void) {
    initTranslateTable();
    if (nvmTable1[0] == 0xFF) // if first flash table is uninitialized
        putNvmTable(); //   then init to defaults
    getNvmTable(); // always retrieve saved translate table
}

void putNvmTable(void) {
    unsigned int pageBuffer[PAGE_SIZE]; // temp storage for NVM functions
    unsigned int flashAddress = NVM_PROGRAM_PAGE; // 0xbd010000+
    NVMErasePage((void *) flashAddress);
    NVMProgram((void *) flashAddress, (const void *) translateTable,
            sizeof (translateTable), (void *) pageBuffer);
}

void getNvmTable(void) {
    unsigned int flashAddress = NVM_PROGRAM_PAGE;
    memcpy((void *) translateTable, (const void *) flashAddress, sizeof (translateTable));
}

void getNvmTableRow(int tableRow) {
    unsigned int flashAddress = NVM_PROGRAM_PAGE + 0x200 * tableRow;
    void * dest = &translateTable[tableRow];
    memcpy((void *) dest, (const void *) flashAddress, 0x200);
}

void initI2C(void) {
    int i, j;

    for (i = 0; i < 4; i++) {
        i2cRxData[i] = 0xffff;            
    }
    // Set the I2C baudrate
    i2cActualClock1 = I2CSetFrequency(CENTIPEDE_I2C_BUS1, GetPeripheralClock(), I2C_CLOCK_FREQ);
    // Enable the I2C1 bus
    I2CConfigure(CENTIPEDE_I2C_BUS1, I2C_ENABLE_HIGH_SPEED);
    I2CEnable(CENTIPEDE_I2C_BUS1, true);
    
    // TODO: fix DUAL and SINGLE to reflect actual system configuration
    chipAddress = CENT_CHIP_1;
    for (i = 0; i < SINGLE_CENT_NUM_CHIPS; i++) {
        initCentipedeChip(CENTIPEDE_I2C_BUS1, chipAddress++, false);
    }
}

#define ON_VALUE 1
#define OFF_VALUE 0

void getBits(int matrixColumn) {
    int i = 0;
    if (matrixColumn == 0) {
        keyBit[0] = (SWELL_INPUT_6) ? ON_VALUE : OFF_VALUE;
        keyBit[64] = (GREAT_INPUT_6) ? ON_VALUE : OFF_VALUE;
    }
    else if (matrixColumn < 11) {
        i = (matrixColumn * 6) - 5;
        keyBit[i++] = (SWELL_INPUT_1) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (SWELL_INPUT_2) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (SWELL_INPUT_3) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (SWELL_INPUT_4) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (SWELL_INPUT_5) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (SWELL_INPUT_6) ? ON_VALUE : OFF_VALUE;
        i += 58;
        keyBit[i++] = (GREAT_INPUT_1) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (GREAT_INPUT_2) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (GREAT_INPUT_3) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (GREAT_INPUT_4) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (GREAT_INPUT_5) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (GREAT_INPUT_6) ? ON_VALUE : OFF_VALUE;
    }
    else if (matrixColumn == 11) {
        keyBit[128] = (PEDAL_INPUT_6) ? ON_VALUE : OFF_VALUE;
        keyBit[196] = (TRANSPOSE_INPUT_1) ? ON_VALUE : OFF_VALUE;
    }
    else {
        i = ((matrixColumn - 11) * 6) + 128 - 5;
        keyBit[i++] = (PEDAL_INPUT_1) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (PEDAL_INPUT_2) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (PEDAL_INPUT_3) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (PEDAL_INPUT_4) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (PEDAL_INPUT_5) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (PEDAL_INPUT_6) ? ON_VALUE : OFF_VALUE;
        i += 58;
        keyBit[i++] = (TRANSPOSE_INPUT_1) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (TRANSPOSE_INPUT_2) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (TRANSPOSE_INPUT_3) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (TRANSPOSE_INPUT_4) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (TRANSPOSE_INPUT_5) ? ON_VALUE : OFF_VALUE;
        keyBit[i++] = (TRANSPOSE_INPUT_6) ? ON_VALUE : OFF_VALUE;
    }
}

void updateKeyTable(int matrix) {
    unsigned char key;
    //unsigned char * pData = &keyTable[key];
    unsigned char prevData;
    int index = matrix * 64;
    for (key = 0; key < 64; key++) { // process one set of 64 keys
        //prevData = *pData;
        prevData = keyTable[index];
        unsigned int newKey = keyBit[index];
        unsigned char keyDown = prevData & 0x80;
        prevData <<= 1; // shift the data left one bit
        if (newKey) prevData++; // set newest bit
        prevData &= 0x07; // strip any high bits

        if (!keyDown) { // if note is off in keyTable see if it should be on
            if (prevData == 0b00000111) { // if 3 consecutive keyDown
                keyDown = 1; // turn key on in keyTable
                midiTxMsg[0] = M_NOTE_ON | matrix;
                midiTxMsg[1] = key;
                midiTxMsg[2] = M_VELOCITY_ON;
                putMidiMsg();
            }
        } else if (keyDown) { // if note is on in keyTable...
            if (prevData == 0b00000000) { // if 3 consecutive keyUp
                keyDown = 0; // turn key off in keyTable
                midiTxMsg[0] = M_NOTE_OFF | matrix;
                midiTxMsg[1] = key;
                midiTxMsg[2] = M_VELOCITY_OFF;
                putMidiMsg(); // send note off msg
            }
        }
        if (keyDown) prevData |= 0b10000000; // store the data
        keyTable[index++] = prevData;
    }
}

void putMidiMsg() {
    uint8_t mType = midiTxMsg[0] & 0xf0;
    int8_t key = midiTxMsg[1];
    int8_t kybd = midiTxMsg[0] & 0x0f;
    int index = (kybd * 64) + key;
    switch (mType) {
        case M_NOTE_ON:
            if (kybd == 3) {                            // MIDI ch4
                if (key == 0) {     // if a transpose encoder bit
                    int8_t transposeTemp = 5;
                    if (keyBit[index++]) transposeTemp -= 1;
                    if (keyBit[index++]) transposeTemp -= 2;
                    if (keyBit[index++]) transposeTemp -= 4;
                    if (keyBit[index]) transposeTemp -= 8;
                    transposeSW = transposeTemp;
                    if (transposeSW > transposeHW) {
                        kybd = 7;
                        key = 0x3d;                     // increment
                    } else if (transposeHW > transposeSW) {
                        kybd = 7;
                        key = 0x3e;                     // decrement
                    } else {
                        mType = 0;                      // send nothing
                    }
                }
                else if (key < 4) {
                    mType = 0;                          // send nothing
                }
            }
            midiTxMsg[0] = translateTable[kybd][key][0];
            midiTxMsg[1] = translateTable[kybd][key][1];
            midiTxMsg[2] = translateTable[kybd][key][2];
            if (mType)
            sendMidiMsg();
            break;
        case M_NOTE_OFF:
            if (kybd == 3) {                            // MIDI ch4
                if (key == 0) {     // if a transpose encoder bit
                    int8_t transposeTemp = 5;
                    if (keyBit[index++]) transposeTemp -= 1;
                    if (keyBit[index++]) transposeTemp -= 2;
                    if (keyBit[index++]) transposeTemp -= 4;
                    if (keyBit[index]) transposeTemp -= 8;
                    transposeSW = transposeTemp;
                    if (transposeSW > transposeHW) {
                        kybd = 7;
                        key = 0x3d;                     // increment
                    } else if (transposeHW > transposeSW) {
                        kybd = 7;
                        key = 0x3e;                     // decrement
                    } else {
                        mType = 0;                      // send nothing
                    }
                }
                else if (key < 4) {
                    mType = 0;                          // send nothing
                }
            }
            midiTxMsg[0] = translateTable[kybd][key][4];
            midiTxMsg[1] = translateTable[kybd][key][5];
            midiTxMsg[2] = translateTable[kybd][key][6];
            if (mType)
            sendMidiMsg();
            break;
    }
}

void sendMidiMsg(void) {
    while (USBHandleBusy(USBTxHandle));
    if (!USBHandleBusy(USBTxHandle)) {
        midiData.Val = 0;
        midiData.CableNumber = 0;
        midiData.CodeIndexNumber = midiTxMsg[0] >> 4;
        midiData.DATA_0 = midiTxMsg[0];
        midiData.DATA_1 = midiTxMsg[1];
        midiData.DATA_2 = midiTxMsg[2];
        USBTxHandle = USBTxOnePacket(MIDI_EP, (BYTE*) & midiData, 4);
    }
}

void getPots() {
    
    int i = 0;
    int pot = 0;
    uint32_t diff;
    //for (i = 0; i < 1; i++) {
        freshPot[i] = *adcBufferPtr[i] >> 3;
        freshPot[i] = (freshPot[i] + stalePot[i]) >> 1; // average old and new
        diff = freshPot[i] - stalePot[i]; // find difference
        if (diff) {
            /*if (i == 0) {
                pot = 7;
            }
            else pot = i - 1;*/
            pot = 0;
            midiTxMsg[0] = M_CTRL_CHANGE | pot;
            midiTxMsg[1] = M_CC_VOLUME;
                midiTxMsg[2] = (uint8_t)freshPot[i];
            stalePot[i] = freshPot[i]; // save previous value
            sendMidiMsg();
        }
    //}
}

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {

    //	The USB specifications require that USB peripheral devices must never source
    //	current onto the Vbus pin.  Additionally, USB peripherals should not source
    //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
    //	When designing a self powered (as opposed to bus powered) USB peripheral
    //	device, the firmware should make sure not to turn on the USB module and D+
    //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
    //	firmware needs some means to detect when Vbus is being powered by the host.
    //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
    // 	can be used to detect when Vbus is high (host actively powering), or low
    //	(host is shut down or otherwise not supplying power).  The USB firmware
    // 	can then periodically poll this I/O pin to know when it is okay to turn on
    //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
    //	peripheral device, it is not possible to source current on D+ or D- when the
    //	host is not actively providing power on Vbus. Therefore, implementing this
    //	bus sense feature is optional.  This firmware can be made to use this bus
    //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
    //	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

    //	If the host PC sends a GetStatus (device) request, the firmware must respond
    //	and let the host know if the USB peripheral device is currently bus powered
    //	or self powered.  See chapter 9 in the official USB specifications for details
    //	regarding this request.  If the peripheral device is capable of being both
    //	self and bus powered, it should not return a hard coded value for this request.
    //	Instead, firmware should check if it is currently self or bus powered, and
    //	respond accordingly.  If the hardware has been configured like demonstrated
    //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
    //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
    //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
    //	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
    //	to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:
 *
 *****************************************************************************/
void UserInit(void) {
    //Initialize all of the LED pins
    //mInitAllLEDs();

    //Initialize all of the push buttons
    //mInitAllSwitches();

    //initialize the variable holding the handle for the last
    // transmission
    USBTxHandle = NULL;
    USBRxHandle = NULL;
}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void) {
    
    //Blink the LEDs according to the USB device status
    //Don't blink if LCD is used -- RE pin conflicts
    
    // User Application USB tasks
//    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    if (!USBHandleBusy(USBRxHandle)) {
        //We have received a MIDI packet from the host, process it and then
        //  prepare to receive the next packet
        //putsLCD("Parse RX Msg");
        //INSERT MIDI PROCESSING CODE HERE
        parseMidiMsg();

        //Get ready for next packet (this will overwrite the old data)
        USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
    }

    if (keyScanTime) {
        mLED_1_On();
        int i, j, k;
        // get keyBit[] values by reading ports from low note to
        //   high note (61 notes)
        for (i = 0; i < 18; i++) {
            setMatrixColumn(i); // set one column at a time low
            getBits(i);
            clrMatrixColumn(i);
            for (j = 0; j < 100; j++) {
                //Needed to allow pull-up resistors time to work?
            }
        }
        
        for (i = 0; i < 4; i++) {
            updateKeyTable(i);
        }
        if (keyScanCount % 4 == 0) { // if 0, 4, 8, 12, 
            i = keyScanCount / 4;    // i = 0, 1, 2,3

            chipAddress = CENT_CHIP_1 + i;
            i2cRxData[i] = getCentipedeBytes(CENTIPEDE_I2C_BUS1, chipAddress, CENT_REG_A);
            
            /* Every 32 scans write the chip data (pistons and stops) to the
             * keyBit[512] table. */
            if (keyScanCount == 0) { // once every 32 key scans
                int offset = 256;
                for (j = 0; j < 8; j++) { // for 8 16-bit words
                    uint16_t i2cBits = i2cRxData[j];
                    for (k = 0; k < 16; k++) { // for 16 bits
                        keyBit[offset++] = (i2cBits & 0x0001) ? 0 : 1;
                        i2cBits = i2cBits >> 1;
                    }
                }
                updateKeyTable(4);
            }
            /* Every 32 scans read the potentiometers */
            
            if (keyScanCount == 8) {
                if (IFS1bits.AD1IF) {
                    //AD1CON1bits.ASAM = 0;
                    getPots();             // read all 8 pots
                    IFS1bits.AD1IF = 0;
                    AD1CON1bits.ASAM = 1;   // restart auto sampling
                }
            } 
        }        
        keyScanTime = false; // don't re-enter while running!
    }
}//end ProcessIO

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void) {
    static uint32_t led_count = 0;

    if (led_count == 0)led_count = 300000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    //if (USBSuspendControl == 1) {
        if (led_count == 0) {
            if (mGetLED_1()) {
                mLED_1_On();
                mLED_2_Off();
            } else {
                mLED_2_On();
                mLED_1_Off();
            }
        }//end if
    /*} else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
            mLED_3_On();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                if (mGetLED_1()) {
                    mLED_2_Off();
                } else {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)
*/
}//end BlinkUSBStatus

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void) {
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.


#if defined(__C30__)
#if 0
    U1EIR = 0xFFFF;
    U1IR = 0xFFFF;
    U1OTGIR = 0xFFFF;
    IFS5bits.USB1IF = 0;
    IEC5bits.USB1IE = 1;
    U1OTGIEbits.ACTVIE = 1;
    U1OTGIRbits.ACTVIF = 1;
    Sleep();
#endif
#endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *                  In this example the interrupt is only used when the device
 *                  goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0

void __attribute__((interrupt)) _USB1Interrupt(void) {
#if !defined(self_powered)
    if (U1OTGIRbits.ACTVIF) {
        IEC5bits.USB1IE = 0;
        U1OTGIEbits.ACTVIE = 0;
        IFS5bits.USB1IF = 0;

        //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
        USBClearInterruptFlag(USBIdleIFReg, USBIdleIFBitNum);
        //USBSuspendControl = 0;
    }
#endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                  suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                  mode, the host may wake the device back up by sending non-
 *                  idle state signalling.
 *
 *                  This call back is invoked when a wakeup from USB suspend
 *                  is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void) {
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // a few milliseconds of wakeup time, after which the device must be
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.

    //    if (msCounter != 0) {
    //        msCounter--;
    //    }
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void) {
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.

    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}

/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                  firmware must process the request and respond
 *                  appropriately to fulfill the request.  Some of
 *                  the SETUP packets will be for standard
 *                  USB "chapter 9" (as in, fulfilling chapter 9 of
 *                  the official USB specifications) requests, while
 *                  others may be specific to the USB device class
 *                  that is being implemented.  For example, a HID
 *                  class device needs to be able to respond to
 *                  "GET REPORT" type of requests.  This
 *                  is not a standard USB chapter 9 request, and
 *                  therefore not handled by usb_device.c.  Instead
 *                  this request should be handled by class specific
 *                  firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void) {
}//end

/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                  called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                  arrives.  Typically SET_DESCRIPTOR requests are
 *                  not used in most applications, and it is
 *                  optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void) {
    // Must claim session ownership if supporting this request
}//end

/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                  SET_CONFIGURATION (wValue not = 0) request.  This
 *                  callback function should initialize the endpoints
 *                  for the device's usage according to the current
 *                  configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void) {
    //enable the HID endpoint
    USBEnableEndpoint(MIDI_EP, USB_OUT_ENABLED | USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *                  peripheral devices to wake up a host PC (such
 *                  as if it is in a low power suspend to RAM state).
 *                  This can be a very useful feature in some
 *                  USB applications, such as an Infrared remote
 *                  control receiver.  If a user presses the "power"
 *                  button on a remote control, it is nice that the
 *                  IR receiver can detect this signalling, and then
 *                  send a USB "command" to the PC to wake up.
 *
 *                  The USBCBSendResume() "callback" function is used
 *                  to send this special USB signalling which wakes
 *                  up the PC.  This function may be called by
 *                  application firmware to wake up the PC.  This
 *                  function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *                  1.  The USB driver used on the host PC supports
 *                      the remote wakeup capability.
 *                  2.  The USB configuration descriptor indicates
 *                      the device is remote wakeup capable in the
 *                      bmAttributes field.
 *                  3.  The USB host PC is currently sleeping,
 *                      and has previously sent your device a SET
 *                      FEATURE setup packet which "armed" the
 *                      remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *                  This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void) {
    static uint16_t delay_count;

    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager
    //properties page for the USB device, power management tab, the
    //"Allow this device to bring the computer out of standby." checkbox
    //should be checked).
    if (USBGetRemoteWakeupStatus() == true) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == true) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = false; //So we don't execute this code again,
            //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at
            //least 3ms from bus idle to USBIsBusSuspended() == true, yields
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT
            //      endpoints).
            break;
        default:
            break;
    }
    return true;
}
