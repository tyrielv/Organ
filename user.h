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
/* User Level #define Macros                                                  */
/******************************************************************************/

#define USE_PICKIT3         /* Comment out to use bootloader */
#define SIZEOF_MSG_BUF      64
#define SIZEOF_RX_BUF       64
#define SIZEOF_TX_BUF       4
#define NUM_SWITCHES        512    // default total keys and pistons

/** NVM definitions ***********************************************************/
#define NVM_PROGRAM_PAGE 0xbd010000 // new value for use with UBW32 bootloader
#define NVM_ADDRESS_PAGE (NVM_PROGRAM_PAGE & 0x9fffffff)

/* which ports are which functions */
//RA2-5, 14-15 are inputs for transpose and (great?)
//RD0, 8-11, and RC13 are inputs for pedals and (swell?)
//RF2,4,5,8,12 RD14,15, RB12-15 are columns for swell and great
//RC14, RD1-7,12-13, RF0-1, RG1 are columns for pedals and transpose
#define SWELL_INPUT_1 PORTDbits.RD10
#define SWELL_INPUT_2 PORTDbits.RD9
#define SWELL_INPUT_3 PORTDbits.RD8
#define SWELL_INPUT_4 PORTDbits.RD0
#define SWELL_INPUT_5 PORTDbits.RD11
#define SWELL_INPUT_6 PORTCbits.RC13

#define PEDAL_INPUT_1 PORTDbits.RD0
#define PEDAL_INPUT_2 PORTDbits.RD8
#define PEDAL_INPUT_3 PORTCbits.RC13
#define PEDAL_INPUT_4 PORTDbits.RD11
#define PEDAL_INPUT_5 PORTDbits.RD10
#define PEDAL_INPUT_6 PORTDbits.RD9

#define GREAT_INPUT_1 PORTAbits.RA5  
#define GREAT_INPUT_2 PORTAbits.RA14 
#define GREAT_INPUT_3 PORTAbits.RA15 
#define GREAT_INPUT_4 PORTFbits.RF2
#define GREAT_INPUT_5 PORTFbits.RF8
#define GREAT_INPUT_6 PORTAbits.RA4 

#define TRANSPOSE_INPUT_1 PORTFbits.RF2
#define TRANSPOSE_INPUT_2 PORTAbits.RA4 
#define TRANSPOSE_INPUT_3 PORTFbits.RF8
#define TRANSPOSE_INPUT_4 PORTAbits.RA5
#define TRANSPOSE_INPUT_5 PORTAbits.RA15 
#define TRANSPOSE_INPUT_6 PORTAbits.RA14

#define MANUAL_OUTPUT_1 LATFbits.LATF12
#define MANUAL_OUTPUT_2 LATBbits.LATB12 
#define MANUAL_OUTPUT_3 LATBbits.LATB13 
#define MANUAL_OUTPUT_4 LATBbits.LATB15
#define MANUAL_OUTPUT_5 LATBbits.LATB14
#define MANUAL_OUTPUT_6 LATDbits.LATD15
#define MANUAL_OUTPUT_7 LATFbits.LATF4
#define MANUAL_OUTPUT_8 LATDbits.LATD14
#define MANUAL_OUTPUT_9 LATAbits.LATA1
#define MANUAL_OUTPUT_10 LATFbits.LATF13
#define MANUAL_OUTPUT_11 LATFbits.LATF5


#define PEDAL_OUTPUT_1 LATDbits.LATD12
#define PEDAL_OUTPUT_2 LATDbits.LATD1
#define PEDAL_OUTPUT_3 LATDbits.LATD13
#define PEDAL_OUTPUT_4 LATDbits.LATD4
#define PEDAL_OUTPUT_5 LATCbits.LATC14
#define PEDAL_OUTPUT_6 LATDbits.LATD3
#define PEDAL_OUTPUT_7 LATDbits.LATD2

#define TRANPOSE_OUTPUT_1 LATDbits.LATD7
#define TRANPOSE_OUTPUT_2 LATGbits.LATG1
#define TRANPOSE_OUTPUT_3 LATFbits.LATF0
#define TRANPOSE_OUTPUT_4 LATFbits.LATF1
#define TRANPOSE_OUTPUT_5 LATDbits.LATD5 
#define TRANPOSE_OUTPUT_6 LATDbits.LATD6 

#define TRANSPOSE_RESET 63
#define TRANSPOSE_UP 61
#define TRANSPOSE_DOWN 62
/*Uncomment to send transpose neutral, up, and down
instead of shifting sent keys */
//#define USE_MIDI_TRANSPOSE 

/******************************************************************************/
/* User Function Prototypes                                                    /
/******************************************************************************/

void InitApp(void);         /* I/O and Peripheral Initialization */
void putNvmTable(void);
void getNvmTable(void);
void getNvmTableRow(int tableRow);
void initPorts(void);
void initBuffers(void);
void initTranslateTable(void);
void initADC(void);
void initTimer2(void);
void initTimer3(void);
void initTimer4(void);
void resetTimer2(void);
void resetTimer3(void);
void resetTimer4(void);
void delayTimer1(int preset);
void initI2C(void);
void setMatrixColumn(int matrixColumn);
void clrMatrixColumn(int matrixColumn);
void getTwelveBits(int matrixColumn);
void getSixteenBits(int matrixColumn);
void updateKeyTable(int matrix);
void eraseRxBuffer(void);
void eraseMidiTxMsg(void);
void eraseMidiRxMsg(void);
void putMidiMsg(void);
void sendMidiMsg(void);
void getPots();
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
