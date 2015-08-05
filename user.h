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
/* User Level #define Macros                                                  */
/******************************************************************************/

//#define USE_LCD           /* Comment out if no LCD is connected             */
//#define USE_I2C           /* Comment out if no Centipede Shields used       */
#define USE_11x6_MATRIX     /* Comment out for 8x8 matrix size                */
//#define USE_8x4_PEDAL     /* Uncomment to use an 8x4 pedalboard (some ADCs) */

#define USE_PICKIT3         /* Comment out to use bootloader */
#define SIZEOF_MSG_BUF      64
#define SIZEOF_RX_BUF       64
#define SIZEOF_TX_BUF       4
#define NUM_SWITCHES        512    // default total keys and pistons

/** NVM definitions ***********************************************************/
#define NVM_PROGRAM_PAGE 0xbd010000 // new value for use with UBW32 bootloader
#define NVM_ADDRESS_PAGE (NVM_PROGRAM_PAGE & 0x9fffffff)

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
void getPots(int numPots);
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
