/* 
 * File:   hdweProfile.h
 * Author: John Kinkennon
 *
 * Created on June 4, 2012, 12:11 PM
 */

#ifndef HDWEPROFILE_H
#define	HDWEPROFILE_H

#ifdef	__cplusplus
extern "C" {
#endif

/*******************************************************************/
/******** USB stack hardware selection options *********************/
/*******************************************************************/
//This section is the set of definitions required by the MCHPFSUSB
//  framework.  These definitions tell the firmware what mode it is
//  running in, and where it can find the results to some information
//  that the stack needs.
//These definitions are required by every application developed with
//  this revision of the MCHPFSUSB framework.  Please review each
//  option carefully and determine which options are desired/required
//  for your application.

//#define USE_SELF_POWER_SENSE_IO
//#define tris_self_power     TRISBbits.TRISB4    // Input 2
#define self_power          0

//#define USE_USB_BUS_SENSE_IO
//#define tris_usb_bus_sense  TRISBbits.TRISB5    // Input 1
#define USB_BUS_SENSE       0

/** LED ************************************************************/

//#define mInitAllLEDs()      LATE &= 0xFFF0; TRISE &= 0xFFF0;

//#define mLED_1              LATEbits.LATE0  // Yellow LED
//#define mLED_2              LATEbits.LATE1  // Red LED
//#define mLED_3              LATEbits.LATE2  // White LED
//#define mLED_4              LATEbits.LATE3  // Green LED

//#define mGetLED_1()         mLED_1
//#define mGetLED_2()         mLED_2
//#define mGetLED_3()         mLED_3
//#define mGetLED_4()         mLED_4

//#define mLED_1_On()         mLED_1 = 0;
//#define mLED_2_On()         mLED_2 = 0;
//#define mLED_3_On()         mLED_3 = 0;
//#define mLED_4_On()         mLED_4 = 0;

//#define mLED_1_Off()        mLED_1 = 1;
//#define mLED_2_Off()        mLED_2 = 1;
//#define mLED_3_Off()        mLED_3 = 1;
//#define mLED_4_Off()        mLED_4 = 1;

//#define mLED_1_Toggle()     mLED_1 = !mLED_1;
//#define mLED_2_Toggle()     mLED_2 = !mLED_2;
//#define mLED_3_Toggle()     mLED_3 = !mLED_3;
//#define mLED_4_Toggle()     mLED_4 = !mLED_4;

/** SWITCH *********************************************************/
//#define mInitSwitch2()      TRISEbits.TRISE6=1;
//#define mInitSwitch3()      TRISEbits.TRISE7=1;
//#define mInitAllSwitches()  mInitSwitch2(); mInitSwitch3();
//#define sw2                 PORTEbits.RE6
//#define sw3                 PORTEbits.RE7

/** I/O pin definitions ********************************************/
#define INPUT_PIN   1
#define OUTPUT_PIN  0

#ifdef	__cplusplus
}
#endif

#endif	/* HDWEPROFILE_H */

