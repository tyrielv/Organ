/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>              /* Include to use PIC32 peripheral libraries     */
#include <sys/attribs.h>     /* For __ISR definition                          */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "HardwareProfile.h"
/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* VECTOR NAMES:                                                              */
/*                                                                            */
/* _CORE_TIMER_VECTOR          _COMPARATOR_2_VECTOR                           */
/* _CORE_SOFTWARE_0_VECTOR     _UART_2A_VECTOR                                */
/* _CORE_SOFTWARE_1_VECTOR     _I2C_2A_VECTOR                                 */
/* _EXTERNAL_0_VECTOR          _SPI_2_VECTOR                                  */
/* _TIMER_1_VECTOR             _SPI_2A_VECTOR                                 */
/* _INPUT_CAPTURE_1_VECTOR     _I2C_4_VECTOR                                  */
/* _OUTPUT_COMPARE_1_VECTOR    _UART_3_VECTOR                                 */
/* _EXTERNAL_1_VECTOR          _UART_2_VECTOR                                 */
/* _TIMER_2_VECTOR             _SPI_3A_VECTOR                                 */
/* _INPUT_CAPTURE_2_VECTOR     _I2C_3A_VECTOR                                 */
/* _OUTPUT_COMPARE_2_VECTOR    _UART_3A_VECTOR                                */
/* _EXTERNAL_2_VECTOR          _SPI_4_VECTOR                                  */
/* _TIMER_3_VECTOR             _I2C_5_VECTOR                                  */
/* _INPUT_CAPTURE_3_VECTOR     _I2C_2_VECTOR                                  */
/* _OUTPUT_COMPARE_3_VECTOR    _FAIL_SAFE_MONITOR_VECTOR                      */
/* _EXTERNAL_3_VECTOR          _RTCC_VECTOR                                   */
/* _TIMER_4_VECTOR             _DMA_0_VECTOR                                  */
/* _INPUT_CAPTURE_4_VECTOR     _DMA_1_VECTOR                                  */
/* _OUTPUT_COMPARE_4_VECTOR    _DMA_2_VECTOR                                  */
/* _EXTERNAL_4_VECTOR          _DMA_3_VECTOR                                  */
/* _TIMER_5_VECTOR             _DMA_4_VECTOR                                  */
/* _INPUT_CAPTURE_5_VECTOR     _DMA_5_VECTOR                                  */
/* _OUTPUT_COMPARE_5_VECTOR    _DMA_6_VECTOR                                  */
/* _SPI_1_VECTOR               _DMA_7_VECTOR                                  */
/* _I2C_3_VECTOR               _FCE_VECTOR                                    */
/* _UART_1A_VECTOR             _USB_1_VECTOR                                  */
/* _UART_1_VECTOR              _CAN_1_VECTOR                                  */
/* _SPI_1A_VECTOR              _CAN_2_VECTOR                                  */
/* _I2C_1A_VECTOR              _ETH_VECTOR                                    */
/* _SPI_3_VECTOR               _UART_4_VECTOR                                 */
/* _I2C_1_VECTOR               _UART_1B_VECTOR                                */
/* _CHANGE_NOTICE_VECTOR       _UART_6_VECTOR                                 */
/* _ADC_VECTOR                 _UART_2B_VECTOR                                */
/* _PMP_VECTOR                 _UART_5_VECTOR                                 */
/* _COMPARATOR_1_VECTOR        _UART_3B_VECTOR                                */
/*                                                                            */
/* Refer to the device specific .h file in the C32 Compiler                   */
/* pic32mx\include\proc directory for a complete Vector and IRQ mnemonic      */
/* listings for the PIC32 device.                                             */
/*                                                                            */
/* PRIORITY OPTIONS:                                                          */
/*                                                                            */
/* (default) IPL0AUTO, IPL1, IPL2, ... IPL7 (highest)                         */
/*                                                                            */
/* Example Shorthand Syntax                                                   */
/*                                                                            */
/* void __ISR(<Vector Name>,<PRIORITY>) user_interrupt_routine_name(void)     */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more interrupt macro examples refer to the C compiler User Guide in    */
/* the C compiler /doc directory.                                             */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/*
 * INTERRUPT SERVICE ROUTINES
 * Note that usb_device.c installs a USB interrupt, ipl4, vector 45.
 * Timer2 sets a semiphore, keyScanTime, which starts a new scan of the
 * input pins once every 1 msec.  samScanTime counts 0 to 31ms.
 * samScanTime == 0, read first i2c bus
 * samScanTime == 12, read second i2c bus
 * samScanTime == 24, auto scan all 8 pots
 * All keys and pistons get scanned 1000 times per second.
 */

extern volatile bool keyScanTime;
extern volatile uint8_t keyScanCount;
extern volatile bool setSAMsTime;
extern volatile bool SAMsPowerOn;

void __ISR(_TIMER_2_VECTOR, ipl2) TMR2_InterruptHandler(void) {
    if (!keyScanTime) {
        if (++keyScanCount > 31)
            keyScanCount = 0;        // reset samScanTime counter
        keyScanTime = true;
        
    }
    IFS0bits.T2IF = 0;  // Clear the interrupt flag
}

void __ISR(_TIMER_3_VECTOR, ipl1) TMR3_InterruptHandler(void) {
    /* Timer 3 does not run unless started by parseMidiMsg()
     * If Timer 4 has not run yet then try to setSAMsTime in
     * another 50ms. */
    if (!setSAMsTime) {
        setSAMsTime = true; // must be SAMs that need set or cleared
        T3CONbits.ON = 0;   // Turn Timer 3 off
    }
    IFS0bits.T3IF = 0;  // Clear the interrupt flag
}

void __ISR(_TIMER_4_VECTOR, ipl2) TMR4_InterruptHandler(void) {
    /* Timer 4 does not run unless started by program */
    LATFSET = 0x0134;   // 1's turns off SAMs power for RF2, RF4, RF5, RF8
    SAMsPowerOn = false;
    T4CONbits.ON = 0;   // Turn Timer 4 off
    IFS0bits.T4IF = 0;  // Clear the interrupt flag
}

