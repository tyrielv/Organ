#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <peripheral/i2c.h>
#include "i2c_jk.h"

/*******************************************************************************
  Function:
    bool StartTransfer( bool restart )

  Summary:
    Starts (or restarts) a transfer to/from the CENTIPEDE SHIELD.

  Description:
    This routine starts (or restarts) a transfer to/from the CENTIPEDE, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If false, send a "Start" condition
            - If true, send a "Restart" condition

  Returns:
    true    - If successful
    false   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(false);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
 *****************************************************************************/

bool StartTransfer(I2C_MODULE i2cModule, bool restart) {
    I2C_STATUS status;
    // Send the Start (or Restart) signal
    if (restart) {
        I2CRepeatStart(i2cModule);
    } else {
        // Wait for the bus to be idle, then start the transfer
        while (!I2CBusIsIdle(i2cModule));
        if (I2CStart(i2cModule) != I2C_SUCCESS) {
            return false;
        }
    }
    // Wait for the signal to complete
    do {
        status = I2CGetStatus(i2cModule);
    } while (!(status & I2C_START));
    return true;
}

/*******************************************************************************
  Function:
    bool TransmitOneByte( uint8_t data )

  Summary:
    This transmits one byte to the CENTIPEDE SHIELD.

  Description:
    This transmits one byte to the CENTIPEDE, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    true    - Data was sent successfully
    false   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
 *****************************************************************************/

bool TransmitOneByte(I2C_MODULE i2cModule, uint8_t data) {
    // Wait for the transmitter to be ready
    while (!I2CTransmitterIsReady(i2cModule));
    // Transmit the byte
    if (I2CSendByte(i2cModule, data) == I2C_MASTER_BUS_COLLISION) {
        return false;
    }
    // Wait for the transmission to finish
    while (!I2CTransmissionHasCompleted(i2cModule));
    return true;
}

/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the CENTIPEDE SHIELD.

  Description:
    This routine Stops a transfer to/from the CENTIPEDE, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
 *****************************************************************************/

void StopTransfer(I2C_MODULE i2cModule) {
    I2C_STATUS status;
    // Send the Stop signal
    I2CStop(i2cModule);
    // Wait for the signal to complete
    do {
        status = I2CGetStatus(i2cModule);
    } while (!(status & I2C_STOP));
}

bool initCentipedeChip(I2C_MODULE i2cModule, uint8_t chipAddress, bool isOutput) {
    uint8_t i2cData[10];
    I2C_7_BIT_ADDRESS SlaveAddress;
    bool Success = true;
    int Index;
    int DataSz;

    // Initialize the data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, chipAddress, I2C_WRITE);
    if (isOutput) {
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = 0x00; // CENTIPEDE IODIRA register (defaults to inputs)
        i2cData[2] = 0x00; // PORT A all outputs
        i2cData[3] = 0x00; // PORT B all outputs
        DataSz = 4;
    }
    else {
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = 0x0C; // CENTIPEDE GPPUA register (pullups)
        i2cData[2] = 0xFF; // Enable PORT A pullups
        i2cData[3] = 0xFF; // Enable PORT B pullups
        DataSz = 4;
    }
    // Start the transfer to write data to the CENTIPEDE
    StartTransfer(i2cModule, false);
    // Transmit all data
    Index = 0;
    while (Index < DataSz) {
        // Transmit a byte
        if (TransmitOneByte(i2cModule, i2cData[Index])) {
            // Advance to the next byte
            Index++;
            // Verify that the byte was acknowledged
            if (!I2CByteWasAcknowledged(i2cModule))
                Success = false;
        }
    }
    StopTransfer(i2cModule);
    return Success;
}

bool putCentipedeBytes(I2C_MODULE i2cModule, uint8_t chipAddress, UINT16_VAL wordA) {
    uint8_t i2cData[10];
    I2C_7_BIT_ADDRESS SlaveAddress;
    bool Success = true;
    int Index;
    int DataSz;

    // Initialize the data buffer
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, chipAddress, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;
    i2cData[1] = 0x12;  // CENTIPEDE GPIOA register
    i2cData[2] = wordA.byte.LB; // PORT A all outputs
    i2cData[3] = wordA.byte.HB; // PORT B all outputs
    DataSz = 4;

    // Start the transfer to write data to the CENTIPEDE
    StartTransfer(i2cModule, false);
    // Transmit all data
    Index = 0;
    while (Index < DataSz) {
        // Transmit a byte
        if (TransmitOneByte(i2cModule, i2cData[Index])) {
            // Advance to the next byte
            Index++;
            // Verify that the byte was acknowledged
            if (!I2CByteWasAcknowledged(i2cModule))
                Success = false;
        }
    }
    StopTransfer(i2cModule);
    return Success;
}

uint16_t getCentipedeBytes(I2C_MODULE i2cModule, uint8_t chipAddress, uint8_t chipRegister) {

    I2C_7_BIT_ADDRESS SlaveAddress;
    uint16_t i2cResult = 0;
    uint16_t tempData = 0;
    
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, chipAddress, I2C_WRITE);
    // Start the transfer to read the CENTIPEDE.
    StartTransfer(i2cModule, false);
    // Address the CENTIPEDE.
    TransmitOneByte(i2cModule, SlaveAddress.byte);
    while (!I2CByteWasAcknowledged(i2cModule));
    TransmitOneByte(i2cModule, chipRegister);
    while (!I2CByteWasAcknowledged(i2cModule));
    // Restart and send the CENTIPEDE's internal address to switch to a read transfer
    StartTransfer(i2cModule, true);
    // Transmit the address with the READ bit set
    SlaveAddress.rw = I2C_READ;
    TransmitOneByte(i2cModule, SlaveAddress.byte);
    while (!I2CByteWasAcknowledged(i2cModule));

    // Read the data from PORTA
    I2CReceiverEnable(i2cModule, true);
    while (!I2CReceivedDataIsAvailable(i2cModule));
    i2cResult = I2CGetByte(i2cModule);
    I2CAcknowledgeByte(i2cModule, true);
    while (!I2CAcknowledgeHasCompleted(i2cModule));
    
    // Read the data from PORTB
    I2CReceiverEnable(i2cModule, true);
    while (!I2CReceivedDataIsAvailable(i2cModule));
    tempData = I2CGetByte(i2cModule);
    I2CAcknowledgeByte(i2cModule, false);
    while (!I2CAcknowledgeHasCompleted(i2cModule));
    
    i2cResult |= tempData << 8;
    StopTransfer(i2cModule);
    return i2cResult;
}
