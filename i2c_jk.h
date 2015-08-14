// Clock Constants
#define GetSystemClock()        (SYS_CLOCK) // a preprocessor macro for compiler
#define GetPeripheralClock()    (SYS_CLOCK) // (SYS_CLOCK/2) if FPBDIV = DIV_2
#define GetInstructionClock()   (SYS_CLOCK)
#define I2C_CLOCK_FREQ          400000      // 400000 usable with shorter cables

// Centipede Constants
#define CENTIPEDE_I2C_BUS1      I2C2
#define CENTIPEDE_BASE_ADDRESS  0x20    // 0b0100000 Centipede Shield address

#define CENT_CHIP_1             0x20
#define CENT_CHIP_2             0x21
#define CENT_CHIP_3             0x22
#define CENT_CHIP_4             0x23
#define SINGLE_CENT_NUM_CHIPS   4

#define CENT_REG_A              0x12
#define CENT_REG_B              0x13

// Function Prototypes
bool StartTransfer(I2C_MODULE i2cModule, bool restart);
bool TransmitOneByte(I2C_MODULE i2cModule, uint8_t data);
void StopTransfer(I2C_MODULE i2cModule);
bool initCentipedeChip(I2C_MODULE i2cModule, uint8_t chipAddress, bool isOutput);
uint16_t getCentipedeBytes(I2C_MODULE i2cModule, uint8_t chipAddress, uint8_t chipRegister);
bool putCentipedeBytes(I2C_MODULE i2cModule, uint8_t chipAddress, UINT16_VAL wordA);