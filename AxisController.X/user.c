/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <p30Fxxxx.h>        /* Device header file                            */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */

#include "user.h"            /* variables/params used by user.c               */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    /* Setup analog functionality and port direction */
    // Configure all four port D pins (RD0, RD1, RD2, RD3)
    // as digital outputs
    LATD = 0;
    TRISD = 0b1111111111110000;

//    ADPCFG = 0x1FF;                  // Set all pins to digital

//    TRISB = 0xFFFE;                     // Set PORT pins RP0 output - rest input
//    PORTB = 0x01;                       // Set RP0 high
    /* Initialize peripherals */
}
void initSerial(void)
{

    U2MODEbits.STSEL = 0;               // 1 Stop bit
    U2MODEbits.PDSEL = 0;               // No Parity, 8 data bits
    U2MODEbits.ABAUD = 0;               // Auto-Baud Disabled
//    U2BRG = 64;                     // BAUD Rate Setting for 9600
    U2BRG = 31;                     // BAUD Rate Setting for 19200
    U2STAbits.UTXISEL = 0;             // Interrupt after one TX Character is transmitted
    IEC0bits.U1TXIE = 0;                // Enable UART TX Interrupt
    U2MODEbits.UARTEN = 1;              // Enable UART
    U2STAbits.UTXEN = 1;                // Enable UART TX
}
void IC2Init(void) // Not used
{
    I2CCONbits.I2CEN = 1;               // Enable I2C controller
    I2CBRG = 15;                        // Set I2C Clock to 400kHz at 10 MIPS

}
void InitMPU6050(unsigned char I2Caddr)
{
    LDByteWriteI2C(I2Caddr, 0x6B, 0x00); // Wakeup MPU6050
    /* ACCEL_CONFIG 0x1C selftest and Full Scale Range
     0b00000000 -> 2g
     0b00001000 -> 4g
     0b00010000 -> 8g
     0b00011000 -> 16g                                                        */
    LDByteWriteI2C(I2Caddr, 0x1C, 0b00001000); // FSR 4g
    /* GYRO_CONFIG 0x1B selftest and Full Scale Range
     0b00000000 -> 250 deg/s
     0b00001000 -> 500 deg/s
     0b00010000 -> 1000 deg/s
     0b00011000 -> 2000 deg/s                                                 */
    LDByteWriteI2C(I2Caddr, 0x1B, 0b00010000); // FSR 1000 deg/s
}
