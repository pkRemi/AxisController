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
void IC2Init(void)
{
    I2CCONbits.I2CEN = 1;               // Enable I2C controller
    I2CBRG = 15;                        // Set I2C Clock to 400kHz at 10 MIPS

}

