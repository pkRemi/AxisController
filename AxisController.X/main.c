/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <p30Fxxxx.h>      /* Device header file                              */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>      /* Includes delay definition                       */
#include <uart.h>
#include <stdio.h>         /*For text to serial port (remove if notneeded     */
//#include <libq.h>           /*For _itoaQ15 integer to string*/
//#include <math.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "i2c.h"           /* I2C functions                                   */

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/


unsigned char ControlByte;		//Control Byte (I2C address)
unsigned char HighAdd;			//High Address byte
unsigned char LowAdd, HighAdd;	//Low and High Address byte
unsigned char Data;				//Data Byte
unsigned char Length;			//Length of Bytes to Read
unsigned char PageString[64];	//Array to hold page data to/from I2C device
char serString[64] = {'H','e','l','l','o',' ','w','o','r','l','d','!'};	//Array to hold page data to/from I2C device


/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();
    while(1)
    {
        _LATD0 = 0;
        __delay32(30000);
        _LATD0 = 1;
        __delay32(30000);
    }
    initSerial();
    InitI2C();
    /* I2C control variables */
    ControlByte = 0x00D0; // mpu6050 address
    LowAdd = 0x00;
    HighAdd = 0x5A;
    Data = 0xAA;
    Length = 0x01;
    __delay32(1500000);
    LDByteWriteI2C(ControlByte, 0x6B, 0x00); // Wakeup MPU6050
    unsigned int i;
    int serStringN;
    unsigned char TemperatureRAW[2] = {11,22};
    float TemperatureC = 11.11;
    while(1)
    {
        _LATD0 = 0;
        __delay32(1500000);
        _LATD0 = 1;
//        serStringN = sprintf(serString, "Sensor reading: %d \n\r", Data);
        TemperatureC = (TemperatureRAW[1]+TemperatureRAW[0]*255)/340+36.53;
        serStringN = sprintf(serString, "Sensor reading: %d %d Temp:%f\n\r", TemperatureRAW[0], TemperatureRAW[1],TemperatureC);
        //U2TXREG = Data; // Transmit one character
        for (i = 0; i < serStringN; i = i++)
        {
            while(!U2STAbits.TRMT);
            U2TXREG = serString[i];
        }
        __delay32(1500000);
        LDByteReadI2C(ControlByte, 0x41, &TemperatureRAW[0], 1);
        LDByteReadI2C(ControlByte, 0x42, &TemperatureRAW[1], 1);
        //I2C Commands
        //LDByteWriteI2C(ControlByte, LowAdd, Data);
        //HDByteWriteI2C(ControlByte, HighAdd, LowAdd, Data);
        //HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString);
        //LDByteReadI2C(ControlByte, LowAdd, &Data, 1);
        //HDByteReadI2C(ControlByte, HighAdd, LowAdd, &Data, 1);
        //HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PAGESIZE);
    }

}
