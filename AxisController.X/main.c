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
#include <math.h>          /* For double acos(double)                         */
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
char serString[64] = {'H','e','l','l','o',' ','w','o','r','l','d','!','\n'};	//Array to hold page data to/from I2C device
double delaytime = 1500000;
double countpos = 0;
unsigned char TemperatureRAW[2] = {11,22};
float TemperatureC = 11.11;
unsigned char accel[6];
unsigned char gyro[6];
unsigned int iaccel;
unsigned char rawsensor[14];

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();
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
    int serStringN = 13;
    delaytime = 1;
    while(1)
    {
//        _LATD0 = 0;
        __delay32(delaytime);
//        _LATD0 = 1;
        /* Send serial data to PC */
        for (i = 0; i < serStringN; i = i++)
        {
            while(!U2STAbits.TRMT);
            U2TXREG = serString[i];
        }
        __delay32(1000); // Without this delay, the I2C command acts funny...
        readSensorData();
        _LATD0 = 0;

//        serStringN = sprintf(serString, "Sensor reading: %d \n\r", Data);
        //TemperatureC = (TemperatureRAW[1]+TemperatureRAW[0]*255)/340+36.53;
        /* The following sprintf command takes 18.744ms or 187440 instructions!!! */
        serStringN = sprintf(serString, "Sensor: %02X%02X Accel: %02X%02X %02X%02X %02X%02X Gyro: %02X%02X %02X%02X %02X%02X %04X\n\r",
                TemperatureRAW[0], TemperatureRAW[1],
                accel[0], accel[1], accel[2], accel[3], accel[4], accel[5],
                gyro[0], gyro[1], gyro[2], gyro[3], gyro[4], gyro[5], iaccel);
        _LATD0 = 1;

        //U2TXREG = Data; // Transmit one character



        __delay32(delaytime);

//        calcdelay();
    }

}

void calcdelay(void)
{
    double tau = 6.2831;
    double cstep = 1*tau/(500);
    countpos = countpos + cstep;
    if (countpos >= tau)
        countpos = 0;
    delaytime = (fabs(cos(countpos))*50000+20000)*1;

}
void readSensorData(void)
{
//        LDByteReadI2C(ControlByte, 0x41, &TemperatureRAW[0], 1);
//        LDByteReadI2C(ControlByte, 0x42, &TemperatureRAW[1], 1);
//        LDByteReadI2C(ControlByte, 0x3B, &accel[0], 1);
//        LDByteReadI2C(ControlByte, 0x3C, &accel[1], 1);
//        LDByteReadI2C(ControlByte, 0x3D, &accel[2], 1);
//        LDByteReadI2C(ControlByte, 0x3E, &accel[3], 1);
//        LDByteReadI2C(ControlByte, 0x3F, &accel[4], 1);
//        LDByteReadI2C(ControlByte, 0x40, &accel[5], 1);
//        LDByteReadI2C(ControlByte, 0x43, &gyro[0], 1);
//        LDByteReadI2C(ControlByte, 0x44, &gyro[1], 1);
//        LDByteReadI2C(ControlByte, 0x45, &gyro[2], 1);
//        LDByteReadI2C(ControlByte, 0x46, &gyro[3], 1);
//        LDByteReadI2C(ControlByte, 0x47, &gyro[4], 1);
//        LDByteReadI2C(ControlByte, 0x48, &gyro[5], 1);
//        iaccel = accel[0] << 8;
//        iaccel = iaccel | accel[1];
    LDSequentialReadI2C(ControlByte, 0x3B, rawsensor,14);
        //I2C Commands
        //LDByteWriteI2C(ControlByte, LowAdd, Data);
        //HDByteWriteI2C(ControlByte, HighAdd, LowAdd, Data);
        //HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString);
        //LDByteReadI2C(ControlByte, LowAdd, &Data, 1);
        //HDByteReadI2C(ControlByte, HighAdd, LowAdd, &Data, 1);
        //HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PAGESIZE);
}
