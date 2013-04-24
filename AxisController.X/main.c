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
//unsigned char HighAdd;			//High Address byte
//unsigned char LowAdd, HighAdd;	//Low and High Address byte
//unsigned char Data;				//Data Byte
//unsigned char Length;			//Length of Bytes to Read
unsigned char PageString[64];	//Array to hold page data to/from I2C device
char serString[64] = {'H','e','l','l','o',' ','w','o','r','l','d','!','\n'};	//Array to hold page data to/from I2C device
double delaytime = 1500000;
double countpos = 0;
int TemperatureRAW = 100;
float TemperatureC = 11.11;
int accel[3];
int gyro[3];
//unsigned int iaccel;
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
    __delay32(1500000);
    ControlByte = 0x00D0; // mpu6050 address
    InitMPU6050(ControlByte);
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
//        _LATD0 = 1;
        readSensorData();
        _LATD0 = 0;

//        serStringN = sprintf(serString, "Sensor reading: %d \n\r", Data);
        TemperatureC = (TemperatureRAW)/340+36.53;
        /* The following sprintf command takes 18.744ms or 187440 instructions!!! */
//        serStringN = sprintf(serString, "Sensor: %04X Accel: %04X %04X %04X Gyro: %04X %04X %04X\n\r",
        serStringN = sprintf(serString, "Sensor: %3.0f Accel: %05d %05d %05d Gyro: %05d %05d %05d\n\r",
                TemperatureC,
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2]);

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
    int i;
    int ii = 0;
    for (i = 0; i < 3; i++)
    {
        accel[i] = rawsensor[ii] << 8;
        ii++;
        accel[i] = accel[i] | rawsensor[ii];
        ii++;
    }
    TemperatureRAW = rawsensor[ii] << 8;
    ii++;
    TemperatureRAW = TemperatureRAW | rawsensor[ii];
    ii++;
    for (i = 0; i < 3; i++)
    {
        gyro[i] = rawsensor[ii] << 8;
        ii++;
        gyro[i] = gyro[i] | rawsensor[ii];
        ii++;
    }
        //I2C Commands
        //LDByteWriteI2C(ControlByte, LowAdd, Data);
        //HDByteWriteI2C(ControlByte, HighAdd, LowAdd, Data);
        //HDPageWriteI2C(ControlByte, HighAdd, LowAdd, PageString);
        //LDByteReadI2C(ControlByte, LowAdd, &Data, 1);
        //HDByteReadI2C(ControlByte, HighAdd, LowAdd, &Data, 1);
        //HDSequentialReadI2C(ControlByte, HighAdd, LowAdd, PageString, PAGESIZE);
}
