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
// Test for averaging
int n = 0;
float average = 0;
// Test of stepper motor speed control.
int axdelay = 4000;
int axdir = 0;
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
    SetupInterrupts();
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
        _LATD0 = 1;
       GyroZaverage();
        _LATD0 = 0;
        Pcontroller();
//        serStringN = sprintf(serString, "Sensor reading: %d \n\r", Data);
        TemperatureC = (TemperatureRAW)/340+36.53;
        /* The following sprintf command takes 18.744ms or 187440 instructions!!! */
//        serStringN = sprintf(serString, "Sensor: %04X Accel: %04X %04X %04X Gyro: %04X %04X %04X\n\r",
        serStringN = sprintf(serString, "Sensor: %3.0f Accel: %05d %05d %05d Gyro: %05d %05d %05d AZ: %f\n\r",
                TemperatureC,
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2], average);

        //U2TXREG = Data; // Transmit one character



        __delay32(delaytime);

//        calcdelay();
    }

}
void Pcontroller(void)
{
    // Setpoint, measured value, output - speed
    int axset = 200;
    float speed;
    speed = (axset - accel[0]);
    Speed2Delay(speed);
}
void Speed2Delay(float speed)
{
    // Speed is in steps/sec, max is:
    // 1 rps * 200 step/rev * 32 microstep/step * 2 int/step = 12800
    // Period = PR1 * prescaler * Tcy = 78 * 64 * 100ns = 2ms
    float delay = 0;
    if (fabs(speed) > 12800) // Maximum speed. Avoids overspeed on the motor and potential lockup of the interrupt handler.
        delay = 17;
    else if (fabs(speed) < 10) // Minimum speed. Makes sure that the interrupt is run at least 10 times per sec.
        delay = 21739;
    else
        delay = 1/(speed * 0.0000046);

    axdelay = fabs(delay);
    if (speed >= 0) // Check direction of speed
        axdir = 1;
    else
        axdir = 0;

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

    LDSequentialReadI2C(ControlByte, 0x3B, rawsensor,14); // Read sensor data
    // Put sensor data into 16 bit int variables
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

}
void GyroZaverage(void)
{

      n++; 
      average = average + ((gyro[2] - average)/n); 
}
// Timer 1 interrupt service routine toggles LED on RD1
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{

    // Toggle LED on RD1
    _LATD1 = 1 - _LATD1;
    PR1 = axdelay;
    _LATD3 = axdir;
    // Clear Timer 1 interrupt flag
    _T1IF = 0;


}
