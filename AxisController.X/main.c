/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <p24Fxxxx.h>
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>      /* Included delay function                         */
#include <math.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */


/******************************************************************************/
/* Function prototypes                                                        */
/******************************************************************************/
int limitSpeed(int motnr);

/* Default interrupt handler */
void __attribute__((interrupt,no_auto_psv)) _DefaultInterrupt(void);
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void);
void __attribute__((interrupt, auto_psv)) _T3Interrupt(void);

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
long int ADCvalue1=0;
long int ADCvalue2=0;
long int xvelocity=0;
long int yvelocity=0;
volatile float motdir[3] = {0,0,0};
//volatile long actualmotdir[3] = {0,0,0}; // may not be needed
volatile float targetdelay[3] = {0,0,0};
volatile float lastdelay[3]= {1250,1250,1250};
float bbbspeed=0;
float angle =0;
volatile int intdirection[3] = {0,0,0};
volatile float maxSpeed = 4000;
volatile float minSpeed = 200; // minSpeed is dependent on maxAcceleration (16MHz/64)/maxDelay
//volatile int maxChange = 15;

volatile float maxDelay = 1250;       // Max delay calculated from (16MHz/64)/sqrt(2*maxAcceleration)
volatile float maxAcceleration = 20000; // Stepper motor acceleration in steps/(second^2)
volatile float delaymultiplierR = 0;  // Constant for delay calculation = a/(F^2)
/* i.e. uint16_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();
    setuptimer1();
    setuptimer2();
    setuptimer3();
    setupADC();
    initSPI1();

    /* Local Variables */
    float motdirtemp[3] = {0,0,0};
    int i;
    int oorFlag = 0;
    unsigned int command = 0;
    signed int xSpeed = 0;
    signed int ySpeed = 0;
    float angleOffset = 0;
    unsigned int *chptr;                // For receiving a float value
    bool spiInt = 0;
    int simState = 0;
    while(1)
    {
        spiInt = IFS0bits.SPI1IF;
        if (spiInt)
        {
            command = SPI1BUF;
            while(SPI1STATbits.SRXMPT); // Wait if Buffer is empty
            xSpeed = SPI1BUF;
            while(SPI1STATbits.SRXMPT);
            ySpeed = SPI1BUF;
            chptr = (unsigned char *) &angleOffset;  // For receiving a float value
            while(SPI1STATbits.SRXMPT);
            *chptr++ = SPI1BUF;
            while(SPI1STATbits.SRXMPT);
            *chptr++ = SPI1BUF;
            IFS0bits.SPI1IF = 0;
        }
        if (command & 0b0100000000000000) // Check if bit14 is set (enable)
        {
            _LATB4 = 1; // ENABLE STEPPER
        }
        else
        {
            _LATB4 = 0; // DISABLE STEPPER
        }

        AD1CON1bits.ADON = 0; // ADC off
        AD1CHS = 0x0000;
        AD1CON1bits.ADON = 1; // ADC on
        AD1CON1bits.SAMP = 1;   //Start ADConversion

        while (!AD1CON1bits.DONE){}; // conversion done?
        ADCvalue1 = ADC1BUF0;  // yes then get ADC value
        
        AD1CON1bits.ADON = 0; // ADC off
        AD1CHS = 0x0001;
        AD1CON1bits.ADON = 1; // ADC on
        AD1CON1bits.SAMP = 1;   //Start ADConversion

        while (!AD1CON1bits.DONE){}; // conversion done?
        ADCvalue2 = ADC1BUF0;  // yes then get ADC value
//        maxAcceleration = ADCvalue1;
        delaymultiplierR = maxAcceleration/62500000000.0; // (16MHz/64)^2
//        xvelocity = (ADCvalue1-512)*10;
//        yvelocity = (ADCvalue2-512)*10;
        _LATB11 = 1;
        xvelocity = xSpeed;
        yvelocity = ySpeed;
//        xvelocity = 30000;
//        yvelocity = 30000;
        /** Simulate going to max speed and reversing *************************/
//        if (simState == 0) // initial state
//        {
//            xvelocity = 30000;
//            yvelocity = 0;
//            simState = 1;
//        }
//        else if (simState == 1 && lastdelay[2] <= 50.0) // reverse when max forward speed is reached
//        {
//            xvelocity = -30000;
//            yvelocity = 0;
//            simState = 2;
//        }
//        else if (simState == 2 && lastdelay[2] < 0.0)
//        {
//            simState = 3;
//        }
//        else if (simState == 3 && lastdelay[2] >= -50.0) // forward when max reverse speed is reached
//        {
//            xvelocity = 30000;
//            yvelocity = 0;
//            simState = 4;
//        }
//        else if (simState == 4 && lastdelay[2] > 0.0)
//        {
//            simState = 1;
//        }
        bbbspeed = sqrt(xvelocity*xvelocity + yvelocity*yvelocity);  //Calculate the magnitude of the combined two vectors
        angle = atan2(yvelocity,xvelocity) + angleOffset;  //Calculate the driving angle to balance the robot
        motdirtemp[0] = bbbspeed*cos(1.570796327 -angle);  //mot1 at 0 degrees from x axis driving to 90 degree direction
        motdirtemp[1]= bbbspeed*cos(3.665191429 - angle); //mot2 at 120 degree from x axis driving to 210 degree
        motdirtemp[2] = bbbspeed*cos(5.759586532 - angle); //mot3 at 240 degrees from x axis driving to 330 degrees

        for (i=0;i<3;i++)
        {
            oorFlag = 0;
            if (motdirtemp[i]>0)
            {
                if (motdirtemp[i] > maxSpeed)     //maxSpeed
                {
                    motdir[i] = maxSpeed;
                    oorFlag = 1;
                }
                else if (motdirtemp[i] < minSpeed)  //minSpeed (step frequency)
                {
                    motdir[i] = minSpeed;
                    oorFlag = 1;
                }
            }
            else
            {
                if (motdirtemp[i] < -maxSpeed)
                {
                    motdir[i] = -maxSpeed;
                    oorFlag = 1;
                }
                else if (motdirtemp[i] > -minSpeed)
                {
                    motdir[i] = -minSpeed;
                    oorFlag = 1;
                }
            }
            if (!oorFlag)
                motdir[i] = motdirtemp[i];
            targetdelay[i] = 250000.0/motdir[i];   /** Change speed to delay **/
        }
    _LATB11 = 0;



    }
}

/******************************************************************************/
/* Default Interrupt Handler                                                  */
/*                                                                            */
/* This executes when an interrupt occurs for an interrupt source with an     */
/* improperly defined or undefined interrupt handling routine.                */
/******************************************************************************/
void __attribute__((interrupt,no_auto_psv)) _DefaultInterrupt(void)
{
        while(1);
}


void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
/* Interrupt Service Routine code goes here */
    int motnr = 0;
    float delay;
    int delaytemp;
    float change;
    _LATB5 = 1;         //Motor1 step
    change = targetdelay[motnr] - lastdelay[motnr];

    if (targetdelay[motnr] > 0 && lastdelay[motnr] > 0) // both positive cw
    {
        intdirection[motnr] = 1;
        if (change < 0)                                 //change negative
        {
            // accelerate cw
            delay = lastdelay[motnr] * (1 - delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay < targetdelay[motnr])
                delay = targetdelay[motnr];
        }
        else
        {
            // decelerate cw
            delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay > targetdelay[motnr])
                delay = targetdelay[motnr];
        }
    }
    else if(targetdelay[motnr] < 0 && lastdelay[motnr] < 0) //both negative ccw
    {
        intdirection[motnr] = 0;

        if (change > 0)
        {
            // accelerate ccw
            delay = lastdelay[motnr] * (1 - delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay > targetdelay[motnr])
                delay = targetdelay[motnr];
        }
        else
        {
            // decelerate ccw
            delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay < targetdelay[motnr])
                delay = targetdelay[motnr];
        }
    }

    else
    {
        // decelerate and switch direction
        delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
        if (lastdelay[motnr] >0)
        {
            // Decelerating cw
            if (delay > maxDelay)
            {
                delay = -maxDelay;      // Change direction if going to slow
                intdirection[motnr] = 0; // ccw
            }

        }
        else
        {
            // Decelerating ccw
            if (delay < -maxDelay)
            {
                delay = maxDelay;      // Change direction if going to slow
                intdirection[motnr] = 1; // cw
            }
        }

    }
    lastdelay[motnr] = delay;
    if (intdirection[motnr])
        delaytemp = delay;  // Change from float to int
    else
        delaytemp = -delay;
    PR1 = delaytemp; //Load the Period register with the delay
    _LATA4 = intdirection[0];
    _LATB5 = 0;         //Motor1 end step
    IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
   
/* Interrupt Service Routine code goes here */
    int motnr = 1;
    float delay;
    int delaytemp;
    float change;
    _LATB7 = 1;         //Motor2 step
    change = targetdelay[motnr] - lastdelay[motnr];

    if (targetdelay[motnr] > 0 && lastdelay[motnr] > 0) // both positive cw
    {
        intdirection[motnr] = 1;
        if (change < 0)                                 //change negative
        {
            // accelerate cw
            delay = lastdelay[motnr] * (1 - delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay < targetdelay[motnr])
                delay = targetdelay[motnr];
        }
        else
        {
            // decelerate cw
            delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay > targetdelay[motnr])
                delay = targetdelay[motnr];
        }
    }
    else if(targetdelay[motnr] < 0 && lastdelay[motnr] < 0) //both negative ccw
    {
        intdirection[motnr] = 0;

        if (change > 0)
        {
            // accelerate ccw
            delay = lastdelay[motnr] * (1 - delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay > targetdelay[motnr])
                delay = targetdelay[motnr];
        }
        else
        {
            // decelerate ccw
            delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
            if (delay < targetdelay[motnr])
                delay = targetdelay[motnr];
        }
    }

    else
    {
        // decelerate and switch direction
        delay = lastdelay[motnr] * (1 + delaymultiplierR * lastdelay[motnr] * lastdelay[motnr]);
        if (lastdelay[motnr] >0)
        {
            // Decelerating cw
            if (delay > maxDelay)
            {
                delay = -maxDelay;      // Change direction if going to slow
                intdirection[motnr] = 0; // ccw
            }

        }
        else
        {
            // Decelerating ccw
            if (delay < -maxDelay)
            {
                delay = maxDelay;      // Change direction if going to slow
                intdirection[motnr] = 1; // cw
            }
        }

    }
    lastdelay[motnr] = delay;
    if (intdirection[motnr])
        delaytemp = delay;  // Change from float to int
    else
        delaytemp = -delay;
    PR2 = delaytemp; //Load the Period register with the delay
    _LATB6 = intdirection[1];
    _LATB7 = 0;          //Motor2 end step
    IFS0bits.T2IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}

void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{

/* Interrupt Service Routine code goes here */
    int motnr = 2;
    float delay;
    int delaytemp;
    float change;
    _LATB9 = 1;         //Motor3 step
_LATB10 = 1;
    change = targetdelay[2] - lastdelay[2];
_LATB10 = 0;

    if (targetdelay[2] > 0 && lastdelay[2] > 0) // both positive cw
    {
        intdirection[2] = 1;
        if (change < 0)                                 //change negative
        {
            // accelerate cw
_LATB10 = 1;
            delay = lastdelay[2] * (1 - delaymultiplierR * lastdelay[2] * lastdelay[2]);
_LATB10 = 0;
            if (delay < targetdelay[2])
                delay = targetdelay[2];
        }
        else
        {
            // decelerate cw
_LATB10 = 1;
            delay = lastdelay[2] * (1 + delaymultiplierR * lastdelay[2] * lastdelay[2]);
_LATB10 = 0;
            if (delay > targetdelay[2])
                delay = targetdelay[2];
        }
    }
    else if(targetdelay[2] < 0 && lastdelay[2] < 0) //both negative ccw
    {
        intdirection[2] = 0;

        if (change > 0)
        {
            // accelerate ccw
_LATB10 = 1;
            delay = lastdelay[2] * (1 - delaymultiplierR * lastdelay[2] * lastdelay[2]);
_LATB10 = 0;
            if (delay > targetdelay[2])
                delay = targetdelay[2];
        }
        else
        {
            // decelerate ccw
_LATB10 = 1;
            delay = lastdelay[2] * (1 + delaymultiplierR * lastdelay[2] * lastdelay[2]);
_LATB10 = 0;
            if (delay < targetdelay[2])
                delay = targetdelay[2];
        }
    }

    else
    {
        // decelerate and switch direction
_LATB10 = 1;
        delay = lastdelay[2] * (1 + delaymultiplierR * lastdelay[2] * lastdelay[2]);
_LATB10 = 0;
        if (lastdelay[2] >0)
        {
            // Decelerating cw
            if (delay > maxDelay)
            {
                delay = -maxDelay;      // Change direction if going to slow
                intdirection[2] = 0; // ccw
            }

        }
        else
        {
            // Decelerating ccw
            if (delay < -maxDelay)
            {
                delay = maxDelay;      // Change direction if going to slow
                intdirection[2] = 1; // cw
            }
        }

    }
_LATB10 = 1;
    lastdelay[2] = delay;
_LATB10 = 0;
    if (intdirection[2])
        delaytemp = delay;  // Change from float to int
    else
        delaytemp = -delay;
    PR3 = delaytemp; //Load the Period register with the delay
    _LATB8 = intdirection[2];
    _LATB9=0;          //Motor3 end step
    IFS0bits.T3IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}
//void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
//{
//
///* Interrupt Service Routine code goes here */
//    int motnr = 2;
//    int delay;
//    int change;
//    int absmotdir = 0;
//    _LATB9=1;          //Motor3 step
//    change = actualmotdir[motnr] - motdir[motnr];
//    if ( change < maxChange && change > - maxChange)
//    {
//        actualmotdir[motnr] = motdir[motnr];
//    }
//    else
//    {
//        if (change>0)
//            actualmotdir[motnr] = actualmotdir[motnr] - (lastdelay[motnr] >> 4);
//        if (change<0)
//            actualmotdir[motnr] = actualmotdir[motnr] + (lastdelay[motnr] >> 4);
//    }
//    if (actualmotdir[motnr]>0)
//    {
//        intdirection[motnr] = 1;
//        absmotdir = actualmotdir[motnr];
//    }
//    else
//    {
//        intdirection[motnr] = 0;
//        absmotdir = -actualmotdir[motnr];
//    }
//    if (absmotdir <= 3264)
//        delay = 125000/absmotdir;
//    else if (absmotdir>5102)
//        delay = 24;
//    else if (absmotdir>4907)
//        delay = 25;
//    else if (absmotdir>4716)
//        delay = 26;
//    else if (absmotdir>4545)
//        delay = 27;
//    else if (absmotdir>4385)
//        delay = 28;
//    else if (absmotdir>4237)
//        delay = 29;
//    else if (absmotdir>4098)
//        delay = 30;
//    else if (absmotdir>3968)
//        delay = 31;
//    else if (absmotdir>3446)
//        delay = 32;
//    else if (absmotdir>3731)
//        delay = 33;
//    else if (absmotdir>3623)
//        delay = 34;
//    else if (absmotdir>3521)
//        delay = 35;
//    else if (absmotdir>3424)
//        delay = 36;
//    else if (absmotdir>3333)
//        delay = 37;
//    else if (absmotdir>3246)
//        delay = 38;
//    lastdelay[motnr] = delay;
//    PR3 = delay; //Load the Period register with the value ADCvalue
//    _LATB8 = intdirection[2];
//    _LATB9=0;          //Motor3 end step
//    IFS0bits.T3IF = 0; //Reset Timer2 interrupt flag and Return from ISR
//}
