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
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void);

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
long int ADCvalue1=0;
long int ADCvalue2=0;
long int xvelocity=0;
long int yvelocity=0;
//long int mot1speed=0;
//long int mot2speed=0;
//long int mot3speed=0;
long motdir[3] = {0,0,0};
//float mot1direction=0;
//float mot2direction=0;
//float mot3direction=0;
long actualmotdir[3]= {0,0,0};
long lastdelay[3]= {0,0,0};
float bbbspeed=0;
float angle =0;
int intdirection[3] = {0,0,0};
int maxSpeed = 5000;
int minSpeed = 30;
int maxChange = 15;
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
    long motdirtemp[3] = {0,0,0};
    int i;
    int oorFlag = 0;
    unsigned int command = 0;
    signed int xSpeed = 0;
    signed int ySpeed = 0;
    float angleOffset = 0;
    unsigned int *chptr;                // For receiving a float value
    bool spiInt = 0;
    while(1)
    {
        spiInt = IFS0bits.SPI1IF;
        if (spiInt)
        {
            _LATB11 = 1;
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
            _LATB11 = 0;
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
//        maxChange = ADCvalue1/10;
//        xvelocity = (ADCvalue1-512)*10;
//        yvelocity = (ADCvalue2-512)*10;
        xvelocity = xSpeed;
        yvelocity = ySpeed;
        _LATB11 = 1;
        bbbspeed = sqrt(xvelocity*xvelocity + yvelocity*yvelocity);  //Calculate the magnitude of the combined two vectors
        _LATB10 = 1;
        angle = atan2(yvelocity,xvelocity) + angleOffset;  //Calculate the driving angle to balance the robot
        _LATB10 = 0;

        motdirtemp[0] = bbbspeed*cos(1.570796327 -angle);  //mot1 at 0 degrees from x axis driving to 90 degree direction
        _LATB10 = 1;

        motdirtemp[1]= bbbspeed*cos(3.665191429 - angle); //mot2 at 120 degree from x axis driving to 210 degree
        _LATB10 = 0;

        motdirtemp[2] = bbbspeed*cos(5.759586532 - angle); //mot3 at 240 degrees from x axis driving to 330 degrees
        _LATB11 =0;

        for (i=0;i<3;i++)
        {
            oorFlag = 0;
            if (motdirtemp[i]>0)
            {
                if (motdirtemp[i] > 5208)     //maxSpeed
                {
                    motdir[i] = 5208;
                    oorFlag = 1;
                }
                else if (motdirtemp[i] < 60)  //minSpeed (step frequency)
                {
                    motdir[i] = 60;
                    oorFlag = 1;
                }
            }
            else
            {
                if (motdirtemp[i] < -5208)
                {
                    motdir[i] = -5208;
                    oorFlag = 1;
                }
                else if (motdirtemp[i] > -60)
                {
                    motdir[i] = -60;
                    oorFlag = 1;
                }
            }
            if (!oorFlag)
                motdir[i] = motdirtemp[i];
        }
//        mot1speed = 3200/(fabs(mot1direction));         //Calculate appropriate driving speed so that
//        mot2speed = 3200/(fabs(mot2direction));         //driving speeds never go too slow and so that
//        mot3speed = 3200/(fabs(mot3direction));         //and so that motor is fast enough
//
//        if(mot1direction<0)
//        {
//            int1direction=0;
//        }
//        else
//        {
//            int1direction=1;
//        }
//
//        if(mot2direction<0)
//        {
//            int2direction=0;
//        }
//        else
//        {
//            int2direction=1;
//        }
//
//        if(mot3direction<0)
//        {
//            int3direction=0;
//        }
//        else
//        {
//            int3direction=1;
//        }

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


void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
/* Interrupt Service Routine code goes here */
    int motnr = 0;
    int delay;
    int change;
    int absmotdir = 0;

    change = actualmotdir[motnr] - motdir[motnr];
    if ( change < maxChange && change > - maxChange)
    {
        actualmotdir[motnr] = motdir[motnr];
    }
    else
    {
        if (change>0)
            actualmotdir[motnr] = actualmotdir[motnr] - (lastdelay[motnr] >> 3);
        if (change<0)
            actualmotdir[motnr] = actualmotdir[motnr] + (lastdelay[motnr] >> 3);
    }
    if (actualmotdir[motnr]>0)
    {
        intdirection[motnr] = 1;
        absmotdir = actualmotdir[motnr];
    }
    else
    {
        intdirection[motnr] = 0;
        absmotdir = -actualmotdir[motnr];
    }
    if (absmotdir <= 3264)
        delay = 125000/absmotdir;
    else if (absmotdir>5102)
        delay = 24;
    else if (absmotdir>4907)
        delay = 25;
    else if (absmotdir>4716)
        delay = 26;
    else if (absmotdir>4545)
        delay = 27;
    else if (absmotdir>4385)
        delay = 28;
    else if (absmotdir>4237)
        delay = 29;
    else if (absmotdir>4098)
        delay = 30;
    else if (absmotdir>3968)
        delay = 31;
    else if (absmotdir>3446)
        delay = 32;
    else if (absmotdir>3731)
        delay = 33;
    else if (absmotdir>3623)
        delay = 34;
    else if (absmotdir>3521)
        delay = 35;
    else if (absmotdir>3424)
        delay = 36;
    else if (absmotdir>3333)
        delay = 37;
    else if (absmotdir>3246)
        delay = 38;
    lastdelay[motnr] = delay;
    PR1 = delay; //Load the Period register with the value ADCvalue
    _LATA4 = intdirection[0];
    _LATB5 =(1-_LATB5);         //Motor1 step
    IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void)
{
   
/* Interrupt Service Routine code goes here */
    int motnr = 1;
    int delay;
    int change;
    int absmotdir = 0;
    change = actualmotdir[motnr] - motdir[motnr];
    if ( change < maxChange && change > - maxChange)
    {
        actualmotdir[motnr] = motdir[motnr];
    }
    else
    {
        if (change>0)
            actualmotdir[motnr] = actualmotdir[motnr] - (lastdelay[motnr] >> 3);
        if (change<0)
            actualmotdir[motnr] = actualmotdir[motnr] + (lastdelay[motnr] >> 3);
    }
    if (actualmotdir[motnr]>0)
    {
        intdirection[motnr] = 1;
        absmotdir = actualmotdir[motnr];
    }
    else
    {
        intdirection[motnr] = 0;
        absmotdir = -actualmotdir[motnr];
    }
    if (absmotdir <= 3264)
        delay = 125000/absmotdir;
    else if (absmotdir>5102)
        delay = 24;
    else if (absmotdir>4907)
        delay = 25;
    else if (absmotdir>4716)
        delay = 26;
    else if (absmotdir>4545)
        delay = 27;
    else if (absmotdir>4385)
        delay = 28;
    else if (absmotdir>4237)
        delay = 29;
    else if (absmotdir>4098)
        delay = 30;
    else if (absmotdir>3968)
        delay = 31;
    else if (absmotdir>3446)
        delay = 32;
    else if (absmotdir>3731)
        delay = 33;
    else if (absmotdir>3623)
        delay = 34;
    else if (absmotdir>3521)
        delay = 35;
    else if (absmotdir>3424)
        delay = 36;
    else if (absmotdir>3333)
        delay = 37;
    else if (absmotdir>3246)
        delay = 38;
    lastdelay[motnr] = delay;
    PR2 = delay; //Load the Period register with the value ADCvalue
    _LATB6 = intdirection[1];
    _LATB7=(1-_LATB7);          //Motor2 step
    IFS0bits.T2IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}

void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void)
{
   
/* Interrupt Service Routine code goes here */
    int motnr = 2;
    int delay;
    int change;
    int absmotdir = 0;
    change = actualmotdir[motnr] - motdir[motnr];
    if ( change < maxChange && change > - maxChange)
    {
        actualmotdir[motnr] = motdir[motnr];
    }
    else
    {
        if (change>0)
            actualmotdir[motnr] = actualmotdir[motnr] - (lastdelay[motnr] >> 3);
        if (change<0)
            actualmotdir[motnr] = actualmotdir[motnr] + (lastdelay[motnr] >> 3);
    }
    if (actualmotdir[motnr]>0)
    {
        intdirection[motnr] = 1;
        absmotdir = actualmotdir[motnr];
    }
    else
    {
        intdirection[motnr] = 0;
        absmotdir = -actualmotdir[motnr];
    }
    if (absmotdir <= 3264)
        delay = 125000/absmotdir;
    else if (absmotdir>5102)
        delay = 24;
    else if (absmotdir>4907)
        delay = 25;
    else if (absmotdir>4716)
        delay = 26;
    else if (absmotdir>4545)
        delay = 27;
    else if (absmotdir>4385)
        delay = 28;
    else if (absmotdir>4237)
        delay = 29;
    else if (absmotdir>4098)
        delay = 30;
    else if (absmotdir>3968)
        delay = 31;
    else if (absmotdir>3446)
        delay = 32;
    else if (absmotdir>3731)
        delay = 33;
    else if (absmotdir>3623)
        delay = 34;
    else if (absmotdir>3521)
        delay = 35;
    else if (absmotdir>3424)
        delay = 36;
    else if (absmotdir>3333)
        delay = 37;
    else if (absmotdir>3246)
        delay = 38;
    lastdelay[motnr] = delay;
    PR3 = delay; //Load the Period register with the value ADCvalue
    _LATB8 = intdirection[2];
    _LATB9=(1-_LATB9);          //Motor3 step
    IFS0bits.T3IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}
