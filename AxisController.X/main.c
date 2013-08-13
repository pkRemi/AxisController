/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__PIC24E__)
    	#include <p24Exxxx.h>
    #elif defined (__PIC24F__)||defined (__PIC24FK__)
	#include <p24Fxxxx.h>
    #elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
    #endif
#endif

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>       /*included delay function */
#include <math.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */


/******************************************************************************/
/* Function prototypes                                               */
/******************************************************************************/


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
long int mot1speed=0;
float mot1direction=0;
long int mot2speed=0;
float mot2direction=0;
long int mot3speed=0;
float mot3direction=0;
float bbbspeed=0;
float angle =0;
signed int int1direction=0;
signed int int2direction=0;
signed int int3direction=0;
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
    /* TODO <INSERT USER APPLICATION CODE HERE> */

    while(1)
    {
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

        xvelocity = ADCvalue1-512;
        yvelocity = ADCvalue2-512;

        bbbspeed = sqrt(xvelocity*xvelocity + yvelocity*yvelocity);  //Calculate the magnitude of the combined two vectors

        angle = atan2(yvelocity,xvelocity);     //Calculate the driving angle to balance the robot

        mot1direction = bbbspeed*cos(1.570796327 -angle);    //mot1 at 0 degrees from x axis driving to 90 degree direction

        mot2direction = bbbspeed*cos(3.665191429 - angle); //mot2 at 120 degree from x axis driving to 210 degree

        mot3direction = bbbspeed*cos(5.759586532 - angle); //mot3 at 240 degrees from x axis driving to 330 degrees

        mot1speed = 3200/(fabs(mot1direction));         //Calculate appropriate driving speed so that
        mot2speed = 3200/(fabs(mot2direction));         //driving speeds never go too slow and so that
        mot3speed = 3200/(fabs(mot3direction));         //and so that motor is fast enough
 
        if(mot1direction<0)
    {
        int1direction=0;
    }
    else
    {
        int1direction=1;
    }

         if(mot2direction<0)
    {
        int2direction=0;
    }
    else
    {
        int2direction=1;
    }

         if(mot3direction<0)
    {
        int3direction=0;
    }
    else
    {
        int3direction=1;
    }

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
   _LATA4 = int1direction;
    PR1 = (mot1speed); //Load the Period register with the value ADCvalue
    _LATB5 =(1-_LATB5);         //Motor1 step
IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag and Return from ISR
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt(void)
{
   
/* Interrupt Service Routine code goes here */
    _LATB6 = int2direction;
    PR2 = (mot2speed); //Load the Period register with the value ADCvalue
    _LATB7=(1-_LATB7);          //Motor2 step
IFS0bits.T2IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}

void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void)
{
   
/* Interrupt Service Routine code goes here */
    _LATB8 = int3direction;
    PR3 = (mot3speed); //Load the Period register with the value ADCvalue
    _LATB9=(1-_LATB9);          //Motor3 step
IFS0bits.T3IF = 0; //Reset Timer2 interrupt flag and Return from ISR
}

