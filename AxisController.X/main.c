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
unsigned int ReadSPI1();
void getsSPI( unsigned char *rdptr, unsigned char length );


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
    initSPI1();

    /* Local Variables */
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
/********************************************************************
*     Function Name:    getsSPI                                     *
*     Return Value:     void                                        *
*     Parameters:       address of read string storage location and *
*                       length of string bytes to read              *
*     Description:      This routine reads a string from the SPI    *
*                       bus.  The number of bytes to read is deter- *
*                       mined by parameter 'length'.                *
********************************************************************/
void getsSPI( unsigned char *rdptr, unsigned char length )
{
  while ( length )                  // stay in loop until length = 0
  {
    *rdptr++ = ReadSPI1();          // read a single byte
    length--;                       // reduce string length count by 1
  }
}
/******************************************************************************
*     Function Name :   ReadSPI1                                              *
*     Description   :   This function will read single byte/ word  from SPI   *
*                       bus. If SPI is configured for byte  communication     *
*                       then upper byte of SPIBUF is masked.                  *
*     Parameters    :   None                                                  *
*     Return Value  :   contents of SPIBUF register                           *
******************************************************************************/

unsigned int ReadSPI1()
{
    int spiin = 0x42;
    SPI1STATbits.SPIROV = 0;
  SPI1BUF = 0xFF;                  // initiate bus cycle , was 00 changed to FF
  while(!SPI1STATbits.SPIRBF);
  //__delay32(1000);
  /* Check for Receive buffer full status bit of status register*/
  if (SPI1STATbits.SPIRBF)
  {
      SPI1STATbits.SPIROV = 0;

      if (SPI1CON1bits.MODE16)
      {
          spiin = SPI1BUF;
          return (spiin);           /* return word read */

      }
      else
          spiin = SPI1BUF;
          return (spiin & 0xff);    /* return byte read */
  }
  return -1;                  		/* RBF bit is not set return error*/
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

