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

#include <stdint.h>          /* For uint32_t definition */
#include <stdbool.h>         /* For true/false definition */

#include "user.h"            /* variables/params used by user.c */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    _LATB2 = 0; // RESET STEPPER
    _LATB4 = 0; // ENABLE STEPPER
    /* Setup analog functionality and port direction */
    TRISA=0b00011;       // Set RA0 and RA1 pins as inputs, rest as ouputs
    TRISB=0b0000000000000000;       // Set RBx pins as outputs
    AD1PCFG=0b1111111111111100;     //set RA0 as AN0  and RA1 as AN0 analog input
    //AD1PCFG = 0xEFFF; // all PORTB = Digital; RB12 = analog
    /* Initialize peripherals */
    _LATB2 = 0; // RESET STEPPER
    _LATB4 = 0; // ENABLE STEPPER
    __delay32(16000000);
    _LATB2 = 1; // RESET STEPPER
    __delay32(16000000);
    _LATB4 = 1; // ENABLE STEPPER

}

void setuptimer1 (void)
{
    /* The following code example will enable Timer1 interrupts, load the Timer1
Period register and start Timer1.
When a Timer1 period match interrupt occurs, the interrupt service
routine must clear the Timer1 interrupt status flag in software.
*/
T1CON = 0x00; //Stops the Timer1 and reset control reg.
TMR1 = 0x00; //Clear contents of the timer register
PR1 = 62500; //Load the Period register with the value 62500
IPC0bits.T1IP = 0x01; //Setup Timer1 interrupt for desired priority level
// (This example assigns level 1 priority)
T1CONbits.TCKPS =3; //Prescale 1:256
IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
IEC0bits.T1IE = 1; //Enable Timer1 interrupts
T1CONbits.TON = 1; //Start Timer1 with prescaler settings at 1:1 and

//clock source set to the internal instruction cycle
/* Example code for Timer1 ISR*/
}

void setuptimer2 (void)
{
    /* The following code example will enable Timer1 interrupts, load the Timer1
Period register and start Timer1.
When a Timer1 period match interrupt occurs, the interrupt service
routine must clear the Timer1 interrupt status flag in software.
*/
T2CON = 0x00; //Stops the Timer1 and reset control reg.
TMR2 = 0x00; //Clear contents of the timer register
PR2 = 62500; //Load the Period register with the value 62500
IPC1bits.T2IP = 0x01; //Setup Timer1 interrupt for desired priority level
// (This example assigns level 1 priority)
T2CONbits.TCKPS =3; //Prescale 1:256
IFS0bits.T2IF = 0; //Clear the Timer2 interrupt status flag
IEC0bits.T2IE = 1; //Enable Timer2 interrupts
T2CONbits.TON = 1; //Start Timer2 with prescaler settings at 1:1 and

//clock source set to the internal instruction cycle
/* Example code for Timer2 ISR*/
}

void setuptimer3 (void)
{
    /* The following code example will enable Timer1 interrupts, load the Timer3
Period register and start Timer1.
When a Timer1 period match interrupt occurs, the interrupt service
routine must clear the Timer1 interrupt status flag in software.
*/
T3CON = 0x00; //Stops the Timer3 and reset control reg.
TMR3 = 0x00; //Clear contents of the timer register
PR3 = 65000; //Load the Period register with the value 62500
IPC2bits.T3IP = 0x01; //Setup Timer3 interrupt for desired priority level
// (This example assigns level 1 priority)
T3CONbits.TCKPS =3; //Prescale 1:256
IFS0bits.T3IF = 0; //Clear the Timer3 interrupt status flag
IEC0bits.T3IE = 1; //Enable Timer3 interrupts
T3CONbits.TON = 1; //Start Timer3 with prescaler settings at 1:1 and

//clock source set to the internal instruction cycle
/* Example code for Timer3 ISR*/
}

//Setup ADC for AN0
void setupADC (void)
{

        
         
       
    AD1CON1bits.FORM = 0b00;  //Result as integer
    AD1CON1bits.SSRC = 0b111; //Automatic conversion
    AD1CON1bits.ASAM = 0;   //Sampling begins when SAMP bit is set in main.c
     AD1CON2bits.VCFG = 0b000; //Voltage references as Vdd and Vss
    AD1CON2bits.CSCNA = 0;  //no input scan form MUX A, change this when adding more ADCs
    AD1CON2bits.BUFS = 0;   //Registers ADC1BUF0-ADC1BUF7 are used
    AD1CON2bits.SMPI = 0b0000;    //Interupt after every conversion
    AD1CON2bits.BUFM = 0;   //Buffer configured as one 16-word buffer (ADC1BUF0 to ADC1BUFF)
    AD1CON2bits.ALTS = 0;   //no alternation on MUX A and MUX B input multiplexer settings

    AD1CON3bits.ADRC = 0;   //Uses system defined clock
    AD1CON3bits.SAMC = 0b1111;   //Autosample = 31Tad
    AD1CON3bits.ADCS = 0b00000010;    //ADC conversion clock period 3xTcy
    //AD1CSSL = 0b0000000000000000; // Include all channels in scan

    

   IEC0bits.AD1IE  = 0; // disable A/D conversion interrupt
   IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt
    

/*
AD1CON1 = 0x00E0;  // SSRC<2:0> = 111 implies internal counter ends sampling
// and starts converting.
AD1CHS  = 0x000C;   // Connect AN12 as S/H input.
// in this example AN12 is the input
AD1CSSL = 0;
AD1CON3 = 0x1F02;  // Sample time = 31Tad, Tad = 3Tcy
AD1CON2 = 0;
AD1CON1bits.ADON = 1;  // turn ADC ON
*/
}
