/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <p30Fxxxx.h>      /* Device header file                              */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */

/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* Refer to the C30 (MPLAB C Compiler for PIC24F MCUs and dsPIC33F DSCs) User */
/* Guide for an up to date list of the available interrupt options.           */
/* Alternately these names can be pulled from the device linker scripts.      */
/*                                                                            */
/* Primary Interrupt Vector Names:                                            */
/*                                                                            */
/* _INT0Interrupt  _INT2Interrupt                                             */
/* _IC1Interrupt   _U2RXInterrupt                                             */
/* _OC1Interrupt   _U2TXInterrupt                                             */
/* _T1Interrupt    _SPI2Interrupt                                             */
/* _IC2Interrupt   _C1Interrupt                                               */
/* _OC2Interrupt   _IC3Interrupt                                              */
/* _T2Interrupt    _IC4Interrupt                                              */
/* _T3Interrupt    _IC5Interrupt                                              */
/* _SPI1Interrupt  _IC6Interrupt                                              */
/* _U1RXInterrupt  _OC5Interrupt                                              */
/* _U1TXInterrupt  _OC6Interrupt                                              */
/* _ADCInterrupt   _OC7Interrupt                                              */
/* _NVMInterrupt   _OC8Interrupt                                              */
/* _SI2CInterrupt  _INT3Interrupt                                             */
/* _MI2CInterrupt  _INT4Interrupt                                             */
/* _CNInterrupt    _C2Interrupt                                               */
/* _INT1Interrupt  _PWMInterrupt                                              */
/* _IC7Interrupt   _QEIInterrupt                                              */
/* _IC8Interrupt   _DCIInterrupt                                              */
/* _OC3Interrupt   _LVDInterrupt                                              */
/* _OC4Interrupt   _FLTAInterrupt                                             */
/* _T4Interrupt    _FLTBInterrupt                                             */
/* _T5Interrupt                                                               */
/*                                                                            */
/* For alternate interrupt vector naming, simply add 'Alt' between the prim.  */
/* interrupt vector name '_' and the first character of the primary interrupt */
/* vector name.                                                               */
/*                                                                            */
/* For example, the vector name _ADC2Interrupt becomes _AltADC2Interrupt in   */
/* the alternate vector table.                                                */
/*                                                                            */
/* Example Syntax:                                                            */
/*                                                                            */
/* void __attribute__((interrupt,auto_psv)) <Vector Name>(void)               */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more comprehensive interrupt examples refer to the C30 (MPLAB C        */
/* Compiler for PIC24 MCUs and dsPIC DSCs) User Guide in the                  */
/* <compiler installation directory>/doc directory for the latest compiler    */
/* release.                                                                   */
/*                                                                            */
/******************************************************************************/
/* Interrupt Prototypes                                                         */
/******************************************************************************/

void SetupInterrupts(void);


/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* TODO Add interrupt routine code here. */

void SetupInterrupts(void)
{
    // Configure Timer 1.
    // PR1 and TCKPS are set to call interrupt every 2ms.
    // Period = PR1 * prescaler * Tcy = 78 * 256 * 100ns = 2ms
    
    T1CON = 0;            // Clear Timer 1 configuration
    T1CONbits.TCKPS = 2;  // Set timer 1 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
    PR1 = 78;             // Set Timer 1 period (max value is 65535)
    _T1IP = 1;            // Set Timer 1 interrupt priority
    _T1IF = 0;            // Clear Timer 1 interrupt flag
    _T1IE = 1;            // Enable Timer 1 interrupt
    T1CONbits.TON = 1;    // Turn on Timer 1
}


///** I N T E R R U P T S ***********************************************/
//// From SERIAL example(Remi)
////----------------------------------------------------------------------------
//// High priority interrupt vector
//
//#pragma code InterruptVectorHigh = 0x08
//void InterruptVectorHigh (void)
//{
//  _asm
//    goto InterruptServiceHigh //jump to interrupt routine
//  _endasm
//}
//
////----------------------------------------------------------------------------
//// Low priority interrupt vector
//
//#pragma code InterruptVectorLow = 0x18
//void InterruptVectorLow (void)
//{
//  _asm
//    goto InterruptServiceLow //jump to interrupt routine
//  _endasm
//}
//void Timer0_Init(void)
//{
//    // Set up Interrupts for timer
//    INTCONbits.TMR0IF = 0;          // clear roll-over interrupt flag
//    INTCON2bits.TMR0IP = 1;         // Timer0 is high priority interrupt
//    INTCONbits.TMR0IE = 1;          // enable the Timer0 interrupt.
//    // Set up timer itself
//    T0CON = 0b00000000;             // prescale 1:4 - about 1 second maximum delay.
//    TMR0H = 0;                      // clear timer - always write upper byte first
//    TMR0L = 0;
//    T0CONbits.TMR0ON = 1;           // start timer
//}
//void prompt(void)
//{
//	printf("STEPPER ->");
//}
//// -------------------- Iterrupt Service Routines --------------------------
//#pragma interrupt InterruptServiceHigh  // "interrupt" pragma also for high priority
//void InterruptServiceHigh(void)
//{
//    // Check to see what caused the interrupt
//    // (Necessary when more than 1 interrupt at a priority level)
//
//    // Check for INT0 interrupt
//    if (INTCONbits.INT0IF)
//    {
//        // clear (reset) flag
//        INTCONbits.INT0IF = 0;
//    }
//    // Check for Timer0 Interrupt
//    if  (INTCONbits.TMR0IF)
//    {
//        INTCONbits.TMR0IF = 0;          // clear (reset) flag
//		if (direction == 0)
//		{
//			LATA = 0;
//		}
//		else if (direction == 1)
//		{
//		PatternNr++;
//		if(PatternNr > 3)
//			PatternNr = 0;
//		LATA = PatternLookup[PatternNr];
//		if (relativeSteps < 65535)
//		{
//			relativeSteps--;
//			if (relativeSteps == 0)
//				direction = 0;
//		}
//		}
//		else if (direction == 2)
//		{
//		PatternNr--;
//		if(PatternNr > 4)
//			PatternNr = 3;
//		LATA = PatternLookup[PatternNr];
//		if (relativeSteps < 65535)
//		{
//			relativeSteps--;
//			if (relativeSteps == 0)
//				direction = 0;
//		}
//		}
//		TMR0H = 255;      // MSB from serial input
//		TMR0L = Speed;                  // LSB = 0
//	}
//    // Check for another interrupt, examples:
//    // if (PIR1bits.TMR1IF)     // Timer 1
//    // if (PIR1bits.ADIF)       // ADC
//
//}  // return from high-priority interrupt
//
//#pragma interruptlow InterruptServiceLow// "interruptlow" pragma for low priority
//void InterruptServiceLow(void)
//{
//    // Check to see what caused the interrupt
//    // (Necessary when more than 1 interrupt at a priority level)
//
//    // Check for Timer0 Interrupt
//    if  (INTCONbits.TMR0IF)
//    {
//        INTCONbits.TMR0IF = 0;          // clear (reset) flag
//
//    }
//	if (PIR1bits.RCIF)
//	{
//		//unsigned char serialChar;
//		serInput = RCREG;
//
//		if (serInput == 0x0D)
//		{
//			printFlag = 1;
//		}
//		else
//		{
//			if (serInputBufferLoc > 62)
//				serInputBufferLoc = 62;
//			serInputBuffer[serInputBufferLoc] = serInput;
//			serInputBufferLoc++;
//		}
//
//	}
//    // Check for another interrupt, examples:
//    // if (PIR1bits.TMR1IF)     // Timer 1
//    // if (PIR1bits.ADIF)       // ADC
//}
