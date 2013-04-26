/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define FCY             10000000

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */

void ConfigureOscillator(void); /* Handles clock switching/osc initialization */

void calcdelay(void);     /* Temporary function to create sine motion of motor*/
void readSensorData(void);/*Temporary function to read I2C sensor data        */
void GyroZaverage(void);  /* Temporary function to test averaging             */
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);
