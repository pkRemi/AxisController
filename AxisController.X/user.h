/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void InitApp(void); /* I/O and Peripheral Initialization */
void initSerial(void); /* initialization of serial port UART*/
void InitMPU6050(unsigned char I2Caddr); /* initialization of MPU6050*/

