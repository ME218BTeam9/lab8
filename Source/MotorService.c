/****************************************************************************
 Module
   MotorService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include <math.h>
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "termio.h"
#include "ES_Port.h"
#include "MotorService.h"
#include "PWM8Tiva.h"
#include "ADService.h"
#include "inc/hw_pwm.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"

/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define PWMTicksPerMS 40
#define PeriodInMS 1000 //10000Hz
#define TicksPerMS 40000
#define BitsPerNibble 4
#define ONELED BIT0HI
#define TWOLED BIT0HI|BIT1HI
#define THREELED BIT0HI|BIT1HI|BIT2HI
#define FOURLED BIT0HI|BIT1HI|BIT2HI|BIT3HI
#define FIVELED BIT0HI|BIT1HI|BIT2HI|BIT3HI|BIT4HI
#define SIXLED BIT0HI|BIT1HI|BIT2HI|BIT3HI|BIT4HI|BIT5HI
#define SEVENLED BIT0HI|BIT1HI|BIT2HI|BIT3HI|BIT4HI|BIT5HI|BIT6HI
#define EIGHTLED BIT0HI|BIT1HI|BIT2HI|BIT3HI|BIT4HI|BIT5HI|BIT6HI|BIT7HI

static uint32_t Period;
static uint32_t LastCapture;
static uint32_t PWM_INTERVAL;
static uint32_t Timeout = 0;
static int32_t TargetRPM;
static uint32_t RPM;
static float RPMError = 0;
static float SumError = 0;
static uint32_t RequestedDuty;
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitPWM(void);
void PWMUpdate(uint32_t PWM_INTERVAL);
void InitInputCapturePeriod( void );
static void InitPortB();
void InputCaptureResponse(void);
static void LEDUpdate(void);
void InitControlPeriod( void );
void ControlResponse (void);
uint32_t GetRPM(void);




/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static MotorServiceState_t CurrentMotorState;


// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;


/*------------------------------ Module Code ------------------------------*/

/****************************************************************************
 Function
     InitMotorService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
bool InitMotorService( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
	InitPWM();
  InitPortB();
	InitInputCapturePeriod();
	InitControlPeriod();
	_HW_Timer_Init(ES_Timer_RATE_1mS);
	ES_Timer_InitTimer(MOTOR_TIMER, 100);
	ES_Timer_InitTimer(AD_TIMER, 100);
	ThisEvent.EventType = ES_INIT;
	CurrentMotorState = InitMotorStep;
	Timeout=0;
	printf("Initialization done\n\r");
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostMotorService

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     Han Ye
****************************************************************************/
bool PostMotorService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunMotorService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   Han Ye
****************************************************************************/
ES_Event RunMotorService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentMotorState)
  {
    case InitMotorStep:       // If current state is initial Psedudo State
        if ( ThisEvent.EventType == ES_INIT )// only respond to ES_Init
        {
            CurrentMotorState = MotorDCState;		
            //printf("CurrentStep %d\r\n", CurrentStep);		
						printf("Motor DC\r\n");	
            ES_Timer_InitTimer(MOTOR_TIMER, 50);					
         }
    break;

    case MotorDCState:       // If current state is state one

        if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == MOTOR_TIMER) 
				{ // Should init another FullStepTimer
					PWM_INTERVAL = GetInterval();
					PWMUpdate(PWM_INTERVAL);
					ES_Timer_InitTimer(MOTOR_TIMER, 50);
					//printf("Period: %d\r\n", Period);
					HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT5HI;
					RPM = 60 * 1000* TicksPerMS/ (Period * 4) / 298;
	        printf("RPM is %d\n\r", RPM);
					printf("TargetRPM = %u \n\r ", TargetRPM);
          printf("RPMError = %f\n\r ", RPMError);
          printf("SumError = %f\n\r ", SumError);
					//printf("Timeout is %d\n\r", Timeout);
          HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT5LO;	
					
				}
				
		break;

  }                                   // end switch on Current State
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
     InitPWM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
void InitPWM()
{
volatile uint32_t Dummy;
	// start by enabling the clock to the PWM Module (PWM0)
HWREG(SYSCTL_RCGCPWM) |= SYSCTL_RCGCPWM_R0;
// enable the clock to Port E
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;
// Select the PWM clock as System Clock/32
HWREG(SYSCTL_RCC) = (HWREG(SYSCTL_RCC) & ~SYSCTL_RCC_PWMDIV_M) |
(SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_32);
// make sure that the PWM module clock has gotten going
while ((HWREG(SYSCTL_PRPWM) & SYSCTL_PRPWM_R0) != SYSCTL_PRPWM_R0)
;
// disable the PWM while initializing
HWREG( PWM0_BASE+PWM_O_2_CTL ) = 0;
// program generator A to go to 0 at rising compare A, 1 on falling compare A
HWREG( PWM0_BASE+PWM_O_2_GENA) =
(PWM_2_GENA_ACTCMPAU_ZERO | PWM_2_GENA_ACTCMPAD_ONE );
// program generator B to go to 0 at rising compare B, 1 on falling compare B
HWREG( PWM0_BASE+PWM_O_2_GENB) =
(PWM_2_GENA_ACTCMPBU_ZERO | PWM_2_GENA_ACTCMPBD_ONE );
// Set the PWM period. Since we are counting both up & down, we initialize
// the load register to 1/2 the desired total period
HWREG( PWM0_BASE+PWM_O_2_LOAD) = ((PeriodInMS * PWMTicksPerMS)-1)>>1;

// Set the initial Duty cycle on A to 50% by programming the compare value
// to 1/2 the period to count up (or down)
HWREG( PWM0_BASE+PWM_O_2_CMPA) = ((PeriodInMS * PWMTicksPerMS)-1)>>2;
// Set the initial Duty cycle on B to 25% by programming the compare value
// to 1/4 the period
//HWREG( PWM0_BASE+PWM_O_0_CMPB) = ((PeriodInMS * PWMTicksPerMS))>>3;
// enable the PWM outputs
HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN);
// Set the Output Enables to happen locally synchronized to counter=0
HWREG( PWM0_BASE+PWM_O_ENUPD) = (HWREG( PWM0_BASE+PWM_O_ENUPD) &
~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
(PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);
// now configure the Port B pins to be PWM outputs
// start by selecting the alternate function for Pe4 & 5
HWREG(GPIO_PORTE_BASE+GPIO_O_AFSEL) |= (BIT4HI | BIT5HI);
// now choose to map PWM to those pins, this is a mux value of 4 that we
// want to use for specifying the function on bits 4 & 5
HWREG(GPIO_PORTE_BASE+GPIO_O_PCTL) =
(HWREG(GPIO_PORTE_BASE+GPIO_O_PCTL) & 0xff00ffff) + (4<<(5*BitsPerNibble)) + (4<<(4*BitsPerNibble));
// Enable pins 4 & 5 on Port E for digital I/O
HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (BIT4HI | BIT5HI);
// make pins 4 & 5 on Port E into outputs
HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= (BIT4HI |BIT5HI);
// set the up/down count mode and enable the PWM generator, both generator updates locally synchronized to zero count
HWREG(PWM0_BASE+ PWM_O_2_CTL) |= (PWM_2_CTL_MODE | PWM_2_CTL_ENABLE | PWM_2_CTL_GENAUPD_LS | PWM_2_CTL_GENBUPD_LS);
}
/****************************************************************************
 Function
     PWMUpdate

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
void PWMUpdate(uint32_t PWM_INTERVAL)
{
	// PWM_INTERVAL = GetInterval();
	if (PWM_INTERVAL == 100) {
		HWREG( PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ONE );
	  //printf("PWM_INTERVAL = %u \r\n", PWM_INTERVAL);
	}
	else {
		HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD))*(100-PWM_INTERVAL)/100;
		HWREG( PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ZERO);	
		//printf("\r\nPWM_INTERVAL = %u \r", PWM_INTERVAL);
	}
}

/****************************************************************************
 Function
     InitPortB

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
static void InitPortB()
{
	 // Enable Peripheral Clocks 
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	 GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);// Enable pin PB2 for GPIOOutput
	 GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);// Enable pin PB3 for GPIOOutput
	 	// Activate Port B
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R1;
  while ((HWREG(SYSCTL_RCGCGPIO) & SYSCTL_PRGPIO_R1)!= SYSCTL_PRGPIO_R1);
	HWREG(GPIO_PORTB_BASE+GPIO_O_DEN) |= GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	HWREG(GPIO_PORTB_BASE+GPIO_O_DIR) |= GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	HWREG(GPIO_PORTB_BASE + (GPIO_O_DATA + ALL_BITS)) &= BIT7LO; // set all low first
	
}
/****************************************************************************
 Function
     InitInputCapturePeriod

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
void InitInputCapturePeriod( void ){
			// start by enabling the clock to the timer (Wide Timer 0)
			HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0;
			// enable the clock to Port C
			HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R2;
			// since we added this Port C clock init, we can immediately start
			// into configuring the timer, no need for further delay
			// make sure that timer (Timer A) is disabled before configuring
			HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
			// set it up in 32bit wide (individual, not concatenated) mode
			// the constant name derives from the 16/32 bit timer, but this is a 32/64
			// bit timer so we are setting the 32bit mode
			HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
			// we want to use the full 32 bit count, so initialize the Interval Load
			// register to 0xffff.ffff (its default value :-)
			HWREG(WTIMER0_BASE+TIMER_O_TAILR) = 0xffffffff;
			// set up timer A in capture mode (TAMR=3, TAAMS = 0),
			// for edge time (TACMR = 1) and up-counting (TACDIR = 1)
			HWREG(WTIMER0_BASE+TIMER_O_TAMR) =
			(HWREG(WTIMER0_BASE+TIMER_O_TAMR) & ~TIMER_TAMR_TAAMS) |
			(TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP);
			// To set the event to rising edge, we need to modify the TAEVENT bits
			// in GPTMCTL. Rising edge = 00, so we clear the TAEVENT bits
			HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TAEVENT_M;
			// Now Set up the port to do the capture (clock was enabled earlier)
			// start by setting the alternate function for Port C bit 4 (WT0CCP0)
			HWREG(GPIO_PORTC_BASE+GPIO_O_AFSEL) |= BIT4HI;
			// Then, map bit 4's alternate function to WT0CCP0
			// 7 is the mux value to select WT0CCP0, 16 to shift it over to the
			// right nibble for bit 4 (4 bits/nibble * 4 bits)
			HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) =
			(HWREG(GPIO_PORTC_BASE+GPIO_O_PCTL) & 0xfff0ffff) + (7<<16);
			// Enable pin on Port C for digital I/O
			HWREG(GPIO_PORTC_BASE+GPIO_O_DEN) |= BIT4HI;
			// make pin 4 on Port C into an input
			HWREG(GPIO_PORTC_BASE+GPIO_O_DIR) &= BIT4LO;
			// back to the timer to enable a local capture interrupt
			HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_CAEIM;
			// enable the Timer A in Wide Timer 0 interrupt in the NVIC
			// it is interrupt number 94 so appears in EN2 at bit 30
			HWREG(NVIC_EN2) |= BIT30HI;
			// make sure interrupts are enabled globally
			__enable_irq();
			// now kick the timer off by enabling it and enabling the timer to
			// stall while stopped by the debugger
			HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TAEN | TIMER_CTL_TASTALL);
			// printf("InitInputCapturePeriod Done\r\n");
}
/****************************************************************************
 Function
     InitInputCapturePeriod

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
void InputCaptureResponse( void ){
		uint32_t ThisCapture;
		// start by clearing the source of the interrupt, the input capture event

		HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_CAECINT;
		// now grab the captured value and calculate the period
		ThisCapture = HWREG(WTIMER0_BASE+TIMER_O_TAR);
		Period = ThisCapture - LastCapture;
		// update LastCapture to prepare for the next edge
		LastCapture = ThisCapture;
	  //printf("Period: %d\r\n", Period);
	  //LEDUpdate();
}

/****************************************************************************
 Function
     LEDUpdate

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     Han Ye
****************************************************************************/
static void LEDUpdate(void){
	HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT6HI;
		if (Period <= 20000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= ONELED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= ONELED;
	}
		else	if (Period > 20000 && Period < 21000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= TWOLED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= TWOLED;
	}
		else	if (Period > 21000 && Period <= 26000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= THREELED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= THREELED;
	}
		else	if (Period > 26000 && Period <= 30000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= FOURLED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= FOURLED;
	}
		else	if (Period> 30000 && Period <= 50000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= FIVELED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= FIVELED;
	}
		else	if (Period > 50000 && Period <= 75000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= SIXLED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= SIXLED;
	}
		else	if (Period > 75000 && Period <= 100000)
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= SEVENLED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= SEVENLED;
	}
		else
	{
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) |= EIGHTLED; 
		HWREG(GPIO_PORTB_BASE+(GPIO_O_DATA + ALL_BITS)) &= EIGHTLED;
	}
  HWREG(GPIO_PORTC_BASE+(GPIO_O_DATA + ALL_BITS)) &= BIT6LO;

}

uint32_t GetRPM(void){
	return RPM;
}

void InitControlPeriod( void ){
		//volatile uint32_t Dummy; // use volatile to avoid over-optimization
		// start by enabling the clock to the timer (Wide Timer 0)
		HWREG(SYSCTL_RCGCWTIMER) |= SYSCTL_RCGCWTIMER_R0; // kill a few cycles to let the clock get going
		//Dummy = HWREG(SYSCTL_RCGCGPIO);
		// make sure that timer (Timer A) is disabled before configuring
		HWREG(WTIMER0_BASE+TIMER_O_CTL) &= ~TIMER_CTL_TBEN;
		// set it up in 32bit wide (individual, not concatenated) mode
		HWREG(WTIMER0_BASE+TIMER_O_CFG) = TIMER_CFG_16_BIT;
		// set up timer A in periodic mode so that it repeats the time-outs
		HWREG(WTIMER0_BASE+TIMER_O_TBMR) =
		(HWREG(WTIMER0_BASE+TIMER_O_TBMR)& ~TIMER_TBMR_TBMR_M)| TIMER_TBMR_TBMR_PERIOD;
		// set timeout to 2mS
		HWREG(WTIMER0_BASE+TIMER_O_TBILR) = TicksPerMS * 2;
		// enable a local timeout interrupt
		HWREG(WTIMER0_BASE+TIMER_O_IMR) |= TIMER_IMR_TBTOIM;
		// enable the Timer A in Wide Timer 0 interrupt in the NVIC
		// it is interrupt number 95 so appears in EN2 at bit 31
		HWREG(NVIC_EN2) = BIT31HI;
		// make sure interrupts are enabled globally
		__enable_irq();
		// now kick the timer off by enabling it and enabling the timer to
		// stall while stopped by the debugger
		HWREG(WTIMER0_BASE+TIMER_O_CTL) |= (TIMER_CTL_TBEN | TIMER_CTL_TBSTALL);
		//printf("ControlPeriod Initialized");
}

void ControlResponse (void){
	// start by clearing the source of the interrupt
	HWREG(WTIMER0_BASE+TIMER_O_ICR) = TIMER_ICR_TBTOCINT;
	//Timeout++;
	//PWM_INTERVAL = GetInterval();
	
	float kp = .3;
	float ki = .1;
	
	//TargetRPM =  ((float)PWM_INTERVAL * 0.7683 - 11.544);
	TargetRPM =  ((float)PWM_INTERVAL * 0.205 + 1.033);
	RPM = 60 * 1000* TicksPerMS/ (Period * 4) / 298;
	RPMError = (float)TargetRPM - RPM;
	SumError += RPMError;
	RequestedDuty = kp*(RPMError + (ki*SumError));
	if (RequestedDuty >= 100)
	{ 
		RequestedDuty = 100;
		SumError -= RPMError;
	}
	else if (RequestedDuty <= 0)
	{ 
		RequestedDuty = 0;
		SumError -= RPMError;
	}
	PWMUpdate((uint32_t)RequestedDuty);
  //printf("RequestedDuty = %u \r\n", RequestedDuty);
}

