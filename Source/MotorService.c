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
#include "ADMulti.h"
#include "inc/hw_pwm.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "SPIservice.h"

/*----------------------------- Module Defines ----------------------------*/
#define ALL_BITS (0xff<<2)
#define PWMTicksPerMS 40000
#define PeriodInMS 1
#define BitsPerNibble 4


static uint32_t Period;
static uint32_t LastCapture;
static uint32_t PWM_INTERVAL;
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
void InitPWM(void);
static void PWMUpdateMotorRight(int32_t);
static void PWMUpdateMotorLeft(int32_t);
static void InitPortB();






/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static MotorServiceState_t CurrentMotorState;
static uint32_t data[180];
static uint32_t IRResult[1];
static uint32_t TimePerDegree = 150;
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
	_HW_Timer_Init(ES_Timer_RATE_1mS);
	ES_Timer_InitTimer(MOTOR_TIMER, 100);
	//ES_Timer_InitTimer(AD_TIMER, 100);
	ThisEvent.EventType = ES_INIT;
	CurrentMotorState = InitMotorStep;
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
	ES_Event New_Event;
  ES_Event ReturnEvent;
	ES_Event PostEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentMotorState)
  {
    case InitMotorStep:       // If current state is initial Psedudo State
        if ( ThisEvent.EventType == ES_INIT )// only respond to ES_Init
        {
            CurrentMotorState = MotorCommand;		
            //printf("CurrentStep %d\r\n", CurrentStep);		
						printf("\r\n Motor Inited \r\n");	
//            New_Event.EventParam = Stop;
//            New_Event.EventType = NEW_COMMAND;
//            PostMotorService(New_Event);	
           ES_Timer_InitTimer(MOTOR_TIMER, 100);			
//					PWMUpdateMotorRight(-100);//  PWM counterclockwise	
//          PWMUpdateMotorLeft(-100);//  PWM counterclockwise					
         }
    break;

    case MotorCommand:       
				if (ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == DEGREE_TIMER)
				{
					printf("\r\n Motor task finished \r\n");	
					PWMUpdateMotorRight(0);// 0% PWM
					PWMUpdateMotorLeft(0); //0% PWM
				}
        if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == Stop) 
				{ 
					printf("\r\n Motor stop \r\n");	
					PWMUpdateMotorRight(0);// 0% PWM
					PWMUpdateMotorLeft(0); //0% PWM					
	        ES_Timer_InitTimer(DEGREE_TIMER, 100);	
				}		
				// 90  degree clockwise
				 if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == CW90) 
				 {
					printf("\r\n Motor 90 clockwise \r\n");	
					PWMUpdateMotorLeft(50); //	 PWM clockwise	
					PWMUpdateMotorRight(-50);//  PWM counterclockwise
					ES_Timer_InitTimer(DEGREE_TIMER, 90 * TimePerDegree);
				 } 

				 // 45 degree clockwise
				 if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == CW45) 
				 {
					  printf("\r\n Motor 45 clockwise \r\n");	
					PWMUpdateMotorRight(-80);// 50% PWM counterclockerwise
					PWMUpdateMotorLeft(80); // 50% PWM clockwise
					ES_Timer_InitTimer(DEGREE_TIMER, 45 * TimePerDegree);						 
				 }
					// 90 degree counterwise
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == CCW90) 
				 {
					 printf("\r\n Motor 90 counterclockwise \r\n");	
					PWMUpdateMotorRight(50);// clockwise
					PWMUpdateMotorLeft(-50); // counterwise	
          ES_Timer_InitTimer(DEGREE_TIMER, 90 * TimePerDegree);					 
				 }
			 
				 	// 45 degree counterwise
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == CCW45) 
				 {
					 printf("\r\n Motor 45 counterclockwise \r\n");
					PWMUpdateMotorRight(80);// clockwise
					PWMUpdateMotorLeft(-80); // counterwise    			
					ES_Timer_InitTimer(DEGREE_TIMER, 45 * TimePerDegree);						 

				 }
				 	// drive farward half speed
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == ForwardHalf) 
				 {
					 printf("\r\n Motor forward half speed \r\n");	
					PWMUpdateMotorRight(50);// clockwise half PWM
					PWMUpdateMotorLeft(50); // clockwise half PWM			 
				 }

				 	// drive farward full speed
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == ForwardFull) 
				 {
					 printf("\r\n Motor forward full speed \r\n");	
					PWMUpdateMotorRight(100);// clockwise full PWM
					PWMUpdateMotorLeft(100); // clockwise full PWM						 
				 }				 
         // drive reverse half speed
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == ReverseHalf) 
				 {
					  printf("\r\n Motor reverse half speed \r\n");	
					PWMUpdateMotorRight(-50);// clockwise half PWM
					PWMUpdateMotorLeft(-50); // clockwise half PWM					 
				 }			 
         // drive reverse full speed
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == ReverseFull) 
				 {
					 printf("\r\n Motor reverse full speed \r\n");	
					PWMUpdateMotorRight(-100);// clockwise full PWM
					PWMUpdateMotorLeft(-100); // clockwise full PWM					 
				 }			 
          // align with beacon				 
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == Align) 
				 {
					for (int i = 0; i < 360/5; i++){
						// rotate 5 degree each time
						PWMUpdateMotorLeft(50); //	 PWM clockwise	
						PWMUpdateMotorRight(-50);
						ES_Timer_InitTimer(DEGREE_TIMER, 5 * TimePerDegree);	
						PWMUpdateMotorLeft(0);						
						// get data from IR sensor and put into DATA array
						ADC_MultiInit(1);
						ADC_MultiRead(IRResult);
						data[i] = IRResult[0];	
					}
				 }					 
         // drive forward until tape detected
				  if (ThisEvent.EventType == NEW_COMMAND && ThisEvent.EventParam == Drive2Tape) 
				 {
         // rotate and find signal
					//PWMUpdateMotorRight();// clockwise
					//PWMUpdateMotorLeft(); // clockwise
          // use interupt to stop					 
					 ES_Timer_InitTimer(MOTOR_TIMER, TimePerDegree);	
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
// Set PE3 & PF4 as digital outputs and set them low
	
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R4;     //Port E
while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_RCGCGPIO_R4) != SYSCTL_RCGCGPIO_R4);
HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (BIT3HI);
HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) |= ( BIT3HI);
HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~(BIT3HI);

HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5;     //Port F
while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_RCGCGPIO_R5) != SYSCTL_RCGCGPIO_R5);
HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (BIT4HI);
HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) |= ( BIT4HI);
HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~(BIT4HI);	

	
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
// HWREG( PWM0_BASE+PWM_O_0_CMPB) = ((PeriodInMS * PWMTicksPerMS))>>3;
//HWREG( PWM0_BASE+PWM_O_2_CMPB) = ((PeriodInMS * PWMTicksPerMS)-1)>>2;

// enable the PWM outputs
HWREG( PWM0_BASE+PWM_O_ENABLE) |= (PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN);
// Set the Output Enables to happen locally synchronized to counter=0
HWREG( PWM0_BASE+PWM_O_ENUPD) = (HWREG( PWM0_BASE+PWM_O_ENUPD) &
~(PWM_ENUPD_ENUPD0_M | PWM_ENUPD_ENUPD1_M)) |
(PWM_ENUPD_ENUPD0_LSYNC | PWM_ENUPD_ENUPD1_LSYNC);
// now configure the Port B pins to be PWM outputs
// start by selecting the alternate function for Pe4 & 5
HWREG(GPIO_PORTE_BASE+GPIO_O_AFSEL) |= (BIT4HI|BIT5HI);
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
static void PWMUpdateMotorRight(int32_t DutyRight)
{
	if (DutyRight < 0) {
		// Set PF4 to high, use the lower part of PWM
	  HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT4HI;
		
		if(DutyRight <= -100){
			HWREG( PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ONE);
			HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) / 2;
		}		
		else {
			HWREG(PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD))*(-DutyRight)/100;
		}
		
	}
	else {
		// Set PF4 to low, use the higher part of PWM
	  HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~BIT4HI;
		if (DutyRight == 0){
			HWREG( PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ZERO | PWM_2_GENA_ACTCMPAD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) >> 1;
		} 
		else if(DutyRight >= 100){
			HWREG( PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ONE );
			HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) >> 1;
		} else {
		  HWREG(PWM0_BASE+PWM_O_2_GENA) = (PWM_2_GENA_ACTCMPAU_ONE | PWM_2_GENA_ACTCMPAD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPA) = (HWREG(PWM0_BASE+PWM_O_2_LOAD))*(100-DutyRight)/100;
		}

	}
}

static void PWMUpdateMotorLeft(int32_t DutyRight)
{
	if (DutyRight < 0) {
		// Set PE3 to high, use the lower part of PWM
	  HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) |= BIT3HI;
		
		if(DutyRight <= -100){
			HWREG( PWM0_BASE+PWM_O_2_GENB) = (PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ONE);
			HWREG(PWM0_BASE+PWM_O_2_CMPB) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) >> 1;
		}		
		else {
			HWREG(PWM0_BASE+PWM_O_2_GENB) = (PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPB) = (HWREG(PWM0_BASE+PWM_O_2_LOAD))*(-DutyRight)/100;
		}
		
	}
	else {
		// Set PE3 to low, use the higher part of PWM
	  HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA + ALL_BITS)) &= ~BIT3HI;
		if (DutyRight == 0){
			HWREG( PWM0_BASE+PWM_O_2_GENB) = (PWM_2_GENB_ACTCMPBU_ZERO | PWM_2_GENB_ACTCMPBD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPB) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) >> 1;
		} 
		else if(DutyRight >= 100){
			HWREG( PWM0_BASE+PWM_O_2_GENB) = (PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ONE );
			HWREG(PWM0_BASE+PWM_O_2_CMPB) = (HWREG(PWM0_BASE+PWM_O_2_LOAD)) >> 1;
		} else {
		  HWREG(PWM0_BASE+PWM_O_2_GENB) = (PWM_2_GENB_ACTCMPBU_ONE | PWM_2_GENB_ACTCMPBD_ZERO);
			HWREG(PWM0_BASE+PWM_O_2_CMPB) = (HWREG(PWM0_BASE+PWM_O_2_LOAD))*(100-DutyRight)/100;
		}

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
