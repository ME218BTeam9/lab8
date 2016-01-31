/****************************************************************************
 Module
   TemplateFSM.c

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
#include "StrumDebounceSM.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "BITDEFS.h"
#include "EnablePA25_PB23_PD7_PF0.h"

/*----------------------------- Module Defines ----------------------------*/
#define STRUM_PIN BIT4HI
#define STRUM_PIN_HI BIT4HI
#define STRUM_PIN_LO BIT4LO

#define GUITAR_BUT_1 BIT2HI
#define GUITAR_BUT_1_HI BIT2HI
#define GUITAR_BUT_1_LO BIT2LO

#define GUITAR_BUT_2 BIT3HI
#define GUITAR_BUT_2_HI BIT3HI
#define GUITAR_BUT_2_LO BIT3LO

#define GUITAR_BUT_3 BIT6HI
#define GUITAR_BUT_3_HI BIT6HI
#define GUITAR_BUT_3_LO BIT6LO

#define GUITAR_BUT_4 BIT7HI
#define GUITAR_BUT_4_HI BIT7HI
#define GUITAR_BUT_4_LO BIT7LO

#define GUITAR_BUT_5 BIT0HI
#define GUITAR_BUT_5_HI BIT0HI
#define GUITAR_BUT_5_LO BIT0LO

#define ALL_BITS (0xff<<2)
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static uint8_t ReadGuitarButtons(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static SDState_t CurrentState;
static uint8_t LastStrumState;
// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateFSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitStrumDebounceSM ( uint8_t Priority )
{
  MyPriority = Priority;
  // put us into the Initial PseudoState
	PortFunctionInit();
  CurrentState = Debouncing;

	//STRUM
	// Activate Port E
  HWREG(SYSCTL_RCGCGPIO) |= BIT4HI;
  while ((HWREG(SYSCTL_RCGCGPIO) & BIT4HI) != BIT4HI)
  ;
  HWREG(GPIO_PORTE_BASE+GPIO_O_DEN) |= (STRUM_PIN);
  HWREG(GPIO_PORTE_BASE+GPIO_O_DIR) &= (STRUM_PIN_LO);

	//GUITAR BUTTONS
	// Activate Port D
  HWREG(SYSCTL_RCGCGPIO) |= BIT3HI;
  while ((HWREG(SYSCTL_RCGCGPIO) & BIT3HI) != BIT3HI)
  ;
  HWREG(GPIO_PORTD_BASE+GPIO_O_DEN) |= (GUITAR_BUT_1|GUITAR_BUT_2|GUITAR_BUT_3|GUITAR_BUT_4);
  HWREG(GPIO_PORTD_BASE+GPIO_O_DIR) &= (GUITAR_BUT_1_LO|GUITAR_BUT_2_LO|GUITAR_BUT_3_LO|GUITAR_BUT_4_LO);

	LastStrumState = (HWREG(GPIO_PORTA_BASE+(GPIO_O_DATA+ALL_BITS)) & STRUM_PIN);

	// Activate Port F
  HWREG(SYSCTL_RCGCGPIO) |= BIT5HI;
  while ((HWREG(SYSCTL_RCGCGPIO) & BIT5HI) != BIT5HI)
  ;
  HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= (GUITAR_BUT_5);
  HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) &= (GUITAR_BUT_5_LO);

	LastStrumState = (HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA+ALL_BITS)) & STRUM_PIN);

	// post the initial transition event
  ES_Timer_InitTimer(STRUM_TIMER,5);
  return true;
}

bool CheckStrumEvents(void)
{
	//printf("checking button event \n\r");
  bool ReturnVal = false;
  uint8_t CurrentButtonState;
  CurrentButtonState = (HWREG(GPIO_PORTE_BASE+(GPIO_O_DATA+ALL_BITS)) & STRUM_PIN);
  if 	(CurrentButtonState != LastStrumState)
  {
    ReturnVal = true;
    if(CurrentButtonState != 0)
    {
      ES_Event ThisEvent;
      ThisEvent.EventType = STRUM_DOWN;
      PostStrumDebounceSM(ThisEvent);
    }
    else
    {
    ES_Event ThisEvent;
    ThisEvent.EventType = STRUM_UP;
    PostStrumDebounceSM(ThisEvent);
    }
  }
  LastStrumState = CurrentButtonState;
  return ReturnVal;
}


/****************************************************************************
 Function
     PostTemplateFSM

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostStrumDebounceSM( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateFSM

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunStrumDebounceSM( ES_Event ThisEvent )
{
  ES_Event New_Event;
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  switch ( CurrentState )
  {
    case Debouncing :       // If current state is state one
      switch ( ThisEvent.EventType )
      {
        case ES_TIMEOUT: //If event is event one
            if(ThisEvent.EventParam == STRUM_TIMER)
            {
              CurrentState = Ready2Sample;
            }
            break;

        // repeat cases as required for relevant events
        default :
            ;
      }  // end switch on CurrentEvent
      break;
    case Ready2Sample :
      switch(ThisEvent.EventType)
      {
        case STRUM_UP:
            ES_Timer_StartTimer(STRUM_TIMER);
            CurrentState = Debouncing;
            break;
        case STRUM_DOWN:
            ES_Timer_StartTimer(STRUM_TIMER);
            CurrentState = Debouncing;
            New_Event.EventType = STRUM_OCCURED;
						New_Event.EventParam = ReadGuitarButtons();
						ES_PostList00(New_Event);
            break;
        default:
            ;
      }
    // repeat state pattern as required for other states
    default :
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
SDState_t QueryStrumDebounceSM ( void )
{
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/

static uint8_t ReadGuitarButtons(void)
{
	uint8_t ButtonVals = 0;
	ButtonVals = ButtonVals & ((HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA+ALL_BITS)) & (GUITAR_BUT_1 | GUITAR_BUT_2)) >> 2 );
	ButtonVals = ButtonVals & ((HWREG(GPIO_PORTD_BASE+(GPIO_O_DATA+ALL_BITS)) & (GUITAR_BUT_3 | GUITAR_BUT_4)) >> 4 );
	ButtonVals = ButtonVals & ((HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA+ALL_BITS)) & (GUITAR_BUT_5)) << 4 );
	return ButtonVals;
}
