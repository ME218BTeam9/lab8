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
#include "ButtonService.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/gpio.h"
#include "BITDEFS.h"
#include "MotorService.h"
#include "ES_Port.h"
#include "termio.h"



/*----------------------------- Module Defines ----------------------------*/

#define ALL_BITS (0xff<<2)
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static bool ReadButton(void);
/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static ButtonState_t CurrentState;
static bool LastButtonState;
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
bool InitButtonDebounce ( uint8_t Priority )
{
	ES_Event ThisEvent;
  MyPriority = Priority;
  CurrentState = Debouncing;

	// Activate Port F
  HWREG(SYSCTL_RCGCGPIO) |= BIT5HI;
  while ((HWREG(SYSCTL_RCGCGPIO) & BIT5HI) != BIT5HI)
  ;
	HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R5; 
	HWREG(GPIO_PORTF_BASE+GPIO_O_DEN) |= GPIO_PIN_4; 
  HWREG(GPIO_PORTF_BASE+GPIO_O_DIR) &= ~GPIO_PIN_4; 
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK ) = 0x4C4F434B;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR ) |= BIT0HI; 
	// enable internal pull up
	HWREG(GPIO_PORTF_BASE + GPIO_O_PUR ) |= GPIO_PIN_4; 
  LastButtonState = ReadButton();
	_HW_Timer_Init(ES_Timer_RATE_1mS);
  ES_Timer_InitTimer(BUTTON_TIMER,450);
	
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
bool PostButtonDebounce( ES_Event ThisEvent )
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
ES_Event RunButtonDebounce( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  switch ( CurrentState )
  {
    case Debouncing :       // If current state is state one
      switch ( ThisEvent.EventType )
      {
        case ES_TIMEOUT: //If event is event one
            if(ThisEvent.EventParam == BUTTON_TIMER)
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
        case BUTTON_UP:
            ES_Timer_InitTimer(BUTTON_TIMER,450);
            CurrentState = Debouncing;
            break;
        case BUTTON_DOWN:
            ES_Timer_InitTimer(BUTTON_TIMER,450);
            CurrentState = Debouncing;
            ThisEvent.EventType = BUTTON_DOWN;
//						New_Event.EventParam = ReadGuitarButtons();
				    PostMotorService (ThisEvent);
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

///****************************************************************************
// Function
//     QueryTemplateSM

// Parameters
//     None

// Returns
//     TemplateState_t The current state of the Template state machine

// Description
//     returns the current state of the Template state machine
// Notes

// Author
//     J. Edward Carryer, 10/23/11, 19:21
//****************************************************************************/
//SDState_t QueryStrumDebounceSM ( void )
//{
//   return(CurrentState);
//}
bool CheckButtonEvents(void)
{
	//printf("checking button event \n\r");
  bool ReturnVal = false;
  bool CurrentButtonState;
  CurrentButtonState = ReadButton();
	//printf("ReturnVal %d\r\n", ReturnVal);
  if 	(CurrentButtonState != LastButtonState)
  {
    ReturnVal = true;
    if(CurrentButtonState)
    {
      ES_Event ThisEvent;
      ThisEvent.EventType = BUTTON_DOWN;
      PostButtonDebounce(ThisEvent);
			printf("Button down \r\n");
    }
    if (!CurrentButtonState)
    {
      ES_Event ThisEvent;
      ThisEvent.EventType = BUTTON_UP;
      PostButtonDebounce(ThisEvent);
			printf("Button up \r\n");
    }
  }
  LastButtonState = CurrentButtonState;
  return ReturnVal;
}

/***************************************************************************
 private functions
 ***************************************************************************/

static bool ReadButton(void)
{
	uint8_t ButtonVal = HWREG(GPIO_PORTF_BASE+(GPIO_O_DATA+ALL_BITS));
	if ((ButtonVal & BIT4HI) != BIT4HI) {
		return false;
	} else {
		return true;
	}
}
