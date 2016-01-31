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
#include "ADService.h"
#include "ADMulti.h"

/*----------------------------- Module Defines ----------------------------*/

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
#define ALL_BITS (0xff<<2)
#define POTADJUSTINGCONST (50000)
//#define POTADJUSTINGCONST (30000) // Half Step
//#define POTADJUSTINGCONST (35000) // Half Step


/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file



// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;
static uint32_t Interval;
static uint32_t PotValue[1];

static void ReadPotValue();
uint32_t GetInterval();

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
bool InitADService( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
	ADC_MultiInit(1); // PE0
	
	_HW_Timer_Init(ES_Timer_RATE_1mS);
	ES_Timer_InitTimer(AD_TIMER, 20);
	ThisEvent.EventType = ES_INIT;

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
     PostADService

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
bool PostADService( ES_Event ThisEvent )
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
ES_Event RunADService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
	if (ThisEvent.EventType == ES_TIMEOUT)
	{
		ES_Timer_InitTimer(AD_TIMER, 100);
		ReadPotValue();	
  }
  return ReturnEvent;
}

/****************************************************************************
 Function
     PWMControl

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
    Han Ye
****************************************************************************/
static void ReadPotValue(void){
    ADC_MultiRead(PotValue);
	  uint32_t Read = PotValue[0];
	  //printf("Pot = %u \r\n", Read);
	  Interval =(100)*((float)(Read-1))/(4094);
//	  Interval = (float)POTADJUSTINGCONST / (float)Read;
	  //printf("Interval = %u \r\n", Interval);
}

uint32_t GetInterval()
{
	if (Interval > 100){
		Interval = 100; // just in case when the Read is 0
	}
	//Interval = Interval/2;
	//printf("Interval = %u \r\n", Interval);
	return Interval;
}


/***************************************************************************
 private functions
 ***************************************************************************/

