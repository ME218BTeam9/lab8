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

#define ALL_BITS (0xff<<2)
#define PWMTicksPerMS 40
#define PeriodInMS 1000 //10000Hz
#define TicksPerMS 40000
#define BitsPerNibble 4

static uint8_t MyPriority;

static uint32_t Period;
static uint32_t LastCapture;
static uint32_t PWM_INTERVAL;
static uint32_t Timeout = 0;
static int32_t TargetRPM;
static uint32_t RPM;
static float RPMError = 0;
static float SumError = 0;
static uint32_t RequestedDuty;

void InitControlPeriod( void );
void ControlResponse (void);

bool InitPIDService ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
  InitControlPeriod();
	ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
    return true;
  }else
  {
		return false;
  }
}

bool PostPIDService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}


ES_Event RunPIDService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors	
  return ReturnEvent;
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
	PWM_INTERVAL = GetInterval();
	//printf("PWM_INTERVAL= %u \r\n", PWM_INTERVAL);
	float kp = .3;
	float ki = .1;
	uint32_t RPM = GetRPM();
	//TargetRPM =  ((float)PWM_INTERVAL * 0.7683 - 11.544);
	TargetRPM =  ((float)PWM_INTERVAL * 0.205 + 1.033);
	//RPM = 60 * 1000* TicksPerMS/ (Period * 4) / 298;
	RPMError = TargetRPM - RPM;
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
	//printf("RPM is %d\n\r", RPM);
//  printf("TargetRPM = %u \n\r ", TargetRPM);
//  printf("RPMError = %f\n\r ", RPMError);
//  printf("SumError = %f\n\r ", SumError);
	PWMUpdate((uint32_t)RequestedDuty);
  //printf("RequestedDuty = %u \r\n", RequestedDuty);
}