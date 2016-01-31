#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include "PWM8Tiva.h"

//  UPDATED on 11/16/14 at 16:10 by jec to fix bug in PWM8_TIVA_SetPulseWidth

static uint32_t ulPeriod[4];
//static uint8_t  LocalDuty[8] = {0,0,0,0,0,0,0,0};
static uint8_t  LocalDuty[8] = {0,0,0,0,0,0,0,0};
static const uint32_t Channel2PWMconst[8]={PWM_OUT_0,PWM_OUT_1,PWM_OUT_2,PWM_OUT_3,
                                         PWM_OUT_4,PWM_OUT_5,PWM_OUT_6,PWM_OUT_7};
static const uint32_t Group2GENconst[4]={PWM_GEN_0,PWM_GEN_1,PWM_GEN_2,PWM_GEN_3};
//static const uint32_t Channel2PWMconst[4]={PWM_OUT_0,PWM_OUT_1,PWM_OUT_2,PWM_OUT_3
//                                         };
//static const uint32_t Group2GENconst[2]={PWM_GEN_0,PWM_GEN_1};

void PWM8_TIVA_Init(void)
{    
    uint8_t i;
  
   //Configure PWM Clock to system / 32
   SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins

    //Configure PB6,PB7,PB4, PB5 Pins as PWM
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);


  // Configure PE4, PE5 for PWM0 M0PWM4, M0PWM5
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);

    // Configure PD0, PD1 for PWM0 M0PWM6, M0PWM7
    GPIOPinConfigure(GPIO_PD0_M0PWM6);
    GPIOPinConfigure(GPIO_PD1_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);

  // Calculate the period for 500 Hz including divide by 32 in PWM clock
    ulPeriod[0] = SysCtlClockGet()/32 / 500; //PWM frequency 500HZ
    // set all of the periods the same initially
    for (i=1; i<(sizeof(Group2GENconst)/sizeof(Group2GENconst[0])); i++){
      ulPeriod[i] = ulPeriod[0];
    }

    //Configure PWM Options
    //PWM_GEN_0 Covers M0PWM0 and M0PWM1
    //PWM_GEN_1 Covers M0PWM2 and M0PWM3 See page 207 4/11/13 DriverLib doc
    //PWM_GEN_2 Covers M0PWM4 and M0PWM5
    //PWM_GEN_3 Covers M0PWM6 and M0PWM7
    for (i=0; i<(sizeof(Group2GENconst)/sizeof(Group2GENconst[0])); i++){
      PWMGenConfigure(PWM0_BASE, Group2GENconst[i], PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); 
    //Set the Period (expressed in clock ticks)
      PWMGenPeriodSet(PWM0_BASE, Group2GENconst[i], ulPeriod[i]);
    }

    //Set PWM duty to initial value
    for ( i = 0; i<(sizeof(LocalDuty)/sizeof(LocalDuty[0])); i++){
      PWM8_TIVA_SetDuty( LocalDuty[i], i);
    }

    // Enable the PWM generators
    for (i=0; i<(sizeof(Group2GENconst)/sizeof(Group2GENconst[0])); i++){
          PWMGenEnable(PWM0_BASE, Group2GENconst[i]); 
    }
    PWMSyncUpdate(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT |
                  PWM_GEN_3_BIT);
    // Turn on the Output pins
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | 
                  PWM_OUT_3_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT |
                  PWM_OUT_7_BIT, true);

}

void PWM8_TIVA_SetDuty( uint8_t dutyCycle, uint8_t channel)
{
  uint32_t updateVal;
  
  if (channel > 7) // sanity check, reasonable channel number
    return;
  if ((0 == dutyCycle) || ( dutyCycle >= 100)){ // don't try to calculate with 0 DC
    updateVal = 0;
  }else{ // reasonable duty cycle number, so calculate new pulse width
    updateVal = (ulPeriod[channel>>1]*dutyCycle)/100;    
  }
  // update local copy of DC
  LocalDuty[channel] = dutyCycle;
  // set the new pulse width based on requested DC
  PWMPulseWidthSet(PWM0_BASE, Channel2PWMconst[channel],updateVal); 
 }

void PWM8_TIVA_SetPulseWidth( uint16_t NewPW, uint8_t channel)
{
  if (channel > 7) // sanity check, reasonable channel number
    return;
  // make sure that the requested PW is less than the period before updating 
  if ( NewPW < ulPeriod[channel>>1]){  
    PWMPulseWidthSet(PWM0_BASE, Channel2PWMconst[channel],NewPW); 
  }    
}

/*****************************************************************************
  PWM8_TIVA_SetPeriod( uint16_t reqPeriod, uint8_t group)
    sets the requested PWM group's period to the Requested Period
    group 0 = channels 0 & 1
    group 1 = channels 2 & 3
    group 2 = channels 4 & 5
    group 3 = channels 6 & 7
*****************************************************************************/

void PWM8_TIVA_SetPeriod( uint16_t reqPeriod, uint8_t group)
{
    //Set the Period (expressed in clock ticks)
   ulPeriod[group] = reqPeriod;
   PWMGenPeriodSet(PWM0_BASE, Group2GENconst[group], reqPeriod); 
   // Set new Duty after period change
   PWM8_TIVA_SetDuty( LocalDuty[group<<1], group<<1);
   PWM8_TIVA_SetDuty( LocalDuty[(group<<1)+1], (group<<1)+1);
 }

 /*****************************************************************************
  PWM8_TIVA_SetFreq( uint16_t reqPeriod, uint8_t group)
    sets the requested PWM group's frequency to the Requested Frequency, in Hz
    group 0 = channels 0 & 1
    group 1 = channels 2 & 3
    group 2 = channels 4 & 5
    group 3 = channels 6 & 7
*****************************************************************************/

void PWM8_TIVA_SetFreq( uint16_t reqFreq, uint8_t group)
{
    //Use the Frequency (expressed in Hz) to calculate a new period
  ulPeriod[group] = SysCtlClockGet()/32 /reqFreq;
  // apply the new period
  PWM8_TIVA_SetPeriod( ulPeriod[group], group);

}

