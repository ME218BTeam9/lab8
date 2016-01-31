/****************************************************************************
 Module
     PWM10Tiva.c
 Description
     Implementation file for the 10-channel version of the PWM Library for
     the Tiva
 Notes
     Channels 0-7 are implemented using the PWM Module 0 and channels
     8 & 9 are channels 0 & 1 on PWM Module 1
 History
 When           Who     What/Why
 -------------- ---     --------
 11/11/15   		jec		 converted from PWM8Tiva.c dated 11/16/14 added 2 more
                       channels and the ability to specify how many channels
                       of PWM you want
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include "PWM10Tiva.h"

#define MAX_NUM_CHANNELS 10
                                         
static uint32_t ulPeriod[MAX_NUM_CHANNELS>>1];
static uint8_t  LocalDuty[MAX_NUM_CHANNELS] = {0,0,0,0,0,0,0,0,0,0};
static const uint32_t Channel2PWMconst[MAX_NUM_CHANNELS]={
                                         PWM_OUT_0,PWM_OUT_1,PWM_OUT_2,PWM_OUT_3,
                                         PWM_OUT_4,PWM_OUT_5,PWM_OUT_6,PWM_OUT_7,
                                         PWM_OUT_0,PWM_OUT_1};
static const uint32_t Channel2PWM_MOD[MAX_NUM_CHANNELS]={
                                         PWM0_BASE,PWM0_BASE,PWM0_BASE,PWM0_BASE,
                                         PWM0_BASE,PWM0_BASE,PWM0_BASE,PWM0_BASE,
                                         PWM1_BASE,PWM1_BASE};
static const uint32_t Group2GENconst[MAX_NUM_CHANNELS>>1]={
                                         PWM_GEN_0,PWM_GEN_1,PWM_GEN_2,PWM_GEN_3,
                                         PWM_GEN_0};


static int8_t MaxConfiguredChannel = -1; // init to illegal value


bool PWM_TIVA_Init(uint8_t HowMany){    
 static   uint8_t i;

//sanity check
    if ((HowMany == 0) || (HowMany >MAX_NUM_CHANNELS)){
      return false;
  }
  MaxConfiguredChannel = HowMany-1; // note how many we have configured
  
  //Configure PWM Clock to system / 32
  SysCtlPWMClockSet(SYSCTL_PWMDIV_32);

  // Calculate the period for 500 Hz including divide by 32 in PWM clock
  ulPeriod[0] = SysCtlClockGet()/32 / 500; //PWM frequency 500HZ
  // set all of the periods the same initially
  for (i=1; i<(sizeof(Group2GENconst)/sizeof(Group2GENconst[0])); i++){
    ulPeriod[i] = ulPeriod[0];
  }
  // Enable the minimal ports & PWM module.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while (SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB) != true)
    ;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  
  while (SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0) != true)
    ;      
  // Enable the ports for channels 4 & 5 if needed
  if (HowMany > 4){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE) != true)
      ;
  }
  // Enable the ports for channels 6 & 7 if needed
  if (HowMany > 6){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC) != true)
      ;
  }
  // Enable the ports & PWM Module for channels 8 & 9 if needed
  if (HowMany > 8){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD) != true)
      ;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  
    while (SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1) != true)
      ;      
  }
  
    //Configure PWM Options for the generators (1 generator for 2 channels)
    for (i=0; i<HowMany; i+=2){
      PWMGenConfigure(Channel2PWM_MOD[i], Group2GENconst[i>>1], 
                      PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); 
    //Set the Period (expressed in clock ticks)
      PWMGenPeriodSet(Channel2PWM_MOD[i], Group2GENconst[i>>1], ulPeriod[i>>1]);
    }

    //Set PWM duty to initial value
    for ( i = 0; i<HowMany; i++){
      PWM_TIVA_SetDuty( LocalDuty[i], i);
    }
    
    // Enable the PWM generators
    for (i=0; i<HowMany; i+=2){
      PWMGenEnable(Channel2PWM_MOD[i], Group2GENconst[i>>1]); 
    }
    // set synchonous update 
    PWMSyncUpdate(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT | PWM_GEN_2_BIT |
                  PWM_GEN_3_BIT);
    if (HowMany >8){ // only mess with PWM module 1 if we need to
      PWMSyncUpdate(PWM1_BASE, PWM_GEN_0_BIT);
    }
    // Turn on the Output pins as required

    switch ( HowMany ){
    case 10: // intentional fall throughs to the lower channels
      GPIOPinConfigure(GPIO_PD1_M1PWM1);
      GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
      PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    case 9:
      GPIOPinConfigure(GPIO_PD0_M1PWM0);
      GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
      PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    case 8:
      GPIOPinConfigure(GPIO_PC5_M0PWM7);
      GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
      PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    case 7:
      GPIOPinConfigure(GPIO_PC4_M0PWM6);
      GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
      PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    case 6:
      GPIOPinConfigure(GPIO_PE5_M0PWM5);
      GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
      PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
    case 5:
      GPIOPinConfigure(GPIO_PE4_M0PWM4);
      GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
      PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    case 4:
      GPIOPinConfigure(GPIO_PB5_M0PWM3);
      GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
      PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
    case 3:
      GPIOPinConfigure(GPIO_PB4_M0PWM2);
      GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
      PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    case 2:
      GPIOPinConfigure(GPIO_PB7_M0PWM1);
      GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
      PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    case 1:
      GPIOPinConfigure(GPIO_PB6_M0PWM0);
      GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
      PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);      
  }
  return true;
}

bool PWM_TIVA_SetDuty( uint8_t dutyCycle, uint8_t channel)
{
  uint32_t updateVal;
  
  if (channel > MaxConfiguredChannel) // sanity check, reasonable channel number
    return false;
  if ((0 == dutyCycle) || ( dutyCycle >= 100)){ // don't try to calculate with 0 DC
    updateVal = 0;
  }else{ // reasonable duty cycle number, so calculate new pulse width
    updateVal = (ulPeriod[channel>>1]*dutyCycle)/100;    
  }
  // update local copy of DC
  LocalDuty[channel] = dutyCycle;
  // set the new pulse width based on requested DC
  PWMPulseWidthSet(Channel2PWM_MOD[channel], Channel2PWMconst[channel],updateVal);
  return true;
}

bool PWM_TIVA_SetPulseWidth( uint16_t NewPW, uint8_t channel)
{
  if (channel > MaxConfiguredChannel) // sanity check, reasonable channel number
    return false;
  // make sure that the requested PW is less than the period before updating 
  if ( NewPW < ulPeriod[channel>>1]){  
    PWMPulseWidthSet(Channel2PWM_MOD[channel], Channel2PWMconst[channel],NewPW); 
  }    
  return true;
}

/*****************************************************************************
  PWM_TIVA_SetPeriod( uint16_t reqPeriod, uint8_t group)
    sets the requested PWM group's period to the Requested Period
    group 0 = channels 0 & 1
    group 1 = channels 2 & 3
    group 2 = channels 4 & 5
    group 3 = channels 6 & 7
    group 4 = channels 8 & 9
*****************************************************************************/

bool PWM_TIVA_SetPeriod( uint16_t reqPeriod, uint8_t group)
{
  if(group > (MaxConfiguredChannel>>1)){  // sanity check
    return false;
  }
  //Set the Period (expressed in clock ticks)
  ulPeriod[group] = reqPeriod;
  PWMGenPeriodSet(Channel2PWM_MOD[group<<1], Group2GENconst[group], reqPeriod); 
  // Set new Duty after period change
  PWM_TIVA_SetDuty( LocalDuty[group<<1], group<<1);
  PWM_TIVA_SetDuty( LocalDuty[(group<<1)+1], (group<<1)+1);
  return true;
}

 /*****************************************************************************
  PWM_TIVA_SetFreq( uint16_t reqPeriod, uint8_t group)
    sets the requested PWM group's frequency to the Requested Frequency, in Hz
    group 0 = channels 0 & 1
    group 1 = channels 2 & 3
    group 2 = channels 4 & 5
    group 3 = channels 6 & 7
    group 4 = channels 8 & 9
*****************************************************************************/

bool PWM_TIVA_SetFreq( uint16_t reqFreq, uint8_t group)
{
  if(group > (MaxConfiguredChannel>>1)){ // sanity check
    return false;
  }
  //Use the Frequency (expressed in Hz) to calculate a new period
  ulPeriod[group] = SysCtlClockGet()/32 /reqFreq;
  // apply the new period
  PWM_TIVA_SetPeriod( ulPeriod[group], group);
  return true;
}

