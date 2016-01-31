#ifndef _PWM8_TIVA_
#define _PWM8_TIVA_
#include <stdint.h>

void PWM8_TIVA_Init(void);
void PWM8_TIVA_SetDuty( uint8_t dutyCycle, uint8_t channel);
void PWM8_TIVA_SetPeriod( uint16_t reqPeriod, uint8_t group);
void PWM8_TIVA_SetFreq( uint16_t reqFreq, uint8_t group);
void PWM8_TIVA_SetPulseWidth( uint16_t NewPW, uint8_t channel);

#endif
