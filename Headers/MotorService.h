#ifndef MotorService_H
#define MotorService_H

#include "ES_Types.h"
#include "ES_Configure.h"

// typedefs for the states
// State definitions for use with the query function
typedef enum { InitMotorStep, MotorDCState} MotorServiceState_t;

bool InitMotorService (uint8_t Priority);
bool PostMotorService (ES_Event ThisEvent);
ES_Event RunMotorService(ES_Event ThisEvent);
uint32_t GetRPM(void);
void PWMUpdate(uint32_t PWM_INTERVAL);

#endif /* MOTORSERVICE_H */