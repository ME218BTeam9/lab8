#ifndef ADService_H
#define ADService_H

#include "ES_Types.h"
#include "ES_Configure.h"

// typedefs for the states
// State definitions for use with the query function

bool InitADService (uint8_t Priority);
bool PostADService (ES_Event ThisEvent);
ES_Event RunADService(ES_Event ThisEvent);
uint32_t GetInterval(); // time converted from the pot to pass into MotorService

#endif /* ADSERVICE_H */