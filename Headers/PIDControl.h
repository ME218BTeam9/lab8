#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "ES_Types.h"
#include "ES_Configure.h"

// typedefs for the states
// State definitions for use with the query function

bool InitPIDService(uint8_t Priority);
bool PostPIDService (ES_Event ThisEvent);
ES_Event RunPIDService(ES_Event ThisEvent);


#endif /* PIDCONTROL_H */