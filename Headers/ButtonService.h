/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef BUTTONSERVICE_H
#define BUTTONSERVICE_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */


// typedefs for the states
// State definitions for use with the query function
typedef enum {Debouncing, Ready2Sample } ButtonState_t ;


// Public Function Prototypes

bool InitButtonDebounce (uint8_t Priority);
bool PostButtonDebounce(ES_Event ThisEvent);
ES_Event RunButtonDebounce( ES_Event ThisEvent );
bool CheckButtonEvents();



#endif /* BUTTONSERVICE_H */
