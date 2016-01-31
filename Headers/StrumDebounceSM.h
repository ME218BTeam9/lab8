/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef STURMDEBOUNCE_H
#define STRUMDEBOUNCE_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {Debouncing, Ready2Sample } SDState_t ;


// Public Function Prototypes

bool InitStrumDebounceSM ( uint8_t Priority );
bool PostStrumDebounceSM( ES_Event ThisEvent );
ES_Event RunStrumDebounceSM( ES_Event ThisEvent );
bool CheckStrumEvents(void);
SDState_t QueryButtonDebounceSM ( void );


#endif /* STRUMDEBOUNCE_H */
