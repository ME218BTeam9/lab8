/****************************************************************************
 
  Header file for template service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef SPIService_H
#define SPIService_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitSPIService ( uint8_t Priority );
bool PostSPIService( ES_Event ThisEvent );
ES_Event RunSPIService( ES_Event ThisEvent );
void EOTResponse( void );

#endif /* SPIService_H */

