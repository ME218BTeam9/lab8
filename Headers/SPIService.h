/****************************************************************************
 
  Header file for template service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef SPIService_H
#define SPIService_H

#include "ES_Types.h"
#include "ES_Configure.h"
// Public Function Prototypes

bool InitSPIService ( uint8_t Priority );
bool PostSPIService( ES_Event ThisEvent );
ES_Event RunSPIService( ES_Event ThisEvent );
void EOTResponse( void );
typedef enum {SPIinit, Send, Wait2Send} SPIServiceState_t;
typedef enum {Stop, CW90, CW45, CCW90, CCW45, ForwardFull, ForwardHalf, ReverseHalf,ReverseFull, Align, Drive2Tape} Command_t; 
#endif /* SPIService_H */

