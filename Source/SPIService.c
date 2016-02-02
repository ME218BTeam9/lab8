/****************************************************************************
 Module
   SPIService.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "SPIService.h"
#include "BITDEFS.H"
#include "termio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"  // Define PART_TM4C123GH6PM in project
#include "driverlib/gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_timer.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "MotorService.h"

/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 4
#define TicksPerMS    40000
#define ALL_BITS     (0xff<<2)
#define CPSDVSR 0x0014 //(20)
#define SCR 0x0009
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void SPIInit(void);
void EOTResponse( void );
static bool SPISendSlave( void );
static uint16_t NewSlaveRead;
static uint16_t LastSlaveRead;
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitSPIService ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;

  SPIInit();
	ES_Timer_InitTimer(SPI_TIMER, 100);
	LastSlaveRead = 0x00;
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostTemplateService

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostSPIService( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}


static void SPIInit(void){

// Enable clock to the GPIO port (A)
HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
// Enable clock to the SSI Module 0
HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCSSI_R0;
// Wait for GPIO port A to be ready
while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R0 ) != SYSCTL_PRSSI_R0 ) ;
// Program pins A2-A5 as alternate functions on the GPIO to use SSI (Set High)
HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);
// Changes: GPIO_O_ODR
// SS (Pin A3) now configured as open drain, see TIVA Data Sheet page 676 
//HWREG(GPIO_PORTA_BASE + GPIO_O_ODR) |= (BIT3HI);
// Select the SSI alternate functions on pins A2-A5
// Set mux position on GPIOPCTL to select the use of the pins (2 for SSI) 
HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) |=(HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & ~0x00ffff00) + (2 << ( 2 * BitsPerNibble)) +
(2 << ( 3 * BitsPerNibble)) + (2 << ( 4 * BitsPerNibble)) + (2 << ( 5 * BitsPerNibble));
// Program port lines for pins A2-A5 as digital I/O
HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);
// Program required data directions on the port lines (2/3/5 Output, 4 Input)
HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) |= (BIT2HI | BIT3HI | BIT5HI);
HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) &= BIT4LO;
// Program pullup resistor on clock line (A2), (changes) pull down resistor on receive line (A4)
HWREG(GPIO_PORTA_BASE + GPIO_O_PUR) |= (BIT2HI); 
//HWREG(GPIO_PORTA_BASE + GPIO_O_PDR) |= (BIT4HI);
// Wait for the SSI0 Module 0 to be ready
while((HWREG(SYSCTL_RCGCSSI) & SYSCTL_RCGCSSI_R0) != SYSCTL_RCGCSSI_R0);
// Make sure SSI Module 0 is disabled before programming mode bits
HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;
// Select master mode (MS) & TXRIS interrupt indicating end of transmission (EOT)
HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_MS; 
HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
// Configure the SSI clock source to the system clock
HWREG(SSI0_BASE + SSI_O_CC) |= SSI_CC_CS_SYSPLL; 
// Configure the clock pre-scaler  = 20
HWREG(SSI0_BASE + SSI_O_CPSR) |= CPSDVSR;
// Configure the clock rate (SCR), phase (SPH) & polarity (SPO), mode (FRF), and data size (DSS)
HWREG(SSI0_BASE + SSI_O_CR0) |= SCR<<8;
// Set SCR to 8
HWREG(SSI0_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO); // Set SPH and SPO to 1
HWREG(SSI0_BASE + SSI_O_CR0) |= SSI_CR0_FRF_MOTO; // freescale SPI Frame Format 
// Set frame mode (pg 969)
HWREG(SSI0_BASE + SSI_O_CR0) |= SSI_CR0_DSS_8; // Set data size to 8-bit
// Locally enable interrupts on TXRIS
	HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
// Set NVIC enable
HWREG(NVIC_EN0) |= BIT7HI;
// Make sure interrupts are enables globally
__enable_irq( );
// Make sure that the SSI is enabled for operation
HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
printf("\r\nInit SPI successfully.");
}


/**************************************************************************** Function
EOTResponse
Parameters
none
Returns none
Description
End of Transfer interrupt response, checks command received from Reckoning System and saves it to an array
****************************************************************************/
void EOTResponse( void )
{
uint16_t ThisRead;
ES_Event PostEvent;
//LastSlaveRead = NewSlaveRead;
// Make sure the SPI it not transmitting/receiving before reading receive register
if( (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_BSY) != SSI_SR_BSY ) {
            // Check if the data input FIFO queue is full
//if( (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RFF) == SSI_SR_RFF ) {
// Read 2 bytes received from the RS into NewSlaveRead

                  // Save each read as 16bit
                  ThisRead = HWREG(SSI0_BASE + SSI_O_DR);
	
                  // Store thisRead as newSlaveRead
                  NewSlaveRead = ThisRead;
				//printf("Receive %x",NewSlaveRead);
					PostEvent.EventType = ES_EOT;
					PostSPIService(PostEvent);
        }
HWREG(NVIC_EN0) &= ~BIT7HI;

   // }
}

static bool SPISendSlave( void )
{
// Initialize ReturnFlag as false (in case of unsuccessful transfer init)
bool ReturnFlag = false;
//int i;
// Check if the data output FIFO queue is empty
if( (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE) == SSI_SR_TFE )
{
            // Write query byte to the data output register
      HWREG(SSI0_BASE + SSI_O_DR) = 0xAA;
			//printf("\r\n send AA");
	while ((HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_BSY) != SSI_SR_BSY) ;

      // Set the SSI transmit interrupt as unmasked to enable EOT interrupt (SSI_IM_TXIM)
      HWREG(NVIC_EN0) |= BIT7HI;
ReturnFlag = true;
}
// Set ReturnFlag to true to indicate a successful write
// Return if write was successful
return ReturnFlag;
}


// Skips if we are setting commands manually
// Post NewRead event for the DRS to handle
//ES_Event NewEvent = {EV_DRSNewRead};
//PostMaster(NewEvent);

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunSPIService( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
	ES_Event PostEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  if(ThisEvent.EventType == ES_TIMEOUT && ThisEvent.EventParam == SPI_TIMER){
		ES_Timer_InitTimer(SPI_TIMER, 10);
		if(SPISendSlave()== false){
			printf("\r\n Fail to send the query");
		}
	}
	else if (ThisEvent.EventType == ES_EOT){
		if(NewSlaveRead != LastSlaveRead){
			PostEvent.EventType = NEW_COMMAND;
			switch(NewSlaveRead)
			{

			  case 0x00:
					{
					PostEvent.EventParam = Stop;
					PostMotorService(PostEvent);
					printf("\r\nStop");
					break;
				}
					
			  case 0x02:
					{
					PostEvent.EventParam = CW90;
					PostMotorService(PostEvent);
					printf("\r\nClockwise90");
					break;
				}
											
				case 0x03:
					{
					PostEvent.EventParam = CW45;
					PostMotorService(PostEvent);
					printf("\r\nClockwise45");
					break;
				}
				case 0x04:
					{
					PostEvent.EventParam = CCW90;
					PostMotorService(PostEvent);
					printf("\r\nCounterClockwise90");
						break;
				}			
				case 0x05:
					{
					PostEvent.EventParam = CCW45;
					PostMotorService(PostEvent);
					printf("\r\nCounterClockwise45");
					break;
				}
				case 0x08:
					{
					PostEvent.EventParam = ForwardHalf;
					PostMotorService(PostEvent);
					printf("\r\nForward half speed");
					break;
				}	
				case 0x09:
					{
					PostEvent.EventParam = ForwardFull;
					PostMotorService(PostEvent);
					printf("\r\nForward full speed");
					break;
				}
				case 0x10:
					{
					PostEvent.EventParam = ReverseHalf;
					PostMotorService(PostEvent);
					printf("\r\nReverse half speed");
					break;
				}		
				case 0x11:
					{
					PostEvent.EventParam = ReverseFull;
					PostMotorService(PostEvent);
					printf("\r\nReverse full speed");
					break;
				}	
				case 0x20:
					{
					PostEvent.EventParam = Align;
					PostMotorService(PostEvent);
					printf("\r\nAlign the beacon");
					break;
				}
				case 0x40:
					{
					PostEvent.EventParam = Drive2Tape;
					PostMotorService(PostEvent);
					printf("\r\nDrive to tape detected");
					break;
				}
				default :
					break;
		}
	}
		LastSlaveRead = NewSlaveRead;
}
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

