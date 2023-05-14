#include <stdint.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "TM4C123GH6PM.h"
#include "macros.h"

#define STOP 1
#define OPEN 2
#define CLOSE 3


//define a Semaphore handle
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xMutex;
xQueueHandle xQueue;

void sensorButtonInit(void);
void timer0Init(void);
void timer0_Delay(int time);
void motorInit(void);
void limitInit(void);
void buttonsInit(void);
void lockButtonInit(void);

void jamTask(void* pvParameters) {
    //TAKE SEMAPHORE
    xSemaphoreTake(xBinarySemaphore, 0);
    while (1) {
        //TAKE SEMAPHORE
        xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

        // Set motor direction to reverse
        GPIO_PORTF_DATA_R |= (1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
       
        timer0_Delay(2000);

				// Stop motor
			  GPIO_PORTF_DATA_R &= ~(1 << 3);
        GPIO_PORTF_DATA_R &= ~(1 << 2);
   }  
}


void recieveQueue(void* pvParameters) {
		int Val;
		portBASE_TYPE xStatus;
		while(1)
		{
			xStatus=xQueueReceive(xQueue,&Val,portMAX_DELAY);		
			if(Val==STOP)
			{
					GPIO_PORTF_DATA_R &= ~(1 << 3);
					GPIO_PORTF_DATA_R &= ~(1 << 2);
			}
			else if(Val==OPEN)
			{
					GPIO_PORTF_DATA_R |= (1 << 3);
					GPIO_PORTF_DATA_R &= ~(1 << 2);
			}
			else if(Val==CLOSE)
			{
					GPIO_PORTF_DATA_R &= ~(1 << 3);
					GPIO_PORTF_DATA_R |= (1 << 2);
			}		
		}
}

void driver(void* pvParameters){
		int Val;
		int state;
	  portBASE_TYPE xStatus;
	  //Val= (int) pvParameters;
		while(1)
		{
			xSemaphoreTake(xMutex,portMAX_DELAY );
			if (GET_BIT(GPIO_PORTD_DATA_R,0)==0){ //pullup
					Val=OPEN;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,0)==0) //still pressing then it is manual
						{
							while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,0)==1 | GET_BIT(GPIO_PORTD_DATA_R,1)==0));
						}
			 
					else if (GET_BIT(GPIO_PORTD_DATA_R,0)==1) // then it will be automatic
					{   
								while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,0)==0 | GET_BIT(GPIO_PORTD_DATA_R,1)==0)); 
					}
					Val=STOP;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
			}
			if (GET_BIT(GPIO_PORTD_DATA_R,1)==0){ //pullup
						Val=CLOSE;
						xStatus = xQueueSendToBack(xQueue,&Val,0);
						vTaskDelay(1000); 
						if (GET_BIT(GPIO_PORTD_DATA_R,1)==0) //still pressing then it is manual
						{

						while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,1)==1| GET_BIT(GPIO_PORTD_DATA_R,0)==0));
						}
			 
						else if (GET_BIT(GPIO_PORTD_DATA_R,1)==1) // then it will be automatic
						{   
									while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,1)==0 | GET_BIT(GPIO_PORTD_DATA_R,0)==0)); 
						}
						Val=STOP;
						xStatus = xQueueSendToBack(xQueue,&Val,0);				
				}
// Lock Switch
				if (GET_BIT(GPIO_PORTA_DATA_R,3)==0){
				
				vTaskPrioritySet(NULL,2);
				
				}
				else
				{
					vTaskPrioritySet(NULL,1);
				}
				portBASE_TYPE	 lockLed= uxTaskPriorityGet(NULL);
				if(lockLed==2)
				{
					GPIO_PORTF_DATA_R |=0x02;
				}
				else
				{
					GPIO_PORTF_DATA_R &=~(0x02);
				}
			xSemaphoreGive(xMutex);
			//vTaskDelay(100);
				timer0_Delay(200);
		}
	}

void passenger(void* pvParameters){
		int Val;
		portBASE_TYPE xStatus;
		while(1)
		{
			xSemaphoreTake(xMutex,portMAX_DELAY );
			if (GET_BIT(GPIO_PORTD_DATA_R,2)==0){ //pullup
					Val=OPEN;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,2)==0) //still pressing then it is manual
						{
							while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==0));
						}
			 
					else if (GET_BIT(GPIO_PORTD_DATA_R,2)==1) // then it will be automatic
						{   
							while(!(GET_BIT(GPIO_PORTA_DATA_R,7)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==0 | GET_BIT(GPIO_PORTD_DATA_R,3)==0)); 
						}
					Val=STOP;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
			}
			
			if (GET_BIT(GPIO_PORTD_DATA_R,3)==0){ //pullup
					Val=CLOSE;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
					vTaskDelay(1000); 
					if (GET_BIT(GPIO_PORTD_DATA_R,3)==0) //still pressing then it is manual
					{
						while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==1 | GET_BIT(GPIO_PORTD_DATA_R,2)==0));
					}
				 
					else if (GET_BIT(GPIO_PORTD_DATA_R,3)==1) // then it will be automatic
					{   
						while(!(GET_BIT(GPIO_PORTD_DATA_R,6)==1 | GET_BIT(GPIO_PORTD_DATA_R,3)==0 | GET_BIT(GPIO_PORTD_DATA_R,2)==0)); 
					}
					Val=STOP;
					xStatus = xQueueSendToBack(xQueue,&Val,0);
			}
			xSemaphoreGive(xMutex);
			vTaskDelay(100); 
		}
}

                         /*main function*/
/*------------------------------------------------------------------------*/
int main( void )
{
	  xQueue = xQueueCreate(2,sizeof(int));
	  xMutex = xSemaphoreCreateMutex(); 
    sensorButtonInit();
	  lockButtonInit();
		buttonsInit();
		limitInit();
		motorInit();
	  timer0Init();
		__ASM("CPSIE i");
		
		vSemaphoreCreateBinary(xBinarySemaphore);

	if( xBinarySemaphore != NULL )
		{
			xTaskCreate( jamTask, "jamTask", 200, NULL, 5, NULL );
			
			xTaskCreate( recieveQueue, "recieveQueue", 200, NULL, 3, NULL );
			
			xTaskCreate( passenger, "passenger", 270, NULL, 1, NULL );
			
			xTaskCreate( driver, "driver", 270, NULL, 1, NULL );
			
			vTaskStartScheduler();
		}
}

/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
//Port-A handler
void GPIOA_Handler(void)
{
    //Clear Interrupt Flag
    GPIO_PORTA_ICR_R |= (1<<2);
	
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	  /* 'Give' the semaphore to unblock the task. */
    xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


void timer0Init(void)
{
	SYSCTL_RCGCTIMER_R |= 0x01;
	TIMER0_CTL_R=0x00;
	TIMER0_CFG_R=0x00;
	TIMER0_TAMR_R=0x02;
	TIMER0_CTL_R=0x03;
}

void timer0_Delay(int time)
{
	TIMER0_CTL_R=0x00;
	TIMER0_TAILR_R=16000*time-1;
	TIMER0_ICR_R=0x01;
	TIMER0_CTL_R |=0x03;
	while((TIMER0_RIS_R & 0x01)==0);
}


void sensorButtonInit(void)
{
	
	   //Enable Port A
    SYSCTL_RCGCGPIO_R |= 0x01;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 2 in Port A as input
    GPIO_PORTA_DIR_R &= ~(1 << 2);
    GPIO_PORTA_CR_R |= (1 << 2);
    GPIO_PORTA_PUR_R |= (1 << 2);	
    GPIO_PORTA_DEN_R |= (1 << 2);
	
    //Enable Interrupt on PORT A & set priority to 0
	  NVIC_PRI0_R |= (1<<7) | (1<<6) | (1<<5);
    NVIC_EN0_R |= (1<<0);
    
	
    //Configure Interrupt on Pin 2 to detect FALLING edge
    GPIO_PORTA_IM_R &=0;
    GPIO_PORTA_IS_R &= ~(1<<2);
    GPIO_PORTA_IEV_R &= ~(1<<2);
    GPIO_PORTA_ICR_R |= (1<<2);
    GPIO_PORTA_IM_R |= (1<<2);	
}

void lockButtonInit(void)
{

    //Configure Pin 3 in Port A as input
		GPIO_PORTA_DIR_R &= ~(1 << 3);
		GPIO_PORTA_CR_R |= (1<<3);
    GPIO_PORTA_PUR_R |= (1<<3);
    GPIO_PORTA_DEN_R |= (1<<3);
	

	
}

void buttonsInit(void)
{
		//Enable Port D
    SYSCTL_RCGCGPIO_R |= 0x08;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 0 -> 3 in Port D as input
    GPIO_PORTD_DIR_R &= ~((1 << 0)|(1<<1)|(1<<2)|(1<<3));
    GPIO_PORTD_CR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
    GPIO_PORTD_PUR_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);	
    GPIO_PORTD_DEN_R |= (1 << 0)|(1<<1)|(1<<2)|(1<<3);
}

void limitInit(void)
{

    SYSCTL_RCGCGPIO_R |= 0x08;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pins FOR limit switches in Port D and A as inputs
    GPIO_PORTD_DIR_R &= ~(1 << 6);
    GPIO_PORTD_CR_R |= (1 << 6);
    GPIO_PORTD_PUR_R |= (1 << 6);
    GPIO_PORTD_DEN_R |= (1 << 6);
		GPIO_PORTA_DIR_R &= ~(1 << 7);
    GPIO_PORTA_CR_R |= (1<<7);
    GPIO_PORTA_PUR_R |= (1<<7);
    GPIO_PORTA_DEN_R |= (1<<7);
}


void motorInit(void)
{
    SYSCTL_RCGCGPIO_R |= 0x20;
	
    __asm__("NOP; NOP; NOP; NOP;");
	
    //Configure Pin 2,3 in Port F as inputs 
    GPIO_PORTF_DIR_R |= ((1 << 2)|(1<<3));
    GPIO_PORTF_CR_R |= (1 << 2)|(1<<3);
    GPIO_PORTF_DEN_R |= (1 << 2)|(1<<3);
	
		GPIO_PORTF_DIR_R |= (1 << 1);
		GPIO_PORTF_CR_R |= (1<<1);
   // GPIO_PORTA_PUR_R |= (1<<2);
    GPIO_PORTF_DEN_R |= (1<<1);
}
