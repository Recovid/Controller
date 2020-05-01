#include <stdint.h>
#include <stdio.h>
#include "stm32f3xx.h"
#include "log_timings.h"
#include "common.h"
#include <string.h> /* strlen */

typedef struct
{
	uint8_t  id;    /* id: MSB=1 start, else end. 7 low bits = id. */
	uint16_t date;  /* date in us from TIM6. there are overflows   */
} log_time_entry;

/* save logs before sending externally (through uart..)*/
/* 3 bytes per entry */
volatile log_time_entry log_time_tab[LOG_TIME_SIZE];
volatile int log_time_current_size; /* current size */


void log_time_initHw(){
    //start TIM6
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    __asm("nop");
    RCC->APB1RSTR |=  RCC_APB1RSTR_TIM6RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM6RST;
    __asm("nop");

    //configure TIM6
    TIM6->PSC = 72-1;     //tick@1us
    TIM6->EGR = TIM_EGR_UG; //update event => load PSC
    TIM6->CR1 = TIM_CR1_CEN; //enable, all other fields to 0

	//init data
	log_time_current_size = 0;
	for(int i = 0;i<LOG_TIME_SIZE;i++){
		log_time_tab[i].id  = 0;
		log_time_tab[i].date= 0;
	}
}

void log_time_event(int id)
{
    __disable_irq();
	const uint16_t date = TIM6->CNT;
	log_time_tab[log_time_current_size].id   = id;
	log_time_tab[log_time_current_size].date = date;
	/* if full, restart */
	log_time_current_size = (log_time_current_size+1)%LOG_TIME_SIZE;
    __enable_irq();
}

void log_time_dump()
{
	const int indexW = log_time_current_size;
	dbg_printf("dump log timings\r\n");
	for(int i = 0;i<LOG_TIME_SIZE;i++)
	{
		int indexR = (i+indexW)%LOG_TIME_SIZE;
		const uint8_t  id   = log_time_tab[indexR].id;
		const uint16_t date = log_time_tab[indexR].date;
		if(id != 0 || date !=0)
		{
			char c;
			if(id & LOG_TIME_EVENT_START) c = 's';
			else c = 'e';
			dbg_printf("%c\t%d\t%d\r\n",c,id & 0x7F,date);
		}
	}
}

/* store the task handle of tasks */
TaskHandle_t idToTaskHandle[4];
void log_time_init_task(const char *name, int id)
{
	TaskHandle_t handle = xTaskGetHandle(name);
    if(id < LOG_TIME_NB_TASKS) {
		idToTaskHandle[id] = handle;
	}
}

void log_time_event_task(TaskHandle_t t,int start)
{
	int i=0;
	int found = 0;
	while(i<LOG_TIME_NB_TASKS && !found)
	{
		found = (t==idToTaskHandle[i]);
		if(!found) i++;
	}
	if(found) log_time_event(start | i);
}
