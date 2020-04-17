#include <ihm_hardware.h>
#include <time.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"

// ------------------------------------------------------------------------------------------------
//! OS simulation

//! Simulated clock for testing purposes
static long clock_ms = 0;

long get_time_ms()
{
    return HAL_GetTick();
}

long wait_ms(long t_ms)
{
    return clock_ms += t_ms; // simulated clock for testing purposes
}

bool soft_reset()
{
    return true;
}

// ------------------------------------------------------------------------------------------------
//! IHM simulation based on stdin/stdout


#define UART_DMA_BUFFER_SIZE 2048
#define PARSER_MESSAGE_LIST_SIZE 10
#define PARSER_MESSAGE_SIZE 200

#define FRAMETIMEOUTMS 10

static uint8_t rxbuffer[UART_DMA_BUFFER_SIZE];
size_t dma_head = 0, dma_tail = 0;

extern UART_HandleTypeDef huart4;

bool init_ihm(const char* pathInputFile, const char* pathOutputFile)
{
	HAL_UART_Receive_DMA(&huart4, rxbuffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool send_ihm(const char* frame)
{
    if (!frame || *frame=='\0') return 0;
    size_t len = strlen( frame);
    return HAL_UART_Transmit_DMA(&huart4, frame, len) == HAL_OK;
}

int recv_ihm()
{

	__disable_irq();
	dma_tail = UART_DMA_BUFFER_SIZE - huart4.hdmarx->Instance->CNDTR;
	__enable_irq();

	if (dma_tail != dma_head) {
		size_t tm1=dma_tail == 0 ? UART_DMA_BUFFER_SIZE-1 : dma_tail-1;
		long starttime = HAL_GetTick();
		while(rxbuffer[tm1] != '\n')
		{
			if(HAL_GetTick()-starttime>FRAMETIMEOUTMS) //RESET buffer on timeout
			{
				dma_head=dma_tail;
				return EOF;
			}
			HAL_Delay(1);
			__disable_irq();
			dma_tail = UART_DMA_BUFFER_SIZE - huart4.hdmarx->Instance->CNDTR;
			__enable_irq();
			tm1=dma_tail == 0 ? UART_DMA_BUFFER_SIZE-1 : dma_tail-1;
		}
		char ret = rxbuffer[dma_head];
		dma_head = dma_head == UART_DMA_BUFFER_SIZE - 1 ?
												0 : dma_head + 1;
		return ret;
	}
	else
	{
		return EOF;
	}
}
