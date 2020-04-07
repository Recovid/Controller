#include "hardware_simulation.h"

#include <time.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"

// ------------------------------------------------------------------------------------------------
/*
Config USART2_RY DMA1 Stream5 Mode Circular


*/

//! Simulated clock for testing purposes
static long clock_ms = 0;

long get_time_ms()
{
    return HAL_GetTick();;
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
#define TXBUFFER_SIZE 100

#define FRAMETIMEOUTMS 10

uint8_t txbuff[TXBUFFER_SIZE];
static uint8_t rxbuffer[UART_DMA_BUFFER_SIZE];
size_t dma_head = 0, dma_tail = 0;

UART_HandleTypeDef huart2;

bool init_ihm(const char* pathInputFile, const char* pathOutputFile)
{
    huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		return false;
	}
	HAL_UART_Receive_DMA(&huart2, rxbuffer, UART_DMA_BUFFER_SIZE);
	return true;
}

bool send_ihm(const char* frame)
{
    if (!frame || *frame=='\0') return 0;
    size_t len = strlen((char *) frame);
    return HAL_UART_Transmit(&huart2, frame, len, 1000) == HAL_OK;
}

int recv_ihm()
{
	__disable_irq();
	dma_tail = UART_DMA_BUFFER_SIZE - huart2.hdmarx->Instance->NDTR;
	__enable_irq();

	if (dma_tail != dma_head) {
		size_t tm1=dma_tail == 0 ? UART_DMA_BUFFER_SIZE-1 : dma_tail-1;
		long starttime = HAL_GetTick();
		while(rxbuffer[tm1] != '\n')
		{
			if(HAL_GetTick()-starttime>FRAMETIMEOUTMS) //RESET buffer on timeout
			{
				dma_head=dma_tail;
				return (char)EOF;
			}
			HAL_Delay(1);
			__disable_irq();
			dma_tail = UART_DMA_BUFFER_SIZE - huart2.hdmarx->Instance->NDTR;
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
		return (char)EOF;
	}
}
