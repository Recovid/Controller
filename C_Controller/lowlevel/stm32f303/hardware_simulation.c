#include "hardware_simulation.h"

#include <time.h>
#include <stdio.h>
#include <string.h>
#include "hardware_serial.h"
#include "stm32f3xx_hal.h"
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

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	}
	/** Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	}
}


#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */

	// TODO Implement UART1 for printf LOG

	return ch;
}

// ------------------------------------------------------------------------------------------------


bool init_ihm(const char* pathInputFile, const char* pathOutputFile)
{
	HAL_Init();

	SystemClock_Config();

	return hardware_serial_init(NULL);
}

bool send_ihm(const char* frame)
{

	return hardware_serial_write_data((unsigned char *) frame, strlen(frame));
}

int recv_ihm()
{
	static long last_blocked_s = 0;
	
	unsigned char blocking_read = 0;

    int t_s = hardware_serial_read_data(&blocking_read, sizeof(char));;

    if (t_s > 0) {

        return blocking_read;
    }
    return EOF;
}
