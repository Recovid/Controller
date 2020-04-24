// @Author: Inventhys
// @Date:   2020-04-08 17:14:01
// @Last Modified by:   Inventhys
// @Last Modified time: 2020-04-08 17:14:01
//
// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------

#include "hardware_serial.h"

#include <stdio.h>

#include "ifl_deque.h"
#include "stm32f303xe.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_dma.h"
#include "stm32f3xx_hal_uart.h"
#include "stm32f3xx_hal_rcc.h"
#include "stm32f3xx_hal_gpio.h"

// --------------------------------------------------------------------------------------------------------------------
// ----- local constant macros
// --------------------------------------------------------------------------------------------------------------------

#define UART_TX_BUFFER_SIZE             (4096)
#define UART_RX_BUFFER_SIZE             (1024)
#define UART_TX_DMA_BUFFER_SIZE         (512)
#define UART_TX_SYNC_TIMEOUT_MS         (2000)

#define DEFAULT_RX_LINE_TIMEOUT         (30)
#define UART_RX_DMA_BUFFER_SIZE         (512)

#define UART_COM_IHM                    	(0)
#define UART_ID_MAX                     	(1)


#define UART_COM_IHM_INSTANCE                  UART4
#define UART_COM_IHM_IRQ                       UART4_IRQn
#define UART_COM_IHM_IRQHandler                UART4_IRQHandler
#define UART_COM_IHM_CLK_ENABLE                __HAL_RCC_UART4_CLK_ENABLE

#define UART_COM_IHM_DMA_ENABLE                __HAL_RCC_DMA2_CLK_ENABLE
#define UART_COM_IHM_DMA_CHANNEL_TX            DMA2_Channel5
#define UART_COM_IHM_DMA_CHANNEL_TX_IRQ        DMA2_Channel5_IRQn
#define UART_COM_IHM_DMA_CHANNEL_TX_IRQHandler DMA2_Channel5_IRQHandler
#define UART_COM_IHM_DMA_CHANNEL_RX            DMA2_Channel3
#define UART_COM_IHM_DMA_CHANNEL_RX_IRQ        DMA2_Channel3_IRQn
#define UART_COM_IHM_DMA_CHANNEL_RX_IRQHandler DMA2_Channel3_IRQHandler

#define UART_COM_IHM_GPIO_BANK                 GPIOC
#define UART_COM_IHM_ALTERNATE                 GPIO_AF5_UART4
#define UART_COM_IHM_PIN                       GPIO_PIN_10|GPIO_PIN_11
#define UART_COM_IHM_GPIO_CLK_ENABLE           __HAL_RCC_GPIOC_CLK_ENABLE

// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
typedef enum ifl_hal_uart_result_t
{
    IFL_HAL_UART_ERROR = 0,
    IFL_HAL_UART_SUCCESS,
}ifl_hal_uart_result_t;
typedef enum idle_line_state_t
{
    INVALIDE_IDLE_LINE_STATE = 0,
    IDLE_LINE_DETECTED,
    IDLE_LINE_NOT_DETECTED,
}idle_line_state_t;

typedef struct uart_object_t
{
    UART_HandleTypeDef  husart;
    DMA_HandleTypeDef   hdma_usart_tx;
    DMA_HandleTypeDef   hdma_usart_rx;
    IFL_DEQUE_DECLARE(uart_tx_buffer, uint8_t, UART_TX_BUFFER_SIZE);
    IFL_DEQUE_DECLARE(uart_rx_buffer, uint8_t, UART_RX_BUFFER_SIZE);
    uint8_t dma_buffer_tx[UART_TX_DMA_BUFFER_SIZE];
    uint8_t dma_buffer_rx[UART_RX_DMA_BUFFER_SIZE];
}uart_object_t;

// --------------------------------------------------------------------------------------------------------------------
// ----- local variables
// --------------------------------------------------------------------------------------------------------------------

static uart_object_t uart_object[UART_ID_MAX];

// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------

static ifl_hal_uart_result_t ifl_hal_uart_process_tx_data(uint8_t const id);
static ifl_hal_uart_result_t ifl_hal_uart_process_rx_data(idle_line_state_t idle_line_state, uint8_t const id);

// --------------------------------------------------------------------------------------------------------------------
// ----- local functions
// --------------------------------------------------------------------------------------------------------------------


static ifl_hal_uart_result_t ifl_hal_uart_process_rx_data(idle_line_state_t idle_line_state, uint8_t const id)
{
    static uint16_t last_size[UART_ID_MAX] = { UART_RX_DMA_BUFFER_SIZE };
    uint16_t current_size = (uint16_t) __HAL_DMA_GET_COUNTER(uart_object[id].husart.hdmarx);
    uint16_t length_to_read = 0;
    uint16_t start_read = 0;

    __disable_irq();

    if((idle_line_state == IDLE_LINE_DETECTED) && (current_size == UART_RX_DMA_BUFFER_SIZE))
    {
        __enable_irq();
        return IFL_HAL_UART_SUCCESS;
    }

    start_read = (last_size[id] < UART_RX_DMA_BUFFER_SIZE) ? (uint16_t)(UART_RX_DMA_BUFFER_SIZE - last_size[id]) : 0;

    if(idle_line_state == IDLE_LINE_DETECTED)
    {
        length_to_read = (last_size[id] < UART_RX_DMA_BUFFER_SIZE) ? (uint16_t)(last_size[id] - current_size) : (uint16_t)(UART_RX_DMA_BUFFER_SIZE - current_size);
        last_size[id] = current_size;
    }
    else
    {
        length_to_read = (uint16_t)(UART_RX_DMA_BUFFER_SIZE - start_read);
        last_size[id] = UART_RX_DMA_BUFFER_SIZE;
    }


    for(uint16_t i = start_read; i < (length_to_read + start_read); i++)
    {
        IFL_DEQUE_PUSH_BACK(&uart_object[id].uart_rx_buffer, &uart_object[id].dma_buffer_rx[i % UART_RX_DMA_BUFFER_SIZE]);
    }

    if(idle_line_state == IDLE_LINE_DETECTED || length_to_read == UART_RX_DMA_BUFFER_SIZE)
    {

    }
    __enable_irq();
    return IFL_HAL_UART_SUCCESS;
}

static ifl_hal_uart_result_t ifl_hal_uart_process_tx_data(uint8_t const id)
{
    if (!IFL_DEQUE_EMPTY(&uart_object[id].uart_tx_buffer) &&
        (HAL_UART_GetState(&uart_object[id].husart) != HAL_UART_STATE_BUSY_TX) &&
        (HAL_UART_GetState(&uart_object[id].husart) != HAL_UART_STATE_BUSY_TX_RX))
    {
        // nothing in buffer, and UART is ready: start a new DMA request
        uint16_t size = IFL_DEQUE_SIZE(&uart_object[id].uart_tx_buffer);
        if (size > UART_TX_DMA_BUFFER_SIZE)
        {
            size = UART_TX_DMA_BUFFER_SIZE;
        }
        else
        {
            // don't change size, as it is less than the buffer's size
        }

        for (uint16_t i = 0; i < size; i++)
        {
            uint8_t* byte = NULL;
            IFL_DEQUE_FRONT(&uart_object[id].uart_tx_buffer, byte);
            if (byte != NULL)
            {
                uart_object[id].dma_buffer_tx[i] = *byte;
            }
            else
            {
                // should not happen, TODO: this should be logged somehow, maybe in some kind of statistics?
            }
            IFL_DEQUE_POP_FRONT(&uart_object[id].uart_tx_buffer);
        }

        HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&uart_object[id].husart, uart_object[id].dma_buffer_tx, size);
        if (res != HAL_OK)
        {
            // TODO: should do something: at least log this error in some statistics?
            return IFL_HAL_UART_ERROR;
        }
        else
        {
            // transmission done: continue...
        }
    }
    else
    {
        // buffer empty: nothing to do
    }

    return IFL_HAL_UART_SUCCESS;
}




// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------


__attribute__((used)) void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0; i < UART_ID_MAX; i++)
    {
        if(huart == &uart_object[i].husart)
        {
            if (ifl_hal_uart_process_tx_data(i) != IFL_HAL_UART_SUCCESS)
            {
                // TODO: this should be logged somehow, maybe in some kind of statistics?
            }
            else
            {

            }
            break;
        }
    }
}

__attribute__((used)) void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for(uint8_t i = 0; i < UART_ID_MAX; i++)
    {
        if(huart == &uart_object[i].husart)
        {
            if (ifl_hal_uart_process_rx_data(IDLE_LINE_NOT_DETECTED, i) != IFL_HAL_UART_SUCCESS)
            {
                // TODO: this should be logged somehow, maybe in some kind of statistics?
            }
            else
            {

            }
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//Ignore input error
}


int hardware_serial_read_data(char * data, uint16_t data_size)
{
    uint16_t count = 0;

    if(!IFL_DEQUE_EMPTY(&uart_object[UART_COM_IHM].uart_rx_buffer))
    {
        __disable_irq();
        while (!IFL_DEQUE_EMPTY(&uart_object[UART_COM_IHM].uart_rx_buffer) && (count < data_size))
        {
            uint8_t* ch = NULL;
            IFL_DEQUE_FRONT(&uart_object[UART_COM_IHM].uart_rx_buffer, ch);
            IFL_DEQUE_POP_FRONT(&uart_object[UART_COM_IHM].uart_rx_buffer);
            data[count] = *ch;
            count++;
        }
        __enable_irq();

    }
    return count;
}
int hardware_serial_write_data(const char * data, uint16_t data_size)
{
    __disable_irq();

    // append to deque buffer
    for (uint16_t i = 0; i < data_size; i++)
    {
        IFL_DEQUE_PUSH_BACK(&uart_object[UART_COM_IHM].uart_tx_buffer, &data[i]);
        if(IFL_DEQUE_FULL(&uart_object[UART_COM_IHM].uart_tx_buffer))
        {
            delay(10);
        }
    }

    if (ifl_hal_uart_process_tx_data(UART_COM_IHM) != IFL_HAL_UART_SUCCESS)
    {
        return IFL_HAL_UART_ERROR;
    }
    else
    {
        // new data has been sent to processing: continue...
    }

    __enable_irq();

    return IFL_HAL_UART_SUCCESS;
}
int hardware_serial_init(const char * serial_port)
{
    (void) serial_port;

    GPIO_InitTypeDef GPIO_InitStruct;

    IFL_DEQUE_INIT(&uart_object[UART_COM_IHM].uart_tx_buffer);
    IFL_DEQUE_INIT(&uart_object[UART_COM_IHM].uart_rx_buffer);

    // initialization of clocks
	UART_COM_IHM_CLK_ENABLE();
	UART_COM_IHM_GPIO_CLK_ENABLE();
    UART_COM_IHM_DMA_ENABLE();

    // initialization of TX pin
    GPIO_InitStruct.Pin = UART_COM_IHM_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = UART_COM_IHM_ALTERNATE;
    HAL_GPIO_Init(UART_COM_IHM_GPIO_BANK, &GPIO_InitStruct);

    uart_object[UART_COM_IHM].husart.Instance = UART_COM_IHM_INSTANCE;

    // initialization of DMA for TX
    uart_object[UART_COM_IHM].hdma_usart_tx.Instance = UART_COM_IHM_DMA_CHANNEL_TX;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.MemInc = DMA_MINC_ENABLE;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.Mode = DMA_NORMAL;
    uart_object[UART_COM_IHM].hdma_usart_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&uart_object[UART_COM_IHM].hdma_usart_tx) != HAL_OK)
    {
        return IFL_HAL_UART_ERROR;
    }
    else
    {
        // DMA init was ok: continue
    }
    __HAL_LINKDMA(&uart_object[UART_COM_IHM].husart, hdmatx, uart_object[UART_COM_IHM].hdma_usart_tx);

    uart_object[UART_COM_IHM].hdma_usart_rx.Instance = UART_COM_IHM_DMA_CHANNEL_RX;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.MemInc = DMA_MINC_ENABLE;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.Mode = DMA_CIRCULAR;
    uart_object[UART_COM_IHM].hdma_usart_rx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&uart_object[UART_COM_IHM].hdma_usart_rx) != HAL_OK)
    {
        return IFL_HAL_UART_ERROR;
    }

    __HAL_LINKDMA(&uart_object[UART_COM_IHM].husart, hdmarx, uart_object[UART_COM_IHM].hdma_usart_rx);

    uart_object[UART_COM_IHM].husart.Init.WordLength = UART_WORDLENGTH_8B;
    uart_object[UART_COM_IHM].husart.Init.BaudRate = 115200;
    uart_object[UART_COM_IHM].husart.Init.Mode = UART_MODE_TX_RX;
    uart_object[UART_COM_IHM].husart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_object[UART_COM_IHM].husart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_object[UART_COM_IHM].husart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_object[UART_COM_IHM].husart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    HAL_StatusTypeDef res = HAL_UART_Init(&uart_object[UART_COM_IHM].husart);
    if (res != HAL_OK)
    {
        return IFL_HAL_UART_ERROR;
    }
    else
    {
        // UART init was ok: continue
    }

    LL_USART_EnableRxTimeout(uart_object[UART_COM_IHM].husart.Instance);
    LL_USART_SetRxTimeout(uart_object[UART_COM_IHM].husart.Instance, DEFAULT_RX_LINE_TIMEOUT); // Wait DEFAULT_RX_LINE_TIMEOUT after last STOP Bit

    LL_USART_EnableIT_RTO(uart_object[UART_COM_IHM].husart.Instance);

    if(HAL_UART_Receive_DMA(&uart_object[UART_COM_IHM].husart,  uart_object[UART_COM_IHM].dma_buffer_rx, UART_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return IFL_HAL_UART_ERROR;
    }

    HAL_NVIC_SetPriority(UART_COM_IHM_DMA_CHANNEL_RX_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(UART_COM_IHM_DMA_CHANNEL_RX_IRQ);

    HAL_NVIC_SetPriority(UART_COM_IHM_DMA_CHANNEL_TX_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(UART_COM_IHM_DMA_CHANNEL_TX_IRQ);


    HAL_NVIC_SetPriority(UART_COM_IHM_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(UART_COM_IHM_IRQ);

    return IFL_HAL_UART_SUCCESS;
}



/**
  * @brief This function handles Service UART global interrupt.
  */
__attribute__((used))void UART_COM_IHM_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_RTO(uart_object[UART_COM_IHM].husart.Instance))
    {
        LL_USART_ClearFlag_RTO(uart_object[UART_COM_IHM].husart.Instance);
        ifl_hal_uart_process_rx_data(IDLE_LINE_DETECTED, UART_COM_IHM);
    }

    HAL_UART_IRQHandler(&uart_object[UART_COM_IHM].husart);
}

/**
  * @brief This function handles DMA stream for Service TX UART
  */
__attribute__((used))void UART_COM_IHM_DMA_CHANNEL_TX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&uart_object[UART_COM_IHM].hdma_usart_tx);
}

/**
  * @brief This function handles DMA stream for Service RX UART
  */
__attribute__((used))void UART_COM_IHM_DMA_CHANNEL_RX_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&uart_object[UART_COM_IHM].hdma_usart_rx);
}
