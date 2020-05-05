// @Author: Inventhys
// @Date:   2020-04-08 17:14:01
// @Last Modified by:   Inventhys
// @Last Modified time: 2020-04-08 17:14:01
//
// --------------------------------------------------------------------------------------------------------------------
// ----- includes
// --------------------------------------------------------------------------------------------------------------------

#include "recovid_revB.h"
#include "platform.h"

#include "ifl_deque.h"
#include "stm32f3xx_ll_usart.h"

#include <stdio.h>
#include <string.h>


// --------------------------------------------------------------------------------------------------------------------
// ----- local defines
// --------------------------------------------------------------------------------------------------------------------

#define HMI_TX_BUFFER_SIZE                  (4096)
#define HMI_RX_BUFFER_SIZE                  (1024)

#define HMI_TX_DMA_BUFFER_SIZE              (512)
#define HMI_RX_DMA_BUFFER_SIZE              (512)

#define HMI_TX_SYNC_TIMEOUT_MS              (2000)
#define HMI_RX_LINE_TIMEOUT                 (30)


// --------------------------------------------------------------------------------------------------------------------
// ----- local function macros
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local typedefs, structures, unions and enums
// --------------------------------------------------------------------------------------------------------------------
typedef enum idle_line_state_t
{
    INVALIDE_IDLE_LINE_STATE = 0,
    IDLE_LINE_DETECTED,
    IDLE_LINE_NOT_DETECTED,
}idle_line_state_t;


static    IFL_DEQUE_DECLARE(_uart_tx_buffer, uint8_t, HMI_TX_BUFFER_SIZE);
static    IFL_DEQUE_DECLARE(_uart_rx_buffer, uint8_t, HMI_RX_BUFFER_SIZE);
static    uint8_t _dma_buffer_tx[HMI_TX_DMA_BUFFER_SIZE];
static    uint8_t _dma_buffer_rx[HMI_RX_DMA_BUFFER_SIZE];

// --------------------------------------------------------------------------------------------------------------------
// ----- local variables
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// ----- local function prototypes
// --------------------------------------------------------------------------------------------------------------------

static bool hal_uart_process_tx_data();
static bool hal_uart_process_rx_data(idle_line_state_t idle_line_state);

static void uart_ErrorCallback();
static void uart_RxCpltCallback();
static void uart_TxCpltCallback();




// --------------------------------------------------------------------------------------------------------------------
// ----- public functions
// --------------------------------------------------------------------------------------------------------------------


bool init_uart()
{
    IFL_DEQUE_INIT(&_uart_tx_buffer);
    IFL_DEQUE_INIT(&_uart_rx_buffer);

    // Register IT callbacks
    HAL_UART_RegisterCallback(&hmi_uart, HAL_UART_RX_COMPLETE_CB_ID, uart_RxCpltCallback);
    HAL_UART_RegisterCallback(&hmi_uart, HAL_UART_TX_COMPLETE_CB_ID, uart_TxCpltCallback);
    HAL_UART_RegisterCallback(&hmi_uart, HAL_UART_ERROR_CB_ID, uart_ErrorCallback);

    // Enable RX timout
    LL_USART_EnableRxTimeout(hmi_uart.Instance);
    LL_USART_SetRxTimeout(hmi_uart.Instance, HMI_RX_LINE_TIMEOUT); // Wait DEFAULT_RX_LINE_TIMEOUT after last STOP Bit
    LL_USART_EnableIT_RTO(hmi_uart.Instance);

    // Start asynchronous DMA rx process
    if(HAL_UART_Receive_DMA(&hmi_uart,  _dma_buffer_rx, HMI_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return false;
    }

    // Enable IT
    HAL_NVIC_EnableIRQ(HMI_DMA_CHANNEL_RX_IRQn);
    HAL_NVIC_EnableIRQ(HMI_DMA_CHANNEL_TX_IRQn);
    HAL_NVIC_EnableIRQ(HMI_UART_IRQn);

    return true;
}

int uart_read_data(char * data, uint16_t data_size)
{
    uint16_t count = 0;

    if(!IFL_DEQUE_EMPTY(&_uart_rx_buffer))
    {
        __disable_irq();
        while (!IFL_DEQUE_EMPTY(&_uart_rx_buffer) && (count < data_size))
        {
            uint8_t* ch = NULL;
            IFL_DEQUE_FRONT(&_uart_rx_buffer, ch);
            IFL_DEQUE_POP_FRONT(&_uart_rx_buffer);
            data[count] = *ch;
            count++;
        }
        __enable_irq();

    }
    return count;
}

bool uart_write_data(const char * data, uint16_t data_size)
{
    __disable_irq();

    // append to deque buffer
    for (uint16_t i = 0; i < data_size; i++)
    {
        IFL_DEQUE_PUSH_BACK(&_uart_tx_buffer, &data[i]);
        if(IFL_DEQUE_FULL(&_uart_tx_buffer))
        {
            // TODO Check implem
            __enable_irq();
            return false;
        }
    }

    if (hal_uart_process_tx_data() != true)
    {
        __enable_irq();
        return false;
    }
    else
    {
        // new data has been sent to processing: continue...
    }

    __enable_irq();

    return true;
}


bool uart_send(const char* frame)
{

    return uart_write_data(frame, strlen(frame));
}

int uart_recv()
{
    char blocking_read = 0;

    int t_s = uart_read_data(&blocking_read, sizeof(char));;

    if (t_s > 0) {

        return blocking_read;
    }
    return EOF;
}


// --------------------------------------------------------------------------------------------------------------------
// ----- Private functions
// --------------------------------------------------------------------------------------------------------------------


static bool hal_uart_process_rx_data(idle_line_state_t idle_line_state)
{
    static uint16_t last_size = { HMI_RX_DMA_BUFFER_SIZE };
    uint16_t current_size = (uint16_t) __HAL_DMA_GET_COUNTER(&hmi_dma_rx);
    uint16_t length_to_read = 0;
    uint16_t start_read = 0;

    __disable_irq();

    if((idle_line_state == IDLE_LINE_DETECTED) && (current_size == HMI_RX_DMA_BUFFER_SIZE))
    {
        __enable_irq();
        return true;
    }

    start_read = (last_size < HMI_RX_DMA_BUFFER_SIZE) ? (uint16_t)(HMI_RX_DMA_BUFFER_SIZE - last_size) : 0;

    if(idle_line_state == IDLE_LINE_DETECTED)
    {
        length_to_read = (last_size < HMI_RX_DMA_BUFFER_SIZE) ? (uint16_t)(last_size - current_size) : (uint16_t)(HMI_RX_DMA_BUFFER_SIZE - current_size);
        last_size = current_size;
    }
    else
    {
        length_to_read = (uint16_t)(HMI_RX_DMA_BUFFER_SIZE - start_read);
        last_size = HMI_RX_DMA_BUFFER_SIZE;
    }


    for(uint16_t i = start_read; i < (length_to_read + start_read); i++)
    {
        IFL_DEQUE_PUSH_BACK(&_uart_rx_buffer, &_dma_buffer_rx[i % HMI_RX_DMA_BUFFER_SIZE]);
    }

    if(idle_line_state == IDLE_LINE_DETECTED || length_to_read == HMI_RX_DMA_BUFFER_SIZE)
    {

    }
    __enable_irq();
    return true;
}

static bool hal_uart_process_tx_data()
{
    if (!IFL_DEQUE_EMPTY(&_uart_tx_buffer) &&
        (HAL_UART_GetState(&hmi_uart) != HAL_UART_STATE_BUSY_TX) &&
        (HAL_UART_GetState(&hmi_uart) != HAL_UART_STATE_BUSY_TX_RX))
    {
        // nothing in buffer, and UART is ready: start a new DMA request
        uint16_t size = IFL_DEQUE_SIZE(&_uart_tx_buffer);
        if (size > HMI_TX_DMA_BUFFER_SIZE)
        {
            size = HMI_TX_DMA_BUFFER_SIZE;
        }
        else
        {
            // don't change size, as it is less than the buffer's size
        }

        for (uint16_t i = 0; i < size; i++)
        {
            uint8_t* byte = NULL;
            IFL_DEQUE_FRONT(&_uart_tx_buffer, byte);
            if (byte != NULL)
            {
                _dma_buffer_tx[i] = *byte;
            }
            else
            {
                // should not happen, TODO: this should be logged somehow, maybe in some kind of statistics?
            }
            IFL_DEQUE_POP_FRONT(&_uart_tx_buffer);
        }

        HAL_StatusTypeDef res = HAL_UART_Transmit_DMA(&hmi_uart, _dma_buffer_tx, size);
        if (res != HAL_OK)
        {
            // TODO: should do something: at least log this error in some statistics?
            return false;
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

    return true;
}


static void uart_TxCpltCallback()
{
    if (hal_uart_process_tx_data() != true)
    {
        // TODO: this should be logged somehow, maybe in some kind of statistics?
    }
    else
    {

    }
}

static void uart_RxCpltCallback()
{
    hal_uart_process_rx_data(IDLE_LINE_NOT_DETECTED);
}

static void uart_ErrorCallback()
{
}


// Implement UART4 IRQ receive timeout callback
__attribute((used)) void uart_RxTimeoutCallback()
{
    hal_uart_process_rx_data(IDLE_LINE_DETECTED);
}
