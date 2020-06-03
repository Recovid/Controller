#include "recovid_revB.h"
#include "platform.h"

static TIM_HandleTypeDef *g_motor_tim = NULL;

static volatile uint32_t g_nb_steps;
static volatile bool g_is_moving;
static volatile bool g_is_homing;
static volatile bool g_is_home;
static volatile bool g_limit_sw_A;
static volatile bool g_limit_sw_B;
static volatile bool g_active;
static volatile motor_step_callback_t g_motor_step_callback;

static void period_elapsed_callback(TIM_HandleTypeDef *tim);
static void do_motor_stop();
static void check_home();



bool motor_init(uint32_t home_step_us)
{
    if (g_motor_tim == NULL)
    {
        g_motor_tim = &motor_tim;
        // register IT callbacks
        HAL_TIM_RegisterCallback(g_motor_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, period_elapsed_callback);

        // // Enable the PWM channel
        TIM_CCxChannelCmd(g_motor_tim->Instance, MOTOR_TIM_CHANNEL, TIM_CCx_ENABLE);

        g_motor_step_callback= NULL;
        g_nb_steps= 0;

        g_active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);

        g_limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
        g_limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
        g_is_home= g_limit_sw_A && g_limit_sw_B;
        g_is_homing= false;
        g_is_moving=false;
        motor_enable(true);

        if (g_is_home)
        {
            taskENTER_CRITICAL();
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
            g_motor_tim->Init.Period = home_step_us;
            HAL_TIM_Base_Init(g_motor_tim);
            g_is_moving = true;
            g_is_homing = false;
            HAL_TIM_Base_Start(g_motor_tim);
            taskEXIT_CRITICAL();
            while (g_is_home) 
            {
                wait_ms(2);
            }
            wait_ms(300);
            motor_stop();
            wait_ms(500);
        }
        taskENTER_CRITICAL();
        g_is_homing = true;
        g_is_moving = true;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        g_motor_tim->Init.Period = home_step_us;
        HAL_TIM_Base_Init(g_motor_tim);
        HAL_TIM_Base_Start(g_motor_tim);
        taskEXIT_CRITICAL();
        while (!g_is_home)
        {
            wait_ms(2);
        }
    }
    return true;
}


bool motor_release(uint32_t step_us)
{
    motor_stop();
    g_nb_steps = 0;
    if (!g_is_home)
    {
        taskENTER_CRITICAL();
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        g_is_moving = true;
        g_is_homing = true;
        g_motor_tim->Init.Period = step_us;
        HAL_TIM_Base_Init(g_motor_tim);
        HAL_TIM_Base_Start(g_motor_tim);
        taskEXIT_CRITICAL();
    }
    return true;
}

bool motor_press(uint32_t *steps_profile_us, unsigned int nb_steps)
{
    motor_stop();
    g_nb_steps = nb_steps;
    if (g_nb_steps > 0)
    {
        taskENTER_CRITICAL();
        motor_enable(true);
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
        g_is_moving = true;
        g_motor_step_callback= NULL;
        g_motor_tim->Init.Period = steps_profile_us[0];
        HAL_TIM_Base_Init(g_motor_tim);
        HAL_DMA_Init(g_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
        HAL_TIM_Base_Start(g_motor_tim);
        HAL_TIM_DMABurst_MultiWriteStart(g_motor_tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE, &steps_profile_us[1], TIM_DMABURSTLENGTH_1TRANSFER, g_nb_steps - 1);
        taskEXIT_CRITICAL();
    }

    return true;
}

uint32_t motor_press_get_current_step() 
{
    if(g_nb_steps>0)
    {
        return g_nb_steps - __HAL_DMA_GET_COUNTER(g_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
    }
    return 0;
}

bool motor_move(motor_dir_t dir, uint32_t step_us, motor_step_callback_t callback)
{
    motor_stop();

    taskENTER_CRITICAL();
    motor_enable(true);
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, dir == MOTOR_PRESS ? MOTOR_PRESS_DIR : MOTOR_RELEASE_DIR);
    g_nb_steps = 0;
    g_is_moving = true;
    g_motor_step_callback= callback;
    g_motor_tim->Init.Period = step_us;
    HAL_TIM_Base_Init(g_motor_tim);
    HAL_TIM_Base_Start_IT(g_motor_tim);
    taskEXIT_CRITICAL();

    return true;
}

bool motor_stop()
{
    taskENTER_CRITICAL();
    if (g_is_moving)
    {
        do_motor_stop();
    }
    taskEXIT_CRITICAL();
    return true;
}

void motor_enable(bool ena)
{
    HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, ena ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


bool motor_is_moving()
{
    return g_is_moving;
}

bool motor_is_home()
{
    return g_is_home;
}

bool motor_is_ok()
{
    return g_motor_tim != NULL;
}


void motor_limit_sw_A_irq()
{
    __disable_irq();
    g_limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    check_home();
    __enable_irq();
}

void motor_limit_sw_B_irq()
{
    __disable_irq();
    g_limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();
    __enable_irq();
}

void motor_active_irq()
{
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    if (time - last_time > 10)
    {
        g_active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);
    }
    last_time = time;
}


static void period_elapsed_callback(TIM_HandleTypeDef *tim)
{
    if(g_motor_step_callback==NULL) 
    {
        do_motor_stop();
    }
    else
    {
        uint32_t next_step_us = g_motor_step_callback(tim->Instance->ARR);
        if(next_step_us==0)
        {
            do_motor_stop();
        }
        else
        {
            tim->Instance->ARR= next_step_us;
        }
        
    }
}


static void check_home()
{
    if (g_limit_sw_A && g_limit_sw_B)
    {
        g_is_home = true;
        if (g_is_homing)
        {
            do_motor_stop();
            g_is_homing = false;
        }
    }
    else
    {
        g_is_home = false;
    }
}

static void do_motor_stop() {
    if (g_is_moving)
    {
        HAL_TIM_DMABurst_WriteStop(g_motor_tim, TIM_DMA_ID_UPDATE);
        HAL_DMA_DeInit(g_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
        // Stop TIM
        g_motor_tim->Instance->CR1 &= ~(TIM_CR1_CEN);
        // Disable TIM IT
        __HAL_TIM_DISABLE_IT(g_motor_tim, TIM_IT_UPDATE);
        g_is_moving = false;
        g_motor_step_callback= NULL;
    }
}

