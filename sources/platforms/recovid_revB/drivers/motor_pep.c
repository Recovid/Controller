#include "recovid_revB.h"
#include "platform.h"
#include <math.h>

static TIM_HandleTypeDef *g_motor_tim = NULL;

static void step_callback(TIM_HandleTypeDef *tim);

static volatile int32_t _absolute_steps;
static volatile int16_t _step_inc;
static volatile int32_t _remaining_steps;
static volatile bool g_is_moving;
static volatile bool g_is_homing;
static volatile bool g_is_home;

typedef enum
{
    ZERO,
    ONE,
    HIGHZ
} step_mode_t;

static void set_stepping(step_mode_t m0, step_mode_t m1);
static void set_direction(GPIO_PinState dir);
static void set_enable(bool ena);

bool motor_pep_init()
{
    if (g_motor_tim == NULL)
    {
        g_motor_tim = &pep_tim;
        // register IT callbacks
        HAL_TIM_RegisterCallback(g_motor_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, step_callback);

        // // Enable the PWM channel
        TIM_CCxChannelCmd(g_motor_tim->Instance, PEP_TIM_CHANNEL, TIM_CCx_ENABLE);

        // microstepping 8
        set_stepping(ZERO, ONE);
        // Set indexer mode
        HAL_GPIO_WritePin(PEP_CONFIG_GPIO_Port, PEP_CONFIG_Pin, GPIO_PIN_SET);
        // Activate
        HAL_GPIO_WritePin(PEP_nSLEEP_GPIO_Port, PEP_nSLEEP_Pin, GPIO_PIN_SET);

        set_enable(false);

        g_is_home = !HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin);
        motor_pep_home();
    }
    return true;
}

bool motor_pep_is_ok()
{
    return g_motor_tim != NULL;
}

bool motor_pep_move(int relative_mmH2O)
{
    if (g_motor_tim == NULL)
        return false;
    if (relative_mmH2O != 0)
    {
        int32_t nb_steps= (int32_t)(relative_mmH2O*MOTOR_PEP_mmH2O_TO_mm_FACTOR) * MOTOR_PEP_STEPS_PER_mm;
        if(_absolute_steps+nb_steps <0 ) 
        {
            nb_steps= -_absolute_steps;
        }
        if(_absolute_steps+nb_steps > MOTOR_PEP_MAX_STEPS)  
        {
            nb_steps= MOTOR_PEP_MAX_STEPS - _absolute_steps;
        }
        
        if(nb_steps==0) return false;
        _step_inc= nb_steps<0? -1 : 1;

        __disable_irq();
        set_direction(nb_steps < 0 ? PEP_DIR_DEC : PEP_DIR_INC);
        set_enable(true);
        _remaining_steps = (uint32_t)fabs(nb_steps);
        g_motor_tim->Init.Period = (uint16_t)(1000000.0 / (MOTOR_PEP_MAX_SPEED * MOTOR_PEP_STEPS_PER_mm));
        HAL_TIM_Base_Init(g_motor_tim);
        g_is_moving = true;
        g_is_homing=false;
        __enable_irq();
        HAL_TIM_Base_Start_IT(g_motor_tim);
    }
    return true;
}

bool motor_pep_home()
{
    if (g_motor_tim == NULL) 
    {
        return false;
    }
    if(!g_is_home) {
        __disable_irq();
        set_direction(PEP_DIR_DEC);
        set_enable(true);
        g_motor_tim->Init.Period = (uint16_t)(1000000 / (MOTOR_PEP_HOME_SPEED * MOTOR_PEP_STEPS_PER_mm));
        HAL_TIM_Base_Init(g_motor_tim);
        g_is_moving = true;
        g_is_homing = true;
        __enable_irq();
        HAL_TIM_Base_Start(g_motor_tim);

        while (!g_is_home) 
        {
            wait_ms(5);
        }
        _absolute_steps=0;
    }
    return true;
}

bool motor_pep_stop()
{
    if (g_motor_tim == NULL) 
    {
        return false;
    }
    __disable_irq();
    // Stop TIM
    g_motor_tim->Instance->CR1 &= ~(TIM_CR1_CEN);
    g_is_homing = false;
    _remaining_steps = 0;
    g_is_moving = false;
    set_enable(false);
    __enable_irq();
    return true;
}

bool motor_pep_is_moving()
{
    return g_is_moving;
}

//!
bool motor_pep_is_home()
{
    return g_is_home;
}

void pep_home_irq()
{
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    if (time - last_time > 20)
    {
        g_is_home = !HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin);
        if (g_is_home && g_is_homing)
        {
            motor_pep_stop();
        }
    }
    last_time = time;
}

void pep_nfault_irq()
{
    // TODO
}

static void step_callback(TIM_HandleTypeDef *tim)
{
    if (g_is_homing)
        return;
    if (_remaining_steps > 0)
    {
        --_remaining_steps;
        _absolute_steps+= _step_inc;
        
    }
    if(_absolute_steps==0) {
        _absolute_steps=0;
        _remaining_steps=0;
    }
    if(_absolute_steps>=MOTOR_PEP_MAX_STEPS) {
        _absolute_steps=MOTOR_PEP_MAX_STEPS;
        _remaining_steps=0;
    }

    if(_remaining_steps<=0) {
        motor_pep_stop();
    }

}

static void set_stepping(step_mode_t m0, step_mode_t m1)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = PEP_MODE0_Pin;
    GPIO_InitStruct.Mode = m0 == HIGHZ ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PEP_MODE1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PEP_MODE0_GPIO_Port, PEP_MODE0_Pin, m0 == HIGHZ ? 1 : m0);

    GPIO_InitStruct.Pin = PEP_MODE1_Pin;
    GPIO_InitStruct.Mode = m1 == HIGHZ ? GPIO_MODE_OUTPUT_OD : GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PEP_MODE1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(PEP_MODE1_GPIO_Port, PEP_MODE1_Pin, m1 == HIGHZ ? 1 : m1);
}

static void set_enable(bool ena)
{
    HAL_GPIO_WritePin(PEP_nENBL_GPIO_Port, PEP_nENBL_Pin, ena ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void set_direction(GPIO_PinState dir)
{
    HAL_GPIO_WritePin(PEP_DIR_GPIO_Port, PEP_DIR_Pin, dir);
}
