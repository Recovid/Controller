#include "recovid_revB.h"
#include "platform.h"
#include <math.h>

static TIM_HandleTypeDef *_motor_tim = NULL;

static void step_callback(TIM_HandleTypeDef *tim);

static volatile uint32_t _remaining_steps;
static volatile bool _moving;
static volatile bool _homing;
static volatile bool _home;

typedef enum
{
    ZERO,
    ONE,
    HIGHZ
} step_mode_t;

static void set_stepping(step_mode_t m0, step_mode_t m1);
static void set_direction(GPIO_PinState dir);
static void set_enable(bool ena);

bool init_motor_pep()
{
    if (_motor_tim == NULL)
    {
        _motor_tim = &pep_tim;
        // register IT callbacks
        _motor_tim->PeriodElapsedCallback = step_callback;

        // Enable the PWM channel
        TIM_CCxChannelCmd(_motor_tim->Instance, PEP_TIM_CHANNEL, TIM_CCx_ENABLE);

        // microstepping 8
        set_stepping(ZERO, ONE);
        // Set indexer mode
        HAL_GPIO_WritePin(PEP_CONFIG_GPIO_Port, PEP_CONFIG_Pin, GPIO_PIN_SET);
        // Activate
        HAL_GPIO_WritePin(PEP_nSLEEP_GPIO_Port, PEP_nSLEEP_Pin, GPIO_PIN_SET);

        set_enable(false);

        _home = !HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin);
    }
    return true;
}

bool is_motor_pep_ok()
{
    return _motor_tim != NULL;
}

bool motor_pep_move(int relative_mm)
{
    if (_motor_tim == NULL)
        return false;
    if (relative_mm != 0)
    {
        __disable_irq();
        set_direction(relative_mm < 0 ? PEP_DIR_DEC : PEP_DIR_INC);
        set_enable(true);
        _remaining_steps = (uint32_t)fabs(relative_mm) * PEP_STEPS_PER_MM;
        _motor_tim->Init.Period = (uint16_t)(1000000.0 / (PEP_MAX_SPEED * PEP_STEPS_PER_MM));
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT = 0;
        _moving = true;
        _homing=false;
        __enable_irq();
        HAL_TIM_Base_Start_IT(_motor_tim);
    }
    return true;
}

bool motor_pep_home()
{
    if (_motor_tim == NULL) 
    {
        return false;
    }

    __disable_irq();
    set_direction(PEP_DIR_DEC);
    set_enable(true);
    _motor_tim->Init.Period = (uint16_t)(1000000 / (PEP_MAX_SPEED * PEP_STEPS_PER_MM));
    HAL_TIM_Base_Init(_motor_tim);
    _motor_tim->Instance->CNT = 0;
    _moving = true;
    _homing = true;
    __enable_irq();
    HAL_TIM_Base_Start_IT(_motor_tim);

    while (!_home) 
    {
        wait_ms(5);
    }
    return true;
}

bool motor_pep_stop()
{
    if (_motor_tim == NULL) 
    {
        return false;
    }
    __disable_irq();
    HAL_TIM_Base_Stop(_motor_tim);
    _homing = false;
    _remaining_steps = 0;
    _moving = false;
    set_enable(false);
    __enable_irq();
    return true;
}

bool is_motor_pep_moving()
{
    return _moving;
}

//!
bool is_motor_pep_home()
{
    return _home;
}

void pep_home_irq()
{
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    if (time - last_time > 20)
    {
        _home = !HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin);
        if (_home && _homing)
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
    if (_homing)
        return;
    if (_remaining_steps == 0)
    {
        motor_pep_stop();
    }
    --_remaining_steps;
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
