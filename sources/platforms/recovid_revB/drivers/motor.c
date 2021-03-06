#include "recovid_revB.h"
#include "platform.h"

static TIM_HandleTypeDef *_motor_tim = NULL;


static volatile bool _moving;
static volatile bool _homing;
static volatile bool _home;
static volatile bool _limit_sw_A;
static volatile bool _limit_sw_B;
static volatile bool _active;

static void period_elapsed_callback(TIM_HandleTypeDef *tim);
static void do_motor_stop();
static void check_home();



bool init_motor(uint32_t home_step_us)
{
    if (_motor_tim == NULL)
    {
        _motor_tim = &motor_tim;
        // register IT callbacks
        HAL_TIM_RegisterCallback(_motor_tim, HAL_TIM_PERIOD_ELAPSED_CB_ID, period_elapsed_callback);

        _active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);

        _limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
        _limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
        check_home();
        HAL_TIM_Base_Start(_motor_tim);
        motor_enable(true);

        if (_home)
        {
            taskENTER_CRITICAL();
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
            _motor_tim->Init.Period = home_step_us;
            HAL_TIM_Base_Init(_motor_tim);
            _motor_tim->Instance->CNT=0;
            _moving = true;
            _homing = false;
            HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
            taskEXIT_CRITICAL();
            while (_home) 
            {
                wait_ms(2);
            }
            wait_ms(200);
            motor_stop();
            wait_ms(500);
        }
        taskENTER_CRITICAL();
        _homing = true;
        _moving = true;
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        _motor_tim->Init.Period = home_step_us;
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT=0;
        HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
        taskEXIT_CRITICAL();
        while (!_home)
        {
            wait_ms(2);
        }
    }
    return true;
}


bool motor_release(uint32_t step_us)
{
    motor_stop();
    if (!_home)
    {
        taskENTER_CRITICAL();
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_RELEASE_DIR);
        _moving = true;
        _homing = true;
        _motor_tim->Init.Period = step_us;
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT=0;
        HAL_TIM_PWM_Start(_motor_tim, MOTOR_TIM_CHANNEL);
        taskEXIT_CRITICAL();
    }
    return true;
}

bool motor_press(uint32_t *steps_profile_us, unsigned int nb_steps)
{
    motor_stop();
    if (nb_steps > 0)
    {
        taskENTER_CRITICAL();
        motor_enable(true);
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, MOTOR_PRESS_DIR);
        _moving = true;
        _motor_tim->Init.Period = steps_profile_us[0];
        HAL_TIM_Base_Init(_motor_tim);
        _motor_tim->Instance->CNT=0;
        HAL_DMA_Init(_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
        HAL_TIM_PWM_Start_IT(_motor_tim, MOTOR_TIM_CHANNEL);
        HAL_TIM_DMABurst_MultiWriteStart(_motor_tim, TIM_DMABASE_ARR, TIM_DMA_UPDATE, &steps_profile_us[1], TIM_DMABURSTLENGTH_1TRANSFER, nb_steps - 1);
        taskEXIT_CRITICAL();
    }
    return true;
}

bool motor_stop()
{
    taskENTER_CRITICAL();
    if (_moving)
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


bool is_motor_moving()
{
    return _moving;
}

bool is_motor_home()
{
    return _home;
}

bool is_motor_ok()
{
    return _motor_tim != NULL;
}


void motor_limit_sw_A_irq()
{
    __disable_irq();
    _limit_sw_A = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin);
    check_home();
    __enable_irq();
}

void motor_limit_sw_B_irq()
{
    __disable_irq();
    _limit_sw_B = !HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
    check_home();
    __enable_irq();
}

void motor_active_irq()
{
    static uint32_t last_time = 0;
    uint32_t time = HAL_GetTick();
    if (time - last_time > 10)
    {
        _active = HAL_GPIO_ReadPin(MOTOR_ACTIVE_GPIO_Port, MOTOR_ACTIVE_Pin);
    }
    last_time = time;
}


static void period_elapsed_callback(TIM_HandleTypeDef *tim)
{
    __disable_irq();
    do_motor_stop();
    __enable_irq();
}


static void check_home()
{
    if (_limit_sw_A && _limit_sw_B)
    {
        _home = true;
        if (_homing)
        {
            do_motor_stop();
            _homing = false;
        }
    }
    else
    {
        _home = false;
    }
}

static void do_motor_stop() {
    if (_moving)
    {
        HAL_TIM_DMABurst_WriteStop(_motor_tim, TIM_DMA_ID_UPDATE);
        HAL_DMA_DeInit(_motor_tim->hdma[TIM_DMA_ID_UPDATE]);
        HAL_TIM_PWM_Stop(_motor_tim, MOTOR_TIM_CHANNEL);
        _moving = false;
    }
}

