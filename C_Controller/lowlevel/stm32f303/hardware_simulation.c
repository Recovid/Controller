#include "lowlevel/include/lowlevel.h"

#include <time.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "hardware_serial.h"
#include "stm32f3xx_hal.h"
#include "configuration.h"
#include "lowlevel/include/lowlevel.h"
#include "ihm_communication.h"
#include "stm32f3xx_hal_rcc_ex.h"
// ------------------------------------------------------------------------------------------------

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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM16) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

// ------------------------------------------------------------------------------------------------


bool init_ihm(ihm_mode_t ihm_mode, const char* pathInputFile, const char* pathOutputFile)
{
    (void) ihm_mode; (void) pathInputFile; (void) pathOutputFile;
    return hardware_serial_init(NULL);
}

bool send_ihm(const char* frame)
{

    return hardware_serial_write_data(frame, strlen(frame));
}

int recv_ihm()
{
    char blocking_read = 0;

    int t_s = hardware_serial_read_data(&blocking_read, sizeof(char));;

    if (t_s > 0) {

        return blocking_read;
    }
    return EOF;
}

// ------------------------------------------------------------------------------------------------
//! HW actuators

static int motor_pos = 0;
static int motor_dir = 0;
static long motor_release_ms = -1;

bool motor_press(float VM_Lpm)
{
    motor_release_ms = -1;
    motor_dir = 1; // TODO simulate Vmax_Lpm limiting by determining the approriate speed/steps
    motor_pos = MIN(MOTOR_MAX, motor_pos+motor_dir); // TODO simulate lost steps in range
    //if (motor_pos/0xF) {
    //    DEBUG_PRINTF("motor %X\n", motor_pos);
    //}
    return true; // TODO simulate driver failure
}

bool motor_stop()
{
    motor_release_ms = -1;
    motor_dir = 0;
    return true; // TODO simulate driver failure
}

bool motor_release(uint32_t before_t_ms)
{
    motor_release_ms = get_time_ms();
    motor_dir = -1;
    motor_pos = MAX(0, motor_pos+motor_dir); // TODO simulate lost steps in range
    //if (motor_pos/0xF) {
    //    DEBUG_PRINTF("motor %X\n", motor_pos);
    //}
    return true; // TODO simulate driver failure
}

bool motor_pep_move(float relative_move_cmH2O)
{
    return false; // TODO
}


static enum Valve { Inhale, Exhale } valve_state = Exhale;
static long valve_exhale_ms = -1;

bool valve_exhale()
{
    if (valve_state == Exhale) return true;

    valve_state = Exhale;
    valve_exhale_ms = get_time_ms();
    return true;
}

bool valve_inhale()
{
    valve_state = Inhale;
    valve_exhale_ms = -1;
    return true;
}

//! Usable BAVU volume based on motor position
//! \remark BAVU deformation/elasticity is simulated with a cos to easily derive Q
float BAVU_V_mL()
{
    return cosf(M_PI_2*(((float)motor_pos) /MOTOR_MAX)) * BAVU_V_ML_MAX; // TODO simulate BAVU perforation
}

//! Usable BAVU flow based on motor position and direction
//! \remark a valve normally ensures that Q is always positive
float BAVU_Q_Lpm()
{
    float piover2 = M_PI_2;
    float ratio_motor = ((float)(motor_pos)/MOTOR_MAX);
    float sinus = sinf(piover2 * ratio_motor);
    const float Q_Lpm = sinus * BAVU_Q_LPM_MAX; // TODO simulate BAVU perforation
    return Q_Lpm * (motor_dir > 0 ? 1. : BAVU_VALVE_RATIO);
}

// ------------------------------------------------------------------------------------------------
//! HW sensors simulation





float read_Patmo_mbar()
{
    return 1013. + sinf(2*M_PI*get_time_ms()/1000/60) * PATMO_VARIATION_MBAR; // TODO test failure
}




int read_Battery_level()
{
    return 2; // TODO simulate lower battery levels
}


bool init_Paw () {
    return false;
}
bool init_Patmo () {
    return false;
}
bool init_Pdiff () {
    return false;
}
bool init_valve () {
    return false;
}
bool init_motor () {
    return false;
}
bool init_motor_pep () {
    return false;
}
