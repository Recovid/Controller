#include "stm32f3xx_hal.h"
#include "leds.h"
#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal_i2c.h"

I2C_HandleTypeDef hi2c1;

static const uint8_t BMP280_ADDR = 0xec;
static const uint8_t HSCMRNN001PG2A3_ADDR = 0x28 << 1;  // Datasheet is not 8bit adress !
static const uint8_t SDP6_ADDR = 0x80;
static const int 	 SDP68SCALE_FACTOR_PA = 60;
static const float 	 PA_TO_cmH2O = 0.0101972;

typedef enum ifl_hal_i2c_result_t
{
    ifl_hal_i2c_error = -1,
    ifl_hal_i2c_success = 0,
}ifl_hal_i2c_result_t;

static ifl_hal_i2c_result_t MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  return ifl_hal_i2c_error;
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  return ifl_hal_i2c_error;
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
	  return ifl_hal_i2c_error;
  }

  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, SDP6_ADDR, 10, 10))
  {
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
  }
  HAL_Delay(2000);

  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, HSCMRNN001PG2A3_ADDR, 10, 10))
  {
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
  }
  HAL_Delay(2000);

  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR, 10, 10))
  {
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
	  led_onnucleo_set(1);
	  HAL_Delay(200);
	  led_onnucleo_set(0);
	  HAL_Delay(200);
  }
  HAL_Delay(2000);


  return ifl_hal_i2c_success;

}



int i2c_read_data(uint16_t address, uint8_t* data, uint16_t data_size)
{
	return HAL_I2C_Master_Receive(&hi2c1, address, data, data_size, HAL_MAX_DELAY);
}

int i2c_write_data(uint16_t address, const uint8_t* data, uint16_t data_size)
{
	return HAL_I2C_Master_Transmit(&hi2c1, address, data, data_size, HAL_MAX_DELAY);
}

int i2c_init()
{
	return MX_I2C1_Init();
}

int compute_checksum(uint8_t data[3])
{
	return 0;
}
float read_Pdiff_Lpm()
{
	uint8_t data[3];
	i2c_read_data(SDP6_ADDR, data, 3);
	compute_checksum(data);
	int data_int = data[0] << 8 | data[1];
	float data_f =  ( (float) data_int / SDP68SCALE_FACTOR_PA) * PA_TO_cmH2O; //0.0101972
	return data_f;
}
