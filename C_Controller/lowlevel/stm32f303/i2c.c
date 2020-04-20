#include "stm32f3xx_hal.h"
#include "simple_indicators.h"
#include "stm32f3xx_hal_def.h"
#include "stm32f3xx_hal_i2c.h"
#include <stdbool.h>

#define SENSOR_SCL_Pin GPIO_PIN_8
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_9
#define SENSOR_SDA_GPIO_Port GPIOB

static const uint8_t BMP280_ADDR = 0xec;
static const uint8_t NPA_HSCMRNN001PG2A3_ADDR = 0x28 << 1;  // Datasheet is not 8bit adress !
static const uint8_t NPA_HSCMRNN160MG6A3_ADDR = 0x68 << 1;  // Datasheet is not 8bit adress !
static const uint8_t SDP6_ADDR = 0x80;
static const int 	 SDP68SCALE_FACTOR_PA = 60;
static const float 	 PA_TO_cmH2O = 0.0101972;
static volatile uint8_t       _sdp_measurement_buffer[3] = { 0 };
static volatile uint8_t       _npa_measurement_buffer[2]	= { 0 };
static volatile float _current_flow;
static volatile float _current_pressure;

static uint16_t flow_samples_index  = 0;
static float    flow_samples_time_s = 0.f;
static bool     flow_sampling       = false;

typedef enum {
    STOPPED,
    REQ_SDP_MEASUREMENT,
    READ_SDP_MEASUREMENT,
    READ_NPA_MEASUREMENT
} sensor_state_t;

volatile sensor_state_t _sensor_state;

volatile uint16_t _hyperfrish_sdp_time;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;



void My_I2C_ErrorHandler()
{
	while(_current_flow != 0XFFFFFFFF)
	{
	  light_nucleo(1);
	  HAL_Delay(1000);
	  light_nucleo(0);
	  HAL_Delay(1000);
	}
}

static HAL_StatusTypeDef sdp6_reset()
{
    const uint8_t _sdp_reset_req[1]  = { 0xFE };
    return HAL_I2C_Master_Transmit(&hi2c1, SDP6_ADDR, (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000 );
}

static HAL_StatusTypeDef sdp6_set_no_hold_i2c()
{
  const uint8_t _sdp_readAUR_req[1]  = { 0xE5 };
  uint8_t _sdp_writeAUR_req[3]  = { 0xE4, 0x00, 0x00};
  uint8_t       _sdp_AUR_buffer[3] 		= { 0 };
  int ret;
  // read sdp advanced user register
  if((ret = HAL_I2C_Master_Transmit(&hi2c1, SDP6_ADDR, (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 10000))!= HAL_I2C_ERROR_NONE) {
	  return ret;
  }
  HAL_Delay(100);
  if((ret = HAL_I2C_Master_Receive(&hi2c1, SDP6_ADDR, (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 10000 ))!= HAL_I2C_ERROR_NONE) {
	  return ret;
  }
  // print AUR (Advances User Register)
  uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
  uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
  _sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
  _sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
  // Now disable i2c hold master mode
  if((ret = HAL_I2C_Master_Transmit(&hi2c1, SDP6_ADDR, (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 10000 ))!= HAL_I2C_ERROR_NONE) {
	  return ret;
  }
  return ret;
}

static HAL_StatusTypeDef sdp6_receive_measure_it()
{
    int ret = HAL_I2C_Master_Receive_IT(&hi2c1, SDP6_ADDR , (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );
	if( ret != HAL_I2C_ERROR_NONE){
		My_I2C_ErrorHandler();
	}
	return ret;
}

static HAL_StatusTypeDef sdp6_request_measure_it()
{
    const uint8_t _sdp_measurement_req[1] 	= { 0xF1 };
	int ret = HAL_I2C_Master_Transmit_IT(&hi2c1, SDP6_ADDR, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
	if( ret != HAL_I2C_ERROR_NONE){
		My_I2C_ErrorHandler();
	}
	return ret;
}

static HAL_StatusTypeDef npa700_receive_measure_it()
{
	int ret = HAL_I2C_Master_Receive_IT(&hi2c1, NPA_HSCMRNN001PG2A3_ADDR , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
	if( ret != HAL_I2C_ERROR_NONE){
		My_I2C_ErrorHandler();
	}
	return ret;
}

bool sensors_start() {
    // Start sensor state machine.
    // This state machine is managed in the I2C interupt routine.
    _sensor_state= REQ_SDP_MEASUREMENT;
    sdp6_request_measure_it();
	return true;
}

void sensors_stop() {
    _sensor_state= STOPPED;
}

bool sensors_start_sampling_flow()
{
    samples_Q_index = 0;
    samples_Q_t_ms = 0.f;
    sampling_Q = true;
    return sampling_Q;
}

bool sensors_stop_sampling_flow()
{
    sampling_Q = false;
    return !sampling_Q;
}

bool sensors_sample_flow(uint32_t t_ms, float Q_Lps)
{
    if (sampling_Q && samples_Q_index < COUNT_OF(samples_Q_Lps)) {
        samples_Q_Lps[samples_Q_index] = Q_Lps;
        samples_Q_t_ms  += t_ms;
        samples_Q_index ++;
    }
    return true;
}

float sensors_samples_time_s()
{
    return samples_Q_t_ms;
}

void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
	//static	uint16_t hyperfrish_npa;
	//static	uint16_t hyperfrish_sdp;

	switch (_sensor_state) {
	case STOPPED:
		return;
	case READ_NPA_MEASUREMENT:
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
			_sensor_state= STOPPED;
            npa700_receive_measure_it();
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			My_I2C_ErrorHandler();
			break;
		}
		if( (_npa_measurement_buffer[0]>>6)==0) {
			//_hyperfrish_npa_time = (uint16_t)htim3.Instance->CNT - hyperfrish_npa;
			//hyperfrish_npa = (uint16_t)htim3.Instance->CNT;
			uint16_t praw =  (((uint16_t)_npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
			_current_pressure = 70.307 * ((float) ( praw - 1638.)/13107.);
		} else if((_npa_measurement_buffer[0]>>6)==3) {
			// TODO: Manage error status !!
		}
		_sensor_state= READ_SDP_MEASUREMENT;
        sdp6_receive_measure_it();
		break;

	case REQ_SDP_MEASUREMENT:
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
            sdp6_request_measure_it();
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			My_I2C_ErrorHandler();
			break;
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		sdp6_receive_measure_it();
		break;

	case READ_SDP_MEASUREMENT:
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
			_sensor_state= READ_NPA_MEASUREMENT;
            npa700_receive_measure_it();
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			My_I2C_ErrorHandler();
			break;
		}
		if(_sdp_measurement_buffer[0] != 0xFF || _sdp_measurement_buffer[1] != 0xFF || _sdp_measurement_buffer[2] != 0xFF){
            int16_t dp_raw   = (int16_t)((((uint16_t)_sdp_measurement_buffer[0]) << 8) | (uint8_t)_sdp_measurement_buffer[1]);
            _current_flow = ((float)dp_raw) / CALIB_PDIFF_LPS_RATIO;
            _hyperfrish_sdp_time= (uint16_t)htim3.Instance->CNT - hyperfrish_sdp;
            hyperfrish_sdp = (uint16_t)htim3.Instance->CNT;
            _current_volume += (_current_flow/60.f) * ((float)_hyperfrish_sdp_time/1000000);

            // log flow in global array if needed
            sensors_sample_flow(_hyperfrish_sdp_time/1000, _current_flow/60.f);

            _sensor_state= REQ_SDP_MEASUREMENT;
            sdp6_request_measure_it();
		} else {
			_sensor_state= READ_NPA_MEASUREMENT;
			npa700_receive_measure_it();
		}
		break;
	}
}



///**
//  * @brief This function handles DMA1 channel6 global interrupt.
//  */
//void DMA1_Channel4_IRQHandler(void)
//{
//  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
//}
//
///**
//  * @brief This function handles DMA1 channel7 global interrupt.
//  */
//void DMA1_Channel5_IRQHandler(void)
//{
//  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
//}


void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

void HAL_My_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}


static HAL_StatusTypeDef MX_I2C1_Init(void)
{
//   __HAL_RCC_DMA1_CLK_ENABLE();
//  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0xf, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
//  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0xf, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  HAL_StatusTypeDef ret;
  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = SENSOR_SCL_Pin|SENSOR_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(SENSOR_SCL_GPIO_Port, &GPIO_InitStruct);





  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

  ret = HAL_I2C_Init(&hi2c1);
  if(ret != HAL_OK)
  {
	  My_I2C_ErrorHandler();
  }


  /* I2C1 DMA Init */
  /* I2C1_RX Init */
//  hdma_i2c1_rx.Instance = DMA1_Channel4;
//  hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//  hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//  hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//  hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
//  hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
//  if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
//  {
//	  return ifl_hal_i2c_error;
//  }
//
//  __HAL_LINKDMA(&hi2c1,hdmarx,hdma_i2c1_rx);
//
//  /* I2C1_TX Init */
//  hdma_i2c1_tx.Instance = DMA1_Channel5;
//  hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//  hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//  hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//  hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
//  hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
//  if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
//  {
//	return ifl_hal_i2c_error;
//  }
//
//  __HAL_LINKDMA(&hi2c1,hdmatx,hdma_i2c1_tx);

  /* I2C1 interrupt Init */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0xf, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0xf, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);




  while(HAL_OK != HAL_I2C_IsDeviceReady(&hi2c1, SDP6_ADDR, 10, 10))
  {
	  light_nucleo(1);
	  HAL_Delay(100);
	  light_nucleo(0);
	  HAL_Delay(100);
  }

  //Basic test
  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, SDP6_ADDR, 10, 10))
  {
	  light_nucleo(1);
	  HAL_Delay(400);
	  light_nucleo(0);
	  HAL_Delay(400);
	  if(HAL_OK != sdp6_reset() )
	  {
		  My_I2C_ErrorHandler();
	  }
	  HAL_Delay(400);
	  if(HAL_OK != sdp6_set_no_hold_i2c())
	  {
		  My_I2C_ErrorHandler();
	  }
	  HAL_Delay(400);
  }

    // Sensors settle time

  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, NPA_HSCMRNN001PG2A3_ADDR, 10, 10))
  {
	  light_nucleo(1);
	  HAL_Delay(400);
	  light_nucleo(0);
	  HAL_Delay(400);
  } else {
		  My_I2C_ErrorHandler();
  }
  if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1, BMP280_ADDR, 10, 10))
  {
	  light_nucleo(1);
	  HAL_Delay(400);
	  light_nucleo(0);
	  HAL_Delay(400);
  } else {
		  My_I2C_ErrorHandler();
  }

  HAL_Delay(500);

  return HAL_OK;

}



int i2c_init()
{
	return MX_I2C1_Init();
}




uint8_t crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

int compute_checksum(uint8_t data[3])
{
	uint8_t crc_computed =  crc8(data, 2);
	return crc_computed == data[3];
}

float read_Pdiff_Lpm()
{
	return _current_flow;
}

float read_Paw_cmH2O()
{
    return _current_pressure;
}
