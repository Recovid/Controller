/*
 * sensors.c
 *
 *  Created on: Apr 13, 2020
 *      Author: rix
 */

#include "sensors.h"
#include "utils.h"



typedef enum {
	STOPPED,
	REQ_SDP_MEASUREMENT,
	READ_SDP_MEASUREMENT,
	READ_NPA_MEASUREMENT
} sensors_state_t;



static I2C_HandleTypeDef* _i2c;
static TIM_HandleTypeDef* _timer1kHz;			// 1kHz sensors timer


static const uint8_t _sdp_reset_req[1]  = { 0xFE };
static const uint8_t _sdp_readAUR_req[1]  = { 0xE5 };
static uint8_t _sdp_writeAUR_req[3]  = { 0xE4, 0x00, 0x00};

static const uint8_t _sdp_measurement_req[1] 	= { 0xF1 };
static uint8_t       _sdp_measurement_buffer[3] = { 0 };
static uint8_t       _sdp_AUR_buffer[3] 		= { 0 };

static uint8_t       _npa_measurement_buffer[2]	= { 0 };

static volatile float _current_flow;
static volatile float _current_pressure;
static volatile float _current_volume;

volatile sensors_state_t _sensor_state;

static volatile uint16_t _hyperfrish_sdp_time;
static volatile uint16_t _hyperfrish_npa_time;

static void (*_flow_callback)(float flow, uint32_t delta_t_us);


static void process_i2c_callback(I2C_HandleTypeDef *hi2c);
static void period_callback(TIM_HandleTypeDef* 	tim);



float 		sensors_get_flow() { return _current_flow; } // in slm
float 		sensors_get_pressure() { return _current_pressure; }
float 		sensors_get_volume() { return _current_volume; }
void      sensors_reset_volume() { _current_volume = 0; }


bool sensors_init(I2C_HandleTypeDef* hi2c , TIM_HandleTypeDef* timer1kHz) {

	_i2c= hi2c;

	_i2c->MasterTxCpltCallback=process_i2c_callback;
	_i2c->MasterRxCpltCallback=process_i2c_callback;
	_i2c->ErrorCallback=process_i2c_callback;

	_flow_callback = NULL;

	_sensor_state= STOPPED;

	HAL_Delay(100);

	// First try to complete pending sdp rad request if any !!!
	if(HAL_I2C_Master_Receive(_i2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer), 1000)!= HAL_I2C_ERROR_NONE) {
		printf("Tried to finished pending sdp read request... but nothing came...\n");
	}

	// Reset SDP
	if(HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}


	HAL_Delay(100);
	// Now read sdp advanced user register
	if(HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	if(HAL_I2C_Master_Receive(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// print AUR (Advances User Register)
	uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
//	printf("sdp AUR = %d\n", (uint16_t)(sdp_aur));
	uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
	_sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
	_sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
	// Now disable i2c hold master mode
	if(HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// Sensors settle time
	HAL_Delay(100);

	_timer1kHz= timer1kHz;
	_timer1kHz->PeriodElapsedCallback= period_callback;


	return true;
}

void sensors_start() {
	// Start sensor state machine.
	// This state machine is managed in the I2C interupt routine.
//	_sensor_state= STOPPED;
	_sensor_state= REQ_SDP_MEASUREMENT;
	HAL_I2C_Master_Transmit_IT(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
	while(_sensor_state!=STOPPED);
	HAL_TIM_Base_Start_IT(_timer1kHz);

}

void sensors_stop() {
	HAL_TIM_Base_Stop_IT(_timer1kHz);
	while(_sensor_state!=STOPPED);
}


void sensors_set_flow_callback(void (*callback)(float flow, uint32_t delta_t_us)) {
	_flow_callback= callback;
}


void sensors_scan(I2C_HandleTypeDef *hi2c) {

	for (int t = 1; t < 127; ++t) {
		if(HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(t<<1), 2, 2) == HAL_OK) {
			printf("Found device at address: %02X\n", t);
		}

	}


	printf("Done...\n");

}

static void period_callback(TIM_HandleTypeDef* 	tim) {
	static uint32_t t=0;
	if(_sensor_state==STOPPED) {
		_sensor_state= READ_NPA_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(_i2c, ADDR_NPA700B, (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
		if(++t==500) {
			HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin);
			t=0;

//			printf("%u - %u\n", _hyperfrish_npa_time, _hyperfrish_sdp_time);
		}
	}
}

static void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
	static	uint32_t hyperfrish_npa;
	static	uint32_t hyperfrish_sdp;

	switch (_sensor_state) {
	case STOPPED:
		return;
	case READ_NPA_MEASUREMENT:
//		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
//			HAL_I2C_Master_Receive_IT(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
//			break;
//		}
//		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
//			// TODO: Manage error
//			_sensor_state= STOPPED;
//			break;
//		}
		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_NONE) {
			if( (_npa_measurement_buffer[0]>>6)==0) {
				_hyperfrish_npa_time = get_time_us() - hyperfrish_npa;

				hyperfrish_npa = get_time_us();
				uint16_t praw =  (((uint16_t)_npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
				_current_pressure = 70.307 * ((float) ( praw - 1638.)/13107.);
			} else if((_npa_measurement_buffer[0]>>6)==3) {
				// TODO: Manage error status !!
			}
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );

		break;

	case REQ_SDP_MEASUREMENT:
		_sensor_state= STOPPED;
//		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
//			HAL_I2C_Master_Transmit_IT(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
//			break;
//		}
//		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
//			// TODO: Manage error
//			_sensor_state= STOPPED;
//			break;
//		}
//		_sensor_state= READ_SDP_MEASUREMENT;
//		HAL_I2C_Master_Receive_IT(hi2c, ADDR_SPD610 , (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );
		break;

	case READ_SDP_MEASUREMENT:

//		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
//			_sensor_state= READ_NPA_MEASUREMENT;
//			HAL_I2C_Master_Receive_IT(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
//			break;
//		}
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		if(_sdp_measurement_buffer[0] != 0xFF || _sdp_measurement_buffer[1] != 0xFF || _sdp_measurement_buffer[2] != 0xFF){
			_hyperfrish_sdp_time= get_time_us() - hyperfrish_sdp;
			hyperfrish_sdp = get_time_us();
			int16_t dp_raw   = (int16_t)((((uint16_t)_sdp_measurement_buffer[0]) << 8) | (uint8_t)_sdp_measurement_buffer[1]);
			_current_flow = -((float)dp_raw)/105.0;
			_current_volume += (_current_flow/60.) * ((float)_hyperfrish_sdp_time/1000000);

			if(_flow_callback != NULL) {
				_flow_callback(_current_flow, _hyperfrish_sdp_time);
			}
			_sensor_state= REQ_SDP_MEASUREMENT;
			HAL_I2C_Master_Transmit_IT(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
		} else {
//			_sensor_state= READ_NPA_MEASUREMENT;
//			HAL_I2C_Master_Receive_IT(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
			_sensor_state= STOPPED;
		}
		break;
	}
}
