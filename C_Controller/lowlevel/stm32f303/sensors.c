#include "recovid_revB.h"
#include "lowlevel.h"
#include "recovid.h"

typedef enum {
	STOPPED,
	STOPPING,
	REQ_SDP_MEASUREMENT,
	READ_SDP_MEASUREMENT,
	READ_NPA_MEASUREMENT
} sensors_state_t;


static I2C_HandleTypeDef* _i2c= NULL;

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

// static void (*_flow_callback)(float flow, uint32_t delta_t_us);

float    samples_Q_Lps[2000]; // > max Tinsu_ms
float    average_Q_Lps[2000]; // > max Tinsu_ms

static float    samples_Q_t_ms  = 0.f;
static uint16_t samples_Q_index = 0;
static bool     sampling_Q      = false;


static void process_i2c_callback(I2C_HandleTypeDef *hi2c);
static inline uint16_t get_time_us() { return timer_us.Instance->CNT; }

bool init_sensors() {
  if(_i2c!=NULL) return true;

	// Scan I2C bus
	// TODO: Check if all sensors are responding.
	for (int t = 1; t < 127; ++t) {
			if(HAL_I2C_IsDeviceReady(&sensors_i2c, (uint16_t)(t<<1), 2, 2) == HAL_OK) {
				dbg_printf("Found device at address: %02X\n", t);
			}

		}



	// First try to complete pending sdp rad request if any !!!
	if(HAL_I2C_Master_Receive(&sensors_i2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer), 1000)!= HAL_I2C_ERROR_NONE) {
		// No pending read :-). keep going
	}

	// Reset SDP
	if(HAL_I2C_Master_Transmit(&sensors_i2c, ADDR_SPD610 , (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}


	HAL_Delay(100);
	// Now read sdp advanced user register
	if(HAL_I2C_Master_Transmit(&sensors_i2c, ADDR_SPD610 , (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	if(HAL_I2C_Master_Receive(&sensors_i2c, ADDR_SPD610 , (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// print AUR (Advances User Register)
	uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
//	printf("sdp AUR = %d\n", (uint16_t)(sdp_aur));
	uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
	_sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
	_sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
	// Now disable i2c hold master mode
	if(HAL_I2C_Master_Transmit(&sensors_i2c, ADDR_SPD610 , (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// Sensors settle time
	HAL_Delay(100);

	_i2c = &sensors_i2c;


	_i2c->MasterTxCpltCallback=process_i2c_callback;
	_i2c->MasterRxCpltCallback=process_i2c_callback;
	_i2c->ErrorCallback=process_i2c_callback;

	return true;
}


//! \returns false in case of hardware failure
bool is_Pdiff_ok() {
  return _i2c!=NULL;
}
bool is_Paw_ok() {
  return _i2c!=NULL;
}
bool is_Patmo_ok() {
  return _i2c!=NULL;
}

//! \returns the airflow corresponding to a pressure difference in Liters / minute
float read_Pdiff_Lpm() {
  return _current_flow;
}

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float read_Paw_cmH2O() {
  return _current_pressure;
}

//! \returns the atmospheric pressure in mbar
float read_Patmo_mbar() {
  return 0;
}

 //! \returns the current integrated volume
float read_Vol_mL() {
	return _current_volume;
}

 //! reset current integrated volume to 0
void reset_Vol_mL() {
	_current_volume = 0;
}



// bool sensors_start_sampling_flow()
// {
//     samples_Q_t_ms = 0.f;
//     samples_Q_index = 0;
//     sampling_Q = true;
//     return sampling_Q;
// }

// bool sensors_stop_sampling_flow()
// {
//     sampling_Q = false;
//     return !sampling_Q;
// }

// uint16_t get_samples_Q_index_size()
// {
//     return 0; // TODO
// }

bool sensors_start() {
    // Start the sensor state machine.
    // This state machine is managed in the I2C interupt routine.
		HAL_TIM_Base_Start(&timer_us);
	_sensor_state= REQ_SDP_MEASUREMENT;
	HAL_I2C_Master_Transmit_IT(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
	return true;
}

bool sensors_stop() {
    // Stop the sensor state machine.
	__disable_irq();
	_sensor_state= STOPPED;
	__enable_irq();
	return true;
}

static void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
	// TODO: First flow integration will not be correct !!! Find a way to indicate that this is the first sampling !!
	static	uint32_t hyperfrish_sdp = 0;

	switch (_sensor_state) {
	case STOPPING:
		_sensor_state=STOPPED;
		return;
	case STOPPED:
		// TODO: remove
		light_red(On);
		return;
	case READ_NPA_MEASUREMENT:
		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
			// Retry
			HAL_I2C_Master_Receive_DMA(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
			break;
		}
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_NONE) {
			if( (_npa_measurement_buffer[0]>>6)==0) {
				uint16_t praw =  (((uint16_t)_npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
				_current_pressure = 1.01972 * ((float) 160*( praw - 1638.)/13107.);
			} else if((_npa_measurement_buffer[0]>>6)==3) {
				// TODO: Manage error status !!
			}
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );

		break;

	case REQ_SDP_MEASUREMENT:
		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
			// retry
			HAL_I2C_Master_Transmit_IT(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
			break;
		}
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(hi2c, ADDR_SPD610 , (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );
		break;

	case READ_SDP_MEASUREMENT:
		if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
			_sensor_state= READ_NPA_MEASUREMENT;
			HAL_I2C_Master_Receive_DMA(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
			break;
		}
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

			// if(_flow_callback != NULL) {
			// 	_flow_callback(_current_flow, _hyperfrish_sdp_time);
			// }
			_sensor_state= REQ_SDP_MEASUREMENT;
			HAL_I2C_Master_Transmit_IT(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
		} else {
			_sensor_state= READ_NPA_MEASUREMENT;
			HAL_I2C_Master_Receive_DMA(hi2c, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
		}
		break;
	}
}
