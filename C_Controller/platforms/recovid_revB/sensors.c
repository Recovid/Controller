#include "recovid_revB.h"
#include "platform.h"
#include "recovid.h"
#include "log_timings.h"

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

static volatile float _current_flow_slm;
static volatile float _current_Paw_cmH2O;
static volatile float _current_vol_mL;
static volatile	uint16_t last_sdp_t_us;
static volatile	uint16_t last_npa_t_us;
volatile sensors_state_t _sensor_state;


static void process_i2c_callback(I2C_HandleTypeDef *hi2c);
static inline uint16_t get_time_us() { return timer_us.Instance->CNT; }

static bool sensors_init(I2C_HandleTypeDef *hi2c) {
	if (_i2c != NULL)
		return true;

	// Scan I2C bus
	// TODO: Check if all sensors are responding.
	for (int t = 1; t < 127; ++t) {
		if(HAL_I2C_IsDeviceReady(&sensors_i2c, (uint16_t)(t<<1), 2, 2) == HAL_OK) {
			dbg_printf("Found device at address: %02X\n", t);
		}

	}
	// First try to complete pending sdp rad request if any !!!
	if (HAL_I2C_Master_Receive(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer), 1000) != HAL_I2C_ERROR_NONE) {
		printf("Tried to finished pending sdp read request... but nothing came...\n");
		LOG_TIME_DUMP()
	}

	// Reset SDP
	if (HAL_I2C_Master_Transmit(hi2c, ADDR_SPD610, (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}

	HAL_Delay(100);
	// Now read sdp advanced user register
	if (HAL_I2C_Master_Transmit(hi2c, ADDR_SPD610, (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	if (HAL_I2C_Master_Receive(hi2c, ADDR_SPD610, (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}
	// print AUR (Advances User Register)
	uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
	//	printf("sdp AUR = %d\n", (uint16_t)(sdp_aur));
	uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
	_sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
	_sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
	// Now disable i2c hold master mode
	if (HAL_I2C_Master_Transmit(hi2c, ADDR_SPD610, (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}
	// Sensors settle time
	HAL_Delay(100);

	_i2c = hi2c;

	_i2c->MasterTxCpltCallback = process_i2c_callback;
	_i2c->MasterRxCpltCallback = process_i2c_callback;
	_i2c->ErrorCallback = process_i2c_callback;

	return true;
}

bool init_sensors() {
	return sensors_init(&sensors_i2c);
}

//! \returns false in case of hardware failure
bool is_sensors_ok() {
	return _i2c != NULL;
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
	// TODO: correct with ambiant pressure and T°
  return _current_flow_slm;
}

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float read_Paw_cmH2O() {
  return _current_Paw_cmH2O;
}

//! \returns the atmospheric pressure in mbar
float read_Patmo_mbar() {
  return 0;
}

 //! \returns the current integrated volume
float read_Vol_mL() {
	return _current_vol_mL;
}

 //! reset current integrated volume to 0
void reset_Vol_mL() {
	_current_vol_mL = 0;
}
static void readSDP() {
	_sensor_state = READ_SDP_MEASUREMENT;
	HAL_I2C_Master_Receive_DMA(_i2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer));
}

static void reqSDP() {
	_sensor_state = REQ_SDP_MEASUREMENT;
	HAL_I2C_Master_Transmit_IT(_i2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req));
}

static void readNPA() {
	_sensor_state = READ_NPA_MEASUREMENT;
	HAL_I2C_Master_Receive_DMA(_i2c, ADDR_NPA700B, (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer));
}
bool sensors_start() {
    // Start the sensor state machine.
    // This state machine is managed in the I2C interupt routine.
	HAL_TIM_Base_Start(&timer_us);
	HAL_I2C_Master_Transmit_IT(_i2c, ADDR_SPD610 , (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
	reqSDP();
	return true;
}

float compute_corrected_pressure(uint16_t read)
{
    return 1.01972f/*mbar/cmH2O*/
                        * (160.f*(read - 1638.f)/13107.f); // V1 Calibration
}

//! \warning TODO compute corrected QPatientSLM (Standard Liters per Minute) based on Patmo
float compute_corrected_flow(int16_t read)
{
    return -(float) read / 105.f; // V1 Calibration
}


static void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
	switch (_sensor_state) {
		case STOPPING:
			_sensor_state = STOPPED;
			return;
		case STOPPED:
			return;
		case READ_NPA_MEASUREMENT: {
			if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
				// Retry
				readNPA();
				break;
			}
			if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
				// TODO: Manage error
				_sensor_state = STOPPED;
				break;
			}
			if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_NONE) {
				if ((_npa_measurement_buffer[0] >> 6) == 0) {
					uint16_t npa_t_us = (uint16_t)get_time_us();
					uint16_t npa_dt_us = (uint16_t)npa_t_us - last_npa_t_us;
					uint16_t praw = (((uint16_t) _npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
					_current_Paw_cmH2O = compute_corrected_pressure(praw);
					last_npa_t_us = npa_t_us;
				} else if ((_npa_measurement_buffer[0] >> 6) == 3) {
					// TODO: Manage error status !!
				}
			}
			readSDP();

			break;
		}
		case REQ_SDP_MEASUREMENT: {
			if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
					// retry
					HAL_I2C_Master_Transmit_IT(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req));
					break;
			}
			if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
					// TODO: Manage error
					_sensor_state = STOPPED;
					break;
			}
			readSDP();
			break;
		}
		case READ_SDP_MEASUREMENT: {
			if (HAL_I2C_GetError(hi2c) == HAL_I2C_ERROR_AF) {
				readNPA();
				break;
			}
			if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
				// TODO: Manage error
				_sensor_state = STOPPED;
				break;
			}
			if (_sdp_measurement_buffer[0] != 0xFF || _sdp_measurement_buffer[1] != 0xFF || _sdp_measurement_buffer[2] != 0xFF) {
				uint16_t sdp_t_us = (uint16_t)get_time_us();
				uint16_t sdp_dt_us = (uint16_t)sdp_t_us - last_sdp_t_us;
				int16_t dp_raw   = (int16_t)((((uint16_t)_sdp_measurement_buffer[0]) << 8) | (uint8_t)_sdp_measurement_buffer[1]);
				_current_flow_slm = compute_corrected_flow(dp_raw);
      			_current_vol_mL += (_current_flow_slm/60.) * ((float)sdp_dt_us/1000);
				last_sdp_t_us = sdp_t_us;
                readSDP();
			} else {
				readNPA();
			}
			break;
		}
	}
}



