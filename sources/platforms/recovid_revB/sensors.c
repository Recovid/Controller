#include "recovid_revB.h"
#include "platform.h"
#include "platform_config.h"
#include "bmp280.h"
#include <string.h>

/*
 * Notes concernant l'utilisation du bmp280.
 *
 * - La fonction d'initialisation utilise directement la lib Bosch de façon synchrone.
 *
 * - Cette lib  n'est pas adaptée pour une utilisation asynchrone avec usage d'interruptions,
 * la partie lecture dans la boucle principale, interroge directement le capteur sans passer par
 * la lib pour récupérer les valeurs brutes. La compensation des valeurs est effectuée par un appel à une
 * fonction de la lib Bosch.
 *
 */

/* Trick : utilisation d'un masque pour timer la mesure de pression / température.
 * Permet de gérer une durée par puissance de 2. l'unité étant 4.8ms (période de mesure du SDP610)
 */
#define BMP280_PERIOD 0xFFF  // une mesure toutes les 4.8 * 4095 = 19.656s

typedef enum {
	STOPPED,
	STOPPING,
	REQ_SDP_MEASUREMENT,
	READ_SDP_MEASUREMENT,
	READ_NPA_MEASUREMENT,
    READ_BMP280_STAGE_1,
    READ_BMP280_STAGE_2,
} sensors_state_t;


static I2C_HandleTypeDef* _i2c= NULL;
static bool initialized = false;

static const uint8_t _sdp_reset_req[1]  = { 0xFE };
static const uint8_t _sdp_readAUR_req[1]  = { 0xE5 };
static uint8_t _sdp_writeAUR_req[3]  = { 0xE4, 0x00, 0x00};

static const uint8_t _sdp_measurement_req[1] 	= { 0xF1 };
static uint8_t       _sdp_measurement_buffer[3] = { 0 };
static uint8_t       _sdp_AUR_buffer[3] 		= { 0 };

static uint8_t       _npa_measurement_buffer[2]	= { 0 };

static struct bmp280_dev bmp;
struct bmp280_uncomp_data ucomp_data;
static uint8_t bmp280_DMA_buffer[6];

static volatile float _current_flow_slm;
static volatile float _current_Patmo_mbar;
static volatile float _current_Paw_cmH2O;
static volatile float _current_temp_degreeC;
static volatile float _current_vol_mL;
static volatile	uint16_t last_sdp_t_us;
static volatile	uint16_t last_npa_t_us;
volatile sensors_state_t _sensor_state;


static void process_i2c_callback(I2C_HandleTypeDef *hi2c);

//--- BMP280
static int8_t bmp280_write_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
static int8_t bmp280_read_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

static void bmp280_delay_ms(uint32_t period_ms);
//---

static bool initBMP280();
static bool initSDP610();
static HAL_StatusTypeDef readBMP280_stage_1();
static HAL_StatusTypeDef readBMP280_stage_2();



static inline uint16_t get_time_us() { return timer_us.Instance->CNT; }

static bool sensors_init(I2C_HandleTypeDef *hi2c) {
	if (initialized)
		return true;

    _i2c = hi2c;

    if (!initSDP610() || !initBMP280())
        return false;

    // Sensors settle time
    HAL_Delay(100);

    initialized = true;

    _i2c->MasterTxCpltCallback = process_i2c_callback;
    _i2c->MasterRxCpltCallback = process_i2c_callback;
    _i2c->ErrorCallback = process_i2c_callback;

    return true;
}	

static bool initSDP610() 
{
	//
	// Scan I2C bus
	// TODO: Check if all sensors are responding.
	for (int t = 1; t < 127; ++t) {
		if(HAL_I2C_IsDeviceReady(_i2c, (uint16_t)(t<<1), 2, 2) == HAL_OK) {
			dbg_printf("Found device at address: %02X\n", t);
		}

	}
	// First try to complete pending sdp rad request if any !!!
	if (HAL_I2C_Master_Receive(_i2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer), 1000) != HAL_I2C_ERROR_NONE) {
		printf("Tried to finished pending sdp read request... but nothing came...\n");
	}

	// Reset SDP
	if (HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610, (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}

	HAL_Delay(100);
	// Now read sdp advanced user register
	if (HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610, (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	if (HAL_I2C_Master_Receive(_i2c, ADDR_SPD610, (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}
	// print AUR (Advances User Register)
	uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
	//	printf("sdp AUR = %d\n", (uint16_t)(sdp_aur));
	uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
	_sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
	_sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
	// Now disable i2c hold master mode
	if (HAL_I2C_Master_Transmit(_i2c, ADDR_SPD610, (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 1000) != HAL_I2C_ERROR_NONE) {
		return false;
	}

	return true;
}

bool init_sensors() {
	return sensors_init(&sensors_i2c);
}


static bool initBMP280() {

    struct bmp280_config conf;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = bmp280_delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = bmp280_read_direct;
    bmp.write = bmp280_write_direct;

    if (bmp280_init(&bmp) != BMP280_OK)
        return false;
    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */

    if (bmp280_get_config(&conf, &bmp) != BMP280_OK)
        return false;

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;


    /* Temperature oversampling set at 4x */
    conf.os_temp = BMP280_OS_4X;
   
    /* Pressure oversampling set at 4x */
    conf.os_pres = BMP280_OS_4X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    if (bmp280_set_config(&conf, &bmp) != BMP280_OK)
        return false;

    /* Always set the power mode after setting the configuration */
    if (bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp) != BMP280_OK)
        return false;


    return true;
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
  return _current_Patmo_mbar;
}

//! \returns the atmospheric pressure in mbar
float read_temp_degreeC() {
  return _current_temp_degreeC;
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

static HAL_StatusTypeDef readBMP280_stage_1() {
    _sensor_state = READ_BMP280_STAGE_1;
    bmp280_DMA_buffer[0] = BMP280_PRES_MSB_ADDR;
    /*
     * Envoi de l'adresse du premier registre à lire. Dans un deuxième temps (après interruption = readBMP280_stage_2), on lira la donnée
     * Cet envoi n'est pas fait en DMA car le capteur termine la séquence par un ACK qui ne lève pas l'interruptions et
     * qui ne permet pas de rentrer dans la machine d'état. On utilise donc la fonction d'envoi avec interruptions moins
     * gourmande que la version synchone. (mais la version synchrone fonctionne également si besoin.)
     */
    return HAL_I2C_Master_Transmit_IT(_i2c, BMP280_I2C_ADDR_PRIM << 1, bmp280_DMA_buffer, 1);
}

static HAL_StatusTypeDef readBMP280_stage_2() {
    uint8_t bytesToRead = 6;
    assert(bytesToRead <= sizeof(bmp280_DMA_buffer));
    _sensor_state = READ_BMP280_STAGE_2;
    return HAL_I2C_Master_Receive_DMA(_i2c, BMP280_I2C_ADDR_PRIM << 1, bmp280_DMA_buffer, bytesToRead);
}


//--- BMP280 library callbacks

static int8_t bmp280_write_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    assert(length <= sizeof(bmp280_DMA_buffer));
    bmp280_DMA_buffer[0] = reg_addr;
    memcpy(bmp280_DMA_buffer + 1, reg_data, length);
    return HAL_I2C_Master_Transmit(_i2c, i2c_addr << 1, bmp280_DMA_buffer, length + 1, 1000);
}

static int8_t bmp280_read_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    //--- envoi l'adresse du registre à lire
    HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(_i2c, i2c_addr << 1, &reg_addr, 1, 1000);
    if (r != HAL_OK)
        return r;
    //--- réception de la donnée
    return HAL_I2C_Master_Receive(_i2c, i2c_addr << 1, reg_data, length, 1000);
}

/**
 * bmp280_delay_ms n'est appelée par la lib bmp280 que lors de l'initialisation, cela ne pose donc
 * pas de pb de temps perdu dans le fonctionnement régulier du système.
 */
static void bmp280_delay_ms(uint32_t period_ms) {
    HAL_Delay(period_ms);
}

//---



bool sensors_start() {
    // Start the sensor state machine.
    // This state machine is managed in the I2C interupt routine.
	HAL_TIM_Base_Start(&timer_us);
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
                static uint16_t count = BMP280_PERIOD;
                if ((count++ & BMP280_PERIOD) == BMP280_PERIOD)
                    readBMP280_stage_1();
                else
                    readNPA();
            }
            break;
        }
        case READ_BMP280_STAGE_1: {
            uint16_t x = BMP280_PERIOD;
            readBMP280_stage_2();
            break;
        }
        case READ_BMP280_STAGE_2: {
            uint8_t *temp = bmp280_DMA_buffer;
            uint32_t uncomp_press = (int32_t)((((uint32_t)(temp[0])) << 12) | (((uint32_t)(temp[1])) << 4) | ((uint32_t) temp[2] >> 4));
            uint32_t uncomp_temp = (int32_t)((((int32_t)(temp[3])) << 12) | (((int32_t)(temp[4])) << 4) | (((int32_t)(temp[5])) >> 4));

            if ((uncomp_press > BMP280_ST_ADC_P_MIN) && (uncomp_press < BMP280_ST_ADC_P_MAX)) {
                uint32_t pressure_Pa;
                bmp280_get_comp_pres_32bit(&pressure_Pa, uncomp_press, &bmp); // inutile de gérer l'erreur, la fonction appelée vérifie juste que bmp n'est pas null.
				_current_Patmo_mbar = (float)pressure_Pa / 100.0;
            } else {
                // todo gérer l'erreur
             }

            if ((uncomp_temp > BMP280_ST_ADC_T_MIN) && (uncomp_temp < BMP280_ST_ADC_T_MAX)) {
                int32_t temperature_degreeCx100;
                bmp280_get_comp_temp_32bit(&temperature_degreeCx100, uncomp_temp, &bmp); // inutile de gérer l'erreur, la fonction appelée vérifie juste que bmp n'est pas null.
				_current_temp_degreeC = (float)temperature_degreeCx100 / 100.0;
            } else {
                // todo gérer l'erreur
            }
            readNPA();
			break;
		}
	}
}



