#include "configuration.h"
#include "recovid_revB.h"
#include "lowlevel.h"

#include "sensing.h"

typedef enum {
    STOPPED,
    STOPPING,
    REQ_SDP_MEASUREMENT,
    READ_SDP_MEASUREMENT,
    READ_NPA_MEASUREMENT
} sensors_state_t;

static I2C_HandleTypeDef *_i2c = NULL;

static const uint8_t _sdp_reset_req[1] = { 0xFE };
static const uint8_t _sdp_readAUR_req[1] = { 0xE5 };
static uint8_t _sdp_writeAUR_req[3] = { 0xE4, 0x00, 0x00 };

static const uint8_t _sdp_measurement_req[1] = { 0xF1 };
static uint8_t _sdp_measurement_buffer[3] = { 0 };
static uint8_t _sdp_AUR_buffer[3] = { 0 };

static uint8_t _npa_measurement_buffer[2] = { 0 };

volatile sensors_state_t _sensor_state;

static void process_i2c_callback(I2C_HandleTypeDef *hi2c);
static void readSDP();
static void reqSDP();
static void readNPA();

static bool sensors_init(I2C_HandleTypeDef *hi2c) {
    if (_i2c != NULL)
        return true;

    // First try to complete pending sdp rad request if any !!!
    if (HAL_I2C_Master_Receive(hi2c, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer), 1000) != HAL_I2C_ERROR_NONE) {
        printf("Tried to finished pending sdp read request... but nothing came...\n");
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

bool sensors_start() {
    // Start sensor state machine.
    // This state machine is managed in the I2C interupt routine.
    reqSDP();
    return true;
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

static void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
    static uint32_t last_sdp_t_ms = 0;
    static uint32_t last_npa_t_ms = 0;

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
                    uint16_t praw = (((uint16_t) _npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
                    const uint32_t npa_t_ms = get_time_ms();
                    if (last_npa_t_ms < npa_t_ms + (SAMPLES_T_US / 1000)) {
                        sensors_sample_P(praw); // Pressure (Paw) sensor is assumed to provide responses @ 1kHz
                        last_npa_t_ms = npa_t_ms;
                    }
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
                const uint32_t t_ms = get_time_ms();
                if (last_sdp_t_ms < t_ms + (SAMPLES_T_US / 1000)) {
                    light_yellow(On);
                    const uint32_t dt_ms = t_ms - last_sdp_t_ms;
                    last_sdp_t_ms = t_ms;

                    int16_t uncorrected_flow = (int16_t)((((uint16_t) _sdp_measurement_buffer[0]) << 8) | (uint8_t) _sdp_measurement_buffer[1]);

                    sensors_sample_flow(uncorrected_flow, dt_ms); // Flow (Pdiff) sensor is assumed to provide responses @ 200Hz
                }
                readSDP();
            } else {
                readNPA();
            }
            break;
        }
    }
}
