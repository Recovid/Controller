#include "configuration.h"
#include "recovid_revB.h"
#include "lowlevel.h"
#include "bmp280.h"

#include "sensing.h"

typedef enum {
    STOPPED, STOPPING, REQ_SDP_MEASUREMENT, READ_SDP_MEASUREMENT, READ_NPA_MEASUREMENT, READ_BMP280,
} sensors_state_t;

static I2C_HandleTypeDef *_i2c = NULL;

static const uint8_t _sdp_reset_req[1] = { 0xFE };
static const uint8_t _sdp_readAUR_req[1] = { 0xE5 };
static uint8_t _sdp_writeAUR_req[3] = { 0xE4, 0x00, 0x00 };

static const uint8_t _sdp_measurement_req[1] = { 0xF1 };
static uint8_t _sdp_measurement_buffer[3] = { 0 };
static uint8_t _sdp_AUR_buffer[3] = { 0 };

static uint8_t _npa_measurement_buffer[2] = { 0 };
static struct bmp280_dev bmp;
struct bmp280_uncomp_data ucomp_data;

volatile sensors_state_t _sensor_state;

static void process_i2c_callback(I2C_HandleTypeDef *hi2c);

//--- BMP280
static int8_t bmp280_write_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
static int8_t bmp280_read_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

static int8_t bmp280_write_DMA(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
static int8_t bmp280_read_DMA(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

static void bmp280_delay_ms(uint32_t period_ms);
//---

static void initBMP280(I2C_HandleTypeDef *hi2c);
static void readSDP();
static void reqSDP();
static void readNPA();
static void readBMP280();

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

    initBMP280(hi2c);

    // Sensors settle time
    HAL_Delay(100);

    _i2c = hi2c;

    _i2c->MasterTxCpltCallback = process_i2c_callback;
    _i2c->MasterRxCpltCallback = process_i2c_callback;
    _i2c->ErrorCallback = process_i2c_callback;

    return true;
}

void initBMP280(I2C_HandleTypeDef *hi2c) {
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

    /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

    /*
     * bmp.dev_id = 0;
     * bmp.read = spi_reg_read;
     * bmp.write = spi_reg_write;
     * bmp.intf = BMP280_SPI_INTF;
     */
    int8_t rslt = bmp280_init(&bmp);
    //print_rslt(" bmp280_init status", rslt);

    /* Always read the current settings before writing, especially when
     * all the configuration is not modified
     */
    rslt = bmp280_get_config(&conf, &bmp);
    //print_rslt(" bmp280_get_config status", rslt);

    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
    conf.filter = BMP280_FILTER_COEFF_2;

    /* Pressure oversampling set at 4x */
    conf.os_pres = BMP280_OS_4X;

    /* Setting the output data rate as 1HZ(1000ms) */
    conf.odr = BMP280_ODR_1000_MS;
    rslt = bmp280_set_config(&conf, &bmp);
    //print_rslt(" bmp280_set_config status", rslt);

    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    //print_rslt(" bmp280_set_power_mode status", rslt);

    bmp.read = bmp280_read_DMA;
    bmp.write = bmp280_write_DMA;

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

static void readBMP280() {
    _sensor_state = READ_BMP280;
    bmp280_get_uncomp_data(&ucomp_data, &bmp);
}

//--- BMP280 library callbacks

static int8_t bmp280_write_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return 0;
}

static int8_t bmp280_read_direct(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return HAL_I2C_Master_Receive(_i2c, i2c_addr, reg_data, length, 1000);
}

static int8_t bmp280_write_DMA(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return 0;
}

static int8_t bmp280_read_DMA(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length) {
    return 0;
}

/**
 * Cette fonction n'est appelée par la lib bmp280 que lors de l'initialisation, cela ne pose donc
 * pas de pb de synchronisation.
 */
static void bmp280_delay_ms(uint32_t period_ms) {
    HAL_Delay(period_ms);
}

//---

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
                static uint8_t count = 0;
                if (count++ == 0)
                    readBMP280();
                else
                    readNPA();
            }
            break;
        }
        case READ_BMP280: {
            double pressure;
            if (bmp280_get_comp_pres_double(&pressure, ucomp_data.uncomp_press, &bmp) != 0) {
                // TODO: Manage error
                _sensor_state = STOPPED;
                break;
            }
            sensors_sample_atmospheric_pressure(pressure);
            readNPA();
            break;
        }
    }
}
