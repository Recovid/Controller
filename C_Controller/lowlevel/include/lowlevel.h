#pragma once

#include "platform.h"

// Public interface to send event/data to the hardware simulation

// TODO Adapt motor map vol->pos
// Ti, débit -> map dt->steps

// ------------------------------------------------------------------------------------------------
//! OS simulation
//

typedef enum ihm_mode_t
{
    IHM_MODE_FILE = 0,
    IHM_MODE_SERIAL,

    IHM_MODE_MAX,
} ihm_mode_t;

uint32_t get_time_ms();
uint32_t wait_ms(uint32_t t_ms);

//! Triggers a soft reset that will restart the Controller in a fresh state with the same memorised settings
bool soft_reset();

// ------------------------------------------------------------------------------------------------
//! UI communication

//! \returns false only if communication will never be possible for some reason
//! \warning since IHM may not be started yet, do not expect any answer yet
bool init_ihm(ihm_mode_t ihm_mode, const char* pathInputFile, const char* pathOutputFile);

//! \returns true if frame sent
//! \sa fputs()
bool send_ihm(const char* frame);

//! \returns EOF if nothing received or an ASCII unsigned char
//! \sa fgetc()
int recv_ihm();

// ------------------------------------------------------------------------------------------------
//! HW actuators

#define MOTOR_MAX (2000)

//! Press the BAVU to insufflate air to the patient according to get_setting_Vmax_Lpm()
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_press();

//! Release the BAVU to prepare next insufflation at any appropriate speed
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_release();

bool motor_stop();

// ------------------------------------------------------------------------------------------------

#define MOTOR_PEP_MAX (100)

//! Move up if steps > 0 else down
//! \warning after init, only ask for small moves controlling PEP during next cycles before moving again
//! \remark unless some calibration procedure or data is added, the only relationship between steps and cm
//!         must be computed based on further read_PEP_cmH2O()
//! \remark any read_PEP_cmH2O() changes > .1 should be checked during following cycles unless you calibrated steps/cm
bool motor_pep_move(int steps);

// ------------------------------------------------------------------------------------------------

//! Positions electrovalve to connect patient with PEP to allow him to exhale
bool valve_exhale();

//! Positions electrovalve to connect patient with BAVU to insufflate him or keep its airway pressure higher than PEP
bool valve_inhale();

// ------------------------------------------------------------------------------------------------
//! HW sensors

//! \returns the airflow corresponding to a pressure difference in Liters / minute
float read_Pdiff_Lpm();

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float read_Paw_cmH2O();

//! \returns the atmospheric pressure in mbar
float read_Patmo_mbar();

//! \returns -1 if battery fails, 0 if battery is about to stop, 1 if battery is low, 2 is battery is high
//! \warning specification still discussed
int read_Battery_level();
