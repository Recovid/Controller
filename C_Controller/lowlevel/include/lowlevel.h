#pragma once

#include "platform.h"
#include <stdint.h>

#include "configuration.h"

// Public interface to the lowlevel hardware and/or simulated hardware

// ------------------------------------------------------------------------------------------------
//! OS simulation

typedef enum { IHM_MODE_FILE = 0, IHM_MODE_SERIAL, IHM_MODE_MAX, } ihm_mode_t;

uint32_t get_time_ms();

//! \returns get_time_ms()
uint32_t wait_ms(uint32_t t_ms);

uint32_t delay(uint32_t ticks_to_wait);

//! Triggers a soft reset that will restart the Controller in a fresh state with the same memorised settings
//! \warning the actual reset MUST be synchronized with cycle_respiration
//! \warning it must also wait for completion of all lowlevel operations that may corrupt the controller state like Flash writes
bool soft_reset();


// ------------------------------------------------------------------------------------------------
//! Non volatile memory (typically Flash)

//! Saves a value to non volatile memory asynchronously
//! \warning the risk of write failure is very low for the limited lifetime of the device
//! \warning soft reset must be blocked until write is finished
bool save_value_async(uint16_t index, uint16_t value);

//! Reads a value saved to non volatile memory
//! \remark value may be cached into main memory
uint16_t read_saved_value(uint16_t idx);


// ------------------------------------------------------------------------------------------------
//! IHM (UI) communication

//! Called during initialisation only
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

#define MOTOR_MAX (4800) // for v2 motor according to preliminary tests

//! Called during initialisation only
bool init_motor();

//! \returns false in case of hardware failure
bool is_motor_ok();

//! Press the BAVU to insufflate air to the patient according to VM_Lpm
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_press(uint16_t* steps_profile_us, uint16_t nb_steps);

//! Release the BAVU to prepare next insufflation at any appropriate speed
//! \param before_t_ms (in) timestamp before which motor should be in position to press BAVU again
//! \remark this includes releasing BAVU until motor home position and possibly moving forward to erase a flat part of pos(Vol) map
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_release();

//! \remark Only used in self-tests
bool motor_stop();

// ------------------------------------------------------------------------------------------------

#define MOTOR_PEP_MAX (100)

//! Called during initialisation only
bool init_motor_pep();

//! \returns false in case of hardware failure
bool is_motor_pep_ok();

//! Move up if steps > 0 else down
//! \warning after init, only ask for small moves controlling PEP during next cycles before moving again
//! \remark unless some calibration procedure or data is added, the only relationship between steps and cm
//!         must be computed based on further read_PEP_cmH2O()
//! \remark any relative_move_cmH2O > +.1 should be checked during following cycles
bool motor_pep_move(int relative_mm);

//! \warning only use during initialisation as moving to 0 is prohibited during cycle_respiration and the water level may have varied
bool motor_pep_home();

// ------------------------------------------------------------------------------------------------

//! Called during initialisation only
bool init_valve();

//! \returns false in case of hardware failure
bool is_valve_ok();

//! Positions electrovalve to connect patient with PEP to allow him to exhale
bool valve_exhale();

//! Positions electrovalve to connect patient with BAVU to insufflate him or keep its airway pressure higher than PEP
bool valve_inhale();


// ------------------------------------------------------------------------------------------------
//! HW sensors

//! Called during initialisation only
bool init_Pdiff();
bool init_Paw();
bool init_Patmo();

bool sensors_start(); //!< Starts I2C sensing of Pdiff, Paw, Patmo using interrupts

extern float motor_step_times_us[MOTOR_STEPS_MAX];

void sensors_start_sampling_flow();
void sensors_stop_sampling_flow();
void sensors_sample_flow();
float sensors_samples_time_s();

//! \returns false in case of hardware failure
bool is_Pdiff_ok();
bool is_Paw_ok();
bool is_Patmo_ok();

//! \returns the airflow corresponding to a pressure difference in Liters / minute
float read_Pdiff_Lpm();

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float read_Paw_cmH2O();

//! \returns the atmospheric pressure in mbar
float read_Patmo_mbar();


// ------------------------------------------------------------------------------------------------
//! HW UPS status

//! \returns false in case of hardware failure (while is_DC_on) or imminent stop
bool is_Battery_ok();

//! \returns true if DC power is on
bool is_DC_on();

//! \returns true if battery autonomy is > 30min
bool is_Battery_charged();


// ------------------------------------------------------------------------------------------------
//! HW indicators

typedef enum { On, Off } OnOff;

//! Called during initialisation only
bool init_indicators();

//! \returns false in case of hardware failure
bool is_light_yellow_ok();
bool is_light_red_ok();
bool is_light_green_ok();
bool is_buzzer_medium(OnOff);
bool is_buzzer_high(OnOff);
bool is_buzzer_low(OnOff);

bool light_nucleo(OnOff); //!< on nucleo led
bool light_yellow(OnOff); //!< 4m visible leds
bool light_red   (OnOff); //!< 4m visible leds
bool light_green (OnOff); //!< 4m visible leds
bool buzzer_medium(OnOff);
bool buzzer_high(OnOff);
bool buzzer_low(OnOff);

#ifndef NTESTS
bool TEST_LOWLEVEL_SIMULATION();
#endif
