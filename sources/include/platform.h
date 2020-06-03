#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include "common.h"
#include "platform_defs.h"



void log_float(uint8_t channel, float val);
void log_uint32(uint8_t channel, uint32_t val);
void log_int32(uint8_t channel, int32_t val);
void log_uint16(uint8_t channel, uint16_t val);
void log_int16(uint8_t channel, int16_t val);
void log_uint8(uint8_t channel, uint8_t val);
void log_int8(uint8_t channel, int8_t val);
void log_str(uint8_t channel, const char* str);


uint32_t get_time_ms();

//! \returns get_time_ms()
uint32_t wait_ms(uint32_t t_ms);


//! Triggers a soft reset that will restart the Controller in a fresh state with the same memorised settings
//! \warning the actual reset MUST be synchronized with cycle_respiration
//! \warning it must also wait for completion of all lowlevel operations that may corrupt the controller state like Flash writes
bool soft_reset();

// ------------------------------------------------------------------------------------------------
//! IHM (UI) communication

//! Called during initialisation only
//! \returns false only if communication will never be possible for some reason
//! \warning since IHM may not be started yet, do not expect any answer yet
bool init_uart();

//! \returns true if frame sent
//! \sa fputs()
bool uart_send(const char* frame);

//! \returns EOF if nothing received or an ASCII unsigned char
//! \sa fgetc()
int uart_recv();


// ------------------------------------------------------------------------------------------------
//! HW actuators

typedef enum {
    MOTOR_PRESS,
    MOTOR_RELEASE
} motor_dir_t;

typedef uint32_t (*motor_step_callback_t)(uint32_t elapsed_step_us);


//! Called during initialisation only
bool motor_init(uint32_t home_step_us);

//! \returns false in case of hardware failure
bool motor_is_ok();

//! Press the BAVU to insufflate air to the patient according to the defined steps_profile in µs/step
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_press(uint32_t* steps_profile_us, unsigned int nb_steps);

uint32_t motor_press_get_current_step();

//! Release the BAVU to prepare next insufflation at any appropriate speed
//! \remark this includes releasing BAVU until motor home position and possibly moving forward to erase a flat part of pos(Vol) map
//! \warning motor driver is responsible to handle low-level errors in the best way to ensure corresponding action
bool motor_release(uint32_t step_us);

void motor_enable(bool ena);

//!
bool motor_is_moving();

//!
bool motor_is_home();

//!
bool motor_stop();

//!
bool motor_move(motor_dir_t dir, uint32_t step_us, motor_step_callback_t callback);



// ------------------------------------------------------------------------------------------------

//! Called during initialisation only
bool motor_pep_init();

//! \returns false in case of hardware failure
bool motor_pep_is_ok();

//! Move up if steps > 0 else down
//! \warning after init, only ask for small moves controlling PEP during next cycles before moving again
//! \remark unless some calibration procedure or data is added, the only relationship between steps and cm
//!         must be computed based on further read_PEP_cmH2O()
//! \remark any relative_move_mmH2O > +1 should be checked during following cycles
bool motor_pep_move(int relative_mmH2O);

//!
bool motor_pep_is_moving();

//!
bool motor_pep_stop();

//! \warning only use during initialisation as moving to 0 is prohibited during cycle_respiration and the water level may have varied
bool motor_pep_home();

//!
bool motor_pep_is_home();

// ------------------------------------------------------------------------------------------------

//! Called during initialisation only
bool valve_init();

//! \returns false in case of hardware failure
bool valve_is_ok();

//! Positions electrovalve to connect patient with PEP to allow him to exhale
bool valve_exhale();

//! Positions electrovalve to connect patient with BAVU to insufflate him or keep its airway pressure higher than PEP
bool valve_inhale();


// ------------------------------------------------------------------------------------------------
//! HW sensors

//! Called during initialisation only
bool sensors_init();

bool sensors_start(); //!< Starts I2C sensing of Pdiff, Paw, Patmo using interrupts
bool sensors_stop(); //!< Starts I2C sensing of Pdiff, Paw, Patmo using interrupts

//! \returns false in case of hardware failure
bool Pdiff_is_ok();
bool Paw_is_ok();
bool Patmo_is_ok();

//! \returns the airflow corresponding to a pressure difference in Liters / minute
float read_Pdiff_Lpm();

//! \returns the sensed pressure in cmH2O (1,019mbar in standard conditions)
float read_Paw_cmH2O();

//! \returns the atmospheric pressure in mbar
float read_Patmo_mbar();

//! \returns the atmospheric pressure in mbar
float read_temp_degreeC();

//! \returns the current integrated volume
float read_Vol_mL();

//! reset current volume integration to 0
void reset_Vol_mL();


// ------------------------------------------------------------------------------------------------
//! HW UPS status

//! \returns false in case of hardware failure (while is_DC_on) or imminent stop
bool is_Battery_ok();

//! \returns true if DC power is on
bool is_DC_on();

//! \returns true if battery autonomy is > 30min
bool is_Battery_charged();

// ------------------------------------------------------------------------------------------------
//! FailSafe
bool is_Failsafe_Enabled();

// ------------------------------------------------------------------------------------------------
//! FailSafe
void enable_Rpi(bool ena);



// ------------------------------------------------------------------------------------------------
//! HW indicators

typedef enum { Off, On } OnOff;

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

#endif
