#ifndef HARDWARE_SIMULATION_H
#define HARDWARE_SIMULATION_H

#include <stdbool.h>
#include "platform.h"

// Public interface to send event/data to the hardware simulation

// ------------------------------------------------------------------------------------------------
//! OS simulation
//

typedef enum ihm_mode_t
{
	IHM_MODE_FILE = 0,
	IHM_MODE_SERIAL,

	IHM_MODE_MAX,
}ihm_mode_t;

long get_time_ms();
long wait_ms(long t_ms);

bool soft_reset();

// ------------------------------------------------------------------------------------------------
//! UI communication

//! \returns false only if communication is not possible for some reason
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

#define MOTOR_MAX 2000
bool motor_press();
bool motor_stop();
bool motor_release();

#define MOTOR_PEP_MAX 100
//! Move up if steps > 0 else down
//! \warning after init, only ask for small moves controlling PEP during next cycles before moving again
//! \remark unless some calibration procedure or data is added, the only relationship between steps and cm
//!         must be computed based on further read_PEP_cmH2O()
//! \remark any read_PEP_cmH2O() changes > .1 should be checked during following cycles unless you calibrated steps/cm
bool motor_pep_move(int steps);

bool valve_exhale();
bool valve_inhale();

enum OnOff { On, Off };

bool light_yellow(enum OnOff new); // onboard + 4m visible leds
bool light_red   (enum OnOff new); // onboard + 4m visible leds
bool buzzer      (enum OnOff new); // onboard

// ------------------------------------------------------------------------------------------------
//! HW sensors

float read_Pdiff_Lpm();
float read_Paw_cmH2O();
float read_Patmo_mbar();

//! \returns 0 if battery is about to stop, 1 if battery is low
int read_Battery_level();


#endif // HARDWARE_SIMULATION_H
