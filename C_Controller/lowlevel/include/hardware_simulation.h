#ifndef HARDWARE_SIMULATION_H
#define HARDWARE_SIMULATION_H
#include <stdbool.h>
#include "platform.h"

// Public interface to send event/data to the hardware simulation

// ------------------------------------------------------------------------------------------------
//! OS simulation

long get_time_ms();
long wait_ms(long t_ms);

bool soft_reset();

// ------------------------------------------------------------------------------------------------
//! UI communication

bool init_ihm(const char* pathInputFile, const char* pathOutputFile);

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

// TODO #define MOTOR_PEP_MAX;
// TODO bool motor_pep_press();
// TODO bool motor_pep_stop();
// TODO bool motor_pep_release();

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

#endif // HARDWARE_SIMULATION_H
