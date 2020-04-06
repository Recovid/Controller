#ifndef HARDWARE_SIMULATION_H
#define HARDWARE_SIMULATION_H

#include <stdbool.h>

// Public interface to send event/data to the hardware simulation

// ------------------------------------------------------------------------------------------------
//! OS simulation

long get_time_ms();
long wait_ms(long t_ms);

bool soft_reset();

// ------------------------------------------------------------------------------------------------
//! UI communication

bool init_ihm();

//! \returns true if frame sent
//! \sa fputs()
bool send_ihm(const char* frame);

//! \returns EOF if nothing received or an ASCII unsigned char
//! \sa fgetc()
int recv_ihm();

// ------------------------------------------------------------------------------------------------
//! HW actuators

bool motor_press();
bool motor_stop();
bool motor_release();

void valve_exhale();
void valve_inhale();

enum OnOff { On, Off };

void light_yellow(enum OnOff new); // onboard + 4m visible leds
void light_red   (enum OnOff new); // onboard + 4m visible leds
void buzzer      (enum OnOff new); // onboard

// ------------------------------------------------------------------------------------------------
//! HW sensors

float read_Pdiff_Lpm();
float read_Paw_cmH2O();
float read_Patmo_mbar();

#endif // HARDWARE_SIMULATION_H
