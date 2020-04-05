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
//! IHM simulation based on stdin/stdout

bool init_ihm(const char* pathInputFile, const char* pathOutputFile);

//! \returns true if frame sent
//! \sa fputs()
bool send_ihm(const char* frame);

//! \returns EOF if nothing received or an ASCII unsigned char
//! \sa fgetc()
int recv_ihm();

#endif // HARDWARE_SIMULATION_H
