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

bool init_ihm(const char* pathInputFile, const char* pathOutputFile);

//! \returns true if frame sent
//! \sa fputs()
bool send_ihm(const char* frame);

//! \returns EOF if nothing received or an ASCII unsigned char
//! \sa fgetc()
int recv_ihm();


#endif // HARDWARE_SIMULATION_H
