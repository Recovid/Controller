#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>

// ------------------------------------------------------------------------------------------------
//! Public interface to the controller state machine

// Global settings that can be atomically read
// Only main.c and communication.h should ever update them

bool run();

#endif // CONTROLLER_H
