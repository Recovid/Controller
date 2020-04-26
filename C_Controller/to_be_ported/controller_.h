#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Public interface to the controller state machine


typedef enum { Insufflation, Plateau, Exhalation, ExhalationPause, Unknown } RespirationState;

//! \returns current RespirationState to schedule state-sensitive tasks
RespirationState current_respiration_state();

typedef enum { Ajustement, Maintien } PEPState;

void cycle_respiration();

const char *get_init_str();

#ifndef NTESTS
bool TEST_CONTROLLER();
#endif

#endif // CONTROLLER_H
