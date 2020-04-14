#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Public interface to the controller state machine

//! \returns test bits where nth bit 0 denotes a failure in test 'n'
//! \warning critical failures should result in a safe stop
int self_tests();

typedef enum { Insufflation, Plateau, Exhalation, ExhalationPause } RespirationState;

//! \returns current RespirationState to schedule state-sensitive tasks
RespirationState current_respiration_state();

void cycle_respiration();

const char *get_init_str();

#endif // CONTROLLER_H
