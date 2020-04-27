#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Public interface to the controller state machine

//! Self-tests must check critical HW failure and ensure we start cycle at PEP
//! \returns test bits where nth bit 0 denotes a failure in test 'n'
//! \warning critical failures should result in a safe stop
int self_tests();

typedef enum { RespirationOn, Insufflation, Plateau, Exhalation, ExhalationPause, RespirationOff } RespirationState;

//! \returns current RespirationState to schedule state-sensitive tasks
RespirationState current_respiration_state();

typedef enum { Ajustement, Maintien } PEPState;

void cycle_respiration();

const char *get_init_str();

#ifndef NTESTS
bool TEST_CONTROLLER();
#endif

#endif // CONTROLLER_H
