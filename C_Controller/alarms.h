#ifndef ALARMS_H
#define ALARMS_H

#include "platform.h"

bool update_alarms();

//! Trigger and send newly activated alarms
void trigger_alarms();

#ifndef NTESTS
bool TEST_ALARMS();
#endif

#endif // ALARMS_H
