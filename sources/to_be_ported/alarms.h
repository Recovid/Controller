#ifndef ALARMS_H
#define ALARMS_H

#include "platform.h"

bool update_alarms();

void set_alarm(int32_t alarm);
void unset_alarm(int32_t alarm);

#ifndef NTESTS
bool TEST_ALARMS();
#endif

#endif // ALARMS_H
