#ifndef UNIT_TESTS_H
#define UNIT_TESTS_H

#include "platform.h"

bool unit_tests_passed();

#ifndef WIN32
bool blink();
#endif

#endif // UNIT_TESTS_H
