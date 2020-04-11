#include "platform.h"

#ifndef WIN32
//FreeRTOS Include
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#endif

//STD include
#include <stdio.h>
#include <string.h>

//Recovid include
#include "sensing.h"
#include "alarms.h"
#include "controller.h"
#include "ihm_communication.h"
#include "lowlevel.h"
#ifndef WIN32
#include "tasks_recovid.h"
#include "TaskSensing.h"
#endif

#include "unit_tests.h"

int main(int argc, const char** argv)
{
    return unit_tests_passed();
}
