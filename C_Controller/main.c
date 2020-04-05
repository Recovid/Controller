#include "platform.h"
#include <stdio.h>
#include <string.h>
#include "controller.h"
#include "controller_settings.h"
#include "alarms.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"

#ifndef NDEBUG
#include "unit_tests.h"
#endif

int main()
{
#ifndef NDEBUG
    if (!unit_tests_passed())
        return -2;
#endif

    int result = self_tests();
    // if (result & 0b111111)
    //     return -1;

    if (argc != 1 && argc != 3)
    {
        printf("Usage: %s [inputFile outputFile]\n", argv[0]);
        return 1;
    }
    if (argc == 1)
        init_ihm(0, 0);
    else
        init_ihm(argv[1], argv[2]);
    // TODO replace with STM32 code that will:
    // - initialize the hardware
    // - schedule cyclic tasks
    // - set init_str (controller.h), then:

    snprintf(init_str, INIT_STR_SIZE, "Start simulation self-tests:%o", result);
    send_INIT(init_str);

    // TODO simulate interrupts/scheduling/preemption using QThread in a separate main.cpp
    for (long t=0; true; wait_ms(1)) { // 1kHz

        sense_and_compute();

        update_alarms();

        cycle_respiration();

        if (++t % 25) { // 40Hz

            if (!send_and_recv()) return 0;

        }
    }
}
