#include "controller.h"
#include "controller_settings.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"

int main()
{
    init_ihm();
    // TODO replace with STM32 code that will:
    // - initialize the hardware
    // - schedule cyclic tasks
    // - set init_str (controller.h), then:
    send_INIT(init_str);

    // TODO simulate interrupts/scheduling/preemption using QThread in a separate main.cpp
    for (long t=0; true; wait_ms(1)) { // 1kHz

        sense_and_compute();

        cycle_respiration();

        if (++t % 25) { // 40Hz

            if (!send_and_recv()) return 0;

        }
    }
}
