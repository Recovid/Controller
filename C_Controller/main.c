#include "controller.h"
#include "controller_settings.h"
#include "ihm_communication.h"

int main()
{
    connect();
    // TODO replace with STM32 code that will:
    // - initialize the hardware
    // - schedule a cyclic task
    // - set init_str (controller.h), then:
    send_INIT(init_str);

    for (;;) {
        if (!run())
            return 0;
    }
}
