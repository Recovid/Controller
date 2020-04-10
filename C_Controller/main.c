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
#include "hardware_simulation.h"
#ifndef WIN32
#include "tasks_recovid.h"
#include "TaskSensing.h"
#endif

#ifndef NDEBUG
#include "unit_tests.h"
#endif

int main(int argc, const char** argv)
{
    if (argc == 1) {
        init_ihm(0, 0, NULL);
    }
    else if(argc == 4 && strstr(argv[1], "-f")) {
        init_ihm(IHM_MODE_FILE, argv[2], argv[3]);
    }
    else if(argc == 3 && strstr(argv[1], "-s")) {
        init_ihm(IHM_MODE_SERIAL, argv[2], NULL);
    }
    else if(argc == 2 && strstr(argv[1], "--unit-tests")) {
        return unit_tests_passed();
    }
    else {
        printf("Usage: %s Default file mode\n", argv[0]);
        printf("Usage : -f [inputFile outputFile] for ihm in file mode\n");
        printf("Usage : -s [serialPort] for ihm in serial mode\n");
        printf("Usage : --unit-tests to run unit tests and get result\n");

        return 1;
    }

    int result = self_tests();
    // if (result & 0b111111)
    //     return -1;

#ifndef WIN32
    for(int task_idx=0; task_idx < size_task_array; task_idx++)
    {
		// Now set up two tasks to run independently.
		xTaskCreate(
				task_array[task_idx].task,
				task_array[task_idx].name,   // A name just for humans
				128, // This stack size can be checked & adjusted by reading the Stack Highwater
				&task_array[task_idx],
				task_array[task_idx].priority,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL );
    }

	// start scheduler
	printf("Start the tasks");
	vTaskStartScheduler();
#else
    // Deterministic simulation for test purposes
    for (long t_ms=0; true; t_ms+=wait_ms(1)) { // 1kHz
        sense_and_compute();
        cycle_respiration();
        if (t_ms % 25) { // 40Hz
            update_alarms();
            send_and_recv();
            if (is_soft_reset_asked() && soft_reset()) { // FIXME Let cycle_respiration handle that
                return 0;
            }
        }
    }
#endif
}
