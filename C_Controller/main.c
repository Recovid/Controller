//FreeRTOS Include
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

//STD include
#include <stdio.h>
#include <string.h>

//Recovid include
#include "platform.h"
#include "controller.h"
#include "alarms.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"
#include "tasks_recovid.h"

#ifndef NDEBUG
#include "unit_tests.h"
#endif

int main(int argc, const char** argv)
{
#ifndef NDEBUG
    if (!unit_tests_passed())
        return -2;
#endif
    int result = self_tests();
    // if (result & 0b111111)
    //     return -1;

    if (argc == 1) {
        init_ihm(0, 0, NULL);
    }
    else if(argc == 4 && strstr(argv[1], "-f")) {
        init_ihm(IHM_MODE_FILE, argv[2], argv[3]);
    }
    else if(argc == 3 && strstr(argv[1], "-s")) {
        init_ihm(IHM_MODE_SERIAL, argv[2], NULL);
    }
    else {
        printf("Usage: %s Default file mode\n", argv[0]);
        printf("Usage : -f [inputFile outputFile] for ihm in file mode\n");
        printf("Usage : -s [serialPort] for ihm in serial mode\n");

        return 1;
    }

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
}
