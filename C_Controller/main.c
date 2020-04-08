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
#include "controller_settings.h"
#include "alarms.h"
#include "ihm_communication.h"
#include "hardware_simulation.h"
#include "tasks_recovid.h"

#ifndef NDEBUG
#include "unit_tests.h"
#endif

const char *byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

QueueHandle_t xQueueMessage;

int main(int argc, const char** argv)
{
#ifndef NDEBUG
    if (!unit_tests_passed())
        return -2;
#endif
    int result = self_tests();
	printf("Test result %s\n",byte_to_binary(result));
    //if (result & 0b111111)
    //	return -1;

    if (argc != 1 && argc != 3)
    {
        printf("Usage: %s [inputFile outputFile]\n", argv[0]);
        return 1;
    }
    if (argc == 1)
        init_ihm(0, 0);
    else {
		printf("Je vais init %s %s\n",argv[1], argv[2]);
        init_ihm(argv[1], argv[2]);
		printf("J'ai init %s %s\n",argv[1], argv[2]);
	}
    // TODO replace with STM32 code that will:
    // - initialize the hardware
    // - schedule cyclic tasks
    // - set init_str (controller.h), then:

    snprintf(init_str, INIT_STR_SIZE, "Start simulation self-tests:%o", result);
    send_INIT(init_str);
	printf("%s\n", init_str);
	xQueueMessage = xQueueCreate( NB_MAX_MESSAGE, sizeof(struct message*));
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
