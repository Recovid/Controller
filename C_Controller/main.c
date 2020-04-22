#include "platform.h"

#include <stdio.h>
#include <string.h>

//High-level include
#include "sensing.h"
#include "alarms.h"
#include "controller.h"
#include "ihm_communication.h"

//Low-level include
#include "lowlevel/include/lowlevel.h"
#include "lowlevel/include/hardware_real.h"
#ifndef WIN32
    // FreeRTOS Include
#   include <FreeRTOS.h>
#   include <task.h>
#   include <queue.h>
#   include "tasks_recovid.h"
#endif

int main(int argc, const char** argv)
{
#ifdef native
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
#else
    UNUSED(argc)
    UNUSED(argv)

    init_hardware();
    init_ihm(IHM_MODE_SERIAL, 0, 0);
#endif

    int self_tests_result = self_tests();
    // TODO TEST(self_tests_result & 0b111111)
    //     return -1;

#ifndef WIN32
    for (size_t task_idx = 0; task_idx < size_task_array; task_idx++)
    {
        // Now set up two tasks to run independently.
        if(xTaskCreate(
                task_array[task_idx].task,
                task_array[task_idx].name,   // A name just for humans
                256, // This stack size can be checked & adjusted by reading the Stack Highwater
                &task_array[task_idx],
                task_array[task_idx].priority,  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL ) != pdPASS)
        {
            return -1;
        }
    }

    for (size_t task_idx = 0; task_idx < size_timer_array; task_idx++)
    {
    	// Set up timer tasks
    	timer_array[task_idx].handle = xTimerCreate(
    			timer_array[task_idx].name,
				pdMS_TO_TICKS(timer_array[task_idx].periodMs),
				timer_array[task_idx].autoReload,
				timer_array[task_idx].id,
				timer_array[task_idx].callback);

		if(timer_array[task_idx].handle == NULL)
		{
			// The timer was not created.
			return -1;
		}
		else
		{
			// Start the timer.
			if(xTimerStart(timer_array[task_idx].handle, 0) != pdPASS )
			{
				// The timer could not be set into the Active
				return -1;
			}
		}
    }

    // start scheduler
    DEBUG_PRINT("Start tasks");
    vTaskStartScheduler();
#else
    // Deterministic simulation for test purposes
    for (uint32_t t_ms=0; true; t_ms=wait_ms(1)) { // 1kHz
        sense_and_compute(current_respiration_state());
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
