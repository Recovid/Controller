#include <STM32FreeRTOS.h>
#include "tasks.h"
#include "struct.h"
#include "variables.h"







// the setup function runs once when you press reset or power the board
void setup() {
  xQueueMessage = xQueueCreate( NB_MAX_MESSAGE, sizeof(struct message*));
  // initialize serial communication at SERIAL_SPEED bits per second:
  Serial.begin(2000000);

  while (!Serial) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for one second
  }; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  
  for(int task_idx; task_idx < size_task_array; task_idx++)
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
  Serial.println("Start the tasks");
  vTaskStartScheduler();
  
  Serial.println("Insufficient RAM");
  while(1) {
      Serial.println("Insufficient RAM");
  }
}

void loop()
{
  // Empty. Things are done in Tasks.
}
