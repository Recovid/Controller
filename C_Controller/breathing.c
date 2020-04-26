#include "recovid.h"
#include "controller.h"
#include "breathing.h"



void breathing_run(void *args) {
  UNUSED(args);
  EventBits_t events;


  while(true) {
    brth_printf("Breathing - STANDBY\n");
    events= xEventGroupWaitBits(eventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    brth_printf("Monitoring started\n");

    do  {
      wait_ms(1000);
      brth_printf("breathing\n");




      events= xEventGroupWaitBits(eventFlags, BREATHING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    } while ( ( events & MONITORING_RUN_FLAG ) != 0 );



    wait_ms(200);
  }
}