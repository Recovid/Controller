#include "recovid.h"
#include "monitoring.h"
#include "controller.h"
#include "breathing.h"



void monitoring_run(void *args) {
  UNUSED(args)
  EventBits_t events;

  while(true) {
    mntr_printf("Monitoring - STANDBY\n");
    events= xEventGroupWaitBits(eventFlags, MONITORING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    mntr_printf("Monitoring started\n");

    do {
      wait_ms(500);
      mntr_printf("monitoring...\n");



      events= xEventGroupWaitBits(eventFlags, MONITORING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    } while ( ( events & MONITORING_RUN_FLAG ) != 0 );
  }
}