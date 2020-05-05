#include "controller.h"
#include "monitoring.h"
#include "breathing.h"
#include "platform.h"
#include "config.h"

void monitoring_run(void *args) {
  UNUSED(args)
  EventBits_t events;

  while(true) {
    mntr_printf("MNTR: Standby\n");
    events= xEventGroupWaitBits(ctrlEventFlags, MONITORING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    mntr_printf("MNTR: Started\n");

    do {
      wait_ms(500);
      mntr_printf("MNTR: ...\n");



      events= xEventGroupGetBits(ctrlEventFlags);
    } while ( ( events & MONITORING_RUN_FLAG ) != 0 );

    mntr_printf("MNTR: stopping\n");      

    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, MONITORING_STOPPED_FLAG);


  }
}