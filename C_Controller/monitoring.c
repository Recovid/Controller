#include "recovid.h"
#include "configuration.h"
#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "lowlevel.h"



bool monitoring_init() {
  return true;
};

void monitoring_run(void *args) {
  UNUSED(args)
  EventBits_t events;

  printf("monitoring started\n");

  while(true) {

    events= xEventGroupWaitBits(controlFlags, RUN_MONITORING_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );

    while ( ( events & RUN_MONITORING_FLAG ) != 0 ) {
      printf(".\n");
      wait_ms(500);

      events= xEventGroupWaitBits(controlFlags, RUN_MONITORING_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    }
  }
}