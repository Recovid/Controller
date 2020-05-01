#include "controller.h"
#include "monitoring.h"
#include "breathing.h"
#include "platform.h"
#include "log_timings.h"



void monitoring_run(void *args) {
  UNUSED(args)
  EventBits_t events;
  LOG_TIME_EVENT(LOG_TIME_EVENT_START | LOG_TIME_TASK_MONITORING)

  while(true) {
    mntr_printf("MNTR: Standby\n");
    LOG_TIME_EVENT(LOG_TIME_EVENT_STOP  | LOG_TIME_TASK_MONITORING)
    events= xEventGroupWaitBits(ctrlEventFlags, MONITORING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    LOG_TIME_EVENT(LOG_TIME_EVENT_START | LOG_TIME_TASK_MONITORING)
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
