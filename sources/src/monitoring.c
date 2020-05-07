#include "common.h"
#include "config.h"
#include "monitoring.h"
#include "controller.h"
#include "breathing.h"
#include "platform.h"


//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
static TaskHandle_t     monitoringTask;

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void monitoring_run(void *args);

//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

bool monitoring_init() {
#ifdef DEBUG
    printf("MNTR: Initializing\n");
#endif

    if (xTaskCreate(monitoring_run, "Monitoring", MONITORING_TASK_STACK_SIZE, NULL, MONITORING_TASK_PRIORITY, &monitoringTask) != pdTRUE)
    {
#ifdef DEBUG
        printf("MNTR: Unable to create monitoringTask\n");
#endif
        return false;
    }
#ifdef DEBUG
    printf("MNTR: Initialized\n");
#endif
    return true;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void monitoring_run(void *args) {
  UNUSED(args)
  EventBits_t events;

  while(true) {
    mntr_printf("MNTR: Standby\n");
    events= xEventGroupWaitBits(g_controllerEvents, MONITORING_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    mntr_printf("MNTR: Started\n");

    do {
      wait_ms(500);
      mntr_printf("MNTR: ...\n");



      events= xEventGroupGetBits(g_controllerEvents);
    } while ( ( events & MONITORING_RUN_FLAG ) != 0 );

    mntr_printf("MNTR: stopping\n");      

    wait_ms(200);
    xEventGroupSetBits(g_controllerEvents, MONITORING_STOPPED_FLAG);


  }
}