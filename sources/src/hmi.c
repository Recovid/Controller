#include "controller.h"
#include "hmi.h"
#include "breathing.h"
#include "protocol.h"
#include "platform.h"
#include "log_timings.h"


void hmi_run(void *args) {
  UNUSED(args)
  EventBits_t ctrlEvents;
  EventBits_t brthEvents;
#ifdef DEBUG
  uint32_t dbg_idx=0;
#endif  
  LOG_TIME_EVENT(LOG_TIME_EVENT_START | LOG_TIME_TASK_HMI)

  init_uart();

  while(true) {
    hmi_printf("HMI: Standby\n");
    LOG_TIME_EVENT(LOG_TIME_EVENT_STOP  | LOG_TIME_TASK_HMI)
    ctrlEvents= xEventGroupWaitBits(ctrlEventFlags, HMI_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    LOG_TIME_EVENT(LOG_TIME_EVENT_START | LOG_TIME_TASK_HMI)
    hmi_printf("HMI: Started\n");



    // TODO: Clarify the synchronization sequence with the RaspberryPi !!!???
    //hmi_printf("HMI: Waiting 40s for RPi to start\n");
    //wait_ms(40000);             // Wait 30 seconds for the RPi to finish starting up
    send_INIT(get_init_str());

    uint32_t last_report_time= get_time_ms();
    bool update_brth_cycle_info=true;
    do {
      send_and_recv(); // TODO rework implem 
      
      if(25 <= (get_time_ms()-last_report_time)) {
#ifdef DEBUG     
        ++dbg_idx;   
        if(40 == dbg_idx) { dbg_idx=0; hmi_printf("updating hmi [%d %d %d]\n", (uint16_t )read_Paw_cmH2O(), (uint16_t )read_Pdiff_Lpm(), (uint16_t)read_Vol_mL()); }
#endif
        send_DATA(read_Paw_cmH2O(), read_Pdiff_Lpm(), read_Vol_mL());
        last_report_time= get_time_ms();
      }

	  LOG_TIME_EVENT(LOG_TIME_EVENT_STOP | LOG_TIME_TASK_HMI)
      // waits for the BRTH_RESULT_UPDATED.
      // TODO: Find a better solution 
      brthEvents= xEventGroupWaitBits(brthCycleState, BRTH_RESULT_UPDATED, pdTRUE, pdTRUE, 5/portTICK_PERIOD_MS);
      
	  LOG_TIME_EVENT(LOG_TIME_EVENT_START | LOG_TIME_TASK_HMI)
      if(BRTH_CYCLE_FINISHED == (brthEvents & BRTH_CYCLE_FINISHED)) {
          // BRTH_CYCLE_FINISHED event received.
          // send cycle info to HMI
          hmi_print("Updating breathing cycle info\n");
          send_RESP(get_breathing_EoI_ratio(),
                    get_breathing_FR_pm(),
                    get_breathing_VTe_mL(),
                    get_breathing_VMe_Lpm(),
                    get_breathing_Pcrete_cmH2O(),
                    get_Pplat_cmH20(),
                    get_PEP_cmH2O());
            
      }

      // Check controller run signal.
      ctrlEvents= xEventGroupGetBits(ctrlEventFlags);
    } while ( ( ctrlEvents & HMI_RUN_FLAG ) != 0 );

    hmi_printf("HMI: Stopping\n");
    wait_ms(200);
    xEventGroupSetBits(ctrlEventFlags, HMI_STOPPED_FLAG);

  }
}
