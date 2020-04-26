#include "recovid.h"
#include "hmi.h"
#include "controller.h"
#include "protocol.h"



void hmi_run(void *args) {
  UNUSED(args)
  EventBits_t events;
#ifdef DEBUG
  uint32_t dbg_idx=0;
#endif  

  init_uart();

  while(true) {
    hmi_printf("HMI - STANDBY\n");
    events= xEventGroupWaitBits(eventFlags, HMI_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    hmi_printf("HMI started\n");



    // TODO: Clarify the synchronization sequence with the RaspberryPi !!!???
   hmi_printf("HMI waiting 40s for RPi to start\n");
   wait_ms(40000);             // Wait 30 seconds for the RPi to finish starting up
   //send_INIT(get_init_str());

    uint32_t last_report_time= get_time_ms();
    do {
      send_and_recv(); // TODO rework implem for a more deterministic execution time
      
      if(25 <= (get_time_ms()-last_report_time)) {
#ifdef DEBUG     
        ++dbg_idx;   
        if(40 == dbg_idx) { dbg_idx=0; hmi_printf("updating hmi [%d %d %d]\n", (uint16_t )read_Paw_cmH2O(), (uint16_t )read_Pdiff_Lpm(), (uint16_t)read_Vol_mL()); }
#endif
        send_DATA(read_Paw_cmH2O(), read_Pdiff_Lpm(), read_Vol_mL());
        last_report_time= get_time_ms();
      }

      wait_ms(5); // we could opt for a fix period loop. But what would be the benefit ??

      events= xEventGroupWaitBits(eventFlags, HMI_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY );
    } while ( ( events & HMI_RUN_FLAG ) != 0 );
  }
}