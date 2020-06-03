#include "common.h"
#include "config.h"
#include "hmi.h"
#include "controller.h"
#include "breathing.h"
#include "protocol.h"
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

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
static void hmi_run(void *args);

//----------------------------------------------------------
// Public variables
//----------------------------------------------------------
TaskHandle_t g_hmiTask;

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

bool hmi_init()
{
#ifdef DEBUG
    printf("HMI : Initializing\n");
#endif
    if (xTaskCreate(hmi_run, "HMI", HMI_TASK_STACK_SIZE, NULL, HMI_TASK_PRIORITY, &g_hmiTask) != pdTRUE)
    {
#ifdef DEBUG
        printf("HMI : Unable to create HMI Task\n");
#endif
        return false;
    }
#ifdef DEBUG
    printf("HMI : Initialized\n");
#endif
    return true;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

static void hmi_run(void *args)
{
    UNUSED(args)
    EventBits_t ctrlEvents;
    EventBits_t brthEvents;
#ifdef DEBUG
    uint32_t dbg_idx = 0;
#endif

    init_uart();

    while (true)
    {
        hmi_printf("HMI : Standby\n");
        ctrlEvents = xEventGroupWaitBits(g_controllerEvents, HMI_RUN_FLAG, pdFALSE, pdTRUE, portMAX_DELAY);
        hmi_printf("HMI : Started\n");

        // TODO: Clarify the synchronization sequence with the RaspberryPi !!!???
#ifndef NO_RASPI_REBOOT
        hmi_printf("HMI: Waiting 30s for RPi to start\n");
        wait_ms(30000); // Wait for the RPi to finish starting up
#endif
        send_INIT(get_init_str());

        uint32_t last_report_time = get_time_ms();
        bool update_brth_cycle_info = true;
        do
        {
            wait_ms(5);
            send_and_recv(); // TODO rework implem

            // waits for the BRTH_CYCLE_UPDATED.
            // TODO: Find a better solution
            brthEvents = xEventGroupGetBits(g_breathingEvents);

            if (BRTH_CYCLE_UPDATED == (brthEvents & BRTH_CYCLE_UPDATED))
            {
                // BRTH_CYCLE_UPDATED event received.
                
                // consume event
                xEventGroupClearBits(g_breathingEvents, BRTH_CYCLE_UPDATED);
                // send cycle info to HMI
                hmi_print("Updating breathing cycle info\n");
                send_RESP(get_cycle_EoI_ratio(),
                          get_cycle_FR_pm(),
                          get_cycle_VTe_mL(),
                          get_cycle_VMe_Lpm(),
                          get_cycle_Pcrete_cmH2O(),
                          get_cycle_Pplat_cmH2O(),
                          get_cycle_PEP_cmH2O());
            }

            if (25 <= (get_time_ms() - last_report_time))
            {
#ifdef DEBUG
                ++dbg_idx;
                if (40 == dbg_idx)
                {
                    dbg_idx = 0;
                    hmi_printf("HMI : updating [%d %d %d]\n", (uint16_t)read_Paw_cmH2O(), (uint16_t)read_Pdiff_Lpm(), (uint16_t)read_Vol_mL());
                }
#endif

                if( 0 != (brthEvents & (BRTH_CYCLE_PINHA | BRTH_CYCLE_PEXHA)))
                {
                    // Inhalation or Exhalation pause
                    send_DATA_X(read_Paw_cmH2O(), read_Pdiff_Lpm(), read_Vol_mL(), get_cycle_Pplat_cmH2O(), get_cycle_PEP_cmH2O());
                }
                else
                {
                    send_DATA(read_Paw_cmH2O(), read_Pdiff_Lpm(), read_Vol_mL());
                }

                last_report_time = get_time_ms();
            }



            // Check controller run signal.
            ctrlEvents = xEventGroupGetBits(g_controllerEvents);
        } while ((ctrlEvents & HMI_RUN_FLAG) != 0);

        hmi_printf("HMI: Stopping\n");
        wait_ms(200);
        xEventGroupSetBits(g_controllerEvents, HMI_STOPPED_FLAG);
    }
}
