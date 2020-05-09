#include "common.h"
#include "config.h"
#include "hmi.h"
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
#ifdef DEBUG
    uint32_t dbg_idx = 0;
#endif

    init_uart();

    hmi_printf("HMI : Started\n");

    uint32_t last_report_time = get_time_ms();
    do
    {
        wait_ms(5);

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
            send_DATA(read_Paw_cmH2O(), read_Pdiff_Lpm(), read_Vol_mL());
            last_report_time = get_time_ms();
        }

    } while (true);

}
