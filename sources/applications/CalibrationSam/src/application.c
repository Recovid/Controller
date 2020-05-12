#include "application.h"
#include "platform.h"
#include "calibration.h"
#include "hmi.h"


//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

#ifdef DEBUG
SemaphoreHandle_t dbgMutex;
#endif

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------

void application_main()
{

#ifdef DEBUG
    printf("Starting Calibration\n");

    dbgMutex = xSemaphoreCreateBinary();
    if(NULL == dbgMutex) {
        printf("Unable to create dbgMutex\n");
        return;
    }
    xSemaphoreGive(dbgMutex);
#endif

    if(hmi_init() == false) 
    {
        return;
    }

    if(calibration_init() == false) 
    {
        return;
    }

#ifdef DEBUG
    printf("Starting scheduler\n");
#endif

    // start scheduler
    vTaskStartScheduler();

    // We should never get here
}
