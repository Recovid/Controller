#ifndef __HMI_H__
#define __HMI_H__

#include "common.h"


// HMI FreeRTOS stuff
extern TaskHandle_t g_hmiTask;
bool hmi_init();


#endif