#ifndef __MONITORING_H__
#define __MONITORING_H__

#include "common.h"

extern TaskHandle_t     g_monitoringTask;

bool monitoring_init();

#endif