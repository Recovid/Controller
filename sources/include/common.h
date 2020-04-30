#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <event_groups.h>

#include "config.h"


#define COUNT_OF(_array) (sizeof(_array)/sizeof(_array[0]))

#ifndef UNUSED
# define UNUSED(_expression) (void)_expression;
#endif
#define STRINGIZE1(s) #s
#define STRINGIZE(s) STRINGIZE1(s)

#define BOUNDED(_a,_b) ((_a)>(_b) ? (_a) : (_b))
#define MAX(_a,_b) ((_a)>(_b) ? (_a) : (_b))
#define MIN(_a,_b) ((_a)<(_b) ? (_a) : (_b))
#define SIGN(_a)   ((_a)<  0  ?  -1  :   1 )





#ifdef DEBUG
extern SemaphoreHandle_t dbgMutex;
#define dbg_print(_fmt)       { xSemaphoreTake(dbgMutex,  portMAX_DELAY); printf(_fmt); xSemaphoreGive(dbgMutex); }
#define dbg_printf(_fmt,...)  { xSemaphoreTake(dbgMutex,  portMAX_DELAY); printf(_fmt, ##__VA_ARGS__); xSemaphoreGive(dbgMutex); }
#else
#define dbg_print(_fmt)       ((void)0)
#define dbg_printf(_fmt,...)  ((void)0)
#endif



#ifdef DEBUG_HMI
#define hmi_print(_fmt)       dbg_print(_fmt)
#define hmi_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define hmi_print(_fmt)       ((void)0)
#define hmi_printf(_fmt,...)  ((void)0)
#endif

#ifdef DEBUG_CONTROLLER
#define ctrl_print(_fmt)       dbg_print(_fmt)
#define ctrl_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define ctrl_print(_fmt)       ((void)0)
#define ctrl_printf(_fmt,...)  ((void)0)
#endif

#ifdef DEBUG_MONITOR
#define mntr_print(_fmt)       dbg_print(_fmt)
#define mntr_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define mntr_print(_fmt)      ((void)0)
#define mntr_printf(_fmt,...)  ((void)0)
#endif

#ifdef DEBUG_BREATHING
#define brth_print(_fmt)       dbg_print(_fmt)
#define brth_printf(_fmt,...)  dbg_printf(_fmt,##__VA_ARGS__)
#else
#define brth_print(_fmt)       ((void)0) 
#define brth_printf(_fmt,...)  ((void)0)
#endif

#endif





