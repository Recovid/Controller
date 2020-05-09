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




#endif





