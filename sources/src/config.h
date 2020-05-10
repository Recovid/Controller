#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "common.h"




#ifndef NDEBUG
#define DEBUG
#endif


#ifdef DEBUG
#define DEBUG_CONTROLLER
#define DEBUG_WATERMARK
#define DEBUG_BREATHING
//#define DEBUG_MONITOR
//#define DEBUG_HMI
#endif





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






// -----------------------------------
// Specific to simulator.
// voided out for now

#ifdef DEBUG
#define DEBUG_PRINTF(_fmt, ...) ((void)0) //(fprintf(stderr, _fmt "\n", __VA_ARGS__))
#define DEBUG_PRINT( _msg     ) ((void)0) //(fprintf(stderr,    "%s\n",      (_msg)))
#define STDERR_PRINTF(_fmt, ...) ((void)0)//(fprintf(stderr, _fmt "\n", __VA_ARGS__))
#define STDERR_PRINT( _msg     ) ((void)0)//(fprintf(stderr,    "%s\n",      (_msg)))
#else
#define DEBUG_PRINTF(_fmt, ...) ((void)0)
#define DEBUG_PRINT( _msg     ) ((void)0)
#define STDERR_PRINTF(_fmt, ...) ((void)0)
#define STDERR_PRINT( _msg     ) ((void)0)
#endif

#ifdef NDEBUG
#define ASSERT_EQUALS(_expected,_evaluated) ((void)0)
#else
#define ASSERT_EQUALS(_expected,_evaluated) ((void)((!!(_expected==_evaluated)) || \
  (_assert("Expected:" #_expected " " #_evaluated,__FILE__,__LINE__),0)))
#endif

#ifdef NDEBUG
#define ASSERT_FALSE(_reason) ((void)0)
#else
#define ASSERT_FALSE(_reason) ((void)(_assert(_reason,__FILE__,__LINE__),0))
#endif

#define CHECK_RANGE(_min,_evaluated,_max) ((!!(((_min)<=(_evaluated)) && (_evaluated)<=(_max))) || \
  (DEBUG_PRINTF("Expected [" #_min ".." #_max "] " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define CHECK_FLT_EQUALS(_expected,_evaluated) CHECK_RANGE((_expected)-.06f,(_evaluated),(_expected)+.06f)

#define CHECK_EQUALS(_expected,_evaluated) ((!!((_expected)==(_evaluated))) || \
  (DEBUG_PRINTF("Expected:" #_expected " " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define CHECK(_predicate) ((!!(_predicate)) || \
  (DEBUG_PRINTF("Failed:" #_predicate " at:" __FILE__ "(%d)",__LINE__),false))

#define TEST_RANGE(_min,_evaluated,_max) ((!!(((_min)<=(_evaluated)) && (_evaluated)<=(_max))) || \
  (STDERR_PRINTF("Expected [" #_min ".." #_max "] " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define TEST_FLT_EQUALS(_expected,_evaluated) TEST_RANGE((_expected)-0.06f,(_evaluated),(_expected)+0.06f) // 1st digit correct

#define TEST_EQUALS(_expected,_evaluated) ((!!((_expected)==(_evaluated))) || \
  (STDERR_PRINTF("Expected:" #_expected " " #_evaluated ":%.1f at:" __FILE__ "(%d)",(double)(_evaluated),__LINE__),false))

#define TEST(_predicate) ((!!(_predicate)) || \
  (STDERR_PRINTF("Failed:" #_predicate " at:" __FILE__ "(%d)",__LINE__),false))

#define TEST_ASSUME(_predicate) if (!TEST(_predicate)) return false









#endif
