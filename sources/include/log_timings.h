#ifndef LOG_TIMING_H
#define LOG_TIMING_H

/* The purpose of log timing is to measure:
 * - time of each interrupt/task (=proc)
 * - periodicity of procs
 * It is useful during dev only, as it consums a few resources
 *
 * Hardware required: TIM6
 * serial output    : uart2
 *
 * in 'native mode', no behavior
 */

/* if the following line is commented, there is no log
 * i.e, #define are expanded in nothing
 */
#define LOG_TIME

/* number of entries. 3 bytes per entry */
#define LOG_TIME_SIZE     1024

#define LOG_TIME_NB_TASKS 4

/* use msb of the id for start/stop */
#define LOG_TIME_EVENT_START 0x80
#define LOG_TIME_EVENT_STOP  0x00

/* id. 7 bits allowed */
#define LOG_TIME_TASK_BREATHING   0x0
#define LOG_TIME_TASK_MONITORING  0x1
#define LOG_TIME_TASK_CONTROLLER  0x2
#define LOG_TIME_TASK_HMI         0x3

#ifdef LOG_TIME
  #include <FreeRTOS.h> 
  #include <task.h> /*we need  TaskHandle_t */

  #ifdef native
    void log_time_initHw()      {}
    void log_time_start()       {}
    void log_time_event(int id) {UNUSED(id);}
    void log_time_dump()        {}
    int  log_time_full()        {return 0;}

    void log_time_init_task(const char *name, int id)  {UNUSED(name); UNUSED(id)   ;}
    void log_time_event_task(TaskHandle_t t,int start) {UNUSED(t)   ; UNUSED(start);}
  #else /* stm32 target => implementation in lowlevel/stm32f303/log_timings.h */
    void log_time_initHw();
    void log_time_start();
    void log_time_event(int id);
    void log_time_dump();
    int  log_time_full();

    void log_time_init_task(const char *name, int id);
    void log_time_event_task(TaskHandle_t t,int start);
  #endif /* native */
  
  /* should be called */
  #define LOG_TIME_INIT_TIM6_HARDWARE() log_time_initHw();
  #define LOG_TIME_START()              log_time_start();
  #define LOG_TIME_EVENT(id)            log_time_event(id);
  #define LOG_TIME_DUMP()               log_time_dump();
  #define LOG_TIME_FULL()               log_time_full() /* no ';' here */

  #define LOG_TIME_INIT_TASK(name,id)   log_time_init_task(name,id);
  #define LOG_TIME_EVENT_TASK(t,s)      log_time_event_task(t,s);

#else /* no log */

  #define LOG_TIME_INIT_TIM6_HARDWARE() 
  #define LOG_TIME_START()
  #define LOG_TIME_EVENT(id)
  #define LOG_TIME_DUMP()
  #define LOG_TIME_FULL()               0

  #define LOG_TIME_INIT_TASK(name,id)
  #define LOG_TIME_EVENT_TASK(t,s)

#endif /* LOG_TIME */

#endif
