/*
 * scheduling.cpp
 *
 *  Created on: 13 feb. 2020
 *      Author: Cyril
 */
#include "scheduling.h"
#include <Arduino.h>
#include "threadManagerSingleton.h"
//#include "HAL.h"
#include "Initialization.h"

#define PROFILING

void setup() {
   static scheduling myschedule;
   static Initialization s_initOfFibre;

   NVIC_SetPriority(TC0_IRQn, 6);
   NVIC_EnableIRQ(TC0_IRQn);
   NVIC_SetPriority(TC1_IRQn, 7);
   NVIC_EnableIRQ(TC1_IRQn);
}

void Launch5msThread()
{
    NVIC_SetPendingIRQ(TC1_IRQn);
}
void Launch1msThread()
{
    NVIC_SetPendingIRQ(TC0_IRQn);
}

scheduling::scheduling()
{
#ifdef PROFILING
  pinMode(PIN_A1, OUTPUT); //Defined for SAMD51J20A (Sparkfun thing plus)
  pinMode(PIN_A2, OUTPUT); //Defined for SAMD51J20A (Sparkfun thing plus)
#endif
}

scheduling::~scheduling() {
  // TODO Auto-generated destructor stub
}


// the loop function runs over and over again forever
void loop() {
  threadManagerSingleton::getInstance().SchedulePolledThread();
}

extern "C" int sysTickHook(void)
{
	threadManagerSingleton::getInstance().DoThreadScheduling();
	return 0;
}

extern "C" void TC0_Handler(void)
{
#ifdef PROFILING
    digitalWrite(PIN_A1,HIGH); //Defined for SAMD51J20A (Sparkfun thing plus)
#endif
    threadManagerSingleton::getInstance().ScheduleFastThread();
#ifdef PROFILING
    digitalWrite(PIN_A1,LOW); //Defined for SAMD51J20A (Sparkfun thing plus)
#endif
}

extern "C" void TC1_Handler(void)
{
#ifdef PROFILING
      digitalWrite(PIN_A2,HIGH); //Defined for SAMD51J20A (Sparkfun thing plus)
#endif
      threadManagerSingleton::getInstance().ScheduleSlowThread();
#ifdef PROFILING
      digitalWrite(PIN_A2,LOW); //Defined for SAMD51J20A (Sparkfun thing plus)
#endif
}

