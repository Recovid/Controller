/*
 * scheduling.cpp
 *
 *  Created on: 3 sept. 2019
 *      Author: Cyril
 */

#include "../mbed/scheduling.h"
#include <Arduino.h>
#include "threadManagerSingleton.h"
#include "HAL.h"
#include "Initialization.h"


int scheduling::cnt = 0;
Thread _polled(osPriorityIdle);
Thread _5msThread(osPriorityHigh);
Thread _1msThread(osPriorityRealtime);
mbed::Ticker sysTick;
//arduino interface
void setup() {
	 static scheduling myschedule;
	 static Initialization s_initOfFibre;
}



void Launch5msThread()
{
	_5msThread.signal_set(0x1);
}
void Launch1msThread()
{
	_1msThread.signal_set(0x2);
}

#define PROFILING
scheduling::scheduling()
{
	_1msThread.start(mbed::Callback<void()>(scheduler1ms));
	_5msThread.start(mbed::Callback<void()>(scheduler5ms));
	sysTick.attach(mbed::Callback<void()>(schedulerSysTick),0.001);
#ifdef PROFILING
	pinMode(port_D3, OUTPUT);
	pinMode(port_D2, OUTPUT);
#endif
}

scheduling::~scheduling() {
	// TODO Auto-generated destructor stub
}

void scheduling::schedulerSysTick()
{
	threadManagerSingleton::getInstance().DoThreadScheduling();
}

void scheduling::scheduler5ms()
{
	while (true)
	{
		// Signal flags that are reported as event are automatically cleared.
		ThisThread::flags_wait_any(0x1);
		#ifdef PROFILING
			digitalWrite(port_D2,HIGH);
		#endif
		threadManagerSingleton::getInstance().ScheduleSlowThread();
		#ifdef PROFILING
			digitalWrite(port_D2,LOW);
		#endif
	}
}
void scheduling::scheduler1ms()
{
	while (true)
	{
		// Signal flags that are reported as event are automatically cleared.
		ThisThread::flags_wait_any(0x2);
#ifdef PROFILING
		digitalWrite(port_D3,HIGH);
#endif
		threadManagerSingleton::getInstance().ScheduleFastThread();
#ifdef PROFILING
		digitalWrite(port_D3,LOW);
#endif
	}
}

void loop() {
	threadManagerSingleton::getInstance().SchedulePolledThread();
	delay(1);

 }
