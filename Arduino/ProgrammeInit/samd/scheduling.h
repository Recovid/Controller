/*
 * scheduling.h
 *
 *  Created on: 3 sept. 2019
 *      Author: Cyril
 */

#ifndef OSAL_THREADING_SCHEDULING_H_
#define OSAL_THREADING_SCHEDULING_H_
#include <Arduino.h>

class scheduling {
public:

	/* Default Constructor */
	scheduling();

	/* Deconstructor */
	virtual ~scheduling();
	static void schedulerSysTick();
	static void scheduler5ms();
	static void scheduler1ms();

private:
	// stop copy constructor and = being called erroneously
	// do not implement
	scheduling(const scheduling &other);
	scheduling& operator=(const scheduling&);
    static int cnt;


#ifdef UNIT_TEST
    friend class scheduler_test;
#endif
#ifdef UNIT_TEST
    friend class scheduling_test;
#endif
};

#endif /* OSAL_THREADING_SCHEDULING_H_ */
