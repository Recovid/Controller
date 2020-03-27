/*
 * robot.hpp
 *
 *  Created on: 11 sept. 2016
 *  Last Modification: 27 fev. 2017
 *      Author: Florian Bianco (florian.bianco@univ-lyon1.fr)
 *              Romain Delpoux (romain.delpoux@insa-lyon.fr)
 */

#ifndef CONTROL_HPP_
#define CONTROL_HPP_

#include <SimpleTimer.h>

enum {
	MODE_1,
	MODE_2,
};

class Control;

extern Control * p_control;

class Control {
private:

	int Id;

	/* Mode -> COMMANDE / REPONSE_INDICIELLE */
	int mode;

	/* State -> START / STOP*/
	int state;

	/* Timer */
	SimpleTimer timer;
	int freq; 		/* Hz */
	int periode; 	/* ms */
	int nbr_ticks;
	int nbr_ticks_impulse;
	int time_ms;
	int time_impulse;

	/* Com */


	void mode_1();
	void mode_2();
  
	

public:

	/* CTor - DTor */
	Control();
	Control(int freq);
	~Control();

	/* Methodes */
	void config(int freq);
	void set_mode(int mode);

	void start();
	void stop();

	bool is_running();

	void timer_run();

	static void run();

};

#endif /* CONTROL_HPP_ */
