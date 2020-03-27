/*
   robot.cpp

    Created on: 11 sept. 2016
    Last Modification: 27 fev. 2017
        Author: Florian Bianco (florian.bianco@univ-lyon1.fr)
                Romain Delpoux (romain.delpoux@insa-lyon.fr)
*/


#include <Arduino.h>

#include "control.hpp"
#include "types.h"

#define DEFAULT_FREQ 25
#define FREQ_MIN 2
#define FREQ_MAX 100

#define CONSIGNE_VEL_MIN 		0
#define CONSIGNE_VEL_DEFAULT 	0
#define CONSIGNE_VEL_MAX 		300

#define DEFAULT_AUTO_COEF (1.0f)

Control * p_control = NULL;

/* CTor - DTor */
Control::Control() {
  this->config(DEFAULT_FREQ);
}

Control::Control(int freq) {

  this->config(freq);

}

Control::~Control() {
  this->state = STATE_STOP;
  this->stop();

}

void Control::config(int freq) {

  this->mode = MODE_1;

  /* Init Serial for Com */
  Serial1.begin(57600); /* Raspberry Com */
  Serial.begin(115200); /* Debug Com */

  Serial1.flush();
  Serial.flush();

  /* Delay */
  Serial.print("\n");
  Serial.print("Control Init : ");
  
  /* Timer */
  if (freq < FREQ_MIN) {
    this->freq = FREQ_MIN;
  } else {
    if (freq > FREQ_MAX) {
      this->freq = FREQ_MAX;
    } else {
      this->freq = freq;
    }
  }

  p_control = this;
  this->nbr_ticks = 0;
  this->periode = 1000 / this->freq;
  this->timer.setInterval(this->periode, this->run);

  this->nbr_ticks = 0;
  this->nbr_ticks_impulse = 0;


  /* Delay */
  delay(1000);
  Serial.print("OK\n");

  Serial.print("Freq = ");
  Serial.print(this->freq);
  Serial.print("\n");

  Serial.print("Periode = ");
  Serial.print(this->periode);
  Serial.print("\n");

  /* Finish */
  this->state = STATE_OK;
}

/* Methodes */
void Control::start() {

  this->state = STATE_RUN;
  //Serial.print(" OK\n");
}

void Control::stop() {

  this->state = STATE_STOP;
}

bool Control::is_running() {
  bool retval = false; 

  if (this->state == STATE_RUN) {
    retval = true;
  }

  return retval;
}

void Control::set_mode(int mode) {

  this->mode = mode;

}

void Control::timer_run() {
  this->timer.run();
}

void Control::run() {
  bool retval;

  p_control->nbr_ticks++;
  p_control->time_ms = p_control->nbr_ticks * p_control->periode;


  p_control->start();
  p_control->mode = MODE_1;

  if (p_control->is_running()) {

    switch (p_control->mode) {
      case MODE_1:
        p_control->mode_1();
        break;
    }




  }

}

void Control::mode_1() {
  Serial.println("hello");
}

void Control::mode_2() {


}
