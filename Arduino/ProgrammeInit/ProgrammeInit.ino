/*
   TProbotArduino V1.4

    Created on: 27 mars 2020
                Romain Delpoux (romain.delpoux@insa-lyon.fr)
*/


#include <Arduino.h>
#include <SimpleTimer.h>           // http://arduino.cc/playground/Code/SimpleTimer


#include "control.hpp"
//#include "encodeur.hpp"
//#include "moteur.hpp"
//#include "frame.hpp"


#define CONTROL_FREQ 100

Control * control;

void setup() {

  control = new Control(CONTROL_FREQ);

}

/* Fonction principale */
void loop() {
  control->timer_run();
}
