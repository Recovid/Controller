//include FreeRTOS
#include <FreeRTOS.h>
#include <task.h>

//Inclde STD
#include <stdlib.h>

//Inlcude Recovid
#include "tasks_recovid.h"
#include "controller.h"
#include "hardware_simulation.h"
#include "ihm_communication.h"

//
//
// DATA

float VolM_Lpm = 0.f;
float P_cmH2O  = 0.f;
float Vol_mL   = 0.f;

int   get_sensed_P_cmH2O()  { return P_cmH2O;  };
int   get_sensed_VolM_Lpm() { return VolM_Lpm; };
int   get_sensed_Vol_mL()   { return Vol_mL;   };



void sense_and_compute()
{
    static long last_sense_ms = 0;
    static long sent_DATA_ms = 0;

    P_cmH2O = read_Paw_cmH2O();
    // TODO float Patmo_mbar = read_Patmo_mbar();
    VolM_Lpm = read_Pdiff_Lpm(); // TODO Compute corrected QPatientSLM based on Patmo
    Vol_mL += (VolM_Lpm * 1000.) * labs(get_time_ms() - last_sense_ms)/1000./60.;
	if ((sent_DATA_ms+50 < get_time_ms())
		&& send_DATA(get_sensed_P_cmH2O(),get_sensed_VolM_Lpm(), get_sensed_Vol_mL(),0, 0 )) {
		sent_DATA_ms = get_time_ms();
	}

	last_sense_ms = get_time_ms();
}


void TaskSensing(void* task_param)  // This is a task.
{
  struct periodic_task* task = (struct periodic_task*) task_param;
  initTask(task);
  for (;;) // A Task shall never return or exit.
  {
    int missed_tick = sleepPeriodic(task);
	sense_and_compute();
  }
  
}
