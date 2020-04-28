#ifndef SENSING_H
#define SENSING_H

#include "lowlevel.h"
#include "platform.h"
#include "controller.h"
#include <stdint.h>

float get_sensed_P_cmH2O   ();

float get_sensed_VolM_Lpm  ();

float get_sensed_Vol_mL    ();
void reset_sensed_Vol_mL  ();

float get_sensed_VTi_mL    ();
float get_sensed_VTe_mL    ();

float get_sensed_Pcrete_cmH2O(); //!< \returns max sensed Paw during Insufflation
float get_sensed_Pplat_cmH2O (); //!< \returns sensed Paw at end of Plateau
float get_sensed_PEP_cmH2O   (); //!< \returns sensed Paw at end of Exhale
float get_sensed_Patmo_mbar  ();

float get_sensed_VMe_Lpm();

float get_last_sensed_ms();

void sensors_sample_VolM( int16_t read, uint32_t dt_ms);
bool sensors_sample_P(uint16_t read, uint16_t dt_us);

void compute_corrected_pressure();
void compute_corrected_flow_volume();

uint32_t compute_samples_average_and_latency_us(); //!< For test purposes

//! \returns the total move time
uint32_t compute_motor_steps_and_Tinsu_ms(float flow_Lps, float vol_mL, uint16_t* steps_t_us);

//! \returns the total move time
uint32_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps, uint16_t* steps_t_us);


//! \remark Do not actually read sensors (this is done by interrupts), but use their data to compute values used by others
void sense_and_compute(RespirationState state);

extern  uint16_t steps_t_us[MOTOR_MAX];
extern  uint16_t last_step;

#define SAMPLING_SIZE	(300)

extern float samples_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms
extern uint16_t samples_Q_Lps_dt_us[SAMPLING_SIZE];
extern float average_Q_Lps[SAMPLING_SIZE]; // > max Tinsu_ms


extern uint16_t samples_P[SAMPLING_SIZE];
extern uint16_t samples_P_dt_us[SAMPLING_SIZE];

#ifndef NTESTS
bool TEST_SENSING();
#endif

#endif // SENSING_H
