#ifndef SENSING_H
#define SENSING_H

#include "platform.h"
#include "controller.h"

float get_sensed_P_cmH2O ();
float get_sensed_VolM_Lpm();
float get_sensed_VTi_mL  ();
float get_sensed_VTe_mL  ();

float get_sensed_Pcrete_cmH2O(); //!< \returns max sensed Paw during Insufflation
float get_sensed_Pplat_cmH2O (); //!< \returns sensed Paw at end of Plateau
float get_sensed_PEP_cmH2O   (); //!< \returns sensed Paw at end of Exhale

float get_sensed_VMe_Lpm();

float get_last_sensed_ms();

uint16_t get_usable_pdiff_readings_start(float time_step_sec);

uint32_t compute_samples_average_and_latency_us(); //!< For test purposes

    //! \returns last steps_t_us motion to reach vol_mL
uint32_t compute_motor_steps_and_Tinsu_ms(float flow_Lps, float vol_mL);

//! \remark Do not actually read sensors (this is done by interrupts), but use their data to compute values used by others
void sense_and_compute(RespirationState state);

#ifndef NTESTS
bool TEST_SENSING();
#endif

#endif // SENSING_H
