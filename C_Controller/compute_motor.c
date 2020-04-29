#include <stdint.h>
#include "lowlevel.h"
#include "sensing.h"

//! \returns estimated latency (µs) between motor motion and Pdiff readings
uint32_t compute_samples_average_and_latency_us()
{
    bool unusable_samples = true;
    uint32_t latency_us = 0;
    float sum = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(samples_Q_Lps) && i < COUNT_OF(average_Q_Lps); ++i) {
        if (unusable_samples) {
            if (samples_Q_Lps[i] > CALIB_UNUSABLE_PDIFF_LPS) {
                unusable_samples = false;
            }
            else {
                latency_us += SAMPLES_T_US;
            }
        }
        // Sliding average over CALIB_PDIFF_SAMPLES_MIN samples at same index
        sum += samples_Q_Lps[i];
        if (i >= CALIB_PDIFF_SAMPLES_MIN) {
            sum -= samples_Q_Lps[i-CALIB_PDIFF_SAMPLES_MIN];
        }




        if (i >= get_samples_Q_index_size() && (i >= 1 + CALIB_PDIFF_SAMPLES_MIN/2) ) {
            average_Q_Lps[ (i-CALIB_PDIFF_SAMPLES_MIN/2) ] = average_Q_Lps[(i-CALIB_PDIFF_SAMPLES_MIN/2)- 1];
        }
        else if ((CALIB_PDIFF_SAMPLES_MIN/2 ) <= i && (i<= get_samples_Q_index_size() - (CALIB_PDIFF_SAMPLES_MIN/2)) ) {
            average_Q_Lps[(i-CALIB_PDIFF_SAMPLES_MIN/2)] = sum / CALIB_PDIFF_SAMPLES_MIN;
        }
        else {
			//if (i >= 1 + CALIB_PDIFF_SAMPLES_MIN/2) {
			//	light_green(On);
			//}
            average_Q_Lps[i] = 0.f;
        }
    }
    return latency_us;
}


float corrections[MOTOR_MAX];
//! \returns last steps_t_us motion to reach vol_mL
uint32_t compute_motor_steps_and_Tinsu_ms(float desired_flow_Lps, float vol_mL, uint16_t* steps_t_us)
{
    uint32_t latency_us = compute_samples_average_and_latency_us(); // removes Pdiff noise and moderates flow adjustments over cycles
//	sprintf(buf, "latency : %d\n", latency_us);
//	hardware_serial_write_data(buf, strlen(buf)); 
//	sprintf(buf, "get_samples_Q_index_size : %d\n", get_samples_Q_index_size());
//	hardware_serial_write_data(buf, strlen(buf)); 

    uint32_t last_step = 0;
    float Tinsu_us = 0.f;
    for (uint16_t i=0 ; i<COUNT_OF(steps_t_us) ; ++i) {
        uint16_t Q_index = (Tinsu_us + latency_us) / SAMPLES_T_US;
        const uint16_t average_Q_index = MIN(get_samples_Q_index_size()-(1+CALIB_PDIFF_SAMPLES_MIN/2),Q_index);
//		if(i % 100 == 0) {
//			sprintf(buf, "Q_Index : %d\n", Q_index);
//			hardware_serial_write_data(buf, strlen(buf)); 
//		}
        const float actual_Lps = average_Q_Lps[average_Q_index];
        float correction;
		if(CHECK_FLT_EQUALS(0.0f, actual_Lps) || CHECK_FLT_EQUALS(0.0f, desired_flow_Lps))
		{
			light_red(On);
			correction = 1;
		}
		else {
			correction = MAX(0.5, MIN(2.0f, desired_flow_Lps / actual_Lps));
			light_red(Off);
		}
		corrections[i] = correction;

        const float new_step_t_us = MAX(MOTOR_STEP_TIME_US_MIN, ((float)steps_t_us[i]) / correction);
        const float vol = Tinsu_us/1000/*ms*/ * desired_flow_Lps;
		Tinsu_us += new_step_t_us;
		if(new_step_t_us < MOTOR_STEP_TIME_INIT - (A*i)) {
			steps_t_us[i] = MOTOR_STEP_TIME_INIT - (A*i);
			last_step = i;
		}
		else if (vol < 1.0f * vol_mL) { // actual Q will almost always be lower than desired TODO +10%
			steps_t_us[i] = new_step_t_us;
			last_step = i;
		}
		else {
			steps_t_us[i] = MIN(UINT16_MAX, new_step_t_us + (A)*(i-last_step));
		}
//#ifndef NTESTS
//            DEBUG_PRINTF("t_us=%f steps_t_us=%d vol=%f", Tinsu_us, steps_t_us[i], vol);
//#endif
    }
//    DEBUG_PRINTF("Tinsu predicted = %d ms", (uint32_t)(Tinsu_us/1000));
    return last_step;
}


unsigned int compute_motor_press_christophe(
										unsigned int step_t_ns_init, 
										unsigned int acc_ns, 
										unsigned int step_t_min_ns, 
										unsigned int speed_down_t_ns, 
										unsigned int speed_down_t2_ps, 
										unsigned int step_t_ns_final,
										unsigned int dec_ns,
										int nb_steps, 
										uint16_t* steps_t_us)
{
	unsigned int step_total_us = 0;
	unsigned int pente_acc, debit_constant, pente_dec;
    const uint16_t max_steps = MIN(nb_steps, MOTOR_MAX);
	for(int i = 0; i < max_steps; i++)
	{

		//Pas acceleration
		// =SI(N°pas<Pas_temps_initial/Acc_temps,ARRONDI.INF((Pas_temps_initial-Acc_temps*N°pas),0)/1000,0)
		if( i< step_t_ns_init / acc_ns ) {
			pente_acc = (step_t_ns_init - (acc_ns*i)) / 1000;
		}
		else {
			pente_acc = 0;
		}

		//Accélération phase
		//=ARRONDI.INF((Pas_temps_min+(N°pas*Ralent_débit_lin)+(N°pas^2*Ralent_débit_2/1000))/1000,0)
		debit_constant =  ((step_t_min_ns + i*speed_down_t_ns + (i*i*speed_down_t2_ps/1000)) /1000);
		
		//Fin
		//=SI(ET(H17<=N_Pas_fin,(H17>(N_Pas_fin-(Pas_temps_fin-Pas_temps_min)/Décc_temps))),
		//	ARRONDI.INF((Pas_temps_fin/1000)+((H17-N_Pas_fin)*(Décc_temps/1000)),0)
		//else
		//	,0)
		if(nb_steps - ((step_t_ns_final - step_t_min_ns)/dec_ns)  < i &&  i < nb_steps)
			pente_dec = (step_t_ns_final / 1000)+  ((i-nb_steps)*(dec_ns/1000));
		else
			pente_dec = 0;

		steps_t_us[i] = MAX(pente_acc, MAX(debit_constant, pente_dec));
		step_total_us += steps_t_us[i];
	}
	for(int i = nb_steps; i < MOTOR_MAX; i++) 
	{
		steps_t_us[i] = UINT16_MAX;
	}
	return step_total_us;
}

uint32_t compute_constant_motor_steps(uint16_t step_t_us, uint16_t nb_steps, uint16_t* steps_t_us)
{
    const uint16_t max_steps = MIN(nb_steps, MOTOR_MAX);
	uint32_t total_time = 0;
    for(unsigned int i=0; i<MOTOR_MAX; ++i) {
        if(i < max_steps) {
               steps_t_us[i] = MAX(step_t_us, MOTOR_STEP_TIME_INIT - (A)*i);
        }
        else {
               steps_t_us[i] = MIN(UINT16_MAX, step_t_us + (A)*(i-max_steps));
        }
		total_time += step_t_us;
	}
    return max_steps;
}
