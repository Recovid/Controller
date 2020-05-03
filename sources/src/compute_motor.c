#include "config.h"
#include <inttypes.h>
#include "compute_motor.h"

unsigned int compute_motor_press_christophe(
										unsigned int step_t_ns_init, 
										unsigned int acc_ns, 
										unsigned int step_t_min_ns, 
										unsigned int speed_down_t_ns, 
										unsigned int speed_down_t2_ps, 
										unsigned int step_t_ns_final,
										unsigned int dec_ns,
										unsigned int nb_steps, 
										uint32_t* steps_t_us)
{
	unsigned int step_total_us = 0;
	uint32_t pente_acc, debit_constant, pente_dec;
    const uint32_t max_steps = MIN(nb_steps, MAX_MOTOR_STEPS);
	for(unsigned int i = 0; i < max_steps; i++)
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
	for(unsigned int i = nb_steps; i < MAX_MOTOR_STEPS; i++) 
	{
		steps_t_us[i] = UINT32_MAX;
	}
	return step_total_us;
}


unsigned int compute_constant_motor_steps(uint32_t step_t_us, unsigned int nb_steps, uint32_t* steps_t_us)
{
    const uint32_t max_steps = MIN(nb_steps, MAX_MOTOR_STEPS);
	unsigned int total_time = 0;
    for(unsigned int i=0; i< MAX_MOTOR_STEPS; ++i) {
		if(i < max_steps) {
			   int pente_acc = (MOTOR_STEP_TIME_INIT) - ((MOTOR_ACC_COEF)*i);
				if(pente_acc > 0)
				   steps_t_us[i] = (uint32_t) MAX(step_t_us, (uint32_t)pente_acc);
				else
				   steps_t_us[i] = step_t_us;
		}
		else {
			   uint32_t pente_dec = ((MOTOR_ACC_COEF)* (i-max_steps));
			   steps_t_us[i] = MIN(UINT32_MAX,pente_dec);
		}
		total_time += step_t_us;
	}
	return total_time;
}


void print_christophe_header(
										unsigned int step_t_ns_init, 
										unsigned int acc_ns, 
										unsigned int step_t_min_ns, 
										unsigned int speed_down_t_ns, 
										unsigned int speed_down_t2_ps, 
										unsigned int step_t_ns_final,
										unsigned int dec_ns,
										int nb_steps)
{
	dbg_printf("step_t_ns_init %u\n", step_t_ns_init);
	dbg_printf("acc_ns %u\n", acc_ns);
	dbg_printf("step_t_min_ns %u\n", step_t_min_ns);
	dbg_printf("speed_down_t_ns %u\n", speed_down_t_ns);
	dbg_printf("speed_down_t2_ps %u\n", speed_down_t2_ps);
	dbg_printf("step_t_ns_final %u\n", step_t_ns_final);
	dbg_printf("dec_ns %u\n", dec_ns);
}


void print_steps(uint32_t* steps_t_us, unsigned int nb_steps)
{
    printf("Steps %u\n", nb_steps);
	for(unsigned int j=0; j < nb_steps; j+=1)
    {
		printf("%"PRIu32"\n", steps_t_us[j]);
    }
}
