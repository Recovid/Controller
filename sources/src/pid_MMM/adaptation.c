#include "breathing.h"
#include "common.h"
#include "compute_motor.h"
#include "config.h"
#include "adaptation.h"
#include "platform.h"
#include "platform_defs.h"
#include "PID.h"
#include <inttypes.h>
// Other dependencies
#include <stdint.h>
#include <math.h>

//----------------------------------------------------------
// Private defines
//----------------------------------------------------------

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
static bool l_adaptation_initialized = false;
static const uint32_t T_omegamax_ms      = 200;
static float flow_samples_motor_steps[MOTOR_MAX_STEPS];
static const uint32_t Sa_max_pas_par_s2 = 45000;
static const uint32_t Sd_max_pas_par_s2 = 3E5;
static const float    AxC50R5           = 0.000204 ;  //constante courbe Vol(mL) de pas
static const float    PxC50R5           = 2.1605 ;    //puissance sur le numéro de pas Vol (pas)
static const float    S4H2O_cm          = 70;         //limite haute Pcrete pour selecteur moteur S4 en position C (85% du courant max)

static float freq_steps_per_slices[NB_SLICES];
static float error_per_slices[NB_SLICES];

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
		   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps);                   //output: nb_steps

void convert_freq_to_motor_steps(float* freq_steps_per_slices,  //input: freq steps on each slices
				 uint32_t nb_freq_slices,	//input: freq of slices
				 uint32_t* t_motor_step_us,     //output: Array of steps time [MAX_MOTOR_STEPS]
                                 uint32_t* nb_steps);           //output: nb_steps

void convert_motor_steps_to_freq(uint32_t* t_motor_step_us,     //input: Array of steps time [MAX_MOTOR_STEPS]
                                 uint32_t nb_steps,            //input: nb_steps
				 uint32_t  nb_slices,	   	//input: nb slices 
				 float* freq_steps_per_slices); //output: array of freq per slices

void convert_samples_t_to_samples_steps(float*    flow_samples_Lpm,         //array of samples
					uint32_t  flow_samples_count,       //nb samples in array
					uint32_t  flow_samples_period_ms,   //delta T between samples
					uint32_t* motor_steps_us,           //Motor steps to feat the samples against
					uint32_t  nb_steps, 	            //Size of the array
					float* flow_samples_motor_steps);//output a array of samples feated on motor steps

static void compute_max_slope(uint32_t nb_steps_slices,
                              uint32_t nb_steps_per_slices,
                              float*   max_slope_acceleration,
                              float*   max_slope_deceleration);

static void compute_slice_flow_error(uint32_t nb_slices,
                                     uint32_t nb_steps_per_slices,
                                     float    target_Flow_Lpm,
                                     float*   flow_samples_motor_steps,
                                     float*   slice_flow_error);


//----------------------------------------------------------
// Public variables
//----------------------------------------------------------

//----------------------------------------------------------
// Public functions
//----------------------------------------------------------
void reset_adaptation()
{
  l_adaptation_initialized = false;
}

// Initialize the adaptation engine.
// Called before the recovid starts the breathing cycles.

// Compute the motor step table based on targeted Vmax, VT, and previous flow samples.
// fills the motor step table and return the number of steps.

void print_samples(float* samples,
		   uint32_t nb_samples)
{
  for(uint32_t i = 0; i < nb_samples; i++)
  {
    printf("sample[%ld] = %d\n", i, (int) samples[i]);
  }
}

void print_steps(uint32_t* steps_t_us, unsigned int nb_steps)
{
    printf("Steps %u\n", nb_steps);
    for(unsigned int j=0; j < nb_steps; j+=1)
    {
      printf("%"PRIu32"\n", steps_t_us[j]);
    }
}
uint32_t adaptation(
    float       target_VT_mL,
    float       target_Flow_Lpm,
    uint32_t    flow_samples_period_ms, 
    uint32_t    flow_samples_count, 
    float*      flow_samples_Lpm, 
    uint32_t    motor_max_steps, 
    uint32_t*   motor_steps_us) 
{
  uint32_t nb_steps = motor_max_steps;

  if(l_adaptation_initialized != true)
  {
    compute_seed(target_Flow_Lpm, target_VT_mL, motor_steps_us, &nb_steps);
    l_adaptation_initialized = true;
		
	// Structure to strore PID data and pointer to PID structure
	struct pid_controller ctrldata_Volume, ctrldata_Debit[NB_SLICES];
	pid_H pid_FlowRate[NB_SLICES];
	float input_volume, flow_setpoint, objective_volume;
	float Flowrate_Slice[NB_SLICES];
	float Freq_Steps[NB_SLICES];
	
	// Prepare PID Volume controller for operation, set limits and enable controller
	pid_H pid_Volume = pid_create(&ctrldata_Volume, &input_volume, &flow_setpoint, &objective_volume, VOL_KP, VOL_KI, VOL_KD);
	pid_limits(pid_Volume, 0, 200);
	pid_auto(pid_Volume);
	
	// Prepare PID Flowrate controller for operation, set limits and enable controller
	for(int slice_index=0; slice_index<NB_SLICES; slice_index++)
	{
		pid_FlowRate[slice_index] = pid_create(&ctrldata_Debit[slice_index], &Flowrate_Slice[slice_index], &Freq_Steps[slice_index], &flow_setpoint, FLOW_KP, FLOW_KI, FLOW_KD);
		pid_limits(pid_FlowRate[slice_index], 0, 200);
		pid_auto(pid_FlowRate[slice_index]);
	}
	
	
    return nb_steps;
  }
  //translate sample for time to steps
  // => F1

  convert_motor_steps_to_freq(motor_steps_us, nb_steps, NB_SLICES, freq_steps_per_slices);

  convert_samples_t_to_samples_steps(flow_samples_Lpm, flow_samples_count, flow_samples_period_ms, motor_steps_us, motor_max_steps, flow_samples_motor_steps);

  compute_slice_flow_error(NB_SLICES, nb_steps/NB_SLICES, target_Flow_Lpm, flow_samples_motor_steps, error_per_slices);
  //convert_freq_to_motor_steps(new_freq, NB_SLICES, motor_step_us, &nb_stepseps);


  return nb_steps;
}

//----------------------------------------------------------
// Private functions
//----------------------------------------------------------

void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
                   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps)                    //output: nb_steps
{
  //for(uint32_t i =0; i < MOTOR_MAX_STEPS/2; i++)
  //{
  //  t_motor_step_us[i] = 800;
  //}
  //*nb_steps = MOTOR_MAX_STEPS/2;

  //Constantes initiales
  const uint32_t Vmax_tour_par_min    = 250;        //Vmax_tour_par_min
  const uint32_t pas_par_tour_mot     = 1000;       //pas_par_tour_mot
  const uint32_t Sa_pas_par_s2        = 45000;      //Sa_pas_par_s²,
  const uint32_t speed_down_t_ns      = 250;        //speed_down_t_ns,
  const uint32_t nb_steps_down_offset = 280;        //npas_down_offset pour 60Lmin/!\ à tester pour les autres débits
  const uint32_t speed_down_t2_ps     = 1150;       //speed_down_t2_us, /!\ pour 60mL à tester pour les autres débits
  const uint32_t Sd_pas_par_s2        = 300000;     //Sd_pas_par_s²,
  const float AxC50R5                 = 0.000204 ;  //constante courbe Vol(mL) de pas
  const float PxC50R5                 = 2.1605 ;    //puissance sur le numéro de pas Vol (pas)
  const float k_vol                   = 0.9f;       // We target 90% of the target volume for the seed
  const float k_debit                 = 1.f;         // 1 = we target the correct volume
  uint32_t t_pente_acc, t_debit_constant, t_pente_dec;
  uint32_t minimal_step_t_ns          = T_omegamax_ms * 1000;     //step_t_minQK_ns pour 600 l/min **** paramétrer la courbe en fonction du débit
  uint32_t step_total_us              = 0;
  uint32_t Vmax_pas_par_s             = ( (float) Vmax_tour_par_min / 60.f) * pas_par_tour_mot;
  uint32_t t_min_vmax_us              = 1000000/ Vmax_pas_par_s;
  uint32_t npas_acc                   = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sa_pas_par_s2);
  uint32_t npas_dec                   = (Vmax_pas_par_s*Vmax_pas_par_s)/(2*Sd_pas_par_s2);
  uint32_t npas_stop                  = (uint32_t) powf(((set_point_volume_mL * k_vol)/AxC50R5), 1.f/PxC50R5);


  uint32_t max_steps                  = MIN(npas_stop, MOTOR_MAX_STEPS);
  //variabnpas_accles float intermédiaires
  float t_debit_constant_f;
  float n_pas_f ;
  float debit_cible_Ls                = flow_set_point_initial_Lmin/60.f;
  //fin variables float intermédiaires

  
  uint32_t k = 0; 
  uint32_t total_nb_steps   = 0;
  uint32_t t_end_acc = 0;

  for(unsigned int n_pas = 1; n_pas <= max_steps; n_pas++)
  {
        if(n_pas <= npas_acc) {
          t_pente_acc = 1000000/sqrt(2*n_pas*Sa_pas_par_s2);
        }
        else {
          if(t_end_acc == 0)
          {
            t_end_acc = step_total_us;
          }
          t_pente_acc = t_min_vmax_us;
        }
        
        //Pas QK
        if(n_pas<npas_stop)
        {
        	n_pas_f = (float) n_pas ;
        	t_debit_constant_f=1000.f*AxC50R5*PxC50R5/debit_cible_Ls*k_debit*(powf (n_pas_f, PxC50R5-1.f)) ;
        	if(t_debit_constant_f < t_min_vmax_us)
        	{
        		t_debit_constant = t_min_vmax_us; //plafond vitesse moteur atteint
        	}
        	else {
        		t_debit_constant = (unsigned int) t_debit_constant_f;
        	}
        }
        else {
        	t_debit_constant = t_min_vmax_us;
        }

        //Pas décélération
        if( n_pas>=npas_stop-npas_dec && n_pas<=npas_stop) {
        	t_pente_dec = (1000000/sqrtf(2*(npas_stop+1-n_pas)*Sd_pas_par_s2));
        }
        else {
        	t_pente_dec = t_min_vmax_us;
        }
        t_motor_step_us[n_pas-1] = MAX(t_pente_acc, MAX(t_debit_constant, t_pente_dec));
        total_nb_steps++;
        step_total_us += t_motor_step_us[n_pas-1];
  }

  *nb_steps = total_nb_steps-1;
};


void convert_freq_to_motor_steps(float* freq_steps_per_slices,  //input: freq steps on each slices
				 uint32_t nb_freq_slices,	//input: freq of slices
				 uint32_t* t_motor_step_us,     //output: Array of steps time [MAX_MOTOR_STEPS]
                                 uint32_t* nb_steps);           //output: nb_steps

void convert_motor_steps_to_freq(uint32_t* t_motor_step_us,     //input: Array of steps time [MAX_MOTOR_STEPS]
                                 uint32_t nb_steps,            //input: nb_steps
				 uint32_t  nb_slices,	   	//input: nb slices 
				 float* freq_steps_per_slices) //output: array of freq per slices
{
}

void convert_samples_t_to_samples_steps(float*    flow_samples_Lpm,         //array of samples
					uint32_t  flow_samples_count,       //nb samples in array
					uint32_t  flow_samples_period_ms,   //delta T between samples
					uint32_t* motor_steps_us,           //Motor steps to feat the samples against
					uint32_t  nb_steps, 	            //Size of the array
					float* flow_samples_motor_steps)//output a array of samples feated on motor steps
{
  uint32_t t_acc_us = 0;
  uint32_t current_sample_idx = 0;
  for (uint32_t i=0 ; i< nb_steps; ++i) 
  {
    uint32_t current_sample_time = current_sample_idx * flow_samples_period_ms;
    
    if(t_acc_us >= current_sample_time*1000)
    {
      current_sample_idx++;
    }

    flow_samples_motor_steps[i] = flow_samples_Lpm[current_sample_idx];
    t_acc_us += motor_steps_us[i];
  }
}




//----------------------------------------------------------
// Private functions
//----------------------------------------------------------
static void compute_max_slope(uint32_t nb_steps_slices,
                              uint32_t nb_steps_per_slices,
                              float*   max_slope_acceleration,
                              float*   max_slope_deceleration)
{
  float VTi    = get_cycle_VTi_mL();
  float Pcrete = get_cycle_Pcrete_cmH2O();
  float PEP    = get_cycle_PEP_cmH2O();
  float VnMax  = AxC50R5*powf(MAX_MOTOR_STEPS, PxC50R5); 
  for(uint32_t k=0; k<nb_steps_slices; k++)
  {
    float Vn = AxC50R5*pow(k*nb_steps_per_slices, PxC50R5);
    float motor_load_percent_n = 0.75*(VTi/Vn*Pcrete/S4H2O_cm) + 0.25*(Vn/VnMax); 
    max_slope_acceleration[k] = Sa_max_pas_par_s2 * (1 - motor_load_percent_n);
    max_slope_deceleration[k] = Sa_max_pas_par_s2 + (Sd_max_pas_par_s2 - Sa_max_pas_par_s2) * motor_load_percent_n;
  }

}

static void compute_slice_flow_error(uint32_t nb_slices,
                                     uint32_t nb_steps_per_slices,
                                     float    target_Flow_Lpm,
                                     float*   flow_samples_motor_steps,
                                     float*   slice_flow_error)
{
  for(uint32_t k=0; k<nb_slices; k++)
  {
    slice_flow_error[k] = 0;
    for(uint32_t i=0; i < nb_steps_per_slices; i++)
    {
      slice_flow_error[k] -= flow_samples_motor_steps[k*nb_steps_per_slices+i];
    }
    slice_flow_error[k] /= nb_steps_per_slices;
    slice_flow_error[k] += target_Flow_Lpm;
  }
}

