#include "breathing.h"
#include "stdio.h"
#include "math.h"

#include "controller.h"
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
#define NB_SLICES 10
#define VOL_KP 0
#define VOL_KI 0
#define VOL_KD 0
#define FLOW_KP 0
#define FLOW_KI 0 
#define FLOW_KD 0

//----------------------------------------------------------
// Private typedefs
//----------------------------------------------------------

//----------------------------------------------------------
// Private variables
//----------------------------------------------------------
static bool l_adaptation_initialized = false;
static const uint32_t T_omegamax_us  = 200;
static float flow_samples_motor_steps[MOTOR_MAX_STEPS];
static const uint32_t Sa_max_pas_par_s2 = 45000;
static const uint32_t Sd_max_pas_par_s2 = 3E5;
static const float    AxC50R5           = 0.000204 ;  //constante courbe Vol(mL) de pas
static const float    PxC50R5           = 2.1605 ;    //puissance sur le numéro de pas Vol (pas)
static const float    S4H2O_cm          = 70;         //limite haute Pcrete pour selecteur moteur S4 en position C (85% du courant max)
static const float    k_phi             = 0.7; //limit de survitesse motor

static float freq_steps_per_slices[NB_SLICES];
// Structure to strore PID data and pointer to PID structure
struct pid_controller ctrldata_Volume;
struct pid_controller ctrldata_Debit[NB_SLICES];
pid_H pid_Volume;
pid_H pid_FlowRate[NB_SLICES];
float PreviousCycle_Vti, flow_command_updated, objective_volume;
static float flow_per_slices[NB_SLICES];
float Freq_Steps_updated[NB_SLICES];

//----------------------------------------------------------
// Private functions prototypes
//----------------------------------------------------------
void compute_seed( float     flow_set_point_initial_Lmin, //input:  spec v8 20-60   L/min
                   float     set_point_volume_mL,         //input:  spec v8 100-600 mL
		   uint32_t* t_motor_step_us,             //output: Array of steps time [MAX_MOTOR_STEPS]
                   uint32_t* nb_steps);                   //output: nb_steps

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

static void compute_max_slope(uint32_t nb_slices,
                              uint32_t nb_steps_per_slices,
                              float*   max_slope_acceleration,
                              float*   max_slope_deceleration);

static void compute_slice_flow(uint32_t nb_slices,
                                     uint32_t nb_steps_per_slices,
                                     float    target_Flow_Lpm,
                                     float*   flow_samples_motor_steps,
                                     float*   slice_flow);

static uint32_t convert_samples_slice_to_steps(uint32_t nb_slices,
                                           uint32_t nb_steps_total,
					   float*   freq_steps_per_slices,
                                           float*   max_slope_acceleration,
                                           float*   max_slope_deceleration,
					   uint32_t Ti_ms,
					   uint32_t nb_steps,
                                           uint32_t* t_motor_step_us, //output: Array of steps time [MAX_MOTOR_STEPS]
					   uint32_t* Ta_ms,  // output : total motor steps time for acceleration
					   uint32_t* Td_ms,  // output : total motor steps time before deceleration
					   uint32_t* Tf_ms ); // output : total motor steps time   

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
      printf("%ld\n", steps_t_us[j]);
    }
}
static bool update_float(float* value_to_update, float new_value)
{
    if(*value_to_update != new_value)
    {
        *value_to_update = new_value;
	return true;
    }
    return false;
}


static uint32_t seed_steps_us[MOTOR_MAX_STEPS];
static uint32_t nb_steps_seed;

uint32_t adaptation(
    float       target_VT_mL,
    float       target_Flow_Lpm,
    uint32_t    flow_samples_period_ms, 
    uint32_t    flow_samples_count, 
    float*      flow_samples_Lpm, 
    uint32_t    motor_max_steps, 
    uint32_t*   motor_steps_us) 
{
  static float l_target_VT_ml = 0.0;
  static float l_target_Flow_Lpm = 0.0;
  bool settings_changed = false;

  settings_changed |= update_float(&l_target_VT_ml, target_VT_mL);
  settings_changed |= update_float(&l_target_Flow_Lpm, target_Flow_Lpm);
  uint32_t nb_steps = motor_max_steps;

  if(l_adaptation_initialized != true || settings_changed)
  {
    compute_seed(target_Flow_Lpm, target_VT_mL, seed_steps_us, &nb_steps_seed);
    for(uint32_t i = 0; i < nb_steps_seed; i++)
    {
      motor_steps_us[i] = seed_steps_us[i];
    }
    nb_steps = nb_steps_seed;

    l_adaptation_initialized = true;

    flow_command_updated = get_setting_Vmax_Lpm();
    // Prepare PID Volume controller for operation, set limits and enable controller
    /*pid_Volume = pid_create(	&ctrldata_Volume, 
	&PreviousCycle_Vti, 
	&flow_command_updated, 
	&objective_volume, 
	VOL_KP, VOL_KI, VOL_KD);
    pid_limits(pid_Volume, 0, 200);
    pid_auto(pid_Volume);*/

    // Prepare PID Flowrate controller for operation, set limits and enable controller
    for(int slice_index=0; slice_index<NB_SLICES; slice_index++)
    {
      pid_FlowRate[slice_index] = pid_create(	&ctrldata_Debit[slice_index], 
	  &flow_per_slices[slice_index], // Input data
	  &Freq_Steps_updated[slice_index], // Output
	  &flow_command_updated, // Command Setpoint
	  FLOW_KP, FLOW_KI, FLOW_KD);
	  
      pid_limits(pid_FlowRate[slice_index], 0, 200);
      pid_auto(pid_FlowRate[slice_index]);
    }
  }
  //translate sample for time to steps
  // => F1

  convert_motor_steps_to_freq(seed_steps_us, nb_steps, NB_SLICES, freq_steps_per_slices);

  convert_samples_t_to_samples_steps(flow_samples_Lpm, flow_samples_count, flow_samples_period_ms, motor_steps_us, motor_max_steps, flow_samples_motor_steps);

  compute_slice_flow(NB_SLICES, nb_steps/NB_SLICES, target_Flow_Lpm, flow_samples_motor_steps, flow_per_slices);

  float max_slope_acceleration[NB_SLICES], max_slope_deceleration[NB_SLICES];
  //// F3 Compute MAX acceleration per Slice
  compute_max_slope(	NB_SLICES,
      nb_steps,
      max_slope_acceleration,
      max_slope_deceleration);

  //// Call Volume PID every X cycles: Read process feedback, Compute new PID output value
  //PreviousCycle_Vti = get_cycle_VTi_mL();
  //pid_compute(pid_Volume); // result is stored in flow_command_updated

  //// Call Debit PID
  for(int slice_index=0; slice_index<NB_SLICES; slice_index++)
  {
    // Compute new PID output value
    pid_compute(pid_FlowRate[slice_index]); // result is stored in Freq_Steps_updated[slice_index]

    // Add the seed to the result
    //Freq_Steps_updated[slice_index] += freq_steps_per_slices[slice_index];
    printf("Freq_St %d = %d\n", slice_index, (int) Freq_Steps_updated[slice_index]);
  }

  uint32_t Ta_ms, Td_ms, Tf_ms;
  // Compute motor step table 
  nb_steps = convert_samples_slice_to_steps(NB_SLICES,
                                            nb_steps,
					    Freq_Steps_updated,
                                            max_slope_acceleration,
                                            max_slope_deceleration,
					    get_setting_Tinsu_ms(),
					    nb_steps,
                                            motor_steps_us, //output: Array of steps time [MAX_MOTOR_STEPS]
					    &Ta_ms,  // output : total motor steps time for acceleration
					    &Td_ms,  // output : total motor steps time before deceleration
					    &Tf_ms ); // output : total motor steps time   
  print_steps(motor_steps_us, nb_steps);
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
  uint32_t minimal_step_t_ns          = T_omegamax_us * 1000;     //step_t_minQK_ns pour 600 l/min **** paramétrer la courbe en fonction du débit
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



void convert_motor_steps_to_freq(uint32_t* t_motor_step_us,       //input: Array of steps time [MAX_MOTOR_STEPS]
                                 uint32_t  nb_steps,              //input: nb_steps
				 uint32_t  nb_slices,	   	  //input: nb slices 
				 float*    freq_steps_per_slices) //output: array of freq per slices
{
  uint32_t nb_steps_per_slices = nb_steps / nb_slices;
  //The last slice may have more steps (between 0 to NB_SLICES more)
  uint32_t last_nb_steps_per_slices = nb_steps%nb_slices + nb_steps_per_slices;
  uint32_t slice_idx = 0;
  uint32_t t_acc_us;
  for(slice_idx = 0; slice_idx < nb_slices; slice_idx++)
  {
    if(slice_idx == nb_slices-1) 
    {
      nb_steps_per_slices = last_nb_steps_per_slices;   
    }

    t_acc_us = 0;
    uint32_t start_idx = slice_idx*nb_steps_per_slices;
    uint32_t end_idx   = start_idx + nb_steps_per_slices;
    for(uint32_t step_idx_within_slice = start_idx; step_idx_within_slice < end_idx; step_idx_within_slice++)
    {
      t_acc_us += t_motor_step_us[step_idx_within_slice];
    }
    if(t_acc_us != 0)
      freq_steps_per_slices[slice_idx] = nb_steps_per_slices*1000000.f  / t_acc_us;
    else
      freq_steps_per_slices[slice_idx] = 0.0f;

    printf("freq %d = %d\n", slice_idx,  (int) freq_steps_per_slices[slice_idx]);
  }
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
static void compute_max_slope(uint32_t nb_slices,
                              uint32_t nb_steps_total,
                              float*   max_slope_acceleration,
                              float*   max_slope_deceleration)
{
  uint32_t nb_steps_per_slices = nb_steps_total / nb_slices;
  //The last slice may have more steps (between 0 to NB_SLICES more)
  uint32_t last_nb_steps_per_slices = nb_steps_total%nb_slices + nb_steps_per_slices;

  float VTi    = get_cycle_VTi_mL();
  float Pcrete = get_cycle_Pcrete_cmH2O();
//  float PEP    = get_cycle_PEP_cmH2O();
  float VnMAX  = AxC50R5*powf(MAX_MOTOR_STEPS, PxC50R5); 
  for(uint32_t k=0; k<nb_slices; k++)
  {
    if(k == nb_slices-1) 
    {
      nb_steps_per_slices = last_nb_steps_per_slices;   
    }
    float Vn = AxC50R5*pow(k*nb_steps_per_slices, PxC50R5);
    float motor_load_percent_n = 0.75*(VTi/Vn*Pcrete/S4H2O_cm) + 0.25*(Vn/VnMAX); 
    max_slope_acceleration[k] = Sa_max_pas_par_s2 * (1 - motor_load_percent_n);
    max_slope_deceleration[k] = Sa_max_pas_par_s2 + (Sd_max_pas_par_s2 - Sa_max_pas_par_s2) * motor_load_percent_n;
  }

}

static void compute_slice_flow(uint32_t nb_slices,
                               uint32_t nb_steps_total,
                               float    target_Flow_Lpm,
                               float*   flow_samples_motor_steps,
                               float*   compute_slice_flow)
{
  uint32_t nb_steps_per_slices = nb_steps_total / nb_slices;
  //The last slice may have more steps (between 0 to NB_SLICES more)
  uint32_t last_nb_steps_per_slices = nb_steps_total%nb_slices + nb_steps_per_slices;
  uint32_t k = 0;
  for( k=0; k<nb_slices; k++)
  {
    if(k == nb_slices-1) 
    {
      nb_steps_per_slices = last_nb_steps_per_slices;   
    }
    compute_slice_flow[k] = 0;
    for(uint32_t i=0; i < nb_steps_per_slices; i++)
    {
      compute_slice_flow[k] += flow_samples_motor_steps[k*nb_steps_per_slices+i];
    }
    compute_slice_flow[k] /= nb_steps_per_slices;
  }
  compute_slice_flow[k] = 0;

}

uint32_t T_minobj(uint32_t current_step, uint32_t flow_set_point_Lpm)
{
  float flow_set_point_Lpm_f = flow_set_point_Lpm;
  return ( (uint32_t) (k_phi *(1000.f*AxC50R5*PxC50R5*powf(current_step,PxC50R5-1)) / (flow_set_point_Lpm_f/60.f)));
}
#define pow2(_a_) ((_a_)*(_a_))

static uint32_t convert_samples_slice_to_steps(uint32_t nb_slices,
                                           uint32_t nb_steps_total,
					   float*   freq_steps_per_slices,
                                           float*   max_slope_acceleration,
                                           float*   max_slope_deceleration,
					   uint32_t Ti_ms,
					   uint32_t nb_steps,
                                           uint32_t* t_motor_step_us, //output: Array of steps time [MAX_MOTOR_STEPS]
					   uint32_t* Ta_ms,  // output : total motor steps time for acceleration
					   uint32_t* Td_ms,  // output : total motor steps time before deceleration
					   uint32_t* Tf_ms ) // output : total motor steps time             
{
    uint32_t nb_steps_per_slices = nb_steps_total / nb_slices;
  //The last slice may have more steps (between 0 to NB_SLICES more)
  uint32_t last_nb_steps_per_slices = nb_steps_total%nb_slices + nb_steps_per_slices;

  uint32_t t_accumulated_steps = 0;
  uint32_t current_step = 0;
  uint32_t nb_steps_for_stop;
  for(uint32_t k=0; k<nb_slices; k++)
  {
    float Sa;
    


    if (k==0)
    {
      //		Sa = freq_steps_per_slices[0]*2;
      Sa = Sa_max_pas_par_s2;
      freq_steps_per_slices[0] = 0.0;
      for(uint32_t i=0; i < nb_steps_per_slices; i++, current_step++)
      {
	t_motor_step_us[i] = 1E6/(sqrtf(2*Sa*(i+1)));
	t_accumulated_steps+= t_motor_step_us[i];
	freq_steps_per_slices[0] += ((1E6 /(float)t_motor_step_us[i]) / (float)nb_steps_per_slices);
      }
      printf("Sa[%ld] = %d Samax =%d Sdmax %d\n", k, (int)Sa, (int) max_slope_acceleration[k], (int) max_slope_deceleration[k]);


    }
    else
    {
      bool     trigger_stop = false;
      bool     is_first_acceleration_ended = false;
      uint32_t steps_offset_per_slice;
      float    freq_steps_per_slices_pow2 = freq_steps_per_slices[k-1]*freq_steps_per_slices[k-1];

      if(k == nb_slices-1) 
      {
	nb_steps_per_slices = last_nb_steps_per_slices;   
      }

      Sa = (pow2(freq_steps_per_slices[k])-pow2(freq_steps_per_slices[k-1]))/(2*nb_steps_per_slices);
      // Compute Sa Sd (=-Sa) and limit to Smax
      //if(! TEST_FLT_EQUALS(Sa, 0.f) )
      //{
      //  Sa = MIN(Sa,  max_slope_acceleration[k]);
      //  Sa = MAX(Sa, -max_slope_deceleration[k]);

      //  if ((Sa < 0) && !is_first_acceleration_ended)
      //  {
      //    *Ta_ms = t_accumulated_steps/1000;
      //    is_first_acceleration_ended = true;
      //  }
      //}
      //else 
      //{
      //  freq_steps_per_slices[k] = freq_steps_per_slices[k-1];
      //}
      printf("Sa[%ld] = %d Samax =%d Sdmax %d\n", k, (int)Sa, (int) max_slope_acceleration[k], (int) max_slope_deceleration[k]);
      // Compute offsets
      //if(TEST_FLT_EQUALS(Sa, 0.f)) 
      //{
      //  //We will apply the mean speed
      //  steps_offset_per_slice = 1;
      //}
      //else
      //{
      //  steps_offset_per_slice = (uint32_t) fabs(freq_steps_per_slices_pow2 / (2. * Sa));
      //}
      // Compute  step time for each slices > 0

      float t_motor_step_us_f = t_motor_step_us[current_step-1];
      for(uint32_t i=0; i < nb_steps_per_slices; i++, current_step++)
      {

	//current_step = k*nb_steps_per_slices+i;	
	//if(! TEST_FLT_EQUALS(Sa, 0.f) ) // TODO SMA sign of float ??
	//{
	  //if(Sa > 0.f)
	  //{
	  //  t_motor_step_us[current_step] = 1E6 / ( sqrtf(2 * (steps_offset_per_slice + i) * Sa) );
	  //}
	  //else
	  //{
	  //  t_motor_step_us[current_step] = 1E6 / ( sqrtf(2 * (steps_offset_per_slice - i) * fabs(Sa)));
	  //}
	  //
	  float old_v_moteur = (1E6 / ( t_motor_step_us_f)) ;
	  float variation = Sa * t_motor_step_us_f * 1E-6;
	  float new_v_moteur = old_v_moteur + variation; 
	  t_motor_step_us_f  = ( 1000.f / (new_v_moteur/ (1000.f)));
	  t_motor_step_us[current_step] = (uint32_t) t_motor_step_us_f;
	  //
	//}
	//else
	//{ 
	  //Acceleration is 0. so speed is constant for all the slice
	  //Mean speed
	//  t_motor_step_us[current_step] = 1E6 / freq_steps_per_slices[k];
	//}
	// limit to motor max speed and throughput max for patient safety
	//printf("T omega = %d minobj = %d motor_step = %d\n",
	//    T_omegamax_us,
	//    T_minobj(current_step, get_setting_Vmax_Lpm()), 
	//    t_motor_step_us[current_step]);
	//t_motor_step_us[current_step] = MAX(T_omegamax_us,  MAX(T_minobj(current_step, get_setting_Vmax_Lpm()), t_motor_step_us[current_step]));
	t_accumulated_steps += t_motor_step_us[current_step];
        //printf("t_accumulated_steps[%d] = %d\n", current_step, t_accumulated_steps);
	
	// Compute nb of steps for stop
	// TODO SMA : check compute in float

	//nb_steps_for_stop = (uint32_t) (1E6/(pow2(((float) t_motor_step_us[current_step]))*2.0f*max_slope_deceleration[k]*1E-6));
	//uint32_t t_for_stop_us = (uint32_t) 1E6/( ((float) t_motor_step_us[current_step]*1E-6) * max_slope_deceleration[k]) ; 
	////printf("nb_steps_for_stop = %d\n", nb_steps_for_stop);

	////// TDO CDE check > vs >= (step_count calculation)
	////	
	//if( (k == (nb_slices - 1) ) && ((current_step+nb_steps_for_stop) >= nb_steps) || ((t_accumulated_steps + t_for_stop_us)> Ti_ms*1000))
	//{
	//  printf("current_step %d +nb_steps_for_stop %d) >= nb_steps %d\n", current_step, nb_steps_for_stop, nb_steps);
	//  printf("t_accumulated_steps %d t_for_stop_us %d Ti_ms*1000 %d\n",t_accumulated_steps, t_for_stop_us, Ti_ms*1000);
	//  trigger_stop = true;
	//  break;
	//}
      }
      // TODO TSE : do you need actuated computed frequency vs requested ??
      //VM_per_slice[k] = 1000000.f/t_motor_step_us[current_step-1];

      // Compute final deceleration
      //if ((k == (nb_slices-1)) || (trigger_stop))
      //{
      //  *Td_ms = t_accumulated_steps/1000;
      //  for(uint32_t j=1;j<nb_steps_for_stop;j++, current_step++)
      //  {
      //    t_motor_step_us[current_step] = 1E6/sqrtf(2*(nb_steps_for_stop-j)*max_slope_deceleration[k]);
      //    t_accumulated_steps += t_motor_step_us[current_step];
      //  }
      //  *Tf_ms = t_accumulated_steps/1000;
      //  for(uint32_t i = current_step; i < nb_steps; i++)
      //  {
      //    t_motor_step_us[i] = 0;
      //  }
      //  return current_step;
      //}
    }
  }
  //We shoulod never be here => the stop condition is on Compute final deceleration
  printf("NB steps = %d\n", nb_steps);
  return nb_steps;
  return 0;
}
