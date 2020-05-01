#ifndef __COMPUTE_MOTOR_H__
#define __COMPUTE_MOTOR_H__


unsigned int compute_motor_press_christophe(
		unsigned int step_t_ns_init, 
		unsigned int acc_ns, 
		unsigned int step_t_min_ns, 
		unsigned int speed_down_t_ns, 
		unsigned int speed_down_t2_ps, 
		unsigned int step_t_ns_final,
		unsigned int dec_ns,
		unsigned int nb_steps, 
		uint32_t* steps_t_us);

void print_christophe_header(
		unsigned int step_t_ns_init, 
		unsigned int acc_ns, 
		unsigned int step_t_min_ns, 
		unsigned int speed_down_t_ns, 
		unsigned int speed_down_t2_ps, 
		unsigned int step_t_ns_final,
		unsigned int dec_ns,
		int nb_steps);


unsigned int compute_constant_motor_steps(uint32_t step_t_us, unsigned int nb_steps, uint32_t* steps_t_us);

void print_steps(uint32_t* steps_t_us, unsigned int nb_steps);

#endif //__COMPUTE_MOTOR_H__

