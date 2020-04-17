


#include "main.h"
#include "utils.h"
#include "motor.h"
#include "sensors.h"
#include "pid.h"

#include <math.h>

#include "ihm_communication.h"


extern UART_HandleTypeDef huart4;		// IHM
extern I2C_HandleTypeDef  hi2c1;		// Sensors
extern TIM_HandleTypeDef htim2;			// MOTOR_STEP
extern TIM_HandleTypeDef htim3;			// PEP_STEP
extern TIM_HandleTypeDef htim4;			// us timer

extern TIM_HandleTypeDef htim15;			// Sensor_reporting



// ------------------------------------------------------------
// Python reporting
// ------------------------------------------------------------

static volatile uint8_t _reportTick;
void reporting_start(uint32_t period);
void report_send(TIM_HandleTypeDef* htim);
void reporting_stop();



// ------------------------------------------------------------
// Bavu Motor
// ------------------------------------------------------------

#define USE_V2

#if defined(USE_V1)
#define STEPS_PER_REVOLUTION 	(200*8*4)   	// rev_steps*microsteps*reduction
#define STEPS_PER_DEGREE		(STEPS_PER_REVOLUTION/360.)
#define MAX_SPEED				120				// MAX speed = MIN step period
#define EXHALE_SPEED			200				// RELEASE speed : release bag
#define STEP_PULSE			 	15
#define HOME_SPEED				400				// HOMEE speed
#define CALIBRATION_SPEED  (1./((STEPS_PER_REVOLUTION)*(200/360.0)))

#define 	RELEASE	 		DIR_CCW
#define 	COMPRESS	 	DIR_CW

motor_handle_t bavu_motor = {
		.tim 			= &htim17,
		.channel	= TIM_CHANNEL_1,
		.ena_port	= MOTOR_ENA_GPIO_Port,
		.ena_pin 	= MOTOR_ENA_Pin,
		.dir_port = MOTOR_DIR_GPIO_Port,
		.dir_pin  = MOTOR_DIR_Pin
};


#elif defined(USE_V2)

#define STEPS_PER_REVOLUTION 	(4000)   	// rev_steps*microsteps*reduction
#define STEPS_PER_DEGREE		(STEPS_PER_REVOLUTION/360.)
#define MAX_SPEED					120				// MAX speed = MIN step period
#define EXHALE_SPEED			200				// RELEASE speed : release bag
#define STEP_PULSE			 	3
#define HOME_SPEED				400				// HOMEE speed

#define MAX_VOLUME_ANGLE	430
#define CALIBRATION_SPEED  (1./((STEPS_PER_REVOLUTION)*(MAX_VOLUME_ANGLE/360.0)))


#define 	RELEASE	 		DIR_CCW
#define 	COMPRESS	 	DIR_CW

motor_handle_t bavu_motor = {
		.tim 			= &htim2,
		.channel	= TIM_CHANNEL_1,
		.ena_port	= MOTOR_ENA_GPIO_Port,
		.ena_pin 	= MOTOR_ENA_Pin,
		.ena_inverted = false,
		.dir_port = MOTOR_DIR_GPIO_Port,
		.dir_pin  = MOTOR_DIR_Pin,
		.pulse_width_us= MOTOR_PULSE_WIDTH_US
};

#endif



static volatile bool  		_is_home;

static uint16_t  bavu_motor_speed_table[6400];

static void bavu_motor_home(uint16_t step_time_us);
static bool bavu_is_home_A();
static bool bavu_is_home_B();
static bool bavu_motor_is_home() { return bavu_is_home_A() && bavu_is_home_B(); }



// ------------------------------------------------------------
// Pep Motor
// ------------------------------------------------------------

#define PEP_STEPS_PER_REVOLUTION 	(200)
#define PEP_STEPS_PER_DEGREE			(STEPS_PER_REVOLUTION/360.)
#define PEP_STEP_TIME_US					1000

static motor_handle_t pep_motor = {
		.tim 			= &htim3,
		.channel	= TIM_CHANNEL_4,
		.ena_port	= PEP_nENBL_GPIO_Port,
		.ena_pin 	= PEP_nENBL_Pin,
		.ena_inverted = true,
		.dir_port = PEP_DIR_GPIO_Port,
		.dir_pin  = PEP_DIR_Pin,
		.pulse_width_us= 3//PEP_PULSE_WIDTH_US
};

#define PEP_INC	DIR_CW
#define PEP_DEC	DIR_CCW

// ------------------------------------------------------------
// Pep Valve
// ------------------------------------------------------------

static void pep_valve_high();
static void pep_valve_low();

#define PEP_VALVE_HIGH 	GPIO_PIN_SET
#define PEP_VALVE_LOW 	GPIO_PIN_RESET

// ------------------------------------------------------------
// Miscellaneous
// ------------------------------------------------------------


void wait_btn_clicked();
#define INHALATION_FLOW_ARRAY_SIZE	400

static float 							_inhalation_flow[INHALATION_FLOW_ARRAY_SIZE];		//  used as buffer for flow analysis and motor command computing
static volatile uint32_t 	_logging_index = 0;
static volatile float 		_logging_time_step_sum = 0;

void flow_callback(float flow, uint32_t delta_t_us);


float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope);
int32_t get_plateau(float* samples, size_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound);

// ------------------------------------------------------------
// Breath Controller
// ------------------------------------------------------------

typedef enum {
	CTRL_STOPPED,
	CTRL_CALIBRATING,
	CTRL_INHALE,
	CTRL_TPLAT,
	CTRL_EXHALE
} controller_state_t;

static controller_state_t 	ctrl_state;


float 	Ti;		// inhaleTime
float 	Va; 	// Volume angle

float A_calibrated = 0.917;
float B_calibrated = 1.;
//uint32_t min_angle = 10;

float flow_setpoint_slm = 60;
void controller_run() {
	uint32_t time;
	uint16_t steps;
	sensors_reset_volume();

	printf("Recovid-F303\n");


	init_time_us(&htim4);
	start_time_us();

  if(!motor_init(&bavu_motor)) {
  	printf("Failed to init bavu motor\n");
  } else {
  	printf("bavu motor initialized\n");
  }


	printf("Press button to HOME bavu.\n");
	wait_btn_clicked();

	// Compress 20° first.
	motor_enable(&bavu_motor);
	motor_move(&bavu_motor, COMPRESS, 200, (uint16_t) (STEPS_PER_REVOLUTION)*(20/360.0));

	HAL_Delay(200);

	bavu_motor_home(HOME_SPEED);
	printf("homed.\n");


//  // PEP Motor
//  // Set stepping: 16 microsteps
//  HAL_GPIO_WritePin(PEP_MODE0_GPIO_Port, PEP_MODE0_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(PEP_MODE0_GPIO_Port, PEP_MODE1_Pin, GPIO_PIN_SET);
//
//  // Set indexer mode
//  HAL_GPIO_WritePin(PEP_CONFIG_GPIO_Port, PEP_CONFIG_Pin, GPIO_PIN_SET);
//  // Activate
//  HAL_GPIO_WritePin(PEP_nSLEEP_GPIO_Port, PEP_nSLEEP_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//
//
//  if(!motor_init(&pep_motor)) {
//  	printf("Failed to init pep motor\n");
//  } else {
//  	printf("pep motor initialized\n");
//  }
//
//
//  printf("Homing pep\n");
//
//	motor_enable(&pep_motor);
//  motor_run(&pep_motor, PEP_DEC, 100 );
//  while(HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin));
//
//  printf("Pep Homed\n");
//
//  motor_disable(&pep_motor);
//
////	motor_disable(&bavu_motor);
////
////	printf("Press button to squeeze.\n");
////	wait_btn_clicked();
////
////	motor_enable(&bavu_motor);
////
////	while(true) {
////		motor_move(&bavu_motor, COMPRESS, 200, 4900);
////
////		while(motor_is_moving(&bavu_motor));
////
////		bavu_motor_home(HOME_SPEED);
////	}
//
//
//


//	printf("Press btn to start\n");
//	wait_btn_clicked();
//
//	Va = 430;
//
//	A_calibrated= 1;
//	B_calibrated= 0.4;
//
//	steps = (uint32_t)(STEPS_PER_DEGREE * Va);
//	Ti = 0.;
//	for(long t=0; t<steps; ++t) {
//		float d = compte_motor_step_time(t, 1., CALIBRATION_SPEED);
//		Ti += d;
//		bavu_motor_speed_table[t]= (uint32_t)d;
//	}
//	printf("Ti_predicted = %ld ms\n", (uint32_t)(Ti/1000));
//
//	motor_move_profile(&bavu_motor, COMPRESS, bavu_motor_speed_table, steps);


	printf("Scanning I2C bus\n");
	sensors_scan(&hi2c1);

	if(!sensors_init(&hi2c1)) {
		printf("I2C Error!!!\n");
		while(true);
	}

	sensors_start();
	printf("Sensors started\n");

	reporting_start(100);
	printf("Reporting started\n");

	// Calibration

	Va= 280; //MAX_VOLUME_ANGLE;

//	calibration(&A_calibrated, &B_calibrated, 2);

	A_calibrated= 3.577;
	B_calibrated= -0.455;

	uint32_t Te= 2000; //millisecond

	steps = (uint32_t)(STEPS_PER_DEGREE * Va);
	Ti = 0.;
	for(long t=0; t<steps; ++t) {
		float d = compte_motor_step_time(t, 1., CALIBRATION_SPEED,A_calibrated,B_calibrated);
		Ti += d;
		//printf("d=%ld\n", (uint32_t)(d));
		bavu_motor_speed_table[t]= (uint32_t)d;
	}
	printf("Ti_predicted = %ld ms\n", (uint32_t)(Ti/1000));

	printf("Press btn to start\n");
	wait_btn_clicked();

	printf("Running\n");

	motor_enable(&bavu_motor);
	bavu_motor_home(HOME_SPEED);
	ctrl_state = CTRL_STOPPED;

	while (1) {
		sensors_reset_volume();  	// Reset volume integrator
		ctrl_state= CTRL_INHALE;
		printf("INHALE\n");

	  // HIGH PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH);
		time= HAL_GetTick(); // Start time

		//************************************** INHALATION MOVEMENT ********************************* //
		//##- Start DMA Burst transfer
		motor_move_profile(&bavu_motor, COMPRESS, bavu_motor_speed_table, steps);

		// Start flow sensor capture in array
		_logging_index = 0;
		_logging_time_step_sum = 0.;
		sensors_set_flow_callback(flow_callback);

		// Wait for motion end.
		while(motor_is_moving(&bavu_motor));
		motor_stop(&bavu_motor);
		//********************************************* TPLAT **************************************** //
		// Stop flow capture
		sensors_set_flow_callback(NULL); // Now we have flow data from the beginning of motor compression to the end of motor compression

		// Take inhalation time and report it
		time= HAL_GetTick() - time;
		printf("Ti = %lu ms\n", time);

		// Inhalation pause : Tplat
		printf("TPLAT\n");
		HAL_Delay(700);
		float volume_cycle = sensors_get_volume();
		printf("Vi = %ld ml\n", (int32_t)(volume_cycle * 1000));
		ctrl_state= CTRL_EXHALE;
		printf("EXHALE\n");

		// LOW PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW);

		time= HAL_GetTick();
		_is_home = false;
		//************************************** EXPIRATION MOVEMENT ********************************* //
		motor_run(&bavu_motor, RELEASE, EXHALE_SPEED);
//************************************************* PID ZONE ********************************************//
		compute_pid(&A_calibrated,&B_calibrated, _logging_index,  _logging_time_step_sum, flow_setpoint_slm,_inhalation_flow);
		// Recompute motor steps
		Ti = 0.;
		for(long t=0; t<steps; ++t) {
			float d = compte_motor_step_time(t, flow_setpoint_slm/60., CALIBRATION_SPEED, A_calibrated, B_calibrated);
			Ti+=d;
			//printf("d=%ld\n", (uint32_t)(d));
			bavu_motor_speed_table[t]= (uint32_t)d;
		}
		printf("Ti_predicted = %ld ms\n", (uint32_t)(Ti/1000));
//*******************************************************************************************************//
		// home will be set in EXTI interrupt handler. PWM will also be stopped.
		while(!bavu_motor_is_home());
		printf("motor homed.\n");
		motor_stop(&bavu_motor);
		//HAL_Delay(1000);
		time = HAL_GetTick() - time;
		if(time < Te-1) {
			HAL_Delay(Te-time);
		}

		printf("Cycle is_done*********************\n");
//		printf("Press button to make a new one.\n");
//		reporting_stop();
//		wait_btn_clicked();
	}
	// Disable motor
	motor_disable(&bavu_motor);
	reporting_stop();
}


void flow_callback(float flow, uint32_t delta_t_us) {
	if(_logging_index>=INHALATION_FLOW_ARRAY_SIZE) return;
	_inhalation_flow[_logging_index] = flow/60.;  // in sls
	_logging_time_step_sum += (float)delta_t_us/1000000;
	++_logging_index;
}

static bool bavu_is_home_A() {
	return HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin)==GPIO_PIN_RESET;
}

static bool bavu_is_home_B() {
	return HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin)==GPIO_PIN_RESET;
}

void bavu_motor_home(uint16_t speed) {
	printf("homing...\n");
	if(bavu_is_home_A() && bavu_is_home_B()) return;

//	// Move 10° inhalewise to make sure (almost) that we're on the right side of the switch !!
//	uint16_t nb_steps= (uint16_t) (STEPS_PER_REVOLUTION)*(20/360.0);
//	motor_move(&bavu_motor, COMPRESS, HOME_SPEED, nb_steps);
//	while(motor_is_moving(&bavu_motor));
//	HAL_Delay(200);

	// Move exhalewise until the switch triggers
  _is_home = false;
  motor_run(&bavu_motor, RELEASE, speed);
	// home will be set in EXTI interrupt handler.
	while(!bavu_motor_is_home());
	motor_stop(&bavu_motor);
}


static uint8_t* SYNC="---START---\r\n";
static uint32_t _report_period=0;
static uint32_t _report_counter=0;

void reporting_start(uint32_t ms) {
	HAL_UART_Transmit_IT(&huart4, SYNC, strlen((const char*)SYNC));

	_report_period=ms;
	_report_counter=ms;

	htim15.PeriodElapsedCallback=report_send;
	htim15.Init.Period = 1000;
	htim15.PeriodElapsedCallback = report_send;
	HAL_TIM_Base_Init(&htim15);
	HAL_TIM_Base_Start_IT(&htim15);
}

static uint8_t data_buffer[13] = { '>', 0,0,0,0,0,0,0,0,0,0,0,0 };

void report_send(TIM_HandleTypeDef* htim) {

	--_report_counter;
	if(_report_counter!=0) return;
	_report_counter= _report_period;


	float flow= sensors_get_flow();
	uint8_t* ptr= (uint8_t*)&flow;
	data_buffer[1]= *ptr++;
	data_buffer[2]= *ptr++;
	data_buffer[3]= *ptr++;
	data_buffer[4]= *ptr++;

	float pressure= sensors_get_pressure();
	ptr= (uint8_t*)&pressure;
	data_buffer[5]= *ptr++;
	data_buffer[6]= *ptr++;
	data_buffer[7]= *ptr++;
	data_buffer[8]= *ptr++;

	float volume= sensors_get_volume();
	ptr= (uint8_t*)&volume;
	data_buffer[9]= *ptr++;
	data_buffer[10]= *ptr++;
	data_buffer[11]= *ptr++;
	data_buffer[12]= *ptr++;

	send_DATA(pressure, flow, volume, 0, 0);
	HAL_UART_Transmit_DMA(&huart4, data_buffer, sizeof(data_buffer));
}

void reporting_stop() {
	HAL_TIM_Base_Stop(&htim15);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// TODO: rework home interrupt logic
//	if(GPIO_Pin==MOTOR_LIMIT_SW_A_Pin || GPIO_Pin==MOTOR_LIMIT_SW_B_Pin) {
//		_is_home=HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_A_GPIO_Port, MOTOR_LIMIT_SW_A_Pin) && HAL_GPIO_ReadPin(MOTOR_LIMIT_SW_B_GPIO_Port, MOTOR_LIMIT_SW_B_Pin);
//	}
}




void wait_btn_clicked() {
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_SET);
	HAL_Delay(50);
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)!=GPIO_PIN_SET);
	HAL_Delay(50);
}

