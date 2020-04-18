


#include "main.h"
#include "utils.h"
#include "motor.h"
#include "sensors.h"

#include <math.h>

#include "ihm_communication.h"


extern UART_HandleTypeDef huart4;		// IHM
extern I2C_HandleTypeDef  hi2c1;		// Sensors
extern TIM_HandleTypeDef htim2;			// MOTOR_STEP
extern TIM_HandleTypeDef htim3;			// PEP_STEP
extern TIM_HandleTypeDef htim4;			// us timer

extern TIM_HandleTypeDef htim7;			// 1kHz sensors timer

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
#define PEP_STEP_TIME_US					4000

static motor_handle_t pep_motor = {
		.tim 			= &htim3,
		.channel	= TIM_CHANNEL_4,
		.ena_port	= PEP_nENBL_GPIO_Port,
		.ena_pin 	= PEP_nENBL_Pin,
		.ena_inverted = true,
		.dir_port = PEP_DIR_GPIO_Port,
		.dir_pin  = PEP_DIR_Pin,
		.pulse_width_us= 10//PEP_PULSE_WIDTH_US
};


typedef enum {
	ZERO,
	ONE,
	HIGHZ
} step_mode_t;

static void pep_motor_stepping(step_mode_t m0, step_mode_t m1) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = PEP_MODE0_Pin;
  GPIO_InitStruct.Mode = m0==HIGHZ?GPIO_MODE_OUTPUT_OD:GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PEP_MODE0_GPIO_Port, PEP_MODE0_Pin, m0==HIGHZ?1:m0);

  GPIO_InitStruct.Pin = PEP_MODE1_Pin;
  GPIO_InitStruct.Mode = m1==HIGHZ?GPIO_MODE_OUTPUT_OD:GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PEP_MODE1_GPIO_Port, PEP_MODE1_Pin, m1==HIGHZ?1:m1);
}


#define PEP_INC	DIR_CCW
#define PEP_DEC	DIR_CW

// ------------------------------------------------------------
// Pep Valve
// ------------------------------------------------------------

#define PEP_VALVE_HIGH 	GPIO_PIN_SET
#define PEP_VALVE_LOW 	GPIO_PIN_RESET

static void pep_valve_high() { HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH); }
static void pep_valve_low()  {HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW); }


// ------------------------------------------------------------
// Mat
// ------------------------------------------------------------

typedef enum {
	MAT_OFF,
	MAT_GREEN,
	MAT_ORANGE,
	MAT_RED
} mat_color_t;

static void mat_set_color(mat_color_t color) {
	HAL_GPIO_WritePin(MAT_LED_GREEN_GPIO_Port, MAT_LED_GREEN_Pin, color==MAT_GREEN?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MAT_LED_ORANGE_GPIO_Port, MAT_LED_ORANGE_Pin, color==MAT_ORANGE?GPIO_PIN_SET:GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MAT_LED_RED_GPIO_Port, MAT_LED_RED_Pin, color==MAT_RED?GPIO_PIN_SET:GPIO_PIN_RESET);
}


// ------------------------------------------------------------
// Buzzer
// ------------------------------------------------------------

typedef enum {
	BUZZER_OFF,
	BUZZER_LOW,
	BUZZER_MEDIUM,
	BUZZER_HIGH,
} buzzer_level_t;

static void buzzer_set_level(buzzer_level_t level) {
	GPIO_PinState low		=GPIO_PIN_RESET;
	GPIO_PinState medium=GPIO_PIN_RESET;
	GPIO_PinState high	=GPIO_PIN_RESET;

	switch(level) {
	case BUZZER_LOW:
		low= GPIO_PIN_SET;
		break;
	case BUZZER_MEDIUM:
		low= GPIO_PIN_SET;
		medium= GPIO_PIN_SET;
		break;
	case BUZZER_HIGH:
		low= GPIO_PIN_SET;
		high= GPIO_PIN_SET;
		break;
	}
	HAL_GPIO_WritePin(BUZZER_HIGH_GPIO_Port, BUZZER_HIGH_Pin, high);
	HAL_GPIO_WritePin(BUZZER_MEDIUM_GPIO_Port, BUZZER_MEDIUM_Pin, medium);
	HAL_GPIO_WritePin(BUZZER_LOW_GPIO_Port, BUZZER_LOW_Pin, low);

}


// ------------------------------------------------------------
// FailSafe Button
// ------------------------------------------------------------

static bool failsafe_is_on() {
	return HAL_GPIO_ReadPin(FS_Enabled_GPIO_Port, FS_Enabled_Pin);
}


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


int32_t calibration(float* A, float* B, uint8_t iterations);
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed);
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

	printf("Turn on FS button to initialize Recovid\n");
	while(!failsafe_is_on());
	HAL_Delay(50);


	printf("Initializing bavu motor\n");

	// Compress 20° first.
  if(!motor_init(&bavu_motor)) {
  	printf("Failed to init bavu motor\n");
  } else {
  	printf("bavu motor initialized\n");
  }

	printf("Homing bavu motor.\n");
	motor_enable(&bavu_motor);
	motor_move(&bavu_motor, COMPRESS, 200, (uint16_t) (STEPS_PER_REVOLUTION)*(20/360.0));

	HAL_Delay(200);

	bavu_motor_home(HOME_SPEED);
	printf("bavu motor homed.\n");


  // PEP Motor
	// Set stepping: 8
	printf("Initializing PEP motor\n");
	pep_motor_stepping(ZERO,ONE);

  // Set indexer mode
  HAL_GPIO_WritePin(PEP_CONFIG_GPIO_Port, PEP_CONFIG_Pin, GPIO_PIN_SET);
  // Activate
  HAL_GPIO_WritePin(PEP_nSLEEP_GPIO_Port, PEP_nSLEEP_Pin, GPIO_PIN_SET);
	HAL_Delay(10);


  if(!motor_init(&pep_motor)) {
  	printf("Failed to init PEP motor\n");
  } else {
  	printf("PEP motor initialized\n");
  }


  printf("Homing PEP motor\n");

	motor_enable(&pep_motor);
  motor_run(&pep_motor, PEP_DEC, 150 );
  while(HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin));
  motor_stop(&pep_motor);
  HAL_Delay(50);
  motor_run(&pep_motor, PEP_INC, 1000 );
  while(!HAL_GPIO_ReadPin(PEP_HOME_GPIO_Port, PEP_HOME_Pin));
  motor_stop(&pep_motor);

  motor_disable(&pep_motor);
  printf("PEP homed\n");


	printf("Scanning I2C bus\n");
	sensors_scan(&hi2c1);

	if(!sensors_init(&hi2c1)) {
		printf("I2C Error!!!\n");
		while(true);
	}

	printf("Testing Mat\n");
	for(int t=0; t<2; ++t) {
		mat_set_color(MAT_GREEN);
		HAL_Delay(500);
		mat_set_color(MAT_ORANGE);
		HAL_Delay(500);
		mat_set_color(MAT_RED);
		HAL_Delay(500);
	}
	mat_set_color(MAT_OFF);


	printf("Testing buzzer\n");


	printf("Testing Alarm LOW.    2 beeps every 30 seconds.\n----- Press button to skip\n");
	buzzer_set_level(BUZZER_LOW);
	wait_btn_clicked();

	printf("Testing Alarm MEDIUM. 3 beeps every 7.5 seconds\n----- Press button to skip\n");
	buzzer_set_level(BUZZER_MEDIUM);
	wait_btn_clicked();

	printf("Testing Alarm HIGH.  10 beeps every 2.5 seconds\n----- Press button to skip\n");
	buzzer_set_level(BUZZER_HIGH);
	wait_btn_clicked();

	buzzer_set_level(BUZZER_OFF);



	printf("Initialization completed...\n");

	printf("Press button to start\n");
	wait_btn_clicked();

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
		float d = compte_motor_step_time(t, 1., CALIBRATION_SPEED);
		Ti += d;
		//printf("d=%ld\n", (uint32_t)(d));
		bavu_motor_speed_table[t]= (uint32_t)d;
	}
	printf("Ti_predicted = %ld ms\n", (uint32_t)(Ti/1000));

	printf("Running\n");
	printf("Turn FS button off to stop the system.\n");

	motor_enable(&bavu_motor);
	bavu_motor_home(HOME_SPEED);
	ctrl_state = CTRL_STOPPED;

	while (failsafe_is_on()) {
		sensors_reset_volume();  	// Reset volume integrator
		ctrl_state= CTRL_INHALE;
		printf("INHALE\n");

	  // HIGH PEEP
		pep_valve_high();
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
		sensors_set_flow_callback(NULL); // Now we have flow data from begining of motor compression to thr end

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
		pep_valve_low();

		time= HAL_GetTick();
		_is_home = false;
		//************************************** EXPIRATION MOVEMENT ********************************* //
		motor_run(&bavu_motor, RELEASE, EXHALE_SPEED);
//************************************************* PID ZONE ********************************************//
		// Compute average flow and slope to adjust A_calibrated and B_calibrated
////		printf("Average flow=%lu slm\n", (uint32_t)(flow_avg));
////		float flow_error = flow_avg - flow_setpoint_slm;
////		B_calibrated += 0.01 * flow_error;
////		printf("error=%ld slm\n", (int32_t)(flow_error));
//		float P_plateau_slope = 0.1;
//		float P_plateau_mean = 0.2;
//		float timeStep = _logging_time_step_sum/_logging_index;
//		uint32_t low;
//		uint32_t high;
//		if(get_plateau(_inhalation_flow, _logging_index, timeStep, 10, &low, &high) == 0) {
//			printf("plateau found from sample %lu to %lu\n", low, high);
//		} else {
//			printf("plateau NOT found, considering from sample %lu to %lu\n", low, high);
//		}
//		float plateau_slope = linear_fit(_inhalation_flow+low, high-low-1, timeStep, &plateau_slope);
//		float plateau_mean = 0;
//		for(int i=low; i<high; i++) {
//			plateau_mean += _inhalation_flow[i];
//		}
//		plateau_mean = plateau_mean/(high-low);
//		printf("plateau slope : %ld\n",(int32_t)(1000*plateau_slope));
//		printf("plateau mean : %ld\n",(int32_t)(1000*plateau_mean));
//
//		float error_mean = plateau_mean - (flow_setpoint_slm/60.);
//
//		A_calibrated += plateau_slope * P_plateau_slope;
//		B_calibrated += error_mean * P_plateau_mean;
//		printf("A = %ld\n", (int32_t)(1000*A_calibrated));
//		printf("B = %ld\n", (int32_t)(1000*B_calibrated));
//
//		// Recompute motor steps
//		Ti = 0.;
//		for(long t=0; t<steps; ++t) {
//			float d = compte_motor_step_time(t, flow_setpoint_slm/60., CALIBRATION_SPEED);
//			Ti+=d;
//			//printf("d=%ld\n", (uint32_t)(d));
//			bavu_motor_speed_table[t]= (uint32_t)d;
//		}
//		printf("Ti_predicted = %ld ms\n", (uint32_t)(Ti/1000));
//*******************************************************************************************************//
		// Wait for motor to home.
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

	printf("Stopping reporting\n");
	reporting_stop();
	printf("Disabling bavu motor\n");
	motor_disable(&bavu_motor);
	printf("Stopping sensors\n");
	sensors_stop();

	printf("Done. Reset to restart\n");
	HAL_GPIO_WritePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin, GPIO_PIN_SET);
	while(true) {
		HAL_Delay(100);
		HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin);
		HAL_Delay(900);
		HAL_GPIO_TogglePin(NUCLEO_LED_GPIO_Port, NUCLEO_LED_Pin);
	}
}


void flow_callback(float flow, uint32_t delta_t_us) {
	if(_logging_index>=INHALATION_FLOW_ARRAY_SIZE) return;
	_inhalation_flow[_logging_index] = flow/60.;  // in sls
	_logging_time_step_sum += (float)delta_t_us/1000000;
	++_logging_index;
}



// Calibration speed is motor step time in seconds
// Desired flow is in sL/s
// Returns step time in us
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed) {
	float res = (0.8*A_calibrated*calibration_speed*calibration_speed*step_number) + B_calibrated * calibration_speed;
	res = res / desired_flow;
	if (res * 1000000 < 110) {return 110;}
	else {return res * 1000000.;}
}

int32_t calibration(float* A, float* B, uint8_t iterations) {
	float slope = 0; // slope of flow(t) cruve
	float originFlow = 0; // origin flow of flow(t) curve
	uint32_t steps;

	printf("Press button to calibrate.\n");
	wait_btn_clicked();

	// Calibrate slope
	printf("---------- Calibrating slope ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
		sensors_reset_volume();
		// HIGH PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(360/360.0);
		double speed = CALIBRATION_SPEED*1000000;
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(20/360.0);
//		motor_move(COMPRESS, speed, steps);
//		while(!motor_is_done());
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(180/360.0);
		motor_move(&bavu_motor, COMPRESS, speed, steps);
		HAL_Delay(200);
		// Start logging flow
		_logging_index = 0;
		_logging_time_step_sum = 0.;
		sensors_set_flow_callback(flow_callback);

//		reporting_start(100);
		while(motor_is_moving(&bavu_motor));
		// Stop logging flow
		sensors_set_flow_callback(NULL);

//		reporting_stop();
		motor_stop(&bavu_motor);
		HAL_Delay(500);
		float volumeIT = sensors_get_volume();
		printf("volume = %lu ml\n", (uint32_t)(1000*volumeIT));
//		// LOW PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW);
		_is_home = false;
		motor_run(&bavu_motor, RELEASE, HOME_SPEED);
		while(!bavu_motor_is_home());
		motor_stop(&bavu_motor);
		HAL_Delay(2000);

		float a = 0;
		float r = linear_fit(_inhalation_flow, _logging_index, _logging_time_step_sum/_logging_index, &a);
		printf("a=%lu\n", (uint32_t)(1000.*a));
		printf("r=%lu\n", (uint32_t)(1000.*r));
		slope += a / (float)iterations;
	}
	*A = slope;
	printf("A=%lu\n", (uint32_t)(1000.* *A));


	// Calibrate originFlow
	printf("---------- Calibrating B ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
		// HIGH PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_HIGH);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(360/360.0);
		sensors_reset_volume();
		motor_move(&bavu_motor, COMPRESS, CALIBRATION_SPEED*1000000., steps);
		while(motor_is_moving(&bavu_motor));
		motor_stop(&bavu_motor);
		HAL_Delay(2000);
		float volumeIT = sensors_get_volume();
		// LOW PEEP
		HAL_GPIO_WritePin(PEP_VALVE_GPIO_Port, PEP_VALVE_Pin, PEP_VALVE_LOW);
		_is_home=false;
		motor_run(&bavu_motor, RELEASE, HOME_SPEED);
		while(!bavu_motor_is_home());
		motor_stop(&bavu_motor);
		printf("volume = %luml\n", (uint32_t)(volumeIT*1000));
		float b = volumeIT/((CALIBRATION_SPEED) * steps) - (*A * (CALIBRATION_SPEED)*steps / 2.);
		printf("b=%ld\n", (int32_t)(1000*b));
		// Add values for averaging over iterations
		originFlow += b/(float)iterations;
	}
	*B = 1.5;
//	*B = 1.3;
	printf("B=%ld\n", (int32_t)(1000*(*B)));
	printf("Calibration...DONE\n");
	return 0;
}

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
	for(int i=0;i<samples_len;i++) {
		sumx  = sumx + (float)i * time_step_sec;
		sumx2 = sumx2 + (float)i*time_step_sec*(float)i*time_step_sec;
		sumy  = sumy + *(samples+i);
		sumy2 = sumy2 + (*(samples+i)) * (*(samples+i));
		sumxy = sumxy + (float)i*(time_step_sec)* (*(samples+i));
	}
	float denom = (samples_len * sumx2 - (sumx * sumx));
	if(denom == 0.) {
		printf("Calibration of A is not possible\n");
		return 1;
	}
	// compute slope a
	*slope = (samples_len * sumxy  -  sumx * sumy) / denom;
//	printf("%ld     ", (int32_t)(1000*((samples_len * sumxy  -  sumx * sumy) / denom)));

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}

int32_t get_plateau(float* samples, size_t samples_len, float time_step_sec, uint8_t windows_number, uint32_t* low_bound, uint32_t* high_bound){
	if(windows_number < 2 || windows_number > 30) {return -1;}
	float slopes[30];
	*high_bound = samples_len-1;
	// Compute slope for time windows to detect when signal start increasing/decreasing
	for(int window=0; window<windows_number; window++) {
		float r = linear_fit(samples+window*(samples_len/windows_number), samples_len/windows_number, time_step_sec, slopes+window);
		printf("%ld    ", (int32_t)(*(slopes+window) * 1000));
	}
	printf("\n");
	for(int window=1; window<windows_number; window++) {
		float delta_slope = slopes[window-1] - slopes[window];
		if(delta_slope > 1.) {
			*low_bound = (uint32_t)((samples_len/windows_number)*(window+1));
			printf("plateau begin at %lu over %lu points\n", *low_bound, (uint32_t)samples_len);
			return 0;
		}
	}
	*low_bound = (uint32_t)(samples_len/2);
	printf("No plateau found\n");
	return 1;
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

	//send_DATA(pressure, flow, volume, 0, 0);
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
	while(HAL_GPIO_ReadPin(NUCLEO_BTN_GPIO_Port, NUCLEO_BTN_Pin)==GPIO_PIN_SET);
	HAL_Delay(50);
	while(HAL_GPIO_ReadPin(NUCLEO_BTN_GPIO_Port, NUCLEO_BTN_Pin)!=GPIO_PIN_SET);
	HAL_Delay(50);
}

