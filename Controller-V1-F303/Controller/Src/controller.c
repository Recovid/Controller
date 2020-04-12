


#include "main.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>



extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;




// ------------------------------------------------------------
// Sensors
// ------------------------------------------------------------
typedef enum {
	STOPPED,
	REQ_SDP_MEASUREMENT,
	READ_SDP_MEASUREMENT,
	READ_NPA_MEASUREMENT
} sensor_state_t;

#define ADDR_SPD610 	((uint16_t)(0x40 <<1))
#define ADDR_NPA700B 	((uint16_t)(0x28 <<1))


const uint8_t _sdp_reset_req[1]  = { 0xFE };
const uint8_t _sdp_readAUR_req[1]  = { 0xE5 };
uint8_t _sdp_writeAUR_req[3]  = { 0xE4, 0x00, 0x00};

const uint8_t _sdp_measurement_req[1] 	= { 0xF1 };
uint8_t       _sdp_measurement_buffer[3] = { 0 };
uint8_t       _sdp_AUR_buffer[3] 		= { 0 };

uint8_t       _npa_measurement_buffer[2]	= { 0 };

volatile float _current_flow;
volatile float _current_pressure;
volatile float _current_volume;

volatile sensor_state_t _sensor_state;

volatile uint16_t _hyperfrish_sdp_time;
volatile uint16_t _hyperfrish_npa_time;


bool sensors_init();
void sensors_start();
void sensors_stop();
float get_flow() { return _current_flow; }
float get_pressure() { return _current_pressure; }
float get_volume() { return _current_volume; }

static void process_i2c_callback(I2C_HandleTypeDef *hi2c);



// ------------------------------------------------------------
// Python reporting
// ------------------------------------------------------------

static volatile uint8_t _reportTick;
void reporting_start(uint32_t period);
void report_send(void);
void reporting_stop();



// ------------------------------------------------------------
// Motor
// ------------------------------------------------------------

#define STEPS_PER_REVOLUTION (200*8*4)   // rev_steps*microsteps*reduction
#define MAX_SPEED				120									 // MAX speed = MIN step period
#define EXHALE_SPEED		200									 // RELEASE speed : release bag
#define STEP_PULSE			 15
#define HOME_SPEED			400									 // HOMEE speed
#define CALIBRATION_SPEED  (1./((STEPS_PER_REVOLUTION)*(200/360.0)))

typedef enum {
	RELEASE,
	COMPRESS
} motor_dir_t;

static volatile bool  		_is_home;
static volatile bool  		_motor_done;
static volatile uint16_t  _motor_steps;

static uint16_t  motor_speed_table[6400];

static bool motor_is_home() { return _is_home; }
static bool motor_is_done() { return _motor_done; }
static bool motor_steps() 	 { return _motor_steps; }

void motor_enable();
void motor_disable();
void motor_home(uint16_t speed);
void motor_run(motor_dir_t dir, uint16_t speed);
void motor_move(motor_dir_t dir, uint16_t speed, int32_t steps);
void motor_move_table(motor_dir_t dir, uint16_t* speed_table, uint16_t steps);
void motor_stop();


// ------------------------------------------------------------
// Miscellaneous
// ------------------------------------------------------------

void scan_I2C(void);

void wait_btn_clicked();
float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope);

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


float Ti;				// inhaleTime
float Va= 200; 	// Volume angle

float inhalation_flow[200];
float inhalation_volume[200];


int32_t calibration(float* A, float* B);
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed);
float A_calibrated = 0.917;
float B_calibrated = 1.;
//uint32_t min_angle = 10;

float flow_setpoint_slm = 60;
void controller_run() {
	uint32_t time;
	uint16_t steps;
	uint32_t t;
	_current_volume = 0.;

	printf("Recovid-F303\n");

	printf("Press button HOME.\n");
	wait_btn_clicked();

	motor_enable();
	motor_home(HOME_SPEED);

	htim3.Instance->CNT= 0;
	HAL_TIM_Base_Start(&htim3);
//	HAL_TIM_Base_Stop(&htim3);



//	uint32_t nb_steps= (uint16_t) (STEPS_PER_REVOLUTION)*(200/360.0);
//	double d = MAX_SPEED;
//	double Ti=d;
//	double max = 0;
//	for(long t=0; t<nb_steps; ++t) {
//			d+=(0.15*sqrt((double)(t)/nb_steps));
//			Ti+=d;
//			if(d>max) max=d;
//			motorSpeedTable[t]= (uint32_t)d;
//	}
//
//	printf("Ti=%ld\n", (uint32_t)(Ti/1000));
//	printf("period max=%ld\n", (uint32_t)max);
//
//	motor_move_table(CCW, motorSpeedTable, nb_steps);
//
//	while(!motor_is_done());
//	motor_stop();
//
//	printf("DONE\n");
//	while(1);

	printf("Homed.\n");

	if(!sensors_init()) {
		printf("I2C Error!!!\n");
		while(true);
	}
	sensors_start();

//	while(true){
//		HAL_Delay(250);
//		printf("NPA : %lu\n", _hyperfrish_npa_time);
//		printf("SDP : %lu\n", _hyperfrish_sdp_time);
//	}

	// Calibration
	calibration(&A_calibrated, &B_calibrated);

	Va= 200;
	steps= (uint32_t) (STEPS_PER_REVOLUTION)*(Va/360.0);
	double d=MAX_SPEED;
	double max=0;
	Ti=d;

/*	for(long t=0; t<steps; ++t) {
		d+=(0.15*sqrt((double)(t)/steps));
		Ti+=d;
		if(d>max) max=d;
		motor_speed_table[t]= (uint32_t)d;
	}  */
	for(long t=0; t<steps; ++t) {
		d = compte_motor_step_time(t, 1., CALIBRATION_SPEED);
		Ti+=d;
		//printf("d=%ld\n", (uint32_t)(d));
		motor_speed_table[t]= (uint32_t)d;
	}

	printf("Ti=%ld\n", (uint32_t)(Ti/1000));
	printf("period max=%ld\n", (uint32_t)max);

	printf("Press btn to start\n");
	wait_btn_clicked();

	printf("Running\n");

	motor_enable();
	motor_home(HOME_SPEED);
	//motor_move(COMPRESS, HOME_SPEED, (uint32_t) (STEPS_PER_REVOLUTION)*(min_angle/360.0));
	while(!motor_is_done());
	motor_stop();
	ctrl_state = STOPPED;

	reporting_start(80);
	float flow_avg;
	while (1) {
		_current_volume = 0.;
		flow_avg = 0.;
		uint32_t t = 0;
		ctrl_state= CTRL_INHALE;
		printf("INHALE\n");

	  // HIGH PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, RESET);

		time= HAL_GetTick();

		//##- Start DMA Burst transfer
		motor_move_table(COMPRESS, motor_speed_table, steps);
		// Wait for motion end.
		HAL_Delay(300);
		//reporting_start(70);
		while(!motor_is_done()) {
			HAL_Delay(10);
			flow_avg += get_flow();
			++t;
		}
		motor_stop();
		float temp = get_flow();
		while(temp > 0.6 * flow_avg/(float)t){
			HAL_Delay(10);
			flow_avg += get_flow();
			++t;
			temp = get_flow();
		}
		//reporting_stop();

		time= HAL_GetTick() -time;
		printf("time=%lu\n", time);

		// Inhalation pause : Tplat
		ctrl_state= CTRL_EXHALE;
		printf("TPLAT\n");
		HAL_Delay(700);
		float volume_cycle = get_volume();
		printf("Vi = %d ml\n", (uint32_t)(volume_cycle * 1000));
		ctrl_state= CTRL_EXHALE;
		printf("EXHALE\n");

		// LOW PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, SET);

		time= HAL_GetTick();
		_is_home=false;
		motor_run(RELEASE, EXHALE_SPEED);

		// Compute average flow and adjust B
		flow_avg = flow_avg / (float)t;
		printf("Average flow=%d slm\n", (uint32_t)(flow_avg));
		float flow_error = flow_avg - flow_setpoint_slm;
		B_calibrated += 0.01 * flow_error;
		printf("error=%d slm\n", (int32_t)(flow_error));
		printf("B=%d\n", (int32_t)(B_calibrated));

		// home will be set in EXTI interrupt handler. PWM will also be stopped.
		while(!motor_is_home());
		motor_stop();
		HAL_Delay(1000);
		//motor_move(COMPRESS, HOME_SPEED, (uint32_t) (STEPS_PER_REVOLUTION)*(min_angle/360.0));
		time= HAL_GetTick() - time;
		if(time<2000-1) {
			HAL_Delay(2000-time);
		}
		for(long t=0; t<steps; ++t) {
			d = compte_motor_step_time(t, flow_setpoint_slm/60., CALIBRATION_SPEED);
			Ti+=d;
			//printf("d=%ld\n", (uint32_t)(d));
			motor_speed_table[t]= (uint32_t)d;
		}

		printf("Ti=%ld\n", (uint32_t)(Ti/1000));
		printf("period max=%ld\n", (uint32_t)max);

		printf("Cycle is_done\n");
	}
	// Disable motor
	reporting_stop();
	motor_disable();
}

// Calibration speed is motor step time in seconds
// Desired flow is in sL/s
float compte_motor_step_time(long step_number, float desired_flow, double calibration_speed) {
	float res = (0.8*A_calibrated*calibration_speed*calibration_speed*step_number) + B_calibrated * calibration_speed;
	res = res / desired_flow;
	if (res * 1000000 < 150) {return 150;}
	else {return res * 1000000.;}
}

int32_t calibration(float* A, float* B) {
	uint16_t iterations = 4;
	uint32_t sensor_period = 10;  // period of sensor fetching in ms
	float slope = 0; // slope of flow(t) cruve
	float originFlow = 0; // origin flow of flow(t) curve
	float correlation_coeff = 0;
	uint32_t steps;
	uint32_t t;
	uint32_t time;

	printf("Press button to calibrate.\n");
	wait_btn_clicked();

	// Calibrate slope
	printf("---------- Calibrating slope ---------------\n");
	reporting_start(100);
	for(int iter=0; iter<iterations; ++iter) {
		_current_volume = 0.;
		// HIGH PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, RESET);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(200/360.0);
		double speed = CALIBRATION_SPEED*1000000;
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(20/360.0);
//		motor_move(COMPRESS, speed, steps);
//		while(!motor_is_done());
//		reporting_start(100);
//		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(180/360.0);
		motor_move(COMPRESS, speed, steps);
		t=0;
		while(!motor_is_done()) {
			HAL_Delay(sensor_period);
			inhalation_flow[t] = get_flow()/60.;
			++t;
		}
//		reporting_stop();
//		// LOW PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, SET);
		_is_home=false;
		motor_run(RELEASE, HOME_SPEED);
		while(!motor_is_home());
		motor_stop();
		HAL_Delay(2000);
		float volumeIT = get_volume();
		printf("volume = %luml\n", (uint32_t)(1000*volumeIT));

		float a = 0;
		float r = linear_fit(inhalation_flow, t, 0.01, &a);
		printf("a=%lu\n", (uint32_t)(1000.*a));
		printf("r=%lu\n", (uint32_t)(1000.*r));
		slope += a / (float)iterations;
	}
	*A = slope;
//	reporting_stop();
	printf("A=%lu\n", (uint32_t)(1000.* *A));


	// Calibrate originFlow
	printf("---------- Calibrating B ---------------\n");
	for(int iter=0; iter<iterations; ++iter) {
		// HIGH PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, RESET);
		steps= (uint32_t) (STEPS_PER_REVOLUTION)*(200/360.0);
//		reporting_start(100);
		_current_volume = 0.;
		motor_move(COMPRESS, CALIBRATION_SPEED*1000000., steps);
		while(!motor_is_done());
		motor_stop();
		HAL_Delay(2000);
		float volumeIT = get_volume();
		// LOW PEEP
		HAL_GPIO_WritePin(PEEP_GPIO_Port, PEEP_Pin, SET);
		_is_home=false;
		motor_run(RELEASE, HOME_SPEED);
		while(!motor_is_home());
		motor_stop();
		printf("volume = %dml\n", (uint32_t)(volumeIT*1000));
		float b = volumeIT/((CALIBRATION_SPEED) * steps) - (*A * (CALIBRATION_SPEED)*steps / 2.);
		printf("b=%d\n", (int32_t)(1000*b));
		// Add values for averaging over iterations
		originFlow += b/(float)iterations;
	}
	*B = 1.;
//	*B = 1.3;
	printf("B=%d\n", (int32_t)(1000*(*B)));
	printf("Calibration...DONE\n");
	reporting_stop();
	return 0;
}

// Compute slope of samples fetched with specified time_step
// Returns 	R  if fit is ok
// 			-1 if fit is not possible
float linear_fit(float* samples, size_t samples_len, float time_step_sec, float* slope){
	float sumx=0,sumy=0,sumxy=0,sumx2=0, sumy2=0;
	float* p = NULL;
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

	// compute correlation coefficient
	return (sumxy - sumx * sumy / samples_len) / sqrtf((sumx2 - (sumx*sumx)/samples_len) * (sumy2 - (sumy*sumy)/samples_len));
}


bool sensors_init() {
	HAL_Delay(100);
	// First reset SDP
	if(HAL_I2C_Master_Transmit(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_reset_req, sizeof(_sdp_reset_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	// Now read sdp advanced user register
	if(HAL_I2C_Master_Transmit(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_readAUR_req, sizeof(_sdp_readAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	HAL_Delay(100);
	if(HAL_I2C_Master_Receive(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_AUR_buffer, sizeof(_sdp_AUR_buffer), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// print AUR (Advances User Register)
	uint16_t sdp_aur = (uint16_t)((_sdp_AUR_buffer[0] << 8) | _sdp_AUR_buffer[1]);
	printf("sdp AUR = %d\n", (uint16_t)(sdp_aur));
	uint16_t sdp_aur_no_i2c_hold = sdp_aur & 0xFFFD;
	_sdp_writeAUR_req[1] = (uint16_t)(sdp_aur_no_i2c_hold >> 8);
	_sdp_writeAUR_req[2] = (uint16_t)(sdp_aur_no_i2c_hold & 0xFF);
	// Now disable i2c hold master mode
	if(HAL_I2C_Master_Transmit(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_writeAUR_req, sizeof(_sdp_writeAUR_req), 1000 )!= HAL_I2C_ERROR_NONE) {
		return false;
	}
	// Sensors settle time
	HAL_Delay(100);
	return true;
}

void sensors_start() {
	// Start sensor state machine.
	// This state machine is managed in the I2C interupt routine.
	_sensor_state= REQ_SDP_MEASUREMENT;
	HAL_I2C_Master_Transmit_DMA(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
}

void sensors_stop() {
	_sensor_state= STOPPED;
}

void motor_enable() { 	// Enable motor
	HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_SET);
}

void motor_disable() { 	// Enable motor
	HAL_GPIO_WritePin(MOTOR_ENA_GPIO_Port, MOTOR_ENA_Pin, GPIO_PIN_RESET);
}

void motor_home(uint16_t speed) {
	if(HAL_GPIO_ReadPin(HOME_GPIO_Port, HOME_Pin)==GPIO_PIN_RESET)	return;

	// Move 10° inhalewise to make sure (almost) that we're on the right side of the switch !!
	uint16_t nb_steps= (uint16_t) (STEPS_PER_REVOLUTION)*(20/360.0);
	motor_move(COMPRESS, HOME_SPEED, nb_steps);
	while(!motor_is_done());

	HAL_Delay(200);

	// Move exhalewise until the switch triggers
  _is_home = false;
  motor_run(RELEASE, speed);
	// home will be set in EXTI interrupt handler.
	while(!motor_is_home());
	motor_stop();
}

void motor_run(motor_dir_t dir, uint16_t speed) {
	motor_move(dir, speed, -1);
}

void motor_move(motor_dir_t dir, uint16_t speed, int32_t steps) {
	if(steps==0) {
		_motor_done=true;
		return;
	}

	_motor_done=false;
	HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, dir);
	_motor_steps= steps;
	htim17.Init.Period = speed;
  HAL_TIM_Base_Init(&htim17);
	HAL_TIM_PWM_Start_IT(&htim17, TIM_CHANNEL_1);
}

void motor_move_table(motor_dir_t dir, uint16_t* speed_table, uint16_t table_size) {
	if(table_size==0) {
		_motor_done=true;
		return;
	}
	if(table_size==1) {
		motor_move(dir, speed_table[0], 1);
		return;
	}

	_motor_done=false;
	HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, dir);
	htim17.Init.Period = speed_table[0];
  HAL_TIM_Base_Init(&htim17);
	HAL_TIM_DMABurst_MultiWriteStart(&htim17, TIM_DMABASE_ARR, TIM_DMA_UPDATE,	(uint32_t*)&speed_table[1], TIM_DMABURSTLENGTH_1TRANSFER, table_size-1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

}

void motor_stop() {
//	HAL_TIM_PWM_Stop_DMA(&htim17, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Stop_IT(&htim17, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	_motor_done=true;
}


static uint8_t* SYNC="---START---\r\n";

void reporting_start(uint32_t ms) {
	HAL_UART_Transmit_IT(&huart2, SYNC, strlen(SYNC));
	htim2.Init.Period = ms*1000;
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
}

static uint8_t data_buffer[13] = { '>', 0,0,0,0,0,0,0,0,0,0,0,0 };

void report_send() {
	uint8_t* ptr= (uint8_t*)&_current_flow;
	data_buffer[1]= *ptr++;
	data_buffer[2]= *ptr++;
	data_buffer[3]= *ptr++;
	data_buffer[4]= *ptr++;

	ptr= (uint8_t*)&_current_pressure;
	data_buffer[5]= *ptr++;
	data_buffer[6]= *ptr++;
	data_buffer[7]= *ptr++;
	data_buffer[8]= *ptr++;

	ptr= (uint8_t*)&_current_volume;
	data_buffer[9]= *ptr++;
	data_buffer[10]= *ptr++;
	data_buffer[11]= *ptr++;
	data_buffer[12]= *ptr++;

	HAL_UART_Transmit_IT(&huart1, data_buffer, 13);
}

void reporting_stop() {
	HAL_TIM_Base_Stop(&htim2);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim17) {
		_motor_done=true;
	} else if(htim==&htim2) {
		report_send();
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if(htim==&htim17) {
		if(motor_steps>0) {
			if(_motor_steps==1) {
				// Stop PWM
				HAL_TIM_PWM_Stop_IT(&htim17, TIM_CHANNEL_1);
			// Indicate DMA Burst done
				_motor_done=true;
			}
			--_motor_steps;
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin==HOME_Pin) {
		_is_home=true;
	}
}



void process_i2c_callback(I2C_HandleTypeDef *hi2c) {
	static	uint16_t hyperfrish_npa;
	static	uint16_t hyperfrish_sdp;

	switch (_sensor_state) {
	case STOPPED:
		return;
	case READ_NPA_MEASUREMENT:
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
//			_sensor_state= STOPPED;
			HAL_I2C_Master_Receive_IT(&hi2c1, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		if( (_npa_measurement_buffer[0]>>6)==0) {
			_hyperfrish_npa_time = (uint16_t)htim3.Instance->CNT - hyperfrish_npa;

			hyperfrish_npa = (uint16_t)htim3.Instance->CNT;
			uint16_t praw =  (((uint16_t)_npa_measurement_buffer[0]) << 8 | _npa_measurement_buffer[1]) & 0x3FFF;
			_current_pressure = 70.307 * ((float) ( praw - 1638.)/13107.);
		} else if((_npa_measurement_buffer[0]>>6)==3) {
			// TODO: Manage error status !!
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(&hi2c1, ADDR_SPD610, (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );

		break;

	case REQ_SDP_MEASUREMENT:
		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
//			_sensor_state= STOPPED;
			HAL_I2C_Master_Transmit_IT(&hi2c1, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		_sensor_state= READ_SDP_MEASUREMENT;
		HAL_I2C_Master_Receive_DMA(&hi2c1, ADDR_SPD610 , (uint8_t*) _sdp_measurement_buffer, sizeof(_sdp_measurement_buffer) );
		break;

	case READ_SDP_MEASUREMENT:

		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
			_sensor_state= READ_NPA_MEASUREMENT;
			HAL_I2C_Master_Receive_DMA(&hi2c1, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
			break;
		}
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE) {
			// TODO: Manage error
			_sensor_state= STOPPED;
			break;
		}
		if(_sdp_measurement_buffer[0] != 0xFF || _sdp_measurement_buffer[1] != 0xFF || _sdp_measurement_buffer[2] != 0xFF){
			_hyperfrish_sdp_time= (uint16_t)htim3.Instance->CNT - hyperfrish_sdp;
			hyperfrish_sdp = (uint16_t)htim3.Instance->CNT;
			int16_t dp_raw   = (int16_t)((((uint16_t)_sdp_measurement_buffer[0]) << 8) | (uint8_t)_sdp_measurement_buffer[1]);
			_current_flow = -((float)dp_raw)/105.0;
			_current_volume += (_current_flow/60.) * ((float)_hyperfrish_sdp_time/1000000);
			_sensor_state= REQ_SDP_MEASUREMENT;
			HAL_I2C_Master_Transmit_DMA(&hi2c1, ADDR_SPD610, (uint8_t*) _sdp_measurement_req, sizeof(_sdp_measurement_req) );
		} else {
			_sensor_state= READ_NPA_MEASUREMENT;
			HAL_I2C_Master_Receive_DMA(&hi2c1, ADDR_NPA700B , (uint8_t*) _npa_measurement_buffer, sizeof(_npa_measurement_buffer) );
		}
		break;
	}
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
		process_i2c_callback(hi2c);
	}
}


void wait_btn_clicked() {
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_SET);
	HAL_Delay(50);
	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)!=GPIO_PIN_SET);
	HAL_Delay(50);
}

void scan_I2C(void) {

	for (int t = 1; t < 127; ++t) {
//		uint32_t time= HAL_GetTick();
		if (HAL_I2C_Master_Transmit_IT(&hi2c1, t /*I2C_ADDRESS*/, (uint8_t*) NULL, 0 /*TXBUFFERSIZE*/) != HAL_OK) {
			/* Error_Handler() function is called when error occurs. */
			Error_Handler();
		}

		/*##-3- Wait for the end of the transfer #################################*/
		/*  Before starting a new communication transfer, you need to check the current
		 state of the peripheral; if it’s busy you need to wait for the end of current
		 transfer before starting a new one.
		 For simplicity reasons, this example is just waiting till the end of the
		 transfer, but application may perform other tasks while transfer operation
		 is ongoing. */
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}
//		time= HAL_GetTick()- time;
//		printf("Time: %lu\n", time);

		if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_NONE) {
			printf("Found device at address: %02X\n", t);
		}

		/* When Acknowledge failure occurs (Slave don't acknowledge it's address)
		 Master restarts communication */
	}
//	while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
	printf("Done...\n");

}


#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}



