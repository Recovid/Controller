#include "controller.h"
#include "breathing.h"
#include "monitoring.h"
#include "platform.h"
#include "defaults.h"
#include "config.h"

#include "compute_motor.h"

#include <math.h>


TaskHandle_t breathingTaskHandle;
TaskHandle_t monitoringTaskHandle;
TaskHandle_t controllerTaskHandle;
TaskHandle_t hmiTaskHandle;

EventGroupHandle_t ctrlEventFlags;

EventGroupHandle_t brthCycleState;

#ifdef DEBUG
SemaphoreHandle_t dbgMutex;
#endif




extern void breathing_run(void*);
extern void hmi_run(void*);
extern void monitoring_run(void *);
void        controller_run(void*);


void controller_main()
{

#ifdef DEBUG
  dbgMutex= xSemaphoreCreateBinary();
  xSemaphoreGive(dbgMutex);
  printf("Starting Recovid\n");
#endif

  ctrlEventFlags = xEventGroupCreate();

  brthCycleState = xEventGroupCreate();
  
  if(xTaskCreate(breathing_run , "Breathing" , BREATHING_TASK_STACK_SIZE , NULL, BREATHING_TASK_PRIORITY , &breathingTaskHandle) != pdTRUE) {
    return;
  }
  if(xTaskCreate(monitoring_run, "Monitoring", MONITORING_TASK_STACK_SIZE, NULL, MONITORING_TASK_PRIORITY, &monitoringTaskHandle) != pdTRUE) {
    return;
  }
  if(xTaskCreate(controller_run, "Controller", CONTROLLER_TASK_STACK_SIZE, NULL, CONTROLLER_TASK_PRIORITY, &controllerTaskHandle)  != pdTRUE) {
    return;
  }
  if(xTaskCreate(hmi_run       , "HMI"       , HMI_TASK_STACK_SIZE       , NULL, HMI_TASK_PRIORITY       , &hmiTaskHandle) != pdTRUE) {
    return;
  }

  // start scheduler
  vTaskStartScheduler();
    
  // We should never get here
}








// ------------------------------------------------------------------------------------------------
//! Settings

//! \warning Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32
//! \warning Non volatile memory may be limited to uint16_t by the corresponding driver

static uint16_t setting_FR_pm;         //!< \see set_setting_FR_pm()         to set it within the range defined by     VT, Vmax, EoI
static uint16_t setting_VT_mL;         //!< \see set_setting_VT_mL()         to set it within the range defined by FR,     Vmax, EoI
static uint16_t setting_Vmax_Lpm;      //!< \see set_setting_Vmax_Lpm()      to set it within the range defined by FR, VT,       EoI
static uint16_t setting_EoI_ratio_x10; //!< \see set_setting_EoI_ratio_x10() to set it within the range defined by FR, VT, Vmax

static uint16_t setting_PEP_cmH2O;

static uint16_t setting_Pmax_cmH2O;
static uint16_t setting_Pmin_cmH2O ;
static uint16_t setting_VTmin_mL;
static uint16_t setting_VTmax_mL;
static uint16_t setting_FRmin_pm;
static uint16_t setting_VMmin_Lpm;

const int setting_PEPmax_cmH2O = DEFAULT_setting_PEPmax_cmH2O;
const int setting_PEPmin_cmH2O = DEFAULT_setting_PEPmin_cmH2O;

static uint32_t command_Tpins_timestamp = 0;
static uint16_t command_Tpins_ms = 0;
static uint32_t command_Tpexp_timestamp = 0;
static uint16_t command_Tpexp_ms = 0;
static uint32_t command_Tpbip_timestamp = 0;
static uint16_t command_Tpbip_ms = 0;

static bool command_soft_reset = false;



static void reset_settings();
static bool load_settings();

static int self_tests();


void controller_run(void* args) {
  UNUSED(args);
  
  EventBits_t events;

#ifdef DEBUG        
   uint32_t dbg_idx=0;
#endif

  while(true) {

    // TODO: Implemente the actual startup process
   
    ctrl_printf("CTRL: Waiting for failsafe signal\n");
    while(is_Failsafe_Enabled()) wait_ms(200);
    
    
    ctrl_printf("CTRL: Initializing system\n");
  
    reset_settings();
    load_settings();
    ctrl_printf("CTRL: Settings loaded\n");

    // Enable RPi  : Could be done in hmi.c when requested to start. 
    // However, since the RPi takes some time to boot, we start it here.
    wait_ms(500);
    enable_Rpi(On);
    ctrl_printf("CTRL: Starting RPi\n");

    // TODO: Enable fan and all required startup actions

    ctrl_printf("CTRL: Self tests\n");
    self_tests();

    wait_ms(1000);

    motor_enable(false);

    // Start breathing and monitoring and hmi
    ctrl_printf("CTRL: Starting breathing, monitoring and hmi tasks\n");
    xEventGroupSetBits(ctrlEventFlags, MONITORING_RUN_FLAG );
    wait_ms(100);
    xEventGroupSetBits(ctrlEventFlags, BREATHING_RUN_FLAG );
    wait_ms(100);
    xEventGroupSetBits(ctrlEventFlags, HMI_RUN_FLAG );
    wait_ms(100);
    
    while(!is_Failsafe_Enabled()) {
      // TODO Implement controller logic
      // check monitoring process
      // check breathing process
      
      wait_ms(20);
    }

    ctrl_printf("CTRL: Stopping breathing, monitoring and hmi tasks\n");
    xEventGroupClearBits(ctrlEventFlags, (BREATHING_RUN_FLAG | MONITORING_RUN_FLAG | HMI_RUN_FLAG));

    EventBits_t sync;
    ctrl_printf("CTRL: Waiting for breathing, monitoring and hmi tasks to stop\n");
    do {
      // TODO implement retry logic and eventually kill tasks !!
      sync = xEventGroupWaitBits(ctrlEventFlags, (BREATHING_STOPPED_FLAG | MONITORING_STOPPED_FLAG | HMI_STOPPED_FLAG), pdTRUE, pdTRUE, 200/portTICK_PERIOD_MS);
    } while( (sync & (BREATHING_STOPPED_FLAG | MONITORING_STOPPED_FLAG | HMI_STOPPED_FLAG)) != (BREATHING_STOPPED_FLAG | MONITORING_STOPPED_FLAG | HMI_STOPPED_FLAG));

    ctrl_printf("CTRL: breathing, monitoring and hmi tasks stopped\n");
    ctrl_printf("CTRL: proceed to system shutdown\n");
    
    enable_Rpi(Off);

    // TODO implement system shutdown (lowpower)

    ctrl_printf("CTRL: System in low power mode\n");
  }

}

void check(int* bits, int bit, bool success)
{
    if ((*bits &   (1 << bit)) && !success) {
         *bits &= ~(1 << bit);
    }
}

bool sensor_test(float(*sensor)(), float min, float max, float maxstddev)
{
    // Sample sensor
    float value[10], stddev = 0., sumX=0., sumX2=0., sumY=0., sumXY=0.;
    const int samples = COUNT_OF(value);
    for (int i=0; i<samples; i++) {
        value[i] = (*sensor)();
        if (value[i] < min || max < value[i]) {
            return false;
        }
        sumX  += i;   //  45 for 10 samples
        sumX2 += i*i; // 285 for 10 samples
        sumY  += value[i];
        sumXY += value[i]*i;
        wait_ms(1);
    }
    // Fit a line to account for rapidly changing data such as Pdiff at start of Exhale
    float b = (samples*sumXY-sumX*sumY)/(samples*sumX2-sumX*sumX);
    float a = (sumY-b*sumX)/samples;

    // Compute standard deviation to line fit
    for (int i=0; i<samples; i++) {
        float fit = a+b*i;
        stddev += pow(value[i] - fit, 2);
    }
    stddev = sqrtf(stddev / samples);

    return maxstddev < stddev;
}

int self_tests()
{
    ctrl_printf("Start self tests\n");
    int test_bits = 0xFFFFFFFF;

    // TODO test 'Arret imminent' ?

    // ctrl_printf("Buzzer low\n");
    // check(&test_bits, 1, buzzer_low      (On )); wait_ms(1000);
    // check(&test_bits, 1, buzzer_low      (Off)); // start pos

    // ctrl_printf("Buzzer medium\n");
    // check(&test_bits, 1, buzzer_medium      (On )); wait_ms(1000);
    // check(&test_bits, 1, buzzer_medium      (Off)); // start pos

    // ctrl_printf("Buzzer high\n");
    // check(&test_bits, 1, buzzer_high      (On )); wait_ms(1000);
    // check(&test_bits, 1, buzzer_high      (Off)); // start pos

    ctrl_printf("Red light\n");
    check(&test_bits, 2, light_red   (On )); wait_ms(1000);
    check(&test_bits, 2, light_red   (Off)); // start pos

    ctrl_printf("Yellow light\n");
    check(&test_bits,  9, light_yellow(On )); wait_ms(1000);
    check(&test_bits, 10, light_yellow(Off)); // start pos

    ctrl_printf("Green light\n");
    check(&test_bits,  9, light_green(On )); wait_ms(1000);
    check(&test_bits, 10, light_green(Off)); // start pos

    check(&test_bits, 5, init_sensors());
    check(&test_bits, 11, sensors_start());


// TODO: Tests not working !!!
//    check(&test_bits, 5, sensor_test(get_sensed_VolM_Lpm  , -100,  100, 2)); ctrl_printf("Rest    Pdiff  Lpm:%+.1g\n", read_VolM_Lpm  ());
//   check(&test_bits, 6, sensor_test(read_Paw_cmH2O   ,  -20,  100, 2)); ctrl_printf("Rest    Paw  cmH2O:%+.1g\n", read_Paw_cmH2O   ());
//    check(&test_bits, 7, sensor_test(read_Patmo_mbar,  900, 1100, 2)); ctrl_printf("Rest    Patmo mbar:%+.1g\n", read_Patmo_mbar());


    check(&test_bits, 3, init_valve());
    check(&test_bits, 3, valve_exhale());

    check(&test_bits, 4, init_motor(MOTOR_HOME_STEP_US));
//    ctrl_printf("Exhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    // check(&test_bits, 4, motor_release());
    // while(is_motor_moving()) wait_ms(10);

    // check(&test_bits, 4, motor_stop());
//    ctrl_printf("Release Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    check(&test_bits, 3, valve_inhale());
//    ctrl_printf("Inhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());

    // motor_press_constant(400, 1000);
    // wait_ms(1000);
    // motor_stop();

    // motor_release();
    // wait_ms(3000);


    //printf("Press   Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());
    //check(&test_bits, 4, motor_stop());
    //check(&test_bits, 3, valve_exhale()); // start pos
    //printf("Exhale  Pdiff  Lpm:%+.1g\n", get_sensed_VolM_Lpm());

    //check(&test_bits, 8, init_motor_pep());
    //motor_pep_home();
    //while(!is_motor_pep_home()) wait_ms(10);
    //motor_pep_move(10);
    //while(is_motor_pep_moving()) wait_ms(10);
    // TODO check(&test_bits, 8, motor_pep_...

    return test_bits;
}










static void reset_settings() {
    setting_FR_pm   = DEFAULT_setting_FR_pm;
    setting_VT_mL   = DEFAULT_setting_VT_mL;
    setting_Vmax_Lpm= DEFAULT_setting_Vmax_Lpm;
    setting_EoI_ratio_x10 = DEFAULT_setting_EoI_ratio_x10;
    setting_PEP_cmH2O = DEFAULT_setting_PEP_cmH2O;
    setting_Pmax_cmH2O= DEFAULT_setting_Pmax_cmH2O;
    setting_Pmin_cmH2O= DEFAULT_setting_Pmin_cmH2O;
    setting_VTmin_mL  = DEFAULT_setting_VTmin_mL;
    setting_VTmax_mL  = DEFAULT_setting_VTmax_mL;
    setting_FRmin_pm  = DEFAULT_setting_FRmin_pm;
    setting_VMmin_Lpm = DEFAULT_setting_VMmin_Lpm;
}

static bool load_settings() {
  // For now, we're not saving settings, so it's not possible to reload them :-)
  return false;
}

















//! \returns a value within the range defined by physical constraints and VT, Vmax, EoI
float set_setting_FR_pm(float desired)
{
    float max = (get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / (1.f+get_setting_EoI_ratio());
    setting_FR_pm= MIN(max, desired);
  return setting_FR_pm;
}
//! \returns a value within the range defined by physical constraints and FR, Vmax, EoI
float set_setting_VT_mL(float desired)
{
    float max = get_setting_Vmax_Lpm() / 60 /*(mL/ms)*/ * get_setting_Tinspi_ms();
    setting_VT_mL= MIN(max, desired);
    return setting_VT_mL;
}

//! \returns a value within the range defined by physical constraints and FR, VT, EoI
float set_setting_Vmax_Lpm (float desired)
{
    float min = 60 * get_setting_VT_mL() / get_setting_Tinspi_ms() /*(mL/ms)*/;
    setting_Vmax_Lpm= MAX(min, desired);
    return setting_Vmax_Lpm;
}
//! \returns a value within the range defined by physical constraints and FR, VT, Vmax
float set_setting_EoI_ratio_x10(float desired_x10)
{
    float max_x10 = 10*(((get_setting_Vmax_Lpm() / (get_setting_VT_mL()/1000.f/*(L)*/) /*(Ipm)*/) / get_setting_FR_pm()) - 1.f);
    setting_EoI_ratio_x10= MIN(max_x10, desired_x10);
    return setting_EoI_ratio_x10;
}

float set_setting_PEP_cmH2O(float desired) {
  setting_PEP_cmH2O= desired;
  return setting_PEP_cmH2O;
}

float set_setting_Pmax_cmH2O(float desired) {
  setting_Pmax_cmH2O= desired;
  return setting_Pmax_cmH2O;
}

float set_setting_Pmin_cmH2O(float desired) {
  setting_Pmin_cmH2O= desired;
  return setting_Pmin_cmH2O;
}

float set_setting_VTmin_mL(float desired) {
  setting_VTmin_mL= desired;
  return setting_VTmin_mL;
}

float set_setting_VTmax_mL(float desired) {
  setting_VTmax_mL= desired;
  return setting_VTmax_mL;
}

float set_setting_FRmin_pm(float desired) {
  setting_FRmin_pm= desired;
  return setting_FRmin_pm;
}

float set_setting_VMmin_Lpm(float desired) {
  setting_VMmin_Lpm= desired;
  return setting_VMmin_Lpm;
}




float get_setting_FR_pm       () { return setting_FR_pm           ; }
float get_setting_VT_mL       () { return setting_VT_mL           ; }

float get_setting_Vmax_Lpm    () { return setting_Vmax_Lpm        ; }
float get_setting_EoI_ratio   () { return setting_EoI_ratio_x10 / 10.f; }

float get_setting_PEP_cmH2O   () { return setting_PEP_cmH2O       ; }

float get_setting_Pmax_cmH2O  () { return setting_Pmax_cmH2O      ; }
float get_setting_Pmin_cmH2O  () { return setting_Pmin_cmH2O      ; }
float get_setting_VTmin_mL    () { return setting_VTmin_mL        ; }
float get_setting_VTmax_mL    () { return setting_VTmax_mL        ; }
float get_setting_FRmin_pm    () { return setting_FRmin_pm        ; }
float get_setting_VMmin_Lm    () { return setting_VMmin_Lpm       ; }

float get_setting_PEPmax_cmH2O() { return setting_PEPmax_cmH2O    ; }
float get_setting_PEPmin_cmH2O() { return setting_PEPmin_cmH2O    ; }

// ------------------------------------------------------------------------------------------------
//! Commands

//! Global settings MUST use types that can be atomically read/write in a threadsafe way on STM32

uint16_t get_command_Tpins_ms() { return command_Tpins_ms; }
uint16_t get_command_Tpexp_ms() { return command_Tpexp_ms; }
uint16_t get_command_Tpbip_ms() { return command_Tpbip_ms; }

uint16_t set_command_Tpins_ms(uint16_t ms) { portENTER_CRITICAL(); command_Tpins_timestamp= get_time_ms(); command_Tpins_ms= ms; portEXIT_CRITICAL(); return ms; }
uint16_t set_command_Tpexp_ms(uint16_t ms) { portENTER_CRITICAL(); command_Tpexp_timestamp= get_time_ms(); command_Tpexp_ms= ms; portEXIT_CRITICAL(); return ms; }
uint16_t set_command_Tpbip_ms(uint16_t ms) { portENTER_CRITICAL(); command_Tpbip_timestamp= get_time_ms(); command_Tpbip_ms= ms; portEXIT_CRITICAL(); return ms; }

bool is_command_Tpins_expired() { 
  portENTER_CRITICAL();
  bool val = command_Tpins_ms < get_time_ms()-command_Tpins_timestamp; 
  portEXIT_CRITICAL(); 
  return val;
  }
bool is_command_Tpexp_expired() { 
  portENTER_CRITICAL();
  bool val = command_Tpexp_ms < get_time_ms()-command_Tpexp_timestamp;
  portEXIT_CRITICAL(); 
  return val;
}
bool is_command_Tpbip_expired() { 
  portENTER_CRITICAL();
  bool val = command_Tpbip_ms < get_time_ms()-command_Tpbip_timestamp; 
  portEXIT_CRITICAL(); 
  return val;
}

void set_command_soft_reset() { command_soft_reset= true; }


