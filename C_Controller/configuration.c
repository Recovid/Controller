#include "configuration.h"

#include "lowlevel/include/lowlevel.h"



const float P_PLATEAU_SLOPE = 0.1f; // V1 was 0.05
const float P_PLATEAU_MEAN  = 0.2f; // V1 was 0.1

const float MOTOR_STEP_TIME_US_MIN = 110.f;

const float CALIB_STEP_TIME_S = 1.f/*s*/ / MOTOR_STEPS_MAX; // V1 was 1/(MOTOR_STEPS_MAX*200/360) ?!
const float CALIB_MAGIC_RATIO = 1.f; // V1 was 0.8

// ------------------------------------------------------------------------------------------------
//! Environment simulation

float    LUNG_COMPLIANCE        =  500./25.0; //!< = 20 typical from healthy patient
float    LUNG_COMPLIANCE_MAX    =  500./12.5; //!< = 40 typical from "SRAS" patient

uint32_t LUNG_EXHALE_MS         =  500;
uint32_t LUNG_EXHALE_MS_MAX     =  500 * 120/100;

float    AIRWAYS_RESISTANCE     =    0.1;
float    AIRWAYS_RESISTANCE_MAX =    1.;

uint32_t BAVU_REST_MS           =  100;
float    BAVU_V_ML_MAX          =  600.;
float    BAVU_Q_LPM_MAX         =  200.;
float    BAVU_VALVE_RATIO       =    0.;

float    EXHAL_VALVE_RATIO      =    1.; // no leak, no obstruction
float    EXHAL_VALVE_P_RATIO    =    2.; // TODO /!\ motor_pep_steps/cmH2O taking into account motor_pep + valve surface ratio

float    PATMO_VARIATION_MBAR   =   50.;
