#include "configuration.h"

#include "lowlevel/include/lowlevel.h"

const float MOTOR_STEP_TIME_US_MIN = 100.f; // V1 was 110 with some risks to lose steps, 150 is more on the safe side

const float SAMPLES_T_US = 10000; //!< Between sensors interrupts

// Calibration
const float   CALIB_PDIFF_LPS_RATIO    = 105.0f; //! To convert raw readings to Lps
const float   CALIB_UNUSABLE_PDIFF_LPS =   0.1f; //!< Part of Pdiff readings that cannot be used to adjust flow
const uint8_t CALIB_PDIFF_SAMPLES_MIN  =  11   ; //!< For sliding average

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
