#include "configuration.h"

#include "lowlevel/include/lowlevel.h"

// ------------------------------------------------------------------------------------------------
//! Environment simulation

float    LUNG_V_ML_MAX          = 3000.;

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

float    PATMO_VARIATION_MBAR   =   50.;
