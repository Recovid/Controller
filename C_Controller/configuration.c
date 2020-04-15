#include "configuration.h"

#include "lowlevel/include/lowlevel.h"

// ------------------------------------------------------------------------------------------------
//! Environment simulation

float    LUNG_V_ML_MAX        = 3000.; // For simulation only
float    LUNG_COMPLIANCE      =  500./25.; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
uint32_t LUNG_EXHALE_MS       =  500;
uint32_t LUNG_EXHALE_MS_MAX   =  500 * 120/100;

float    BAVU_V_ML_MAX        =  600.;
float    BAVU_Q_LPM_MAX       =  200.; //!< due to BAVU perforation ?!
float    BAVU_VALVE_RATIO     =    0.;

float    EXHAL_VALVE_RATIO    =    1.; // no leak, no obstruction

float    PATMO_VARIATION_MBAR =   50.;
