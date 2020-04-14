#include "configuration.h"

#include "lowlevel/include/lowlevel.h"

// ------------------------------------------------------------------------------------------------
//! Environment simulation

int LUNG_V_ML_MAX   = 3000; // For simulation only
int LUNG_COMPLIANCE = 500/25; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
int LUNG_EXHALE_MS  = 500;

int BAVU_V_ML_MAX  = 500;
int BAVU_Q_LPM_MAX =  60;
int BAVU_VALVE_RATIO =  0.;

int EXHAL_VALVE_RATIO =  1.; // no leak, no obstruction

int PATMO_VARIATION_MBAR = 50;
