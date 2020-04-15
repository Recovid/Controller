#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Environment simulation
//! \remark only for test purposes

extern float    LUNG_V_ML_MAX;
extern float    LUNG_COMPLIANCE; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
extern uint32_t LUNG_EXHALE_MS;
extern uint32_t LUNG_EXHALE_MS_MAX; //!< time after which 99% of air exceeding lung's volume at rest should be exhaled

extern float    BAVU_V_ML_MAX;
extern float    BAVU_Q_LPM_MAX;
extern float    BAVU_VALVE_RATIO; //!< To simulate BAVU 'anti-retour' valve perforation

extern float    EXHAL_VALVE_RATIO;

extern float    PATMO_VARIATION_MBAR; // TODO Estimate required range to maintain precise measures and reliable alarms

#endif // CONFIGURATION_H
