#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "platform.h"

// ------------------------------------------------------------------------------------------------
//! Environment simulation
//! \remark only for test purposes

extern int LUNG_V_ML_MAX;
extern int LUNG_COMPLIANCE; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
extern int LUNG_EXHALE_MS;

extern int BAVU_V_ML_MAX;
extern int BAVU_Q_LPM_MAX;
// To simulate BAVU 'anti-retour' valve perforation
extern int BAVU_VALVE_RATIO;

extern int EXHAL_VALVE_RATIO;

extern int PATMO_VARIATION_MBAR; // TODO Estimate required range to maintain precise measures and reliable alarms

#endif // CONFIGURATION_H
