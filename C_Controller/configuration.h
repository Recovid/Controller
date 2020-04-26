#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "platform.h"

extern const float MOTOR_STEP_TIME_US_MIN; //!< Inverse of maximum motor speed in µs/step
// TODO extern const float MOTOR_ACCEL_MAX; //!< Maximum motor acceleration in step/s^2

extern const float SAMPLES_T_US; //!< Between sensors interrupts

extern const float FLOW_DT_US; //!< As measured with µs timer

//! steps_t_us are fully corrected to reach flow setting before reaching this threshold,
//! after which correction ratio is bounded between FLOW_CORRECTION_THRESHOLD..1/FLOW_CORRECTION_THRESHOLD
//! in order to avoid over correcting transient flow errors like those resulting from a twisted tube
extern const float FLOW_CORRECTION_MIN;

// Calibration
extern const float   CALIB_PDIFF_LPS_RATIO   ; //!< To convert raw readings to Lps
extern const float   CALIB_UNUSABLE_PDIFF_LPS; //!< Part of Pdiff readings that cannot be used to adjust flow
extern const uint8_t CALIB_PDIFF_SAMPLES_MIN ; //!< For sliding average

#define ACCEL_STEP_T_US (400)
#define ACCEL_STEPS     ( 50)

#define DECEL_STEP_T_US (800)
#define DECEL_STEPS     (100)

// ------------------------------------------------------------------------------------------------
//! Environment simulation
//! \remark only for test purposes

extern float    LUNG_COMPLIANCE       ; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
extern float    LUNG_COMPLIANCE_MAX   ;

extern uint32_t LUNG_EXHALE_MS        ;
extern uint32_t LUNG_EXHALE_MS_MAX    ; //!< time after which 99% of air exceeding lung's volume at rest should be exhaled

extern float    AIRWAYS_RESISTANCE    ; //!< cmH2O/Lpm \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
extern float    AIRWAYS_RESISTANCE_MAX; //!< cmH2O/Lpm \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf

extern uint32_t BAVU_REST_MS          ;
extern float    BAVU_V_ML_MAX         ;
extern float    BAVU_Q_LPM_MAX        ; //!< due to BAVU perforation ?!
extern float    BAVU_VALVE_RATIO      ; //!< To simulate BAVU 'anti-retour' valve perforation

extern float    EXHAL_VALVE_P_RATIO   ; // TODO /!\ motor_pep_steps/cmH2O taking into account motor_pep + valve surface ratio

extern float    EXHAL_VALVE_RATIO     ;

extern float    PATMO_VARIATION_MBAR  ; // TODO Estimate required range to maintain precise measures and reliable alarms

// ------------------------------------------------------------------------------------------------
//! HW model

const float VTINSU_RATIO_MAX;

uint16_t get_max_steps_for_Vol_mL(float Vol_mL);

#endif // CONFIGURATION_H
