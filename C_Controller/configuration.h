#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__




#define DEFAULT_setting_FR_pm          (18) //!< \see checked_FR_pm()         to set it within the range defined by     VT, Vmax, EoI
#define DEFAULT_setting_VT_mL          (300) //!< \see checked_VT_mL()         to set it within the range defined by FR,     Vmax, EoI
#define DEFAULT_setting_Vmax_Lpm       (60) //!< \see checked_Vmax_Lpm()      to set it within the range defined by FR, VT,       EoI
#define DEFAULT_setting_EoI_ratio_x10  (20) //!< \see checked_EoI_ratio_x10() to set it within the range defined by FR, VT, Vmax

#define DEFAULT_setting_PEP_cmH2O      (5)

#define DEFAULT_setting_Pmax_cmH2O    (60)
#define DEFAULT_setting_Pmin_cmH2O    (20)
#define DEFAULT_setting_VTmin_mL      (150)
#define DEFAULT_setting_VTmax_mL      (1000)
#define DEFAULT_setting_FRmin_pm      (10)
#define DEFAULT_setting_VMmin_Lpm     (3)

#define DEFAULT_setting_PEPmax_cmH2O  (2)
#define DEFAULT_setting_PEPmin_cmH2O  (-2)








// #include "platform.h"

// extern const float MOTOR_STEP_TIME_US_MIN; //!< Inverse of maximum motor speed in µs/step
// // TODO extern const float MOTOR_ACCEL_MAX; //!< Maximum motor acceleration in step/s^2

// extern const float SAMPLES_T_US; //!< Between sensors interrupts

// // Calibration
// extern const float   CALIB_PDIFF_LPS_RATIO   ; //!< To convert raw readings to Lps
// extern const float   CALIB_UNUSABLE_PDIFF_LPS; //!< Part of Pdiff readings that cannot be used to adjust flow
// extern const uint8_t CALIB_PDIFF_SAMPLES_MIN ; //!< For sliding average

// // ------------------------------------------------------------------------------------------------
// //! Environment simulation
// //! \remark only for test purposes

// extern float    LUNG_COMPLIANCE       ; //!< dV_mL/dP_cmH2O \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
// extern float    LUNG_COMPLIANCE_MAX   ;

// extern uint32_t LUNG_EXHALE_MS        ;
// extern uint32_t LUNG_EXHALE_MS_MAX    ; //!< time after which 99% of air exceeding lung's volume at rest should be exhaled

// extern float    AIRWAYS_RESISTANCE    ; //!< cmH2O/Lpm \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf
// extern float    AIRWAYS_RESISTANCE_MAX; //!< cmH2O/Lpm \see https://outcomerea.fr/docs/day2019/Forel_Mechanical_power.pdf

// extern uint32_t BAVU_REST_MS          ;
// extern float    BAVU_V_ML_MAX         ;
// extern float    BAVU_Q_LPM_MAX        ; //!< due to BAVU perforation ?!
// extern float    BAVU_VALVE_RATIO      ; //!< To simulate BAVU 'anti-retour' valve perforation

// extern float    EXHAL_VALVE_P_RATIO   ; // TODO /!\ motor_pep_steps/cmH2O taking into account motor_pep + valve surface ratio

// extern float    EXHAL_VALVE_RATIO     ;

// extern float    PATMO_VARIATION_MBAR  ; // TODO Estimate required range to maintain precise measures and reliable alarms

#endif // __CONFIGURATION_H__
