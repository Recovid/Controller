#include "configuration.h"

#include "lowlevel/include/lowlevel.h"

const float MOTOR_STEP_TIME_US_MIN = 100.f; // V1 was 110 with some risks to lose steps, 150 is more on the safe side

const float SAMPLES_T_US = 10000.f;

const float PDIFF_DT_US = 4763.71f;

// Calibration
const float   CALIB_PDIFF_LPS_RATIO    = 105.00f; //! To convert raw readings to Lps
const float   CALIB_UNUSABLE_PDIFF_LPS =   0.01f; //!< Part of Pdiff readings that cannot be used to adjust flow
const uint8_t CALIB_PDIFF_SAMPLES_MIN  =  11    ; //!< For sliding average


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

// ------------------------------------------------------------------------------------------------
//! HW model

const float VTINSU_RATIO_MAX = .8f;

uint16_t VTinsu_for_step[52][2] = {
    { 1316,	78	},
    { 1365,	98	},
    { 1414,	103	},
    { 1463,	108	},
    { 1512,	114	},
    { 1561,	120	},
    { 1610,	125	},
    { 1659,	131	},
    { 1708,	137	},
    { 1757,	143	},
    { 1806,	149	},
    { 1855,	156	},
    { 1904,	163	},
    { 1953,	169	},
    { 2002,	177	},
    { 2051,	184	},
    { 2100,	192	},
    { 2149,	200	},
    { 2198,	208	},
    { 2247,	216	},
    { 2296,	225	},
    { 2345,	235	},
    { 2394,	244	},
    { 2443,	253	},
    { 2492,	263	},
    { 2541,	278	},
    { 2590,	295	},
    { 2639,	315	},
    { 2688,	332	},
    { 2737,	347	},
    { 2786,	359	},
    { 2835,	372	},
    { 2884,	385	},
    { 2933,	398	},
    { 2982,	411	},
    { 3031,	424	},
    { 3080,	438	},
    { 3129,	452	},
    { 3178,	466	},
    { 3227,	481	},
    { 3276,	496	},
    { 3325,	512	},
    { 3374,	527	},
    { 3423,	542	},
    { 3472,	558	},
    { 3521,	574	},
    { 3570,	590	},
    { 3619,	606	},
    { 3668,	623	},
    { 3717,	640	},
    { 3766,	656	},
    { 3789,	670	}
};

uint16_t get_max_steps_for_Vol_mL(float Vol_mL)
{
    for (size_t s=0 ; s<COUNT_OF(VTinsu_for_step) ; s++) {
        if (VTinsu_for_step[s][1] >= VTINSU_RATIO_MAX*Vol_mL) {
            return s>0 ?
                VTinsu_for_step[s-1][0] :
                VTinsu_for_step[s  ][0] ; // TODO interpolate
        }
    }
    return MOTOR_MAX;
}

// ================================================================================================
#ifndef NTESTS
#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_max_steps_for_Vol_mL)
    return TEST_EQUALS(1316     , get_max_steps_for_Vol_mL(100.f))
        && TEST_EQUALS(2149     , get_max_steps_for_Vol_mL(250.f))
        && TEST_EQUALS(3178     , get_max_steps_for_Vol_mL(600.f))
        && TEST_EQUALS(3717     , get_max_steps_for_Vol_mL(810.f))
        && TEST_EQUALS(MOTOR_MAX, get_max_steps_for_Vol_mL(999.f));
}

bool PRINT(TEST_CONFIGURATION)
    return
        test_max_steps_for_Vol_mL() &&
        true;
}

#endif

