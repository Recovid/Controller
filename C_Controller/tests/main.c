#include "platform.h"

//STD include
#include <stdio.h>
#include <string.h>

//High-level include
#include "sensing.h"
#include "alarms.h"
#include "controller.h"
#include "ihm_communication.h"

//Low-level include
#include "lowlevel/include/lowlevel.h"
#include "lowlevel/include/simple_indicators.h"
#ifndef WIN32
#include "tasks_recovid.h"
#include "TaskSensing.h"

//FreeRTOS Include
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#endif

#define PRINT(_name) _name() { fprintf(stderr,"- " #_name "\n");

bool PRINT(test_assume_failure_not_detected)
    TEST_ASSUME(false);
    return true;
}

bool PRINT(test_failure_not_detected)
    return (!(
        !TEST_RANGE(0      , -1   , INT_MAX) &&
        !TEST_RANGE(INT_MIN,  1   , 0      ) &&
        !TEST_RANGE(0.f    , -1.f , FLT_MAX) &&
        !TEST_RANGE(FLT_MIN,  1.f , 0.f    ) &&
        !TEST_FLT_EQUALS(     1.f , 1.1f   ) &&
        !TEST_EQUALS(         1l  , 2l     ) &&
        !TEST_EQUALS(         1l  , 2      ) &&
        !TEST_EQUALS(         1   , 2      ) &&
        !TEST_EQUALS(         1   , 2.     ) &&
        !TEST_EQUALS(         1.  , 2.     ) &&
        !TEST_EQUALS(         1.f , 2.f    ) &&
        !TEST_EQUALS(         true, false  ) &&
        !TEST(false)));
}

bool PRINT(test_default_settings)
        return
    TEST_FLT_EQUALS( 18.00f, get_setting_FR_pm       ()) &&
    TEST_EQUALS(   3333    , get_setting_T_ms        ()) &&

    TEST_FLT_EQUALS(300.00f, get_setting_VT_mL       ()) &&
    TEST_FLT_EQUALS( 60.00f, get_setting_Vmax_Lpm    ()) &&
    TEST_EQUALS(    300    , get_setting_Tinsu_ms    ()) &&

    TEST_FLT_EQUALS(  0.50f, get_setting_IoE_ratio   ()) &&
    TEST_FLT_EQUALS(  2.00f, get_setting_EoI_ratio   ()) &&
    TEST_EQUALS(   2222    , get_setting_Texp_ms     ()) &&

    TEST_EQUALS(   1111    , get_setting_Tinspi_ms   ()) &&
    TEST_EQUALS(    811    , get_setting_Tplat_ms    ()) &&

    TEST_FLT_EQUALS(  5.00f, get_setting_PEP_cmH2O   ()) &&

    TEST_FLT_EQUALS( 60.00f, get_setting_Pmax_cmH2O  ()) &&
    TEST_FLT_EQUALS( 20.00f, get_setting_Pmin_cmH2O  ()) &&
    TEST_FLT_EQUALS(150.00f, get_setting_VTmin_mL    ()) &&
    TEST_FLT_EQUALS( 10.00f, get_setting_FRmin_pm    ()) &&
    TEST_FLT_EQUALS(  3.00f, get_setting_VMmin_Lm    ()) &&
    TEST_FLT_EQUALS(  2.00f, get_setting_PEPmax_cmH2O()) &&
    TEST_FLT_EQUALS( -2.00f, get_setting_PEPmin_cmH2O()) &&
    TEST_FLT_EQUALS(  0.00f, get_command_Tpins_ms    ()) &&
    TEST_FLT_EQUALS(  0.00f, get_command_Tpexp_ms    ()) &&
    TEST_FLT_EQUALS(  0.00f, get_command_Tpbip_ms    ()) &&
    TEST_FLT_EQUALS(  0.00f, is_soft_reset_asked     ()) &&

    true;
}

int main(int argc, const char** argv)
{
    STDERR_PRINT("Start unit tests");
    int failed = 0;
    failed += ! TEST_IHM(); //!< \warning fails at clock_ms > 0
    failed += ! !test_assume_failure_not_detected();
    failed += ! !test_failure_not_detected();
    failed += ! test_default_settings();
    failed += ! TEST_SENSING();
    #ifdef native
    failed += ! TEST_LOWLEVEL_SIMULATION();
    #endif
    failed += ! TEST_CONTROLLER();
    failed += ! TEST_ALARMS();

    if (failed)
        STDERR_PRINTF("Unit tests: /!\\ FAILED %d", failed);
    else
        STDERR_PRINT ("Unit tests: (i) PASSED");

    return failed;
}
