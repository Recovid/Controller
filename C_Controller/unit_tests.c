#include "unit_tests.h"
#include "leds.h"

#include "ihm_communication.h"

bool test_failure_not_detected()
{
    return (!(
        !TEST_RANGE(0      , -1   , INT_MAX) &&
        !TEST_RANGE(INT_MIN,  1   , 0      ) &&
        !TEST_RANGE(0.f    , -1.f , FLT_MAX) &&
        !TEST_RANGE(FLT_MIN,  1.f , 0.f    ) &&
        !TEST_EQUALS(         1l  , 2l     ) &&
        !TEST_EQUALS(         1l  , 2      ) &&
        !TEST_EQUALS(         1   , 2      ) &&
        !TEST_EQUALS(         1   , 2.     ) &&
        !TEST_EQUALS(         1.  , 2.     ) &&
        !TEST_EQUALS(         1.f , 2.f    ) &&
        !TEST_EQUALS(         true, false  ) &&
        !TEST(false)));
}

bool test_default_settings()
{
    return
        TEST_EQUALS( 18.f, get_setting_FR_pm       ()) &&
        TEST_EQUALS(300.f, get_setting_VT_mL       ()) &&
        TEST_EQUALS(  5.f, get_setting_PEP_cmH2O   ()) &&
        TEST_EQUALS( 60.f, get_setting_Vmax_Lpm    ()) &&
        TEST_EQUALS( 0.5f, get_setting_IoE_ratio   ()) &&
        TEST_EQUALS( 60.f, get_setting_Pmax_cmH2O  ()) &&
        TEST_EQUALS( 20.f, get_setting_Pmin_cmH2O  ()) &&
        TEST_EQUALS(150.f, get_setting_VTmin_mL    ()) &&
        TEST_EQUALS( 10.f, get_setting_FRmin_pm    ()) &&
        TEST_EQUALS(  3.f, get_setting_VMmin_Lm    ()) &&
        TEST_EQUALS(  2.f, get_setting_PEPmax_cmH2O()) &&
        TEST_EQUALS( -2.f, get_setting_PEPmin_cmH2O()) &&
        TEST_EQUALS(  0.f, get_command_Tpins_ms    ()) &&
        TEST_EQUALS(  0.f, get_command_Tpexp_ms    ()) &&
        TEST_EQUALS(  0.f, get_command_Tpbip_ms    ()) &&
        TEST_EQUALS(  0.f, is_soft_reset_asked     ()) &&
        true;
}

bool blink() {
    leds_init();
    while(1) {
        led_onnucleo_toggle();
        // FIXME: does vTaskDelay work correctly on stm32 ?
        #ifdef stm32f303
            HAL_Delay(500);
        #else
            vTaskDelay(500);
        #endif
    }
    return true;
}


bool unit_tests_passed()
{
    STDERR_PRINT("Start unit tests");
    bool passed =
        !test_failure_not_detected() &&
        test_default_settings();
    STDERR_PRINTF("Unit tests:%s", passed ? "passed" : "failed");
    return passed;
}
