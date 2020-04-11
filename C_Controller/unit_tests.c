#include "unit_tests.h"

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

bool test_Y()
{
    return true; // TODO
}

bool unit_tests_passed()
{
    STDERR_PRINT("Start unit tests");
    bool passed =
        !test_failure_not_detected() &&
        test_Y();
    STDERR_PRINTF("Unit tests:%s", passed ? "passed" : "failed");
}
