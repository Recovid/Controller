#include "hardware_simulation.h"

#include <time.h>

long getTimeMs()
{
    static long clock = 0;
    return clock += 40; // simulated clock for testing purposes
}
