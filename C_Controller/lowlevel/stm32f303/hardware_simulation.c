#include "hardware_simulation.h"

#include <time.h>
#include <stdio.h>

// ------------------------------------------------------------------------------------------------
//! OS simulation

//! Simulated clock for testing purposes
static long clock_ms = 0;

long get_time_ms()
{
    return clock_ms;
}

long wait_ms(long t_ms)
{
    return clock_ms += t_ms; // simulated clock for testing purposes
}

bool soft_reset()
{
    return true;
}

// ------------------------------------------------------------------------------------------------
//! IHM simulation based on stdin/stdout

FILE *in ;
FILE *out;

bool init_ihm(const char* pathInputFile, const char* pathOutputFile)
{
    // TODO Replace with HAL_UART_init, no connection per se
    if (pathInputFile)
        in  = fopen(pathInputFile, "r");
    else
        in = stdin;
    if (pathOutputFile)
        out = fopen(pathOutputFile, "w");
    else
        out = stdout;
    return true;
}

bool send_ihm(const char* frame)
{
    if (!frame || *frame=='\0') return 0;
    return fputs(frame, out) >= 0;
}

int recv_ihm()
{
    static time_t last_blocked_s = 0;

    time_t t_s;
    time(&t_s);
    if (last_blocked_s+5 < t_s) {
        int blocking_read = fgetc(in);
        if (blocking_read == '\n') {
            last_blocked_s = t_s;
        }
        return blocking_read;
    }
    return EOF;
}
