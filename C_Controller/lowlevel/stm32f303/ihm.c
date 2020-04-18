#include "lowlevel.h"
#include "hardware_serial.h"
#include <string.h>



bool init_ihm(ihm_mode_t ihm_mode, const char* pathInputFile, const char* pathOutputFile)
{
    (void) ihm_mode; (void) pathInputFile; (void) pathOutputFile;
    return hardware_serial_init(NULL);
}

bool send_ihm(const char* frame)
{

    return hardware_serial_write_data(frame, strlen(frame));
}

int recv_ihm()
{
    char blocking_read = 0;

    int t_s = hardware_serial_read_data(&blocking_read, sizeof(char));;

    if (t_s > 0) {

        return blocking_read;
    }
    return EOF;
}
