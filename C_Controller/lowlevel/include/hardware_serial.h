#pragma once

#include "platform.h"

int hardware_serial_read_data(char * data, uint16_t data_size);
int hardware_serial_write_data(const char * data, uint16_t data_size);
int hardware_serial_init(const char * serial_port);
