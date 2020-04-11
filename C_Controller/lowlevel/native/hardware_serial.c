#include "hardware_serial.h"

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>


volatile int serial_handle;

char current_serial_port[50];

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// static void set_mincount(int fd, int mcount)
// {
//     struct termios tty;

//     if (tcgetattr(fd, &tty) < 0) {
//         printf("Error tcgetattr: %s\n", strerror(errno));
//         return;
//     }

//     tty.c_cc[VMIN] = mcount ? 1 : 0;
//     tty.c_cc[VTIME] = 5;        /* half second timer */

//     if (tcsetattr(fd, TCSANOW, &tty) < 0)
//         printf("Error tcsetattr: %s\n", strerror(errno));
// }


int hardware_serial_read_data(unsigned char * data, uint16_t data_size)
{
    int read_return = 0;

    if(data == NULL)
    {
    	printf("Invalid parameter !! \n");
    }
    else
    {
	    read_return = read(serial_handle, data, data_size);

	    if(read_return < 0)
	    {
	        printf("ERROR: Reading data. %s\n",strerror(errno));
	        // exit(-1);
	    }
    }

    return read_return;
}

int hardware_serial_write_data(const unsigned char * data, uint16_t data_size)
{
    int write_return = 0;

    if(data == NULL)
    {
    	printf("Invalid parameter !! \n");
    }
    else
    {
    	write_return = write(serial_handle, data, data_size);

	    if(write_return < 0)
	    {
	        printf("ERROR: Writing data. %s\n",strerror(errno));
	        // exit(-1);
	    }
	}
	return write_return;
}

int hardware_serial_init(const char * serial_port)
{    
    if(serial_port == NULL)
    {
    	printf("Wrong Parameter");
    }
    else
    {
    	strcpy(current_serial_port, serial_port);
    }

    serial_handle = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_handle < 0)
    {
        printf("Error opening serialport %s. %s\n",serial_port,strerror(errno));
        return 0;
    }

    set_interface_attribs(serial_handle, B115200);

    printf("Port : %s init ok\n", serial_port);

    return 1;
}
