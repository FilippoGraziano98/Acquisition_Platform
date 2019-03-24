#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "serial.h"

int serial_open(const char* name) {
	// O_SYNC : Write operations on the file will complete according
		// to the requirements of synchronized I/O file integrity completion
	int fd = open(name, O_RDWR | O_NOCTTY | O_SYNC);
	if( fd < 0 ) {
		printf("[serial_open] Error %d opening serial device : %s\n", errno, name);
		perror("\t-> ");
		return -1;
	}
	return fd;
}

int serial_set_interface_attribs(int fd, int speed, int parity) {
	int res;
	
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	
	// store in tty the parameters associated the serial device referred by fd
	res = tcgetattr(fd, &tty);
	if( res < 0 ) {
		printf("[serial_set_interface_attribs] Error %d retrieving termios parameters of serial device with fd : %d\n", errno, fd);
		perror("\t-> ");
	}
	//converts speed into one of the speed_t Bnnn constants
	speed_t speed_bnnn = B0;
	switch (speed){
		case 19200:
			speed_bnnn=B19200;
			break;
		case 38400:
			speed_bnnn=B38400;
			break;
		case 57600:
			speed_bnnn=B57600;
			break;
		case 115200:
			speed_bnnn=B115200;
			break;
		default:
			printf("[serial_set_interface_attribs] Cannot set baudrate %d [values expected are : 19200, 38400, 57600, 115200]\n", speed);
			return -1;
	}	
	// sets the input baud rate stored in the termios structure to speed
	cfsetispeed(&tty, speed_bnnn);
	// sets the output baud rate stored in the termios structure to speed
	cfsetospeed(&tty, speed_bnnn);
	
	
	// sets the terminal to something like a "raw" mode
		// input is available character by character, echoing is disabled
		// all special processing of terminal input and output characters is disabled. 
		// serial switchs to noncanonical mode
	cfmakeraw(&tty);
	
	
	// PARENB : Enable parity generation on output and parity checking for input.
	// PARODD : If set, then parity for input and output is odd;
									//otherwise even parity is used.
	if( parity )
		tty.c_cflag |= (PARENB | PARODD);
	else
		tty.c_cflag &= ~PARENB;
	
	
	// sets characters' size to 8-bit
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	
	
	// set blocking mode
		// MIN > 0; TIME > 0:
			// TIME specifies the limit for a timer in tenths of a second.
				// Once an initial byte of input becomes available,
					//timer is restarted after each further byte is received.
			// read returns either
				// when min {number of bytes requested, MIN bytes have been read }
				// when the inter-byte timeout expires.
			// Because the timer is only started after the initial byte
				// becomes available, at least one byte will be read.
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
	
	// sets the parameters associated with the serial device
		// from the termios structure tty
		// TCSANOW : the change occurs immediately
	res = tcsetattr(fd, TCSANOW, &tty);
	if( res < 0 ) {
		printf("[serial_set_interface_attribs] Error %d setting termios parameters of serial device with fd : %d\n", errno, fd);
		perror("\t-> ");
		return -1;
	}
	
	return 0;
}
