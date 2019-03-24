#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include "serial.h"

#define SERIAL_NAME "/dev/ttyACM0"
#define SERIAL_SPEED 38400
#define SERIAL_PARITY 0


#define MAX_BUF 256

int main(int argc, char** argv) {
	int res;
	int serial_fd = serial_open(SERIAL_NAME);
	if ( serial_fd < 0 ) {
		printf("[] Error in serial_open for SERIAL_NAME : %s\n", SERIAL_NAME);
		return -1;
	}
	
	res = serial_set_interface_attribs(serial_fd, SERIAL_SPEED, SERIAL_PARITY);
	if ( res < 0 ) {
		printf("[] Error in serial_set_interface_attribs for SERIAL_SPEED : %d and SERIAL_PARITY : %d\n", SERIAL_SPEED, SERIAL_PARITY);
		return -1;
	}
	
	
	uint8_t buf[MAX_BUF];
	memset(buf, 0, MAX_BUF);
	int n;
	
	
	sleep(1);
	
	n = read(serial_fd, buf, MAX_BUF);
	printf("%s [%d]\n", buf, n);
	
	uint8_t size = 0;
	while ( 1 ) {
		memset(buf, 0, MAX_BUF);
		n = read(STDIN_FILENO, buf, MAX_BUF);
		buf[n-1] = 0;
		printf((char*)"[host] sending %s [%d]\n", buf, n-1);
		write(serial_fd, buf, n);
		
		read(serial_fd, &size, 1);
		printf("[host] size %d [%d]\n", size, n-1);
		
		memset(buf, 0, MAX_BUF);
		n = read(serial_fd, buf, n);
		printf((char*)"[host] received %s [%d]\n", buf, n-1);
		int j;
		for(j=0; j<n; j++) {
			printf("%c-",buf[j]);
		}
		printf("\n");
		read(serial_fd, &size, 1);
		printf("[host] size %d [%d]\n", size, n-1);
	}
}
