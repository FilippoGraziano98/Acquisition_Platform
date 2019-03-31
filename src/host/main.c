#include <stdio.h>

#include "host.h"

#define SERIAL_NAME "/dev/ttyACM0"


int main(int argc, char** argv) {
	int res;
	
	Host* host = Host_init(SERIAL_NAME);
	if( !host ) {
		printf("[] Error opening communication with controller on serial port : %s\n", SERIAL_NAME);
		return -1;
	}
	printf("[] Opened communication with controller on serial port : %s\n", SERIAL_NAME);
	printf("[] Checking communication with controller...");
	res = Host_checkConnection(host, 10);
	if( !res )
		printf("DONE\n");
	else {
		printf("FAILED\n");
		goto EXIT;
	}
	
	EXIT:
		res = Host_destroy(host);
		if( !res )
			printf("[] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
		else
			printf("[] Error closing communication with controller on serial port : %s\n", SERIAL_NAME);
}
