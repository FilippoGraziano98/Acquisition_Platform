#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>

#include "serial/serial.h"

#include "host.h"

#define SERIAL_NAME "/dev/ttyACM0"

//private Host variable
static Host* host = NULL;

void SIGINT_handler(int sig){
	//reads all unread bytes
	uint8_t junk;
	while( serial_sleep_until_input(host->serial_fd, 0, 10000) > 0 )
		serial_receive(host->serial_fd, &junk, 1);
	
	//exits
	int res = Host_destroy(host);
	if( !res )
		printf("[SIGINT] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
	else
		printf("[SIGINT] Error closing communication with controller on serial port : %s\n", SERIAL_NAME);
	exit(0);
}


int install_SIGINT_handler(void){
	struct sigaction sa;
	
	sa.sa_handler = SIGINT_handler;
	sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
	
	return sigaction(SIGINT, &sa, NULL);
}

int main(int argc, char** argv) {
	int res;
	
	host = Host_init(SERIAL_NAME);
	if( !host ) {
		printf("[] Error opening communication with controller on serial port : %s\n", SERIAL_NAME);
		return -1;
	}
	printf("[] Opened communication with controller on serial port : %s\n", SERIAL_NAME);
	
	res = install_SIGINT_handler();
	if( res )
		perror("[] Error installing SIGINT handler");
	
	printf("[] Checking communication with controller...");
	res = Host_checkConnection(host, 10);
	if( !res )
		printf("DONE\n");
	else {
		printf("FAILED\n");
		goto EXIT;
	}
	
	Host_getIMUConfiguration(host);
	Host_printIMUConfiguration(host);
	
	while(1) {
		Host_getAccelerometerData(host);
		Host_getGyroscopeData(host);
		Host_getMagnetometerData(host);
		
		Host_printIMUData(host);
		
		sleep(1);
	}
	
	EXIT:
		res = Host_destroy(host);
		if( !res )
			printf("[] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
		else
			printf("[] Error closing communication with controller on serial port : %s\n", SERIAL_NAME);
		return 0;
}
