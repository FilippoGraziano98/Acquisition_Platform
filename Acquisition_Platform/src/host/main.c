#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>


#include "serial/serial.h"
#include "kalman_filter/kalman_filter.h"

#include "host.h"

#define SERIAL_NAME "/dev/ttyACM0"


void SIGINT_handler(int sig){
	//exits
	int res = Host_destroy();
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
	
	uint8_t calibration_flag = IMU_FAST_RECALIB;
	
	if( argc > 1 ) {
		if( strncmp(argv[1], "--help", sizeof("--help")) == 0 || strncmp(argv[1], "-h", sizeof("-h")) == 0) {
			printf("Usage: %s [PARAMS]\n",argv[0]);
			printf("\t --calibrate : to calibrate imu\n");
			printf("\t --fast-recalibrate : to slightly update old calibration data (not saving)[default]\n");
			printf("\t --no-calibrate : to load old calibration for imu\n");
			return 0;
		} else if( strncmp(argv[1], "--calibrate", sizeof("--calibrate")) == 0 )
			calibration_flag = IMU_CALIB_START;
		else if( strncmp(argv[1], "--fast-recalibrate", sizeof("--fast-recalibrate")) == 0 )
			calibration_flag = IMU_FAST_RECALIB;
		else if( strncmp(argv[1], "--no-calibrate", sizeof("--no-calibrate")) == 0 )
			calibration_flag = IMU_NO_CALIB;
		else {
			printf("Usage: %s --help/-h : to get help\n",argv[0]);
			return -1;
		}
	}
	
	#ifdef DEBUG_PRINTF
	printf("WARNING: DEBUG_PRINTF is defined, packetization may be broken! Use Cutecom\n");
	#endif
	
	res = Host_init(SERIAL_NAME);
	if( res ) {
		printf("[] Error opening communication with controller on serial port : %s\n", SERIAL_NAME);
		return -1;
	}
	printf("[] Opened communication with controller on serial port : %s\n", SERIAL_NAME);
	
	res = install_SIGINT_handler();
	if( res )
		perror("[] Error installing SIGINT handler");
	
	printf("[] Checking communication with controller...");
	res = Host_checkConnection(SYNCHRONIZATION_CYCLES);
	if( !res )
		printf("DONE\n");
	else {
		printf("FAILED\n");
		goto EXIT;
	}
	
	
	#ifdef USE_IMU
	printf("IMU\n");
	#else
	printf("no IMU\n");
	#endif
	#ifdef USE_ENCS
	printf("ENCS\n");
	#else
	printf("no ENCS\n");
	#endif
	
	#ifdef USE_IMU
	//calibration
	Host_handle_IMU_Calibration(calibration_flag);
	Host_printIMUConfiguration();
	#endif
	
	
	while(1) {
		#ifdef ODOM_ENCS
		//Host_getOdometryData();
		Host_printOdometryData();
		#endif
		
		#ifdef ODOM_IMU
		//Host_getAccelerometerData();
		//Host_getGyroscopeData();
		//Host_getMagnetometerData();
		
		//Host_printIMUData();
		Host_printIMUOdometryData();
		#endif
		
		Host_printSensorsData();
		KalmanFilter_OdometryPrint();
		
		sleep(1);
	}
	
	EXIT:
		res = Host_destroy();
		if( !res )
			printf("[] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
		else
			printf("[] Error closing communication with controller on serial port : %s\n", SERIAL_NAME);
		return 0;
}
