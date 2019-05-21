#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>

#include "serial/serial.h"
#include "packets.h"

#include "host.h"

#include "host_ros.h"

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
	
	
  ros::init(argc, argv, "acquisition_platform_node");
  ros::NodeHandle nh("~");
	
	Host_ros host_ros(nh);
	host_ros.setOdomFrameId("/odom");
	host_ros.setOdomTopic("/odom");
	host_ros.advertise();
	
	ros::Rate loop_rate(10); //10 Hz
	
	
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
	
	#ifdef IMU
	Host_getIMUConfiguration();
	Host_printIMUConfiguration();
	#endif
	
	#ifdef IMU
	printf("IMU\n");
	#else
	printf("no IMU\n");
	#endif
	#ifdef ENCS
	printf("ENCS\n");
	#else
	printf("no ENCS\n");
	#endif
	
	OdometryPacket odom_data;
	
	
	while(ros::ok()) {
		#ifdef ENCS
		Host_getOdometryData(&odom_data);
		//Host_printOdometryData();
		
		host_ros.publish(&odom_data);
		#endif
		
		#ifdef IMU
		Host_getAccelerometerData();
		Host_getGyroscopeData();
		Host_getMagnetometerData();
		
		Host_printIMUData();
		#endif
		
		loop_rate.sleep();
	}
	
	EXIT:
		res = Host_destroy();
		if( !res )
			printf("[] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
		else
			printf("[] Error closing communication with controller on serial port : %s\n", SERIAL_NAME);
		return 0;
}
