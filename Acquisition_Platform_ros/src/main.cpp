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
	host_ros.setIMUOdomFrameId("/imu_odom");
	host_ros.setIMUOdomTopic("/imu_odom");
	host_ros.advertise();
	
	ros::Rate loop_rate(10); //10 Hz
	int i = 0;
	
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
	IMUOdometryPacket imu_odom_data;
	
	while(ros::ok()) {
		#ifdef ENCS
		Host_getOdometryData(&odom_data);
				
		host_ros.odom_publish(&odom_data);
		#endif
		
		#ifdef IMU
		Host_getIMUOdometryData(&imu_odom_data);
		
		host_ros.imu_odom_publish(&imu_odom_data);
		#endif
		
		if( i % 10 == 0) {
		
			//Host_printOdometryData();
			printf("%d)[enc] pos : x: %f, y: %f, z: %f\n", odom_data.header.seq, odom_data.odom_x, odom_data.odom_y, 0.);
		
			//Host_printIMUData();
			//printf("%d) yaw(z): %f, pitch(y): %f, roll(x): %f\n", imu_odom_data.header.seq, imu_odom_data.imu_yaw, imu_odom_data.imu_pitch, imu_odom_data.imu_roll);
		
			//printf("%d) accel : x: %f, y: %f, z: %f\n", imu_odom_data.header.seq, imu_odom_data.translational_acceleration_x_axis, imu_odom_data.translational_acceleration_y_axis, imu_odom_data.translational_acceleration_z_axis);
			printf("%d)[imu] pos : x: %f, y: %f, z: %f\n", imu_odom_data.header.seq, imu_odom_data.imu_odom_x, imu_odom_data.imu_odom_y, imu_odom_data.imu_odom_z);
			printf("%d)[imu] vel : x: %f, y: %f, z: %f\n", imu_odom_data.header.seq, imu_odom_data.translational_velocity_x_axis, imu_odom_data.translational_velocity_y_axis, imu_odom_data.translational_velocity_z_axis);
			printf("%d)[imu] accel X : pos: %d, neg: %d, zero: %d\n", imu_odom_data.header.seq, imu_odom_data.total_time_pos_accel_x, imu_odom_data.total_time_neg_accel_x, imu_odom_data.curr_time_zero_accel_x);
			printf("%d)[imu] accel Y : pos: %d, neg: %d, zero: %d\n", imu_odom_data.header.seq, imu_odom_data.total_time_pos_accel_y, imu_odom_data.total_time_neg_accel_y, imu_odom_data.curr_time_zero_accel_y);
			printf("\n");

			i -= 10;
		}
		
		i++;
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
