#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>

#include "serial/serial.h"
#include "packets.h"

#include "kalman_filter/kalman_filter.h"
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
	
  ros::init(argc, argv, "acquisition_platform_node");
  ros::NodeHandle nh("~");
	
	Host_ros host_ros(nh);
	host_ros.setOdomFrameId("/odom");
	host_ros.setOdomTopic("/odom");
	host_ros.setIMUOdomFrameId("/imu_odom");
	host_ros.setIMUOdomTopic("/imu_odom");
	host_ros.setKFOdomFrameId("/kf_odom");
	host_ros.setKFOdomTopic("/kf_odom");
	host_ros.advertise();
	
	ros::Rate loop_rate(10); //10 Hz
	int i = 0;
	
	
	OdometryPacket odom_data = {};
	IMUOdometryPacket imu_odom_data = {};
	
	KFOdometryPacket kf_odom_data = {};
	
	
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
	
	#ifdef USE_ENCS
	printf("ENCS\n");
	#else
	printf("no ENCS\n");
	#endif
	#ifdef USE_IMU
	printf("IMU\n");
	#else
	printf("no IMU\n");
	#endif
	
	#ifdef USE_IMU
	//calibration
	Host_handle_IMU_Calibration(calibration_flag);
	Host_printIMUConfiguration();
	#endif
	
	while(ros::ok()) {
		#ifdef ODOM_ENCS
		Host_getOdometryData(&odom_data);
				
		host_ros.odom_publish(&odom_data);
		#endif
		
		#ifdef ODOM_IMU
		Host_getIMUOdometryData(&imu_odom_data);
		
		host_ros.imu_odom_publish(&imu_odom_data);
		#endif
		
		Host_getKFOdometryData(&kf_odom_data);
		host_ros.kf_odom_publish(&kf_odom_data);
		
		if( i % 10 == 0) {
			#ifdef ODOM_ENCS
			Host_printOdometryData();
			//printf("%d)[enc] pos : x: %f, y: %f, z: %f\n", odom_data.header.seq, odom_data.odom_x, odom_data.odom_y, 0.);
			#endif
			
			#ifdef ODOM_IMU
			//Host_printIMUData();
			//printf("%d)[imu] pos : x: %f, y: %f, z: %f\n", imu_odom_data.header.seq, imu_odom_data.imu_odom_x, imu_odom_data.imu_odom_y, imu_odom_data.imu_odom_z);
//			printf("%d)[imu] vel : x: %f, y: %f, z: %f\n", imu_odom_data.header.seq, imu_odom_data.translational_velocity_x_axis, imu_odom_data.translational_velocity_y_axis, imu_odom_data.translational_velocity_z_axis);
//			printf("%d)[imu] accel X : pos: %d, neg: %d, zero: %d\n", imu_odom_data.header.seq, imu_odom_data.total_time_pos_accel_x, imu_odom_data.total_time_neg_accel_x, imu_odom_data.curr_time_zero_accel_x);
//			printf("%d)[imu] accel Y : pos: %d, neg: %d, zero: %d\n", imu_odom_data.header.seq, imu_odom_data.total_time_pos_accel_y, imu_odom_data.total_time_neg_accel_y, imu_odom_data.curr_time_zero_accel_y);
			Host_printIMUOdometryData();
			#endif
						
			KalmanFilter_OdometryPrint();

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
