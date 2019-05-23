#include <string.h>

#include <avr/interrupt.h>
#include <util/atomic.h>

#include "imu_odometry.h"

static IMU_OdometryController_t IMU_OdometryController;

void IMU_OdometryInit(void) {
	memset(&IMU_OdometryController, 0, sizeof(IMU_OdometryController_t));
	
	IMU_OdometryController.delta_time = 1. / (float)IMU_ODOMETRY_UPDATE_RATE;

	INIT_PACKET(IMU_OdometryController.odometry_status, IMU_ODOMETRY_PACKET_ID);
}

void IMU_OdometryUpdate(AccelerometerPacket* accel_values, GyroscopePacket* gyro_values) {
	IMU_OdometryController.odometry_status.rotational_velocity_z_axis = gyro_values->gyro_z;
	IMU_OdometryController.odometry_status.imu_yaw += IMU_OdometryController.odometry_status.rotational_velocity_z_axis*IMU_OdometryController.delta_time;
	
	IMU_OdometryController.odometry_status.rotational_velocity_y_axis = gyro_values->gyro_y;
	IMU_OdometryController.odometry_status.imu_pitch += IMU_OdometryController.odometry_status.rotational_velocity_y_axis*IMU_OdometryController.delta_time;
	
	IMU_OdometryController.odometry_status.rotational_velocity_x_axis = gyro_values->gyro_x;
	IMU_OdometryController.odometry_status.imu_roll += IMU_OdometryController.odometry_status.rotational_velocity_x_axis*IMU_OdometryController.delta_time;
	
	IMU_OdometryController.imu_time_seq++;
}


uint8_t IMU_sendOdometryToHost(void) {
	uint8_t res, ret = 0;
	
	IMU_OdometryController.odometry_status.header.seq = IMU_OdometryController.imu_time_seq;

	res = UART_send_packet((PacketHeader*)&(IMU_OdometryController.odometry_status));
	if(res > 0)
		ret++;
	
	return ret;
}
