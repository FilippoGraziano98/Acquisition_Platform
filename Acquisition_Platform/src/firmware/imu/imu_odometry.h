#pragma once

#include "../packets/uart_packets.h"
#include "imu.h"

#define M_PI 3.141592654

//IMU_OdometryUpdate will be called in IMU_AccelGyro_Callback
#define IMU_ODOMETRY_UPDATE_RATE IMU_UPDATE_RATE	//500 Hz

typedef struct IMU_OdometryController_t{
	//sequence increased at each timer interrupt occured
	uint16_t imu_time_seq;
	//sequence of the last registered movement TODO
	uint16_t last_move_seq;
	
	//time between two consecutive updates
	float delta_time;	//secs
	
	//odometry is stored in a packet ready to be sent
	IMUOdometryPacket odometry_status;
	
} IMU_OdometryController_t;


void IMU_OdometryInit(void);

void IMU_OdometryUpdate(AccelerometerPacket* accel_values, GyroscopePacket* gyro_values);


/*
 * IMU_get<Sensor>
 *	@params: ptr to variables where to store values
 *	@returns: seq value of the sensor
 */
//TODO uint16_t IMU_getOdometry(<...>);


uint8_t IMU_sendOdometryToHost(void);
