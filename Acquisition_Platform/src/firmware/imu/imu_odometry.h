#pragma once

#include "firmware_constants.h"
#include "../packets/uart_packets.h"
#include "imu.h"

//TODO reset imu at least 100Hz
//so at 100Hz in (i2c callback) I integrate delta_angles, and delta x,y,z
//at lower rate, update matrices, and data in global coordinates


#define M_PI 3.141592654
#define M_180 180

//IMU_OdometryUpdate will be called in IMU_AccelGyro_Callback
#define IMU_ODOMETRY_UPDATE_RATE (IMU_UPDATE_RATE)	//100 Hz

#define IMU_ANG_VEL_THRESHOLD 0.4 //DPS
#define IMU_TRASL_ACC_THRESHOLD 0.05 //G-Forces
#define G_FORCE_ACCEL_G 1 //g[G] = 1
#define G_FORCE_ACCEL_M_S2 9.8 //accel[m/s^2] = accel[G-Force]*9.8

#define STOP_BREAKING_TRESHOLD 2
#define STOP_ZERO_ACCEL_OUTLIER_FILTER 5

typedef struct IMU_OdometryController_t{
	//sequence increased at each timer interrupt occured
	uint16_t imu_time_seq;
	//sequence of the last registered movement TODO
	uint16_t last_move_seq;
	
	//time between two consecutive updates
	float delta_time;	//secs
	
	//variabili di appoggio
	AccelerometerPacket accel_values;
	GyroscopePacket gyro_values;
	
	//odometry is stored in a packet ready to be sent
	IMUOdometryPacket odometry_status;
	
	//to better control braking
	/*TODO
	uint16_t total_time_pos_accel;
	uint16_t total_time_neg_accel;
	uint16_t curr_time_zero_accel;
	*/
	
} IMU_OdometryController_t;


void IMU_OdometryInit(void);

void IMU_OdometryUpdate(void);


/*
 * IMU_get<Sensor>
 *	@params: ptr to variables where to store values
 *	@returns: seq value of the sensor
 */
//TODO uint16_t IMU_getOdometry(<...>);


uint8_t IMU_sendOdometryToHost(void);
