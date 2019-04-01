#include <avr/io.h>
#include <util/delay.h>

#include "imu_i2c.h"
#include "imu.h"

// mg_data : Mxyz[0] = (double) mx * 1200 / 4096;
/*MagnetometerData IMU_MagnetometerData() {*/
/*	MagnetometerSensor mg_sens = IMU_ReadMagnetometer();*/
/*	//TODO*/
/*}*/


/*//accel_data : (double) ax / 16384;*/
/*AccellerometerData IMU_AccellerometerData() {*/

/*	//TODO*/
/*}*/


/*
 * Gyroscope Sensor [ https://learn.sparkfun.com/tutorials/gyroscope/how-to-select-a-gyro ]
 *	GYRO_XOUT = Gyro_Sensitivity * X_angular_rate
 *		where:
 *			- GYRO_XOUT : value read from the sensor (a change in the voltage)
 *			- X_angular_rate : variation of angular rotation
 *			- Gyro_Sensitivity : indicates how much the voltage changes for a given angular velocity (mV / DPS)
 *					we set our gyroscope to have Full Scale Range equal to (+/-)250 DPS
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *					==>> sensitivity = 2^15 / 250
 */
GyroscopeData IMU_GyroscopeData(void) {
	GyroscopeSensor gyro_sens = IMU_ReadGyroscope();
	
	float gyro_sensitivity = (uint16_t)(1<<15) / (float)250;
	
	GyroscopeData gyro_data;
	
	gyro_data.gyro_x = (float)(gyro_sens.gyro_x) / gyro_sensitivity;
	gyro_data.gyro_y = (float)(gyro_sens.gyro_y) / gyro_sensitivity;
	gyro_data.gyro_z = (float)(gyro_sens.gyro_z) / gyro_sensitivity;
	
	return gyro_data;
}

/*TermometerData IMU_TermometerData() {*/
/*	//TODO*/
/*}*/
