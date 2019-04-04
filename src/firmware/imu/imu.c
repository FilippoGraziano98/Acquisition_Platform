#include <avr/io.h>
#include <util/delay.h>

#include "imu_i2c.h"
#include "imu.h"



/*
 * Accelerometer Sensor [ https://learn.sparkfun.com/tutorials/accelerometer-basics ]
 * 	ACCEL_XOUT = Accel_Sensitivity * X_acceleration
 *		where:
 *			-250dps ACCEL_XOUT : value read from the sensor (a change in the voltage)
 *			- X_acceleration : variation of acceleration / force
 *			- Accel_Sensitivity : indicates how much the voltage changes for a given acceleration
 *					we set our gyroscope to have Full Scale Range equal to (+/-)2 G
 *							[ note: G-forces -> a single G-force for us here on planet Earth is equivalent to 9.8 m/s^2 ]
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *					==>> sensitivity = 2^15 / 2
 */
AccelerometerData IMU_AccelerometerData() {
	AccelerometerSensor accel_sens = IMU_ReadAccelerometer();
	
	float accel_sensitivity = (uint16_t)(1<<15) / (float)2;
	
	AccelerometerData accel_data;
	
	accel_data.accel_x = (float)(accel_sens.accel_x) / accel_sensitivity;
	accel_data.accel_y = (float)(accel_sens.accel_y) / accel_sensitivity;
	accel_data.accel_z = (float)(accel_sens.accel_z) / accel_sensitivity;
	
	return accel_data;
}


/*
 * Gyroscope Sensor [ https://learn.sparkfun.com/tutorials/gyroscope ]
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


/*
 * Magnetometer Sensor [ from datasheet ]
 * 	X_magnetic_field = Magnet_Resolution * register_XOUT
 *		where:
 *			- register_XOUT : value read from the sensor (a change in the voltage)
 *			- X_magnetic_field : magnetic field measured on the x-axis
 *			- Magnet_Resolution : indicates how much the voltage changes for a given magnetic field
 *					Full scale measurement range is +/- 4912 μT [datasheet says 4800, register map says 4912]
 *						and we read a 16-bits value in 2's complement (+/-)2^15 mV
 *							( more precisely range is -32760 ~ 32760, and not +/- 32768 )
 *					[ NOTE: Magnet_Resolution = 1/Magnet_Sensitivity ]
 *					==>> Magnet_Resolution =~ 4912 / 32760 =~ 0.14993894993894993
 */
MagnetometerData IMU_MagnetometerData() {
	MagnetometerSensor mg_sens = IMU_ReadMagnetometer();
	
	float mg_resolution = (float)4912 / 32760;
	
	MagnetometerData mg_data;
	
	mg_data.magnet_x = (float)(mg_sens.magnet_x) * mg_resolution;
	mg_data.magnet_y = (float)(mg_sens.magnet_y) * mg_resolution;
	mg_data.magnet_z = (float)(mg_sens.magnet_z) * mg_resolution;
	
	return mg_data;
}

/*TermometerData IMU_TermometerData() {*/
/*	//TODO*/
/*}*/
