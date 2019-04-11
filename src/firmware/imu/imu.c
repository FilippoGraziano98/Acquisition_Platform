#include <avr/io.h>
#include <util/delay.h>

#include "imu_i2c.h"
#include "imu.h"

#define CALIBRATION_SAMPLES	64
#define CALIBRATION_SAMPLES_LOG	6

#ifdef DEBUG_PRINTF
#include <stdio.h>
#endif

/*
As with any sensor, the values you measure will contain some amount of error or bias.
You can see gyro bias by measuring the output when the gyro is still.
These errors are sometimes called bias drift or bias instability.

The temperature of the sensor greatly affects the bias.
To help minimize the source of this error, most gyros have a built in temperature sensor.
Thus, you are able to read the temperature of the sensor and correct or any temperature dependent changes.

In order to correct for these errors, the gyro must be calibrated.

This is usually done by keeping the gyro still and zeroing all of the readings in your code.
*/
GyroscopeCalibrationBiases IMU_GyroscopeCalibration(void) {
	uint32_t gyro_x_sum=0, gyro_y_sum=0, gyro_z_sum=0;
	
	#ifdef DEBUG_PRINTF
	printf("IMU_GyroscopeCalibration\n");
	#endif
	
	GyroscopeSensor gyro_sens_aux;
	int i;
	for(i=0; i<CALIBRATION_SAMPLES; i++) {
		gyro_sens_aux = IMU_ReadGyroscope();
		
		gyro_x_sum += gyro_sens_aux.gyro_x;
		gyro_y_sum += gyro_sens_aux.gyro_y;
		gyro_z_sum += gyro_sens_aux.gyro_z;
		
		#ifdef DEBUG_PRINTF
		printf("    %d) x: %d [sum: %ld], y: %d [sum: %ld], z: %d [sum: %ld]\n", i, gyro_sens_aux.gyro_x, gyro_x_sum, gyro_sens_aux.gyro_y, gyro_y_sum, gyro_sens_aux.gyro_z, gyro_z_sum);
		#endif
	}
	
	GyroscopeCalibrationBiases gyro_biases = {
		.gyro_x_bias = gyro_x_sum >> CALIBRATION_SAMPLES_LOG,
		.gyro_y_bias = gyro_y_sum >> CALIBRATION_SAMPLES_LOG,
		.gyro_z_bias = gyro_z_sum >> CALIBRATION_SAMPLES_LOG
	};
	
	#ifdef DEBUG_PRINTF
	printf("  x: %d, y: %d, z: %d\n", gyro_biases.gyro_x_bias, gyro_biases.gyro_y_bias, gyro_biases.gyro_z_bias);
	#endif
	return gyro_biases;
}


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
GyroscopeData IMU_GyroscopeData(GyroscopeCalibrationBiases* gyro_biases) {
	GyroscopeSensor gyro_sens = IMU_ReadGyroscope();
	
	GyroscopeSensor gyro_sens_calibrated = {
		.gyro_x = gyro_sens.gyro_x - gyro_biases->gyro_x_bias,
		.gyro_y = gyro_sens.gyro_y - gyro_biases->gyro_y_bias,
		.gyro_z = gyro_sens.gyro_z - gyro_biases->gyro_z_bias
	};
	
	#ifdef DEBUG_PRINTF
	printf("[IMU_GyroscopeData] x: %d, y: %d, z: %d\n", gyro_sens_calibrated.gyro_x, gyro_sens_calibrated.gyro_y, gyro_sens_calibrated.gyro_z);
	#endif
	
	float gyro_sensitivity = (uint16_t)(1<<15) / (float)250;
	
	GyroscopeData gyro_data;
	
	#ifdef DEBUG_RAW
	gyro_data.raw_x = gyro_sens.gyro_x;
	gyro_data.raw_y = gyro_sens.gyro_y;
	gyro_data.raw_z = gyro_sens.gyro_z;
	#endif
	
	gyro_data.gyro_x = (float)(gyro_sens_calibrated.gyro_x) / gyro_sensitivity;
	gyro_data.gyro_y = (float)(gyro_sens_calibrated.gyro_y) / gyro_sensitivity;
	gyro_data.gyro_z = (float)(gyro_sens_calibrated.gyro_z) / gyro_sensitivity;
	
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
