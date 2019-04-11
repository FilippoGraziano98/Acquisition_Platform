#pragma once

typedef struct {
	int16_t gyro_x_bias;
	int16_t gyro_y_bias;
	int16_t gyro_z_bias;
} GyroscopeCalibrationBiases;

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
} AccelerometerData;

typedef struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;

	#ifdef DEBUG_RAW
	int16_t raw_x;
	int16_t raw_y;
	int16_t raw_z;
	#endif
} GyroscopeData;

typedef struct {
	float magnet_x;
	float magnet_y;
	float magnet_z;	
} MagnetometerData;

typedef struct {
	float temperature;
} TermometerData;


GyroscopeCalibrationBiases IMU_GyroscopeCalibration(void);

AccelerometerData IMU_AccelerometerData(void);
GyroscopeData IMU_GyroscopeData(GyroscopeCalibrationBiases* gyro_biases);
MagnetometerData IMU_MagnetometerData(void);
TermometerData IMU_TermometerData(void);
