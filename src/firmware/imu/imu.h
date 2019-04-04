#pragma once

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
} AccelerometerData;

typedef struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;
} GyroscopeData;

typedef struct {
	float magnet_x;
	float magnet_y;
	float magnet_z;	
} MagnetometerData;

typedef struct {
	float temperature;
} TermometerData;

AccelerometerData IMU_AccelerometerData(void);
GyroscopeData IMU_GyroscopeData(void);
MagnetometerData IMU_MagnetometerData(void);
TermometerData IMU_TermometerData(void);
