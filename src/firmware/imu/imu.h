#pragma once

typedef struct {
	float magnet_x;
	float magnet_y;
	float magnet_z;	
} MagnetometerData;

typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
} AccellerometerData;

typedef struct {
	float gyro_x;
	float gyro_y;
	float gyro_z;
} GyroscopeData;

typedef struct {
	float temperature;
} TermometerData;

MagnetometerData IMU_MagnetometerData(void);
AccellerometerData IMU_AccellerometerData(void);
GyroscopeData IMU_GyroscopeData(void);
TermometerData IMU_TermometerData(void);
