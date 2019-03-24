#pragma once

typedef struct {
	double magnet_x;
	double magnet_y;
	double magnet_z;	
} MagnetometerData;

typedef struct {
	double accel_x;
	double accel_y;
	double accel_z;
} AccellerometerData;

typedef struct {
	double gyro_x;
	double gyro_y;
	double gyro_z;
} GyroscopeData;

typedef struct {
	double temperature;
} TermometerData;

MagnetometerData IMU_MagnetometerData(void);
AccellerometerData IMU_AccellerometerData(void);
GyroscopeData IMU_GyroscopeData(void);
TermometerData IMU_TermometerData(void);
