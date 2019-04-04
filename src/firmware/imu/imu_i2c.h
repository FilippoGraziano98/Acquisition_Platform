#pragma once

//https://github.com/kriswiner/MPU9250/issues/3

/* DEVICE ADDRESS */
#define AD0 0x68
#define AD1 0x69 //if a second imu on the same i2c bus
#define ACCELGYRO_DEVICE AD0 // we are not using a second imu

#define MAGNET_DEVICE 0x0C

/* ACCELGYRO_DEVICE REGISTERS */
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0X1D

#define INT_PIN_CFG 0X37
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//accellerometer
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

//gyroscope 
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

//termometer
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

/* MAGNET_DEVICE REGISTERS */
#define CNTL1 0x0A

//magnetometer
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08
#define ST2 0x09

typedef struct {
	int16_t magnet_x;
	int16_t magnet_y;
	int16_t magnet_z;	
} MagnetometerSensor;

typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} AccelerometerSensor;

typedef struct {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} GyroscopeSensor;

typedef struct {
	int16_t temperature;
} TermometerSensor;

void IMU_Init(void);
AccelerometerSensor IMU_ReadAccelerometer(void);
GyroscopeSensor IMU_ReadGyroscope(void);
MagnetometerSensor IMU_ReadMagnetometer(void);
TermometerSensor IMU_ReadTermometer(void);
