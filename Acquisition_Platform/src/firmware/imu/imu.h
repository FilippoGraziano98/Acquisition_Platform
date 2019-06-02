#pragma once


#include "../packets/uart_packets.h"


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

#define VALID 1
#define INVALID 0

#define IMU_CALIBRATION_ADDRESS_DETATCHED 0x20
#define IMU_CALIBRATION_ADDRESS_ON_PLATFORM 0x60
#define IMU_CALIBRATION_ADDRESS_3 0x100
#define IMU_CALIBRATION_ADDRESS_DETATCHED_2 0x140
#define IMU_CALIBRATION_ADDRESS IMU_CALIBRATION_ADDRESS_3

typedef struct IMU_t {
	//sequence increased at each timer interrupt occured
	uint16_t imu_time_seq;
	
	
	IMUConfigurationPacket imu_config_values;
	
	/* ACCELEROMETER */
	int16_t accel_raw_x;
	int16_t accel_raw_y;
	int16_t accel_raw_z;
	uint8_t accel_raw_flag;	//flag indicating if data is valid or not
	
//	float accel_x;
//	float accel_y;
//	float accel_z;
	uint16_t accel_seq;	//seq increased when values updated
	AccelerometerPacket accel_values;
	
	/* GYROSCOPE */
//	int16_t gyro_x_bias;	//calibration
//	int16_t gyro_y_bias;
//	int16_t gyro_z_bias;
	
	int16_t gyro_raw_x;		//16-bit values from regs
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;
	uint8_t gyro_raw_flag;
	
//	float gyro_x;					//physical values
//	float gyro_y;
//	float gyro_z;
	uint16_t gyro_seq;
	GyroscopePacket gyro_values;
	
	/* MAGNETOMETER */
	int16_t magnet_raw_x;
	int16_t magnet_raw_y;
	int16_t magnet_raw_z;
	uint8_t magnet_raw_flag;

//	float magnet_x;
//	float magnet_y;
//	float magnet_z;
	uint16_t magnet_seq;
	MagnetometerPacket magnet_values;
	
	/* TERMOMETER */
	int16_t temperature_raw;
	uint8_t temp_raw_flag;
} IMU_t;


/*
 * [ https://www.analog.com/en/products/landing-pages/001/accelerometer-specifications-definitions.html ]
 * output_data_rate : rate at which data is sampled
 * bandwidth : highest frequency signal that can be sampled without aliasing by the specified output_data_rate,
 * Per the Nyquist sampling criterion, bandwidth is half the output_data_rate
 *
 * in IMURaw_Init, we set
 *		Gyroscope Bandwidth to 41 Hz
 *		Accelerometer Bandwidth to 44.8 Hz
 *	so we set output_data_rate to 100 Hz >= max(2*41,2*44.8)
 */
#define IMU_UPDATE_RATE 100 //Hz
void IMU_Init(void);

/*
 * IMU_Calibration
 *	if @param full_calibration == 1, multi-axis calibration
 *	else fast recalibration
 */
void IMU_Calibration(uint8_t full_calibration);

void IMU_getCalibrationData(IMUConfigurationPacket* config_pkt);

/*
 * IMU_get<Sensor>
 *	@params: ptr to x, y, z variables where to store values
 *	@returns: <sensor>_raw_flag indicating if data updated or not
 */
uint8_t IMU_getAccelerometer(AccelerometerPacket* accel_pkt);
uint8_t IMU_getGyroscope(GyroscopePacket* gyro_pkt);
uint8_t IMU_getMagnetometer(MagnetometerPacket* magnet_pkt);
//uint16_t IMU_getTermometer(int16_t* temp);

/*
 * IMU_sendIMUDataToHost
 *	sends Accelerometer, Gyroscope calibration data packet
 *	@returns: >0 if ok, 0 if error
 */
uint8_t IMU_sendCalibrationDataToHost(void);
/*
 * IMU_sendIMUDataToHost
 *	sends Accelerometer, Gyroscope, Magnetometer packets
 *	@returns: >0 if ok, 0 if error
 */
uint8_t IMU_sendIMUDataToHost(void);
