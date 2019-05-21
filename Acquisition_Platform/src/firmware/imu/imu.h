#pragma once

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
typedef struct IMU_t {
	//sequence increased at each timer interrupt occured
	uint16_t imu_time_seq;
	
	/* ACCELEROMETER */
	int16_t accel_raw_x;
	int16_t accel_raw_y;
	int16_t accel_raw_z;
	uint8_t accel_raw_flag;	//flag indicating if data is valid or not
	
	float accel_x;
	float accel_y;
	float accel_z;
	uint16_t accel_seq;	//seq increased when values updated
	
	/* GYROSCOPE */
	int16_t gyro_x_bias;	//calibration
	int16_t gyro_y_bias;
	int16_t gyro_z_bias;
	
	int16_t gyro_raw_x;		//16-bit values from regs
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;
	uint8_t gyro_raw_flag;
	
	float gyro_x;					//physical values
	float gyro_y;
	float gyro_z;
	uint16_t gyro_seq;
	
	/* MAGNETOMETER */
	int16_t magnet_raw_x;
	int16_t magnet_raw_y;
	int16_t magnet_raw_z;
	uint8_t magnet_raw_flag;

	float magnet_x;
	float magnet_y;
	float magnet_z;
	uint16_t magnet_seq;
	
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
 * in IMURaw_Init, we set Gyroscope and Accelerometer Bandwidths to 5Hz
 *	so we set output_data_rate to 10Hz
 */
#define IMU_UPDATE_RATE 10 //Hz
void IMU_Init(void);

void IMU_GyroscopeCalibration(void);

void IMU_getCalibrationData(int16_t* gyro_x_bias, int16_t* gyro_y_bias, int16_t* gyro_z_bias);

/*
 * IMU_get<Sensor>
 *	@params: ptr to x, y, z variables where to store values
 *	@returns: seq value of the sensor
 */
uint16_t IMU_getAccelerometer(float* x, float* y, float* z);
uint16_t IMU_getGyroscope(float* x, float* y, float* z);
uint16_t IMU_getMagnetometer(float* x, float* y, float* z);
//uint16_t IMU_getTermometer(int16_t* temp);
