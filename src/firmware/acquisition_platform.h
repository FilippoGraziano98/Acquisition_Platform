#pragma once

#include <stdint.h>

#include "imu/imu.h"
#include "imu/imu_i2c.h"

typedef struct AcquisitionPlatform {	
	//seq number of the latest packets sent
		//seq is set when updating data from imu
		//and then copied in the packets' headers
  uint16_t global_seq;
  
  //IMU_calibration_parameters
  GyroscopeCalibrationBiases gyroscope_biases;
  
	//these are the system variables, updated reading data from imu
	AccelerometerData accelerometer_data;
  GyroscopeData gyroscope_data;
  MagnetometerData magnetometer_data;
} AcquisitionPlatform;

/*
 * AcquisitionPlatform_init :
 * 	initializes AcquisitionPlatform
 */
void AcquisitionPlatform_init(void);

/*
 * AcquisitionPlatform_init :
 * 	calibrates sensors on the imu
 */
void AcquisitionPlatform_imuCalibrate(void);

/*
 * AcquisitionPlatform_imuUpdate :
 * 	updates data stored in acq_pl reading new ones from imu
 */
//void AcquisitionPlatform_imuUpdate(void);

/*
 * AcquisitionPlatform_setPeriodicIMUupdate :
 * 	enbles a timer interrupt driven update
 * 		of data stored in acq_pl ( reading new ones from imu )
 */
void AcquisitionPlatform_setPeriodicIMUupdate(uint16_t frequency);

/*
 * AcquisitionPlatform_getGlobalSeq :
 * 	returns global seq / epoque
 */
uint16_t AcquisitionPlatform_getGlobalSeq(void);

/*
 * AcquisitionPlatform_getGyroscopeBiases :
 * 	returns gyroscope calibration biases
 */
GyroscopeCalibrationBiases AcquisitionPlatform_getGyroscopeBiases(void);

/*
 * AcquisitionPlatform_getAccelerometer :
 * 	returns accellerometer data
 */
AccelerometerData AcquisitionPlatform_getAccelerometer(void);

/*
 * AcquisitionPlatform_getGyroscope :
 * 	returns gyroscope data
 */
GyroscopeData AcquisitionPlatform_getGyroscope(void);

/*
 * AcquisitionPlatform_getMagnetometer :
 * 	returns magnetometer data
 */
MagnetometerData AcquisitionPlatform_getMagnetometer(void);
