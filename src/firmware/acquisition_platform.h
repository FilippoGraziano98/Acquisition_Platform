#pragma once

#include <stdint.h>

#include "imu/imu.h"
#include "imu/imu_i2c.h"

typedef struct AcquisitionPlatform {	
	//seq number of the latest packets sent
		//seq is set when updating data from imu
		//and then copied in the packets' headers
  uint16_t global_seq;
  
	//these are the system variables, updated reading data from imu
	AccelerometerData accelerometer_data;
  GyroscopeData gyroscope_data;
  MagnetometerData magnetometer_data;
} AcquisitionPlatform;

/*
 * AcquisitionPlatform_init :
 * 	initializes AcuisitionPlatform
 */
AcquisitionPlatform* AcquisitionPlatform_init(void);

/*
 * AcquisitionPlatform_imuUpdate :
 * 	updates data stored in acq_pl reading new ones from imu
 */
void AcquisitionPlatform_imuUpdate(AcquisitionPlatform* acq_pl);

/*
 * AcquisitionPlatform_destroy :
 * 	destroys previously created AcuisitionPlatform
 */
void AcquisitionPlatform_destroy(AcquisitionPlatform* acq_pl);
