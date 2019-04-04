#include <stdlib.h>

#include "acquisition_platform.h"


AcquisitionPlatform* AcquisitionPlatform_init() {
	AcquisitionPlatform* acq_pl = (AcquisitionPlatform*)malloc(sizeof(AcquisitionPlatform));
	
	acq_pl->global_seq = 0;
	
	acq_pl->accelerometer_data = (AccelerometerData){ .accel_x = 0, .accel_y = 0, .accel_z = 0 };
	acq_pl->gyroscope_data = (GyroscopeData){ .gyro_x = 0, .gyro_y = 0, .gyro_z = 0 };
	acq_pl->magnetometer_data = (MagnetometerData){ .magnet_x = 0, .magnet_y = 0, .magnet_z = 0 };
	
	return acq_pl;
}

void AcquisitionPlatform_imuUpdate(AcquisitionPlatform* acq_pl) {
	acq_pl->accelerometer_data = IMU_AccelerometerData();
	acq_pl->gyroscope_data = IMU_GyroscopeData();
	acq_pl->magnetometer_data = IMU_MagnetometerData();
	
	acq_pl->global_seq++;
}

void AcquisitionPlatform_destroy(AcquisitionPlatform* acq_pl) {
	free(acq_pl);
}
