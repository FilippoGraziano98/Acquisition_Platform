#include <stdlib.h>

#include "acquisition_platform.h"

AcquisitionPlatform* AcquisitionPlatform_init() {
	AcquisitionPlatform* acq_pl = (AcquisitionPlatform*)malloc(sizeof(AcquisitionPlatform));
	
	acq_pl->global_seq = 0;
	
	acq_pl->gyroscope_data.gyro_x = 0;
	acq_pl->gyroscope_data.gyro_y = 0;
	acq_pl->gyroscope_data.gyro_z = 0;
	
	return acq_pl;
}

void AcquisitionPlatform_imuUpdate(AcquisitionPlatform* acq_pl) {
	acq_pl->gyroscope_data = IMU_GyroscopeData();
	
	acq_pl->global_seq++;
}

void AcquisitionPlatform_destroy(AcquisitionPlatform* acq_pl) {
	free(acq_pl);
}
