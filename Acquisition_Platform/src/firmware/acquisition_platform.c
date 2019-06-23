#include <stdlib.h>
#include <string.h>

#include "acquisition_platform.h"

// private AcquisitionPlatform variable;
static AcquisitionPlatform_t acq_pl;


void AcquisitionPlatform_init() {
	memset(&acq_pl, 0, sizeof(AcquisitionPlatform_t));
	
	INIT_PACKET(acq_pl.accel_data, ACCELEROMETER_PACKET_ID);
	INIT_PACKET(acq_pl.gyro_data, GYROSCOPE_PACKET_ID);
	INIT_PACKET(acq_pl.encs_data, ENCODER_PACKET_ID);
	INIT_PACKET(acq_pl.sensors_data, SENSORS_PACKET_ID);
}

#if KF_VERSION==1
static const float sin_taylor_coeffs[] = {1., 0., -1./6., 0., 1./120., 0.};
static const float cos_taylor_coeffs[] = {0., 1./2., 0., -1./24., 0., 1./720.};

static void computeThetaTerms(float* sin_theta_over_theta, float* one_minus_cos_theta_over_theta, float theta) {
	*sin_theta_over_theta = 0;
	*one_minus_cos_theta_over_theta = 0;
	float theta_exp = 1;
	
	uint8_t i;
	for(i=0; i < (sizeof(sin_taylor_coeffs)/sizeof(float)); i++) {
		*sin_theta_over_theta += sin_taylor_coeffs[i]*theta_exp;
		*one_minus_cos_theta_over_theta += cos_taylor_coeffs[i]*theta_exp;
		
		theta_exp *= theta;
	}
}

static void AcquisitionPlatform_preprocessEncoderData(float delta_l, float delta_r) {
	float delta_plus = delta_r + delta_l;
	float delta_minus = delta_r - delta_l;
	
	float delta_theta = delta_minus / BASE_LEN;
	
	float sin_dtheta_over_dtheta, one_minus_cos_dtheta_over_dtheta;
	computeThetaTerms(&sin_dtheta_over_dtheta, &one_minus_cos_dtheta_over_dtheta, delta_theta);
	
	float delta_x = .5 * delta_plus * sin_dtheta_over_dtheta;
	float delta_y = .5 * delta_plus * one_minus_cos_dtheta_over_dtheta;
	
	acq_pl.sensors_data.local_dx = delta_x;
	acq_pl.sensors_data.local_dy = delta_y;
	acq_pl.sensors_data.local_dtheta = delta_theta;
}
#endif

uint8_t AcquisitionPlatform_reassembleSensorsData() {
	uint8_t valid_data = 1;
	
	valid_data &= IMU_getAccelerometer(&acq_pl.accel_data);
	valid_data &= IMU_getGyroscope(&acq_pl.gyro_data);
	//if( !valid_data )
		//return 0;
	
	//imu
		//TODO low-pass filter
	acq_pl.sensors_data.imu_accel_x = acq_pl.accel_data.accel_x * G_FORCE_ACCEL_M_S2;
	acq_pl.sensors_data.imu_accel_y = acq_pl.accel_data.accel_y * G_FORCE_ACCEL_M_S2;
	acq_pl.sensors_data.imu_vel_theta = acq_pl.gyro_data.gyro_z * M_PI / M_180;
	
	//encoders
	int32_t encs_cnt[NUM_ENCODERS];
	Encoders_getCounts(encs_cnt);
	
	int32_t left_ticks = encs_cnt[0] - acq_pl.encs_cnt_previous[0];
	int32_t right_ticks = encs_cnt[1] - acq_pl.encs_cnt_previous[1];
	

	float delta_l = left_ticks * (float)METERS_PER_ENCODER_TICK;
	float delta_r = right_ticks * (float)METERS_PER_ENCODER_TICK;


	#if KF_VERSION==0
	acq_pl.sensors_data.delta_l = delta_l;
	acq_pl.sensors_data.delta_r = delta_r;
	#elif KF_VERSION==1
	AcquisitionPlatform_preprocessEncoderData(delta_l, delta_r);
	#endif
	
	
		
	if( left_ticks || right_ticks ) {
		//updates previous values stored
		acq_pl.encs_cnt_previous[0] = encs_cnt[0];
		acq_pl.encs_cnt_previous[1] = encs_cnt[1];
	}
	
	acq_pl.global_seq++;
	return 1;
}

uint8_t AcquisitionPlatform_sendSensorsDataToHost() {
	uint8_t res, ret = 0;
	
	acq_pl.sensors_data.header.seq = acq_pl.global_seq;

	res = UART_send_packet((PacketHeader*)&(acq_pl.sensors_data));
	if(res > 0)
		ret++;
	
	return ret;
}
