#pragma once
#include "stdint.h"

#include "../../common/packets.h"

//CHECK this is congrous with firmware
#define KF_ODOMETRY_UPDATE_RATE 100 // Hz
#define BASE_LEN 0.196	//meters

#define PROCESS_NOISE_COV 0.01 // m/s^3

#define OBSERV_IMU_ACCL_NOISE_COV 0.3
#define OBSERV_IMU_GYRO_NOISE_COV 0.003

#define OBSERV_ENC_NOISE_COV 0.00001
#define OBSERV_ENC_DXY_NOISE_COV OBSERV_ENC_NOISE_COV
#define OBSERV_ENC_DTHETA_NOISE_COV 0.0001




#define KF_STATUS_LEN 8

//KF_STATUS indexes following
#define KF_ODOM_X 0
#define KF_TRANS_VEL_X 1
#define KF_TRANS_ACCL_X 2

#define KF_ODOM_Y 3
#define KF_TRANS_VEL_Y 4
#define KF_TRANS_ACCL_Y 5

#define KF_ODOM_THETA 6
#define KF_ROT_VEL_Z 7


#if KF_VERSION==0

	#define KF_OBSERVATION_LEN 5

	#define KF_OBS_ENC_LEFT 0
	#define KF_OBS_ENC_RIGHT 1
	
	#ifdef KF_IMU
		#define KF_OBS_IMU_ACCL_X 2
		#define KF_OBS_IMU_ACCL_Y 3

		#define KF_OBS_IMU_ROTV_Z 4
		#define KF_OBSERVATION_LEN 5
	#else
		#define KF_OBSERVATION_LEN 2
	#endif
	
#elif KF_VERSION==1

	#define KF_OBS_ENC_DX 0
	#define KF_OBS_ENC_DY 1
	#define KF_OBS_ENC_DTHETA 2
	
	#ifdef KF_IMU
		#define KF_OBS_IMU_ACCL_X 3
		#define KF_OBS_IMU_ACCL_Y 4

		#define KF_OBS_IMU_ROTV_Z 5
	
		#define KF_OBSERVATION_LEN 6
	#else
		#define KF_OBSERVATION_LEN 3
	#endif
#endif


#ifdef __cplusplus
extern "C" {
#endif


typedef struct KalmanFilter_Odometry_t {
	//sequence increased at each timer interrupt occured
	uint16_t kf_time_seq;
	//sequence of the last registered movement TODO
	uint16_t last_move_seq;
	
	//time between two consecutive updates
	float delta_time;	//secs
		
	//I will treat the status as a Gaussian distribution,
		//with the following mean and covariance
	float status_mean[KF_STATUS_LEN];
	float status_covariance[KF_STATUS_LEN][KF_STATUS_LEN];
	
	float observation_matrix[KF_OBSERVATION_LEN][KF_STATUS_LEN];
		
} KalmanFilter_Odometry_t;

void KalmanFilter_OdometryInit(void);

void KalmanFilter_OdometryUpdate(SensorsPacket* sens_obs);

void KalmanFilter_OdometryPrint(void);


#ifdef __cplusplus
}
#endif
