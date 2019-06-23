#include <stdio.h>
#include <string.h>
#include <math.h>

#include "matrix.h"

#include "kalman_filter.h"

static float TRANSITION_MATRIX[KF_STATUS_LEN][KF_STATUS_LEN];
static float TRANSITION_MATRIX_TRANSPOSE[KF_STATUS_LEN][KF_STATUS_LEN];

//transition noise will be modelled with 3 components:
	//[third_derivate_x, third_derivate_y, second_derivate_theta]

#define TRANSITION_NOISE_LEN 3
/*
#define TRANSITION_NOISE_X 0
#define TRANSITION_NOISE_Y 1
#define TRANSITION_NOISE_THETA 2
static float PROCESS_NOISE_COVARIANCE[TRANSITION_NOISE_LEN][TRANSITION_NOISE_LEN];

static float TRANSITION_NOISE_MATRIX[KF_STATUS_LEN][TRANSITION_NOISE_LEN];
static float TRANSITION_NOISE_MATRIX_TRANSPOSE[TRANSITION_NOISE_LEN][KF_STATUS_LEN];
*/
static float PROCESS_NOISE_CONTRIBUTE_MATRIX[KF_STATUS_LEN][KF_STATUS_LEN];


static float OBSERVATION_COVARIANCE_MATRIX[KF_OBSERVATION_LEN][KF_OBSERVATION_LEN];

static KalmanFilter_Odometry_t KF_odom;




static void KalmanFilter_matrix_init(void) {
	//TRANSITION_MATRIX
	memset(&TRANSITION_MATRIX, 0, sizeof(TRANSITION_MATRIX));
	
		//x axis
	TRANSITION_MATRIX[KF_ODOM_X][KF_ODOM_X] = 1.;
	TRANSITION_MATRIX[KF_ODOM_X][KF_TRANS_VEL_X] = KF_odom.delta_time;
	TRANSITION_MATRIX[KF_ODOM_X][KF_TRANS_ACCL_X] = .5*KF_odom.delta_time*KF_odom.delta_time;
	
	TRANSITION_MATRIX[KF_TRANS_VEL_X][KF_TRANS_VEL_X] = 1.;
	TRANSITION_MATRIX[KF_TRANS_VEL_X][KF_TRANS_ACCL_X] = KF_odom.delta_time;
	
	TRANSITION_MATRIX[KF_TRANS_ACCL_X][KF_TRANS_ACCL_X] = 1.;
	
		//y axis
	TRANSITION_MATRIX[KF_ODOM_Y][KF_ODOM_Y] = 1.;
	TRANSITION_MATRIX[KF_ODOM_Y][KF_TRANS_VEL_Y] = KF_odom.delta_time;
	TRANSITION_MATRIX[KF_ODOM_Y][KF_TRANS_ACCL_Y] = .5*KF_odom.delta_time*KF_odom.delta_time;
	
	TRANSITION_MATRIX[KF_TRANS_VEL_Y][KF_TRANS_VEL_Y] = 1.;
	TRANSITION_MATRIX[KF_TRANS_VEL_Y][KF_TRANS_ACCL_Y] = KF_odom.delta_time;
	
	TRANSITION_MATRIX[KF_TRANS_ACCL_Y][KF_TRANS_ACCL_Y] = 1.;
	
		//theta
	TRANSITION_MATRIX[KF_ODOM_THETA][KF_ODOM_THETA] = 1.;
	TRANSITION_MATRIX[KF_ODOM_THETA][KF_ROT_VEL_Z] = KF_odom.delta_time;
	
	TRANSITION_MATRIX[KF_ROT_VEL_Z][KF_ROT_VEL_Z] = 1.;
	
	//TRANSITION_MATRIX_TRANSPOSE
	matrix_transpose(KF_STATUS_LEN, KF_STATUS_LEN, TRANSITION_MATRIX, TRANSITION_MATRIX_TRANSPOSE);
	
	/*
	//PROCESS_NOISE_COVARIANCE
	matrix_set_identity(TRANSITION_NOISE_LEN, PROCESS_NOISE_COVARIANCE);
	matrix_scalar_mul(TRANSITION_NOISE_LEN,TRANSITION_NOISE_LEN, PROCESS_NOISE_COVARIANCE, PROCESS_NOISE_COV, PROCESS_NOISE_COVARIANCE);
	
	//TRANSITION_NOISE_MATRIX
	memset(&TRANSITION_NOISE_MATRIX, 0, sizeof(TRANSITION_NOISE_MATRIX));
	TRANSITION_NOISE_MATRIX[KF_ODOM_X][TRANSITION_NOISE_X] = KF_odom.delta_time*KF_odom.delta_time*KF_odom.delta_time / 6.;
	TRANSITION_NOISE_MATRIX[KF_TRANS_VEL_X][TRANSITION_NOISE_X] = .5*KF_odom.delta_time*KF_odom.delta_time;
	TRANSITION_NOISE_MATRIX[KF_TRANS_ACCL_X][TRANSITION_NOISE_X] = KF_odom.delta_time;
	
	TRANSITION_NOISE_MATRIX[KF_ODOM_Y][TRANSITION_NOISE_X] = KF_odom.delta_time*KF_odom.delta_time*KF_odom.delta_time / 6.;
	TRANSITION_NOISE_MATRIX[KF_TRANS_VEL_Y][TRANSITION_NOISE_X] = .5*KF_odom.delta_time*KF_odom.delta_time;
	TRANSITION_NOISE_MATRIX[KF_TRANS_ACCL_Y][TRANSITION_NOISE_X] = KF_odom.delta_time;
	
	TRANSITION_NOISE_MATRIX[KF_ODOM_THETA][TRANSITION_NOISE_THETA] = .5*KF_odom.delta_time*KF_odom.delta_time;
	TRANSITION_NOISE_MATRIX[KF_ROT_VEL_Z][TRANSITION_NOISE_THETA] = KF_odom.delta_time;
	
	//TRANSITION_NOISE_MATRIX_TRANSPOSE
	matrix_transpose(KF_STATUS_LEN, TRANSITION_NOISE_LEN, TRANSITION_NOISE_MATRIX, TRANSITION_NOISE_MATRIX_TRANSPOSE);
	*/
	
	//PROCESS_NOISE_CONTRIBUTE_MATRIX
	float process_noise_covariance[TRANSITION_NOISE_LEN][TRANSITION_NOISE_LEN] =
			{{PROCESS_NOISE_COV, 0., 0.},
			{0.,PROCESS_NOISE_COV,0.},
			{0.,0.,PROCESS_NOISE_COV}};
	
	float dt3_6 = KF_odom.delta_time*KF_odom.delta_time*KF_odom.delta_time / 6.;
	float dt2_2 = .5*KF_odom.delta_time*KF_odom.delta_time;
	float dt = KF_odom.delta_time;
	float transition_noise_matrix[KF_STATUS_LEN][TRANSITION_NOISE_LEN] = 
			{{dt3_6,0.,0.},
			{dt2_2,0.,0.},
			{dt,0.,0.},
			{0.,dt3_6,0.},
			{0.,dt2_2,0.},
			{0.,dt,0.},
			{0.,0.,dt2_2},
			{0.,0.,dt}};
	float transition_noise_matrix_transpose[TRANSITION_NOISE_LEN][KF_STATUS_LEN];
	matrix_transpose(KF_STATUS_LEN, TRANSITION_NOISE_LEN, transition_noise_matrix, transition_noise_matrix_transpose);
	
	float aux_cov[KF_STATUS_LEN][TRANSITION_NOISE_LEN];
	matrix_product(KF_STATUS_LEN, TRANSITION_NOISE_LEN, TRANSITION_NOISE_LEN, transition_noise_matrix, process_noise_covariance, aux_cov);
	matrix_product(KF_STATUS_LEN, TRANSITION_NOISE_LEN, KF_STATUS_LEN, aux_cov, transition_noise_matrix_transpose, PROCESS_NOISE_CONTRIBUTE_MATRIX);
	
	//OBSERVATION_COVARIANCE_MATRIX
	//TODO not necessarily diagonal !!
	#if KF_VERSION==0
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_ENC_LEFT][KF_OBS_ENC_LEFT] = OBSERV_ENC_NOISE_COV;
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_ENC_RIGHT][KF_OBS_ENC_RIGHT] = OBSERV_ENC_NOISE_COV;
	
	#elif KF_VERSION==1
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_ENC_DX][KF_OBS_ENC_DX] = OBSERV_ENC_DXY_NOISE_COV;
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_ENC_DY][KF_OBS_ENC_DY] = OBSERV_ENC_DXY_NOISE_COV;
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_ENC_DTHETA][KF_OBS_ENC_DTHETA] = OBSERV_ENC_DTHETA_NOISE_COV;
	#endif
	
	#ifdef KF_IMU
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_IMU_ACCL_X][KF_OBS_IMU_ACCL_X] = OBSERV_IMU_ACCL_NOISE_COV;
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_IMU_ACCL_Y][KF_OBS_IMU_ACCL_Y] = OBSERV_IMU_ACCL_NOISE_COV;
	OBSERVATION_COVARIANCE_MATRIX[KF_OBS_IMU_ROTV_Z][KF_OBS_IMU_ROTV_Z] = OBSERV_IMU_GYRO_NOISE_COV;
	#endif
}

void KalmanFilter_OdometryInit(void) {
	memset(&KF_odom, 0, sizeof(KalmanFilter_Odometry_t));
	
	KF_odom.delta_time = 1. / KF_ODOMETRY_UPDATE_RATE;
	
	KalmanFilter_matrix_init();	
	
	#ifdef DEBUG_KF_MATRIXES
	printf(" ** TRANSITION_MATRIX ** \n");
	matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, TRANSITION_MATRIX);
	printf(" ** PROCESS_NOISE_CONTRIBUTE_MATRIX ** \n");
	matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, PROCESS_NOISE_CONTRIBUTE_MATRIX);
	printf(" ** OBSERVATION_COVARIANCE_MATRIX ** \n");
	matrix_print(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, OBSERVATION_COVARIANCE_MATRIX);
	#endif
}





static void KalmanFilter_TransitionModel(void) {
	//mean = A*mean
	matrix_vector_product(KF_STATUS_LEN, KF_STATUS_LEN, TRANSITION_MATRIX, KF_odom.status_mean, KF_odom.status_mean);
	
	//covariance = A*Cov*A_t
	square_matrix_product(KF_STATUS_LEN, TRANSITION_MATRIX, KF_odom.status_covariance, KF_odom.status_covariance);
	square_matrix_product(KF_STATUS_LEN, KF_odom.status_covariance, TRANSITION_MATRIX_TRANSPOSE, KF_odom.status_covariance);

	//covariance += B*Cov_noise*B_t
	matrix_add(KF_STATUS_LEN, KF_STATUS_LEN, KF_odom.status_covariance, PROCESS_NOISE_CONTRIBUTE_MATRIX, KF_odom.status_covariance);
	
}

//note: @param obs is an array of KF_OBSERVATION_LEN elements
static void KalmanFilter_ObservationModel(float obs[], int8_t debug) {
	//TODO
	float sin_theta = sin(KF_odom.status_mean[KF_ODOM_THETA]);
	float cos_theta = cos(KF_odom.status_mean[KF_ODOM_THETA]);
	
	float dt2_2 = .5*KF_odom.delta_time*KF_odom.delta_time;
	float dt = KF_odom.delta_time;
	
	memset(&KF_odom.observation_matrix, 0, sizeof(KF_odom.observation_matrix));
	
	//ENCODERS
	#if KF_VERSION==0
	KF_odom.observation_matrix[KF_OBS_ENC_RIGHT][KF_TRANS_VEL_X] = cos_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_RIGHT][KF_TRANS_ACCL_X] = cos_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_RIGHT][KF_TRANS_VEL_Y] = sin_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_RIGHT][KF_TRANS_ACCL_Y] = sin_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_RIGHT][KF_ROT_VEL_Z] = .5*BASE_LEN*dt;
		
	KF_odom.observation_matrix[KF_OBS_ENC_LEFT][KF_TRANS_VEL_X] = cos_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_LEFT][KF_TRANS_ACCL_X] = cos_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_LEFT][KF_TRANS_VEL_Y] = sin_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_LEFT][KF_TRANS_ACCL_Y] = sin_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_LEFT][KF_ROT_VEL_Z] = -.5*BASE_LEN*dt;
	#elif KF_VERSION==1
	
	KF_odom.observation_matrix[KF_OBS_ENC_DX][KF_TRANS_VEL_X] = cos_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_DX][KF_TRANS_ACCL_X] = cos_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_DX][KF_TRANS_VEL_Y] = sin_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_DX][KF_TRANS_ACCL_Y] = sin_theta*dt2_2;
		
	KF_odom.observation_matrix[KF_OBS_ENC_DY][KF_TRANS_VEL_X] = -sin_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_DY][KF_TRANS_ACCL_X] = -sin_theta*dt2_2;
	KF_odom.observation_matrix[KF_OBS_ENC_DY][KF_TRANS_VEL_Y] = cos_theta*dt;
	KF_odom.observation_matrix[KF_OBS_ENC_DY][KF_TRANS_ACCL_Y] = cos_theta*dt2_2;
	
	KF_odom.observation_matrix[KF_OBS_ENC_DTHETA][KF_ROT_VEL_Z] = dt;
	#endif
	
	
	// IMU
	//nel reference frame del robot:
		//	accel_x_local = accel_x_global * cos_theta + accel_y_global * sin_theta
		//	accel_y_local = -accel_x_global * sin_theta + accel_y_global * cos_theta
	#ifdef KF_IMU
	KF_odom.observation_matrix[KF_OBS_IMU_ACCL_X][KF_TRANS_ACCL_X] = cos_theta;
	KF_odom.observation_matrix[KF_OBS_IMU_ACCL_X][KF_TRANS_ACCL_Y] = sin_theta;
	
	KF_odom.observation_matrix[KF_OBS_IMU_ACCL_Y][KF_TRANS_ACCL_X] = -sin_theta;
	KF_odom.observation_matrix[KF_OBS_IMU_ACCL_Y][KF_TRANS_ACCL_Y] = cos_theta;
	
	KF_odom.observation_matrix[KF_OBS_IMU_ROTV_Z][KF_ROT_VEL_Z] = 1.;
	#endif
	
	//attended observation = C*status_mean
	float attended_obs[KF_OBSERVATION_LEN];
	matrix_vector_product(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_odom.observation_matrix, KF_odom.status_mean, attended_obs);
	
	if(debug) {
		printf(" ** attended_obs ** \n");
		vector_print(KF_OBSERVATION_LEN, attended_obs);
	}
	
	
	//K = cov_status*C_T * inverse(cov_noise + C*cov_status*C_T)
	float K_matrix[KF_STATUS_LEN][KF_OBSERVATION_LEN];
	
	float obs_matrix_transpose[KF_STATUS_LEN][KF_OBSERVATION_LEN];
	matrix_transpose(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_odom.observation_matrix, obs_matrix_transpose);
	
	if(debug) {
		printf(" ** observation_matrix ** \n");
		matrix_print(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_odom.observation_matrix);
		matrix_print(KF_STATUS_LEN, KF_OBSERVATION_LEN, obs_matrix_transpose);
	}
	
	
	float aux_m_5_8[KF_OBSERVATION_LEN][KF_STATUS_LEN];
	matrix_product(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_STATUS_LEN, KF_odom.observation_matrix, KF_odom.status_covariance, aux_m_5_8);
	
	if(debug) {
		printf(" ** aux_m_5_8 ** \n");
		matrix_print(KF_OBSERVATION_LEN, KF_STATUS_LEN, aux_m_5_8);
	}
	
	
	float aux_m_5_5[KF_OBSERVATION_LEN][KF_OBSERVATION_LEN];
	matrix_product(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_OBSERVATION_LEN, aux_m_5_8, obs_matrix_transpose, aux_m_5_5);
	
	if(debug) {
		printf(" ** aux_m_5_5 ** \n");
		matrix_print(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, aux_m_5_5);
	}
	
	
	matrix_add(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, OBSERVATION_COVARIANCE_MATRIX, aux_m_5_5, aux_m_5_5);
	
	if(debug) {
		printf(" ** aux_m_5_5 ** \n");
		matrix_print(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, aux_m_5_5);
	}
	
	
	matrix_inverse(KF_OBSERVATION_LEN, aux_m_5_5, aux_m_5_5);
	
	if(debug) {
		printf(" ** aux_m_5_5 ** \n");
		matrix_print(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, aux_m_5_5);
	}
	
	
	float aux_m_8_5[KF_STATUS_LEN][KF_OBSERVATION_LEN];
	matrix_product(KF_STATUS_LEN, KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, obs_matrix_transpose, aux_m_5_5, aux_m_8_5);
	
	if(debug) {
		printf(" ** aux_m_8_5 ** \n");
		matrix_print(KF_STATUS_LEN, KF_OBSERVATION_LEN, aux_m_8_5);
	}
	
	
	matrix_product(KF_STATUS_LEN, KF_STATUS_LEN, KF_OBSERVATION_LEN, KF_odom.status_covariance, aux_m_8_5, K_matrix);
	
	if(debug) {
		printf(" ** K_matrix ** \n");
		matrix_print(KF_STATUS_LEN, KF_OBSERVATION_LEN, K_matrix);
	}
	
	
	// status_mean += K_matrix * (obs - attended_obs)
	float delta_obs[KF_OBSERVATION_LEN];
	vector_sub(KF_OBSERVATION_LEN, obs, attended_obs, delta_obs);
	
	if(debug) {
		printf(" ** delta_obs ** \n");
		vector_print(KF_OBSERVATION_LEN, delta_obs);
	}
	
	
	float delta_status[KF_STATUS_LEN];
	matrix_vector_product(KF_STATUS_LEN, KF_OBSERVATION_LEN, K_matrix, delta_obs, delta_status);
	
	if(debug) {
		printf(" ** delta_status ** \n");
		vector_print(KF_STATUS_LEN, delta_status);
	}
	
	
	vector_add(KF_STATUS_LEN, KF_odom.status_mean, delta_status, KF_odom.status_mean);
	
	//status_covariance -= (K_matrix*C ) * status_covariance
	float aux_m_8_8[KF_STATUS_LEN][KF_STATUS_LEN];
	matrix_product(KF_STATUS_LEN, KF_OBSERVATION_LEN, KF_STATUS_LEN, K_matrix, KF_odom.observation_matrix, aux_m_8_8);
	
	if(debug) {
		printf(" ** aux_m_8_8 ** \n");
		matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, aux_m_8_8);
	}
	
	
	square_matrix_product(KF_STATUS_LEN, aux_m_8_8, KF_odom.status_covariance, aux_m_8_8);
	
	if(debug) {
		printf(" ** aux_m_8_8 ** \n");
		matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, aux_m_8_8);
	}
	
	
	matrix_sub(KF_STATUS_LEN, KF_STATUS_LEN, KF_odom.status_covariance, aux_m_8_8, KF_odom.status_covariance);
	
	if(debug) {
		printf("Mean:\n");
		vector_print(KF_STATUS_LEN, KF_odom.status_mean);
		printf("Covariance:\n");
		matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, KF_odom.status_covariance);
	}
	
	
}

#ifdef DEBUG_KF_OBS
static int enc_log_idx = 0;
static float enc_dx_log[8192] = {0.};
static float enc_dy_log[8192] = {0.};
static float enc_dtheta_log[8192] = {0.};


static float status_x_log[8192] = {0.};
#endif

void KalmanFilter_OdometryUpdate(SensorsPacket* sens_obs) {
	KalmanFilter_TransitionModel();
	
	float obs[KF_OBSERVATION_LEN];
	#if KF_VERSION==0
	obs[KF_OBS_ENC_LEFT] = sens_obs->delta_l;
	obs[KF_OBS_ENC_RIGHT] = sens_obs->delta_r;
	#elif KF_VERSION==1
	obs[KF_OBS_ENC_DX] = sens_obs->local_dx;
	obs[KF_OBS_ENC_DY] = sens_obs->local_dy;
	obs[KF_OBS_ENC_DTHETA] = sens_obs->local_dtheta;
	#endif
	#ifdef KF_IMU
	obs[KF_OBS_IMU_ACCL_X] = sens_obs->imu_accel_x;
	obs[KF_OBS_IMU_ACCL_Y] = sens_obs->imu_accel_y;
	obs[KF_OBS_IMU_ROTV_Z] = sens_obs->imu_vel_theta;
	#endif

	#ifndef DEBUG_KF_MATRIXES
	KalmanFilter_ObservationModel(obs,0);
	#else
	if( (sens_obs->header.seq & 0x7f) == 0) {
		printf("\n\nDEBUG %d\n", sens_obs->header.seq);
		KalmanFilter_ObservationModel(obs,1);
		printf("\n======\n");
	}
	else
		KalmanFilter_ObservationModel(obs,0);
	#endif
	
	#ifdef DEBUG_KF_OBS
	enc_dx_log[enc_log_idx] = obs[KF_OBS_ENC_DX];
	enc_dy_log[enc_log_idx] = obs[KF_OBS_ENC_DY];
	enc_dtheta_log[enc_log_idx] = obs[KF_OBS_ENC_DTHETA];
	status_x_log[enc_log_idx] = KF_odom.status_mean[KF_ODOM_X];
	enc_log_idx++;
	#endif
}

void KalmanFilter_OdometryPrint(void){
	printf("Mean:\n");
	vector_print(KF_STATUS_LEN, KF_odom.status_mean);
	printf("Covariance:\n");
	matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, KF_odom.status_covariance);
	
	
	#ifdef DEBUG_KF_MATRIXES
	printf(" ** TRANSITION_MATRIX ** \n");
	matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, TRANSITION_MATRIX);
	printf(" ** PROCESS_NOISE_CONTRIBUTE_MATRIX ** \n");
	matrix_print(KF_STATUS_LEN, KF_STATUS_LEN, PROCESS_NOISE_CONTRIBUTE_MATRIX);
	printf(" ** OBSERVATION_COVARIANCE_MATRIX ** \n");
	matrix_print(KF_OBSERVATION_LEN, KF_OBSERVATION_LEN, OBSERVATION_COVARIANCE_MATRIX);
	printf(" ** observation_matrix ** \n");
	matrix_print(KF_OBSERVATION_LEN, KF_STATUS_LEN, KF_odom.observation_matrix);
	#endif
	
	#ifdef DEBUG_KF_OBS
	int i;
	
	
	printf("status_x_log\n");
	for(i=0; i<enc_log_idx; i++)
		printf("%f, ",status_x_log[i]);
	printf("\n");
	
	printf("enc_dx_log\n");
	for(i=0; i<enc_log_idx; i++)
		printf("%f, ",enc_dx_log[i]);
	printf("\n");
	
	printf("enc_dy_log\n");
	for(i=0; i<enc_log_idx; i++)
		printf("%f, ",enc_dy_log[i]);
	printf("\n");
	
	printf("enc_dtheta_log\n");
	for(i=0; i<enc_log_idx; i++)
		printf("%f, ",enc_dtheta_log[i]);
	printf("\n");
	#endif
}




