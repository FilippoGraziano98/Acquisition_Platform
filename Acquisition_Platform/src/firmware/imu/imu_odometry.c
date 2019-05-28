#include <math.h>
#include <string.h>

#include <avr/interrupt.h>
#include <util/atomic.h>

#include "imu_odometry.h"

#ifdef DEBUG_PRINTF
#include <stdio.h>
#endif

//static float rotation_matrix[IMU_N*IMU_N];

static IMU_OdometryController_t IMU_OdometryController;

ISR(TIMER4_COMPA_vect) {
	uint8_t valid_data = 1;
	
	valid_data &= IMU_getAccelerometer(&IMU_OdometryController.accel_values);
	valid_data &= IMU_getGyroscope(&IMU_OdometryController.gyro_values);
	if( valid_data ) {
		IMU_OdometryUpdate();
		IMU_OdometryController.imu_time_seq++;
	}
}

static void IMU_setPeriodicOdometryUpdate(uint16_t frequency) {
  uint16_t period_ms = 1000 / frequency; //from a frequency in Hz, we get a period in millisecs
  
  // configure timer1, prescaler : 256, CTC (Clear Timer on Compare match)
  TCCR4A = 0;
  TCCR4B = (1 << WGM12) | (1 << CS12); 
  
  /*
	 * cpu frequency 16MHz = 16.000.000 Hz
	 * prescaler 256
	 *	-->> TCNT1 increased at a frequency of 16.000.000/256 Hz = 62500 Hz
	 *	so 1 ms will correspond do 62.5 counts
	 */
  OCR4A = (uint16_t)(62.5 * period_ms);

	// timer-interrupt enabling will be executed atomically (no other interrupts)
		// and ATOMIC_FORCEON ensures Global Interrupt Status flag bit in SREG set afetrwards
		// (sei() not needed)
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
  	TIMSK4 |= (1 << OCIE4A);  // enable the timer interrupt (istruz. elementare, no interrupt)
  }
}

void IMU_OdometryInit(void) {
	memset(&IMU_OdometryController, 0, sizeof(IMU_OdometryController_t));
	
	IMU_OdometryController.delta_time = 1. / (float)IMU_ODOMETRY_UPDATE_RATE;

	INIT_PACKET(IMU_OdometryController.odometry_status, IMU_ODOMETRY_PACKET_ID);

	IMU_setPeriodicOdometryUpdate(IMU_ODOMETRY_UPDATE_RATE);
}

//TODO set static?
void IMU_OdometryUpdate() {
	
	// filtro passa-alto per le accelerazioni traslazionali
	IMU_OdometryController.odometry_status.translational_acceleration_x_axis = (fabs(IMU_OdometryController.accel_values.accel_x) > IMU_TRASL_ACC_THRESHOLD) ? IMU_OdometryController.accel_values.accel_x*G_FORCE_ACCEL : 0.;
	IMU_OdometryController.odometry_status.translational_acceleration_y_axis = (fabs(IMU_OdometryController.accel_values.accel_y) > IMU_TRASL_ACC_THRESHOLD) ? IMU_OdometryController.accel_values.accel_y*G_FORCE_ACCEL : 0.;
	IMU_OdometryController.odometry_status.translational_acceleration_z_axis = (fabs(IMU_OdometryController.accel_values.accel_z) > IMU_TRASL_ACC_THRESHOLD) ? IMU_OdometryController.accel_values.accel_z*G_FORCE_ACCEL : 0.;
	
	
	//integro nel vettore spostamento (relativo all'attuale orientazione)
	// dx = v*t + .5*a*t*t = (v + .5*a*t)*t
	float delta_x_local = (IMU_OdometryController.odometry_status.translational_velocity_x_axis + .5*IMU_OdometryController.odometry_status.translational_acceleration_x_axis*IMU_OdometryController.delta_time) * IMU_OdometryController.delta_time;
	float delta_y_local = (IMU_OdometryController.odometry_status.translational_velocity_y_axis + .5*IMU_OdometryController.odometry_status.translational_acceleration_y_axis*IMU_OdometryController.delta_time) * IMU_OdometryController.delta_time;
	float delta_z_local = (IMU_OdometryController.odometry_status.translational_velocity_z_axis + .5*IMU_OdometryController.odometry_status.translational_acceleration_z_axis*IMU_OdometryController.delta_time) * IMU_OdometryController.delta_time;
	
	// update global odom x, y, z
	float sin_yaw = sin(IMU_OdometryController.odometry_status.imu_yaw);
	float cos_yaw = cos(IMU_OdometryController.odometry_status.imu_yaw);
	
	IMU_OdometryController.odometry_status.imu_odom_x += delta_x_local*cos_yaw - delta_y_local*sin_yaw,
	IMU_OdometryController.odometry_status.imu_odom_y += delta_x_local*sin_yaw + delta_y_local*cos_yaw;
	IMU_OdometryController.odometry_status.imu_odom_z += delta_z_local;// 0.0
	
	// integro nelle velocità traslazionali
	IMU_OdometryController.odometry_status.translational_velocity_x_axis += IMU_OdometryController.odometry_status.translational_acceleration_x_axis*IMU_OdometryController.delta_time;
	IMU_OdometryController.odometry_status.translational_velocity_y_axis += IMU_OdometryController.odometry_status.translational_acceleration_y_axis*IMU_OdometryController.delta_time;
	IMU_OdometryController.odometry_status.translational_velocity_z_axis += IMU_OdometryController.odometry_status.translational_acceleration_z_axis*IMU_OdometryController.delta_time;
	
	// filtro passa-alto per le velocità angolari
	IMU_OdometryController.odometry_status.rotational_velocity_z_axis = (fabs(IMU_OdometryController.gyro_values.gyro_z) > IMU_ANG_VEL_THRESHOLD) ? IMU_OdometryController.gyro_values.gyro_z : 0.;
	IMU_OdometryController.odometry_status.rotational_velocity_y_axis = (fabs(IMU_OdometryController.gyro_values.gyro_y) > IMU_ANG_VEL_THRESHOLD) ? IMU_OdometryController.gyro_values.gyro_y : 0.;
	IMU_OdometryController.odometry_status.rotational_velocity_x_axis = (fabs(IMU_OdometryController.gyro_values.gyro_x) > IMU_ANG_VEL_THRESHOLD) ? IMU_OdometryController.gyro_values.gyro_x : 0.;
			
	//update yaw, roll, pitch
	IMU_OdometryController.odometry_status.imu_yaw += IMU_OdometryController.odometry_status.rotational_velocity_z_axis*IMU_OdometryController.delta_time;
	IMU_OdometryController.odometry_status.imu_pitch += IMU_OdometryController.odometry_status.rotational_velocity_y_axis*IMU_OdometryController.delta_time;
	IMU_OdometryController.odometry_status.imu_roll += IMU_OdometryController.odometry_status.rotational_velocity_x_axis*IMU_OdometryController.delta_time;
}


uint8_t IMU_sendOdometryToHost(void) {
	uint8_t res, ret = 0;
	
	IMU_OdometryController.odometry_status.header.seq = IMU_OdometryController.imu_time_seq;

	res = UART_send_packet((PacketHeader*)&(IMU_OdometryController.odometry_status));
	if(res > 0)
		ret++;
	
	return ret;
}
