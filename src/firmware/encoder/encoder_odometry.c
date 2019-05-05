#include <math.h>

#include <avr/interrupt.h>
#include <util/atomic.h>

#include "encoder_odometry.h"


#ifdef DEBUG_PRINTF
#include <stdio.h>
#endif


static Encoder_OdometryController_t Encoder_OdometryController;

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

ISR(TIMER2_COMPA_vect) {
	Encoder_OdometryUpdate();

	Encoder_OdometryController.enc_time_seq++;
}

static void Encoder_setPeriodicOdometryUpdate(uint16_t frequency) {
	//we are now using Timer2
  uint16_t period_ms = 1000 / frequency; //from a frequency in Hz, we get a period in millisecs
  
  // configure timer1, prescaler : 256, CTC (Clear Timer on Compare match)
  TCCR2A = 0;
  TCCR2B = (1 << WGM12) | (1 << CS12); 
  
  /*
	 * cpu frequency 16MHz = 16.000.000 Hz
	 * prescaler 256
	 *	-->> TCNT1 increased at a frequency of 16.000.000/256 Hz = 62500 Hz
	 *	so 1 ms will correspond do 62.5 counts
	 */
  OCR2A = (uint16_t)(62.5 * period_ms);

	// timer-interrupt enabling will be executed atomically (no other interrupts)
		// and ATOMIC_FORCEON ensures Global Interrupt Status flag bit in SREG set afetrwards
		// (sei() not needed)
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
  	TIMSK2 |= (1 << OCIE2A);  // enable the timer interrupt (istruz. elementare, no interrupt)
  }
}


void Encoder_OdometryInit(void) {
	Encoder_OdometryController.enc_time_seq = 0;
	
	Encoder_OdometryController.delta_time = 1. / (float)ODOMETRY_UPDATE_RATE;

	Encoder_OdometryController.odom_x = 0.;
	Encoder_OdometryController.odom_y = 0.;
	Encoder_OdometryController.odom_theta = 0.;

	Encoder_OdometryController.translational_velocity = 0.;
	Encoder_OdometryController.rotational_velocity = 0.;

	uint8_t i;
	for(i=0; i<NUM_ENCODERS; i++)
		Encoder_OdometryController.encs_cnt_previous[i] = 0;
	
	//we are going to update odomotery at 100Hz
	Encoder_setPeriodicOdometryUpdate(ODOMETRY_UPDATE_RATE);
}

void Encoder_OdometryUpdate(void) {
	int32_t encs_cnt[NUM_ENCODERS];
	Encoders_getCounts(encs_cnt);
	
	int32_t left_ticks = encs_cnt[0] - Encoder_OdometryController.encs_cnt_previous[0];
	int32_t right_ticks = encs_cnt[1] - Encoder_OdometryController.encs_cnt_previous[1];
	
	//if encoders didn't detect any move,
		//robot didn't move
	if( left_ticks || right_ticks ) {
		//updates previous values stored
		Encoder_OdometryController.encs_cnt_previous[0] = encs_cnt[0];
		Encoder_OdometryController.encs_cnt_previous[1] = encs_cnt[1];
		
		//gets movement in meters
		float delta_l = left_ticks * (float)METERS_PER_ENCODER_TICK;
		float delta_r = right_ticks * (float)METERS_PER_ENCODER_TICK;
		
		float delta_plus = delta_r + delta_l;
		float delta_minus = delta_r - delta_l;
		
		float delta_theta = delta_minus / BASE_LEN;
		
		float sin_dtheta_over_dtheta, one_minus_cos_dtheta_over_dtheta;
		computeThetaTerms(&sin_dtheta_over_dtheta, &one_minus_cos_dtheta_over_dtheta, delta_theta);
		
		float delta_x = .5 * delta_plus * sin_dtheta_over_dtheta;
		float delta_y = .5 * delta_plus * one_minus_cos_dtheta_over_dtheta;		
		
		#ifdef DEBUG_ODOM
		Encoder_OdometryController.enc_left = left_ticks;
		Encoder_OdometryController.enc_right = right_ticks;
		Encoder_OdometryController.delta_l = delta_l;
		Encoder_OdometryController.delta_r = delta_r;
		Encoder_OdometryController.delta_x = delta_x;
		Encoder_OdometryController.delta_y = delta_y;
		Encoder_OdometryController.delta_theta = delta_theta;
		#endif
		
		//update global odometry
		float sin_theta = sin(Encoder_OdometryController.odom_theta);
		float cos_theta = cos(Encoder_OdometryController.odom_theta);
		
		Encoder_OdometryController.odom_x += delta_x*cos_theta - delta_y*sin_theta;
		Encoder_OdometryController.odom_y += delta_x*sin_theta + delta_y*cos_theta;
		Encoder_OdometryController.odom_theta += delta_theta;
		
		//update global current volocity
		Encoder_OdometryController.translational_velocity = .5 * delta_plus / Encoder_OdometryController.delta_time;
		Encoder_OdometryController.rotational_velocity = delta_theta / Encoder_OdometryController.delta_time;
	}
}

#ifndef DEBUG_ODOM
uint16_t Encoder_getOdometry(float* odom_x, float* odom_y, float* odom_theta, float* t_vel, float* r_vel) {
	*odom_x = Encoder_OdometryController.odom_x;
	*odom_y = Encoder_OdometryController.odom_y;
	*odom_theta = Encoder_OdometryController.odom_theta;
	
	*t_vel = Encoder_OdometryController.translational_velocity;
	*r_vel = Encoder_OdometryController.rotational_velocity;
	
	return Encoder_OdometryController.enc_time_seq;
}
#else
uint16_t Encoder_getOdometry(float* odom_x, float* odom_y, float* odom_theta, float* t_vel, float* r_vel, int32_t* enc_left, int32_t* enc_right, float* delta_l, float* delta_r, float* delta_x, float* delta_y, float* delta_theta) {
	*odom_x = Encoder_OdometryController.odom_x;
	*odom_y = Encoder_OdometryController.odom_y;
	*odom_theta = Encoder_OdometryController.odom_theta;
	*t_vel = Encoder_OdometryController.translational_velocity;
	*r_vel = Encoder_OdometryController.rotational_velocity;
	
	*enc_left = Encoder_OdometryController.enc_left;
	*enc_right = Encoder_OdometryController.enc_right;
	*delta_l = Encoder_OdometryController.delta_l;
	*delta_r = Encoder_OdometryController.delta_r;
	*delta_x = Encoder_OdometryController.delta_x;
	*delta_y = Encoder_OdometryController.delta_y;
	*delta_theta = Encoder_OdometryController.delta_theta;
	
	return Encoder_OdometryController.enc_time_seq;
}
#endif
