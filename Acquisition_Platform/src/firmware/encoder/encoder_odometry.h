#pragma once
#include "encoder.h"

#include "../packets/uart_packets.h"

#define M_PI 3.141592654

#define WHEEL_RADIUS 0.0255	//meters
#define BASE_LEN 0.196	//meters

#define METERS_PER_ENCODER_TICK (M_PI*WHEEL_RADIUS / ENCODER_TICKS_FOR_HALF_TURN)

#define ODOMETRY_UPDATE_RATE 100	//Hz

typedef struct Encoder_OdometryController_t{
	//sequence increased at each timer interrupt occured
	uint16_t enc_time_seq;
	//sequence of the last registered movement TODO
	uint16_t last_move_seq;
	
	//time between two consecutive updates
	float delta_time;	//secs
	
	//odometry is stored in a packet ready to be sent
	OdometryPacket odometry_status;
	
	// encoder saved from the previous iteration
	int32_t encs_cnt_previous[NUM_ENCODERS];
} Encoder_OdometryController_t;


void Encoder_OdometryInit(void);

// TODO set static??
void Encoder_OdometryUpdate(void);


/*
 * Encoder_get<Sensor>
 *	@params: ptr to variables where to store values
 *	@returns: seq value of the sensor
 */
#ifndef DEBUG_ODOM
uint16_t Encoder_getOdometry(float* odom_x, float* odom_y, float* odom_theta, float* t_vel, float* r_vel);
#else
uint16_t Encoder_getOdometry(float* odom_x, float* odom_y, float* odom_theta, float* t_vel, float* r_vel, int32_t* enc_left, int32_t* enc_right, float* delta_l, float* delta_r, float* delta_x, float* delta_y, float* delta_theta);
#endif

uint8_t Encoder_sendOdometryToHost(void);
