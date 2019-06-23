#pragma once

#include <stdint.h>

#include "firmware_constants.h"
#include "packets/uart_packets.h"
#include "imu/imu.h"
#include "encoder/encoder.h"


#define M_PI 3.141592654
#define M_180 180

#define WHEEL_RADIUS 0.0255	//meters
#define BASE_LEN 0.196	//meters

#define METERS_PER_ENCODER_TICK (M_PI*WHEEL_RADIUS / ENCODER_TICKS_FOR_HALF_TURN)



#define G_FORCE_ACCEL_M_S2 9.8 //accel[m/s^2] = accel[G-Force]*9.8

typedef struct AcquisitionPlatform {	
	//seq number of the latest packets sent
		//seq is set when updating data from imu
		//and then copied in the packets' headers
	uint16_t global_seq;
	
	//these are the system variables
		//updated reading data from imu
	AccelerometerPacket accel_data; //in G-forces
	GyroscopePacket gyro_data;			//in DPS
		//and from encoders
	EncoderPacket	encs_data;
	
	// encoder saved from the previous iteration
	int32_t encs_cnt_previous[NUM_ENCODERS];
	
	//data ready to be sent
	SensorsPacket sensors_data;
} AcquisitionPlatform_t;

/*
 * AcquisitionPlatform_init :
 * 	initializes AcquisitionPlatform
 */
void AcquisitionPlatform_init(void);

/*
 * AcquisitionPlatform_sendSensorsDataToHost :
 * 	updates sensor readings
 *	fills SensorsPacket
 */
uint8_t AcquisitionPlatform_reassembleSensorsData(void);

/*
 * AcquisitionPlatform_sendSensorsDataToHost :
 *	sends SensorsPacket to the host
 */
uint8_t AcquisitionPlatform_sendSensorsDataToHost(void);

