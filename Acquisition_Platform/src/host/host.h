#pragma once

#include <stdint.h>

#include "../common/packets.h"



#ifdef __cplusplus
extern "C" {
#endif



typedef struct Host_t {
	// file descriptor of the serial port
	int serial_fd;
	//uint8_t packet_buffer[PACKET_SIZE_MAX];
	
	//seq number of the latest packets received
  uint16_t global_seq;
  
	//these are the system variables, updated by the serial communiction
  EncoderPacket encoder_packet;
	OdometryPacket odom_packet;
	
	IMUConfigurationPacket imu_config_packet;
	
  AccelerometerPacket accelerometer_packet;
  GyroscopePacket gyroscope_packet;
  MagnetometerPacket magnetometer_packet;
} Host_t;

/*
 * Host_init :
 * 	initializes the global host, opening a serial connection on a device
 */
int Host_init(const char* device);

/*
 * Host_checkConnection :
 * 	sends a few (cycles) EchoPacket testing serial connection
 */
int Host_checkConnection(int cycles);

/*
 * Host_get<...>Data :
 *	sends correspondent packet to the controller
 *		( not waiting for the answer )
 *  	asking the controller for updated data
 */
//int Host_getEncoderData(void);	//deprecated
//int Host_getOdometryData(void);	//deprecated
//int Host_getIMUConfiguration(void);
//int Host_getAccelerometerData(void);
//int Host_getGyroscopeData(void);
//int Host_getMagnetometerData(void);

/*
 * Host_get<...>Data :
 *	fils the packet taken as parameter with correct info
 */
void Host_getOdometryData(OdometryPacket* odom);

/*
 * Host_print<...>Data :
 *  prints to stdout correspondent data
 */
void Host_printEncoderData(void);
void Host_printOdometryData(void);
void Host_printIMUConfiguration(void);
void Host_printIMUData(void);

/*
 * Host_destroy :
 * 		destroyes a previouslt created host
 *	@returns:
 *			0 on success,
 *			on error, -1 is returned, and errno is set by close
 */
int Host_destroy(void);



#ifdef __cplusplus
}
#endif
