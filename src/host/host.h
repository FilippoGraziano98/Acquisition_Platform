#pragma once

#include <stdint.h>

#include "../common/packets.h"

typedef struct Host {
	// file descriptor of the serial port
	int serial_fd;
	//uint8_t packet_buffer[PACKET_SIZE_MAX];
	
	//seq number of the latest packets received
  uint16_t global_seq;
  
	//these are the system variables, updated by the serial communiction
	IMUConfigurationPacket imu_config_packet;
	
  AccelerometerPacket accelerometer_packet;
  GyroscopePacket gyroscope_packet;
  MagnetometerPacket magnetometer_packet;
} Host;

/*
 * Host_init :
 * 	creates a new host, opening a serial connection on a device
 */
Host* Host_init(const char* device);

/*
 * Host_checkConnection :
 * 	sends a few (cycles) EchoPacket testing serial connection
 */
int Host_checkConnection(Host* host, int cycles);

/*
 * Host_getIMUConfiguration :
 *  asks the controller for update imu configuration
 * 	and saves it in host->imu_config_packet
 */
int Host_getIMUConfiguration(Host* host);

/*
 * Host_getAccelerometerData :
 *  asks the controller for update accelerometer_data
 * 	and saves it in host->acceleremoter_packet
 */
int Host_getAccelerometerData(Host* host);

/*
 * Host_getGyroscopeData :
 *  asks the controller for update gyroscope_data
 * 	and saves it in host->gyroscope_packet
 */
int Host_getGyroscopeData(Host* host);

/*
 * Host_getMagnetometerData :
 *  asks the controller for update magnetometer_data
 * 	and saves it in host->magnetometer_packet
 */
int Host_getMagnetometerData(Host* host);

/*
 * Host_printIMUConfiguration :
 *  prints to stdout imu configuration
 */
void Host_printIMUConfiguration(Host* host);

/*
 * Host_printIMUData :
 *  prints to stdout accelerometer_data, gyroscope_data
 */
void Host_printIMUData(Host* host);

/*
 * Host_destroy :
 * 		destroyes a previouslt created host
 *	@returns:
 *			0 on success,
 *			on error, -1 is returned, and errno is set by close
 */
int Host_destroy(Host* host);
