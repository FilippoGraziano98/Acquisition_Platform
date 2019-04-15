#include "host.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "serial/serial.h"
#include "serial/serial_communication.h"


#define SERIAL_SPEED 57600
#define SERIAL_PARITY 0

Host* Host_init(const char* device) {
	int res;
	
	Host* host = (Host*)malloc(sizeof(Host));
	
	host->serial_fd = serial_open(device);
	if ( host->serial_fd < 0 ) {
		printf("[Host_init] Error in serial_open for SERIAL_NAME : %s\n", device);
		free(host);
		return NULL;
	}		
	
	res = serial_set_interface_attribs(host->serial_fd, SERIAL_SPEED, SERIAL_PARITY);
	if ( res < 0 ) {
		printf("[Host_init] Error in serial_set_interface_attribs for SERIAL_SPEED : %d and SERIAL_PARITY : %d\n", SERIAL_SPEED, SERIAL_PARITY);
		free(host);
		return NULL;
	}
	
	host->global_seq = 0;
	
	//initializes global packets memory
	INIT_PACKET(host->encoder_packet, ENCODER_PACKET_ID);
	INIT_PACKET(host->imu_config_packet, IMU_CONFIG_PACKET_ID);
	INIT_PACKET(host->accelerometer_packet, ACCELEROMETER_PACKET_ID);
	INIT_PACKET(host->gyroscope_packet, GYROSCOPE_PACKET_ID);
	INIT_PACKET(host->magnetometer_packet, MAGNETOMETER_PACKET_ID);
	
	//initialize Host_serial (packets_ interface)
	Host_Serial_init(host->serial_fd);
	
	//waits for controller to be ready
	sleep(1);
	
	return host;
}

int Host_checkConnection(Host* host, int cycles) {
	EchoPacket send_pkt;
	INIT_PACKET(send_pkt, ECHO_PACKET_ID);
	
	EchoPacket recv_pkt;
	INIT_PACKET(recv_pkt, ECHO_PACKET_ID);
	
	int i, res;
	for(i=0; i<cycles; i++) {
		//resets recv_pkt
		memset((uint8_t*)&recv_pkt+sizeof(PacketHeader), 0, sizeof(EchoPacket)-sizeof(PacketHeader));
		
		send_pkt.info = i;
		res = Host_Serial_sendPacket((PacketHeader*)&send_pkt);
		if( res != SERIAL__SUCCESS)
			printf("[Host_checkConnection, iter: %d/10] Host_Serial_sendPacket error_code : %d\n", i, res);
		
		res = Host_Serial_receivePacket((PacketHeader*)&recv_pkt);
		if( res != SERIAL__SUCCESS)
			printf("[Host_checkConnection, iter: %d/10] Host_Serial_receivePacket error_code : %d\n", i, res);
		
		if( memcmp(&send_pkt, &recv_pkt, sizeof(EchoPacket)) !=0 )
			return -1;
	}
	return 0;
}

int Host_getEncoderData(Host* host) {
	int res;
	//asks the controller for update imu configuration
	res = Host_Serial_sendPacket((PacketHeader*)&(host->encoder_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getEncoderData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in host->encoder_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(host->encoder_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getEncoderData] Host_Serial_receivePacket error_code : %d\n", res);
	
	return 0;
}

int Host_getIMUConfiguration(Host* host) {
	int res;
	//asks the controller for update imu configuration
	res = Host_Serial_sendPacket((PacketHeader*)&(host->imu_config_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getIMUConfiguration] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in host->imu_config_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(host->imu_config_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getIMUConfiguration] Host_Serial_receivePacket error_code : %d\n", res);
	
	return 0;
}

int Host_getAccelerometerData(Host* host) {
	int res;
	//asks the controller for update accelerometer_data
	res = Host_Serial_sendPacket((PacketHeader*)&(host->accelerometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getAccelerometerData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in host->accelerometer_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(host->accelerometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getAccelerometerData] Host_Serial_receivePacket error_code : %d\n", res);
	
	host->global_seq = (host->global_seq > host->accelerometer_packet.header.seq) ? host->global_seq : host->accelerometer_packet.header.seq;
	
	return 0;
}

int Host_getGyroscopeData(Host* host) {
	int res;
	//asks the controller for update gyroscope_data
	res = Host_Serial_sendPacket((PacketHeader*)&(host->gyroscope_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getGyroscopeData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in host->gyroscope_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(host->gyroscope_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getGyroscopeData] Host_Serial_receivePacket error_code : %d\n", res);
	
	host->global_seq = (host->global_seq > host->gyroscope_packet.header.seq) ? host->global_seq : host->gyroscope_packet.header.seq;
	
	return 0;
}

int Host_getMagnetometerData(Host* host) {
	int res;
	//asks the controller for update gyroscope_data
	res = Host_Serial_sendPacket((PacketHeader*)&(host->magnetometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getMagnetometerData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in host->magnetometer_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(host->magnetometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getMagnetometerData] Host_Serial_receivePacket error_code : %d\n", res);
	
	host->global_seq = (host->global_seq > host->magnetometer_packet.header.seq) ? host->global_seq : host->magnetometer_packet.header.seq;
	
	return 0;
}


void Host_printEncoderData(Host* host) {
	printf("[Encoder] counter: %d\n", host->encoder_packet.counter);
}

void Host_printIMUConfiguration(Host* host) {
	printf("=== IMU CONFIGURATION ===\n");
	printf("Gyroscope [seq: %d]:\n", host->imu_config_packet.header.seq);
	printf("\tx-axis bias: %d\n\ty-axis bias: %d\n\tz-axis bias: %d\n", host->imu_config_packet.gyro_x_bias, host->imu_config_packet.gyro_y_bias, host->imu_config_packet.gyro_z_bias);
}

void Host_printIMUData(Host* host) {
	printf("[Accelerometer %d] x-axis: %f, y-axis: %f, z-axis: %f\n", host->global_seq, host->accelerometer_packet.accel_x, host->accelerometer_packet.accel_y, host->accelerometer_packet.accel_z);
	printf("[Gyroscope %d] x-axis: %f, y-axis: %f, z-axis: %f\n", host->global_seq, host->gyroscope_packet.gyro_x, host->gyroscope_packet.gyro_y, host->gyroscope_packet.gyro_z);
	printf("[Magnetometer %d] x-axis: %f, y-axis: %f, z-axis: %f\n", host->global_seq, host->magnetometer_packet.magnet_x, host->magnetometer_packet.magnet_y, host->magnetometer_packet.magnet_z);
	printf("\n");
}

int Host_destroy(Host* host) {
	int res;
	res = close(host->serial_fd);
	
	free(host);
	return res;
}
