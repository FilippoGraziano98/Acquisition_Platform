#include "host.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "serial/serial.h"
#include "serial/serial_communication.h"


#define SERIAL_SPEED 57600
#define SERIAL_PARITY 0

// global Host variable
static Host_t Global_Host;

//callbacks for corresponding pkts
static void Host_printSystemStatusPkt(PacketHeader* pkt);
static void Host_saveEncoderPkt(PacketHeader* pkt);
static void Host_saveOdometryPkt(PacketHeader* pkt);
static void Host_saveAccelerometerPkt(PacketHeader* pkt);
static void Host_saveGyroscopePkt(PacketHeader* pkt);
static void Host_saveMagnetometerPkt(PacketHeader* pkt);
static void Host_saveIMUOdometryPkt(PacketHeader* pkt);


int Host_init(const char* device) {
	int res;
	
	Global_Host.serial_fd = serial_open(device);
	if ( Global_Host.serial_fd < 0 ) {
		printf("[Host_init] Error in serial_open for SERIAL_NAME : %s\n", device);
		return -1;
	}
	
	serial_reset(Global_Host.serial_fd);
	
	res = serial_set_interface_attribs(Global_Host.serial_fd, SERIAL_SPEED, SERIAL_PARITY);
	if ( res < 0 ) {
		printf("[Host_init] Error in serial_set_interface_attribs for SERIAL_SPEED : %d and SERIAL_PARITY : %d\n", SERIAL_SPEED, SERIAL_PARITY);
		return -1;
	}
	
	Global_Host.global_seq = 0;
	
	//initializes global packets memory
	INIT_PACKET(Global_Host.encoder_packet, ENCODER_PACKET_ID);
	INIT_PACKET(Global_Host.odom_packet, ODOMETRY_PACKET_ID);
	INIT_PACKET(Global_Host.imu_config_packet, IMU_CONFIG_PACKET_ID);
	INIT_PACKET(Global_Host.accelerometer_packet, ACCELEROMETER_PACKET_ID);
	INIT_PACKET(Global_Host.gyroscope_packet, GYROSCOPE_PACKET_ID);
	INIT_PACKET(Global_Host.magnetometer_packet, MAGNETOMETER_PACKET_ID);
	INIT_PACKET(Global_Host.imu_odom_packet, IMU_ODOMETRY_PACKET_ID);
	
	//initialize Host_serial (packets_ interface)
	Host_Serial_init(Global_Host.serial_fd);

	//initialization of packetOps_vector
	res = Host_Serial_registerPacketHandler(SYSTEM_STATUS_PACKET_ID, Host_printSystemStatusPkt);
	res |= Host_Serial_registerPacketHandler(ENCODER_PACKET_ID, Host_saveEncoderPkt);
	res |= Host_Serial_registerPacketHandler(ODOMETRY_PACKET_ID, Host_saveOdometryPkt);
	res |= Host_Serial_registerPacketHandler(ACCELEROMETER_PACKET_ID, Host_saveAccelerometerPkt);
	res |= Host_Serial_registerPacketHandler(GYROSCOPE_PACKET_ID, Host_saveGyroscopePkt);
	res |= Host_Serial_registerPacketHandler(MAGNETOMETER_PACKET_ID, Host_saveMagnetometerPkt);
	res |= Host_Serial_registerPacketHandler(IMU_ODOMETRY_PACKET_ID, Host_saveIMUOdometryPkt);
	if ( res < 0 ) {
		printf("[Host_init] Error in Host_Serial_registerPacketHandler\n");
		return -1;
	}
	
	//waits for controller to be ready
	sleep(1);
	
	return 0;
}

int Host_checkConnection(int cycles) {
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
			printf("[Host_checkConnection, iter: %d/%d] Host_Serial_sendPacket error_code : %d\n", i, cycles, res);
		
		res = Host_Serial_receivePacket((PacketHeader*)&recv_pkt);
		if( res != SERIAL__SUCCESS)
			printf("[Host_checkConnection, iter: %d/%d] Host_Serial_receivePacket error_code : %d\n", i, cycles, res);
		
		if( memcmp(&send_pkt, &recv_pkt, sizeof(EchoPacket)) !=0 )
			return -1;
	}
	return 0;
}


static void Host_printSystemStatusPkt(PacketHeader* pkt) {
	if( pkt->type != SYSTEM_STATUS_PACKET_ID) {
		printf("[Host_printSystemStatusPkt] Packet Handling CORRUPTED: pkt is not SystemStatusPacket\n");
		return;
	}
	
	SystemStatusPacket* system_pkt = (SystemStatusPacket*)pkt;
	printf("*****\n");
	printf("time: %d secs, idle_cycles: %d, rx_count: %d, tx_count: %d\n", system_pkt->global_secs_count, system_pkt->idle_cycles, system_pkt->rx_count, system_pkt->tx_count);
}

static void Host_saveEncoderPkt(PacketHeader* pkt) {
	if( pkt->type != ENCODER_PACKET_ID) {
		printf("[Host_saveEncoderPkt] Packet Handling CORRUPTED: pkt is not EncoderPacket\n");
		return;
	}
	
	memcpy(&(Global_Host.encoder_packet), pkt, sizeof(EncoderPacket));
}

static void Host_saveOdometryPkt(PacketHeader* pkt) {
	if( pkt->type != ODOMETRY_PACKET_ID) {
		printf("[Host_saveOdometryPkt] Packet Handling CORRUPTED: pkt is not OdometryPacket\n");
		return;
	}
	
	memcpy(&(Global_Host.odom_packet), pkt, sizeof(OdometryPacket));
}

static void Host_saveAccelerometerPkt(PacketHeader* pkt) {
	if( pkt->type != ACCELEROMETER_PACKET_ID) {
		printf("[Host_saveAccelerometerPkt] Packet Handling CORRUPTED: pkt is not AccelerometerPacket\n");
		return;
	}
	
	memcpy(&(Global_Host.accelerometer_packet), pkt, sizeof(AccelerometerPacket));
}

static void Host_saveGyroscopePkt(PacketHeader* pkt) {
	if( pkt->type != GYROSCOPE_PACKET_ID) {
		printf("[Host_saveGyroscopePkt] Packet Handling CORRUPTED: pkt is not GyroscopePacket\n");
		return;
	}
	
	memcpy(&(Global_Host.gyroscope_packet), pkt, sizeof(GyroscopePacket));
}

static void Host_saveMagnetometerPkt(PacketHeader* pkt) {
	if( pkt->type != MAGNETOMETER_PACKET_ID) {
		printf("[Host_saveMagnetometerPkt] Packet Handling CORRUPTED: pkt is not MagnetometerPacket\n");
		return;
	}
	
	memcpy(&(Global_Host.magnetometer_packet), pkt, sizeof(MagnetometerPacket));
}


static void Host_saveIMUOdometryPkt(PacketHeader* pkt) {
	if( pkt->type != IMU_ODOMETRY_PACKET_ID) {
		printf("[Host_saveIMUOdometryPkt] Packet Handling CORRUPTED: pkt is not IMUOdometryPacket\n");
		return;
	}
	
	memcpy(&(Global_Host.imu_odom_packet), pkt, sizeof(IMUOdometryPacket));
}

/** DEPRECATED
int Host_getEncoderData() {
	int res;
	//asks the controller for update imu configuration
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.encoder_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getEncoderData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.encoder_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.encoder_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getEncoderData] Host_Serial_receivePacket error_code : %d\n", res);
	
	return 0;
}

int Host_getOdometryData() {
	int res;
	//asks the controller for update imu configuration
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.odom_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getOdometryData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.odom_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.odom_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getOdometryData] Host_Serial_receivePacket error_code : %d\n", res);
	
	return 0;
}
*/
int Host_getIMUConfiguration() {
	int res;
	//asks the controller for update imu configuration
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.imu_config_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getIMUConfiguration] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.imu_config_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.imu_config_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getIMUConfiguration] Host_Serial_receivePacket error_code : %d\n", res);
	
	return 0;
}
/*
int Host_getAccelerometerData() {
	int res;
	//asks the controller for update accelerometer_data
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.accelerometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getAccelerometerData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.accelerometer_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.accelerometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getAccelerometerData] Host_Serial_receivePacket error_code : %d\n", res);
	
	Global_Host.global_seq = (Global_Host.global_seq > Global_Host.accelerometer_packet.header.seq) ? Global_Host.global_seq : Global_Host.accelerometer_packet.header.seq;
	
	return 0;
}

int Host_getGyroscopeData() {
	int res;
	//asks the controller for update gyroscope_data
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.gyroscope_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getGyroscopeData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.gyroscope_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.gyroscope_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getGyroscopeData] Host_Serial_receivePacket error_code : %d\n", res);
	
	Global_Host.global_seq = (Global_Host.global_seq > Global_Host.gyroscope_packet.header.seq) ? Global_Host.global_seq : Global_Host.gyroscope_packet.header.seq;
	
	return 0;
}

int Host_getMagnetometerData() {
	int res;
	//asks the controller for update gyroscope_data
	res = Host_Serial_sendPacket((PacketHeader*)&(Global_Host.magnetometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getMagnetometerData] Host_Serial_sendPacket error_code : %d\n", res);
	
	//saves it in Global_Host.magnetometer_packet
	res = Host_Serial_receivePacket((PacketHeader*)&(Global_Host.magnetometer_packet));
	if( res != SERIAL__SUCCESS)
		printf("[Host_getMagnetometerData] Host_Serial_receivePacket error_code : %d\n", res);
	
	Global_Host.global_seq = (Global_Host.global_seq > Global_Host.magnetometer_packet.header.seq) ? Global_Host.global_seq : Global_Host.magnetometer_packet.header.seq;
	
	return 0;
}
*/


void Host_getOdometryData(OdometryPacket* odom) {
	memcpy(odom, &(Global_Host.odom_packet), sizeof(OdometryPacket));
}

void Host_getIMUOdometryData(IMUOdometryPacket* imu_odom) {
	memcpy(imu_odom, &(Global_Host.imu_odom_packet), sizeof(IMUOdometryPacket));
}



void Host_printEncoderData() {
	printf("[EncoderLeft] counter: %d\n", Global_Host.encoder_packet.counters[0]);
	printf("[EncoderRight] counter: %d\n", Global_Host.encoder_packet.counters[1]);
	printf("\n");
}

void Host_printOdometryData() {
	#ifdef DEBUG_ODOM
	printf("%d) left %fm [%d ticks], right %fm [%d ticks]\n", Global_Host.odom_packet.header.seq, Global_Host.odom_packet.delta_l, Global_Host.odom_packet.enc_left, Global_Host.odom_packet.delta_r, Global_Host.odom_packet.enc_right);

	printf("%d) delta_x %fm, delta_y %fm, delta_theta %f\n", Global_Host.odom_packet.header.seq, Global_Host.odom_packet.delta_x, Global_Host.odom_packet.delta_y, Global_Host.odom_packet.delta_theta);
	#endif
	
	printf("%d) odom_x %fm, odom_y %fm, odom_theta %f\n", Global_Host.odom_packet.header.seq, Global_Host.odom_packet.odom_x, Global_Host.odom_packet.odom_y, Global_Host.odom_packet.odom_theta);
	printf("%d) translational_velocity %fm/s, rotational_velocity %frad/s\n", Global_Host.odom_packet.header.seq, Global_Host.odom_packet.translational_velocity, Global_Host.odom_packet.rotational_velocity);
	
	printf("\n");
}

void Host_printIMUConfiguration() {
	printf("=== IMU CONFIGURATION ===\n");
	printf("Gyroscope [seq: %d]:\n", Global_Host.imu_config_packet.header.seq);
	printf("\tx-axis bias: %d\n\ty-axis bias: %d\n\tz-axis bias: %d\n", Global_Host.imu_config_packet.gyro_x_bias, Global_Host.imu_config_packet.gyro_y_bias, Global_Host.imu_config_packet.gyro_z_bias);
	printf("Accelerometer [seq: %d]:\n", Global_Host.imu_config_packet.header.seq);
	printf("\tx-axis bias: %d\n\ty-axis bias: %d\n\tz-axis bias: %d\n", Global_Host.imu_config_packet.accel_x_bias, Global_Host.imu_config_packet.accel_y_bias, Global_Host.imu_config_packet.accel_z_bias);
}

void Host_printIMUData() {
	printf("Global Seq: %d\n", Global_Host.global_seq);
	printf("[Accelerometer %d] x-axis: %f, y-axis: %f, z-axis: %f\n", Global_Host.accelerometer_packet.header.seq, Global_Host.accelerometer_packet.accel_x, Global_Host.accelerometer_packet.accel_y, Global_Host.accelerometer_packet.accel_z);
	printf("[Gyroscope %d] x-axis: %f, y-axis: %f, z-axis: %f\n", Global_Host.gyroscope_packet.header.seq, Global_Host.gyroscope_packet.gyro_x, Global_Host.gyroscope_packet.gyro_y, Global_Host.gyroscope_packet.gyro_z);
	printf("[Magnetometer %d] x-axis: %f, y-axis: %f, z-axis: %f\n", Global_Host.magnetometer_packet.header.seq, Global_Host.magnetometer_packet.magnet_x, Global_Host.magnetometer_packet.magnet_y, Global_Host.magnetometer_packet.magnet_z);
	printf("\n");
}

void Host_printIMUOdometryData() {
	printf("%d) yaw(z): %f, pitch(y): %f, roll(x): %f\n", Global_Host.imu_odom_packet.header.seq, Global_Host.imu_odom_packet.imu_yaw, Global_Host.imu_odom_packet.imu_pitch, Global_Host.imu_odom_packet.imu_roll);
	printf("%d) rotational velocities: z-axis: %f, y-axis: %f, x-axis: %f\n", Global_Host.imu_odom_packet.header.seq, Global_Host.imu_odom_packet.rotational_velocity_z_axis, Global_Host.imu_odom_packet.rotational_velocity_y_axis, Global_Host.imu_odom_packet.rotational_velocity_x_axis);
	printf("\n");
}

int Host_destroy() {
	int res;
	serial_reset(Global_Host.serial_fd);
	
	res = close(Global_Host.serial_fd);
	
	return res;
}
