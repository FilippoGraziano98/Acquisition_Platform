#include "host.h"

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "serial/serial.h"
#include "serial/serial_packets.h"


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
	
	//initializes global packets memory
	INIT_PACKET(host->gyroscope_packet, GYROSCOPE_PACKET_ID);
	
	//waits for controller to be ready
	sleep(1);
	
	return host;
}

int Host_checkConnection(Host* host, int cycles) {
	EchoPacket send_pkt;
	INIT_PACKET(send_pkt, ECHO_PACKET_ID);
	
	EchoPacket recv_pkt;
	INIT_PACKET(recv_pkt, ECHO_PACKET_ID);
	
	int i;
	for(i=0; i<cycles; i++) {
		memset(&recv_pkt, 0, sizeof(EchoPacket));
		
		send_pkt.info = i;
		serial_send_packet(host->serial_fd, (PacketHeader*)&send_pkt);
		
		serial_receive_packet(host->serial_fd, (PacketHeader*)&recv_pkt);
		
		if( memcmp(&send_pkt, &recv_pkt, sizeof(EchoPacket)) !=0 )
			return -1;
	}
	return 0;
}

int Host_getGyroscopeData(Host* host) {
	//asks the controller for update gyroscope_data
	serial_send_packet(host->serial_fd, (PacketHeader*)&(host->gyroscope_packet));
	//saves it in host->gyroscope_packet
	serial_receive_packet(host->serial_fd, (PacketHeader*)&(host->gyroscope_packet));
	
	host->global_seq = (host->global_seq > host->gyroscope_packet.header.seq) ? host->global_seq : host->gyroscope_packet.header.seq;
	
	return 0;
}

void Host_printGyroscopeData(Host* host) {
	printf("[Gyroscope %d] x-axis: %f, y-axis: %f, z-axis: %f\n", host->global_seq, host->gyroscope_packet.gyro_x, host->gyroscope_packet.gyro_y, host->gyroscope_packet.gyro_z);
}

int Host_destroy(Host* host) {
	int res;
	res = close(host->serial_fd);
	
	free(host);
	return res;
}
