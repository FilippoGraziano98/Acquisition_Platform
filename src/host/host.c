#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include "serial.h"
#include "serial_packets.h"
#include "../common/packets.h"

#define SERIAL_NAME "/dev/ttyACM0"
#define SERIAL_SPEED 38400
#define SERIAL_PARITY 0


#define MAX_BUF 256



int main(int argc, char** argv) {
	int res;
	int serial_fd = serial_open(SERIAL_NAME);
	if ( serial_fd < 0 ) {
		printf("[] Error in serial_open for SERIAL_NAME : %s\n", SERIAL_NAME);
		return -1;
	}		
	
	res = serial_set_interface_attribs(serial_fd, SERIAL_SPEED, SERIAL_PARITY);
	if ( res < 0 ) {
		printf("[] Error in serial_set_interface_attribs for SERIAL_SPEED : %d and SERIAL_PARITY : %d\n", SERIAL_SPEED, SERIAL_PARITY);
		return -1;
	}
	
	printf("[] Opened communication with controller on serial port : %s\n", SERIAL_NAME);
	
	//waits for controller to be ready
	sleep(1);
	
	EchoPacket send_pkt;
	INIT_PACKET(send_pkt, ECHO_PACKET_ID);
	
	EchoPacket recv_pkt;
	INIT_PACKET(recv_pkt, ECHO_PACKET_ID);
	
	int i;
	for(i=0; i<50; i++) {
		memset(&recv_pkt, 0, sizeof(EchoPacket));
		
		send_pkt.info = i;
		serial_send_packet(serial_fd, (PacketHeader*)&send_pkt);
		printf((char*)"[host] sent %d\n", send_pkt.info);
		
		
		serial_receive_packet(serial_fd, (PacketHeader*)&recv_pkt);
		printf((char*)"[host] received %d\n", recv_pkt.info);
		
		if( memcmp(&send_pkt, &recv_pkt, sizeof(EchoPacket)) !=0 )
			printf("\t ERROR\n");
		
		printf("\n");
	}
	
	serial_close(serial_fd);
	printf("[] Closed communication with controller on serial port : %s\n", SERIAL_NAME);
}
