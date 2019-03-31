#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include "serial_packets.h"
#include "serial.h"
#include "../common/packets.h"

int serial_send_packet(int fd, PacketHeader* pkt) {
	return serial_send(fd, (uint8_t*)pkt, pkt->size);
}

int serial_receive_packet(int fd, PacketHeader* pkt) {	
	PacketHeader head;
	
	int res_head = serial_receive(fd, (uint8_t*)&head, sizeof(PacketHeader));
	if(res_head == -1)
		return -1;	
	
	//checks we are receiving a packet of the wanted type
	if(head.type != pkt->type)//
		return -1;//TODO crashes but the rest of the eventual packet stays on the fd !
	
	memcpy(pkt, &head, sizeof(PacketHeader));
	return res_head + serial_receive(fd, ((uint8_t*)pkt)+sizeof(PacketHeader), pkt->size - sizeof(PacketHeader));
}
