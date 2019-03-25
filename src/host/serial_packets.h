#pragma once

#include "../common/packets.h"
/*
 * serial_send_packet :
 *		sends a packet on fd
 */
int serial_send_packet(int fd, PacketHeader* pkt);

/*
 * serial_receive_packet :
 *		receives a packet from fd, and stores it in an already initialized packet pkt (with id and size)
 *		if received packet is not of the correct id, an error will be generated
 */
int serial_receive_packet(int fd, PacketHeader* pkt);
