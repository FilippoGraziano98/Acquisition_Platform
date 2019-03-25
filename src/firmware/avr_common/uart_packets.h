#pragma once

#include "../common/packets.h"
/*
 * serial_send_packet :
 *		sends a packet on fd
 */
uint8_t UART_send_packet(PacketHeader* pkt);

/*
 * serial_receive_packet :
 *		receives a packet from fd, and stores it in an already initialized packet pkt (with id and size)
 *		if received packet is not of the correct id, an error will be generated
 */
uint8_t UART_receive_packet(PacketHeader* pkt);
