#pragma once

#include "../common/packets.h"

/*
 * UART_check_packet :
 *		check if there is a packet in mailbox
 *	@return :
 *		1 if yes
 *		0 if no
 */
uint8_t UART_check_packet(void);

/*
 * UART_send_packet :
 *		sends a packet on fd
 */
uint8_t UART_send_packet(PacketHeader* pkt);

/*
 * UART_receive_packet :
 *		receives a packet from fd, and stores it in an already initialized packet pkt (with id and size)
 *		if received packet is not of the correct id, an error will be generated
 */
uint8_t UART_receive_packet(PacketHeader* pkt);
