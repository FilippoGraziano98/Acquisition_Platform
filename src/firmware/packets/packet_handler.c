#include "packet_handler.h"

#include "../../common/packet_header.h"
#include "../../common/packets.h"
#include "uart_packets.h"

static PacketOpFunctionType packetOps_vector[PACKET_MAX_ID];

static void EchoPacketHandler(PacketHeader* pkt) {
	//all we must do with an echo packet is sending it back to host
	UART_send_packet(pkt);
}

void PacketHandler_init(void) {
	packetOps_vector[ECHO_PACKET_ID] = EchoPacketHandler;
}

uint8_t PacketHandler_process(PacketHeader* pkt) {
	PacketType pkt_id =	pkt->type;
	if( pkt_id < 0 || pkt_id > PACKET_MAX_ID )
		return PACKET_ID_OUT_OF_RANGE;

	PacketOpFunctionType pkt_op = packetOps_vector[pkt_id];
	if( !pkt_op )
		return PACKET_OP_NOT_IMPLEMENTED;
	
	(*pkt_op)(pkt);
	
	return PACKET_OP_SUCCESS;
}
