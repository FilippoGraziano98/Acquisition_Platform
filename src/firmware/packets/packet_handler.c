#include "packet_handler.h"

#include "../../common/packet_header.h"
#include "../../common/packets.h"
#include "uart_packets.h"

#include "../imu/imu.h"

static PacketOpFunctionType packetOps_vector[PACKET_MAX_ID];

static uint8_t EchoPacketHandler(PacketHeader* pkt) {
	//all we must do with an echo packet is sending it back to host
	uint8_t size = UART_send_packet(pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

static uint8_t GyroscopePacketHandler(PacketHeader* pkt) {
	//we must fill in the gyroscope data
		//TODO should't we also set the seq/epoque
	if(pkt->type != GYROSCOPE_PACKET_ID)
		return PACKET_OPS_VECTOR_CORRUPTED;
	
	GyroscopePacket* gyro_pkt = (GyroscopePacket*)pkt;
	
	GyroscopeData gyro = IMU_GyroscopeData();
	
	gyro_pkt->gyro_x = gyro.gyro_x;
	gyro_pkt->gyro_y = gyro.gyro_y;
	gyro_pkt->gyro_z = gyro.gyro_z;
	
	uint8_t size = UART_send_packet((PacketHeader*)gyro_pkt);
	
	if( size == pkt->size)
		return PACKET_OP_SUCCESS;
	else
		return PACKET_SEND_INCOMPLETE;
}

void PacketHandler_init(void) {
	packetOps_vector[ECHO_PACKET_ID] = EchoPacketHandler;
	
	packetOps_vector[GYROSCOPE_PACKET_ID] = GyroscopePacketHandler;
}

uint8_t PacketHandler_process(PacketHeader* pkt) {
	PacketType pkt_id =	pkt->type;
	if( pkt_id < 0 || pkt_id > PACKET_MAX_ID )
		return PACKET_ID_OUT_OF_RANGE;

	PacketOpFunctionType pkt_op = packetOps_vector[pkt_id];
	if( !pkt_op )
		return PACKET_OP_NOT_IMPLEMENTED;
	
	return (*pkt_op)(pkt);
}
