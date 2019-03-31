#pragma once
#include "../../common/packets.h"

typedef uint8_t(*PacketOpFunctionType)(PacketHeader* pkt);

//error conditions
#define PACKET_ID_OUT_OF_RANGE -1
#define PACKET_OP_NOT_IMPLEMENTED -2
#define PACKET_OPS_VECTOR_CORRUPTED -3
#define PACKET_SEND_INCOMPLETE -4

//success
#define PACKET_OP_SUCCESS 0

/*
 * PacketHandler_Init :
 * 	populates the static vector of PacketOpFunctionType [packetOps_vector]
 */
void PacketHandler_init(void);

/* 
 * PacketHandler :
 * 	generic packet handler, which will call the specific one for this packet
 */
uint8_t PacketHandler_process(PacketHeader* pkt);

