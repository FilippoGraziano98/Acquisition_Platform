#pragma once
#include "packet_header.h"

//#define NUM_JOINTS 2
#define ORAZIO_PROTOCOL_VERSION 0x20190324

  // simple macro to initialize a packet
#define INIT_PACKET(var, id)			\
  {																\
    var.header.type=id;						\
    var.header.size=sizeof(var);	\
    var.header.seq=0;							\
  }																\

#define PACKET_MAX_ID 16	//TODO check

#define PACKET_MAX_SIZE (sizeof(EchoPacket))
#define PACKET_MIN_SIZE (sizeof(EchoPacket))

#pragma pack(push, 1)

//! sent from the pc to the robot causes
//! the robot to send it again to the pc
typedef struct {
  PacketHeader header;
	uint8_t info; //random info
} EchoPacket;
#define ECHO_PACKET_ID 0


#pragma pack(pop)
