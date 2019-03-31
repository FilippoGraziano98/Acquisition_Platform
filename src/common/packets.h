#pragma once
#include "packet_header.h"

//#define NUM_JOINTS 2
#define ORAZIO_PROTOCOL_VERSION 0x20190324

//TODO set seq to a global epoque id
  // simple macro to initialize a packet
#define INIT_PACKET(var, id)			\
  {																\
    var.header.type=id;						\
    var.header.size=sizeof(var);	\
    var.header.seq=0;							\
  }																\

#define PACKET_MAX_ID 16	//TODO check

#define MIN_SIZE(t1,t2) ((sizeof(t1)<sizeof(t2)) ? sizeof(t1) : sizeof(t2))
#define MAX_SIZE(t1,t2) ((sizeof(t1)>sizeof(t2)) ? sizeof(t1) : sizeof(t2))

#define PACKET_MIN_SIZE (MIN_SIZE(EchoPacket, GyroscopePacket))
#define PACKET_MAX_SIZE (MAX_SIZE(EchoPacket, GyroscopePacket))

#pragma pack(push, 1)

//! sent from the pc to the robot causes
//! the robot to send it again to the pc
typedef struct {
  PacketHeader header;
	uint8_t info; //random info
} EchoPacket;
#define ECHO_PACKET_ID 0

//NOTE: according to
	// https://learn.sparkfun.com/tutorials/data-types-in-arduino/all
	// float is 32bit-long on Arduino (as weel as on C)

//! sent from the pc to the robot causes with just the header filled
//! the robot sends it again to the pc filled with the correct info
typedef struct {
  PacketHeader header;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} GyroscopePacket;
#define GYROSCOPE_PACKET_ID 1

#pragma pack(pop)
