#pragma once
#include <stdint.h>


// here are the limits
typedef uint8_t PacketType;
typedef uint8_t PacketSize;
typedef uint16_t PacketSeq;      


//#pragma pack instructs the compiler to pack structure members with particular alignment.
	//#pragma pack(push, 1) will tell the compiler not to padd the struct
#pragma pack(push, 1)
typedef struct {
  PacketType type;  // type of the packet < PACKET_MAX_ID
  PacketSize size;  // size of the packet in bytes (PacketHeader included)
  PacketSeq  seq;   // sequential number always increased
} PacketHeader;
#pragma pack(pop)
