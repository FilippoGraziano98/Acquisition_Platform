#include <stdint.h>

#include "../common/packets.h"

typedef struct Host {
	// file descriptor of the serial port
	int serial_fd;
	//uint8_t packet_buffer[PACKET_SIZE_MAX];
	
	//seq number of the latest packets received
	uint16_t global_seq;
	
	//these are the system variables, updated by the serial communiction
	EchoPacket sync_packet;
} Host;

/*
 * Host_init :
 * 	creates a new host, opening a serial connection on a device
 */
Host* Host_init(const char* device);

/*
 * Host_checkConnection :
 * 	sends a few (cycles) EchoPacket testing serial connection
 */
int Host_checkConnection(Host* host, int cycles);

/*
 * Host_destroy :
 * 		destroyes a previouslt created host
 *	@returns:
 *			0 on success,
 *			on error, -1 is returned, and errno is set by close
 */
int Host_destroy(Host* host);
