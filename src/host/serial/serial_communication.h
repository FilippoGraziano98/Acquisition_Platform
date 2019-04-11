#pragma	once

#include <pthread.h>
#include <semaphore.h>

#include "../common/packets.h"

#define SERIAL_BUFFER_SIZE 16*PACKET_MAX_SIZE

//errors
//tx
#define SERIAL__NOT_ENOUGH_SPACE_IN_BUFFER -1
//rx
#define SERIAL__REQUESTED_PACKET_ID_NOT_FOUND -2
#define SERIAL__PACKET_INCOMPLETE -3

#define SERIAL__SUCCESS 0

typedef struct Host_Serial {
	int serial_fd;

  uint8_t tx_buffer[SERIAL_BUFFER_SIZE];
  volatile uint16_t tx_start;	//pos where to read in buffer
  volatile uint16_t tx_end;		//pos where to write in buffer
  volatile uint16_t tx_size;
  sem_t tx_sem_control;	//atomicity when updating tx_size
  sem_t tx_sem_data;		//1 if bytes in tx_buffer

  uint8_t rx_buffer[SERIAL_BUFFER_SIZE];
  volatile uint16_t rx_start;
  volatile uint16_t rx_end;
  volatile uint16_t rx_size;
  sem_t rx_sem_control;	//atomicity when updating rx_size
  sem_t rx_sem_data;		//1 if bytes in rx_buffer
  
  pthread_t tx_thread;
  pthread_t rx_thread;
} Host_Serial;

/*
 * Host_Serial_init:
 * 	initializes buffers, semaphores
 * 	creates the threads which will carry out serial communication
 */
void Host_Serial_init(int serial_fd);

/*
 * Host_Serial_sendPacket :
 *		writes a packet on the tx_buffer,
 *			then tx_thread will send it
 *	@returns:
 *		 0 if success,
 *		<0 else
 */
int Host_Serial_sendPacket(PacketHeader* pkt);

/*
 * Host_Serial_receivePacket :
 *		reads a packet from the rx_buffer,
 *			where rx_thread wrote it
 *	@returns:
 *		 0 if success,
 *		<0 else
 */
int Host_Serial_receivePacket(PacketHeader* pkt);

/*
 * Host_Serial_init:
 * 	destroys semaphores
 * 	kills the other threads
 */
void Host_Serial_destroy(void);
