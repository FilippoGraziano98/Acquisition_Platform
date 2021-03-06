#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "serial_communication.h"
#include "serial.h"

static Host_Serial_t Host_serial;

#define handle_semaphore_error(err, msg) if( err ) { perror(msg); } 
#define handle_pthread_error(err, msg) if( err ) { errno = err; perror(msg); }

static void _txThreadFn_sendBytes(uint8_t* buf, uint16_t size) {
	int err, sent_bytes;
	
	//send bytes
	sent_bytes = serial_send(Host_serial.serial_fd, buf, size);
	if( sent_bytes < 0)
		printf("[Host_Serial: _txThreadFn_sendBytes] Error in serial_send\n");
	else {
		Host_serial.tx_start += sent_bytes;
		if (Host_serial.tx_start >= SERIAL_BUFFER_SIZE)
			Host_serial.tx_start = 0;
	}
	
	//update tx_size
	err = sem_wait(&(Host_serial.tx_sem_control));
	handle_semaphore_error(err, "[Host_Serial: _txThreadFn_sendBytes] Error in sem_wait on tx_sem_control");

	Host_serial.tx_size -= sent_bytes;

	err = sem_post(&(Host_serial.tx_sem_control));
	handle_semaphore_error(err, "[Host_Serial: _txThreadFn_sendBytes] Error in sem_post on tx_sem_control");
}

static void* _txThreadFn(void* args_) {
	int err;
	while(1) {
		// sem_wait on tx_sem_data : waits for data to be transmitted
		err = sem_wait(&(Host_serial.tx_sem_data));
		handle_semaphore_error(err, "[Host_Serial: _txThreadFn] Error in sem_wait on tx_sem_data");
		
		if( Host_serial.tx_size == 0 )
			continue;
		
		if ( Host_serial.tx_start + Host_serial.tx_size >= SERIAL_BUFFER_SIZE ) {
			// send from tx_start to SERIAL_BUFFER_SIZE
			_txThreadFn_sendBytes((uint8_t*)&(Host_serial.tx_buffer[Host_serial.tx_start]), SERIAL_BUFFER_SIZE - Host_serial.tx_start);
		}
		
		// send from tx_start to tx_size
		_txThreadFn_sendBytes((uint8_t*)&(Host_serial.tx_buffer[Host_serial.tx_start]), Host_serial.tx_size);
	}
	return args_;
}

static void* _rxThreadFn(void* args_) {
	int err, bytes_to_read, recv_bytes;

	uint8_t _curr_pkt[PACKET_MAX_SIZE] = {0};
	PacketHeader* curr_pkt = (PacketHeader*)_curr_pkt;
	uint8_t curr_pkt_size = 0;
	
	while(1) {
		//tries to read up to complete curr_pkt 
		if( curr_pkt_size == 0 )
			bytes_to_read = PACKET_MIN_SIZE;
		else if ( curr_pkt_size < sizeof(PacketHeader))
			bytes_to_read = PACKET_MIN_SIZE - curr_pkt_size;
		else
			bytes_to_read = curr_pkt->size - curr_pkt_size;
		
		recv_bytes = serial_receive(Host_serial.serial_fd, (uint8_t*)&(_curr_pkt[curr_pkt_size]), bytes_to_read);
		if( recv_bytes < 0 )
			printf("[Host_Serial: _rxThreadFn] Error in serial_receive");
		
		curr_pkt_size += recv_bytes;
		
		
		if( curr_pkt_size > sizeof(PacketHeader) && curr_pkt_size == curr_pkt->size ) {
			PacketType pkt_id =	curr_pkt->type;
			if( pkt_id < 0 || pkt_id > PACKET_MAX_ID ) {
				printf("[Host_Serial: _rxThreadFn] ERROR: pkt_id (: %d) not in [0, PACKET_MAX_ID], discarding packet\n", pkt_id);
				goto SKIP;
			}
			
			
			PacketOpFunctionType pkt_op = Host_serial.packetOps_vector[pkt_id];
			if( pkt_op )
				(*pkt_op)(curr_pkt); //TODO ret value ignored??
			else {
				//move to rx_buffer (slow)
				
				//checks if there is enough space in rx_buffer
				if( (uint16_t)curr_pkt_size > SERIAL_BUFFER_SIZE - Host_serial.rx_size ) {
					printf("[Host_Serial: _rxThreadFn] ERROR: rx_buffer full (pkt_ID: %d), discarding packet\n", curr_pkt->type);
					goto SKIP;
				}
				// writes from tx_end to SERIAL_BUFFER_SIZE
				int wrote_bytes = 0;
				if ( Host_serial.rx_end + curr_pkt_size >= SERIAL_BUFFER_SIZE ) {
					wrote_bytes = SERIAL_BUFFER_SIZE - Host_serial.rx_end;
					memcpy(&(Host_serial.rx_buffer[Host_serial.rx_end]), _curr_pkt, wrote_bytes);
					Host_serial.rx_end = 0;
				}
				//writes remaining bytes
				memcpy(&(Host_serial.rx_buffer[Host_serial.rx_end]), ((uint8_t*)_curr_pkt)+wrote_bytes, curr_pkt_size - wrote_bytes);
				Host_serial.rx_end += (curr_pkt_size - wrote_bytes);
				
				//updates rx_size
				err = sem_wait(&(Host_serial.rx_sem_control));
				handle_semaphore_error(err, "[Host_Serial: _rxThreadFn] Error in sem_wait on rx_sem_control");

				Host_serial.rx_size += curr_pkt_size;

				err = sem_post(&(Host_serial.rx_sem_control));
				handle_semaphore_error(err, "[Host_Serial: _rxThreadFn] Error in sem_post on rx_sem_control");

				//wakes up main thread (if it was waiting data)
				err = sem_post(&(Host_serial.rx_sem_data));
				handle_semaphore_error(err, "[Host_Serial: _rxThreadFn] Error in sem_post on rx_sem_data");
			}
			SKIP:
				//discards packet
				memset(_curr_pkt, 0, PACKET_MAX_SIZE);
				curr_pkt_size = 0;
		}	else if( curr_pkt_size > sizeof(PacketHeader) && curr_pkt_size > curr_pkt->size ) {
			printf("[Host_Serial: _rxThreadFn] Error curr_pkt_size %d > curr_pkt->size %d\n", curr_pkt_size, curr_pkt->size);
			
		}
	}
	return args_;
}

void Host_Serial_init(int serial_fd) {
	Host_serial.serial_fd = serial_fd;

	//initialize rx_buffer
	Host_serial.rx_start = 0;
	Host_serial.rx_end = 0;
	Host_serial.rx_size = 0;
	
	//initialize tx_buffer
	Host_serial.tx_start = 0;
	Host_serial.tx_end = 0;
	Host_serial.tx_size = 0;

	int err;

	//initializes semaphores
	err = sem_init(&(Host_serial.tx_sem_control), 0, 1);
	handle_semaphore_error(err, "[Host_Serial_init] Error in sem_init for tx_sem_control");
	
	err = sem_init(&(Host_serial.tx_sem_data), 0, 0);
	handle_semaphore_error(err, "[Host_Serial_init] Error in sem_init for tx_sem_data");
	
	err = sem_init(&(Host_serial.rx_sem_control), 0, 1);
	handle_semaphore_error(err, "[Host_Serial_init] Error in sem_init for rx_sem_control");
	
	err = sem_init(&(Host_serial.rx_sem_data), 0, 0);
	handle_semaphore_error(err, "[Host_Serial_init] Error in sem_init for rx_sem_data");

	//creates buffer threads
	pthread_attr_t attr;
	err = pthread_attr_init(&attr);
	handle_pthread_error(err, "[Host_Serial_init] Error in pthread_attr_init");

	pthread_create(&(Host_serial.tx_thread), &attr, _txThreadFn, NULL);
	handle_pthread_error(err, "[Host_Serial_init] Error in pthread_create of tx_thread");
	pthread_create(&(Host_serial.rx_thread), &attr, _rxThreadFn, NULL);
	handle_pthread_error(err, "[Host_Serial_init] Error in pthread_create of rx_thread");
}

int Host_Serial_sendPacket(PacketHeader* pkt) {
	//checks if there is enough space in tx_buffer
	if( (uint16_t)(pkt->size) > SERIAL_BUFFER_SIZE - Host_serial.tx_size ) {
		printf("[Host_Serial_sendPacket] Not Enough Space in tx_buffer (pkt_size: %d, buffer_space: %d)\n", pkt->size, (int)(SERIAL_BUFFER_SIZE - Host_serial.tx_size));
		return SERIAL__NOT_ENOUGH_SPACE_IN_BUFFER;
	}
	
	// writes from tx_end to SERIAL_BUFFER_SIZE
	int wrote_bytes = 0;
	if ( Host_serial.tx_end + pkt->size >= SERIAL_BUFFER_SIZE ) {
		wrote_bytes = SERIAL_BUFFER_SIZE - Host_serial.tx_end;
		memcpy(&(Host_serial.tx_buffer[Host_serial.tx_end]), (uint8_t*)pkt, wrote_bytes);
		Host_serial.tx_end = 0;
	}
	//writes remaining bytes
	memcpy(&(Host_serial.tx_buffer[Host_serial.tx_end]), ((uint8_t*)pkt)+wrote_bytes, pkt->size - wrote_bytes);
	Host_serial.tx_end += (pkt->size - wrote_bytes);
	
	int err;
	
	//updates tx_size
	err = sem_wait(&(Host_serial.tx_sem_control));
	handle_semaphore_error(err, "[Host_Serial_sendPacket] Error in sem_wait on tx_sem_control");
	
	Host_serial.tx_size += pkt->size;
	
	err = sem_post(&(Host_serial.tx_sem_control));
	handle_semaphore_error(err, "[Host_Serial_sendPacket] Error in sem_post on tx_sem_control");
	
	//wakes up tx_thread
	err = sem_post(&(Host_serial.tx_sem_data));
	handle_semaphore_error(err, "[Host_Serial_sendPacket] Error in sem_post on tx_sem_data");
	
	return SERIAL__SUCCESS;
}


int Host_Serial_receivePacket(PacketHeader* pkt) {
	int err;
	do {
		// sem_wait on rx_sem_data : waits for data to be received
		err = sem_wait(&(Host_serial.rx_sem_data));
		handle_semaphore_error(err, "[Host_Serial_receivePacket] Error in sem_wait on rx_sem_data");
	} while( Host_serial.rx_size < PACKET_MIN_SIZE );
	//} while( Host_serial.rx_size < pkt->size );
	
	// reads PacketHeader
	PacketHeader pkt_head;

	//reads PacketHeader without updating rx_start (and so leeving it in the buffer)
		//just to check if requested packet
	int pkt_head_bytes_read = 0, pkt_head_idx_to_read=Host_serial.rx_start;
	if ( Host_serial.rx_start + sizeof(PacketHeader) >= SERIAL_BUFFER_SIZE ) {
		pkt_head_bytes_read = SERIAL_BUFFER_SIZE - Host_serial.rx_start;
		memcpy((void*)&pkt_head, &(Host_serial.rx_buffer[pkt_head_idx_to_read]), pkt_head_bytes_read);
		pkt_head_idx_to_read = 0;	
	}
	memcpy(((uint8_t*)&pkt_head)+pkt_head_bytes_read, &(Host_serial.rx_buffer[pkt_head_idx_to_read]), sizeof(PacketHeader) - pkt_head_bytes_read);
	
	//checks we are receiving a packet of the wanted type
	if( pkt_head.type != pkt->type )
		return SERIAL__REQUESTED_PACKET_ID_NOT_FOUND;
	
	//checks for integrity of the packet
	if( pkt_head.size > Host_serial.rx_size )
		return SERIAL__PACKET_INCOMPLETE;
	
	int recvd_bytes = 0;
	if ( Host_serial.rx_start + pkt_head.size >= SERIAL_BUFFER_SIZE ) {
		recvd_bytes = SERIAL_BUFFER_SIZE - Host_serial.rx_start;
		memcpy((void*)pkt, &(Host_serial.rx_buffer[Host_serial.rx_start]), recvd_bytes);
		
		Host_serial.rx_start = 0;
	}
	memcpy(((void*)pkt)+recvd_bytes, &(Host_serial.rx_buffer[Host_serial.rx_start]), pkt_head.size - recvd_bytes);
	
	Host_serial.rx_start += (pkt_head.size - recvd_bytes);
	
	//update rx_size
	err = sem_wait(&(Host_serial.rx_sem_control));
	handle_semaphore_error(err, "[Host_Serial_receivePacket] Error in sem_wait on rx_sem_control");

	Host_serial.rx_size -= pkt_head.size;

	err = sem_post(&(Host_serial.rx_sem_control));
	handle_semaphore_error(err, "[Host_Serial_receivePacket] Error in sem_post on rx_sem_control");
	
	
	return SERIAL__SUCCESS;
}

int Host_Serial_registerPacketHandler(PacketType packet_type, PacketOpFunctionType callback) {
	if( packet_type < 0 || packet_type > PACKET_MAX_ID)
		return SERIAL__PACKET_ID_UNEXISTING;
	Host_serial.packetOps_vector[packet_type] = callback;
	return SERIAL__SUCCESS;
}

void Host_Serial_Print_Debug_Info(){
	printf("[_rxThreadFn] rx_buffer %d\n", Host_serial.rx_size);
	int e;
	for(e=Host_serial.rx_start; e<Host_serial.rx_start+Host_serial.rx_size; e++){
		printf("%d) %d 0x%02x\n", e, Host_serial.rx_buffer[e], Host_serial.rx_buffer[e]);
	}
}

void Host_Serial_destroy(void) {
	//TODO
	//pthread_cancel
	//sem_destroy
}
