#include "uart_packets.h"
#include "uart.h"

uint8_t UART_send_packet(PacketHeader* pkt){
	uint8_t* pkt_aux = (uint8_t*)pkt;
	
	int i;
	for(i=0; i<pkt->size; i++) {
		UART_TxByte(*pkt_aux);
		pkt_aux++;
	}
	return pkt_aux - (uint8_t*)pkt;
}

uint8_t UART_receive_packet(PacketHeader* pkt) {
	uint8_t* pkt_aux = (uint8_t*)pkt;

	int i;
	for(i=0; i<sizeof(PacketHeader); i++) {
		*pkt_aux = UART_RxByte();
		pkt_aux++;		
	}
	
	//TODO how can i be sure pkt is buffer of eough space
	for(i=0; i < pkt->size - sizeof(PacketHeader); i++) {
		*pkt_aux = UART_RxByte();
		pkt_aux++;		
	}
	
	return pkt_aux - (uint8_t*)pkt;
}
