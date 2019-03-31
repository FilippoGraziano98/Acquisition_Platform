#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_common/uart.h"
#include "avr_common/i2c.h"
#include "avr_common/i2c_communication.h"
#include "packets/uart_packets.h"
#include "packets/packet_handler.h"

#include "../common/packets.h"

#define MAX_BUF 256
int main(void){
  UART_Init();
  PacketHandler_init();
  
  EchoPacket pkt;
	
  while(1) {
		memset(&pkt, 0, sizeof(EchoPacket));
    
    if( UART_check_packet() ) {
    	UART_receive_packet((PacketHeader*)&pkt);
    	PacketHandler_process((PacketHeader*)&pkt);
    } else {
    	_delay_ms(10);
    }
  }
}
