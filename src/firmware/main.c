#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_common/uart.h"
#include "avr_common/uart_packets.h"
#include "avr_common/i2c.h"
#include "avr_common/i2c_communication.h"

#include "../common/packets.h"

#define MAX_BUF 256
int main(void){
  UART_Init();
  
  EchoPacket pkt;
	
  while(1) {
		memset(&pkt, 0, sizeof(EchoPacket));
    UART_receive_packet((PacketHeader*)&pkt);
    
    UART_send_packet((PacketHeader*)&pkt);
  }
}
