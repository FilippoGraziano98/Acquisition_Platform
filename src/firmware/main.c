#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_common/uart.h"
#include "avr_common/i2c.h"
#include "encoder/encoder.h"
#include "imu/imu.h"
#include "packets/uart_packets.h"
#include "packets/packet_handler.h"

#include "../common/packets.h"

#ifdef DEBUG_PRINTF
#include <stdio.h>
#endif

int main(void){
	UART_Init();
	I2C_Init();
	
	Encoder_Init();
	IMU_Init();
 
	PacketHandler_init();
	
	IMU_GyroscopeCalibration();
	
	uint8_t buffer[PACKET_MAX_SIZE];
	PacketHeader* pkt = (PacketHeader*)buffer;
	
	while(1) {
		memset(pkt, 0, PACKET_MAX_SIZE);
		
		//controlla se sono arrivati pacchetti
		if( UART_check_packet() ) {
			UART_receive_packet(pkt);
			PacketHandler_process(pkt);
		} else {
			_delay_ms(10);
		}
	}
}
