#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_common/uart.h"
#include "avr_common/i2c.h"
#include "imu/imu_i2c.h"
#include "acquisition_platform.h"
#include "packets/uart_packets.h"
#include "packets/packet_handler.h"

#include "../common/packets.h"

int main(void){
  UART_Init();
  I2C_Init();
  
  IMU_Init();
  
  AcquisitionPlatform* acq_pl = AcquisitionPlatform_init();
  
  PacketHandler_init(acq_pl);
  
	uint8_t buffer[PACKET_MAX_SIZE];
	PacketHeader* pkt = (PacketHeader*)buffer;
	
  while(1) {
		memset(pkt, 0, PACKET_MAX_SIZE);
		
		//aggiorna dati
    AcquisitionPlatform_imuUpdate(acq_pl);
    
        
    //controlla se sono arrivati pacchetti
    if( UART_check_packet() ) {
    	UART_receive_packet(pkt);
    	PacketHandler_process(pkt);
    } else {
    	_delay_ms(10);
    }
  }
}
