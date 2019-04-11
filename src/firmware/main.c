#include <util/delay.h>
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

#ifdef DEBUG_PRINTF
#include "avr_common/i2c_communication.h"
#include <stdio.h>
#endif

int main(void){
  UART_Init();
  I2C_Init();
  
  IMU_Init();
  
  #ifdef DEBUG_PRINTF
  printf("GYRO_CONFIG : 0x%02x\n", I2C_ReadRegister(ACCELGYRO_DEVICE, GYRO_CONFIG));
  printf("CONFIG : 0x%02x\n", I2C_ReadRegister(ACCELGYRO_DEVICE, CONFIG));
  #endif
  
  AcquisitionPlatform_init();
  PacketHandler_init();
  
  AcquisitionPlatform_imuCalibrate();
  AcquisitionPlatform_setPeriodicIMUupdate(10);
  
	uint8_t buffer[PACKET_MAX_SIZE];
	PacketHeader* pkt = (PacketHeader*)buffer;
	
  while(1) {
		memset(pkt, 0, PACKET_MAX_SIZE);
    
		//AcquisitionPlatform_imuUpdate();
    //_delay_ms(100);
    
		#ifdef DEBUG_PRINTF
		printf("Acq_pl : %d\n", AcquisitionPlatform_getGlobalSeq());
  	_delay_ms(1000);
    #else
    //controlla se sono arrivati pacchetti
    if( UART_check_packet() ) {
    	UART_receive_packet(pkt);
    	PacketHandler_process(pkt);
    } else {
    	;//_delay_ms(10);
    }
    #endif
  }
}
