#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "avr_common/uart.h"
#include "avr_common/i2c.h"
#include "avr_common/eeprom.h"
#include "encoder/encoder.h"
#include "encoder/encoder_odometry.h"
#include "imu/imu.h"
#include "imu/imu_odometry.h"
#include "packets/uart_packets.h"
#include "packets/packet_handler.h"

#include "../common/packets.h"

#ifdef DEBUG_PRINTF
#include <stdio.h>
#endif

#define FIRMWARE_COMMUNICATION_RATE 1 //Hz

// Global Status of the Firmware Controller
SystemStatusPacket system_status;


static void Firmware_checkConnection(int cycles) {
	EchoPacket echo_pkt;
	PacketHeader* pkt = (PacketHeader*)&echo_pkt;
	
	int i=0;
	while( i < cycles ) {
		//controlla se sono arrivati pacchetti
		if( UART_check_packet() ) {
			UART_receive_packet(pkt);
			PacketHandler_process(pkt);
			i++;
		} else {
			_delay_ms(10);
		}
	}
}

static void Firmware_manageCalibrationPacket(void) {
	IMUCalibrateRequest calib_req;
	PacketHeader* pkt = (PacketHeader*)&calib_req;
	
	uint8_t calib_req_recv_flag=0;
	while( !calib_req_recv_flag ) {
		//controlla se sono arrivati pacchetti
		if( UART_check_packet() ) {
			UART_receive_packet(pkt);
			PacketHandler_process(pkt);
			calib_req_recv_flag = 1;
		} else {
			_delay_ms(10);
		}
	}
}

ISR(TIMER5_COMPA_vect) {
	UART_send_packet((PacketHeader*)&(system_status));
	
	uint8_t tx_pkt = 0;
	
	#ifdef USE_ENCS
	tx_pkt += Encoder_sendOdometryToHost();
	#endif
	
	#ifdef USE_IMU
	tx_pkt += IMU_sendIMUDataToHost();
	tx_pkt += IMU_sendOdometryToHost();
	#endif

	system_status.tx_count += tx_pkt;

	system_status.idle_cycles = 0;
	system_status.global_secs_count++;
	//system_status.tx_count+=; //TODO
}


static void Firmware_setPeriodicDataCommunication(void) {
  uint16_t period_ms = 1000 / FIRMWARE_COMMUNICATION_RATE; //from a frequency in Hz, we get a period in millisecs
  
  // configure timer1, prescaler : 1024, CTC (Clear Timer on Compare match)
  TCCR5A = 0;
  TCCR5B = (1 << WGM12)| (1 << CS10) | (1 << CS12); 
  
  /*
	 * cpu frequency 16MHz = 16.000.000 Hz
	 * prescaler 256
	 *	-->> TCNT1 increased at a frequency of 16.000.000/1024 Hz = 15625 Hz
	 *	so 1 ms will correspond do 15.625 counts
	 */
  OCR5A = (uint16_t)(15.625 * period_ms);

	// timer-interrupt enabling will be executed atomically (no other interrupts)
		// and ATOMIC_FORCEON ensures Global Interrupt Status flag bit in SREG set afetrwards
		// (sei() not needed)
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
  	TIMSK5 |= (1 << OCIE5A);  // enable the timer interrupt (istruz. elementare, no interrupt)
  }
}


int main(void){
	memset(&system_status, 0, sizeof(SystemStatusPacket));
	INIT_PACKET(system_status, SYSTEM_STATUS_PACKET_ID);

	UART_Init();
	I2C_Init();
	EEPROM_init();

	#ifdef USE_ENCS
	Encoders_Init();
	Encoder_OdometryInit();
	#endif
	
	#ifdef USE_IMU
	IMU_Init();
	IMU_OdometryInit();
	#endif
	
	#ifndef DEBUG_PRINTF
	PacketHandler_init();
	Firmware_checkConnection(SYNCHRONIZATION_CYCLES);
	#endif
	
	#ifdef USE_IMU
	Firmware_manageCalibrationPacket();
	#endif
	
	#ifndef DEBUG_PRINTF
	Firmware_setPeriodicDataCommunication();
	#endif
	
	#ifdef DEBUG_PRINTF
	printf("start\n");
	#endif
	
	
	uint8_t buffer[PACKET_MAX_SIZE];
	PacketHeader* pkt = (PacketHeader*)buffer;
	memset(pkt, 0, PACKET_MAX_SIZE);
	
	while(1) {
		system_status.idle_cycles++;
		
		//controlla se sono arrivati pacchetti
		if( UART_check_packet() ) {
			UART_receive_packet(pkt);
			PacketHandler_process(pkt);
			memset(pkt, 0, PACKET_MAX_SIZE);
			system_status.rx_count++;
		}
	}
}
