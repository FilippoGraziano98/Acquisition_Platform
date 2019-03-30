#pragma once

// UART : Universal Asynchronous Receiver/Transmitter

// Baud Rate at which we want the serial to communicate
#define UART_BAUD_RATE 38400

// Baud Rate = F_CPU / (16*(UBRRn+1))
	// 16 is beacuse we are in Asynchronous Normal Mode
#define UBRR ( (F_CPU/16)/UART_BAUD_RATE - 1 )
// NOTE : F_CPU defined at compile time:
	// -DF_CPU=16000000UL

#include <stdio.h>
#include "../../common/packets.h"
#define UART_BUFFER_SIZE 8*MAX_PACKET_SIZE

typedef struct UART {
  int tx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t tx_start;
  volatile uint8_t tx_end;
  volatile uint8_t tx_size;

  int rx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t rx_start;
  volatile uint8_t rx_end;
  volatile uint8_t rx_size;
  
  int baudrate;
} UART;

/*
 * initialize the UART at UART_BAUD_RATE baud rate
 */
void UART_Init(void);

/*
 * transmit a char through UART module.
 */
void UART_TxByte(uint8_t data);

/*
 * receive a char from UART module.
 *		@return : the received char.
 */
uint8_t UART_RxByte(void);
