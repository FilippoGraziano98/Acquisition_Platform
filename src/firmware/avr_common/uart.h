#pragma once

// UART : Universal Asynchronous Receiver/Transmitter

// Baud Rate at which we want the serial to communicate
#define UART_BAUD_RATE 57600

// Baud Rate = F_CPU / (16*(UBRRn+1))
	// 16 is beacuse we are in Asynchronous Normal Mode
#define UBRR ( (F_CPU/16)/UART_BAUD_RATE - 1 )
// NOTE : F_CPU defined at compile time:
	// -DF_CPU=16000000UL

#include <stdio.h>
#include "../../common/packets.h"
#define UART_BUFFER_SIZE 8*PACKET_MAX_SIZE

typedef struct UART {
  uint8_t tx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t tx_start;
  volatile uint8_t tx_end;
  volatile uint16_t tx_size;

  uint8_t rx_buffer[UART_BUFFER_SIZE];
  volatile uint8_t rx_start;
  volatile uint8_t rx_end;
  volatile uint16_t rx_size;
  
  uint16_t baudrate;
} UART;

/*
 * UART_Init :
 * 	initialize the UART at UART_BAUD_RATE baud rate
 */
void UART_Init(void);

/*
 * UART_TxByte :
 *  puts a byte in the transmit buffer
 *		as soon as possible it will be transmitted
 *			through the UART module
 */
void UART_TxByte(uint8_t data);

/*
 * UART_RxBufferSize :
 *		@return : number of received bytes.
 */
uint16_t UART_RxBufferSize(void);

/*
 * UART_RxByte :
 *  gets a byte from the receive buffer
 *		(buffer populated automatically from UART module)
 *		@return : the received byte.
 */
uint8_t UART_RxByte(void);
