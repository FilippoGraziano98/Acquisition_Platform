#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>

#include "uart.h"

// baud rate regs
#define UBRRnH UBRR0H
#define UBRRnL UBRR0L

//data reg
#define UDRn UDR0

// control regs
#define UCSRnA UCSR0A
#define UCSRnB UCSR0B
#define UCSRnC UCSR0C

// UCSRnA flags
#define UDREn UDRE0
#define RXCn RXC0
// UCSRnB flags
#define RXENn RXEN0
#define TXENn TXEN0
#define RXCIEn RXCIE0
// UCSRnC flags
#define USBSn USBS0
#define UCSZn1 UCSZ01
#define UCSZn0 UCSZ00

void UART_Init(void){
	// Clear the USART status register
	//UCSRnA = 0x00;
	
	// Set baud rate
	UBRRnH = (uint8_t)((UBRR)>>8);
	UBRRnL = (uint8_t)UBRR;
	
	// Enable receiver and transmitter
		// RXCIE0 : RX Complete Interrupt Enable. Set to allow receive complete interrupts.
	UCSRnB = (1<<RXENn)|(1<<TXENn);//|(1<<RXCIEn);
	// Set frame format: 8data, 2stop bit
	UCSRnC = (1<<UCSZn1)|(1<<UCSZn0);
}

void UART_TxByte(uint8_t data) {
	/* Wait for empty transmit buffer */
	while ( !( UCSRnA & (1<<UDREn) ) );
	/* Put data into buffer, sends the data */
	UDRn = data;
}

uint8_t UART_RxByte(void) {
	/* Wait for data to be received */
	while ( !(UCSRnA & (1<<RXCn) ) );
	/* Get and return received data from buffer */
	return UDRn;
}

uint8_t UART_RxFlush(void) {
	uint8_t dummy, count=0;
	while ( UCSRnA & (1<<RXCn) ) {
		dummy = UDRn;
		count++;
	}
	return count;
}

uint8_t UART_TxString(uint8_t *buf) {
  uint8_t* buf_start = buf; //beginning of buffer
	while( *buf ) {
		UART_TxByte(*buf);
		buf++;
	}
	UART_TxByte(0x0);
	return buf - buf_start;
}

uint8_t UART_RxString(uint8_t* buf) {
  uint8_t* buf_start = buf; //beginning of buffer
  
	char c;
	while( 1 ) {
		c = UART_RxByte();

		if( (c==0) || (c=='\r') || (c=='\n') ) {
			//read till enter key is pressed
			*buf = 0;	//null terminate the string
			break;
		}
		
		*buf = c;	//copy the char into string
		buf++;		//and increment the pointer
	}
	//UART_RxFlush(); //TODO RxFlush seems not working!!
	return buf - buf_start;
}
