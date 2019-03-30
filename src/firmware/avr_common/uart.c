#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdbool.h>
#include <util/atomic.h>

#include "uart.h"

static UART uart0;

static void UART_buffers_init(void) {
	//initialize rx_buffer
	uart0.rx_start = 0;
	uart0.rx_end = 0;
	uart0.rx_size = 0;
	
	//initialize tx_buffer
	uart0.tx_start = 0;
	uart0.tx_end = 0;
	uart0.tx_size = 0;
}

void UART_Init(void){
	//initializes the global buffers
	UART_buffers_init();

	uart0.baudrate = UART_BAUD_RATE;

	// Clear the USART status register
	//UCSR0A = 0x00;
	
	// Set baud rate
	UBRR0H = (uint8_t)((UBRR)>>8);
	UBRR0L = (uint8_t)UBRR;
	
	// Enable receiver and transmitter
		// RXCIE0 : RX Complete Interrupt Enable. Set to allow receive complete interrupts.
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	// Set frame format: 8data, 2stop bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	sei();
}

//NOTE
//ATOMIC_BLOCK creates a block of code
	//guaranteed to be executed atomically.
	//Upon entering the block the Global Interrupt Status flag in SREG
	//is disabled, and re-enabled upon exiting the block from any exit path.
//ATOMIC_RESTORESTATE
	//When used, it will cause the ATOMIC_BLOCK
		//to restore the previous state of the SREG register,
		//saved before the Global Interrupt Status flag bit was disabled.	 
//ATOMIC_FORCEON (Grisetti's choice)
	//When used, it will cause the ATOMIC_BLOCK
		//to force the state of the SREG register on exit,
		//enabling the Global Interrupt Status flag bit.

void UART_TxByte(uint8_t data) {
  // loops until there is some space in the buffer
  while (uart0.tx_size >= UART_BUFFER_SIZE);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    uart0.tx_buffer[uart0.tx_end] = data;
    uart0.tx_end++;
    
    //se alla fine del buffer circolare, lo riporto all'inizio
		if (uart0.tx_end >= UART_BUFFER_SIZE)
			uart0.tx_end = 0;
		
		uart0.tx_size++;
  }
  
	UCSR0B |= (1<<UDRIE0); //activate buffer empty interrupt
}

uint8_t UART_RxByte(void) {
  // loops until there is some data in the buffer
  while(uart0.rx_size == 0);
	
  uint8_t data;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    data = uart0.rx_buffer[uart0.rx_start];
		uart0.rx_start++;
		
    //se alla fine del buffer circolare, lo riporto all'inizio
		if (uart0.rx_start >= UART_BUFFER_SIZE)
			uart0.rx_start = 0;
		
		uart0.rx_size--;
  }
  
  return data;
}

//Receive Interrupt Handler
ISR(USART0_RX_vect) {
	//legge il nuovo byte da UDR0,
		// e lo scrive in rx_buffer
	
  if (uart0.rx_size < UART_BUFFER_SIZE) {
		uart0.rx_buffer[uart0.rx_end] = UDR0;
  	uart0.rx_end++;
  
		//se alla fine del buffer circolare, lo riporto all'inizio
		if (uart0.rx_end >= UART_BUFFER_SIZE)
			uart0.rx_end = 0;
	
		uart0.rx_size++;
	}
}

//note in transmission data is put on UDR by user
	//automatically moved to the Transmission Buffer
	//when moved to the Transmission Buffer, UDRE is set
		//because UDR ready to get next data
	//when Transmission completed on the bus, TXC set

//Transmit Complete (TXCn) Flag
	//is set when the entire frame
	//in the Transmit Shift Register has been shifted out
	//and there are no new data
	//currently present in the transmit buffer
/*ISR(USART0_TX_vect) {*/
/*	;*/
/*}*/

//Data Register Empty (UDRE0) Flag
	//indicates whether the transmit buffer
	//is ready to receive new data
ISR(USART0_UDRE_vect) {	
	//se buffer vuoto
		// disactivate buffer empty interrupt
	if( uart0.tx_size == 0 )
		UCSR0B &= ~(1<<UDRIE0);
	else {
		//prende un byte da tx_buff
			//e lo scrive su UDR0
	  UDR0 = uart0.tx_buffer[uart0.tx_start];
		uart0.tx_start++;
	
	  //se alla fine del buffer circolare, lo riporto all'inizio
		if (uart0.tx_start >= UART_BUFFER_SIZE)
			uart0.tx_start = 0;
	
		uart0.tx_size--;
  }
}
