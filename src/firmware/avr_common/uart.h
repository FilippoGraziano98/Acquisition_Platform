#pragma once

// UART : Universal Asynchronous Receiver/Transmitter

// Baud Rate at which we want the serial to communicate
#define UART_BAUD_RATE 38400

// Baud Rate = F_CPU / (16*(UBRRn+1))
	// 16 is beacuse we are in Asynchronous Normal Mode
#define UBRR ( (F_CPU/16)/UART_BAUD_RATE - 1 )
// NOTE : F_CPU defined at compile time:
	// -DF_CPU=16000000UL

/*
 * initialize the UART at UART_BAUD_RATE baud rate
 */
void UART_Init(void);

/*
 * transmit a char through UART module.
 *		It waits till previous char is transmitted (i.e. until UDRE is set),
 *		then the new byte to be transmitted is loaded into UDR.
 */
void UART_TxByte(uint8_t data);

/*
 * receive a char from UART module.
 *		It waits till a char is received (i.e.till RXC is set),
 *		then returns the received char.
 */
uint8_t UART_RxByte(void);

/*
 * reads the UDRn I/O location until the RXCn Flag is cleared
 *	i.e. flushes the receive buffer
 *		returns the number of bytes thrown away
 */
uint8_t UART_RxFlush(void);

/*
 *  transmit an ASCII string through UART.
 *		each a char is sent using UART_TxChar()
 *		returns the size of the string transmitted
 *	NOTE: sends a NULL terminated string (ending 0x00 included)
 */
uint8_t UART_TxString(uint8_t* buf);

/*
 *  receive an ASCII string through UART till the carriage_return / new_line / 0.
 *		returns the size of the string received
 *	NOTE: receives a NULL terminated string (ending 0x00 included)
 */
uint8_t UART_RxString(uint8_t* buf);
