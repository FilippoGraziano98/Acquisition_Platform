#pragma once

// returns the descriptor of a serial port
int serial_open(const char* name);

// sets the attributes
int serial_set_interface_attribs(int fd, int speed, int parity);

/*
 * serial_sleep_until_input
 *		sleeps until data us available on the serial or the timeout has elapsed
 *		@returns: 
 *		  0 if timeout,
 *		  1 if input available,
 *		 -1 if error.
 */
int serial_sleep_until_input(int fd, unsigned int timeout_secs);

/*
 * serial_send
 * 		sends a packet of given size and reading it from a given buf
 *			note this is a blocking call !
 *		@returns: 
 * 			-1 if error,
 *  		number of bytes sent else.
 */
int serial_send(int fd, uint8_t* buf, uint8_t size);

/*
 * serial_receive
 *		receive a packet of given size and stores it in given buf
 * 			note this is a blocking call !
 *		@returns: 
 * 			-1 if error,
 *      number of bytes received else.
 */
int serial_receive(int fd, uint8_t* buf, uint8_t size);
