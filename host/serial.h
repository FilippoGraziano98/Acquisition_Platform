#pragma once

// returns the descriptor of a serial port
int serial_open(const char* name);

// sets the attributes
int serial_set_interface_attribs(int fd, int speed, int parity);
