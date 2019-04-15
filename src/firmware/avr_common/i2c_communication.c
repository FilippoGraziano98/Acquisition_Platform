#include <avr/io.h>
#include <util/delay.h>

#include <stdlib.h>
#include <string.h>

#include "i2c.h"
#include "i2c_communication.h"

#define READ 1
#define WRITE 0

void I2C_WriteRegister(uint8_t device_address, uint8_t device_reg, uint8_t data) {
	uint8_t buf[2] = {device_reg, data};
	I2C_Enqueue_Operation(device_address, WRITE, 2, buf, NULL);
}

void I2C_WriteNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, uint8_t* data) {
	uint8_t buf[n+1];
	buf[0] = device_reg_start;
	memcpy(buf+1, data, n);
	I2C_Enqueue_Operation(device_address, WRITE, n+1, buf, NULL);	
}

void I2C_ReadRegister(uint8_t device_address, uint8_t device_reg, PostProcessFunction_t post_process_fn) {
	I2C_Enqueue_Operation(device_address, WRITE, 1, &device_reg, NULL);
	I2C_Enqueue_Operation(device_address, READ, 1, NULL, post_process_fn);
}

void I2C_ReadNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, PostProcessFunction_t post_process_fn) {
	I2C_Enqueue_Operation(device_address, WRITE, 1, &device_reg_start, NULL);
	I2C_Enqueue_Operation(device_address, READ, n, NULL, post_process_fn);
}
