#include <avr/io.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"
#include "i2c_communication.h"

#define READ 1
#define WRITE 0


static void I2C_WritePreamble(uint8_t device_address, uint8_t device_reg) {
	uint8_t err;
	
	err = I2C_Start();
	check_err(err, "[I2C_WritePreamble] Error transmitting the Start condition");
	
	err = I2C_SendAddress(device_address, WRITE);
	check_err(err, "[I2C_WritePreamble] Error transmitting the Device Address");

	err = I2C_Write(device_reg);
	check_err(err, "[I2C_WritePreamble] Error transmitting the Internal Device Register");
}

void I2C_WriteRegister(uint8_t device_address, uint8_t device_reg, uint8_t data) {
	uint8_t err;
	
	I2C_WritePreamble(device_address, device_reg);
	
	err = I2C_Write(data);
	check_err(err, "[I2C_WriteRegister] Error transmitting the data to be written");

	I2C_Stop();
}

void I2C_WriteNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, uint8_t* data) {
	uint8_t err;
	
	I2C_WritePreamble(device_address, device_reg_start);
	
	int i;
	for(i=0; i<n; i++) {
		err = I2C_Write(data[i]);
		check_err(err, "[I2C_WriteNRegisters] Error transmitting the data to be written");
	}
	I2C_Stop();
}





static void I2C_ReadPreamble(uint8_t device_address, uint8_t device_reg) {
	uint8_t err;
	
	err = I2C_Start();
	check_err(err, "[I2C_ReadPreamble] Error transmitting the Start condition");
	
	err = I2C_SendAddress(device_address, WRITE);
	check_err(err, "[I2C_ReadPreamble] Error transmitting the Device Address");
	
	err = I2C_Write(device_reg);
	check_err(err, "[I2C_ReadPreamble] Error transmitting the Internal Device Register");
	
	
	err = I2C_Repeated_Start();
	check_err(err, "[I2C_ReadPreamble] Error re-transmitting the Start condition");
	
	//construct the device address, adding the read bit(1) at the end
	err = I2C_SendAddress(device_address, READ);
	check_err(err, "[I2C_ReadPreamble] Error transmitting the Device Address");
}

uint8_t I2C_ReadRegister(uint8_t device_address, uint8_t device_reg) {	
	I2C_ReadPreamble(device_address, device_reg);
	
	//send a read without outputting acks (==>> NACK)
	unsigned char data = I2C_Read(0);
	
	I2C_Stop();
	
	return data;
}

void I2C_ReadNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, uint8_t* data) {
	I2C_ReadPreamble(device_address, device_reg_start);
	
	int i;
	for(i=0; i<n-1; i++) {
		data[i] = I2C_Read(ACK);
	}
	//send a read without outputting acks (==>> NACK)
	data[n-1] = I2C_Read(NACK);
	
	I2C_Stop();
}

void I2C_WriteBits(uint8_t device_address, uint8_t device_reg, uint8_t bit_mask, uint8_t new_bits) {
	uint8_t old_value = I2C_ReadRegister(device_address, device_reg);
	
	// (old_value & ~bit_mask) to reset bits to change
	// ( ... ) | (new_bits & mask) to get new values for the bits
	uint8_t new_value = (old_value & ~bit_mask) | (new_bits & bit_mask);
	
	I2C_WriteRegister(device_address, device_reg, new_value);
}
