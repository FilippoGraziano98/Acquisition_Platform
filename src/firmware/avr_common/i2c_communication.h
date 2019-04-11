#pragma once


#define check_err(err, msg) if (err!=1){printf(msg); exit(1);}

/*
 *	now we will use the i2c basics defined in i2c.h
 *	to implement the specific protocol adpoted by our imu
 */

/*
 *	Writing registers on the slave device
 *		- master transmits the start condition
 *		- master transmits the (7bits) device address and the write bit (0)
 *				device acks
 *		- master puts the internal register address of the device on the bus
 *				device acks
 *		- master puts the data to be written on the bus
 *				device acks
 *		- eventually master could go on outputting data
 *				which will be written in consecutive registers
 *					device acks each data
 *		- master transmits the stop condition
 */
void I2C_WriteRegister(uint8_t device_address, uint8_t device_reg, uint8_t data);
/*
 * I2C_WriteNRegisters: writes to n consecutive registers
 *	@param device_reg_start : addres of first reg to write
 *	@param n : number of contiguos register to write
 *	@param data : array of n bytes
 */
void I2C_WriteNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, uint8_t* data);


/*
 *	Reading registers on the slave device
 *		- master transmits the start condition
 *		- master transmits the (7bits) device address and the write bit (0)
 *				device acks
 *		- master puts the internal register address of the device on the bus
 *				device acks
 *
 *		- master transmits the start condition (restart)
 *		- master transmits the (7bits) device address and the read bit (1)
 *				device acks
 *		- device puts the data to be written on the bus
 *				master acks if more data is expected
 *				else master nacks
 *		- if more data was expected it could all be received
 *				device will stop sending data upon receiving a nack
 *		- master transmits the stop condition
 */
uint8_t I2C_ReadRegister(uint8_t device_address, uint8_t device_reg);
/*
 * I2C_ReadNRegisters: reads from n consecutive registers
 *	@param device_reg_start : addres of first reg to read
 *	@param n : number of contiguos register to read
 *	@param data : array of n bytes (preallocated)
 */
void I2C_ReadNRegisters(uint8_t device_address, uint8_t device_reg_start, int n, uint8_t* data);


/*
 * I2C_WriteBits :
 *	@param device_reg : register to update
 *	positions set to 0 in @param bit_mask won't be changed
 *	while for pos. set to 1 new value will be get in @param new_bits
 */
void I2C_WriteBits(uint8_t device_address, uint8_t device_reg, uint8_t bit_mask, uint8_t new_bits);
