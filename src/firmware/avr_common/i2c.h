/*
 * this version is now obsolete !!
 *		i2c.h and i2c.c contains an interrupt driven uart implementation
 */

#pragma once

#define FREQ_CPU 16000000L	//cpu clock spedd at 16MHz
//#define FREQ_I2C 100000L		//i2c clock spped at 100KHz
//according to datasheet, imu works at 400KHz
#define FREQ_I2C 400000L		//i2c clock spped at 400KHz


//status indicating START condition successfully sent
#define TWSR_START 0x08
#define TWSR_REPEATED_START 0x10


//status codes for Master Transmitter mode (MT)
//status indicating address packet successfully sent
#define TWSR_MT_SLA_ACK 0x18
#define TWSR_MT_SLA_NACK 0x20
//status indicating data packet successfully sent
#define TWSR_MT_DATA_ACK 0x28
#define TWSR_MT_DATA_NACK 0x30


//status codes for Master Receiver mode (MR)
//status indicating address packet successfully sent
#define TWSR_MR_SLA_ACK 0x40
#define TWSR_MR_SLA_NACK 0x48
//status indicating data packet successfully sent
#define TWSR_MR_DATA_ACK 0x50
#define TWSR_MR_DATA_NACK 0x58

/*
 *	This function is used to initialize the I2c Module.
 */
void I2C_Init(void);


/*
 *	This function is used to generate I2C Start Condition.
 *		Start Condition: SDA goes low when SCL is High.
 *	@return: 1 if ok, 0 else
 */
uint8_t I2C_Start(void);

/*
 *	This function is used to generate a repeated I2C Start Condition.
 *		Start Condition: SDA goes low when SCL is High.
 *	@return: 1 if ok, 0 else
 */
uint8_t I2C_Repeated_Start(void);

/*
 *	This function is used to generate I2C Stop Condition.
 *		Stop Condition: SDA goes High when SCL is High.
 */
void I2C_Stop(void);

/*
 *	This function is used to send a Device Address over the bus
 *	@param address_7bit : is the 7bit address of the slave device
 *													you want to communicate with
 *	@param rw : rw flag to determine if master transmitter/receiver mode
 *								if rw = 1, read
 *								else, write
 *	@return: 1 if ok, 0 else
 */
uint8_t I2C_SendAddress(uint8_t address_7bit, uint8_t rw);

/*
 *	This function is used to send a byte on SDA line using I2C protocol
 *		8bit data is sent bit-by-bit on each clock cycle.
 *			MSB(bit) is sent first and LSB(bit) is sent at last.
 *		Data is sent when SCL is low.
 *	@return: 1 if ok, 0 else
 */
uint8_t I2C_Write(uint8_t data);


/*
 *	This function is used to receive a byte on SDA line using I2C protocol.
 *		8bit data is received bit-by-bit each clock and finally packed into Byte.
 *			MSB(bit) is received first and LSB(bit) is received at last.
 */
#define ACK 1
#define NACK 0
uint8_t I2C_Read(uint8_t ack);
