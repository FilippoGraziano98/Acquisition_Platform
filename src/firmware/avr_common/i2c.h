#pragma once

#define FREQ_CPU 16000000L	//cpu clock spedd at 16MHz
//#define FREQ_I2C 100000L		//i2c clock spped at 100KHz
//according to datasheet, imu works at 400KHz
#define FREQ_I2C 400000L		//i2c clock spped at 400KHz


//status indicating START condition successfully sent
#define TWSR_START 0x08
#define TWSR_REPEATED_START 0x10
#define TWSR_SLARW_ARB_LOST 0x38

//status codes for Master Transmitter mode (MT)
#define TWSR_MT_SLA_ACK 0x18
#define TWSR_MT_SLA_NACK 0x20
#define TWSR_MT_DATA_ACK 0x28
#define TWSR_MT_DATA_NACK 0x30

//status codes for Master Receiver mode (MR)
#define TWSR_MR_SLA_ACK 0x40
#define TWSR_MR_SLA_NACK 0x48
#define TWSR_MR_DATA_ACK 0x50
#define TWSR_MR_DATA_NACK 0x58

typedef void(*PostProcessFunction_t)(uint8_t* buffer, uint8_t buflen);

typedef struct I2C_Operation {
	struct I2C_Operation* next;	//linked list of remaining ops
	uint8_t device_address;//7 bits for device_address, 1 bit for read/write
	
	//note: if write device_reg will be first byte in buffer
		// if read buffer starts as empty
	uint8_t* buffer;	//full if write, empty if read
	uint8_t buflen;		//length of the buffer (#data to read/write)
	uint8_t bufpos;		//current pos in buffer
	
	PostProcessFunction_t post_process_fn; //to store values if it was a read op
} I2C_Operation;

/*
 *	This function is used to initialize the I2c Module.
 */
void I2C_Init(void);

/*
 * I2C_ReadNRegisters: reads from n consecutive registers
 *	@param device_address : address of device on the bus
 *	@param rw : read/write flag (1 if read, 0 if write)
 *	@param n : number of contiguos register to read/write
 *			NOTE: n must be > 0
 *	@param data : array of n bytes (preallocated) [full if write, empty if read]
 *			NOTE: if write, device_reg_start must be first byte in data
 *				 			if read, data can be null
 *	@param post_process_fn : function to be called when operation finished
 *			NOTE: if read, post_process_fn to store data in right variables
 *				 			if write, post_process_fn can be null
 *	@param post_process_args_n : number of post_process_args for the post_process_fn
 *			NOTE: post_process_args_n must be <= MAX_POST_PROCESS_ARGS
 *	@param post_process_args : post_process_args for the post_process_fn
 */
void I2C_Enqueue_Operation(uint8_t device_address_7bit, uint8_t rw, uint8_t n, uint8_t* data, PostProcessFunction_t post_process_fn);
