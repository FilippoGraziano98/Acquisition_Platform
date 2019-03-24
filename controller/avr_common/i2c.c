//https://exploreembedded.com/wiki/AVR_C_Library
#include <avr/io.h>
#include <util/delay.h>

#include "i2c.h"

#define DEBUG 0

#if DEBUG
#include <stdio.h> //needed if DEBUG==1
#endif


void I2C_Init(void) {
	//set te frequency of the Serial Data Clock
		//no need of prescaler, so TWSR = 0
  TWSR = 0x00;
  TWBR = ((FREQ_CPU/FREQ_I2C)-16)/2; // = ((16/0.1)-16)/2 = 144/2 = 72.
  
	//TWEN to start TWI interface
  TWCR = (1<<TWEN); //enab1e TWI module
}


void _I2C_Start(void) {
	//TWINT to start the operation od the TWI(i2c)
	//TWEN to start TWI interface
	//TWSTA to make the microcontroller master on the bus
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA);
	//current task: make microcontroller master on the bus

	//wait current task ended
  while (!(TWCR & (1<<TWINT)));
}

uint8_t I2C_Start(void) {
	_I2C_Start();
  
  //check if status code generated is 0x08
  #if DEBUG
  printf("[I2C_Start] status : %02x\n",(int)(TWSR&0xF8));
  #endif
  return (TWSR&0xF8) == TWSR_START;
}

uint8_t I2C_Repeated_Start(void) {
	_I2C_Start();
  
  //check if status code generated is 0x08
  #if DEBUG
  printf("[I2C_Repeated_Start] status : %02x\n",(int)(TWSR&0xF8));
  #endif
  return (TWSR&0xF8) == TWSR_REPEATED_START;
}

void I2C_Stop(void) {
	//TWSTO generates a STOP condition
  TWCR = (1<< TWINT)|(1<<TWEN)|(1<<TWSTO);
   _delay_us(10) ; //wait for a short time
}

//data is an 8 bit info
static void _I2C_Write(unsigned char data) {
  TWDR = data;
	//TWINT to start the operation od the TWI(i2c)
	//TWEN to start TWI interface
  TWCR = (1<< TWINT)|(1<<TWEN);
  
  //wait current task ended
  while (!(TWCR & (1 <<TWINT)));
}

uint8_t I2C_SendAddress(uint8_t address_7bit, uint8_t rw) {
	uint8_t addr = address_7bit<<1;
	if(rw) //read
		addr |= 1;	//set rw bit to 1
	
  _I2C_Write(addr);
  
  //check if status code generated is 0x28
  #if DEBUG
  printf("[I2C_SlaveAddress] status : %02x\n",(int)(TWSR&0xF8));
  #endif
  if(rw) //read, Master Receiver
	  return (TWSR&0xF8) == TWSR_MR_SLA_ACK;
	 else //write, Master Transmitter
	 	return (TWSR&0xF8) == TWSR_MT_SLA_ACK;
}

uint8_t I2C_Write(unsigned char data) {
  _I2C_Write(data);
  
  //check if status code generated is 0x28
  #if DEBUG
  printf("[I2C_Write] status : %02x\n",(int)(TWSR&0xF8));
  #endif
  return (TWSR&0xF8) == TWSR_MT_DATA_ACK;
}

unsigned char I2C_Read(uint8_t ack) {
	//TWINT to start the operation od the TWI(i2c)
	//TWEN to start TWI interface
	//TWEA controls the generation of the acknowledge pulse.
	//	- if 1, then ACK pulse is generated
	//	- else, not
	TWCR = (1<<TWINT)|(1<<TWEN)|(ack<<TWEA);
	
  //wait current task ended
	while (!(TWCR & (1 <<TWINT)));
	
	return TWDR;
}
