#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include <stdlib.h>
#include <string.h>

#ifdef DEBUG_PRINTF_
#include <stdio.h>
#endif

#include "i2c.h"

static volatile I2C_Operation* global_I2C_ops;


void I2C_Init(void) {
	cli();//TODO need?
	
	//set te frequency of the Serial Data Clock
		//no need of prescaler, so TWSR = 0
	TWSR = 0x00;
	TWBR = ((FREQ_CPU/FREQ_I2C)-16)/2; // = ((16/0.1)-16)/2 = 144/2 = 72.
	
	//Active internal pull-up resistors for SCL and SDA //TODO need?
		//on atmega 2560 they are PD0 and PD1
	PORTD |= 1|(1<<1);

	// Disable slave mode
	TWAR = 0;
	
	//TWEN: TWI Enable, to start TWI interface
	//TWIE : TWI Interrupt Enable in TWCR
	TWCR = (1<<TWEN)|(1<<TWIE);
	sei();
}

/*
 * When the TWINT Flag is asserted,
 * the TWI has finished an operation
 * 		and awaits application response.
 * In this case, the TWI Status Register (TWSR)
 * 	contains a value indicating the current state of the TWI bus.
 */


void I2C_Enqueue_Operation(uint8_t device_address_7bit, uint8_t rw, uint8_t n, uint8_t* data, PostProcessFunction_t post_process_fn) {
	if( n <= 0 )	// no empty operations in the queue
		return;
	
/*  #ifdef DEBUG_PRINTF_*/
/*  if( rw )//read*/
/*  	printf("I2C_Enqueue_Operation READ\n");*/
/*  else*/
/*  	printf("I2C_Enqueue_Operation WRITE, device: 0x%02x, reg: 0x%02x\n", device_address_7bit, data[0]);*/
/*  #endif*/
	
	I2C_Operation* op = (I2C_Operation*)malloc(sizeof(I2C_Operation));
	
	op->next = NULL;
	
	op->device_address = (rw) ? (device_address_7bit<<1)|1 : (device_address_7bit<<1);
	
	op->buffer = (uint8_t*)malloc(n*sizeof(uint8_t));
	if( data != NULL )
		memcpy(op->buffer, data, n);
	op->buflen = n;
	op->bufpos = 0;
	
	op->post_process_fn = post_process_fn;
	
	// updates on global list of I2C_Operations will be executed atomically
		// (no other interrupts)
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		if( global_I2C_ops == NULL ) {
			//if there was no pending operation,
				//appends current one
			global_I2C_ops = op;
				//and schedules it immediately
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA);
		} else {
			I2C_Operation* aux = global_I2C_ops;
			while( aux->next != NULL)
				aux = aux->next;
			aux->next = op;
		}
	}
}

ISR(TWI_vect) {
	#ifdef DEBUG_PRINTF_
	printf("ISR TWI: 0x%02x\n", TWSR & 0xF8);
	#endif

	//if there is no global I2C_Operation, exits
	if( global_I2C_ops == NULL )
		return;	//in an ISR return is translated to reti()
	
	switch( TWSR & 0xF8 ) {	//TW Status
		//a START condition has been transmitted
		case TWSR_START:
		
		//a repeated START condition has been transmitted
		case TWSR_REPEATED_START:
			//send address_7bit + read_bit
			TWDR = global_I2C_ops->device_address;
			#ifdef DEBUG_PRINTF_
			printf("ISR TWI[TWSR_REPEATED_START : 0x%02x] : device_address 0x%02x, %d [1: r, 0: w]\n", TWSR & 0xF8, TWDR>>1, TWDR&1);
			#endif
			//resets bufpos (should already be 0)
			global_I2C_ops->bufpos = 0;
			TWCR = (1<< TWINT)|(1<<TWEN)|(1<<TWIE);
			break;
		
		// arbitration lost in SLA+R/W or NOT ACK bit
		case TWSR_SLARW_ARB_LOST:
			//start condition will be retransmitted
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA);
			break;
		
	/* WRITE operation, Master Transmitter Mode */
	
		//SLA+W has been transmitted; ACK has been received
		case TWSR_MT_SLA_ACK:
			//sends first byte (device reg)
			TWDR = global_I2C_ops->buffer[global_I2C_ops->bufpos];
			#ifdef DEBUG_PRINTF_
			printf("ISR TWI[TWSR_MT_SLA_ACK : 0x%02x] : reg_address 0x%02x\n", TWSR & 0xF8, TWDR);
			#endif
			global_I2C_ops->bufpos++;
			TWCR = (1<< TWINT)|(1<<TWEN)|(1<<TWIE);
			break;
		
		// SLA+W has been transmitted; NOT ACK has been received
		case TWSR_MT_SLA_NACK:
			//STOP condition followed by a START condition will be transmitted
				//and TWSTO Flag will be reset
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA)|(1<<TWSTO);
			break;
		
		// data byte has been transmitted; ACK has been received
		case TWSR_MT_DATA_ACK:
			//sends next byte
			if( global_I2C_ops->bufpos < global_I2C_ops->buflen ) {
				TWDR = global_I2C_ops->buffer[global_I2C_ops->bufpos];
				#ifdef DEBUG_PRINTF_
				printf("ISR TWI[TWSR_MT_DATA_ACK : 0x%02x] : data 0x%02x\n", TWSR & 0xF8, TWDR);
				#endif
				global_I2C_ops->bufpos++;
				TWCR = (1<< TWINT)|(1<<TWEN)|(1<<TWIE);
				break;
			} else {
				//calls post-processing function
				if( global_I2C_ops->post_process_fn != NULL ) {
					#ifdef DEBUG_PRINTF_
					//printf("ISR TWI MT call callback\n");
					#endif
					(*(global_I2C_ops->post_process_fn))(global_I2C_ops->buffer, global_I2C_ops->buflen);
				} else {
					#ifdef DEBUG_PRINTF_
					//printf("ISR TWI MT no callback\n");
					#endif			
				}
				#ifdef DEBUG_PRINTF_
				printf("ISR TWI[TWSR_MT_DATA_ACK : 0x%02x] : switch to next op\n", TWSR & 0xF8);
				#endif
				goto next_operation;
			}
			
		case TWSR_MT_DATA_NACK:
			//STOP condition followed by a START condition will be transmitted
				//and TWSTO Flag will be reset
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA)|(1<<TWSTO);
			break;

	/* READ operation, Master Receiver Mode */
		
		// SLA+R has been transmitted; ACK has been received
		case TWSR_MR_SLA_ACK:
			//if just one read, ACK pulse not generated
			if (global_I2C_ops->buflen == 1)
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
			else 	//else ACK is generated
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
			break;
		
		// SLA+R has been transmitted; NOT ACK has been received
		case TWSR_MR_SLA_NACK:
			//STOP condition followed by a START condition will be transmitted
				//and TWSTO Flag will be reset
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA)|(1<<TWSTO);
			break;
		
		// data byte has been received; ACK has been returned
		case TWSR_MR_DATA_ACK:
			//read byte
			global_I2C_ops->buffer[global_I2C_ops->bufpos] = TWDR;
			global_I2C_ops->bufpos++;
			//if this is last operation, ACK pulse not generated
			if( global_I2C_ops->bufpos+1 == global_I2C_ops->buflen )
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
			else //else ACK is generated
				TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWEA);
			break;
		
		// data byte has been received; NOT ACK has been returned
		case TWSR_MR_DATA_NACK:
			//read byte
			global_I2C_ops->buffer[global_I2C_ops->bufpos] = TWDR;
			global_I2C_ops->bufpos++;
			//calls post-processing function
			if( global_I2C_ops->post_process_fn != NULL ) {
				#ifdef DEBUG_PRINTF_
				printf("ISR TWI MR call callback\n");
				#endif
				(*(global_I2C_ops->post_process_fn))(global_I2C_ops->buffer, global_I2C_ops->buflen);
			} else {
				#ifdef DEBUG_PRINTF_
				//printf("ISR TWI MR no callback\n");
				#endif			
			}
			goto next_operation;
					
		default:
			//it should never get here, I treated all possible Status Codes
			exit(1);
	}
	return;
	
	next_operation: {
		I2C_Operation* ended_op = global_I2C_ops;
			//forwards to next transaction
		global_I2C_ops = global_I2C_ops->next;
		
		if( global_I2C_ops != NULL )	//repeated start
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA);
		else 	//if there is no next operation, transmits a stop
			TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTO);
		
		free(ended_op->buffer);
		free(ended_op);
		return;
	}
}
