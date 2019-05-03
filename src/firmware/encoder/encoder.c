#include <avr/interrupt.h>

#include "encoder.h"

// Encoders
#define ENC_CTL DDRB	//encoder port control
#define ENC_WR	PORTB	//encoder port write	
#define ENC_RD	PINB	//encoder port read

#define ENC0_A 2	//PB1, pin 52
#define ENC0_B 1	//PB0, pin 53
#define ENC1_A 4	//PB2, pin 51
#define ENC1_B 8	//PB3, pin 50

#define ENC_MASK (ENC0_A|ENC0_B|ENC1_A|ENC1_B)

static Encoder_t Encs[NUM_ENCODERS];

//! @brief this represents a transition table
//!				each entry represents the output of that transition
//!				the index of the table is [prev_state, next_state]
static const int8_t _transition_table []= {
		0,	//0000
	 -1,	//0001
		1,	//0010
		0,	//0011
		1,	//0100
		0,	//0101
		0,	//0110
	 -1,	//0111
	 -1,	//1000
		0,	//1001
		0,	//1010
		1,	//1011
		0,	//1100
		1,	//1101
	 -1,	//1110
		0		//1111
};

// interrupt routine for position PCINT0
ISR(PCINT0_vect) {
	uint8_t port_value = ENC_RD & ENC_MASK;
	
	uint8_t i;
	for(i=0; i<NUM_ENCODERS; i++){
		Encs[i].prev_value <<=2;	//remember previous state
			// port_value memorizza i valore di tutti gli encoder
			// &0x03 è perchè consideriamo un encoder alla volta
			// con >>= 2 a fine ciclo, passiamo all'encoder successivo
		Encs[i].prev_value = Encs[i].prev_value | ( port_value & 0x03 );
		Encs[i].counter += _transition_table[ Encs[i].prev_value & 0x0F ];
		
		port_value >>= 2;
	}
}


void Encoders_Init(void) {
	cli();

	ENC_CTL &= ~ENC_MASK;			//set ENC_MASK pins as input
	ENC_WR |= ENC_MASK; 			//enable pull up resistors
	PCICR |= (1 << PCIE0); 		//set interrupt on change, looking up PCMSK0
	PCMSK0 |= ENC_MASK;	 			//set PCINT0 to trigger an interrupt on state change

	uint8_t i;
	for(i=0; i<NUM_ENCODERS; i++){
		Encs[i].prev_value = 3; 	//0b11
		Encs[i].counter = 0;
	}
	
	sei();
}


void Encoders_getCounts(int32_t* cnts) {
	uint8_t i;
	for(i=0; i<NUM_ENCODERS; i++)
		cnts[i] = Encs[i].counter;
}
