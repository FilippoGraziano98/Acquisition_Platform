#include <avr/interrupt.h>

#include "encoder.h"

// Encoder 0
#define ENC0_CTL	DDRB	//encoder port control
#define ENC0_WR	PORTB	//encoder port write	
#define ENC0_RD	PINB	//encoder port read
#define ENC0_A 1	//PB0, pin 53
#define ENC0_B 2	//PB1, pin 52
#define ENC0_MASK (ENC0_A|ENC0_B)
static Encoder_t Enc0;

//! @brief this represents a transition table
//!        each entry represents the output of that transition
//!        the index of the table is [prev_state, next_state]
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
  Enc0.prev_value <<=2;  //remember previous state
  Enc0.prev_value = Enc0.prev_value | ( ENC0_RD & ENC0_MASK );
  Enc0.counter += _transition_table[ Enc0.prev_value & 0x0F ];
}

void Encoder_Init(void) {
  ENC0_CTL &= ~ENC0_MASK; //set ENC_MASK pins as input
  ENC0_WR |= ENC0_MASK; //enable pull up resistors
  PCICR |= (1 << PCIE0); // set interrupt on change, looking up PCMSK0
  PCMSK0 |= ENC0_MASK;   // set PCINT0 to trigger an interrupt on state change
  
  Enc0.prev_value = 3; //0b11
  Enc0.counter = 0;
  
  sei();
}


void Encoder_getCounts(int32_t* cnt0) {
	*cnt0 = Enc0.counter;
}
