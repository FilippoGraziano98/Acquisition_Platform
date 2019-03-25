#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "../avr_common/uart.h" // this includes the printf and initializes it


#define ENC_CTL	DDRB	//encoder port control
#define ENC_WR	PORTB	//encoder port write	
#define ENC_RD	PINB	//encoder port read
#define ENC_A 1	//PB0, pin 53
#define ENC_B 2	//PB1, pin 52

#define ENC_MASK 3 //ENC_A|ENC_B

volatile uint8_t int_occurred=0;


uint8_t prev_value;
int counter;

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
	int_occurred = 1;
  prev_value <<=2;  //remember previous state
  prev_value = prev_value | ( ENC_RD & ENC_MASK );
  counter += _transition_table[ prev_value & 0x0F ];
}


int main(void){
  printf_init(); 
  ENC_CTL &= ~ENC_MASK; //set ENC_MASK pins as input
  ENC_WR |= ENC_MASK; //enable pull up resistors
  PCICR |= (1 << PCIE0); // set interrupt on change, looking up PCMSK0
  PCMSK0 |= ENC_MASK;   // set PCINT0 to trigger an interrupt on state change
  
  prev_value = 3; //0b11
  counter = 0;
  
  sei();
  while(1){
    while (! int_occurred);
    // we reset the flag;
    int_occurred=0;
    printf("counter value = %d\n", counter);
    printf("***********************\n");
  }
}
