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

volatile uint8_t previous_pins;
volatile uint8_t current_pins;

volatile uint8_t int_occurred=0;
volatile uint16_t int_count=0;

// interrupt routine for position PCINT0
ISR(PCINT0_vect) {
  previous_pins=current_pins;
  current_pins=ENC_RD&ENC_MASK;
  int_occurred=1;
  //int_count=1;
  int_count++;
}


int main(void){
  printf_init(); 
  ENC_CTL &= ~ENC_MASK; //set ENC_MASK pins as input
  ENC_WR |= ENC_MASK; //enable pull up resistors
  PCICR |= (1 << PCIE0); // set interrupt on change, looking up PCMSK0
  PCMSK0 |= ENC_MASK;   // set PCINT0 to trigger an interrupt on state change 
  sei();
  while(1){
    while (! int_occurred);
    // we reset the flag;
    int_occurred=0;
    printf("int %u, p:%x, c:%x!\n", int_count, previous_pins, current_pins);
  }
}
