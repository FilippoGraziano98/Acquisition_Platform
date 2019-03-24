#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "avr_common/uart.h" // this includes the printf and initializes it
#include "avr_common/i2c.h"
#include "avr_common/i2c_communication.h"


#define MAX_BUF 256
int main(void){
  UART_Init();
  UART_TxString((uint8_t*)"write something, i'll repeat it\n");
  uint8_t buf[MAX_BUF];
  uint8_t size=0;
  while(1) {
		memset(buf, 0, MAX_BUF);
    size = UART_RxString(buf);
    UART_TxByte(size);
    
    size = UART_TxString(buf);
    UART_TxByte(size);
  }
}
