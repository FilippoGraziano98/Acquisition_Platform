#pragma once

typedef struct Encoder_t {
	uint8_t prev_value;	// previous state of the encoder
	int32_t counter;		// global counter of the encoder
} Encoder_t;


void Encoder_Init(void);

void Encoder_getCounts(int32_t* cnt0);
