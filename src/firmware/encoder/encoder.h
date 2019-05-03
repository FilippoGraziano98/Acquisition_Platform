#pragma once

#define NUM_ENCODERS 2

typedef struct Encoder_t {
	uint8_t prev_value;	// previous state of the encoder
	int32_t counter;		// global counter of the encoder
} Encoder_t;

/*
 * Encoders_Init
 *		initializes both encoders
 */
void Encoders_Init(void);

/*
 * Encoders_getCounts
 *		fills cnts array with encoders' current counter values
 *	NOTE: assumes cnts is a preallocated array
 *					with enough space for NUM_ENCODERS int32_t
 */
void Encoders_getCounts(int32_t* cnts);
