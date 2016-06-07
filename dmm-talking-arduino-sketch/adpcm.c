/*
 * IMA_ADPCM Decoding. Tweaked to produce better assembly on the Atmel AVR.
 * Copyright (C) 2016 by Nicolas Pitre <nico@fluxnic.net>
 * This code is GPL licensed.
 */

#include <stdint.h>
#include "adpcm.h"

#ifndef SELF_TEST
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_byte(ptr) (*(ptr))
#define pgm_read_word(ptr) (*(ptr))
#endif

static const uint16_t ADPCM_step_table[89] PROGMEM = {
      7,      8,      9,     10,     11,     12,     13,     14,     16,
     17,     19,     21,     23,     25,     28,     31,     34,     37,
     41,     45,     50,     55,     60,     66,     73,     80,     88,
     97,    107,    118,    130,    143,    157,    173,    190,    209,
    230,    253,    279,    307,    337,    371,    408,    449,    494,
    544,    598,    658,    724,    796,    876,    963,   1060,   1166,
   1282,   1411,   1552,   1707,   1878,   2066,   2272,   2499,   2749,
   3024,   3327,   3660,   4026,   4428,   4871,   5358,   5894,   6484,
   7132,   7845,   8630,   9493,  10442,  11487,  12635,  13899,  15289,
  16818,  18500,  20350,  22385,  24623,  27086,  29794,  32767
};

static const int8_t ADPCM_index_ttable[16] PROGMEM = {
     -1,     -1,     -1,     -1,      2,      4,      6,      8,
     -1,     -1,     -1,     -1,      2,      4,      6,      8
};

static int8_t ADPCM_index;
static uint16_t ADPCM_prev_sample;

/*
 * ADPCM_decode: decode next sample given a 4-bit encoded sample
 * Returns an unsigned 16-bit sample value.
 * Must be called after ADPCM_init).
 */
uint16_t ADPCM_decode(uint8_t code)
{
	uint8_t index = ADPCM_index;
	ADPCM_index += pgm_read_byte(&ADPCM_index_ttable[code]);
	if (ADPCM_index < 0) ADPCM_index = 0;
	if (ADPCM_index > 88) ADPCM_index = 88;

	uint16_t step = pgm_read_word(&ADPCM_step_table[index]);
	uint16_t delta = 0;
	if (code & 4) delta += step;
	step >>= 1;
	if (code & 2) delta += step;
	step >>= 1;
	if (code & 1) delta += step;
	step >>= 1;
	delta += step;

	uint16_t sample = ADPCM_prev_sample;
	if (code & 8) {
		sample -= delta;
		if (sample > ADPCM_prev_sample) sample = 0;
	} else {
		sample += delta;
		if (sample < ADPCM_prev_sample) sample = 65535;
	}
	ADPCM_prev_sample = sample;

	return sample;
}

/*
 * ADPCM_init: initialize decoder state.
 */
void ADPCM_init(void)
{
	ADPCM_prev_sample = 0x8000;
	ADPCM_index = 0;
}

#ifdef SELF_TEST

#include <stdio.h>

int main()
{
	uint16_t sample;

	ADPCM_init();

	while (!feof(stdin)) {
		int x = getchar();
		sample = ADPCM_decode((x >> 4) & 0xf);
		sample ^= 0x8000;
		putchar(sample);
		putchar(sample >> 8);
		sample = ADPCM_decode((x >> 0) & 0xf);
		sample ^= 0x8000;
		putchar(sample);
		putchar(sample >> 8);
	}

	return 0;
}

#endif
