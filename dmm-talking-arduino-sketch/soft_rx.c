/*
 * Fully Interrupt Driven Software Based Serial Reception.
 *
 * Copyright 2016 by Nicolas Pitre <nico@fluxnic.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Mode ofOperation:
 *
 * The interrupt handler for the pin that receives the serial signal only
 * records the time between signal transitions. The timing is analysed
 * and data bytes reconstructed outside of interrupt context for better
 * overall IRQ reliability. Most importantly, there is NO busy looping
 * anywhere. Obviously, for this to work reliably, there must not be any
 * other IRQ handlers disabling IRQs for more than half of a bit duration.
 */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "soft_rx.h"


#define RX_BAUDRATE 19200

#define RX_PIN 2
#define RX_INVERTED 1	/* hardware signal is inverted */

#define RX_ERR_COUNT 0	/* maintain error counters */
#define RX_DEBUG 0	/* print debug traces */
#define RX_SLEEP_HACK 1	/* needed when disabling the BOD during sleep */

/*
 * We sample TCNT0 and timer0_overflow_count on every edge transition.
 * TCNT0 is prescaled by 64 from F_CPU. Given F_CPU = 8MHz we have a time
 * resolution of 8 microsecs. That should be good up to 57600 bauds.
 * Higher baudrates do require a faster time source.
 */
#define RX_BIT_TICKS (F_CPU / 64 / RX_BAUDRATE)
#if RX_BIT_TICKS < 2
#error "clock source too slow for baudrate"
#endif

/*
 * The longest period without transition corresponds to the start bit
 * and 8 zero bits. We don't really care how long the stop bit lasts.
 * Let's make sure we can record that period duration plus 5% of slack
 * with 8 bits.
 */
#define RX_LARGEST_PERIOD_TICKS (F_CPU / 64 * 9 / RX_BAUDRATE)
#if RX_LARGEST_PERIOD_TICKS >= (2048 - 102)
#error "baudrate too small or clock too fast for available downscale values"
#elif RX_LARGEST_PERIOD_TICKS >= (1024 - 51)
#define RX_TICK_DOWNSCALE 3
#elif RX_LARGEST_PERIOD_TICKS >= (512 - 26)
#define RX_TICK_DOWNSCALE 2
#elif RX_LARGEST_PERIOD_TICKS >= (256 - 13)
#define RX_TICK_DOWNSCALE 1
#else
#define RX_TICK_DOWNSCALE 0
#endif

/*
 * Now if bit pulses are short, the summing of multiple consecutive bits
 * might require fractional bit duration computations. Let's scale up the
 * recorded durations to take advantage of the available 8 bits.
 */
#if RX_LARGEST_PERIOD_TICKS < (32 - 2)
#define RX_TICK_UPSCALE 3
#elif RX_LARGEST_PERIOD_TICKS < (64 - 3)
#define RX_TICK_UPSCALE 2
#elif RX_LARGEST_PERIOD_TICKS < (128 - 6)
#define RX_TICK_UPSCALE 1
#else
#define RX_TICK_UPSCALE 0
#endif

/*
 * Our bit duration according to the above.
 * (Beware bit truncation.)
 */
#define RX_BIT_DURATION \
	((((F_CPU / 64) << RX_TICK_UPSCALE) / RX_BAUDRATE) >> RX_TICK_DOWNSCALE)

/*
 * Debugging facilities.
 */
#if !RX_DEBUG
/* to disactivate debug trace code */
#define _dbg_(a,b,c,d) do{}while(0)
#endif
#define _rx_dbg(string)          _dbg_(string,0,0,0)
#define _rx_dbg_ln(string)       _dbg_(string,0,0,1)
#define _rx_dbg_v(string,val)    _dbg_(string,1,val,0)
#define _rx_dbg_v_ln(string,val) _dbg_(string,1,val,1)

/*
 * Let's use a buffer with 256 entries. This allows for having 8-bit indexes
 * that wrap automatically without any special handling. In the worst case
 * i.e. with a transition for every bit, that means 24 bytes can be buffered
 * without overflowing.
 */
static uint8_t rx_delta[256];
static volatile uint8_t rx_head, rx_tail;

/*
 * Error counters.
 */
#if !RX_ERR_COUNT
#define _rx_err(type)  _rx_dbg_ln(" ERR_" #type)
#else
unsigned int soft_rx_err_overflow;
unsigned int soft_rx_err_irq_sync;
unsigned int soft_rx_err_irq_time;
unsigned int soft_rx_err_stop_sync;
#define _rx_err(type) \
do { \
	soft_rx_err_##type++; \
	_rx_dbg_v_ln(" ERR_" #type, soft_rx_err_##type); \
} while (0)
#endif
	
/* direct hardware access */
static volatile uint8_t *rx_port;
static uint8_t rx_mask;

/* from .../avr/cores/arduino/wiring.c */
extern volatile unsigned long timer0_overflow_count;

static inline uint32_t get_tstamp(void)
{
	uint8_t t_lo = TCNT0;
	uint32_t t_hi = timer0_overflow_count;
	if ((TIFR0 & _BV(TOV0)) && t_lo < 255) t_hi++;
	return t_lo | (t_hi << 8);
}

static uint32_t rx_last_time;

static inline uint8_t get_delta(bool update)
{
	uint32_t now = get_tstamp();
	uint32_t delta32 = now - rx_last_time;
	if (update) rx_last_time = now;
	uint8_t delta = delta32 >> RX_TICK_DOWNSCALE;
	if (delta32 > RX_LARGEST_PERIOD_TICKS)
		return 255;
	else
		return delta << RX_TICK_UPSCALE;
}

ISR(PCINT2_vect)
{
	uint8_t delta = get_delta(1);
	uint8_t signal = (!!(*rx_port & rx_mask)) ^ RX_INVERTED;

	uint8_t curr = rx_head, next = curr + 1;
next:	if (next == rx_tail) {
		/* overflow */
		_rx_err(overflow);
		return;
	}

#if RX_SLEEP_HACK
	/*
	 * There is a delay of about 60 microsecs before the ISR is serviced
	 * when coming back from sleep where the BOD was disabled. Try to
	 * mitigate this which otherwise eats up our start bit.
	 */
	if (_SLEEP_CONTROL_REG & _SLEEP_ENABLE_MASK) {
		sleep_disable();
		/* back up our time stamp by 60 microsecs */
		rx_last_time -= F_CPU/1000000*60/64;
		if (!(curr & 1) || delta != 255) {
			/* should happen only from a long stop bit */
			_rx_dbg_ln("bad soft_rx sync after wakeup");
		} else if (signal) {
			/*
			 * Start (low) bit was probably missed. We can't be
			 * sure for how long the signal has been back high.
			 * Let's presume this just happened and fake the
			 * missing start bit.
			 */
			rx_delta[curr] =255;
			rx_head = next;
			curr = next;
			next++;
			delta = RX_BIT_DURATION;
			signal = 0;
			goto next;
		}
	}
#endif

	/* 
	 * The LSB of the index corresponds to the signal level.
	 * The transition we just got marks the end of the period
	 * for which we're about to record the duration. Therefore
	 * current signal should be opposite to the index LSB.
	 */
	if ((curr & 1) == signal) {
		_rx_err(irq_sync);
		return;
	}

	rx_delta[curr] = delta;
	rx_head = next;
}

bool soft_rx_empty(void)
{
	return rx_head == rx_tail;
}

int soft_rx_get(void)
{
	uint8_t curr = rx_tail;
	uint8_t last = rx_head;

	if (curr == last) return SOFT_RX_EMPTY;

	_rx_dbg_v((curr & 1) ? " > +" : " > -", rx_delta[curr]);

	/* move away from stop bit */
	curr++;
	/* we need at least one start bit */
	if (curr == last)
		return SOFT_RX_INCOMPLETE;

	uint8_t data = 0;
	uint8_t bits = 8;

	/*
	 * curr is on the start bit.  curr_bit_time should start with
	 * RX_BIT_DURATION/2 but we offset both curr_bit_time and
	 * level_duration by -RX_BIT_DURATION/2 instead to avoid possible
	 * overflow with curr_bit_time if all bits are low.
	 */
	uint8_t curr_bit_time = 0;
	uint8_t level_duration = rx_delta[curr];
	_rx_dbg_v((curr & 1) ? " +" : " -", level_duration);
	if (level_duration < RX_BIT_DURATION/2) {
		/* bad interrupt timing? */
		_rx_err(irq_time);
		/* make it at least one bit */
		level_duration = RX_BIT_DURATION;
	}
	level_duration -= RX_BIT_DURATION/2;

	do {
		curr_bit_time += RX_BIT_DURATION;
		if (curr_bit_time > level_duration) {
			curr_bit_time = 0;
			if (++curr == rx_head) break;
			level_duration = rx_delta[curr];
			_rx_dbg_v((curr & 1) ? " +" : " -", level_duration);
			if (level_duration < RX_BIT_DURATION/2) {
				_rx_err(irq_time);
				level_duration = RX_BIT_DURATION;
			}
			level_duration -= RX_BIT_DURATION/2;
		}
		/* record current data bit */
		data >>= 1;
		if (curr & 1) data |= 0x80;
		_rx_dbg_v(" ", curr & 1);
	} while (--bits);

	if (bits) {
		/* If low period then stop bit not received yet. */
		if (!(curr & 1))
			return SOFT_RX_INCOMPLETE;
		/* Otherwise that might be an on-going high period. */
		cli();
		if (curr != rx_head) {
			/* head moved, try again */
			sei();
			return soft_rx_get();
		}
		/* Otherwise get duration since last transition. */
		level_duration = get_delta(0);
		sei();
		_rx_dbg_v(" +*", level_duration);
		/* Maybe the whole byte has not been received yet? */
		if (level_duration < bits * RX_BIT_DURATION + RX_BIT_DURATION/2)
			return SOFT_RX_INCOMPLETE;
		/*
		 * This is an unterminated high period that covers
		 * all the remaining bits, including the stop bit.
		 */
		do {
			data >>= 1;
			data |= 0x80;
			_rx_dbg_v(" ", curr & 1);
		} while (--bits);
	}

	curr_bit_time += RX_BIT_DURATION;
	if (curr_bit_time > level_duration) {
		/*
		 * We don't know the next period duration but we know
		 * the current one is over.
		 */
		curr++;
		_rx_dbg((curr & 1) ? " +?" : " -?");
	}

	if (curr & 1) {
		/* We have a stop bit, all is good! */
		rx_tail = curr;
		_rx_dbg_v_ln(" OK ", data);
		return data; 
	}

	/*
	 * Something unexpected happened. Let's move to the next high period
	 * hoping that we'll resync with a real stop bit eventually.
	 */
	rx_tail = curr + 1;

	/*
	 * If the line was stuck low for a long period, this is
	 * actually a break condition.
	 */
	if (data == 0) {
		_rx_dbg_ln(" BRK");
		return SOFT_RX_BREAK;
	}

	_rx_err(stop_sync);
	return SOFT_RX_ERROR;
}

void soft_rx_start(void)
{
	uint8_t port = digitalPinToPort(RX_PIN);
	rx_port = portInputRegister(port);
	rx_mask = digitalPinToBitMask(RX_PIN);
	pinMode(RX_PIN, INPUT);
	digitalWrite(RX_PIN, RX_INVERTED ? LOW : HIGH);

	_rx_dbg_v_ln("RX_BAUDRATE ", RX_BAUDRATE);
	_rx_dbg_v_ln("RX_LARGEST_PERIOD_TICKS ", RX_LARGEST_PERIOD_TICKS);
	_rx_dbg_v_ln("RX_TICK_UPSCALE ", RX_TICK_UPSCALE);
	_rx_dbg_v_ln("RX_TICK_DOWNSCALE ", RX_TICK_DOWNSCALE);
	_rx_dbg_v_ln("RX_BIT_DURATION ", RX_BIT_DURATION);

	cli();
	*digitalPinToPCICR(RX_PIN) |= _BV(digitalPinToPCICRbit(RX_PIN));
	*digitalPinToPCMSK(RX_PIN) |= _BV(digitalPinToPCMSKbit(RX_PIN));
	rx_head = rx_tail = 1;  /* presume a stop bit */
	rx_last_time = get_tstamp();
	sei();
}

void soft_rx_stop(void)
{
	cli();
	*digitalPinToPCMSK(RX_PIN) &= ~_BV(digitalPinToPCMSKbit(RX_PIN));
	rx_head |= 1;  /* force the end on a high period */
	sei();
}

