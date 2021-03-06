/*
 * Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Cyrustek ES519XX protocol parser.
 *
 * Code inspiration:
 * -libsigrok project: git://sigrok.org/lib
 * -dmm_es51922.py: https://bitbucket.org/k
 * -dmmut61e utility by Steffen Vogel.
 * Many thanks.
 */

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "talking.h"
#include "es519xx.h"

#define dbg(msg, ...)

// Error codes:
// -2 Invalid function byte
// -3 Invalid range index byte
// -4 Invalid range entry.
// -5 Invalid digit
// -6 Did not find CRLF.
// -7 Overflowed utter_buffer utterances array.

static long abs(long a) {
  if (a<0) return -a;
  return a;
}

struct range_spec {
  const int8_t n_decimals;
  const utter_t scale;  // f.e. WORD_milli
  const utter_t unit;  // f.e. WORD_volts
};

struct function_spec {
  const uint8_t func_code;
  const utter_t name;
  const struct range_spec ranges[8];
};

static const struct function_spec functions[] = {
#define NONE {-1, 0, 0}
#define FN_VOLTAGE 0b0111011
  {FN_VOLTAGE, WORD_voltage,
   {{4, NO_WORD, WORD_volts}, {3, NO_WORD, WORD_volts},
    {2, NO_WORD, WORD_volts}, {1, NO_WORD, WORD_volts},
    {2, WORD_milli, WORD_volts}, NONE,
    NONE, NONE}},
#define FN_CURRENT_UA 0b0111101
  {FN_CURRENT_UA, WORD_current, // AUTO_UA
   {{2, WORD_micro, WORD_amperes}, {1, WORD_micro, WORD_amperes},
    NONE, NONE, NONE, NONE, NONE, NONE}},
#define FN_CURRENT_MA 0b0111111
  {FN_CURRENT_MA, WORD_current, // AUTO_MA
   {{3, WORD_milli, WORD_amperes}, {2, WORD_milli, WORD_amperes},
    NONE, NONE, NONE, NONE, NONE, NONE}},
#define FN_CURRENT_22A 0b0110000
  {FN_CURRENT_22A, WORD_current, // 22A
   {{3, NO_WORD, WORD_amperes},
    NONE, NONE, NONE, NONE, NONE, NONE, NONE}},
#if 0  // I cannnot actually trigger this function on my DMM.
  {0b0111001, WORD_current, // manual
   {{4, NO_WORD, WORD_amperes}, {3, NO_WORD, WORD_amperes},
    {2, NO_WORD, WORD_amperes}, {1, NO_WORD, WORD_amperes},
    {0, NO_WORD, WORD_amperes}, NONE,
    NONE, NONE,}},
#endif
#define FN_RESISTANCE 0b0110011
  {FN_RESISTANCE, WORD_resistance,
   {{2, NO_WORD, WORD_ohms}, {4, WORD_kilo, WORD_ohms},
    {3, WORD_kilo, WORD_ohms}, {2, WORD_kilo, WORD_ohms},
    {4, WORD_mega, WORD_ohms}, {3, WORD_mega, WORD_ohms},
    {2, WORD_mega, WORD_ohms}, NONE}},
#define FN_CONTINUITY 0b0110101
  {FN_CONTINUITY, WORD_continuity,
   {{2, NO_WORD, WORD_ohms}, NONE,
   NONE, NONE, NONE, NONE, NONE, NONE}},
#define FN_DIODE 0b0110001
  {FN_DIODE, WORD_diode,
   {{4, NO_WORD, WORD_volts},
    NONE, NONE, NONE, NONE, NONE, NONE, NONE}},
#define FN_FREQUENCY 0b0110010
  {FN_FREQUENCY, WORD_frequency,
   {{2, NO_WORD, WORD_hertz}, {1, NO_WORD, WORD_hertz},
    NONE, {3, WORD_kilo, WORD_hertz},
    {2, WORD_kilo, WORD_hertz}, {4, WORD_mega, WORD_hertz},
    {3, WORD_mega, WORD_hertz}, {2, WORD_mega, WORD_hertz}}},
#define FN_CAPACITANCE 0b0110110
  {FN_CAPACITANCE, WORD_capacitance,
   {{3, WORD_nano, WORD_farads}, {2, WORD_nano, WORD_farads},
    {4, WORD_micro, WORD_farads}, {3, WORD_micro, WORD_farads},
    {2, WORD_micro, WORD_farads}, {4, WORD_milli, WORD_farads},
    {3, WORD_milli, WORD_farads}, {2, WORD_milli, WORD_farads}}},
#undef NONE
};

// Fake function codes for reading.fn_code that let us distinguish the two
// different dial positions for voltage, for which the protocol uses
// the same function codes. They happen to be distinguishible by the
// scale they use, which does not change (even with autorange).
// We just pick arbitrary values with the high bit set, this happens to
// not collide with the protocol function codes.
#define FAKEFN_VOLTAGE_VOLTS 0x81
#define FAKEFN_VOLTAGE_MILLIVOLTS 0x82
// And alternative functions.
#define FAKEFN_FREQUENCY 0x83
#define FAKEFN_DUTY_CYCLE 0x84

static const struct range_spec vahz_frequency_ranges = {
  1, NO_WORD, WORD_hertz};
static const struct range_spec vahz_duty_cycle_range = {
  1, NO_WORD, WORD_percent};

// Options, in order in which we speek them.
// For the first few options, until LAST_INTERRUPTING_OPT, we're willing to
// interrupt an ungoing utterance.
#define OPT_BATT 0
#define OPT_DC 1
#define OPT_AC 2
#define OPT_AUTORANGE 3
		/*
		* Note: HOLD only affects the number displayed on the LCD,
		* but not the value sent via the protocol! It also does not
		* affect the bargraph on the LCD.
		*/
#define OPT_HOLD 4
    // Same comment??
#define OPT_REL 5
#define LAST_INTERRUPTING_OPT OPT_REL
#define OPT_PMAX 6
#define OPT_PMIN 7
#define OPT_OL 8
#define OPT_UL 9
// All these I can't elicit on my meter.
#define OPT_MAX 10
#define OPT_MIN 11
//Also RMR VBAR LPF.
#define N_OPTS 12

static const utter_t _option_utterances[] = {
  WORD_dmm_low_batt, // OPT_BATT
  WORD_dc, // OPT_DC
  WORD_ac, // OPT_AC
  WORD_autorange, // OPT_AUTO
  WORD_hold, // OPT_HOLD
  WORD_relative, // OPT_REL
  WORD_pmax, // OPT_PMAX
  WORD_pmin, // OPT_PMIN
  WORD_overlimit, // OPT_OL
  WORD_underlimit, // OPT_UL
  WORD_max, // OPT_MAX
  WORD_min, // OPT_MIN
};

struct reading_s {
  uint8_t fn_code;  // function_spec func_code or one of the FAKEFN_* values.
  utter_t func_name;
  bool opts[N_OPTS];
  utter_t scale;
  utter_t unit;
  uint32_t int_value;
  int8_t n_decimals;
  bool is_negative;
};

static int8_t parse(
    const uint8_t *buf,
    struct reading_s *reading) {
	/* Status byte */
  bool is_judge = (buf[7] & (1 << 3)) != 0;
  reading->opts[OPT_BATT] = (buf[7] & (1 << 1)) != 0; /* battery low */
  reading->opts[OPT_OL] = (buf[7] & (1 << 0)) != 0; /* input overflow */
  /* Option 1 byte */
  reading->opts[OPT_MAX] = (buf[8] & (1 << 3)) != 0;
  reading->opts[OPT_MIN] = (buf[8] & (1 << 2)) != 0;
  reading->opts[OPT_REL] = (buf[8] & (1 << 1)) != 0;  // relative
  //reading->opts[OPT_RMR] = (buf[8] & (1 << 0)) != 0;

  /* Option 2 byte */
  reading->opts[OPT_UL] = (buf[9] & (1 << 3)) != 0; /* under limit */
  reading->opts[OPT_PMAX] = (buf[9] & (1 << 2)) != 0; /* Max peak value */
  reading->opts[OPT_PMIN] = (buf[9] & (1 << 1)) != 0; /* Min peak value */

  /* Option 3 byte */
  reading->opts[OPT_DC] = (buf[10] & (1 << 3)) != 0;  // DC measurement mode, either voltage or current.
  reading->opts[OPT_AC] = (buf[10] & (1 << 2)) != 0;
  reading->opts[OPT_AUTORANGE] = (buf[10] & (1 << 1)) != 0;  // 1-automatic mode, 0-manual
  bool is_vahz = (buf[10] & (1 << 0)) != 0;

  /* Option 4 byte */
  //reading->opts[OPT_VBAR] = (buf[11] & (1 << 2)) != 0;  // 1-VBAR pin is connected to V-.
  reading->opts[OPT_HOLD] = (buf[11] & (1 << 1)) != 0;
  //reading->opts[OPT_LPF] = (buf[11] & (1 << 0)) != 0;  // low-pass-filter feature is activated.

	/* Function byte */
  uint8_t func_code = buf[6];
  const struct function_spec *fn = NULL;
  uint8_t i;
  for (i=0; i<sizeof(functions)/sizeof(functions[0]); i++) {
    if (func_code == functions[i].func_code) {
      fn = &functions[i];
      break;
    }
  }
  if (fn == NULL) {
    dbg("Invalid function byte: 0x%02x", func_code);
    return -2;
  }
  reading->func_name = fn->name;

  if ((func_code == 0b0111101 || func_code == 0b0111111 /*|| func_code == 0b0110000*/)
      && !reading->opts[OPT_AUTORANGE]) {
    dbg("missing expected is_auto for func_code 0x%02x", func_code);
  }
#if 0  // This function does not seem to be used by my DMM.
  if (func_code == 0b0111001 && reading->opts[OPT_AUTORANGE]) {
    dbg("Unexpected is_auto for manual current");
  }
#endif
  if (reading->opts[OPT_AC] && reading->opts[OPT_DC]) {
    dbg("Both ac and dc asserted");
  }

  const struct range_spec *range;
  if (is_vahz && !is_judge) {
    reading->func_name = WORD_frequency;
    reading->fn_code = FAKEFN_FREQUENCY;
    range = &vahz_frequency_ranges;
  } else if ((is_vahz || func_code == 0b0110010) && is_judge) {
    reading->func_name = WORD_duty_cycle;
    reading->fn_code = FAKEFN_DUTY_CYCLE;
    range = &vahz_duty_cycle_range;
  } else {
    reading->fn_code = 0;  // unset yet.

    int8_t range_idx = buf[0] - '0';
    if (range_idx < 0 || range_idx > 7) {
      dbg("Invalid range index byte: 0x%02x", buf[0]);
      return -3;
    }
    range = &fn->ranges[range_idx];
  }
  if (range->n_decimals < 0) {
    dbg("Invalid range: func 0x%02x range index byte 0x%02x", func_code, buf[0]);
    return -4;
  }
  reading->n_decimals = range->n_decimals;
  reading->scale = range->scale;
  reading->unit = range->unit;
  if (reading->fn_code == 0) {
    if (func_code == FN_VOLTAGE) {
      reading->fn_code = reading->scale == WORD_milli ?
        FAKEFN_VOLTAGE_MILLIVOLTS : FAKEFN_VOLTAGE_VOLTS;
    } else reading->fn_code = func_code;
  }

  if (reading->opts[OPT_UL] || reading->opts[OPT_OL]) {
    reading->int_value = 0;
    reading->n_decimals = 0;
    return 0;
  }

  // Digits: bytes 1-5.
  for (i=0; i<5; i++) {
    if (!isdigit(buf[1+i])) {
      dbg("Invalid digit at index %d value 0x%02x", 1+i, buf[1+i]);
      return -5;
    }
  }

  uint32_t val = 0;
  for (i=0; i<5; i++) {
    val = 10*val + (buf[1+i] - '0');
  }
  reading->int_value = val;
  reading->is_negative = (buf[7] & (1 << 2)) != 0;

  return 0;
}

// The reading comes in as a series of digits (which we've packed into
// an int) and a number of decimals. Return the integer part of the
// reading and fill out an array of word codes corresponding to
// decimal digits.
static uint32_t quantize(
    uint32_t int_base,
    int8_t n_decimals,
    utter_t *decimals)
{
  int8_t i;
  if (n_decimals > 0) {
    for (i=n_decimals - 1; i >= 0; i--) {
      decimals[i] = WORD_0 + (int_base % 10);
      int_base /= 10;
    }
    decimals[n_decimals] = NO_WORD;
  } else {
    decimals[0] = NO_WORD;
    for (i=0; i > n_decimals; i--) {
      int_base *= 10;
    }
  }
  return int_base;
}

static struct reading_s last_reading;
// Last rounded value, if subsequent measurements don't change much then
// we stay silent for a bit.
static int32_t last_rnd_base;
// Time we last spoke.
static unsigned long last_spoke;
// Time we last spoke out the unit and scale (as opposed to just the measurement).
static unsigned long last_spoke_unit;
// In terse mode, if the rounded value hasn't changed significantly,
// we stay silent, up to this interval at which point we do report the
// current measurement.
#define UNCHANGING_REPORT_INTERVAL_MS 6000
// We don't speak measurements when in over limit mode, but then that
// makes us totally silent. Can easily forget to turn off the DMM that
// way. So repeat that state information.
#define OVER_LIMIT_REPEAT_INTERVAL_MS 20000
// We speak only the measurement value unless something else changes,
// except every once in a while we'll remind the user of the unit.
#define REPEAT_UNIT_INTERVAL_MS 45000
// pmin/pmax mode: we get both the pmin and pmax packets, and we can't tell
// which one the user has toggled to. So we'll keep our own toggle, and the
// way you flip it is by exiting pmin/max mode and reentering it.
bool last_pminmax = 0;

int8_t handle_packet(
    const uint8_t *buf,
    bool verbose,
    bool already_talking,
    unsigned long now_millis,
    struct utter_buffer *utterbuf)
{
  struct reading_s reading;
  memset(&reading, 0, sizeof(reading));
  const utter_t decimals[6];

  if (buf[14 - 2] != '\r' || buf[14 - 1] != '\n') {
    dbg("Did not find CRLF");
    return -6;
  }

  int8_t r = parse(buf, &reading);
  if (r != 0) {
    return r;
  }

  bool spoke = 0, spoke_unit = 0;
  if (reading.fn_code != last_reading.fn_code) {
    // Interrupt ourselves if we switched to a different function.
    UTTER(reading.func_name);
    spoke = 1;
  }
  // pmin/max: If this reading isn't a pmin/max but the previous was,
  // then toggle which of the two to speak out next time.
  if (!reading.opts[OPT_PMAX] && !reading.opts[OPT_PMIN]
      && (last_reading.opts[OPT_PMAX] || last_reading.opts[OPT_PMIN])) {
    last_pminmax ^= 1;
  }
  if (reading.opts[OPT_PMAX] && last_pminmax)
    return 0;
  if (reading.opts[OPT_PMIN] && !last_pminmax)
    return 0;
  uint8_t o;
  for (o=0; o<N_OPTS; o++) {
    if (!spoke && o > LAST_INTERRUPTING_OPT && already_talking)
      return 0;  // finish what we were saying, don't interrupt.
    // Reasons not to speak.
    switch (o) {
    case OPT_AUTORANGE:
      if (!reading.opts[o]
          && (reading.fn_code == FAKEFN_VOLTAGE_MILLIVOLTS
              || reading.fn_code == FN_CURRENT_22A
              || reading.fn_code == FN_CONTINUITY
              || reading.fn_code == FN_DIODE
              || reading.fn_code == FAKEFN_DUTY_CYCLE)) {
        // Automatically turns off for these functions, presumably because
        // there's a single range available. Too verbose, don't announce.
        continue;
      }
      // Otherwise, turning off is notable. Reactivation coinciding
      // with a function change should be ignored.
      if (reading.opts[o] && reading.fn_code != last_reading.fn_code)
        continue;
      break;
    };
    // For many options, being off means we just don't mention them.
    if (!reading.opts[o]) {
      switch (o) {
      // For these we do want to know when they toggle off.
      case OPT_AUTORANGE:
      case OPT_HOLD:
      case OPT_REL:
        break;
      default:
        // Skip over any option that is off.
        continue;
      };
    }
    // Generally we speak an option if it changed, but also for these
    // special cases.
    bool do_speak = 0;  // Speak even if option did not change.
    switch (o) {
    case OPT_DC:
    case OPT_AC:
      // Reannounce on function change.
      if (reading.fn_code != last_reading.fn_code)
        do_speak = 1;
      break;
    case OPT_OL:
    case OPT_UL:
      // If we've been OL/UL for long enough, repeat it.
      if (now_millis - last_spoke > OVER_LIMIT_REPEAT_INTERVAL_MS)
        do_speak = 1;
      break;
    // pmin/max is special, keep announcing it.
    case OPT_PMAX:
    case OPT_PMIN:
      spoke = 1;
      break;
    };
    if (reading.opts[o] != last_reading.opts[o] || do_speak) {
      spoke = 1;
      UTTER(_option_utterances[o]);
      if (!reading.opts[o]){
        UTTER(WORD_off);
      }
    }
  }
  if (reading.scale != last_reading.scale
      || reading.unit != last_reading.unit
      || now_millis - last_spoke_unit > 30000
      || spoke) {
    UTTER(reading.scale);
    UTTER(reading.unit);
    spoke_unit = 1;
    spoke = 1;
  }
  if (!reading.opts[OPT_OL] && !reading.opts[OPT_UL]) {
    uint32_t int_value;
    // Round off the value so it's faster to speak.
    int32_t rnd_base;
    int8_t rnd_n_decimals;
    if (reading.int_value >= 10000) {
      rnd_base = (reading.int_value +49) /100 * (reading.is_negative ? -1 : 1);
      rnd_n_decimals = reading.n_decimals -2;
    } else if (reading.int_value >= 1000
               || (reading.int_value >= 100 && reading.n_decimals >= 2)
               || reading.n_decimals >= 3) {
      rnd_base = (reading.int_value +4) /10 * (reading.is_negative ? -1 : 1);
      rnd_n_decimals = reading.n_decimals -1;
    } else {
      rnd_base = reading.int_value * (reading.is_negative ? -1 : 1);
      rnd_n_decimals = reading.n_decimals;
    }
    if (verbose)
      int_value = quantize(reading.int_value, reading.n_decimals, decimals);
    else
      int_value = quantize(abs(rnd_base), rnd_n_decimals, decimals);
    // In terse mode, we wait a bit for the value to change before speaking again.
    if (verbose || spoke
        || now_millis - last_spoke > UNCHANGING_REPORT_INTERVAL_MS
        || abs(rnd_base - last_rnd_base) > 5) {
      spoke = 1;
      last_rnd_base = rnd_base;
      if (reading.is_negative) UTTER(WORD_minus);
      sayNumber(int_value, utterbuf);
      const utter_t *d = decimals;
      if (d[0] != NO_WORD) {
        UTTER(WORD_point);
        do {
          UTTER(*d++);
        } while (*d != NO_WORD);
      }
    }
  }

  if (spoke)
    last_spoke = now_millis;
  if (spoke_unit)
    last_spoke_unit = now_millis;

  memcpy(&last_reading, &reading, sizeof(last_reading));

  if (utterbuf->utter_pos + 1 >= NUM_UTTERS)
    return -7;

  return 0;
}

// Make it speak the full state when turning the DMM back on.
void reset_on_powercycle()
{
  memset(&last_reading, 0, sizeof(last_reading));
  last_rnd_base = 0;
  last_spoke = 0;
}
