/*
 * Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Managing series of words.
 */

#include "talking.h"

void appendUtterance(utter_t utt, struct utter_buffer *utterbuf) {
  if (utt == NO_WORD)
    return;
  if (utterbuf->utter_pos + 1 >= NUM_UTTERS) return; // full.
  utterbuf->utterances[utterbuf->utter_pos++] = utt;
}

void sayNumber(long n, struct utter_buffer *utterbuf) {
  if (n == 0) {
    UTTER(WORD_0);
    return;
  }
  if (n < 0) {
    UTTER(WORD_minus);
    n = -n;
  }
  if (n > 99999) {
    UTTER(WORD_unknown);
    return;
  }

  int u = n / 1000;
  n = n % 1000;
#ifdef WORD_1000
  if (u == 1) {
    UTTER(WORD_1000);
  } else
#endif
  if (u) {
    sayNumber(u, utterbuf);
    UTTER(WORD_thousand);
  }

  u = n / 100;
  n = n % 100;
#ifdef WORD_100
  if (u == 1) {
    UTTER(WORD_100);
  } else
#endif
  if (u) {
    sayNumber(u, utterbuf);
    UTTER(WORD_hundred);
  }

#ifdef WORD_21
  if (n == 21) { UTTER(WORD_21); return; }
#endif
#ifdef WORD_31
  if (n == 31) { UTTER(WORD_31); return; }
#endif
#ifdef WORD_41
  if (n == 41) { UTTER(WORD_41); return; }
#endif
#ifdef WORD_51
  if (n == 51) { UTTER(WORD_51); return; }
#endif
#ifdef WORD_61
  if (n == 61) { UTTER(WORD_61); return; }
#endif
#ifdef WORD_71
  if (n == 71) { UTTER(WORD_71); return; }
#endif
#ifdef WORD_81
  if (n == 81) { UTTER(WORD_81); return; }
#endif
#ifdef WORD_91
  if (n == 91) { UTTER(WORD_91); return; }
#endif

#ifdef WORD_80_99
  if (n >= 80) {
    UTTER(WORD_80_99);
    u = n - 80;
    if (u) UTTER(WORD_0 + u);
    return;
  }
#endif
#ifdef WORD_60_79
  if (n >= 60 && n <= 79) {
    UTTER(WORD_60_79);
    u = n - 60;
    if (u) UTTER(WORD_0 + u);
    return;
  }
#endif
#ifdef WORD_21_29
  if (n >= 21 && n <= 29) {
    UTTER(WORD_21_29);
    UTTER(WORD_0 + n - 20);
    return;
  }
#endif

  if (n >= 20) {
    UTTER(WORD_20 + n/10 - 2);
    n = n % 10;
  }
  if (n) {
    UTTER(WORD_0 + n);
  }
}
