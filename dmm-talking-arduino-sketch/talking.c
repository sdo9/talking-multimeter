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
    UTTER(WORD_zero);
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
  if (u) {
    sayNumber(u, utterbuf);
    UTTER(WORD_thousand);
  }
  u = n / 100;
  n = n % 100;
  if (u) {
    sayNumber(u, utterbuf);
    UTTER(WORD_hundred);
  }
  if (n >= 20) {
    UTTER(WORD_twenty + n/10 - 2);
    n = n % 10;
  }
  if (n >= 10) {
    UTTER(WORD_ten + n-10);
    return;
  }
  if (n) {
    UTTER(WORD_zero + n);
  }
}
