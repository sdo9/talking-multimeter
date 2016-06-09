/*
 * Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Talking multimeter main sketch.
 */

// Pin connections:
#define FLASH_CS_PIN 7
#define PCM_PIN 9  // Actually hardcoded in play().
// Pin 10 seems to be stuck high, not sure why.
#define AMP_ENABLE_PIN 4
#define FET_ENABLE_PIN 6
#define BATT_MEAS_ANALOG_PIN 0
// Moving these around requires picking the right ISR(PCINTX_vect. They
// are chosen so the button and serial rx use different interrupts.
#define BUTTON1_PIN 8
#define DMM_SERIAL_RX_PIN 2

#include <Arduino.h>
#include <avr/sleep.h>

#include <Adafruit_TinyFlash.h>
Adafruit_TinyFlash flash(FLASH_CS_PIN);

// First 3 bytes of flash to validate:
#define FLASH_MAGIC1 0xFE
#define FLASH_MAGIC2 0xEE
#define FLASH_MAGIC3 0xED

#include "soft_rx.h"
#include "es519xx.h"
#include "talking.h"
#include "adpcm.h"

// This used to matter a lot in an earlier version that managed lots of
// strings for filenames on a Wave shield; not so much anymore.
int getFreeRam(void) {
  extern int  __bss_end;
  extern int  *__brkval;
  int free_memory;
  if((int)__brkval == 0) {
    // if no heap use from end of bss section
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  }
  else {
    // use from top of stack to heap
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}

#define INTER_PACKET_TIMEOUT_MS 40
#define BUTTON_BOUNCE_DELAY_MS 10
#define DMM_OFF_TIMEOUT_MS 3500
#define INTER_ANNOUNCEMENT_DELAY_MS 500
// Check battery when we begin talking, on wakeup and again every 2mins of
// continuous awake operation.
#define BATTERY_CHECK_INTERVAL 2L*60*1000
// Speak low battery warning no more often than every 2mins of active
// operation, spanning sleeps.
#define LOW_BATTERY_ANNOUNCE_INTERVAL 2L*60*1000

int8_t mode;
unsigned long last_rcv;
unsigned long last_battery_check, last_low_batt_announce;
unsigned long last_packet;
unsigned long woke_at;
uint8_t buf[15];
int8_t buf_len;

// Two utterance sequence buffers, one being spoken, one being filled out with
// the next upcoming utterance sequence, then we swap them.
struct utter_buffer utterbufs[2];
struct utter_buffer *playing_utterbuf = &utterbufs[0];
struct utter_buffer *new_utterbuf = &utterbufs[1];
// Points to the next utterance to speak in the series.
utter_t *playing_utterances = new_utterbuf->utterances;
unsigned long finished_talking;

bool isPlaying;
// Number of sound samples left in current clip.
volatile unsigned playLen;
// Prefetched audio sample
uint16_t nextSample;
// Next ADPCM nibble: zero if already consumed */
uint8_t nextNibble;

struct ButtonState {
  int8_t pin;
  bool isPressed;
  unsigned long pressTime, releaseTime;
};
volatile struct ButtonState button1_btn;

void setup(){
  Serial.begin(115200);
  Serial.println(F("Hi!"));

  pinMode(PCM_PIN, OUTPUT);
  pinMode(FET_ENABLE_PIN, OUTPUT);
  pinMode(AMP_ENABLE_PIN, OUTPUT);

  pinMode(FLASH_CS_PIN, OUTPUT);
  digitalWrite(FLASH_CS_PIN, HIGH);
  if(!flash.begin()) {
    flash.releasePowerDown();
    if(!flash.begin()) {
      Serial.println(F("flash failed"));
      while(1);
    }
  }
  flash.beginRead(0);
  if (flash.readNextByte() != FLASH_MAGIC1
      || flash.readNextByte() != FLASH_MAGIC2
      || flash.readNextByte() != FLASH_MAGIC3) {
    Serial.println(F("wrong flash"));
    while(1);
  }
  flash.endRead();
  Serial.println(F("Flash OK"));

  pinMode(BUTTON1_PIN, INPUT);
  digitalWrite(BUTTON1_PIN, HIGH);
  memset((void*)&button1_btn, 0, sizeof(button1_btn));
  button1_btn.pin = BUTTON1_PIN;
  setPinChangeInterrupt(BUTTON1_PIN);

  analogReference(INTERNAL);

  soft_rx_start();

  Serial.print(F("RAM ")); Serial.println(getFreeRam());

  uint16_t mv = GetBatteryVoltage();
  Serial.print(F("batt mv ")); Serial.println(mv);

  playSync(WORD_hello);

  last_packet = millis();
  Serial.println(F("go!"));
}

void pinChangeISR()
{
  handleButton(&button1_btn);
}

void setPinChangeInterrupt(uint8_t pin) {
  cli();
  *digitalPinToPCICR(pin) |= _BV(digitalPinToPCICRbit(pin));
  *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
  sei();
}

// Button on pin 8 using PCINT0_vect.
// Serial rx from DMM on pin 2, using PCINT2_vect.
// This way we have one pin per interrupt.
ISR(PCINT0_vect) { pinChangeISR(); }

void handleButton(volatile struct ButtonState* button)
{
  bool isPressed = digitalRead(button->pin) == LOW;
  if (isPressed == button->isPressed)
    return;
  if (isPressed) {
    button->pressTime = millis();
  } else {
    button->releaseTime = millis();
  }
  button->isPressed = isPressed;
}

bool WasButtonPressed(volatile struct ButtonState *button)
{
  if (!button->pressTime)
    return 0;
  cli();
  bool ret = 0;
  if (button->pressTime) {
    if (button->releaseTime > button->pressTime) {
      if (button->releaseTime - button->pressTime > BUTTON_BOUNCE_DELAY_MS) {
        ret = 1;
      }
      button->pressTime = 0;
    } else {
      if (button->pressTime - button->releaseTime < BUTTON_BOUNCE_DELAY_MS) {
        button->pressTime = 0;
      } else if (millis() - button->pressTime > BUTTON_BOUNCE_DELAY_MS) {
        // We could have a long press notion if needed...
        ret = 1;
        button->pressTime = 0;
      }
    }
  }
  sei();
  return ret;
}

void waitForSpeech()
{
  while(isPlaying) {
    playLoop();
  }
}

#define PWM_SCALE (F_CPU / SAMPLING_RATE)

void play(utter_t utt) {
  //Serial.print(F("play: ")); Serial.println(utt);
  if (utt <= 0 || utt >= WORDS_LEN) {
    stopPlay();
    return;
  }
  if (isPlaying) {
    playLen = 0;  // Interrupting: have the ISR stop looking at the flash.
    flash.endRead();
  }
  // utterance numbers are 1 based, but the 3-byte magic number counts as slot 0.
  flash.beginRead(utt * 3);
  uint8_t a = flash.readNextByte();
  uint8_t b = flash.readNextByte();
  uint8_t c = flash.readNextByte();
  uint8_t d = flash.readNextByte();
  uint8_t e = flash.readNextByte();
  uint8_t f = flash.readNextByte();
  flash.endRead();
#if 0
  Serial.print(a); Serial.print(" ");
  Serial.print(b); Serial.print(" ");
  Serial.print(c); Serial.print(" ");
  Serial.print(d); Serial.print(" ");
  Serial.print(e); Serial.print(" ");
  Serial.println(f);
#endif
  uint32_t addr = ((uint32_t)a<<16) | ((uint32_t)b<<8) | (uint32_t)c;
  uint32_t end_addr = ((uint32_t)d<<16) | ((uint32_t)e<<8) | (uint32_t)f;
  unsigned len = end_addr - addr;
  //Serial.print(addr); Serial.print(F(" ")); Serial.println(len);
  flash.beginRead(addr);
  ADPCM_init();
  nextNibble = flash.readNextByte();
  /* first (top) nibble */
  nextSample = ADPCM_decode(nextNibble >> 4);
  /* we consumed top nibble, make sure nextNibble is non zero */
  nextNibble |= 0x80;

  playLen = len;

  if (!isPlaying) {
    isPlaying = 1;
    digitalWrite(AMP_ENABLE_PIN, HIGH);
    noInterrupts();
    ICR1 = PWM_SCALE;
    TCCR1A = _BV(WGM11) | _BV(COM1A1); //WGM11,12,13 all set to 1 = fast PWM/w ICR TOP
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    TIMSK1 = _BV(TOIE1);
    interrupts();
  }
}

void stopPlay() {
  if (!isPlaying) return;
  digitalWrite(AMP_ENABLE_PIN, LOW);
  isPlaying = 0;
  TIMSK1 &= ~_BV(TOIE1);
  TCCR1A = TCCR1B = 0;
  flash.endRead();
  finished_talking = millis();
}

static volatile bool inAudioISR;

ISR(TIMER1_OVF_vect) {
  /* protect ourself against reentrancy */
  if (inAudioISR) return;
  inAudioISR = 1;

  OCR1A = ((uint32_t)nextSample * PWM_SCALE) >> 16;

  /* the code that follows is heavy, be nice to other ISRs */
  sei();

  if (playLen && nextNibble) {
    /* second (bottom) nibble */
    nextSample = ADPCM_decode(nextNibble & 0x0f);
    nextNibble = 0;
  } else if(playLen && --playLen) {
    nextNibble = flash.readNextByte();
    /* first (top) nibble */
    nextSample = ADPCM_decode(nextNibble >> 4);
    /* we consumed top nibble, make sure nextNibble is non zero */
    nextNibble |= 0x80;
  } else {
    nextSample = 32768;
  }

  cli();
  inAudioISR = 0;
}

void playLoop() {
  if (isPlaying && !playLen) {
    if (*playing_utterances) {
      ++playing_utterances;
    }
    if (*playing_utterances) {
      play(*playing_utterances);
    } else {
      stopPlay();
      //Serial.println(F("Done playing"));
    }
  }
}

void doSleep()
{
  Serial.println(F("sleep"));
  delay(50);  // flush serial
  flash.powerDown();
  ADCSRA &= ~_BV(ADEN);
  // There's a pull up on the FET's gate.
  pinMode(FET_ENABLE_PIN, INPUT);
  cli();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();

  cli();
  sleep_disable();
  sei();
  pinMode(FET_ENABLE_PIN, OUTPUT);
  flash.releasePowerDown();
  Serial.println(F("awake"));
}

struct utter_buffer *setupNewUtterance()
{
  memset(new_utterbuf, 0, sizeof(struct utter_buffer));
  return new_utterbuf;
}

void playNewUtterances()
{
  if (new_utterbuf->utterances[0]) {
    playing_utterances = new_utterbuf->utterances;
    struct utter_buffer *tmp = playing_utterbuf;
    playing_utterbuf = new_utterbuf;
    new_utterbuf = tmp;
    play(*playing_utterances);
  }
}

void playSync(utter_t utt) {
  struct utter_buffer *utterbuf = setupNewUtterance();
  UTTER(utt);
  playNewUtterances();
  waitForSpeech();
}

void sayInt(int n) {
  struct utter_buffer *utterbuf = setupNewUtterance();
  sayNumber(n, utterbuf);
  playNewUtterances();
}

uint16_t GetBatteryVoltage()
{
  ADCSRA |= _BV(ADEN);
  delay(10);
  uint16_t b;
  for(uint8_t i = 0; i<10; i++)
    b = analogRead(BATT_MEAS_ANALOG_PIN);
  // Measurement is out of 1024 against a 1.1V reference. Given the voltage
  // divider, the battery voltage should be ~5times the measured value.
  // And we want it in mV.
  // So b/1024*1.1*5*1000 = b*5500 >> 10.
  b = (((uint32_t)b)*5500L) >> 10;
  return b;
}

void SpeakBattLevel()
{
  uint16_t mv = GetBatteryVoltage();
  uint8_t l;
#define NBATT 3
#define LOW_BATT (1100 *NBATT)
  if (mv < LOW_BATT) l = 0;
  else if (mv < 1133 *NBATT) l = 1;
  else if (mv < 1166 *NBATT) l = 2;
  else if (mv < 1200 *NBATT) l = 3;
  else if (mv < 1250 *NBATT) l = 4;
  else if (mv < 1300 *NBATT) l = 5;
  else if (mv < 1460 *NBATT) l = 6;
  // I have a set stable at 4470mV for weeks, possibly level 6 should be <1500...?
  else if (mv < 1700 *NBATT) l = 7;
  else l = 42;

  struct utter_buffer *utterbuf = setupNewUtterance();
  UTTER(WORD_battery_voltage);
  mv = mv / 10 * 10;  // round it a bit, it's verbose and bullshit.
  sayNumber(mv, utterbuf);
  UTTER(WORD_milli);
  UTTER(WORD_volts);
  UTTER(WORD_silence);
  UTTER(WORD_battery_level);
  sayNumber(l, utterbuf);
  UTTER(WORD_out_of_7);
  playNewUtterances();
  waitForSpeech();
}


bool parityCheck(uint8_t *bp) {
  uint8_t b = *bp;
  *bp = b & 0x7F;
  b ^= b >> 4; b ^= b >> 2; b ^= b >> 1; 
  return b & 0x01 == 1;
}

void loop()
{
  int b = soft_rx_get();
  if (b >= 0) {
    unsigned long now = millis();
    if (now - last_rcv > INTER_PACKET_TIMEOUT_MS)
      buf_len = 0;
    last_rcv = now;
    buf[buf_len++] = b;
    if (buf_len == 14 && !soft_rx_empty())
      buf_len = 0;
    if (buf_len == 14) {
      bool ok = 1;
      for (uint8_t i = 0; i < buf_len; i++) {
        if (!(ok = parityCheck(buf+i)))
          break;
      }
      if (!ok) {
        Serial.println(F("parity error"));
        sayError(1);
      } else {
        buf[14] = 0;
        Serial.print(F("buf: ")); Serial.print((char*)buf);
        handle(buf);
        last_packet = millis();
      }
      buf_len = 0;
    }
  }
  playLoop();
  if (isPlaying && (last_battery_check == 0
                    || millis() - last_battery_check > BATTERY_CHECK_INTERVAL)) {
    last_battery_check = millis();
    // While playing so we have the amp's draw.
    uint16_t mv = GetBatteryVoltage();
    Serial.print(F("batt mv ")); Serial.println(mv);
    if (mv < LOW_BATT) {
      if (last_low_batt_announce == 0 || millis() - last_low_batt_announce > LOW_BATTERY_ANNOUNCE_INTERVAL) {
        // This one spans sleeps.
	last_low_batt_announce = millis();
        appendUtterance(WORD_talker_low_batt, playing_utterbuf);
      }
    }
  }
  if (WasButtonPressed(&button1_btn)) {
    // If we got no packets since waking, infer that the button woke us up.
    if (last_packet == woke_at) {
      SpeakBattLevel();
    } else {
      mode ^= 1;
      switch (mode) {
      case 0:
        playSync(WORD_terse_mode); break;
      case 1:
        playSync(WORD_verbose_mode); break;
      };
      delay(INTER_ANNOUNCEMENT_DELAY_MS/2);
    }
  }
  if (!isPlaying
      && millis() - last_packet > DMM_OFF_TIMEOUT_MS) {
    //playSync(WORD_goodbye);
    doSleep();
    woke_at = millis();
    last_packet = millis();
    last_battery_check = 0;
    reset_on_powercycle();
  }
}

void handle(uint8_t *buf) {
  struct utter_buffer *utterbuf = setupNewUtterance();
  unsigned long now = millis();
  bool already_talking = isPlaying || now - finished_talking < INTER_ANNOUNCEMENT_DELAY_MS;
  int8_t r = handle_packet(buf, mode == 1, already_talking, now, utterbuf);
  if (r < 0) {
    Serial.print(F("packet error ")); Serial.println(r);
    sayError(-r);
    return;
  }
  playNewUtterances();
}

void sayError(uint8_t code) {
  playSync(WORD_packet_error);
  if (code < 1 || code > 9) {
    playSync(WORD_packet_error);
    return;
  }
  sayInt(code);
  waitForSpeech();
  delay(400);
}

/*
 * This is pulled by soft_rx when debugging is active,
 * discarded by the linker due to -gc-sections otherwise.
 */
void _dbg_(char *string, bool do_val, long val, bool do_ln)
{
  Serial.print(string);
  if (do_val) Serial.print(val);
  if (do_ln) Serial.println();
}
