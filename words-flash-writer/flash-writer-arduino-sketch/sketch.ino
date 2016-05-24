/*
 * Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Winbond SPI flash loader. Simplistic.
 *
 * TODO: Sometimes the writing appears to proceed fine but the chip does not
 * initially get erased. Dunno why. Should verify after write.
 */

#define FLASH_CS_PIN 7

#include <Adafruit_TinyFlash.h>
#include <SPI.h>

Adafruit_TinyFlash flash(FLASH_CS_PIN);

void error(char *msg)
{
  Serial.print(F("e"));
  Serial.println(msg);
  while(1);
}

uint32_t capacity;

void setup() {
  Serial.begin(38400);
  Serial.println(F("flasher1"));

  pinMode(FLASH_CS_PIN, OUTPUT);
  digitalWrite(FLASH_CS_PIN, HIGH);
  capacity = flash.begin();
  if(!capacity) {
    flash.releasePowerDown();
    capacity = flash.begin();
  }
  while(!Serial.find("P")); // Wait for P command from host
  if(!capacity) {
    error("flash");
  }
  Serial.print(F("OK "));
  Serial.println(capacity);

  while(!Serial.find("ERASE")); // Wait for ERASE command from host
  if(!flash.eraseChip()) {
    error("erase");
  }
  Serial.println(F("OK"));
}

void loop()
{
  while(!Serial.find("W"));
  // 'W' followed by 3bytes for address MSB, then 256bytes data, then 1byte checksum.
  uint8_t buf[3+256+1];
  uint8_t csum = 0;
  unsigned long start_time = millis();
  for (int i = 0; i < 3+256+1; i++) {
    while (!Serial.available()) {
      if (millis() - start_time > 1000) {
        error("short");
      }
    }
    buf[i] = Serial.read();
    csum += buf[i];
  }

  if (csum != 0) {
    error("csum");
  }

  uint32_t address = (((uint32_t)buf[0] << 16) 
      | ((uint32_t)buf[1] << 8)
      | ((uint32_t)buf[2]));
  if (address+256 > capacity) {
    error("addr");
  }

  if(!flash.writePage(address, buf+3)) {
    error("write");
  }
  Serial.print(F("OK "));
  Serial.println(address);
}
