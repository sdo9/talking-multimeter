# Talking multimeter (v2)

Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Thanks to Nicolas Pitre <nico@fluxnic.net> for contributions.

I made a talking module for a UT61E digital multimeter. These multimeters have
an IR output meant for logging to a computer; instead I made a module
that speaks out the multimeter's setting and measurements, so it can
be used by a blind person.

This runs as an arduino sketch on a bare ATMEGA328P microcontroller.
The serial signal is received from the multimeter through an IR diode.
The Cyrustek ES519XX protocol is then decoded. Speech is produced by
playing back pre-generated sound clips, read from an SPI flash chip (one
of the few in DIP through-hole form factor). AVR PWM register twiddling
is used for sound output.

The exact multimeter model I used:

RioRand UT61E AC/DC Modern Digital Auto Ranging Multimeters  Multitester True RMS
https://www.amazon.ca/gp/product/B007THZMWI

## Brief history

In 2014-2015 we made an earlier version of this project, using an
Arduino Uno R3 and an Adafruit Wave Shield Kit. We used a voltage
booster to run it at 5V from 3 batteries. The second version
documented here is made with a bare ATMEGA328P running at 3.3V, rather
than an Arduino. It produces the audio directly (with its PWM
function), reading from a flash chip. The sound level however is too
low in practice and an amplifier was needed, I resorted to a pre-made
breakout board for that.

This circuit is quite a bit more complicated than using the Wave shield.
The audio quality out of an ATMEGA328P also isn't all that good but
perfectly sufficient for the purpose. The main point of this redesign
for me was to make the talking module an always on device: it consumes
less than 10uA in sleep mode, and wakes up as soon as it perceives an IR
signal from the multimeter. This way I don't have to turn the talking
module on or off, which I like a lot. We also came up with a sturdier
and less clunky physical setup where the multimeter won't become
misaligned and made it so I can actually pick it up in one hand and bring
it where I need it.

## Contents

This repository contains:

*   circuit-spec.txt: detailed explanation of the circuit used.
*   Makefile: makefile to obtain files, patch, build and upload, assuming
    you use Arturo (aka ano).
*   words-flash-writer/ : program to generate sound data, Arduino sketch and
    python client utility to upload sound data to the flash chip through
    the ATMEGA.
*   patches/ : modifications to existing software that that this project
    relies on.
*   dmm-talking-arduino-sketch: arduino code for the talking module.

## Code Inspiration

My Cyrustek ES519XX protocol parsing code was derived from a few
existing projects, many thanks to those folks:

-   libsigrok project: git://sigrok.org/libsigrok
-   dmm\_es51922.py: https://bitbucket.org/kuzavas/dmm_es51922.git
-   dmmut61e utility by Steffen Vogel.

The code for audio using AVR PWM was cobbled from a couple code
examples found on the net (one for AtTiny85 as I recall) but I've lost
the references.

Thanks to Adafruit for the code to access the flash chip.

Thanks to Nicolas Pitre <nico@fluxnic.net> for major contributions:

-   Completely rewritten software serial receive function with no busy waiting
    in the ISR.
-   Improved sound playback.
-   ADPCM speech compression.
-   Makefile
-   French/multilingual language support.

## Software Dependencies

_Python_ is required for the program that generates sound files and the
one that uploads them to be written into the flash. The latter also
requires _pyserial_.

For speech generation, I use _espeak_, you'll want to install that package.
I'm used to it and consider it very intelligible even in noisy environments.
It is however far from natural sounding, and may sound strange the first
time you hear it.

Sound data generation also relies on _sox_, the audio processing tool,
install that too.

Last tested on Arduino software version 1.6.9.

The _avrdude_ tool is used to prepare the ATMEGA328P chip, but that
should come as a dependency of the Arduino tools.

Personally, I absolutely want a command-line interface to my development
tools. I used to use "ino", now continued as _Arturo_ (command called `ano`):
https://github.com/scottdarch/Arturo
The Makefile assumes this is installed.
I recommend installing the _picocom_ package which its "serial" function uses.

The board model and cpu setting you want when using ano with a bare
ATMEGA328P at 8MHz is: `-m pro --cpu 8MHzatmega328`.

We use Adafruit's TinyFlash library,
https://github.com/adafruit/Adafruit_TinyFlash
The Makefile will download and patch it automatically.

## Bare ATMEGA328P Preparation

When starting from a brand new ATMEGA328P chip, or if reusing one from
an Arduino Uno R3, the fuses must be reprogrammed. You need an AVR
programmer to do this. I used Adafruit's USBtinyISP.

-   Set it to run at 8MHz using the internal oscillator, so turn off `div8`.
-   Set brown out detector (BOD) to 1.8V.

    avrdude -c usbtiny -p atmega328p \
      -U lfuse:w:0xe2:m -U hfuse:w:0xDA:m -U efuse:w:0xFE:m

(We want *efuse* = 6 but it sometimes insists on having the high bits set.)

I find it convenient for development to install a bootloader, that way
I can upload a new sketch from the FTDI header I incorporated into my
circuit.

    cd /usr/share/arduino/hardware/arduino/avr/bootloaders/
    avrdude -c usbtiny -p atmega328p \
      -U flash:w:./atmega/ATmegaBOOT_168_atmega328_pro_8MHz.hex:i

## Makefile targets

*   `make writer`: Build the audio flash writer.
*   `make dmm`: Build the main DMM talking module.
*   `make upload_dmm`: Build and upload the DMM talking module.
*   `make upload_words`: Build and upload the audio flash writer, then upload the audio data.
*   `make everything`: Build and upload both the audio data and then the DMM talking module.
*   `make watch`: Watch serial output.
*   `make clean`: Remove the build, except for downloaded libraries.
*   `make distclean`: Delete the entire build tree.

No need to read further if all you want is to build and install the code.

## Driver for the flash chip

Flash chips I tried: Winbond 25Q80BV or MX25L4006EPI-12G.

The flash chips driver is needed by both the flash writer sketch and
DMM talking module sketch.

Grab a copy of Adafruit's TinyFlash library:

    git clone https://github.com/adafruit/Adafruit_TinyFlash

Apply my patch: patches/tiny-flash.diff:

-   Adds support for the 512KiB version of the flash chip.
-   Adds the POWERDOWN and RELEASEPOWERDOWN commands.

TODO: I need to send those up to Adafruit.

Copy the resulting folder into the library folder of your arduino development
environment.

## Generating Sound Data

This is all accomplished by the "upload_words" Makefile target but 
here's an explanation.

Under words-flash-writer/ you'll find:

-   list-en: List of words to synthesize, English version.
-   list-fr: List of words to synthesize, French version.

See the "Word List Definitions" section below for details.

The make.py script reads in this list and outputs two files:

-   words_def.h: include file, with #defines for each word key indicating the
    word's code (index number).
    This file is to be copied to the DMM talking module sketch directory.
-   snd.data: a data file to be uploaded onto the flash chip.
    The data file contains a table indexed by word number and giving the address
    of each word's sound data, followed by sound data for all words.

Compile flash-writer-arduino-sketch and upload onto the ATMEGA.

Run sender.py: it transmits snd.data to the ATMEGA, which in turn writes it
to the flash chip.

## Word List Definitions

Definitions for generating English or French words is provided in
list-en and list-fr under words-flash-writer/. The one to use is
specified at the top of the Makefile. Those list files contain:

-   One word per line.
-   The first field is the key as used in the code.
-   Optionally, it may be followed by a colon and the text to feed
    to the text-to-speech engine. If not specified then the same text as
    the key is used.
-   Two special values are also recognized:
    -   silence=<duration> for a silence of given duration.
    -   alias=<key> to produce another key with the same audio content
        as another key without duplicating the actual data in flash memory.

Some keys are optional but their presence influence how numbers are
pronounced to cope with different language peculiarities:

1.   Literal numbers are pronounced as is. From 0 to 19 must be provided and in
     ascending order.
2.   Multiples of ten i.e. 20, 30 40, 50, 60, 70, 80, 90 are pronounced
     as is. By default they are also combined with numbers from 1 to 9
     for values in between, e.g. 43 will concatenate "40" and "3".
3.   If provided, 21, 31, 41, 51, 61, 71, 81 and 91 will be pronounced
     as is and override rule #2 above. This is used when e.g. 21 is not
     appropriately pronounced by concatenating "20" and "1". This is the
     case in French.
4.   If provided, the following keys represents special range:
     -   `60_79`: overrides rule #2 by concatenating this key (normally defined
         with the text "60") and a number between 1 and 19.
     -   `80_99`: overrides rule #2 by concatenating this key (normally defined
         with the text "80") and a number between 1 and 19.

     For example: 74 would concatenate `60_79` (defined as "60") and 14.
    This is how those ranges are pronounced in French, except for Belgian
    and Swiss French where rule #2 should apply without this.
5.   Another optional range, `22_29`, is provided because the pronunciation of
     "20" is slightly different when followed by another number. Works just
     like #4.
6.   The key "hundred" must be provided. It is concatenated to the whole number
     of hundreds in a value. Example: 300 concatenates "3" and "hundred".
7.   If provided the key "100" allows for a literal pronounciation of one
     hundred where rule #6 doesn't produce proper result. For example, in
     French the "one" is not pronounced when There is a single hundred
     therefore this is defined as an alias of "hundred".
8.   The mandatory "thousand" and optional "1000" keys follow the same
     rules as #6 and #7 respectively.
