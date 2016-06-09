#!/usr/bin/python
# -*- encoding: utf-8
#
# Copyright 2014-2016 by Stéphane Doyon <steph@electrons.space>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# Reads the list of words from "list", produces words_def.h with the
# #defines for the word codes and the snd.data file with the sound
# data. Relies on the espeak text-to-speech engine and the sox audio
# processing tool.

import os
import subprocess

SAMPLING_RATE = 18000  # Reduce if we run out of space.
MIN_FLASH_SIZE = 512*1024

def main():
  entries = []
  for line in file('list').read().strip().split('\n'):
    if ':' in line:
      var, text = line.split(':', 1)
    else:
      var = text = line
    entries += [(var, text)]
  defs = ['#define SAMPLING_RATE %d' % SAMPLING_RATE]
  cnt = 1  # #defines are 1-based to allow 0 as a terminator.
  inx = [chr(0xFE), chr(0xEE), chr(0xED)]  # magic header.
  start_addr = 3 + (len(entries) +1)*3
  addr = start_addr
  data = ''
  for var, text in entries:
    if text.startswith('silence='):
      dur = float(text[len('silence='):])
      subprocess.call([
        'sox', '-n', 'out.wav', 'synth', str(dur), 'sin', '0'])
    else:
      subprocess.call([
        'espeak',
        '-a', '200',
        '-s', '250',
        '-z',
        '-w', 'out.wav',
        text])
    subprocess.call([
      'sox',
      '-G',
      '-D',
      'out.wav',
      '-t', 'ima',
      '-e', 'ima-adpcm',
      '-r', str(SAMPLING_RATE),
      '-c', '1',
      'snd.ima'])
    snd_data = file('snd.ima').read()
    os.unlink('out.wav')
    os.unlink('snd.ima')
    defs += ['#define WORD_%s %d  // %s' % (var, cnt, text)]
    cnt += 1
    inx += [chr(addr>>16)]
    inx += [chr(addr>>8 & 0xFF)]
    inx += [chr(addr &0xFF)]
    addr += len(snd_data)
    data += snd_data
  defs += ['#define WORDS_LEN %d' % cnt]
  inx += [chr(addr>>16)]
  inx += [chr(addr>>8 & 0xFF)]
  inx += [chr(addr &0xFF)]
  assert cnt == len(entries) + 1
  assert len(inx) == start_addr
  assert len(inx) + len(data) == addr
  with file('words_def.h', 'w') as f:
    f.write('\n'.join(defs) + '\n')
  out_data = ''.join(inx) + data
  with file('snd.data', 'w') as f:
    f.write(out_data)
  print '%d words, %.1fKiB of sound data' % (cnt, float(len(out_data)/1024))
  if len(out_data) > MIN_FLASH_SIZE:
    sys.stderr.write('snd.data is too large for %d bytes flash\n'
                     % MIN_FLASH_SIZE)
    sys.exit(1)

if __name__ == "__main__":
  main()
