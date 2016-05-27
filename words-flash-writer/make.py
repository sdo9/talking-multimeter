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

def main():
  entries = []
  for line in file('list').read().strip().split('\n'):
    if ':' in line:
      var, text = line.split(':', 1)
    else:
      var = text = line
    entries += [(var, text)]
  defs = []
  cnt = 1  # #defines are 1-based to allow 0 as a terminator.
  inx = [chr(0xFE), chr(0xEE), chr(0xED)]  # magic header.
  start_addr = 3 + (len(entries) +1)*3
  addr = start_addr
  data = ''
  SAMPLING_RATE = str(16000)
  for var, text in entries:
    if text.startswith('silence='):
      dur = float(text[len('silence='):])
      subprocess.call([
        'sox', '-n', 'out.wav', 'synth', str(dur), 'sin', '0'])
    else:
      subprocess.call([
        'espeak',
        '-s', '250',
        '-z',
        '-w', 'out.wav',
        text])
    subprocess.call([
      'sox',
      '-D',
      'out.wav',
      '-r', SAMPLING_RATE,
      '-e', 'unsigned-integer',
      '-b', '8',
      '-c', '1',
      'snd.raw',
      'vol', '1.2'])
    snd_data = file('snd.raw').read()
    os.unlink('out.wav')
    os.unlink('snd.raw')
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
  with file('snd.data', 'w') as f:
    f.write(''.join(inx))
    f.write(data)

if __name__ == "__main__":
  main()
