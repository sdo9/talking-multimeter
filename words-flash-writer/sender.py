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
# Sends the file snd.data to the flash-writer sketch running on the ATMEGA.

import serial
import sys
import time

if len(sys.argv) > 1:
  # Serial port to use if first arg is given.
  port = sys.argv[1]
else:
  # Try and use ano's guessing facility for consistency.
  import ano.environment
  e = ano.environment.Environment()
  port = e.guess_serial_port()

s = serial.Serial(
  port=port,
  baudrate=38400,
  bytesize=serial.EIGHTBITS,
  parity=serial.PARITY_NONE,
  )

data = file('snd.data').read()
print 'Original data length: %d' % len(data)
data += '\0' * -(len(data)%-256)
print 'padded to length: %d' % len(data)
assert len(data) % 256 == 0

s.setTimeout(1.5)
line = s.readline().strip()
while line:
  print line
  line = s.readline().strip()
s.write('P')
line = s.readline().strip()
while line.startswith('flasher') or line.startswith('manID'):
  print '>>>', line
  line = s.readline().strip()
print '>>>', line
if not line.startswith('OK '):
  print 'error'
  sys.exit(1)
capacity = int(line.split(' ',1)[1])
print 'Capacity: %d' % capacity

if len(data) > capacity:
  raise ValueError('data too long')

print 'Erasing'
s.setTimeout(6.0)
s.write('ERASE')
line = s.readline().strip()
print '>>>', line
if line != 'OK':
  print 'error'
  sys.exit(1)

def csum(buf):
  sum = 0
  for i in buf:
    sum += ord(i)
  return chr((-sum)&0xff)

for offs in xrange(0, len(data), 256):
  buf = ( chr((offs>>16)&0xFF)
         + chr((offs>>8)&0xFF)
         + chr(offs&0xFF)
          + data[offs:offs+256])
  buf = 'W' + buf + csum(buf)
  s.write(buf)
  line = s.readline().strip()
  print '>>>', line
  if not line.startswith('OK'):
    print 'error'
    sys.exit(1)

print 'Done'
