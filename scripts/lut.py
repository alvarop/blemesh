#
# Script to generate WS2812 LUT for use w/ SPI peripheraL
#
zero = 0x1C0  
one = 0x1F0

import sys

print (one, '[{:0X}] [{:0X}]'.format(one << 0, one << 9*9))

for byte in range(256):
  sys.stdout.write('{')
  num = 0
  for bit in range(8):
    if (byte >> (7-bit)) & 1:
      num += one << (9 * (7-bit))
    else:
      num += zero << (9 * (7-bit))
  
  for new_byte in range(8,-1,-1):
    sys.stdout.write('0x{:02X}, '.format((num >> new_byte*8)&0xFF))
  sys.stdout.write('},\n')

