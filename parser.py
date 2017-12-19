import json
import hexdump
import struct
import math
import pprint
import sys
from PyCRC.CRC16 import CRC16
from details import parser
if len(sys.argv) == 1:
    print 'Usage: parser.py [packet json file]'
    exit()
f = json.load(open(sys.argv[1], 'rb'))
crc_ta = [0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef]
def crc16(data):
    crc = 0
    for c in data:
        crc &= 0xffff
        da = (crc >> 12) & 0xff
        crc = (crc_ta[(((c & 255) >> 4) & 15) ^ da] ^ ((crc << 4) & 0xffff));
        crc &= 0xffff
        da = (crc >> 12) & 0xff
        crc = (crc_ta[(c & 15) ^ da] ^ ((crc << 4) & 0xffff));
    return crc
prev = ''
prev_rx = ''
KNOWN_CMDS = {
    0: '<general callback>',
    -127: 'calibration magnitude for drone'
}
def signed(x):
    return x - 256 if x & 0x80 else x
def parseDetails(parsePacketBytes):
    info = parser(parsePacketBytes)
    return info['broadcasts']

def parse(packet, is_rx=False):
    if is_rx:
        return
    packet = bytearray(packet)
    if packet[4] in (0x20, 0x41):
        print '%02x' % packet[4]
        print str(packet).encode('hex')
        print ' '.join(str(x) for x in packet)
        print '!'
    else:
        return
    if crc16(packet) != 0:
        print 'Malformed packet:', str(packet).encode('hex')
        return
    cmd = packet[4]
    print '=' * 80
    print ['RX', 'TX'][is_rx == False] + ':',
    print '%-30s(%-3d) | subframe: %02x (%d)' % (KNOWN_CMDS.get(packet[4], '<Unknown command>'), packet[4] - 256, packet[36], signed(packet[36]))
    if is_rx:
        pprint.pprint(parseDetails(packet))
    hexdump.hexdump(str(packet))
for data in f:
    try:
        value = data['_source']['layers']['btatt']['btatt.value']
    except:
        continue
    value = value.replace(':', '').decode('hex')
    op = int(data['_source']['layers']['btatt']['btatt.opcode_tree']['btatt.opcode.method'], 16)
    if op == 0x1b: # rx
        prev_rx += value
        if crc16(bytearray(prev_rx)) == 0:
            parse(prev_rx, True)
            prev_rx = ''
    elif op == 0x16: # tx
        offset = data['_source']['layers']['btatt'].get('btatt.offset')
        offset = int(offset)
        if offset == 0:
            if prev:
                parse(prev, False)
            prev = ''
        prev += value
