mac = '04:A3:16:A6:2F:E0'
uuid = "0000fff1-0000-1000-8000-00805f9b34fb"
# path: /org/bluez/hci0/dev_04_A3_16_A6_2F_E0/service0010/char0011

import pygatt
import pdb
import time
import traceback
from binascii import hexlify
import logging

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)

adapter = pygatt.GATTToolBackend()

def handle_data(handle, value):
    """
    handle -- integer, characteristic read handle the data was received on
    value -- bytearray, the data returned in the notification
    """
    print("Received data: %s" % hexlify(value))

try:
    adapter.start()
    device = adapter.connect(mac)

    device.subscribe(uuid,
                     callback=handle_data)
    for i in range(10):
        device.char_write(uuid, bytearray("A1C10000208095B5B64B1E1567160B13FEFF000000000000000000000000000000000000C80000000000000000E5D7".decode("hex")))
        time.sleep(0.5)
        print i
    time.sleep(100)
    print `device.char_read(uuid)`
    pdb.set_trace()
except:
    traceback.print_exc()
finally:
    adapter.stop()
