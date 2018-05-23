import atexit
import math
import time

from threading import Thread, Event
from flask import Flask, render_template, request
from flask_sse import sse

from tool import Tool

import details
import connector
import parser

app = Flask(__name__)
app.config["REDIS_URL"] = "redis://localhost"
app.register_blueprint(sse, url_prefix='/stream')

threads = []

first = None
second = None

prev_packet = bytes.fromhex(
    'a1c10000208095b5b64b1e1567160b13feff000000000000000000000000000000000000c80000000000000000e5d7'
    # Captured packet from device
)


# GPS doesn't work inside, so...
overwrittenLat = 37.5845751
overwrittenLng = 127.0265592

# Current, and destination lat, lng
curLat = destLat = overwrittenLat
curLng = destLng = overwrittenLng

# Speed
curVelocity = float(0.2)

def on_receive(attribute, packet):
    global first, second, prev_packet
    if len(packet) == 20:
        if packet.startswith(b'\xb1\xa1'):
            first = packet
        elif first:
            second = packet
    elif first and second:
        prev_packet = packet = first + second + packet
        data = details.parser(packet)
        broadcasts = data['broadcasts']

        with app.app_context():
            for msg in broadcasts:
                print(msg.action, msg.extras)
                if msg.action == 'android.action.planelatlng' and msg.extras:
                    data = msg.extras.data
                    data['uavlat'] = curLat
                    data['uavlng'] = curLng
                sse.publish({
                    'action': msg.action,
                    'extras': msg.extras.data if msg.extras else {}
                })


ble, t = connector.init_with_callback(on_receive)

threads.append(t)


class TimerThread(Thread):
    def __init__(self, event):
        super().__init__()
        self.stopped = event

    def run(self):
        while not self.stopped.wait(1):
            ble.write_data(Packet(command=0).dump())


stopFlag = Event()
t = TimerThread(stopFlag)
t.start()

threads.append(t)


class Packet(object):
    def __init__(self, **kwargs):
        # Replaced from ALL-ZERO packet to packet sample from device
        # self.buffer = bytearray(b'\x00' * 47)
        self.buffer = bytearray(prev_packet)
        self.buffer[:2] = b'\xa1\xc1'

        for key, value in kwargs.items():
            setattr(self, key, value)
        pass

    @property
    def command(self):
        return self.buffer[4]

    @command.setter
    def command(self, value):
        self.buffer[4] = value

    def adjust_crc(self):
        crc_hash = parser.crc16(self.buffer[:-2])
        crc_value = [crc_hash // 256, crc_hash % 256]
        self.buffer[-2:] = crc_value
        assert parser.crc16(self.buffer) == 0x0000

    def dump(self):
        self.adjust_crc()
        return bytearray(self.buffer)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/takeoff')
def takeoff():
    ble.write_data(Packet(command=0x20).dump())
    return 'OK'


@app.route('/api/land')
def land():
    ble.write_data(Packet(command=0x41).dump())
    return 'OK'


@app.route('/api/follow')
def follow():
    global destLat, destLng, velocity
    lat = request.args.get('lat')
    lng = request.args.get('lng')
    velocity = request.args.get('velocity')
    if lat is None or lng is None:
        return 'lat or lng or velocity not provided. usage: /api/follow?lat=<latitude>&lng=<longitude>&velocity=<velocity>'
    lat = float(lat)
    lng = float(lng)
    velocity = float(velocity)

    p = Packet(command=75)
    sendPacket = p.buffer
    sendPacket[36] = 80
    sendPacket[37] = p.toPointWaypointParameter_number = 1
    sendPacket[38] = p.toPointWaypointParameter_currentIndex = 0
    sendPacket[39] = p.toPointWaypointParameter_nextIndex = 0
    sendPacket[40] = p.toPointWaypointParameter_property = 0
    longitudeInt = int(lng * 1.0E7)
    waypointParameterLngBytes = Tool.intToByteArray_ZSY(longitudeInt)
    sendPacket[41] = waypointParameterLngBytes[3]
    sendPacket[42] = waypointParameterLngBytes[2]
    sendPacket[43] = waypointParameterLngBytes[1]
    sendPacket[44] = waypointParameterLngBytes[0]
    ble.write_data(p.dump())


    sendPacket[36] = 81
    sendPacket[37] = p.toPointWaypointParameter_number = 1
    sendPacket[38] = p.toPointWaypointParameter_currentIndex = 0
    waypointParameterVelocityBytes = Tool.getBytesFromShort(0xffff & int(velocity * 10))
    sendPacket[39] = waypointParameterVelocityBytes[0]
    sendPacket[40] = waypointParameterVelocityBytes[1]
    latitudeInt = int(lat * 1.0E7)
    waypointParameterLatBytes = Tool.intToByteArray_ZSY(latitudeInt)
    sendPacket[41] = waypointParameterLatBytes[3]
    sendPacket[42] = waypointParameterLatBytes[2]
    sendPacket[43] = waypointParameterLatBytes[1]
    sendPacket[44] = waypointParameterLatBytes[0]

    destLat, destLng, curVelocity = lat, lng, velocity

    ble.write_data(p.dump())

    ble.write_data(Packet(command=0x48).dump())
    takeoff()
    return 'OK'


def stop_threads():
    global threads, stopFlag, ble

    stopFlag.set()
    ble.stop()
    for t in threads:
        t.cancel()



def followThread():
    global curLat, curLng, destLat, destLng, curVelocity
    while True:
        # First, get angle between cur <-> destination
        distanceLat, distanceLng = (destLat - curLat), (destLng - curLng)
        angle = math.atan2(distanceLng, distanceLat)
        # Then, calculate how far to move
        goLat, goLng = curVelocity * math.cos(angle), curVelocity * math.sin(angle)
        if distanceLat == 0:
            goLat = 0.0
        if distanceLng == 0:
            goLng = 0.0
        curLat, curLng = goLat + curLat, goLng + curLng
        newDistanceLat, newDistanceLng = (destLat - curLat), (destLng - curLng)
        # Adjust it after move
        if distanceLat * newDistanceLat < 0: # already arrived
            curLat = destLat
        if distanceLng * newDistanceLng < 0: # too
            curLng = destLng
        print(curLat, curLng, goLat, goLng, distanceLat, distanceLng)
        sse.publish({
            'action': 'android.action.planelatlng',
            'extras': {'uavlat': curLat, 'uavlng': curLng}
        })
        time.sleep(0.5)


if __name__ == '__main__':
    atexit.register(stop_threads)
    # followThread works in background
    t = Thread(target=followThread)
    threads.append(t)
    # let's run the web server
    app.jinja_env.auto_reload = True
    app.run(host='0.0.0.0', port=5000, threaded=True)
