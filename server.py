from threading import Thread, Event
from flask import Flask, render_template
from flask_sse import sse

import details
import connector
import parser

app = Flask(__name__)
app.config["REDIS_URL"] = "redis://localhost"
app.register_blueprint(sse, url_prefix='/stream')

first = None
second = None

prev_packet = bytes.fromhex(
    'a1c10000208095b5b64b1e1567160b13feff000000000000000000000000000000000000c80000000000000000e5d7')


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
                sse.publish({
                    'action': msg.action,
                    'extras': msg.extras.data if msg.extras else {}
                })


ble = connector.init_with_callback(on_receive)


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


class Packet(object):
    def __init__(self, **kwargs):
        self.buffer = bytearray(b'\x00' * 47)
        self.buffer = bytearray(
            prev_packet)
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
    return ''


@app.route('/api/land')
def land():
    ble.write_data(Packet(command=0x41).dump())
    return ''


if __name__ == '__main__':
    app.jinja_env.auto_reload = True
    app.run(port=5000, threaded=True)
