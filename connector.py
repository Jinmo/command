import threading
import gatt


class Config(object):
    MAC = '04:A3:16:A6:2F:E0'
    UUID_SERVICE = '0000fff0-0000-1000-8000-00805f9b34fb'
    UUID_CHAR = "0000fff1-0000-1000-8000-00805f9b34fb"


manager = gatt.DeviceManager(adapter_name='hci0')


class AnyDevice(gatt.Device):
    def __init__(self, *args, **kwargs):
        self.drone_information_characteristic = None
        self.callback = kwargs.pop('callback')
        super().__init__(*args, **kwargs)

    def services_resolved(self):
        super().services_resolved()

        drone_information_service = next(
            s for s in self.services
            if s.uuid == Config.UUID_SERVICE)
        drone_information_characteristic = next(
            c for c in drone_information_service.characteristics
            if c.uuid == Config.UUID_CHAR)
        drone_information_characteristic.enable_notifications()

        self.drone_information_characteristic = drone_information_characteristic

    def characteristic_value_updated(self, characteristic, value):
        self.callback(characteristic, value)
        pass

    def write_data(self, data):
        self.drone_information_characteristic.write_value(data)


def init_with_callback(callback):
    device = AnyDevice(mac_address='04:A3:16:A6:2F:E0', manager=manager, callback=callback)
    device.connect()

    t = threading.Thread(target=manager.run)
    t.start()

    return device
