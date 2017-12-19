#!/usr/bin/env python
""" generated source for module <stdin> """
from abc import ABCMeta, abstractmethod
from tool import Tool
from math import sqrt
from pprint import pformat
# package: com.ceewa.demoforceewauav.service
pending_broadcasts = []
def sendBroadcast(x):
    # print 'sent broadcast', x
    pending_broadcasts.append(x)
def connectBleDevice():
    return 'connecting to ble device'
class Service(object):
    pass
class Container(object):
    pass
class Bundle(Container):
    def __init__(self):
        self.data = {}
    def putByte(self, x, y):
        self.data[x] = y & 0xff
    def putDouble(self, x, y):
        self.data[x] = float(y)
    def putShort(self, x, y):
        self.data[x] = y & 0xffff
    def putInt(self, x, y):
        self.data[x] = y & 0xffffffff
    def putByteArray(self, x, y):
        if y is None:
            if x in self.data:
                del self.data[x]
        else:
            self.data[x] = bytearray(y)
    def __repr__(self):
        return '%r' % (self.data)
    pass
class WayPointParameter(Container):
    pass
class Intent(object):
    def __init__(self):
        self.extras = self.action = None
    def putExtras(self, x):
        self.extras = x
    def setAction(self, x):
        self.action = x
    def __repr__(self):
        return 'Intent<action=%r, extras=%r>' % (self.action, self.extras)
    pass
class File(object):
    registered_path = {}
    def __init__(self, x):
        self.path = x
    def __repr__(self, x):
        return 'File <%r>' % x
    def getPath(self):
        return self.path
    def exists(self):
        return self.path in File.registered_path
    def mkdir(self):
        print 'mkdir(%r)' % self.path
        File.registered_path[self.path] = {
        'type': 'directory'
        }
        return
    def isDirectory(self):
        return File.registered_path[self.path]['type'] == 'directory'
    def getAbsolutePath(self):
        return self.path
    pass
class StringBuilder(object):
    def __init__(self, x=''):
        self.data = [x]
    def append(self, x):
        self.data.append(x)
        return self
    def __str__(self):
        return ''.join(self.data)
    pass
class Environment(object):
    @staticmethod
    def getExternalStorageDirectory():
        return File('<external storage>')
class LocalBinder(object):
    pass
class BluetoothGattCallback(object):
    pass
class Timer(object):
    def cancel(self):
        pass
    pass
class List(object):
    def __init__(self):
        self.data = []
    def clear(self):
        del self.data[:]
class ArrayList(List):
    pass
class Handler(object):
    pass
class LinkedList(List):
    pass
class BroadcastReceiver(object):
    pass
class Thread(object):
    pass
class Binder(object):
    pass
class IntentFilter(object):
    def __init__(self):
        self.actions = []
    def addAction(self, x):
        self.actions.append(x)
def registerReceiver(receiver, filter):
    # print 'registering receiver %r on %r' % (receiver, filter)
    pass
class Application(object):
    def setSaveLogUtil(self, logpath):
        print 'setting logpath: %r' % logpath
        return
    def getSaveLogUtil(self):
        return
    pass
g_app = Application()
def getApplication():
    return g_app
class ScheduledThreadPoolExecutor(object):
    def __init__(self, threads):
        self.threads = threads
    def scheduleAtFixedRate(self, thread, initialDelay, period, unit):
        pass
class TimeUnit(object):
    MILLISECONDS = 0
class bytearray(object):
    def __init__(self, x):
        if all(type(x) is str for x in x):
            x = map(ord, x)
        x = [x if x < 128 else x - 256 for x in x]
        self.data = x
    def __getitem__(self, x):
        return self.data.__getitem__(x)
    def __setitem__(self, x):
        return self.data.__setitem__(x)
# package: com.ceewa.demoforceewauav.entity
class UavInformation(object):
    """ generated source for class UavInformation """
    serialVersionUID = 1
    ahrsstate = int()
    altitude = float()
    cruise_aircrafr_status = int()
    cruise_heading_mode = int()
    cruise_sub_mode = int()
    current = int()
    flight_mode = int()
    follow_coordinate = int()
    follow_cruise_sub_mode = int()
    follow_heading_mode = int()
    gimbal_control_mode = int()
    gps_hold_heading_mode = int()
    gpssvs = int()
    heading = float()
    home_altitude = float()
    home_latitude = float()
    home_longitude = float()
    home_valid = int()
    ioc_mode = int()
    latitude = float()
    led_failsafe = int()
    led_forbid_alarm = int()
    led_ioc_record = int()
    led_vpowerstatus = int()
    life_time = int()
    longitude = float()
    magerror = int()
    motor_status = int()
    pitch = float()
    roll = float()
    rpm = int()
    ve = float()
    vn = float()
    vpower = int()
    vu = float()

class TrackFollowWaypointParameter(object):

    def createFromParcel(self, source):
        """ generated source for method createFromParcel """
        track = TrackFollowWaypointParameter()
        track.number = source.readByte()
        track.currentIndex = source.readByte()
        track.nextIndex = source.readByte()
        track.longitude = source.readFloat()
        track.latitude = source.readFloat()
        track.altitude = int(source.readInt())
        return track

    def newArray(self, size):
        """ generated source for method newArray """
        return [None] * size

    altitude = int()
    currentIndex = int()
    latitude = float()
    longitude = float()
    nextIndex = int()
    number = int()

    def describeContents(self):
        """ generated source for method describeContents """
        return 0

    def writeToParcel(self, dest, flags):
        """ generated source for method writeToParcel """
        dest.writeByte(self.number)
        dest.writeByte(self.currentIndex)
        dest.writeByte(self.nextIndex)
        dest.writeFloat(self.longitude)
        dest.writeFloat(self.latitude)
        dest.writeInt(self.altitude)

    def clone(self):
        """ generated source for method clone """
        return super(TrackFollowWaypointParameter, self).clone()

class CacheForSelfie(object):
    """ generated source for class CacheForSelfie """
    altitude = float()
    distance = float()
    velocity = int()

    def getDistance(self):
        """ generated source for method getDistance """
        return self.distance

    def setDistance(self, distance):
        """ generated source for method setDistance """
        self.distance = distance

    def getAltitude(self):
        """ generated source for method getAltitude """
        return self.altitude

    def setAltitude(self, altitude):
        """ generated source for method setAltitude """
        self.altitude = altitude

    def getVelocity(self):
        """ generated source for method getVelocity """
        return self.velocity

    def setVelocity(self, velocity):
        """ generated source for method setVelocity """
        self.velocity = velocity

    def clone(self):
        """ generated source for method clone """
        return super(CacheForSelfie, self).clone()

class CacheForRelativePositionFollow(object):
    """ generated source for class CacheForRelativePositionFollow """
    altitude = float()
    latitude = float()
    longitude = float()

    def getLatitude(self):
        """ generated source for method getLatitude """
        return self.latitude

    def setLatitude(self, latitude):
        """ generated source for method setLatitude """
        self.latitude = latitude

    def getLongitude(self):
        """ generated source for method getLongitude """
        return self.longitude

    def setLongitude(self, longitude):
        """ generated source for method setLongitude """
        self.longitude = longitude

    def getAltitude(self):
        """ generated source for method getAltitude """
        return self.altitude

    def setAltitude(self, altitude):
        """ generated source for method setAltitude """
        self.altitude = altitude

    def clone(self):
        """ generated source for method clone """
        return super(CacheForRelativePositionFollow, self).clone()

class CacheForCircleFollow(object):
    """ generated source for class CacheForCircleFollow """
    altitude = float()
    radius = int()
    vcircle = int()

    def getAltitude(self):
        """ generated source for method getAltitude """
        return self.altitude

    def setAltitude(self, altitude):
        """ generated source for method setAltitude """
        self.altitude = altitude

    def getVcircle(self):
        """ generated source for method getVcircle """
        return self.vcircle

    def setVcircle(self, vcircle):
        """ generated source for method setVcircle """
        self.vcircle = vcircle

    def getRadius(self):
        """ generated source for method getRadius """
        return self.radius

    def setRadius(self, radius):
        """ generated source for method setRadius """
        self.radius = radius

    def clone(self):
        """ generated source for method clone """
        return super(CacheForCircleFollow, self).clone()

class CacheForAutoFollow(object):
    """ generated source for class CacheForAutoFollow """
    altitude = float()
    latitude = float()
    longitude = float()

    def getLatitude(self):
        """ generated source for method getLatitude """
        return self.latitude

    def setLatitude(self, latitude):
        """ generated source for method setLatitude """
        self.latitude = latitude

    def getLongitude(self):
        """ generated source for method getLongitude """
        return self.longitude

    def setLongitude(self, longitude):
        """ generated source for method setLongitude """
        self.longitude = longitude

    def getAltitude(self):
        """ generated source for method getAltitude """
        return self.altitude

    def setAltitude(self, altitude):
        """ generated source for method setAltitude """
        self.altitude = altitude

    def clone(self):
        """ generated source for method clone """
        return super(CacheForAutoFollow, self).clone()

class MoniteSocketForReceiveService(Service):
    """ generated source for class MoniteSocketForReceiveService """
    PACKAGEHEAD_ONE = int(-79)
    PACKAGEHEAD_TWO = int(-95)
    accelerateForModuleBundle = Bundle()
    accelerateForModuleIntent = Intent()
    accelerateForUavBundle = Bundle()
    accelerateForUavIntent = Intent()
    accelerateValueForModule_X = int()
    accelerateValueForModule_Y = int()
    accelerateValueForModule_Z = int()
    accelerateValueForUav_X = int()
    accelerateValueForUav_Y = int()
    accelerateValueForUav_Z = int()
    acceleratorValueByte = int()
    agilityForDirectionByte = int()
    agilityForFollowByte = int()
    agilityForGimbalByte = int()
    ahrsstateByte = int()
    altitudeDouble = float()
    altitudeLimitShort = int()
    autoFollowWaypointParameterForUpload = WayPointParameter()
    backAltitudeShort = int()
    barometerBundle = Bundle()
    barometerIntent = Intent()
    batteryBundle = Bundle()
    batteryForModuleByte = int()
    batteryIntent = Intent()
    batteryLifeByte = int()
    batteryStateByte = int()
    bleAddressStr = None
    bleConnectStateIntent = Intent()
    bluetoothAdapter = None
    bluetoothDevice = None
    bluetoothGatt = None
    bluetoothManager = None
    bluetoothStateChangeReceiver = None
    breakTypeStr = ""
    breakUploadWaypointReceiver = None
    cacheForAutoFollow = CacheForAutoFollow()
    cacheForCircleFollow = CacheForCircleFollow()
    cacheForRelativePositionFollow = CacheForRelativePositionFollow()
    cacheForSelfie = CacheForSelfie()
    calibBundle = Bundle()
    calibIntent = Intent()
    circleFollowWaypointParameterForUpload = WayPointParameter()
    clearCommandBeforeCalibrateBarometerReceiver = None
    clearCommandBeforeCalibrateGimbalReceiver = None
    clearCommandBeforeCalibrateMagneticOfModuleReceiver = None
    clearCommandBeforeCalibrateMagneticOfUavReceiver = None
    clearCommandBeforeCalibrateSensorOfModuleReceiver = None
    clearCommandBeforeCalibrateSensorOfUavReceiver = None
    clearCommandBeforeCalibrateStickNeutralReceiver = None
    clearCommandIntent = Intent()
    countPacketsThread = None
    countThreadScheduledPool = None
    dataForShowBundle = Bundle()
    dataForShowIntent = Intent()
    deviceTypeByte = int()
    deviceVersionByte = int()
    digitalBundle = Bundle()
    digitalIntent = Intent()
    digitalTransferByte = int()
    disconnectBluetoothReceiver = None
    downloadParamsBytes_10 = [None] * 7
    downloadParamsBytes_17 = [None] * 7
    downloadParamsBytes_21 = [None] * 2
    downloadParamsBytes_24 = [None] * 4
    downloadedWaypointBundle = Bundle()
    downloadedWaypointIntent = Intent()
    downloadingSubframeByteBundle = Bundle()
    downloadingSubframeByteIntent = Intent()
    downloadingWaypointTypeIndex = int()
    downloadingWaypointTypeReceiver = None
    droneVersionByte = int()
    editorForShowFlightFollow = None
    elevatorValueByte = int()
    flapValueByte = int()
    flightMode = int()
    flightModelBundle = Bundle()
    flightModelIntent = Intent()
    followBundle = Bundle()
    followIntent = Intent()
    gimbalStateByte = int()
    gimbalVersionByte = int()
    gpsStateForModuleByte = int()
    gpssvsByte = int()
    gyroscopeForModuleBundle = Bundle()
    gyroscopeForModuleIntent = Intent()
    gyroscopeForUavBundle = Bundle()
    gyroscopeForUavIntent = Intent()
    gyroscopeValueForModule_X = int()
    gyroscopeValueForModule_Y = int()
    gyroscopeValueForModule_Z = int()
    gyroscopeValueForUav_X = int()
    gyroscopeValueForUav_Y = int()
    gyroscopeValueForUav_Z = int()
    homeAltInt = int()
    homeBundle = Bundle()
    homeIntent = Intent()
    homeLatInt = int()
    homeLngInt = int()
    homeValidByte = int()
    isBleFirstConnecting = True
    isBluetoothDeviceChanged = False
    isBreakUploadWaypointReceived = False
    isClearCommandBeforeCalibrateBarometerReceived = False
    isClearCommandBeforeCalibrateGimbalReceived = False
    isClearCommandBeforeCalibrateMagneticOfModuleReceived = False
    isClearCommandBeforeCalibrateMagneticOfUavReceived = False
    isClearCommandBeforeCalibrateSensorOfModuleReceived = False
    isClearCommandBeforeCalibrateSensorOfUavReceived = False
    isClearCommandBeforeCalibrateStickNeutralReceived = False
    isFirstConnectDrone = True
    isFirstListenForMotorRun = True
    isHeadOfPacketFound = False
    isMidPacketFound = False
    isMotorRunCommandReceived = False
    isMotorRunCommandReceivedFirst = True
    isMotorRunTimerCanceled = False
    isResetAllParamsValueReceived = False
    isStartSendBackToUavPositionCommandTwoReceived = False
    isStartSendBackToUavPositionCommandTwoReceivedFirst = True
    landFlagForLowBattery = int()
    landFlagOK = int()
    linkStateForOSDByte = int()
    loseControlByte = int()
    mBinder = LocalBinder()
    mGattCallback = BluetoothGattCallback()

    def onConnectionStateChange(self, gatt, status, newState):
        """ generated source for method onConnectionStateChange """
        if newState == 2:
            print("*************")
            MoniteSocketForReceiveService.self.isBleFirstConnecting = True
            MoniteSocketForReceiveService.self.bleConnectStateIntent.setAction(Tool.BLEISCONNECTED_ACTION)
            MoniteSocketForReceiveService.self.sendBroadcast(MoniteSocketForReceiveService.self.bleConnectStateIntent)
            MoniteSocketForReceiveService.self.bluetoothGatt.discoverServices()
            if MoniteSocketForReceiveService.self.isBluetoothDeviceChanged:
                MoniteSocketForReceiveService.self.isBluetoothDeviceChanged = False
            if MoniteSocketForReceiveService.self.reconnectBleTimer != None:
                MoniteSocketForReceiveService.self.reconnectBleTimer.cancel()
                MoniteSocketForReceiveService.self.reconnectBleTimer.cancel()
                MoniteSocketForReceiveService.self.reconnectBleTask = None
                MoniteSocketForReceiveService.self.reconnectBleTimer = None
            MoniteSocketForReceiveService.self.isHeadOfPacketFound = False
        elif newState == 0:
            print("^^^^^^^^^^")
            MoniteSocketForReceiveService.self.isBleFirstConnecting = True
            MoniteSocketForReceiveService.self.bleConnectStateIntent.setAction(Tool.BLEISDISCONNECTED_ACTION)
            MoniteSocketForReceiveService.self.sendBroadcast(MoniteSocketForReceiveService.self.bleConnectStateIntent)
            MoniteSocketForReceiveService.self.reconnectBle()
        elif newState == 1 and MoniteSocketForReceiveService.self.isBleFirstConnecting:
            MoniteSocketForReceiveService.self.isBleFirstConnecting = False
            MoniteSocketForReceiveService.self.bleConnectStateIntent.setAction(Tool.BLEISCONNECTING_ACTION)
            MoniteSocketForReceiveService.self.sendBroadcast(MoniteSocketForReceiveService.self.bleConnectStateIntent)

    def onServicesDiscovered(self, gatt, status):
        """ generated source for method onServicesDiscovered """
        if status == 0:
            MoniteSocketForReceiveService.self.myHandler.postDelayed(Runnable(), 1000)

    def onCharacteristicRead(self, gatt, characteristic, status):
        """ generated source for method onCharacteristicRead """

    def onCharacteristicWrite(self, gatt, characteristic, status):
        """ generated source for method onCharacteristicWrite """

    def onCharacteristicChanged(self, gatt, characteristic):
        """ generated source for method onCharacteristicChanged """
        tmpPacket = characteristic.getValue()
        MoniteSocketForReceiveService.self.receivePacketForBle(tmpPacket, )

    def onDescriptorRead(self, gatt, descriptor, status):
        """ generated source for method onDescriptorRead """

    def onDescriptorWrite(self, gatt, descriptor, status):
        """ generated source for method onDescriptorWrite """

    def onReliableWriteCompleted(self, gatt, status):
        """ generated source for method onReliableWriteCompleted """

    def onReadRemoteRssi(self, gatt, rssi, status):
        """ generated source for method onReadRemoteRssi """

    magneticErrorByte = int()
    magneticForModuleBundle = Bundle()
    magneticForModuleIntent = Intent()
    magneticForUavBundle = Bundle()
    magneticForUavIntent = Intent()
    magneticValueForModule_X = int()
    magneticValueForModule_Y = int()
    magneticValueForModule_Z = int()
    magneticValueForUav_X = int()
    magneticValueForUav_Y = int()
    magneticValueForUav_Z = int()
    moduleAltDouble = float()
    moduleAltInt = int()
    moduleBundle = Bundle()
    moduleIntent = Intent()
    moduleLatDouble = float()
    moduleLatInt = int()
    moduleLngDouble = float()
    moduleLngInt = int()
    moduleVersionByte = int()
    motorRunByte = int()
    motorRunTimer = Timer()
    multiPointWaypointParameterUploaded = WayPointParameter()
    multiPointWaypointParameterUploadedList = ArrayList()
    myHandler = Handler()
    noWaypointToDownloadIntent = Intent()
    parsereceiveCount = 0
    positionOfUploadParams = 0
    postureBundle = Bundle()
    postureIntent = Intent()
    powerCurrentShort = int()
    radiusLimitShort = int()
    receiveCount = 0
    receivePacketCountBundle = Bundle()
    receivePacketCountIntent = Intent()
    receivePacketExecutorService = None
    receivedHeadOfPacket = []
    receivedMidPacket = []
    reconnectBleTask = None
    reconnectBleTimer = None
    relativePositionFollowWaypointParameterForUpload = WayPointParameter()
    remainPowerShort = int()
    remainTimeShort = int()
    resetAllParamsValueReceiver = None
    rudderValueByte = int()
    satelliteBundle = Bundle()
    satelliteIntent = Intent()
    satelliteNumForModuleByte = int()
    saveLogUtil = None
    sdStateForOSDByte = int()
    selfieWaypointParameterForUpload = WayPointParameter()
    settingBundle = Bundle()
    settingIntent = Intent()
    sharedPreferencesForBleAddress = None
    sharedPreferencesForShowFlightFollow = None
    signalCircleWaypointParameterUploaded = WayPointParameter()
    signalStateForModuleByte = int()
    skillLevelByte = int()
    speedByte = int()
    startMotorCommandReceiver = None
    stickModelByte = int()
    subframeByte = int()
    subframeByteList = LinkedList()
    subframeByteSend = int()
    subframeByteSendReceiver = None
    subframeHeadByteReceived = int(-58)
    subframeHeadByteReceivedBundle = Bundle()
    subframeHeadByteReceivedIntent = Intent()
    subframeIndex = int()
    takeoffFlagOK = int()
    temperatureByte = int()
    timeChargeShort = int()
    toPointWaypointParameterUploaded = WayPointParameter()
    totalCapacityShort = int()
    trackAltForUpload = int()
    trackLatForUpload = float()
    trackLngForUplad = float()
    trackWaypointParameterUploaded = TrackFollowWaypointParameter()
    trackWaypointParameterUploadedList = ArrayList()
    uavAlt = float()
    uavAltInt = int()
    uavBundle = Bundle()
    uavInformation = UavInformation()
    uavIntent = Intent()
    uavLat = float()
    uavLatInt = int()
    uavLng = float()
    uavLngInt = int()
    uavSpeed = float()
    uavVeShort = int()
    uavVnShort = int()
    uavVuShort = int()
    uploadedWaypointBundle = Bundle()
    uploadedWaypointIntent = Intent()
    uploadingSubframeByteBundle = Bundle()
    uploadingSubframeByteIntent = Intent()
    uploadingWaypointTypeIndex = int()
    uploadingWaypointTypeReceiver = None
    versionBundle = None
    versionIntent = None
    vlcApplication = None
    voltageForBatteryOneShort = int()
    voltageForBatteryThreeShort = int()
    voltageForBatteryTwoShort = int()
    voltageShort = int()
    warnForForbidenFlight = int()
    waypointParameterAltBytesForDownload = [None] * 4
    waypointParameterAltBytesForUpload = [None] * 4
    waypointParameterAltForDownload = float()
    waypointParameterAltForUpload = float()
    waypointParameterClimbRateBytesForDownload = [None] * 2
    waypointParameterClimbRateBytesForUpload = [None] * 2
    waypointParameterClimbRateForDownload = int()
    waypointParameterClimbRateForUpload = int()
    waypointParameterDistanceBytesForDownload = [None] * 2
    waypointParameterDistanceBytesForUpload = [None] * 2
    waypointParameterDistanceForDownload = int()
    waypointParameterDistanceForUpload = int()
    waypointParameterDurationBytesForDownload = [None] * 2
    waypointParameterDurationBytesForUpload = [None] * 2
    waypointParameterDurationForDownload = int()
    waypointParameterDurationForUpload = int()
    waypointParameterForDownload = WayPointParameter()
    waypointParameterHeadingBytesForDownload = [None] * 2
    waypointParameterHeadingBytesForUpload = [None] * 2
    waypointParameterHeadingForDownload = int()
    waypointParameterHeadingForUpload = int()
    waypointParameterLatBytesForDownload = [None] * 4
    waypointParameterLatBytesForUpload = [None] * 4
    waypointParameterLatForDownload = float()
    waypointParameterLatForUpload = float()
    waypointParameterLngBytesForDownload = [None] * 4
    waypointParameterLngBytesForUpload = [None] * 4
    waypointParameterLngForDownload = float()
    waypointParameterLngForUpload = float()
    waypointParameterRadiusBytesForDownload = [None] * 2
    waypointParameterRadiusBytesForUpload = [None] * 2
    waypointParameterRadiusForDownload = int()
    waypointParameterRadiusForUpload = int()
    waypointParameterVelocityBytesForDownload = [None] * 2
    waypointParameterVelocityBytesForUpload = [None] * 2
    waypointParameterVelocityForDownload = int()
    waypointParameterVelocityForUpload = int()
    waypointParametervCircleBytesForDownload = [None] * 2
    waypointParametervCircleBytesForUpload = [None] * 2
    waypointParametervCircleForDownload = int()
    waypointParametervCircleForUpload = int()

    class BluetoothStateChangeReceiver(BroadcastReceiver):
        """ generated source for class BluetoothStateChangeReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """

    class BreakUploadWaypointReceiver(BroadcastReceiver):
        """ generated source for class BreakUploadWaypointReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isBreakUploadWaypointReceived = True
            MoniteSocketForReceiveService.self.breakTypeStr = intent.getExtras().getString("breaktype")

    class ClearCommandBeforeCalibrateBarometerReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateBarometerReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateBarometerReceived = True

    class ClearCommandBeforeCalibrateGimbalReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateGimbalReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateGimbalReceived = True

    class ClearCommandBeforeCalibrateMagneticOfModuleReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateMagneticOfModuleReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateMagneticOfModuleReceived = True

    class ClearCommandBeforeCalibrateMagneticOfUavReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateMagneticOfUavReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateMagneticOfUavReceived = True

    class ClearCommandBeforeCalibrateSensorOfModuleReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateSensorOfModuleReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateSensorOfModuleReceived = True

    class ClearCommandBeforeCalibrateSensorOfUavReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateSensorOfUavReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateSensorOfUavReceived = True

    class ClearCommandBeforeCalibrateStickNeutralReceiver(BroadcastReceiver):
        """ generated source for class ClearCommandBeforeCalibrateStickNeutralReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isClearCommandBeforeCalibrateStickNeutralReceived = True

    class CountPacketsThread(Thread):
        """ generated source for class CountPacketsThread """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def run(self):
            """ generated source for method run """
            super(self.__class__, self).run()
            MoniteSocketForReceiveService.self.receivePacketCountBundle.putInt("receivecount", MoniteSocketForReceiveService.self.receiveCount)
            MoniteSocketForReceiveService.self.receivePacketCountBundle.putInt("parse", MoniteSocketForReceiveService.self.parsereceiveCount)
            MoniteSocketForReceiveService.self.receivePacketCountIntent.putExtras(MoniteSocketForReceiveService.self.receivePacketCountBundle)
            MoniteSocketForReceiveService.self.receivePacketCountIntent.setAction(Tool.COUNTPACKETSFORRECTHREAD)
            MoniteSocketForReceiveService.self.sendBroadcast(MoniteSocketForReceiveService.self.receivePacketCountIntent)
            MoniteSocketForReceiveService.self.receiveCount = 0
            MoniteSocketForReceiveService.self.parsereceiveCount = 0

    class DisconnectBluetoothReceiver(BroadcastReceiver):
        """ generated source for class DisconnectBluetoothReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            if MoniteSocketForReceiveService.self.bluetoothGatt != None:
                MoniteSocketForReceiveService.self.bluetoothGatt.disconnect()
                MoniteSocketForReceiveService.self.bluetoothGatt = None
            MoniteSocketForReceiveService.self.connectBleDevice()
            MoniteSocketForReceiveService.self.isBluetoothDeviceChanged = True

    class DownloadingWaypointTypeReceiver(BroadcastReceiver):
        """ generated source for class DownloadingWaypointTypeReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.downloadingWaypointTypeIndex = intent.getExtras().getInt("downloadingwaypointtype")

    class LocalBinder(Binder):
        """ generated source for class LocalBinder """
        def getService(self):
            """ generated source for method getService """
            return MoniteSocketForReceiveService.self

    class OnSubframeByteReceivedListener(object):
        """ generated source for interface OnSubframeByteReceivedListener """
        __metaclass__ = ABCMeta
        @abstractmethod
        def onSubframeByteReceived(self, i, b):
            """ generated source for method onSubframeByteReceived """

    class ResetAllParamsValueReceiver(BroadcastReceiver):
        """ generated source for class ResetAllParamsValueReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isResetAllParamsValueReceived = True
            MoniteSocketForReceiveService.self.positionOfUploadParams = intent.getExtras().getInt("position")

    class StartMotorCommandReceiver(BroadcastReceiver):
        """ generated source for class StartMotorCommandReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.isFirstListenForMotorRun = True
            MoniteSocketForReceiveService.self.isMotorRunTimerCanceled = False
            motorRunTask = TimerTask()
            MoniteSocketForReceiveService.self.motorRunTimer = Timer()
            MoniteSocketForReceiveService.self.motorRunTimer.schedule(motorRunTask, 2000)

    class SubframeByteSendReceiver(BroadcastReceiver):
        """ generated source for class SubframeByteSendReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.subframeByteSend = intent.getExtras().getByte("subframesend")

    class UploadingWaypointTypeReceiver(BroadcastReceiver):
        """ generated source for class UploadingWaypointTypeReceiver """
        def __init__(self):
            """ generated source for method __init__ """
            super(self.__class__, self).__init__()

        def onReceive(self, context, intent):
            """ generated source for method onReceive """
            MoniteSocketForReceiveService.self.uploadingWaypointTypeIndex = intent.getExtras().getInt("uploadingwaypointtypeindex")

    def onCreate(self):
        """ generated source for method onCreate """
        super(MoniteSocketForReceiveService, self).onCreate()

    def onBind(self, intent):
        """ generated source for method onBind """
        if self.bluetoothGatt != None:
            self.bluetoothGatt.disconnect()
        self.subframeByteList.clear()
        connectBleDevice()
        moniteSocketForReceiveService = self
        self.subframeByteSendReceiver = self.SubframeByteSendReceiver()
        subframeByteSendFilter = IntentFilter()
        subframeByteSendFilter.addAction(Tool.SUBFRAMEBYTESEND_ACTION)
        registerReceiver(self.subframeByteSendReceiver, subframeByteSendFilter)
        moniteSocketForReceiveService = self
        self.bluetoothStateChangeReceiver = self.BluetoothStateChangeReceiver()
        bluetoothStateChangeFilter = IntentFilter()
        bluetoothStateChangeFilter.addAction("android.bluetooth.adapter.action.STATE_CHANGED")
        registerReceiver(self.bluetoothStateChangeReceiver, bluetoothStateChangeFilter)
        moniteSocketForReceiveService = self
        self.startMotorCommandReceiver = self.StartMotorCommandReceiver()
        startMotorCommandFilter = IntentFilter()
        startMotorCommandFilter.addAction(Tool.STARTMOTOR_ACTION)
        registerReceiver(self.startMotorCommandReceiver, startMotorCommandFilter)
        moniteSocketForReceiveService = self
        self.downloadingWaypointTypeReceiver = self.DownloadingWaypointTypeReceiver()
        downloadingWaypointTypeFilter = IntentFilter()
        downloadingWaypointTypeFilter.addAction(Tool.DOWNLOADINGWAYPOINTTYPE_ACTION)
        registerReceiver(self.downloadingWaypointTypeReceiver, downloadingWaypointTypeFilter)
        moniteSocketForReceiveService = self
        self.uploadingWaypointTypeReceiver = self.UploadingWaypointTypeReceiver()
        uploadingWaypointTypeFilter = IntentFilter()
        uploadingWaypointTypeFilter.addAction(Tool.UPLOADINGWAYPOINTTYPE_ACTION)
        registerReceiver(self.uploadingWaypointTypeReceiver, uploadingWaypointTypeFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateSensorOfUavReceiver = self.ClearCommandBeforeCalibrateSensorOfUavReceiver()
        clearCommandBeforeCalibrateSensorOfUavFilter = IntentFilter()
        clearCommandBeforeCalibrateSensorOfUavFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATESENSOROFUAV_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateSensorOfUavReceiver, clearCommandBeforeCalibrateSensorOfUavFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateMagneticOfUavReceiver = self.ClearCommandBeforeCalibrateMagneticOfUavReceiver()
        clearCommandBeforeCalibrateMagneticOfUavFilter = IntentFilter()
        clearCommandBeforeCalibrateMagneticOfUavFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATEMAGNETICOFUAV_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateMagneticOfUavReceiver, clearCommandBeforeCalibrateMagneticOfUavFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateSensorOfModuleReceiver = self.ClearCommandBeforeCalibrateSensorOfModuleReceiver()
        clearCommandBeforeCalibrateSensorOfModuleFilter = IntentFilter()
        clearCommandBeforeCalibrateSensorOfModuleFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATESENSOROFMODULE_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateSensorOfModuleReceiver, clearCommandBeforeCalibrateSensorOfModuleFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateMagneticOfModuleReceiver = self.ClearCommandBeforeCalibrateMagneticOfModuleReceiver()
        clearCommandBeforeCalibrateMagneticOfModuleFilter = IntentFilter()
        clearCommandBeforeCalibrateMagneticOfModuleFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATEMAGNETICOFMODULE_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateMagneticOfModuleReceiver, clearCommandBeforeCalibrateMagneticOfModuleFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateGimbalReceiver = self.ClearCommandBeforeCalibrateGimbalReceiver()
        clearCommandBeforeCalibrateGimbalFilter = IntentFilter()
        clearCommandBeforeCalibrateGimbalFilter.addAction(Tool.CLEARCOMMANDFORCALIBRATEGIMBAL_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateGimbalReceiver, clearCommandBeforeCalibrateGimbalFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateBarometerReceiver = self.ClearCommandBeforeCalibrateBarometerReceiver()
        clearCommandBeforeCalibrateBarometerFilter = IntentFilter()
        clearCommandBeforeCalibrateBarometerFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATEBAROMETER_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateBarometerReceiver, clearCommandBeforeCalibrateBarometerFilter)
        moniteSocketForReceiveService = self
        self.clearCommandBeforeCalibrateStickNeutralReceiver = self.ClearCommandBeforeCalibrateStickNeutralReceiver()
        clearCommandBeforeCalibrateStickNeutralFilter = IntentFilter()
        clearCommandBeforeCalibrateStickNeutralFilter.addAction(Tool.CLEARCOMMANDBEFORECALIBRATESTICKNEUTRAL_ACTION)
        registerReceiver(self.clearCommandBeforeCalibrateStickNeutralReceiver, clearCommandBeforeCalibrateStickNeutralFilter)
        moniteSocketForReceiveService = self
        self.disconnectBluetoothReceiver = self.DisconnectBluetoothReceiver()
        disconnectBluetoothFilter = IntentFilter()
        disconnectBluetoothFilter.addAction(Tool.DISCONNECTBLUETOOTH_ACTION)
        registerReceiver(self.disconnectBluetoothReceiver, disconnectBluetoothFilter)
        moniteSocketForReceiveService = self
        self.breakUploadWaypointReceiver = self.BreakUploadWaypointReceiver()
        breakUploadWaypointFilter = IntentFilter()
        breakUploadWaypointFilter.addAction(Tool.BREAKUPLOADWAYPOINT_ACTION)
        registerReceiver(self.breakUploadWaypointReceiver, breakUploadWaypointFilter)
        moniteSocketForReceiveService = self
        self.resetAllParamsValueReceiver = self.ResetAllParamsValueReceiver()
        resetAllParamsValueFilter = IntentFilter()
        resetAllParamsValueFilter.addAction(Tool.RESETALLPARAMS_ACTION)
        registerReceiver(self.resetAllParamsValueReceiver, resetAllParamsValueFilter)
        self.vlcApplication = getApplication()
        self.countPacketsThread = self.CountPacketsThread()
        self.countThreadScheduledPool = ScheduledThreadPoolExecutor(1)
        self.countThreadScheduledPool.scheduleAtFixedRate(self.countPacketsThread, 0, 2000, TimeUnit.MILLISECONDS)
        return self.mBinder

    def onRebind(self, intent):
        """ generated source for method onRebind """
        super(MoniteSocketForReceiveService, self).onRebind(intent)

    def connectBleDevice(self):
        """ generated source for method connectBleDevice """
        self.bluetoothManager = getSystemService("bluetooth")
        self.sharedPreferencesForBleAddress = getSharedPreferences(Tool.SHAREDPERFERENCESFORBLE, 0)
        self.bleAddressStr = self.sharedPreferencesForBleAddress.getString(Tool.ADDRESSOFBLE, "")
        self.bluetoothAdapter = self.bluetoothManager.getAdapter()
        if self.isBluetoothDeviceChanged or self.bluetoothGatt == None:
            self.bluetoothDevice = self.bluetoothAdapter.getRemoteDevice(self.bleAddressStr)
            if self.bluetoothDevice == None:
                return False
            self.bluetoothGatt = self.bluetoothDevice.connectGatt(self, False, self.mGattCallback)
            return True
        elif self.bluetoothGatt.connect():
            return True
        else:
            return False

    def closeBluetoothGatt(self):
        """ generated source for method closeBluetoothGatt """
        if self.bluetoothGatt != None:
            self.bluetoothGatt.close()
            self.bluetoothGatt = None

    def reconnectBle(self):
        """ generated source for method reconnectBle """
        if self.bluetoothAdapter != None and self.bluetoothAdapter.isEnabled() and self.reconnectBleTimer == None:
            self.reconnectBleTask = TimerTask()
            self.reconnectBleTimer = Timer()
            self.reconnectBleTimer.schedule(self.reconnectBleTask, 0, 3000)

    def getSupportedGattServices(self):
        """ generated source for method getSupportedGattServices """
        if self.bluetoothGatt == None:
            return None
        return self.bluetoothGatt.getServices()

    def writeCharacteristic(self, characteristic):
        """ generated source for method writeCharacteristic """
        if self.bluetoothAdapter != None and self.bluetoothGatt != None:
            self.bluetoothGatt.writeCharacteristic(characteristic)

    def readCharacteristic(self, characteristic):
        """ generated source for method readCharacteristic """
        if self.bluetoothAdapter != None and self.bluetoothGatt != None:
            self.bluetoothGatt.readCharacteristic(characteristic)

    def setCharacteristicNotification(self, characteristic, enabled):
        """ generated source for method setCharacteristicNotification """
        if self.bluetoothAdapter != None and self.bluetoothGatt != None:
            self.bluetoothGatt.setCharacteristicNotification(characteristic, enabled)
            clientConfig = characteristic.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
            if enabled:
                if clientConfig != None:
                    clientConfig.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            elif clientConfig != None:
                clientConfig.setValue(BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE)
            if self.bluetoothAdapter != None and self.bluetoothGatt != None and clientConfig != None:
                self.bluetoothGatt.writeDescriptor(clientConfig)

    def onStartCommand(self, intent, flags, startId):
        """ generated source for method onStartCommand """
        return super(MoniteSocketForReceiveService, self).onStartCommand(intent, flags, startId)

    def receivePacketForBle(self, len, tmpPacket):
        """ generated source for method receivePacketForBle """
        if 20 == len:
            if self.PACKAGEHEAD_ONE == tmpPacket[0] and int(-95) == tmpPacket[1]:
                self.receivedHeadOfPacket = tmpPacket
                self.isHeadOfPacketFound = True
            elif self.isHeadOfPacketFound:
                self.receivedMidPacket = tmpPacket
                self.isMidPacketFound = True
            else:
                reset()
        elif self.isHeadOfPacketFound and self.isMidPacketFound:
            receivedPacket = Tool.concatAll(self.receivedHeadOfPacket, self.receivedMidPacket, tmpPacket)
            crcBytes = Tool.getBytesFromChar(Tool.CalculateCrc16(receivedPacket, 47))
            if crcBytes[0] == int(0) and crcBytes[1] == int(0):
                checkReceivedPacket(receivedPacket)
            reset()
        else:
            reset()

    def reset(self):
        """ generated source for method reset """
        self.isHeadOfPacketFound = False
        self.receivedHeadOfPacket = None
        self.isMidPacketFound = False
        self.receivedMidPacket = None

    def checkReceivedPacket(self, checkPacketBytes):
        """ generated source for method checkReceivedPacket """
        crcBytes = Tool.getBytesFromChar(Tool.CalculateCrc16(checkPacketBytes, 47))
        if crcBytes[0] == int(0) and crcBytes[1] == int(0):
            self.receiveCount += 1
            self.subframeByteSend = self.vlcApplication.getSubframeByte()
            if self.subframeByteSend == checkPacketBytes[36]:
                self.parsereceiveCount += 1
                if not (checkPacketBytes[36] == int(-48) or checkPacketBytes[36] == int(-47) or checkPacketBytes[36] == int(-46) or checkPacketBytes[36] == int(-45) or checkPacketBytes[36] == int(-44) or checkPacketBytes[36] == int(-38) or checkPacketBytes[36] == int(-37)):
                    self.vlcApplication.setSubframeFlag(0)
                    self.vlcApplication.setReceivedSubframeByte(checkPacketBytes[36])
                parsePacketData(checkPacketBytes)
                return
            self.vlcApplication.setSubframeFlag(1)
            self.vlcApplication.setReceivedSubframeByte(checkPacketBytes[36])
            return
        self.vlcApplication.setSubframeFlag(1)
        self.vlcApplication.setReceivedSubframeByte(checkPacketBytes[36])

    def parseCommand(self, commandByte):
        """ generated source for method parseCommand """
        if commandByte == int(-127):
            self.calibBundle.putInt("calibmagfordrone", 0)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBMAGFIRDRONE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(-126):
            self.followBundle = Bundle()
            self.followIntent = Intent()
            self.followBundle.putInt("resultofsaveflash", 0)
            self.followBundle.putInt("backtype", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTOFSAVEFLASH_ACTION)
            sendBroadcast(self.followIntent)
            print("&&&")
            return
        elif commandByte == int(-120):
            self.followBundle.putInt("resultforneutralcalibrate", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORNEUTRALPOINTCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-119):
            self.followIntent.setAction(Tool.RESULTFORSTARTCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-118):
            self.followIntent.setAction(Tool.RESULTFORSTOPCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-114):
            self.calibBundle.putInt("calibgyroandaccresultfordrone", 0)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBGYROANDACCFORDRONE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(-113):
            self.calibBundle.putInt("calibgyroandaccresultformodule", 0)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBGYROANDACCFORMODULE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(-104):
            self.calibBundle.putInt("calibmagformodule", 0)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBMAGFORMODULE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(-103):
            self.calibBundle.putInt("calibbarometer", 0)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBBAROMETER_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(-102):
            self.followBundle.putInt("resultforcalibrategimbal", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORCALIBRATEGIMBAL_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-96):
            self.followBundle.putInt("resultformotorrun", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORMOTORRUN_ACTION)
            sendBroadcast(self.followIntent)
            self.isMotorRunCommandReceivedFirst = True
            self.isMotorRunCommandReceived = False
            return
        elif commandByte == int(-93):
            self.followBundle.putInt("resultforstop", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFOREMERGYSTOP_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-87):
            self.followBundle.putInt("resultforsetuavpositionasbackpositioncommandone", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETUAVPOSITIONASBACKPOSITIONCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            self.isStartSendBackToUavPositionCommandTwoReceived = False
            return
        elif commandByte == int(-71):
            self.followBundle.putInt("resultforsettakeoffpositionasbackposition", 0)
            self.followIntent.setAction(Tool.RESULTFORSETTAKEOFFPOSITIONASBACKPOSITION_ACTION)
            self.followIntent.putExtras(self.followBundle)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-70):
            self.followBundle.putInt("resultforsetuavpositionasbackpositioncommandtwo", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETUAVPOSITIONASBACKPOSITIONCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-69):
            self.followBundle.putInt("resultforsetmodulepositionasbackposition", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETMODULEPOSITIONASBACKPOSITION_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-64):
            self.followBundle.putInt("resultfortakeoff", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTAKEOFF_ACTION)
            sendBroadcast(self.followIntent)
            self.isMotorRunCommandReceivedFirst = True
            self.isMotorRunCommandReceived = False
            return
        elif commandByte == int(-63):
            self.followBundle.putInt("resultforland", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORLAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-62):
            self.followBundle.putInt("resultformultipoint", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORMULTIPOINTCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 7)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-60):
            self.followIntent.setAction(Tool.RESULTFORGOHOME_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-58):
            self.followBundle.putInt("resultfortopoint", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTOPOINTCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 6)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-57):
            self.followBundle.putInt("resultforsignalcircle", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSIGNALCIRCLECOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 5)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-56):
            self.followBundle.putInt("resultforupload", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORHOVERANDAIMFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-55):
            self.followBundle.putInt("resultforupload", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORLOOKDOWNFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-54):
            self.followIntent.setAction(Tool.RESULTFORMULTIPOINTCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-53):
            self.followIntent.setAction(Tool.RESULTFORTOPOINTCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-52):
            self.followIntent.setAction(Tool.RESULTFORSIGNALCIRCLECOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-48):
            self.followBundle.putInt("resultforupload", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSIMPLEFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-41):
            self.followBundle.putInt("resultforselfie", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSELFIEFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 3)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-40):
            self.followBundle.putInt("resultforrelativeposition", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORRELATIVEPOSITIONFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 0)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-39):
            self.followBundle.putInt("resultforauto", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORAUTOFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 1)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-38):
            self.followBundle.putInt("resultforcircle", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORCIRCLEFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 2)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-37):
            self.followBundle.putInt("resultfortrack", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTRACKFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 4)
            self.uploadedWaypointBundle.putInt("resultforupload", 0)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(-31):
            self.followIntent.setAction(Tool.RESULTFORSELFIEFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-30):
            self.followIntent.setAction(Tool.RESULTFORRELATIVEPOSITIONFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-29):
            self.followIntent.setAction(Tool.RESULTFORAUTOFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-28):
            self.followIntent.setAction(Tool.RESULTFORCIRCLEFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(-27):
            self.followBundle.putInt("resultfortrackfollowone", 0)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTRACKFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(1):
            self.calibBundle.putInt("calibmagfordrone", 1)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBMAGFIRDRONE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(2):
            self.followBundle = Bundle()
            self.followIntent = Intent()
            self.followBundle.putInt("resultofsaveflash", 1)
            self.followBundle.putInt("backtype", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTOFSAVEFLASH_ACTION)
            sendBroadcast(self.followIntent)
            print("&&&")
            return
        elif commandByte == int(8):
            self.followBundle.putInt("resultforneutralcalibrate", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORNEUTRALPOINTCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(9):
            self.followIntent.setAction(Tool.RESULTFORSTARTCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(10):
            self.followIntent.setAction(Tool.RESULTFORSTOPCALIBRATE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(14):
            self.calibBundle.putInt("calibgyroandaccresultfordrone", 1)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBGYROANDACCFORDRONE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(15):
            self.calibBundle.putInt("calibgyroandaccresultformodule", 1)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBGYROANDACCFORMODULE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(24):
            self.calibBundle.putInt("calibmagformodule", 1)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBMAGFORMODULE_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(25):
            self.calibBundle.putInt("calibbarometer", 1)
            self.calibIntent.putExtras(self.calibBundle)
            self.calibIntent.setAction(Tool.RESULTOFCALIBBAROMETER_ACTION)
            sendBroadcast(self.calibIntent)
            return
        elif commandByte == int(26):
            self.followBundle.putInt("resultforcalibrategimbal", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORCALIBRATEGIMBAL_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(32):
            self.followBundle.putInt("resultformotorrun", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORMOTORRUN_ACTION)
            sendBroadcast(self.followIntent)
            if self.isMotorRunCommandReceivedFirst:
                self.isMotorRunCommandReceivedFirst = False
                self.isMotorRunCommandReceived = True
                return
            return
        elif commandByte == int(35):
            self.followBundle.putInt("resultforstop", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFOREMERGYSTOP_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(41):
            # 41
            self.followBundle.putInt("resultforsetuavpositionasbackpositioncommandone", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETUAVPOSITIONASBACKPOSITIONCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            self.isStartSendBackToUavPositionCommandTwoReceived = True
            self.isStartSendBackToUavPositionCommandTwoReceivedFirst = True
            return
        elif commandByte == int(57):
            self.followBundle.putInt("resultforsettakeoffpositionasbackposition", 1)
            self.followIntent.setAction(Tool.RESULTFORSETTAKEOFFPOSITIONASBACKPOSITION_ACTION)
            self.followIntent.putExtras(self.followBundle)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(58):
            self.followBundle.putInt("resultforsetuavpositionasbackpositioncommandtwo", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETUAVPOSITIONASBACKPOSITIONCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(59):
            self.followBundle.putInt("resultforsetmodulepositionasbackposition", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSETMODULEPOSITIONASBACKPOSITION_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(64):
            self.followBundle.putInt("resultfortakeoff", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTAKEOFF_ACTION)
            sendBroadcast(self.followIntent)
            self.isMotorRunCommandReceivedFirst = True
            self.isMotorRunCommandReceived = False
            return
        elif commandByte == int(65):
            self.followBundle.putInt("resultforland", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORLAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(66):
            self.followBundle.putInt("resultformultipoint", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORMULTIPOINTCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 7)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointBundle.putParcelableArrayList("uploadwaypointparameter", self.multiPointWaypointParameterUploadedList)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(68):
            self.followIntent.setAction(Tool.RESULTFORGOHOME_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(70):
            self.followBundle.putInt("resultfortopoint", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTOPOINTCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 6)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointBundle.putParcelable("uploadwaypointparameter", self.toPointWaypointParameterUploaded)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(71):
            self.followBundle.putInt("resultforsignalcircle", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSIGNALCIRCLECOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 5)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointBundle.putParcelable("uploadwaypointparameter", self.signalCircleWaypointParameterUploaded)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(72):
            # 72
            self.followBundle.putInt("resultforupload", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORHOVERANDAIMFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(73):
            self.followBundle.putInt("resultforupload", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORLOOKDOWNFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(74):
            self.followIntent.setAction(Tool.RESULTFORMULTIPOINTCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(75):
            self.followIntent.setAction(Tool.RESULTFORTOPOINTCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(76):
            # 76
            self.followIntent.setAction(Tool.RESULTFORSIGNALCIRCLECOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(80):
            # 80
            self.followBundle.putInt("resultforupload", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSIMPLEFOLLOWCOMMAND_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(87):
            self.followBundle.putInt("resultforselfie", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORSELFIEFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 3)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(88):
            # 88
            self.followBundle.putInt("resultforrelativeposition", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORRELATIVEPOSITIONFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 0)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(89):
            self.followBundle.putInt("resultforauto", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORAUTOFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 1)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(90):
            # 90
            self.followBundle.putInt("resultforcircle", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORCIRCLEFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 2)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(91):
            self.followBundle.putInt("resultfortrack", 1)
            self.followIntent.putExtras(self.followBundle)
            self.followIntent.setAction(Tool.RESULTFORTRACKFOLLOWCOMMANDTWO_ACTION)
            sendBroadcast(self.followIntent)
            self.uploadedWaypointBundle.putInt("uploadedwaypointmodel", 4)
            self.uploadedWaypointBundle.putInt("resultforupload", 1)
            self.uploadedWaypointBundle.putParcelableArrayList("uploadwaypointparameter", self.trackWaypointParameterUploadedList)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.UPLOADEDALLWAYPOINTS_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            return
        elif commandByte == int(97):
            self.followIntent.setAction(Tool.RESULTFORSELFIEFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(98):
            self.followIntent.setAction(Tool.RESULTFORRELATIVEPOSITIONFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(99):
            self.followIntent.setAction(Tool.RESULTFORAUTOFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(100):
            self.followIntent.setAction(Tool.RESULTFORCIRCLEFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(101):
            # 101
            self.followIntent.setAction(Tool.RESULTFORTRACKFOLLOWCOMMANDONE_ACTION)
            sendBroadcast(self.followIntent)
            return
        elif commandByte == int(127):
            # 127
            self.followIntent.setAction(Tool.WATCHISBUSY_ACTION)
            sendBroadcast(self.followIntent)
            return
        else:
            return

    def parsePacketData(self, parsePacketBytes):
        """ generated source for method parsePacketData """
        self.parseCommand(parsePacketBytes[4])
        if self.isBreakUploadWaypointReceived:
            if self.breakTypeStr != None and (parsePacketBytes[36] == int(80) or parsePacketBytes[36] == int(81) or parsePacketBytes[36] == int(82) or parsePacketBytes[36] == int(83) or parsePacketBytes[36] == int(84)):
                resetSubframeIntent = Intent()
                if self.breakTypeStr == "downloadflightcontroldata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADFLIGHTCONTROLDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloadmoduledata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADMODULEDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloadbatterydata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADBATTERYDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloaduavsensordata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADUAVSENSORDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloadmodulesensordata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADMODULESENSORDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloadbarometerdata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADBAROMETERDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
                if self.breakTypeStr == "downloadgimbaldata":
                    resetSubframeIntent.setAction(Tool.RESETSUBFRAMEAFTERBREAKUPLOADWAYPOINTFORDOWNLOADGIMBALDATA_ACTION)
                    sendBroadcast(resetSubframeIntent)
            self.isBreakUploadWaypointReceived = False
        if parsePacketBytes[4] == int(0):
            if self.isMotorRunCommandReceived:
                self.followIntent.setAction(Tool.STARTSENDTAKEOFFCOMMAND_ACTION)
                sendBroadcast(self.followIntent)
            if self.isClearCommandBeforeCalibrateSensorOfUavReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATESENSOROFUAV_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateSensorOfUavReceived = False
            if self.isClearCommandBeforeCalibrateMagneticOfUavReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATEMAGNETICOFUAV_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateMagneticOfUavReceived = False
            if self.isClearCommandBeforeCalibrateSensorOfModuleReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATESENSOROFMODULE_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateSensorOfModuleReceived = False
            if self.isClearCommandBeforeCalibrateMagneticOfModuleReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATEMAGNETICOFMODULE_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateMagneticOfModuleReceived = False
            if self.isClearCommandBeforeCalibrateGimbalReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATEGIMBAL_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateGimbalReceived = False
            if self.isClearCommandBeforeCalibrateBarometerReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATEBAROMETER_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateBarometerReceived = False
            if self.isClearCommandBeforeCalibrateStickNeutralReceived:
                self.clearCommandIntent.setAction(Tool.COMMANDCLEAREDBEFORECALIBRATESTICKNEUTRAL_ACTION)
                sendBroadcast(self.clearCommandIntent)
                self.isClearCommandBeforeCalibrateStickNeutralReceived = False
            if self.isStartSendBackToUavPositionCommandTwoReceived and self.isStartSendBackToUavPositionCommandTwoReceivedFirst:
                self.followIntent.setAction(Tool.STARTSENDBACKTOUAVPOSITIONCOMMANDTWO_ACTION)
                sendBroadcast(self.followIntent)
                self.isStartSendBackToUavPositionCommandTwoReceivedFirst = False
        self.uavInformation.ahrsstate = int(((parsePacketBytes[5] >> 6) & 3))
        self.ahrsstateByte = self.uavInformation.ahrsstate
        self.uavInformation.gpssvs = int((parsePacketBytes[5] & 63))
        self.gpssvsByte = self.uavInformation.gpssvs
        self.satelliteBundle.putByte("satellitestate", self.ahrsstateByte)
        self.satelliteBundle.putByte("satelliteamount", self.gpssvsByte)
        self.satelliteIntent.putExtras(self.satelliteBundle)
        self.satelliteIntent.setAction(Tool.SATELLITEAMOUNTANDAVAILABLE)
        sendBroadcast(self.satelliteIntent)
        self.uavLngInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[6], parsePacketBytes[7], parsePacketBytes[8], parsePacketBytes[9]]))
        self.uavLng = (float(self.uavLngInt)) / 1.0E7
        self.uavInformation.longitude = self.uavLng
        self.uavLatInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[10], parsePacketBytes[11], parsePacketBytes[12], parsePacketBytes[13]]))
        self.uavLat = (float(self.uavLatInt)) / 1.0E7
        self.uavInformation.latitude = self.uavLat
        self.uavBundle.putDouble("uavlat", self.uavLat)
        self.uavBundle.putDouble("uavlng", self.uavLng)
        self.uavIntent.putExtras(self.uavBundle)
        self.uavIntent.setAction(Tool.PLANELATLNG_ACTION)
        sendBroadcast(self.uavIntent)
        self.uavAltInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[14], parsePacketBytes[15], parsePacketBytes[16], parsePacketBytes[17]]))
        self.altitudeDouble = (float(self.uavAltInt)) / 1000.0
        self.uavInformation.altitude = self.altitudeDouble
        self.uavAlt = self.altitudeDouble
        self.dataForShowBundle.putDouble("uavalt", self.uavAlt)
        self.uavVnShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[18], parsePacketBytes[19]]))
        self.uavInformation.vn = (float(self.uavVnShort)) / 100.0
        self.uavVeShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[20], parsePacketBytes[21]]))
        self.uavInformation.ve = (float(self.uavVeShort)) / 100.0
        self.uavVuShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[22], parsePacketBytes[23]]))
        self.uavInformation.vu = (float(self.uavVuShort)) / 100.0
        self.uavSpeed = sqrt((((float(self.uavVnShort)) / 100.0) * ((float(self.uavVnShort)) / 100.0)) + (((float(self.uavVeShort)) / 100.0) * ((float(self.uavVeShort)) / 100.0)))
        self.dataForShowBundle.putDouble("uavhorizontalspeed", self.uavSpeed)
        self.dataForShowBundle.putDouble("uavverticalspeed", (float(self.uavVuShort)) / 100.0)
        rollBytes = bytearray([parsePacketBytes[28], parsePacketBytes[29]])
        self.uavInformation.roll = (float(Tool.getShortFromBytes(rollBytes))) / 10.0
        pitchBytes = bytearray([parsePacketBytes[30], parsePacketBytes[31]])
        self.uavInformation.pitch = (float(Tool.getShortFromBytes(pitchBytes))) / 10.0
        headingBytes01 = bytearray([parsePacketBytes[32], parsePacketBytes[33]])
        self.uavInformation.heading = (float(Tool.getShortFromBytes(headingBytes01))) / 10.0
        self.postureBundle.putDouble("roll", (float(Tool.getShortFromBytes(rollBytes))) / 10.0)
        self.postureBundle.putDouble("pitch", (float(Tool.getShortFromBytes(pitchBytes))) / 10.0)
        self.postureBundle.putDouble("heading", (float(Tool.getShortFromBytes(headingBytes01))) / 10.0)
        self.postureIntent.putExtras(self.postureBundle)
        self.postureIntent.setAction(Tool.POSTUREVALUEOFDRONE_ACTION)
        sendBroadcast(self.postureIntent)
        self.digitalTransferByte = parsePacketBytes[35]
        self.digitalBundle.putByte("digitaltransferbyte", self.digitalTransferByte)
        self.digitalIntent.setAction(Tool.SENDDIGITALSIGNAL_ACTION)
        self.digitalIntent.putExtras(self.digitalBundle)
        sendBroadcast(self.digitalIntent)
        if self.digitalTransferByte != int(0):
            if self.isFirstConnectDrone:
                self.isFirstConnectDrone = False
                ceewaFile = File(StringBuilder(str(Environment.getExternalStorageDirectory().getPath())).append("/TAROT").__str__())
                if not ceewaFile.exists():
                    ceewaFile.mkdir()
                elif not ceewaFile.isDirectory():
                    ceewaFile.mkdir()
                file_ = File(ceewaFile.getAbsolutePath() + "/Log")
                if not file_.exists():
                    file_.mkdir()
                elif not file_.isDirectory():
                    file_.mkdir()
                self.vlcApplication.setSaveLogUtil(file_.getAbsolutePath() + "/" + Tool.LOGFILENAME)
                self.saveLogUtil = self.vlcApplication.getSaveLogUtil()
                intent = Intent()
                intent.setAction(Tool.STARTTOSAVELOG_ACTION)
                sendBroadcast(intent)
            if self.saveLogUtil != None:
                msg = Message()
                msg.obj = Tool.bytes2HexString(parsePacketBytes)
                self.saveLogUtil.pushMsg(msg)
        self.subframeByte = parsePacketBytes[36]
        paramIntent = None
        if self.subframeByte == int(-119):
            self.stickModelByte = parsePacketBytes[44]
            self.settingBundle.putByte("stickmodel", self.stickModelByte)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.STICKMODELVALUE_ACTION)
            sendBroadcast(self.settingIntent)
        elif self.subframeByte == int(-118):
            self.skillLevelByte = parsePacketBytes[37]
            self.downloadParamsBytes_10[0] = parsePacketBytes[38]
            self.downloadParamsBytes_10[1] = parsePacketBytes[39]
            self.downloadParamsBytes_10[2] = parsePacketBytes[40]
            self.downloadParamsBytes_10[3] = parsePacketBytes[41]
            self.downloadParamsBytes_10[4] = parsePacketBytes[42]
            self.downloadParamsBytes_10[5] = parsePacketBytes[43]
            self.downloadParamsBytes_10[6] = parsePacketBytes[44]
            self.settingBundle = Bundle()
            self.settingIntent = Intent()
            self.settingBundle.putByte("skilllevel", self.skillLevelByte)
            self.settingBundle.putByteArray("paramsbytes", self.downloadParamsBytes_10)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.DOWNLOADSKILLLEVEL_ACTION)
            sendBroadcast(self.settingIntent)
        elif self.subframeByte == int(-111):
            self.agilityForDirectionByte = parsePacketBytes[37]
            self.downloadParamsBytes_17[0] = parsePacketBytes[38]
            self.downloadParamsBytes_17[1] = parsePacketBytes[39]
            self.downloadParamsBytes_17[2] = parsePacketBytes[40]
            self.downloadParamsBytes_17[3] = parsePacketBytes[41]
            self.downloadParamsBytes_17[4] = parsePacketBytes[42]
            self.downloadParamsBytes_17[5] = parsePacketBytes[43]
            self.downloadParamsBytes_17[6] = parsePacketBytes[44]
            self.settingBundle = Bundle()
            self.settingIntent = Intent()
            self.settingBundle.putByte("agility_direction", self.agilityForDirectionByte)
            self.settingBundle.putByteArray("paramsbytes", self.downloadParamsBytes_17)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.DOWNLOADDIRECTIONAGILITY_ACTION)
            sendBroadcast(self.settingIntent)
            flightParamsIntent = Intent()
            flightParamsIntent.setAction(Tool.FLIGHTPARAMSDOWNLOADED_ACTION)
            sendBroadcast(flightParamsIntent)
        elif self.subframeByte == int(-107):
            self.loseControlByte = parsePacketBytes[37]
            self.agilityForGimbalByte = parsePacketBytes[38]
            backAltitudeBytes = bytearray([parsePacketBytes[39], parsePacketBytes[40]])
            self.agilityForFollowByte = parsePacketBytes[41]
            self.speedByte = parsePacketBytes[43]
            self.backAltitudeShort = Tool.getShortFromBytes(backAltitudeBytes)
            self.downloadParamsBytes_21[0] = parsePacketBytes[42]
            self.downloadParamsBytes_21[1] = parsePacketBytes[44]
            self.settingBundle = Bundle()
            self.settingIntent = Intent()
            self.settingBundle.putByte("losecontrolindex", self.loseControlByte)
            self.settingBundle.putByte("agility_gimbal", self.agilityForGimbalByte)
            self.settingBundle.putShort("backalt", self.backAltitudeShort)
            self.settingBundle.putByte("agility_follow", self.agilityForFollowByte)
            self.settingBundle.putByte("speed", self.speedByte)
            self.settingBundle.putByteArray("paramsbytes", self.downloadParamsBytes_21)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.DOWNLOADBACKALTVALUEANDLOSECONTROLMODELFORSETTING_ACTION)
            sendBroadcast(self.settingIntent)
        elif self.subframeByte == int(-104):
            self.radiusLimitShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.altitudeLimitShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.downloadParamsBytes_24[0] = parsePacketBytes[37]
            self.downloadParamsBytes_24[1] = parsePacketBytes[38]
            self.downloadParamsBytes_24[2] = parsePacketBytes[43]
            self.downloadParamsBytes_24[3] = parsePacketBytes[44]
            self.settingBundle.putShort("altlimit", self.altitudeLimitShort)
            self.settingBundle.putShort("radiuslimit", self.radiusLimitShort)
            self.settingBundle.putByteArray("paramsbytes", self.downloadParamsBytes_24)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.DOWNLOADALTANDRADIUSLIMITFORSETTING_ACTION)
            sendBroadcast(self.settingIntent)
        elif self.subframeByte == int(-64):
            self.flapValueByte = parsePacketBytes[37]
            self.elevatorValueByte = parsePacketBytes[38]
            self.acceleratorValueByte = parsePacketBytes[39]
            self.rudderValueByte = parsePacketBytes[40]
            stickBundle = Bundle()
            stickIntent = Intent()
            stickBundle.putByte("flap", self.flapValueByte)
            stickBundle.putByte("elevator", self.elevatorValueByte)
            stickBundle.putByte("accelerator", self.acceleratorValueByte)
            stickBundle.putByte("rudder", self.rudderValueByte)
            stickIntent.putExtras(stickBundle)
            stickIntent.setAction(Tool.CALIBRATESTICKDATA_ACTION)
            sendBroadcast(stickIntent)
        elif self.subframeByte == int(-62):
            self.gyroscopeValueForUav_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.gyroscopeValueForUav_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.gyroscopeValueForUav_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.accelerateValueForUav_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[43], parsePacketBytes[44]]))
            self.gyroscopeForUavBundle.putShort("gyroscopeforuavx", self.gyroscopeValueForUav_X)
            self.gyroscopeForUavBundle.putShort("gyroscopeforuavy", self.gyroscopeValueForUav_Y)
            self.gyroscopeForUavBundle.putShort("gyroscopeforuavz", self.gyroscopeValueForUav_Z)
            self.gyroscopeForUavIntent.putExtras(self.gyroscopeForUavBundle)
            self.gyroscopeForUavIntent.setAction(Tool.GYROSCOPSEVALUEFORUAV_ACTION)
            sendBroadcast(self.gyroscopeForUavIntent)
        elif self.subframeByte == int(-61):
            self.accelerateValueForUav_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.accelerateValueForUav_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.accelerateForUavBundle.putShort("accelerateforuavx", self.accelerateValueForUav_X)
            self.accelerateForUavBundle.putShort("accelerateforuavy", self.accelerateValueForUav_Y)
            self.accelerateForUavBundle.putShort("accelerateforuavz", self.accelerateValueForUav_Z)
            self.accelerateForUavIntent.putExtras(self.accelerateForUavBundle)
            self.accelerateForUavIntent.setAction(Tool.ACCELERATEVALUEFORUAV_ACTION)
            sendBroadcast(self.accelerateForUavIntent)
            self.remainTimeShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[43], parsePacketBytes[44]]))
            self.subframeIndex = 0
            self.batteryBundle.putInt("subframeindex", self.subframeIndex)
            self.batteryBundle.putShort("remaintime", self.remainTimeShort)
            self.batteryIntent.putExtras(self.batteryBundle)
            self.batteryIntent.setAction(Tool.BATTERYINFO_ACTION)
            sendBroadcast(self.batteryIntent)
        elif self.subframeByte == int(-60):
            self.voltageShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.powerCurrentShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.remainPowerShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.totalCapacityShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[43], parsePacketBytes[44]]))
            self.subframeIndex = 1
            self.batteryBundle.putInt("subframeindex", self.subframeIndex)
            self.batteryBundle.putShort("voltage", self.voltageShort)
            self.batteryBundle.putShort("powercurrent", self.powerCurrentShort)
            self.batteryBundle.putShort("remainpower", self.remainPowerShort)
            self.batteryBundle.putShort("totalcapacity", self.totalCapacityShort)
            self.batteryIntent.putExtras(self.batteryBundle)
            self.batteryIntent.setAction(Tool.BATTERYINFO_ACTION)
            sendBroadcast(self.batteryIntent)
        elif self.subframeByte == int(-59):
            self.timeChargeShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.batteryLifeByte = parsePacketBytes[39]
            self.temperatureByte = parsePacketBytes[40]
            self.voltageForBatteryOneShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.voltageForBatteryTwoShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[43], parsePacketBytes[44]]))
            self.subframeIndex = 2
            self.batteryBundle.putInt("subframeindex", self.subframeIndex)
            self.batteryBundle.putShort("timecharge", self.timeChargeShort)
            self.batteryBundle.putByte("batterylife", self.batteryLifeByte)
            self.batteryBundle.putByte("temperature", self.temperatureByte)
            self.batteryBundle.putShort("voltageforbatteryone", self.voltageForBatteryOneShort)
            self.batteryBundle.putShort("voltageforbatterytwo", self.voltageForBatteryTwoShort)
            self.batteryIntent.putExtras(self.batteryBundle)
            self.batteryIntent.setAction(Tool.BATTERYINFO_ACTION)
            sendBroadcast(self.batteryIntent)
        elif self.subframeByte == int(-58):
            self.voltageForBatteryThreeShort = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.subframeIndex = 3
            self.batteryBundle.putInt("subframeindex", self.subframeIndex)
            self.batteryBundle.putShort("voltageforbatterythree", self.voltageForBatteryThreeShort)
            self.batteryIntent.putExtras(self.batteryBundle)
            self.batteryIntent.setAction(Tool.BATTERYINFO_ACTION)
            sendBroadcast(self.batteryIntent)
        elif self.subframeByte == int(-57):
            self.uavInformation.flight_mode = parsePacketBytes[37]
            self.uavInformation.cruise_sub_mode = int(((parsePacketBytes[38] >> 3) & 31))
            self.uavInformation.cruise_aircrafr_status = int((parsePacketBytes[38] & 3))
            self.uavInformation.follow_cruise_sub_mode = int(((parsePacketBytes[39] >> 3) & 31))
            self.uavInformation.cruise_aircrafr_status = int((parsePacketBytes[39] & 3))
            self.uavInformation.cruise_heading_mode = int(((parsePacketBytes[40] >> 6) & 3))
            self.uavInformation.follow_heading_mode = int(((parsePacketBytes[40] >> 4) & 3))
            self.uavInformation.follow_coordinate = int(((parsePacketBytes[40] >> 2) & 3))
            self.uavInformation.gps_hold_heading_mode = int((parsePacketBytes[40] & 3))
            self.uavInformation.gimbal_control_mode = int(((parsePacketBytes[41] >> 6) & 3))
            self.sdStateForOSDByte = int(((parsePacketBytes[41] >> 4) & 3))
            self.gimbalStateByte = int(((parsePacketBytes[41] >> 1) & 7))
            self.linkStateForOSDByte = int((parsePacketBytes[41] & 1))
            if self.uavInformation.flight_mode == int(6):
                self.flightMode = int(0)
            elif self.uavInformation.flight_mode == int(3) and self.uavInformation.gps_hold_heading_mode == int(1):
                self.flightMode = int(1)
            elif self.uavInformation.flight_mode == int(3) and self.uavInformation.gimbal_control_mode == int(2):
                self.flightMode = int(2)
            elif self.uavInformation.flight_mode == int(7) and self.uavInformation.follow_cruise_sub_mode == int(10):
                self.flightMode = int(3)
            elif self.uavInformation.flight_mode == int(7) and self.uavInformation.follow_cruise_sub_mode == int(11):
                self.flightMode = int(4)
            elif self.uavInformation.flight_mode == int(7) and self.uavInformation.follow_cruise_sub_mode == int(12):
                self.flightMode = int(5)
            elif self.uavInformation.flight_mode == int(7) and self.uavInformation.follow_cruise_sub_mode == int(9):
                self.flightMode = int(6)
            elif self.uavInformation.flight_mode == int(7) and self.uavInformation.follow_cruise_sub_mode == int(13):
                self.flightMode = int(7)
            elif self.uavInformation.flight_mode == int(4) and self.uavInformation.cruise_sub_mode == int(7):
                self.flightMode = int(8)
            elif self.uavInformation.flight_mode == int(4) and self.uavInformation.cruise_sub_mode == int(6):
                self.flightMode = int(9)
            elif self.uavInformation.flight_mode == int(4) and self.uavInformation.cruise_sub_mode == int(5):
                self.flightMode = int(10)
            elif self.uavInformation.flight_mode == int(3) and self.uavInformation.gimbal_control_mode != int(2) and self.uavInformation.gps_hold_heading_mode != int(1):
                self.flightMode = int(11)
            elif self.uavInformation.flight_mode == int(4) and self.uavInformation.cruise_sub_mode == int(3):
                self.flightMode = int(12)
            elif self.uavInformation.flight_mode == int(2):
                self.flightMode = int(13)
            else:
                self.flightMode = int(16)
            self.flightModelBundle.putByte("flightmodel", self.flightMode)
            self.flightModelBundle.putByte("sdstateforosd", self.sdStateForOSDByte)
            self.flightModelBundle.putByte("gimbalstate", self.gimbalStateByte)
            self.flightModelBundle.putByte("linkstateforosd", self.linkStateForOSDByte)
            self.flightModelIntent.putExtras(self.flightModelBundle)
            self.flightModelIntent.setAction(Tool.FLIGHTMODEL_ACTION)
            sendBroadcast(self.flightModelIntent)
            self.gimbalVersionByte = parsePacketBytes[43]
            self.droneVersionByte = parsePacketBytes[44]
            self.versionBundle = Bundle()
            self.versionIntent = Intent()
            self.versionBundle.putByte("subframeindex", int(0))
            self.versionBundle.putByte("version_gimbal", self.gimbalVersionByte)
            self.versionBundle.putByte("version_drone", self.droneVersionByte)
            self.versionIntent.putExtras(self.versionBundle)
            self.versionIntent.setAction(Tool.GETVERSIONVALUES_ACTION)
            sendBroadcast(self.versionIntent)
        elif self.subframeByte == int(-56):
            self.batteryStateByte = int((parsePacketBytes[37] & 15))
            self.warnForForbidenFlight = int(((parsePacketBytes[38] >> 2) & 1))
            self.magneticErrorByte = int(((parsePacketBytes[38] >> 1) & 1))
            self.takeoffFlagOK = int(((parsePacketBytes[39] >> 1) & 1))
            self.landFlagOK = int((parsePacketBytes[39] & 1))
            self.landFlagForLowBattery = int(((parsePacketBytes[39] >> 4) & 1))
            self.homeValidByte = int(((parsePacketBytes[39] >> 5) & 1))
            self.homeBundle.putByte("homevalid", self.homeValidByte)
            self.homeBundle.putDouble("homelng", (float(self.homeLngInt)) / 1.0E7)
            self.homeBundle.putDouble("homelat", (float(self.homeLatInt)) / 1.0E7)
            self.homeBundle.putDouble("homealt", (float(self.homeAltInt)) / 1000.0)
            self.homeIntent.putExtras(self.homeBundle)
            self.homeIntent.setAction(Tool.SENDHOMELATLNG_ACTION)
            sendBroadcast(self.homeIntent)
            self.motorRunByte = int(((parsePacketBytes[39] >> 7) & 1))
            if self.motorRunByte == int(1) and self.isFirstListenForMotorRun:
                self.isFirstListenForMotorRun = False
                if not self.isMotorRunTimerCanceled:
                    self.uploadedWaypointBundle.putInt("position", 0)
                    self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
                    self.uploadedWaypointIntent.setAction(Tool.MOTORSTART_ACTION)
                    sendBroadcast(self.uploadedWaypointIntent)
                    self.motorRunTimer.cancel()
                    self.motorRunTimer = None
                    self.isMotorRunTimerCanceled = True
            self.uploadedWaypointBundle = Bundle()
            self.uploadedWaypointIntent = Intent()
            self.uploadedWaypointBundle.putByte("motorstate", self.motorRunByte)
            self.uploadedWaypointBundle.putByte("batterystate", self.batteryStateByte)
            self.uploadedWaypointBundle.putByte("takeoffflag", self.takeoffFlagOK)
            self.uploadedWaypointBundle.putByte("landflag", self.landFlagOK)
            self.uploadedWaypointBundle.putByte("landflagforlowbattery", self.landFlagForLowBattery)
            self.uploadedWaypointBundle.putByte("forbidenflight", self.warnForForbidenFlight)
            self.uploadedWaypointBundle.putByte("magneticerror", self.magneticErrorByte)
            self.uploadedWaypointIntent.putExtras(self.uploadedWaypointBundle)
            self.uploadedWaypointIntent.setAction(Tool.MOTORSTATE_ACTION)
            sendBroadcast(self.uploadedWaypointIntent)
            self.satelliteNumForModuleByte = int((parsePacketBytes[41] & 63))
            self.gpsStateForModuleByte = int(((parsePacketBytes[41] >> 6) & 3))
            self.signalStateForModuleByte = int(((parsePacketBytes[42] >> 2) & 7))
            self.batteryForModuleByte = parsePacketBytes[43]
            self.moduleBundle.putByte("subframeindex", int(0))
            self.moduleBundle.putByte("modulesatellitenum", self.satelliteNumForModuleByte)
            self.moduleBundle.putByte("gpsstateformodule", self.gpsStateForModuleByte)
            self.moduleBundle.putByte("modulebattery", self.batteryForModuleByte)
            self.moduleBundle.putByte("modulesignalstate", self.signalStateForModuleByte)
            self.moduleIntent.putExtras(self.moduleBundle)
            self.moduleIntent.setAction(Tool.SENDMODULEDATA_ACTION)
            sendBroadcast(self.moduleIntent)
        elif self.subframeByte == int(-55):
            self.homeLngInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[37], parsePacketBytes[38], parsePacketBytes[39], parsePacketBytes[40]]))
            self.uavInformation.home_longitude = (float(self.homeLngInt)) / 1.0E7
            self.homeLatInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[41], parsePacketBytes[42], parsePacketBytes[43], parsePacketBytes[44]]))
            self.uavInformation.home_latitude = (float(self.homeLatInt)) / 1.0E7
            self.homeBundle.putByte("homevalid", self.homeValidByte)
            self.homeBundle.putDouble("homelng", (float(self.homeLngInt)) / 1.0E7)
            self.homeBundle.putDouble("homelat", (float(self.homeLatInt)) / 1.0E7)
            self.homeBundle.putDouble("homealt", (float(self.homeAltInt)) / 1000.0)
            self.homeBundle.putByteArray("bytes_homealt", None)
            self.homeIntent.putExtras(self.homeBundle)
            self.homeIntent.setAction(Tool.SENDHOMELATLNG_ACTION)
            sendBroadcast(self.homeIntent)
        elif self.subframeByte == int(-54):
            homeAltBytes = bytearray([parsePacketBytes[37], parsePacketBytes[38], parsePacketBytes[39], parsePacketBytes[40]])
            self.homeAltInt = Tool.byteArrayToInt_ZSY(homeAltBytes)
            self.uavInformation.home_altitude = (float(self.homeAltInt)) / 1000.0
            self.moduleAltInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[41], parsePacketBytes[42], parsePacketBytes[43], parsePacketBytes[44]]))
            self.moduleAltDouble = (float(self.moduleAltInt)) / 1000.0
            self.homeBundle.putByte("homevalid", self.homeValidByte)
            self.homeBundle.putDouble("homelng", (float(self.homeLngInt)) / 1.0E7)
            self.homeBundle.putDouble("homelat", (float(self.homeLatInt)) / 1.0E7)
            self.homeBundle.putDouble("homealt", (float(self.homeAltInt)) / 1000.0)
            self.homeBundle.putByteArray("bytes_homealt", homeAltBytes)
            self.homeIntent.putExtras(self.homeBundle)
            self.homeIntent.setAction(Tool.SENDHOMELATLNG_ACTION)
            sendBroadcast(self.homeIntent)
            self.moduleBundle.putByte("subframeindex", int(1))
            self.moduleBundle.putDouble("modulealt", self.moduleAltDouble)
            self.moduleIntent.putExtras(self.moduleBundle)
            self.moduleIntent.setAction(Tool.SENDMODULEDATA_ACTION)
            sendBroadcast(self.moduleIntent)
            self.barometerBundle.putDouble("uavalt", self.uavAlt)
            self.barometerBundle.putDouble("modulealt", self.moduleAltDouble)
            self.barometerIntent.putExtras(self.barometerBundle)
            self.barometerIntent.setAction(Tool.BAROMETERVALUE_ACTION)
            sendBroadcast(self.barometerIntent)
        elif self.subframeByte == int(-53):
            self.moduleLngInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[37], parsePacketBytes[38], parsePacketBytes[39], parsePacketBytes[40]]))
            self.moduleLngDouble = (float(self.moduleLngInt)) / 1.0E7
            self.moduleLatInt = Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[41], parsePacketBytes[42], parsePacketBytes[43], parsePacketBytes[44]]))
            self.moduleLatDouble = (float(self.moduleLatInt)) / 1.0E7
            self.moduleBundle.putByte("subframeindex", int(2))
            self.moduleBundle.putDouble("modulelng", self.moduleLngDouble)
            self.moduleBundle.putDouble("modulelat", self.moduleLatDouble)
            self.moduleIntent.putExtras(self.moduleBundle)
            self.moduleIntent.setAction(Tool.SENDMODULEDATA_ACTION)
            sendBroadcast(self.moduleIntent)
        elif self.subframeByte == int(-52):
            self.magneticValueForUav_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.magneticValueForUav_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.magneticValueForUav_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.magneticForUavBundle.putShort("magneticforuavx", self.magneticValueForUav_X)
            self.magneticForUavBundle.putShort("magneticforuavy", self.magneticValueForUav_Y)
            self.magneticForUavBundle.putShort("magneticforuavz", self.magneticValueForUav_Z)
            self.magneticForUavIntent.putExtras(self.magneticForUavBundle)
            self.magneticForUavIntent.setAction(Tool.MAGNETICVALUEFORUAV_ACTION)
            sendBroadcast(self.magneticForUavIntent)
            self.deviceTypeByte = parsePacketBytes[43]
            self.deviceVersionByte = parsePacketBytes[44]
            self.versionIntent = Intent()
            self.versionBundle = Bundle()
            self.versionBundle.putByte("subframeindex", int(1))
            self.versionBundle.putByte("type_device", self.deviceTypeByte)
            self.versionBundle.putByte("version_device", self.deviceVersionByte)
            self.versionIntent.putExtras(self.versionBundle)
            self.versionIntent.setAction(Tool.GETVERSIONVALUES_ACTION)
            sendBroadcast(self.versionIntent)
        elif self.subframeByte == int(-51):
            self.gyroscopeValueForModule_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.gyroscopeValueForModule_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.gyroscopeValueForModule_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.accelerateValueForModule_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[43], parsePacketBytes[44]]))
            self.gyroscopeForModuleBundle.putShort("gyroscopeformodulex", self.gyroscopeValueForModule_X)
            self.gyroscopeForModuleBundle.putShort("gyroscopeformoduley", self.gyroscopeValueForModule_Y)
            self.gyroscopeForModuleBundle.putShort("gyroscopeformodulez", self.gyroscopeValueForModule_Z)
            self.gyroscopeForModuleIntent.putExtras(self.gyroscopeForModuleBundle)
            self.gyroscopeForModuleIntent.setAction(Tool.GYROSCOPSEVALUEFORMODULE_ACTION)
            sendBroadcast(self.gyroscopeForModuleIntent)
        elif self.subframeByte == int(-50):
            self.accelerateValueForModule_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.accelerateValueForModule_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.accelerateForModuleBundle.putShort("accelerateformodulex", self.accelerateValueForModule_X)
            self.accelerateForModuleBundle.putShort("accelerateformoduley", self.accelerateValueForModule_Y)
            self.accelerateForModuleBundle.putShort("accelerateformodulez", self.accelerateValueForModule_Z)
            self.accelerateForModuleIntent.putExtras(self.accelerateForModuleBundle)
            self.accelerateForModuleIntent.setAction(Tool.ACCELERATEVALUEFORMODULE_ACTION)
            sendBroadcast(self.accelerateForModuleIntent)
        elif self.subframeByte == int(-49):
            self.magneticValueForModule_X = Tool.getShortFromBytes(bytearray([parsePacketBytes[37], parsePacketBytes[38]]))
            self.magneticValueForModule_Y = Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]]))
            self.magneticValueForModule_Z = Tool.getShortFromBytes(bytearray([parsePacketBytes[41], parsePacketBytes[42]]))
            self.magneticForModuleBundle.putShort("magneticformodulex", self.magneticValueForModule_X)
            self.magneticForModuleBundle.putShort("magneticformoduley", self.magneticValueForModule_Y)
            self.magneticForModuleBundle.putShort("magneticformodulez", self.magneticValueForModule_Z)
            self.magneticForModuleIntent.putExtras(self.magneticForModuleBundle)
            self.magneticForModuleIntent.setAction(Tool.MAGNETICVALUEFORMODULE_ACTION)
            sendBroadcast(self.magneticForModuleIntent)
            self.moduleVersionByte = parsePacketBytes[43]
            self.versionIntent = Intent()
            self.versionBundle = Bundle()
            self.versionBundle.putByte("subframeindex", int(2))
            self.versionBundle.putByte("version_module", self.moduleVersionByte)
            self.versionIntent.putExtras(self.versionBundle)
            self.versionIntent.setAction(Tool.GETVERSIONVALUES_ACTION)
            sendBroadcast(self.versionIntent)
        elif self.subframeByte == int(-48):
            self.subframeHeadByteReceivedBundle = Bundle()
            self.subframeHeadByteReceivedIntent = Intent()
            if parsePacketBytes[37] == int(0):
                self.subframeHeadByteReceived = int(-60)
                self.noWaypointToDownloadIntent.setAction(Tool.NOWAYPOINTTODOWNLOAD_ACTION)
                sendBroadcast(self.noWaypointToDownloadIntent)
            elif parsePacketBytes[37] == int(1):
                self.waypointParameterForDownload = WayPointParameter()
                self.waypointParameterForDownload.number = parsePacketBytes[37]
                self.waypointParameterForDownload.currentIndex = parsePacketBytes[38]
                self.waypointParameterForDownload.nextIndex = parsePacketBytes[39]
                self.waypointParameterForDownload.property = parsePacketBytes[40]
                self.waypointParameterLngBytesForDownload[3] = parsePacketBytes[41]
                self.waypointParameterLngBytesForDownload[2] = parsePacketBytes[42]
                self.waypointParameterLngBytesForDownload[1] = parsePacketBytes[43]
                self.waypointParameterLngBytesForDownload[0] = parsePacketBytes[44]
                self.waypointParameterLngForDownload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForDownload))) / 1000.0
                self.waypointParameterForDownload.longitude = self.waypointParameterLngForDownload
                self.subframeHeadByteReceived = int(-47)
            self.subframeHeadByteReceivedBundle.putByte("subframe", self.subframeHeadByteReceived)
            self.subframeHeadByteReceivedIntent.putExtras(self.subframeHeadByteReceivedBundle)
            self.subframeHeadByteReceivedIntent.setAction(Tool.SETSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.subframeHeadByteReceivedIntent)
            self.downloadingSubframeByteBundle = Bundle()
            self.downloadingSubframeByteIntent = Intent()
            self.downloadingSubframeByteBundle.putInt("downloadingwaypointtypeindex", self.downloadingWaypointTypeIndex)
            self.downloadingSubframeByteBundle.putByte("subframebyte", self.subframeHeadByteReceived)
            self.downloadingSubframeByteIntent.putExtras(self.downloadingSubframeByteBundle)
            self.downloadingSubframeByteIntent.setAction(Tool.DOWNLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.downloadingSubframeByteIntent)
            self.downloadingSubframeByteBundle = None
            self.downloadingSubframeByteIntent = None
        elif self.subframeByte == int(-47):
            self.waypointParameterForDownload.number = parsePacketBytes[37]
            self.waypointParameterForDownload.currentIndex = parsePacketBytes[38]
            self.waypointParameterVelocityBytesForDownload[0] = parsePacketBytes[39]
            self.waypointParameterVelocityBytesForDownload[1] = parsePacketBytes[40]
            self.waypointParameterVelocityForDownload = Tool.getShortFromBytes(self.waypointParameterVelocityBytesForDownload)
            self.waypointParameterForDownload.velocity = int((int(((float(self.waypointParameterVelocityForDownload)) / 10.0))))
            self.waypointParameterLatBytesForDownload[3] = parsePacketBytes[41]
            self.waypointParameterLatBytesForDownload[2] = parsePacketBytes[42]
            self.waypointParameterLatBytesForDownload[1] = parsePacketBytes[43]
            self.waypointParameterLatBytesForDownload[0] = parsePacketBytes[44]
            self.waypointParameterLatForDownload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForDownload))) / 1000.0
            self.waypointParameterForDownload.latitude = self.waypointParameterLatForDownload
            self.subframeHeadByteReceived = int(-46)
            self.subframeHeadByteReceivedBundle.putByte("subframe", self.subframeHeadByteReceived)
            self.subframeHeadByteReceivedIntent.putExtras(self.subframeHeadByteReceivedBundle)
            self.subframeHeadByteReceivedIntent.setAction(Tool.SETSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.subframeHeadByteReceivedIntent)
            self.downloadingSubframeByteBundle = Bundle()
            self.downloadingSubframeByteIntent = Intent()
            self.downloadingSubframeByteBundle.putInt("downloadingwaypointtypeindex", self.downloadingWaypointTypeIndex)
            self.downloadingSubframeByteBundle.putByte("subframebyte", int(-47))
            self.downloadingSubframeByteIntent.putExtras(self.downloadingSubframeByteBundle)
            self.downloadingSubframeByteIntent.setAction(Tool.DOWNLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.downloadingSubframeByteIntent)
            self.downloadingSubframeByteBundle = None
            self.downloadingSubframeByteIntent = None
        elif self.subframeByte == int(-46):
            self.waypointParameterForDownload.number = parsePacketBytes[37]
            self.waypointParameterForDownload.currentIndex = parsePacketBytes[38]
            self.waypointParameterAltBytesForDownload[3] = parsePacketBytes[39]
            self.waypointParameterAltBytesForDownload[2] = parsePacketBytes[40]
            self.waypointParameterAltBytesForDownload[1] = parsePacketBytes[41]
            self.waypointParameterAltBytesForDownload[0] = parsePacketBytes[42]
            self.waypointParameterAltForDownload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForDownload))) / 1000.0
            self.waypointParameterForDownload.altitude = self.waypointParameterAltForDownload
            self.waypointParameterHeadingBytesForDownload[0] = parsePacketBytes[43]
            self.waypointParameterHeadingBytesForDownload[1] = parsePacketBytes[44]
            self.waypointParameterHeadingForDownload = Tool.getShortFromBytes(self.waypointParameterHeadingBytesForDownload)
            self.waypointParameterForDownload.heading = self.waypointParameterHeadingForDownload
            self.subframeHeadByteReceived = int(-45)
            self.subframeHeadByteReceivedBundle.putByte("subframe", self.subframeHeadByteReceived)
            self.subframeHeadByteReceivedIntent.putExtras(self.subframeHeadByteReceivedBundle)
            self.subframeHeadByteReceivedIntent.setAction(Tool.SETSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.subframeHeadByteReceivedIntent)
            self.downloadingSubframeByteBundle = Bundle()
            self.downloadingSubframeByteIntent = Intent()
            self.downloadingSubframeByteBundle.putInt("downloadingwaypointtypeindex", self.downloadingWaypointTypeIndex)
            self.downloadingSubframeByteBundle.putByte("subframebyte", int(-46))
            self.downloadingSubframeByteIntent.putExtras(self.downloadingSubframeByteBundle)
            self.downloadingSubframeByteIntent.setAction(Tool.DOWNLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.downloadingSubframeByteIntent)
            self.downloadingSubframeByteBundle = None
            self.downloadingSubframeByteIntent = None
        elif self.subframeByte == int(-45):
            self.waypointParameterForDownload.number = parsePacketBytes[37]
            self.waypointParameterForDownload.currentIndex = parsePacketBytes[38]
            self.waypointParameterDistanceBytesForDownload[0] = parsePacketBytes[39]
            self.waypointParameterDistanceBytesForDownload[1] = parsePacketBytes[40]
            self.waypointParameterDistanceForDownload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForDownload)
            self.waypointParameterForDownload.distance = self.waypointParameterDistanceForDownload
            self.waypointParametervCircleBytesForDownload[0] = parsePacketBytes[41]
            self.waypointParametervCircleBytesForDownload[1] = parsePacketBytes[42]
            self.waypointParametervCircleForDownload = Tool.getShortFromBytes(self.waypointParametervCircleBytesForDownload)
            self.waypointParameterForDownload.vCircle = self.waypointParametervCircleForDownload
            self.waypointParameterClimbRateBytesForDownload[0] = parsePacketBytes[43]
            self.waypointParameterClimbRateBytesForDownload[1] = parsePacketBytes[44]
            self.waypointParameterClimbRateForDownload = Tool.getShortFromBytes(self.waypointParameterClimbRateBytesForDownload)
            self.waypointParameterForDownload.climbRate = self.waypointParameterClimbRateForDownload
            self.subframeHeadByteReceived = int(-44)
            self.subframeHeadByteReceivedBundle.putByte("subframe", self.subframeHeadByteReceived)
            self.subframeHeadByteReceivedIntent.putExtras(self.subframeHeadByteReceivedBundle)
            self.subframeHeadByteReceivedIntent.setAction(Tool.SETSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.subframeHeadByteReceivedIntent)
            self.downloadingSubframeByteBundle = Bundle()
            self.downloadingSubframeByteIntent = Intent()
            self.downloadingSubframeByteBundle.putInt("downloadingwaypointtypeindex", self.downloadingWaypointTypeIndex)
            self.downloadingSubframeByteBundle.putByte("subframebyte", int(-45))
            self.downloadingSubframeByteIntent.putExtras(self.downloadingSubframeByteBundle)
            self.downloadingSubframeByteIntent.setAction(Tool.DOWNLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.downloadingSubframeByteIntent)
            self.downloadingSubframeByteBundle = None
            self.downloadingSubframeByteIntent = None
        elif self.subframeByte == int(-44):
            self.waypointParameterForDownload.number = parsePacketBytes[37]
            self.waypointParameterForDownload.currentIndex = parsePacketBytes[38]
            self.waypointParameterRadiusBytesForDownload[0] = parsePacketBytes[39]
            self.waypointParameterRadiusBytesForDownload[1] = parsePacketBytes[40]
            self.waypointParameterRadiusForDownload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForDownload)
            self.waypointParameterForDownload.radius = self.waypointParameterRadiusForDownload
            self.waypointParameterDurationBytesForDownload[0] = parsePacketBytes[41]
            self.waypointParameterDurationBytesForDownload[1] = parsePacketBytes[42]
            self.waypointParameterDurationForDownload = Tool.getShortFromBytes(self.waypointParameterDurationBytesForDownload)
            self.waypointParameterForDownload.duration = self.waypointParameterDurationForDownload
            self.waypointParameterForDownload.mode = parsePacketBytes[43]
            self.downloadingSubframeByteBundle = Bundle()
            self.downloadingSubframeByteIntent = Intent()
            self.downloadingSubframeByteBundle.putInt("downloadingwaypointtypeindex", self.downloadingWaypointTypeIndex)
            self.downloadingSubframeByteBundle.putByte("subframebyte", int(-44))
            self.downloadingSubframeByteIntent.putExtras(self.downloadingSubframeByteBundle)
            self.downloadingSubframeByteIntent.setAction(Tool.DOWNLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.downloadingSubframeByteIntent)
            self.downloadingSubframeByteBundle = None
            self.downloadingSubframeByteIntent = None
            self.sharedPreferencesForShowFlightFollow = getSharedPreferences(Tool.SHAREDPREFERENCESFORSHOWFLIGHTFOLLOW, 0)
            self.editorForShowFlightFollow = self.sharedPreferencesForShowFlightFollow.edit()
            if self.downloadingWaypointTypeIndex == 0:
                self.cacheForRelativePositionFollow.setLatitude(self.waypointParameterForDownload.latitude)
                self.cacheForRelativePositionFollow.setLongitude(self.waypointParameterForDownload.longitude)
                self.cacheForRelativePositionFollow.setAltitude(self.waypointParameterForDownload.altitude)
                self.vlcApplication.setCacheDataForRelativePosition(self.cacheForRelativePositionFollow)
                self.editorForShowFlightFollow.putBoolean(Tool.ISFIRSTSHOWRELATIVEPOSITIONFOLLOW, False)
                self.downloadedWaypointBundle.putInt("downloadedwaypointmodel", 0)
                self.downloadedWaypointBundle.putParcelable("downloadedwaypointvalue", self.waypointParameterForDownload)
                self.downloadedWaypointIntent.setAction(Tool.DOWNLOADEDALLWAYPOINTS_ACTION)
                self.downloadedWaypointIntent.putExtras(self.downloadedWaypointBundle)
                sendBroadcast(self.downloadedWaypointIntent)
            elif self.downloadingWaypointTypeIndex == 1:
                self.cacheForAutoFollow.setLatitude(self.waypointParameterForDownload.latitude)
                self.cacheForAutoFollow.setLongitude(self.waypointParameterForDownload.longitude)
                self.cacheForAutoFollow.setAltitude(self.waypointParameterForDownload.altitude)
                self.vlcApplication.setCacheDataForAuto(self.cacheForAutoFollow)
                self.editorForShowFlightFollow.putBoolean(Tool.ISFIRSTSHOWAUTOFOLLOW, False)
                self.downloadedWaypointBundle.putInt("downloadedwaypointmodel", 1)
                self.downloadedWaypointBundle.putParcelable("downloadedwaypointvalue", self.waypointParameterForDownload)
                self.downloadedWaypointIntent.setAction(Tool.DOWNLOADEDALLWAYPOINTS_ACTION)
                self.downloadedWaypointIntent.putExtras(self.downloadedWaypointBundle)
                sendBroadcast(self.downloadedWaypointIntent)
            elif self.downloadingWaypointTypeIndex == 2:
                self.cacheForCircleFollow.setAltitude(self.waypointParameterForDownload.altitude)
                self.cacheForCircleFollow.setRadius(self.waypointParameterForDownload.radius)
                self.cacheForCircleFollow.setVcircle(self.waypointParameterForDownload.vCircle)
                self.vlcApplication.setCacheDataForCircle(self.cacheForCircleFollow)
                self.editorForShowFlightFollow.putBoolean(Tool.ISFIRSTSHOWCIRCLEFOLLOW, False)
                self.downloadedWaypointBundle.putInt("downloadedwaypointmodel", 2)
                self.downloadedWaypointBundle.putParcelable("downloadedwaypointvalue", self.waypointParameterForDownload)
                self.downloadedWaypointIntent.setAction(Tool.DOWNLOADEDALLWAYPOINTS_ACTION)
                self.downloadedWaypointIntent.putExtras(self.downloadedWaypointBundle)
                sendBroadcast(self.downloadedWaypointIntent)
            elif self.downloadingWaypointTypeIndex == 3:
                self.cacheForSelfie.setAltitude(self.waypointParameterForDownload.altitude)
                self.cacheForSelfie.setDistance(self.waypointParameterForDownload.longitude)
                self.cacheForSelfie.setVelocity(self.waypointParameterForDownload.velocity)
                self.vlcApplication.setCacheDataForSelfie(self.cacheForSelfie)
                self.editorForShowFlightFollow.putBoolean(Tool.ISFIRSTSHOWSELFIEFOLLOW, False)
                self.downloadedWaypointBundle.putInt("downloadedwaypointmodel", 3)
                self.downloadedWaypointBundle.putParcelable("downloadedwaypointvalueforselfie", self.waypointParameterForDownload)
                self.downloadedWaypointIntent.setAction(Tool.DOWNLOADEDALLWAYPOINTS_ACTION)
                self.downloadedWaypointIntent.putExtras(self.downloadedWaypointBundle)
                sendBroadcast(self.downloadedWaypointIntent)
            self.editorForShowFlightFollow.commit()
        elif self.subframeByte == int(9):
            self.stickModelByte = parsePacketBytes[44]
            self.settingBundle.putByte("stickmodel", self.stickModelByte)
            self.settingIntent.putExtras(self.settingBundle)
            self.settingIntent.setAction(Tool.SETSTICKMODELSUCCESS_ACTION)
            sendBroadcast(self.settingIntent)
        elif self.subframeByte == int(10):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0) or parsePacketBytes[39] != int(0) or parsePacketBytes[40] != int(0) or parsePacketBytes[41] != int(0) or parsePacketBytes[42] != int(0) or parsePacketBytes[43] != int(0) or parsePacketBytes[44] != int(0):
                if not self.isResetAllParamsValueReceived:
                    paramIntent = Intent()
                    paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSOK_ACTION)
                    sendBroadcast(paramIntent)
            paramIntent = Intent()
            paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSFAIL_ACTION)
            sendBroadcast(paramIntent)
        elif self.subframeByte == int(17):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0) or parsePacketBytes[39] != int(0) or parsePacketBytes[40] != int(0) or parsePacketBytes[41] != int(0) or parsePacketBytes[42] != int(0) or parsePacketBytes[43] != int(0) or parsePacketBytes[44] != int(0):
                if not self.isResetAllParamsValueReceived:
                    paramIntent = Intent()
                    paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSOK_ACTION)
                    sendBroadcast(paramIntent)
            paramIntent = Intent()
            paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSFAIL_ACTION)
            sendBroadcast(paramIntent)
        elif self.subframeByte == int(21):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0) or parsePacketBytes[39] != int(0) or parsePacketBytes[40] != int(0) or parsePacketBytes[41] != int(0) or parsePacketBytes[42] != int(0) or parsePacketBytes[43] != int(0) or parsePacketBytes[44] != int(0):
                if self.isResetAllParamsValueReceived:
                    if self.positionOfUploadParams == 1:
                        self.isResetAllParamsValueReceived = False
                        paramIntent = Intent()
                        paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSOK_ACTION)
                        sendBroadcast(paramIntent)
                paramIntent = Intent()
                paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSOK_ACTION)
                sendBroadcast(paramIntent)
            paramIntent = Intent()
            paramIntent.setAction(Tool.UPLOADFLIGHTPARAMSFAIL_ACTION)
            sendBroadcast(paramIntent)
        elif self.subframeByte == int(24):
            self.followIntent = Intent()
            self.followIntent.setAction(Tool.UPLOADFLIGHTPARAMSOK_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(49):
            syncTimeIntent = Intent()
            syncTimeIntent.setAction(Tool.SYNCHRONIZETIME_ACTION)
            sendBroadcast(syncTimeIntent)
        elif self.subframeByte == int(80):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0):
                if self.uploadingWaypointTypeIndex == 0:
                    self.relativePositionFollowWaypointParameterForUpload = WayPointParameter()
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1000.0
                    self.relativePositionFollowWaypointParameterForUpload.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 1:
                    self.autoFollowWaypointParameterForUpload = WayPointParameter()
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1000.0
                    self.autoFollowWaypointParameterForUpload.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 2:
                    self.circleFollowWaypointParameterForUpload = WayPointParameter()
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1000.0
                    self.circleFollowWaypointParameterForUpload.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 3:
                    self.selfieWaypointParameterForUpload = WayPointParameter()
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1000.0
                    self.selfieWaypointParameterForUpload.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 5:
                    self.signalCircleWaypointParameterUploaded = WayPointParameter()
                    self.signalCircleWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.signalCircleWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.signalCircleWaypointParameterUploaded.nextIndex = parsePacketBytes[39]
                    self.signalCircleWaypointParameterUploaded.property = parsePacketBytes[40]
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1.0E7
                    self.signalCircleWaypointParameterUploaded.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 6:
                    self.toPointWaypointParameterUploaded = WayPointParameter()
                    self.toPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.toPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.toPointWaypointParameterUploaded.nextIndex = parsePacketBytes[39]
                    self.toPointWaypointParameterUploaded.property = parsePacketBytes[40]
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1.0E7
                    self.toPointWaypointParameterUploaded.longitude = self.waypointParameterLngForUpload
                if self.uploadingWaypointTypeIndex == 7:
                    self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                    self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                    self.multiPointWaypointParameterUploaded = WayPointParameter()
                    if parsePacketBytes[38] == int(0):
                        if self.multiPointWaypointParameterUploadedList == None:
                            self.multiPointWaypointParameterUploadedList = ArrayList()
                        else:
                            self.multiPointWaypointParameterUploadedList.clear()
                    self.multiPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.multiPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.multiPointWaypointParameterUploaded.nextIndex = parsePacketBytes[39]
                    self.multiPointWaypointParameterUploaded.property = parsePacketBytes[40]
                    self.waypointParameterLngBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLngBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLngBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLngBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLngForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLngBytesForUpload))) / 1.0E7
                    self.multiPointWaypointParameterUploaded.longitude = self.waypointParameterLngForUpload
                self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
                self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(80))
                self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
                self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
                sendBroadcast(self.uploadingSubframeByteIntent)
            self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(81):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0):
                if self.uploadingWaypointTypeIndex == 0:
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload)
                    self.relativePositionFollowWaypointParameterForUpload.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1000.0
                    self.relativePositionFollowWaypointParameterForUpload.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 1:
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload)
                    self.autoFollowWaypointParameterForUpload.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1000.0
                    self.autoFollowWaypointParameterForUpload.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 2:
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload)
                    self.circleFollowWaypointParameterForUpload.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1000.0
                    self.circleFollowWaypointParameterForUpload.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 3:
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload))) / 10.0))))
                    self.selfieWaypointParameterForUpload.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1000.0
                    self.selfieWaypointParameterForUpload.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 5:
                    self.signalCircleWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.signalCircleWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload))) / 10.0))))
                    self.signalCircleWaypointParameterUploaded.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1.0E7
                    self.signalCircleWaypointParameterUploaded.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 6:
                    self.toPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.toPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload))) / 10.0))))
                    self.toPointWaypointParameterUploaded.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1.0E7
                    self.toPointWaypointParameterUploaded.latitude = self.waypointParameterLatForUpload
                if self.uploadingWaypointTypeIndex == 7:
                    self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                    self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                    self.multiPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.multiPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterVelocityBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterVelocityBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterVelocityForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParameterVelocityBytesForUpload))) / 10.0))))
                    self.multiPointWaypointParameterUploaded.velocity = self.waypointParameterVelocityForUpload
                    self.waypointParameterLatBytesForUpload[3] = parsePacketBytes[41]
                    self.waypointParameterLatBytesForUpload[2] = parsePacketBytes[42]
                    self.waypointParameterLatBytesForUpload[1] = parsePacketBytes[43]
                    self.waypointParameterLatBytesForUpload[0] = parsePacketBytes[44]
                    self.waypointParameterLatForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterLatBytesForUpload))) / 1.0E7
                    self.multiPointWaypointParameterUploaded.latitude = self.waypointParameterLatForUpload
                self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
                self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(81))
                self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
                self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
                sendBroadcast(self.uploadingSubframeByteIntent)
            self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(82):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0):
                if self.uploadingWaypointTypeIndex == 0:
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.relativePositionFollowWaypointParameterForUpload.altitude = self.waypointParameterAltForUpload
                if self.uploadingWaypointTypeIndex == 1:
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.autoFollowWaypointParameterForUpload.altitude = self.waypointParameterAltForUpload
                if self.uploadingWaypointTypeIndex == 2:
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.circleFollowWaypointParameterForUpload.altitude = self.waypointParameterAltForUpload
                if self.uploadingWaypointTypeIndex == 3:
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.selfieWaypointParameterForUpload.altitude = self.waypointParameterAltForUpload
                if self.uploadingWaypointTypeIndex == 5:
                    self.signalCircleWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.signalCircleWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.signalCircleWaypointParameterUploaded.altitude = self.waypointParameterAltForUpload
                    self.waypointParameterHeadingBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterHeadingBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterHeadingForUpload = Tool.getShortFromBytes(self.waypointParameterHeadingBytesForUpload)
                    self.signalCircleWaypointParameterUploaded.heading = self.waypointParameterHeadingForUpload
                if self.uploadingWaypointTypeIndex == 6:
                    self.toPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.toPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.toPointWaypointParameterUploaded.altitude = self.waypointParameterAltForUpload
                    self.waypointParameterHeadingBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterHeadingBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterHeadingForUpload = Tool.getShortFromBytes(self.waypointParameterHeadingBytesForUpload)
                    self.toPointWaypointParameterUploaded.heading = self.waypointParameterHeadingForUpload
                if self.uploadingWaypointTypeIndex == 7:
                    self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                    self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                    self.multiPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.multiPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterAltBytesForUpload[3] = parsePacketBytes[39]
                    self.waypointParameterAltBytesForUpload[2] = parsePacketBytes[40]
                    self.waypointParameterAltBytesForUpload[1] = parsePacketBytes[41]
                    self.waypointParameterAltBytesForUpload[0] = parsePacketBytes[42]
                    self.waypointParameterAltForUpload = (float(Tool.byteArrayToInt_ZSY(self.waypointParameterAltBytesForUpload))) / 1000.0
                    self.multiPointWaypointParameterUploaded.altitude = self.waypointParameterAltForUpload
                    self.waypointParameterHeadingBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterHeadingBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterHeadingForUpload = Tool.getShortFromBytes(self.waypointParameterHeadingBytesForUpload)
                    self.multiPointWaypointParameterUploaded.heading = self.waypointParameterHeadingForUpload
                self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
                self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(82))
                self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
                self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
                sendBroadcast(self.uploadingSubframeByteIntent)
            self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(83):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0):
                if self.uploadingWaypointTypeIndex == 0:
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.relativePositionFollowWaypointParameterForUpload.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload)
                    self.relativePositionFollowWaypointParameterForUpload.vCircle = self.waypointParametervCircleForUpload
                if self.uploadingWaypointTypeIndex == 1:
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.autoFollowWaypointParameterForUpload.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload)
                    self.autoFollowWaypointParameterForUpload.vCircle = self.waypointParametervCircleForUpload
                if self.uploadingWaypointTypeIndex == 2:
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload))) / 10.0))))
                    self.circleFollowWaypointParameterForUpload.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload)
                    self.circleFollowWaypointParameterForUpload.vCircle = self.waypointParametervCircleForUpload
                if self.uploadingWaypointTypeIndex == 3:
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.selfieWaypointParameterForUpload.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload)
                    self.selfieWaypointParameterForUpload.vCircle = self.waypointParametervCircleForUpload
                if self.uploadingWaypointTypeIndex == 5:
                    self.signalCircleWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.signalCircleWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.signalCircleWaypointParameterUploaded.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload))) / 10.0))))
                    self.signalCircleWaypointParameterUploaded.vCircle = self.waypointParametervCircleForUpload
                    self.waypointParameterClimbRateBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterClimbRateBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterClimbRateForUpload = Tool.getShortFromBytes(self.waypointParameterClimbRateBytesForUpload)
                    self.signalCircleWaypointParameterUploaded.climbRate = self.waypointParameterClimbRateForUpload
                if self.uploadingWaypointTypeIndex == 6:
                    self.toPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.toPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.toPointWaypointParameterUploaded.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload))) / 10.0))))
                    self.toPointWaypointParameterUploaded.vCircle = self.waypointParametervCircleForUpload
                    self.waypointParameterClimbRateBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterClimbRateBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterClimbRateForUpload = Tool.getShortFromBytes(self.waypointParameterClimbRateBytesForUpload)
                    self.toPointWaypointParameterUploaded.climbRate = self.waypointParameterClimbRateForUpload
                if self.uploadingWaypointTypeIndex == 7:
                    self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                    self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                    self.multiPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.multiPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterDistanceBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterDistanceBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterDistanceForUpload = Tool.getShortFromBytes(self.waypointParameterDistanceBytesForUpload)
                    self.multiPointWaypointParameterUploaded.distance = self.waypointParameterDistanceForUpload
                    self.waypointParametervCircleBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParametervCircleBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParametervCircleForUpload = int((int(((float(Tool.getShortFromBytes(self.waypointParametervCircleBytesForUpload))) / 10.0))))
                    self.multiPointWaypointParameterUploaded.vCircle = self.waypointParametervCircleForUpload
                    self.waypointParameterClimbRateBytesForUpload[0] = parsePacketBytes[43]
                    self.waypointParameterClimbRateBytesForUpload[1] = parsePacketBytes[44]
                    self.waypointParameterClimbRateForUpload = Tool.getShortFromBytes(self.waypointParameterClimbRateBytesForUpload)
                    self.multiPointWaypointParameterUploaded.climbRate = self.waypointParameterClimbRateForUpload
                self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
                self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(83))
                self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
                self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
                sendBroadcast(self.uploadingSubframeByteIntent)
            self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(84):
            if parsePacketBytes[37] != int(0) or parsePacketBytes[38] != int(0):
                if self.uploadingWaypointTypeIndex == 0:
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.relativePositionFollowWaypointParameterForUpload.radius = self.waypointParameterRadiusForUpload
                    self.cacheForRelativePositionFollow.setLatitude(self.relativePositionFollowWaypointParameterForUpload.latitude)
                    self.cacheForRelativePositionFollow.setLongitude(self.relativePositionFollowWaypointParameterForUpload.longitude)
                    self.cacheForRelativePositionFollow.setAltitude(self.relativePositionFollowWaypointParameterForUpload.altitude)
                    self.vlcApplication.setCacheDataForRelativePosition(self.cacheForRelativePositionFollow)
                if self.uploadingWaypointTypeIndex == 1:
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.autoFollowWaypointParameterForUpload.radius = self.waypointParameterRadiusForUpload
                    self.cacheForAutoFollow.setLatitude(self.autoFollowWaypointParameterForUpload.latitude)
                    self.cacheForAutoFollow.setLongitude(self.autoFollowWaypointParameterForUpload.longitude)
                    self.cacheForAutoFollow.setAltitude(self.autoFollowWaypointParameterForUpload.altitude)
                    self.vlcApplication.setCacheDataForAuto(self.cacheForAutoFollow)
                if self.uploadingWaypointTypeIndex == 2:
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.circleFollowWaypointParameterForUpload.radius = self.waypointParameterRadiusForUpload
                    self.cacheForCircleFollow.setAltitude(self.circleFollowWaypointParameterForUpload.altitude)
                    self.cacheForCircleFollow.setVcircle(self.circleFollowWaypointParameterForUpload.vCircle)
                    self.cacheForCircleFollow.setRadius(self.circleFollowWaypointParameterForUpload.radius)
                    self.vlcApplication.setCacheDataForCircle(self.cacheForCircleFollow)
                if self.uploadingWaypointTypeIndex == 3:
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.selfieWaypointParameterForUpload.radius = self.waypointParameterRadiusForUpload
                    self.cacheForSelfie.setDistance(self.selfieWaypointParameterForUpload.longitude)
                    self.cacheForSelfie.setAltitude(self.selfieWaypointParameterForUpload.altitude)
                    self.cacheForSelfie.setVelocity(self.selfieWaypointParameterForUpload.velocity)
                    self.vlcApplication.setCacheDataForSelfie(self.cacheForSelfie)
                if self.uploadingWaypointTypeIndex == 5:
                    self.signalCircleWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.signalCircleWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.signalCircleWaypointParameterUploaded.radius = self.waypointParameterRadiusForUpload
                    self.waypointParameterDurationBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParameterDurationBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParameterDurationForUpload = Tool.getShortFromBytes(self.waypointParameterDurationBytesForUpload)
                    self.signalCircleWaypointParameterUploaded.duration = self.waypointParameterDurationForUpload
                    self.signalCircleWaypointParameterUploaded.mode = parsePacketBytes[43]
                if self.uploadingWaypointTypeIndex == 6:
                    self.toPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.toPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.toPointWaypointParameterUploaded.radius = self.waypointParameterRadiusForUpload
                    self.waypointParameterDurationBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParameterDurationBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParameterDurationForUpload = Tool.getShortFromBytes(self.waypointParameterDurationBytesForUpload)
                    self.toPointWaypointParameterUploaded.duration = self.waypointParameterDurationForUpload
                    self.toPointWaypointParameterUploaded.mode = parsePacketBytes[43]
                if self.uploadingWaypointTypeIndex == 7:
                    self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                    self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                    self.multiPointWaypointParameterUploaded.number = parsePacketBytes[37]
                    self.multiPointWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                    self.waypointParameterRadiusBytesForUpload[0] = parsePacketBytes[39]
                    self.waypointParameterRadiusBytesForUpload[1] = parsePacketBytes[40]
                    self.waypointParameterRadiusForUpload = Tool.getShortFromBytes(self.waypointParameterRadiusBytesForUpload)
                    self.multiPointWaypointParameterUploaded.radius = self.waypointParameterRadiusForUpload
                    self.waypointParameterDurationBytesForUpload[0] = parsePacketBytes[41]
                    self.waypointParameterDurationBytesForUpload[1] = parsePacketBytes[42]
                    self.waypointParameterDurationForUpload = Tool.getShortFromBytes(self.waypointParameterDurationBytesForUpload)
                    self.multiPointWaypointParameterUploaded.duration = self.waypointParameterDurationForUpload
                    self.multiPointWaypointParameterUploaded.mode = parsePacketBytes[43]
                    if self.multiPointWaypointParameterUploaded.currentIndex == len(self.multiPointWaypointParameterUploadedList):
                        self.multiPointWaypointParameterUploadedList.add(self.multiPointWaypointParameterUploaded)
                self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
                self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(84))
                self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
                self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
                sendBroadcast(self.uploadingSubframeByteIntent)
            self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
            sendBroadcast(self.followIntent)
        elif self.subframeByte == int(90):
            if parsePacketBytes[37] == int(0) and parsePacketBytes[38] == int(0):
                self.followIntent = Intent()
                self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
                sendBroadcast(self.followIntent)
            else:
                self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                self.trackWaypointParameterUploaded = TrackFollowWaypointParameter()
                if parsePacketBytes[38] == int(0):
                    if self.trackWaypointParameterUploadedList == None:
                        self.trackWaypointParameterUploadedList = ArrayList()
                    else:
                        self.trackWaypointParameterUploadedList.clear()
                self.trackWaypointParameterUploaded.number = parsePacketBytes[37]
                self.trackWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                self.trackWaypointParameterUploaded.nextIndex = parsePacketBytes[39]
                self.trackLngForUplad = (float(Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[41], parsePacketBytes[42], parsePacketBytes[43], parsePacketBytes[44]])))) / 1.0E7
                self.trackWaypointParameterUploaded.longitude = self.trackLngForUplad
            self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
            self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(90))
            self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
            self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.uploadingSubframeByteIntent)
        elif self.subframeByte == int(91):
            if parsePacketBytes[37] == int(0) and parsePacketBytes[38] == int(0):
                self.followIntent = Intent()
                self.followIntent.setAction(Tool.WATCHBREAKUPLOADWAYPOINT_ACTION)
                sendBroadcast(self.followIntent)
            else:
                self.uploadingSubframeByteBundle.putByte("number", parsePacketBytes[37])
                self.uploadingSubframeByteBundle.putByte("currentindex", parsePacketBytes[38])
                self.trackWaypointParameterUploaded.number = parsePacketBytes[37]
                self.trackWaypointParameterUploaded.currentIndex = parsePacketBytes[38]
                self.trackAltForUpload = int((int(((float(Tool.getShortFromBytes(bytearray([parsePacketBytes[39], parsePacketBytes[40]])))) / 10.0))))
                self.trackWaypointParameterUploaded.altitude = self.trackAltForUpload
                self.trackLatForUpload = (float(Tool.byteArrayToInt_ZSY(bytearray([parsePacketBytes[41], parsePacketBytes[42], parsePacketBytes[43], parsePacketBytes[44]])))) / 1.0E7
                self.trackWaypointParameterUploaded.latitude = self.trackLatForUpload
                if self.trackWaypointParameterUploaded.currentIndex == len(self.trackWaypointParameterUploadedList):
                    self.trackWaypointParameterUploadedList.add(self.trackWaypointParameterUploaded)
            self.uploadingSubframeByteBundle.putInt("uploadingwaypointtype", self.uploadingWaypointTypeIndex)
            self.uploadingSubframeByteBundle.putByte("uploadingsubframe", int(91))
            self.uploadingSubframeByteIntent.putExtras(self.uploadingSubframeByteBundle)
            self.uploadingSubframeByteIntent.setAction(Tool.UPLOADINGSUBFRAMEBYTE_ACTION)
            sendBroadcast(self.uploadingSubframeByteIntent)
        if parsePacketBytes[36] == int(-48) or parsePacketBytes[36] == int(-47) or parsePacketBytes[36] == int(-46) or parsePacketBytes[36] == int(-45) or parsePacketBytes[36] == int(-44) or parsePacketBytes[36] == int(-38) or parsePacketBytes[36] == int(-37):
            self.vlcApplication.setSubframeFlag(0)
            self.vlcApplication.setReceivedSubframeByte(parsePacketBytes[36])
        self.dataForShowIntent.putExtras(self.dataForShowBundle)
        self.dataForShowIntent.setAction(Tool.DATAFORSHOW_ACTION)
        sendBroadcast(self.dataForShowIntent)

    def onDestroy(self):
        """ generated source for method onDestroy """
        super(MoniteSocketForReceiveService, self).onDestroy()
        if self.subframeByteSendReceiver != None:
            unregisterReceiver(self.subframeByteSendReceiver)
        if self.bluetoothStateChangeReceiver != None:
            unregisterReceiver(self.bluetoothStateChangeReceiver)
        if self.startMotorCommandReceiver != None:
            unregisterReceiver(self.startMotorCommandReceiver)
        if self.downloadingWaypointTypeReceiver != None:
            unregisterReceiver(self.downloadingWaypointTypeReceiver)
        if self.uploadingWaypointTypeReceiver != None:
            unregisterReceiver(self.uploadingWaypointTypeReceiver)
        if self.clearCommandBeforeCalibrateSensorOfUavReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateSensorOfUavReceiver)
        if self.clearCommandBeforeCalibrateMagneticOfUavReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateMagneticOfUavReceiver)
        if self.clearCommandBeforeCalibrateSensorOfModuleReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateSensorOfModuleReceiver)
        if self.clearCommandBeforeCalibrateMagneticOfModuleReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateMagneticOfModuleReceiver)
        if self.clearCommandBeforeCalibrateBarometerReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateBarometerReceiver)
        if self.clearCommandBeforeCalibrateStickNeutralReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateStickNeutralReceiver)
        if self.disconnectBluetoothReceiver != None:
            unregisterReceiver(self.disconnectBluetoothReceiver)
        if self.breakUploadWaypointReceiver != None:
            unregisterReceiver(self.breakUploadWaypointReceiver)
        if self.resetAllParamsValueReceiver != None:
            unregisterReceiver(self.resetAllParamsValueReceiver)
        if self.clearCommandBeforeCalibrateGimbalReceiver != None:
            unregisterReceiver(self.clearCommandBeforeCalibrateGimbalReceiver)
        if not (self.receivePacketExecutorService == None or self.receivePacketExecutorService.isShutdown()):
            self.receivePacketExecutorService.shutdown()
        self.closeBluetoothGatt()
        stopSelf()

service = MoniteSocketForReceiveService()
service.onBind(Intent())
def parser(x):
    del pending_broadcasts[:]
    service.parsePacketData(bytearray(x))
    x = dict(service.__dict__)
    for y in x.keys():
        if y.endswith('Receiver') or y in ('countThreadScheduledPool', 'vlcApplication', 'countPacketsThread'):
            del x[y]
    x['broadcasts'] = list(pending_broadcasts)
    del pending_broadcasts[:]
    return x
if __name__ == '__main__':
    print parser('\x00' * 47)