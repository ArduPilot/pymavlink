#!/usr/bin/env python
'''
convert ArduPilot log file to a CSV file for AVCRM
'''

from pymavlink import mavutil
import time
import csv
import re
from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--output", default="out.csv", help="output file name")
parser.add_argument("log", metavar="LOG")
args = parser.parse_args()

CSV_FIELDS = ["CUSTOM.updateTime", "CUSTOM.hSpeed", "CUSTOM.distance", "OSD.latitude", "OSD.longitude", "OSD.height", "OSD.xSpeed", "OSD.ySpeed", "OSD.zSpeed", "OSD.pitch", "OSD.roll", "OSD.yaw", "OSD.flycState", "OSD.flycCommand", "OSD.canIOCWork", "OSD.groundOrSky", "OSD.isMotorUp", "OSD.isSwaveWork", "OSD.goHomeStatus", "OSD.isImuPreheated", "OSD.isVisionUsed", "OSD.voltageWarning", "OSD.modeChannel", "OSD.compassError", "OSD.waveError", "OSD.gpsLevel", "OSD.batteryType", "OSD.isAcceletorOverRange", "OSD.isVibrating", "OSD.isBarometerDeadInAir", "OSD.isMotorBlocked", "OSD.isNotEnoughForce", "OSD.isPropellerCatapult", "OSD.isGoHomeHeightModified", "OSD.isOutOfLimit", "OSD.gpsNum", "OSD.flightAction", "OSD.flightAction", "OSD.motorStartFailedCause", "OSD.nonGPSCause", "OSD.isQuickSpin", "OSD.battery", "OSD.sWaveHeight", "OSD.flyTime", "OSD.motorRevolution", "OSD.flycVersion", "OSD.droneType", "OSD.imuInitFailReason", "OSD.motorFailReason", "OSD.ctrlDevice", "GIMBAL.pitch", "GIMBAL.roll", "GIMBAL.yaw", "GIMBAL.mode", "GIMBAL.rollAdjust", "GIMBAL.yawAngle", "GIMBAL.isAutoCalibration", "GIMBAL.autoCalibrationResult", "GIMBAL.isPitchInLimit", "GIMBAL.isRollInLimit", "GIMBAL.isYawInLimit", "GIMBAL.isStuck", "GIMBAL.version", "GIMBAL.isSingleClick", "GIMBAL.isDoubleClick", "GIMBAL.isTripleClick", "RC.aileron", "RC.elevator", "RC.throttle", "RC.rudder", "RC.gimbal", "RC.goHome", "RC.mode", "RC.wheelOffset", "RC.record", "RC.shutter", "RC.playback", "RC.custom1", "RC.custom2", "CENTER_BATTERY.relativeCapacity", "CENTER_BATTERY.currentPV", "CENTER_BATTERY.currentCapacity", "CENTER_BATTERY.fullCapacity", "CENTER_BATTERY.life", "CENTER_BATTERY.loopNum", "CENTER_BATTERY.errorType", "CENTER_BATTERY.current", "CENTER_BATTERY.voltageCell1", "CENTER_BATTERY.voltageCell2", "CENTER_BATTERY.voltageCell3", "CENTER_BATTERY.voltageCell4", "CENTER_BATTERY.voltageCell5", "CENTER_BATTERY.voltageCell6", "CENTER_BATTERY.serialNo", "CENTER_BATTERY.productDate", "CENTER_BATTERY.temperature", "CENTER_BATTERY.connStatus", "CENTER_BATTERY.totalStudyCycle", "CENTER_BATTERY.lastStudyCycle", "CENTER_BATTERY.isNeedStudy", "CENTER_BATTERY.isBatteryOnCharge", "SMART_BATTERY.usefulTime", "SMART_BATTERY.goHomeTime", "SMART_BATTERY.landTime", "SMART_BATTERY.goHomeBattery", "SMART_BATTERY.landBattery", "SMART_BATTERY.safeFlyRadius", "SMART_BATTERY.volumeConsume", "SMART_BATTERY.status", "SMART_BATTERY.goHomeStatus", "SMART_BATTERY.goHomeCountdown", "SMART_BATTERY.voltage", "SMART_BATTERY.battery", "SMART_BATTERY.lowWarning", "SMART_BATTERY.lowWarningGoHome", "SMART_BATTERY.seriousLowWarning", "SMART_BATTERY.seriousLowWarningLanding", "SMART_BATTERY.voltagePercent", "DEFORM.isDeformProtected", "DEFORM.deformStatus", "DEFORM.deformMode", "HOME.latitude", "HOME.longitude", "HOME.height", "HOME.isHomeRecord", "HOME.goHomeMode", "HOME.aircraftHeadDirection", "HOME.isDynamicHomePointEnabled", "HOME.goHomeStatus", "HOME.hasGoHome", "HOME.compassCeleStatus", "HOME.isCompassCeleing", "HOME.isBeginnerMode", "HOME.isIOCEnabled", "HOME.iocMode", "HOME.goHomeHeight", "HOME.courseLockAngle", "HOME.dataRecorderStatus", "HOME.dataRecorderRemainCapacity", "HOME.dataRecorderRemainTime", "HOME.dataRecorderFileIndex", "HOME.maxAllowedHeight", "RECOVER.droneType", "RECOVER.appType", "RECOVER.appVersion", "RECOVER.aircraftSn", "RECOVER.aircraftName", "RECOVER.activeTimestamp", "RECOVER.cameraSn", "RECOVER.rcSn", "RECOVER.batterySn", "FIRMWARE.version", "COMPONENT.cameraSn", "COMPONENT.aircraftSn", "COMPONENT.rcSn", "COMPONENT.batterySn", "DETAILS.street", "DETAILS.citypart", "DETAILS.city", "DETAILS.area", "DETAILS.isFavorite", "DETAILS.isNew", "DETAILS.needUpload", "DETAILS.recordLineCount", "DETAILS.timestamp", "DETAILS.latitude", "DETAILS.longitude", "DETAILS.totalDistance", "DETAILS.totalTime", "DETAILS.maxHeight", "DETAILS.maxHorizontalSpeed", "DETAILS.maxVerticalSpeed", "DETAILS.photoNum", "DETAILS.videoTime", "DETAILS.activeTimestamp", "DETAILS.aircraftName", "DETAILS.aircraftSn", "DETAILS.cameraSn", "DETAILS.rcSn", "DETAILS.batterySn", "DETAILS.appType", "DETAILS.appVersion", "APP_GPS.latitude", "APP_GPS.longitude", "APP_GPS.accuracy", "APP_TIP.tip", "APP_WARN.warn", "APP_SER_WARN.warn" ]

mlog = mavutil.mavlink_connection(args.log)

msg_types = [
    'POS', 'VFR_HUD', 'ATT', 'ATTITUDE', 'MODE', 'HEARTBEAT', 'SYS_STATUS', 'BATTERY_STATUS', 'BATT', 'HOME', 'HOME_POSITION',
    'GPS', 'GPA', 'GPS_RAW_INT', 'XKF1', 'RCIN', 'ORGN', 'GLOBAL_POSITION_INT', 'VER', 'AUTOPILOT_VERSION', 'MSG', 'STATUSTEXT',
    'PSC', 'GIMBAL_DEVICE_INFORMATION', 'AHR2'
]

last_emit = None

gps_status = 0

# pre-fill fields
fields = {}
for f in CSV_FIELDS:
    fields[f] = ''

def time_format(tnow):
    '''format time for CSV CUSTOM.updateTime field'''
    t1 = time.strftime("%Y/%m/%d %H:%M:%S", time.gmtime(tnow))
    t1 += ".%03d" % (int(tnow*1000)%1000)
    return t1

cfile = open(args.output, 'w', newline='')
cwriter = csv.DictWriter(cfile, fieldnames=CSV_FIELDS)
cwriter.writeheader()

while True:
    m = mlog.recv_match(type=msg_types, condition=args.condition)
    if m is None:
        break
    tnow = m._timestamp
    fields['CUSTOM.updateTime'] = time_format(tnow)
    if last_emit is None:
        last_emit = tnow
    mtype = m.get_type()

    # Bin
    if mtype == 'GPS' and getattr(m,'I',0) == 0:
        gps_status = m.Status
        # fields['APP_GPS.latitude'] = m.Lat
        # fields['APP_GPS.longitude'] = m.Lng
        fields['CUSTOM.hSpeed'] = m.Spd
        fields['OSD.gpsLevel'] = m.Status
        fields['OSD.gpsNum'] = m.NSats
        fields['OSD.flyTime'] = m.TimeUS*1.0e-6

    # TLog
    if mtype == 'GPS_RAW_INT':
        gps_status = m.fix_type
        # fields['APP_GPS.latitude'] = m.lat*1.0e-7
        # fields['APP_GPS.longitude'] = m.lon*1.0e-7
        fields['OSD.gpsNum'] = m.satellites_visible
        fields['OSD.flyTime'] = m.time_usec*1.0e-6

    # Bin
    if mtype == 'POS':
        fields['OSD.latitude'] = m.Lat
        fields['OSD.longitude'] = m.Lng
        fields['OSD.height'] = m.RelHomeAlt # OR use RelOriginAlt ...

    # TLog
    if mtype == 'GLOBAL_POSITION_INT':
        fields['OSD.latitude'] = m.lat*1.0e-7
        fields['OSD.longitude'] = m.lon*1.0e-7
        fields['OSD.height'] = m.relative_alt*1.0e-3
        fields['OSD.xSpeed'] = m.vx*1.0e-2
        fields['OSD.ySpeed'] = m.vy*1.0e-2
        fields['OSD.zSpeed'] = m.vz*1.0e-2

    if mtype == 'GPA' and getattr(m,'I',0) == 0:
        fields['APP_GPS.accuracy'] = m.HAcc

    if mtype == 'XKF1' and getattr(m,'C',0) == 0:
        fields['OSD.xSpeed'] = m.VN
        fields['OSD.ySpeed'] = m.VE
        fields['OSD.zSpeed'] = m.VD

    # if mtype == 'PSC':
    #     pass

    if mtype == 'ATT':
        fields['OSD.roll'] = m.Roll
        fields['OSD.pitch'] = m.Pitch
        fields['OSD.yaw'] = m.Yaw

    if mtype == 'AHR2':
        fields['OSD.roll'] = m.Roll
        fields['OSD.pitch'] = m.Pitch
        fields['OSD.yaw'] = m.Yaw

    if mtype == 'RCIN':
        fields['RC.aileron'] = m.C1
        fields['RC.elevator'] = m.C2
        fields['RC.throttle'] = m.C3
        fields['RC.rudder'] = m.C4

    if mtype == 'ORGN' and getattr(m,'Type',0) == 0:
        fields['HOME.latitude'] = m.Lat
        fields['HOME.longitude'] = m.Lng
        fields['HOME.height'] = m.Alt

    if mtype == 'VFR_HUD':
        fields['CUSTOM.hSpeed'] = m.groundspeed

    if mtype == 'VER':
        fields['FIRMWARE.version'] = m.FWS

    if mtype == 'AUTOPILOT_VERSION':
        fields['FIRMWARE.version'] = m.flight_sw_version

    if mtype == 'GIMBAL_DEVICE_INFORMATION':
        fields['GIMBAL.version'] = m.firmware_version

    if mtype == 'MSG':
        if re.search("^[a-zA-Z]*( [0-9a-fA-F]{8}){3}$", m.Message):
            fields['DETAILS.aircraftSn'] = m.Message
        else:
            fields['APP_TIP.tip'] = fields['APP_TIP.tip'] + ("\n" if fields['APP_TIP.tip'] != '' else '') + m.Message

    if mtype == 'STATUSTEXT':
        if re.search("^[a-zA-Z]*( [0-9a-fA-F]{8}){3}$", m.text):
            fields['DETAILS.aircraftSn'] = m.text
        else:
            fields['APP_TIP.tip'] = fields['APP_TIP.tip'] + ("\n" if fields['APP_TIP.tip'] != '' else '') + m.text

    if tnow - last_emit >= 0.2:
        cwriter.writerow(fields)
        last_emit = tnow
        fields['APP_TIP.tip'] = ''

cfile.close()

