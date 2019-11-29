#!/usr/bin/env python
'''
useful extra functions for use by mavlink clients

Copyright Andrew Tridgell 2011
Released under GNU GPL version 3 or later
'''
from __future__ import print_function
from __future__ import absolute_import
from builtins import object

from math import *

try:
    # in case numpy isn't installed
    from .quaternion import Quaternion
except:
    pass

try:
    # rotmat doesn't work on Python3.2 yet
    from .rotmat import Vector3, Matrix3
except Exception:
    pass


def kmh(mps):
    '''convert m/s to Km/h'''
    return mps*3.6

def altitude(SCALED_PRESSURE, ground_pressure=None, ground_temp=None):
    '''calculate barometric altitude'''
    from . import mavutil
    self = mavutil.mavfile_global
    if ground_pressure is None:
        if self.param('GND_ABS_PRESS', None) is None:
            return 0
        ground_pressure = self.param('GND_ABS_PRESS', 1)
    if ground_temp is None:
        ground_temp = self.param('GND_TEMP', 0)
    scaling = ground_pressure / (SCALED_PRESSURE.press_abs*100.0)
    temp = ground_temp + 273.15
    return log(scaling) * temp * 29271.267 * 0.001

def altitude2(SCALED_PRESSURE, ground_pressure=None, ground_temp=None):
    '''calculate barometric altitude'''
    from . import mavutil
    self = mavutil.mavfile_global
    if ground_pressure is None:
        if self.param('GND_ABS_PRESS', None) is None:
            return 0
        ground_pressure = self.param('GND_ABS_PRESS', 1)
    if ground_temp is None:
        ground_temp = self.param('GND_TEMP', 0)
    scaling = SCALED_PRESSURE.press_abs*100.0 / ground_pressure
    temp = ground_temp + 273.15
    return 153.8462 * temp * (1.0 - exp(0.190259 * log(scaling)))

def mag_heading(RAW_IMU, ATTITUDE, declination=None, SENSOR_OFFSETS=None, ofs=None):
    '''calculate heading from raw magnetometer'''
    if declination is None:
        from . import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z

    # go via a DCM matrix to match the APM calculation
    dcm_matrix = rotation(ATTITUDE)
    cos_pitch_sq = 1.0-(dcm_matrix.c.x*dcm_matrix.c.x)
    headY = mag_y * dcm_matrix.c.z - mag_z * dcm_matrix.c.y
    headX = mag_x * cos_pitch_sq - dcm_matrix.c.x * (mag_y * dcm_matrix.c.y + mag_z * dcm_matrix.c.z)

    heading = degrees(atan2(-headY,headX)) + declination
    if heading < 0:
        heading += 360
    return heading

def mag_heading_motors(RAW_IMU, ATTITUDE, declination, SENSOR_OFFSETS, ofs, SERVO_OUTPUT_RAW, motor_ofs):
    '''calculate heading from raw magnetometer'''
    ofs = get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs)

    if declination is None:
        from . import mavutil
        declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z

    headX = mag_x*cos(ATTITUDE.pitch) + mag_y*sin(ATTITUDE.roll)*sin(ATTITUDE.pitch) + mag_z*cos(ATTITUDE.roll)*sin(ATTITUDE.pitch)
    headY = mag_y*cos(ATTITUDE.roll) - mag_z*sin(ATTITUDE.roll)
    heading = degrees(atan2(-headY,headX)) + declination
    if heading < 0:
        heading += 360
    return heading

def mag_field(RAW_IMU, SENSOR_OFFSETS=None, ofs=None):
    '''calculate magnetic field strength from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag
    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def mag_field_df(MAG, ofs=None):
    '''calculate magnetic field strength from raw magnetometer (dataflash version)'''
    mag = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)
    offsets = Vector3(MAG.OfsX, MAG.OfsY, MAG.OfsZ)
    if ofs is not None:
        mag = (mag - offsets) + Vector3(ofs[0], ofs[1], ofs[2])
    return mag.length()

def get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs):
    '''calculate magnetic field strength from raw magnetometer'''
    from . import mavutil
    self = mavutil.mavfile_global

    m = SERVO_OUTPUT_RAW
    motor_pwm = m.servo1_raw + m.servo2_raw + m.servo3_raw + m.servo4_raw
    motor_pwm *= 0.25
    rc3_min = self.param('RC3_MIN', 1100)
    rc3_max = self.param('RC3_MAX', 1900)
    motor = (motor_pwm - rc3_min) / (rc3_max - rc3_min)
    if motor > 1.0:
        motor = 1.0
    if motor < 0.0:
        motor = 0.0

    motor_offsets0 = motor_ofs[0] * motor
    motor_offsets1 = motor_ofs[1] * motor
    motor_offsets2 = motor_ofs[2] * motor
    ofs = (ofs[0] + motor_offsets0, ofs[1] + motor_offsets1, ofs[2] + motor_offsets2)

    return ofs

def mag_field_motors(RAW_IMU, SENSOR_OFFSETS, ofs, SERVO_OUTPUT_RAW, motor_ofs):
    '''calculate magnetic field strength from raw magnetometer'''
    mag_x = RAW_IMU.xmag
    mag_y = RAW_IMU.ymag
    mag_z = RAW_IMU.zmag

    ofs = get_motor_offsets(SERVO_OUTPUT_RAW, ofs, motor_ofs)

    if SENSOR_OFFSETS is not None and ofs is not None:
        mag_x += ofs[0] - SENSOR_OFFSETS.mag_ofs_x
        mag_y += ofs[1] - SENSOR_OFFSETS.mag_ofs_y
        mag_z += ofs[2] - SENSOR_OFFSETS.mag_ofs_z
    return sqrt(mag_x**2 + mag_y**2 + mag_z**2)

def angle_diff(angle1, angle2):
    '''show the difference between two angles in degrees'''
    ret = angle1 - angle2
    if ret > 180:
        ret -= 360;
    if ret < -180:
        ret += 360
    return ret

average_data = {}

def average(var, key, N):
    '''average over N points'''
    global average_data
    if not key in average_data:
        average_data[key] = [var]*N
        return var
    average_data[key].pop(0)
    average_data[key].append(var)
    return sum(average_data[key])/N

derivative_data = {}

def second_derivative_5(var, key):
    '''5 point 2nd derivative'''
    global derivative_data
    from . import mavutil
    tnow = mavutil.mavfile_global.timestamp

    if not key in derivative_data:
        derivative_data[key] = (tnow, [var]*5)
        return 0
    (last_time, data) = derivative_data[key]
    data.pop(0)
    data.append(var)
    derivative_data[key] = (tnow, data)
    h = (tnow - last_time)
    # N=5 2nd derivative from
    # http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    ret = ((data[4] + data[0]) - 2*data[2]) / (4*h**2)
    return ret

def second_derivative_9(var, key):
    '''9 point 2nd derivative'''
    global derivative_data
    from . import mavutil
    tnow = mavutil.mavfile_global.timestamp

    if not key in derivative_data:
        derivative_data[key] = (tnow, [var]*9)
        return 0
    (last_time, data) = derivative_data[key]
    data.pop(0)
    data.append(var)
    derivative_data[key] = (tnow, data)
    h = (tnow - last_time)
    # N=5 2nd derivative from
    # http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    f = data
    ret = ((f[8] + f[0]) + 4*(f[7] + f[1]) + 4*(f[6]+f[2]) - 4*(f[5]+f[3]) - 10*f[4])/(64*h**2)
    return ret

lowpass_data = {}

def lowpass(var, key, factor):
    '''a simple lowpass filter'''
    global lowpass_data
    if not key in lowpass_data:
        lowpass_data[key] = var
    else:
        lowpass_data[key] = factor*lowpass_data[key] + (1.0 - factor)*var
    return lowpass_data[key]

last_diff = {}

def diff(var, key):
    '''calculate differences between values'''
    global last_diff
    ret = 0
    if not key in last_diff:
        last_diff[key] = var
        return 0
    ret = var - last_diff[key]
    last_diff[key] = var
    return ret

last_delta = {}

def delta(var, key, tusec=None):
    '''calculate slope'''
    global last_delta
    if tusec is not None:
        tnow = tusec * 1.0e-6
    else:
        from . import mavutil
        tnow = mavutil.mavfile_global.timestamp
    ret = 0
    if key in last_delta:
        (last_v, last_t, last_ret) = last_delta[key]
        if last_t == tnow:
            return last_ret
        if tnow == last_t:
            ret = 0
        else:
            ret = (var - last_v) / (tnow - last_t)
    last_delta[key] = (var, tnow, ret)
    return ret

def delta_angle(var, key, tusec=None):
    '''calculate slope of an angle'''
    global last_delta
    if tusec is not None:
        tnow = tusec * 1.0e-6
    else:
        from . import mavutil
        tnow = mavutil.mavfile_global.timestamp
    dv = 0
    ret = 0
    if key in last_delta:
        (last_v, last_t, last_ret) = last_delta[key]
        if last_t == tnow:
            return last_ret
        if tnow == last_t:
            ret = 0
        else:
            dv = var - last_v
            if dv > 180:
                dv -= 360
            if dv < -180:
                dv += 360
            ret = dv / (tnow - last_t)
    last_delta[key] = (var, tnow, ret)
    return ret

def roll_estimate(RAW_IMU,GPS_RAW_INT=None,ATTITUDE=None,SENSOR_OFFSETS=None, ofs=None, mul=None,smooth=0.7):
    '''estimate roll from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if ATTITUDE is not None and GPS_RAW_INT is not None:
        ry -= ATTITUDE.yawspeed * GPS_RAW_INT.vel*0.01
        rz += ATTITUDE.pitchspeed * GPS_RAW_INT.vel*0.01
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(-asin(ry/sqrt(rx**2+ry**2+rz**2))),'_roll',smooth)

def pitch_estimate(RAW_IMU, GPS_RAW_INT=None,ATTITUDE=None, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    rx = RAW_IMU.xacc * 9.81 / 1000.0
    ry = RAW_IMU.yacc * 9.81 / 1000.0
    rz = RAW_IMU.zacc * 9.81 / 1000.0
    if ATTITUDE is not None and GPS_RAW_INT is not None:
        ry -= ATTITUDE.yawspeed * GPS_RAW_INT.vel*0.01
        rz += ATTITUDE.pitchspeed * GPS_RAW_INT.vel*0.01
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return lowpass(degrees(asin(rx/sqrt(rx**2+ry**2+rz**2))),'_pitch',smooth)

def rotation(ATTITUDE):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(ATTITUDE.roll, ATTITUDE.pitch, ATTITUDE.yaw)
    return r

def mag_rotation(RAW_IMU, inclination, declination):
    '''return an attitude rotation matrix that is consistent with the current mag
       vector'''
    m_body = Vector3(RAW_IMU.xmag, RAW_IMU.ymag, RAW_IMU.zmag)
    m_earth = Vector3(m_body.length(), 0, 0)

    r = Matrix3()
    r.from_euler(0, -radians(inclination), radians(declination))
    m_earth = r * m_earth

    r.from_two_vectors(m_earth, m_body)
    return r

def mag_yaw(RAW_IMU, inclination, declination):
    '''estimate yaw from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    y = degrees(y)
    if y < 0:
        y += 360
    return y

def mag_pitch(RAW_IMU, inclination, declination):
    '''estimate pithc from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    return degrees(p)

def mag_roll(RAW_IMU, inclination, declination):
    '''estimate roll from mag'''
    m = mag_rotation(RAW_IMU, inclination, declination)
    (r, p, y) = m.to_euler()
    return degrees(r)

def gravity(RAW_IMU, SENSOR_OFFSETS=None, ofs=None, mul=None, smooth=0.7):
    '''estimate pitch from accelerometer'''
    if hasattr(RAW_IMU, 'xacc'):
        rx = RAW_IMU.xacc * 9.81 / 1000.0
        ry = RAW_IMU.yacc * 9.81 / 1000.0
        rz = RAW_IMU.zacc * 9.81 / 1000.0
    else:
        rx = RAW_IMU.AccX
        ry = RAW_IMU.AccY
        rz = RAW_IMU.AccZ
    if SENSOR_OFFSETS is not None and ofs is not None:
        rx += SENSOR_OFFSETS.accel_cal_x
        ry += SENSOR_OFFSETS.accel_cal_y
        rz += SENSOR_OFFSETS.accel_cal_z
        rx -= ofs[0]
        ry -= ofs[1]
        rz -= ofs[2]
        if mul is not None:
            rx *= mul[0]
            ry *= mul[1]
            rz *= mul[2]
    return sqrt(rx**2+ry**2+rz**2)



def pitch_sim(SIMSTATE, GPS_RAW):
    '''estimate pitch from SIMSTATE accels'''
    xacc = SIMSTATE.xacc - lowpass(delta(GPS_RAW.v,"v")*6.6, "v", 0.9)
    zacc = SIMSTATE.zacc
    zacc += SIMSTATE.ygyro * GPS_RAW.v;
    if xacc/zacc >= 1:
        return 0
    if xacc/zacc <= -1:
        return -0
    return degrees(-asin(xacc/zacc))

def distance_two(GPS_RAW1, GPS_RAW2, horizontal=True):
    '''distance between two points'''
    if hasattr(GPS_RAW1, 'Lat'):
        lat1 = radians(GPS_RAW1.Lat)
        lat2 = radians(GPS_RAW2.Lat)
        lon1 = radians(GPS_RAW1.Lng)
        lon2 = radians(GPS_RAW2.Lng)
        alt1 = GPS_RAW1.Alt
        alt2 = GPS_RAW2.Alt
    elif hasattr(GPS_RAW1, 'cog'):
        lat1 = radians(GPS_RAW1.lat)*1.0e-7
        lat2 = radians(GPS_RAW2.lat)*1.0e-7
        lon1 = radians(GPS_RAW1.lon)*1.0e-7
        lon2 = radians(GPS_RAW2.lon)*1.0e-7
        alt1 = GPS_RAW1.alt*0.001
        alt2 = GPS_RAW2.alt*0.001
    else:
        lat1 = radians(GPS_RAW1.lat)
        lat2 = radians(GPS_RAW2.lat)
        lon1 = radians(GPS_RAW1.lon)
        lon2 = radians(GPS_RAW2.lon)
        alt1 = GPS_RAW1.alt*0.001
        alt2 = GPS_RAW2.alt*0.001
    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
    ground_dist = 6371 * 1000 * c
    if horizontal:
        return ground_dist
    return sqrt(ground_dist**2 + (alt2-alt1)**2)


first_fix = None

def distance_home(GPS_RAW):
    '''distance from first fix point'''
    global first_fix
    if (hasattr(GPS_RAW, 'fix_type') and GPS_RAW.fix_type < 2) or \
       (hasattr(GPS_RAW, 'Status')   and GPS_RAW.Status   < 2):
        return 0

    if first_fix is None:
        first_fix = GPS_RAW
        return 0
    return distance_two(GPS_RAW, first_fix)

def sawtooth(ATTITUDE, amplitude=2.0, period=5.0):
    '''sawtooth pattern based on uptime'''
    mins = (ATTITUDE.usec * 1.0e-6)/60
    p = fmod(mins, period*2)
    if p < period:
        return amplitude * (p/period)
    return amplitude * (period - (p-period))/period

def rate_of_turn(speed, bank):
    '''return expected rate of turn in degrees/s for given speed in m/s and
       bank angle in degrees'''
    if abs(speed) < 2 or abs(bank) > 80:
        return 0
    ret = degrees(9.81*tan(radians(bank))/speed)
    return ret

def wingloading(bank):
    '''return expected wing loading factor for a bank angle in radians'''
    return 1.0/cos(bank)

def airspeed(VFR_HUD, ratio=None, used_ratio=None, offset=None):
    '''recompute airspeed with a different ARSPD_RATIO'''
    from . import mavutil
    mav = mavutil.mavfile_global
    if ratio is None:
        ratio = 1.9936 # APM default
    if used_ratio is None:
        if 'ARSPD_RATIO' in mav.params:
            used_ratio = mav.params['ARSPD_RATIO']
        else:
            print("no ARSPD_RATIO in mav.params")
            used_ratio = ratio
    if hasattr(VFR_HUD,'airspeed'):
        airspeed = VFR_HUD.airspeed
    else:
        airspeed = VFR_HUD.Airspeed
    airspeed_pressure = (airspeed**2) / used_ratio
    if offset is not None:
        airspeed_pressure += offset
        if airspeed_pressure < 0:
            airspeed_pressure = 0
    airspeed = sqrt(airspeed_pressure * ratio)
    return airspeed

def EAS2TAS(ARSP,GPS,BARO,ground_temp=25):
    '''EAS2TAS from ARSP.Temp'''
    tempK = ground_temp + 273.15 - 0.0065 * GPS.Alt
    return sqrt(1.225 / (BARO.Press / (287.26 * tempK)))


def airspeed_ratio(VFR_HUD):
    '''recompute airspeed with a different ARSPD_RATIO'''
    from . import mavutil
    mav = mavutil.mavfile_global
    airspeed_pressure = (VFR_HUD.airspeed**2) / ratio
    airspeed = sqrt(airspeed_pressure * ratio)
    return airspeed

def airspeed_voltage(VFR_HUD, ratio=None):
    '''back-calculate the voltage the airspeed sensor must have seen'''
    from . import mavutil
    mav = mavutil.mavfile_global
    if ratio is None:
        ratio = 1.9936 # APM default
    if 'ARSPD_RATIO' in mav.params:
        used_ratio = mav.params['ARSPD_RATIO']
    else:
        used_ratio = ratio
    if 'ARSPD_OFFSET' in mav.params:
        offset = mav.params['ARSPD_OFFSET']
    else:
        return -1
    airspeed_pressure = (pow(VFR_HUD.airspeed,2)) / used_ratio
    raw = airspeed_pressure + offset
    SCALING_OLD_CALIBRATION = 204.8
    voltage = 5.0 * raw / 4096
    return voltage


def earth_rates(ATTITUDE):
    '''return angular velocities in earth frame'''
    from math import sin, cos, tan, fabs

    p     = ATTITUDE.rollspeed
    q     = ATTITUDE.pitchspeed
    r     = ATTITUDE.yawspeed
    phi   = ATTITUDE.roll
    theta = ATTITUDE.pitch
    psi   = ATTITUDE.yaw

    phiDot   = p + tan(theta)*(q*sin(phi) + r*cos(phi))
    thetaDot = q*cos(phi) - r*sin(phi)
    if fabs(cos(theta)) < 1.0e-20:
        theta += 1.0e-10
    psiDot   = (q*sin(phi) + r*cos(phi))/cos(theta)
    return (phiDot, thetaDot, psiDot)

def roll_rate(ATTITUDE):
    '''return roll rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return phiDot

def pitch_rate(ATTITUDE):
    '''return pitch rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return thetaDot

def yaw_rate(ATTITUDE):
    '''return yaw rate in earth frame'''
    (phiDot, thetaDot, psiDot) = earth_rates(ATTITUDE)
    return psiDot


def gps_velocity(GLOBAL_POSITION_INT):
    '''return GPS velocity vector'''
    return Vector3(GLOBAL_POSITION_INT.vx, GLOBAL_POSITION_INT.vy, GLOBAL_POSITION_INT.vz) * 0.01


def gps_velocity_old(GPS_RAW_INT):
    '''return GPS velocity vector'''
    return Vector3(GPS_RAW_INT.vel*0.01*cos(radians(GPS_RAW_INT.cog*0.01)),
                   GPS_RAW_INT.vel*0.01*sin(radians(GPS_RAW_INT.cog*0.01)), 0)

def gps_velocity_body(GPS_RAW_INT, ATTITUDE):
    '''return GPS velocity vector in body frame'''
    r = rotation(ATTITUDE)
    return r.transposed() * Vector3(GPS_RAW_INT.vel*0.01*cos(radians(GPS_RAW_INT.cog*0.01)),
                                    GPS_RAW_INT.vel*0.01*sin(radians(GPS_RAW_INT.cog*0.01)),
                                    -tan(ATTITUDE.pitch)*GPS_RAW_INT.vel*0.01)

def earth_accel(RAW_IMU,ATTITUDE):
    '''return earth frame acceleration vector'''
    r = rotation(ATTITUDE)
    accel = Vector3(RAW_IMU.xacc, RAW_IMU.yacc, RAW_IMU.zacc) * 9.81 * 0.001
    return r * accel

def earth_gyro(RAW_IMU,ATTITUDE):
    '''return earth frame gyro vector'''
    r = rotation(ATTITUDE)
    accel = Vector3(degrees(RAW_IMU.xgyro), degrees(RAW_IMU.ygyro), degrees(RAW_IMU.zgyro)) * 0.001
    return r * accel

def airspeed_energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD):
    '''return airspeed energy error matching APM internals
    This is positive when we are going too slow
    '''
    aspeed_cm = VFR_HUD.airspeed*100
    target_airspeed = NAV_CONTROLLER_OUTPUT.aspd_error + aspeed_cm
    airspeed_energy_error = ((target_airspeed*target_airspeed) - (aspeed_cm*aspeed_cm))*0.00005
    return airspeed_energy_error


def energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD):
    '''return energy error matching APM internals
    This is positive when we are too low or going too slow
    '''
    aspeed_energy_error = airspeed_energy_error(NAV_CONTROLLER_OUTPUT, VFR_HUD)
    alt_error = NAV_CONTROLLER_OUTPUT.alt_error*100
    energy_error = aspeed_energy_error + alt_error*0.098
    return energy_error

def rover_turn_circle(SERVO_OUTPUT_RAW):
    '''return turning circle (diameter) in meters for steering_angle in degrees
    '''

    # this matches Toms slash
    max_wheel_turn = 35
    wheelbase      = 0.335
    wheeltrack     = 0.296

    steering_angle = max_wheel_turn * (SERVO_OUTPUT_RAW.servo1_raw - 1500) / 400.0
    theta = radians(steering_angle)
    return (wheeltrack/2) + (wheelbase/sin(theta))

def rover_yaw_rate(VFR_HUD, SERVO_OUTPUT_RAW):
    '''return yaw rate in degrees/second given steering_angle and speed'''
    max_wheel_turn=35
    speed = VFR_HUD.groundspeed
    # assume 1100 to 1900 PWM on steering
    steering_angle = max_wheel_turn * (SERVO_OUTPUT_RAW.servo1_raw - 1500) / 400.0
    if abs(steering_angle) < 1.0e-6 or abs(speed) < 1.0e-6:
        return 0
    d = rover_turn_circle(SERVO_OUTPUT_RAW)
    c = pi * d
    t = c / speed
    rate = 360.0 / t
    return rate

def rover_lat_accel(VFR_HUD, SERVO_OUTPUT_RAW):
    '''return lateral acceleration in m/s/s'''
    speed = VFR_HUD.groundspeed
    yaw_rate = rover_yaw_rate(VFR_HUD, SERVO_OUTPUT_RAW)
    accel = radians(yaw_rate) * speed
    return accel


def demix1(servo1, servo2, gain=0.5):
    '''de-mix a mixed servo output'''
    s1 = servo1 - 1500
    s2 = servo2 - 1500
    out1 = (s1+s2)*gain
    out2 = (s1-s2)*gain
    return out1+1500

def demix2(servo1, servo2, gain=0.5):
    '''de-mix a mixed servo output'''
    s1 = servo1 - 1500
    s2 = servo2 - 1500
    out1 = (s1+s2)*gain
    out2 = (s1-s2)*gain
    return out2+1500

def mixer(servo1, servo2, mixtype=1, gain=0.5):
    '''mix two servos'''
    s1 = servo1 - 1500
    s2 = servo2 - 1500
    v1 = (s1-s2)*gain
    v2 = (s1+s2)*gain
    if mixtype == 2:
        v2 = -v2
    elif mixtype == 3:
        v1 = -v1
    elif mixtype == 4:
        v1 = -v1
        v2 = -v2
    if v1 > 600:
        v1 = 600
    elif v1 < -600:
        v1 = -600
    if v2 > 600:
        v2 = 600
    elif v2 < -600:
        v2 = -600
    return (1500+v1,1500+v2)

def mix1(servo1, servo2, mixtype=1, gain=0.5):
    '''de-mix a mixed servo output'''
    (v1,v2) = mixer(servo1, servo2, mixtype=mixtype, gain=gain)
    return v1

def mix2(servo1, servo2, mixtype=1, gain=0.5):
    '''de-mix a mixed servo output'''
    (v1,v2) = mixer(servo1, servo2, mixtype=mixtype, gain=gain)
    return v2

def wrap_180(angle):
    if angle > 180:
        angle -= 360.0
    if angle < -180:
        angle += 360.0
    return angle


def wrap_360(angle):
    if angle > 360:
        angle -= 360.0
    if angle < 0:
        angle += 360.0
    return angle

class DCM_State(object):
    '''DCM state object'''
    def __init__(self, roll, pitch, yaw):
        self.dcm = Matrix3()
        self.dcm2 = Matrix3()
        self.dcm.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.dcm2.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.mag = Vector3()
        self.gyro = Vector3()
        self.accel = Vector3()
        self.gps = None
        self.rate = 50.0
        self.kp = 0.2
        self.kp_yaw = 0.3
        self.omega_P = Vector3()
        self.omega_P_yaw = Vector3()
        self.omega_I = Vector3() # (-0.00199045287445, -0.00653007719666, -0.00714212376624)
        self.omega_I_sum = Vector3()
        self.omega_I_sum_time = 0
        self.omega = Vector3()
        self.ra_sum = Vector3()
        self.last_delta_angle = Vector3()
        self.last_velocity = Vector3()
        (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()
        (self.roll2, self.pitch2, self.yaw2) = self.dcm2.to_euler()

    def update(self, gyro, accel, mag, GPS):
        if self.gyro != gyro or self.accel != accel:
            delta_angle = old_div((gyro+self.omega_I), self.rate)
            self.dcm.rotate(delta_angle)
            correction = self.last_delta_angle % delta_angle
            #print (delta_angle - self.last_delta_angle) * 58.0
            corrected_delta = delta_angle + 0.0833333 * correction
            self.dcm2.rotate(corrected_delta)
            self.last_delta_angle = delta_angle

            self.dcm.normalize()
            self.dcm2.normalize()

            self.gyro = gyro
            self.accel = accel
            (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()
            (self.roll2, self.pitch2, self.yaw2) = self.dcm2.to_euler()

dcm_state = None

def DCM_update(IMU, ATT, MAG, GPS):
    '''implement full DCM system'''
    global dcm_state
    if dcm_state is None:
        dcm_state = DCM_State(ATT.Roll, ATT.Pitch, ATT.Yaw)

    mag   = Vector3(MAG.MagX, MAG.MagY, MAG.MagZ)
    gyro  = Vector3(IMU.GyrX, IMU.GyrY, IMU.GyrZ)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    accel2 = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    dcm_state.update(gyro, accel, mag, GPS)
    return dcm_state

class PX4_State(object):
    '''PX4 DCM state object'''
    def __init__(self, roll, pitch, yaw, timestamp):
        self.dcm = Matrix3()
        self.dcm.from_euler(radians(roll), radians(pitch), radians(yaw))
        self.gyro = Vector3()
        self.accel = Vector3()
        self.timestamp = timestamp
        (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()

    def update(self, gyro, accel, timestamp):
        if self.gyro != gyro or self.accel != accel:
            delta_angle = gyro * (timestamp - self.timestamp)
            self.timestamp = timestamp
            self.dcm.rotate(delta_angle)
            self.dcm.normalize()
            self.gyro = gyro
            self.accel = accel
            (self.roll, self.pitch, self.yaw) = self.dcm.to_euler()

px4_state = None

def PX4_update(IMU, ATT):
    '''implement full DCM using PX4 native SD log data'''
    global px4_state
    if px4_state is None:
        px4_state = PX4_State(degrees(ATT.Roll), degrees(ATT.Pitch), degrees(ATT.Yaw), IMU._timestamp)

    gyro  = Vector3(IMU.GyroX, IMU.GyroY, IMU.GyroZ)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    px4_state.update(gyro, accel, IMU._timestamp)
    return px4_state

_downsample_N = 0

def downsample(N):
    '''conditional that is true on every Nth sample'''
    global _downsample_N
    _downsample_N = (_downsample_N + 1) % N
    return _downsample_N == 0

def armed(HEARTBEAT):
    '''return 1 if armed, 0 if not'''
    from . import mavutil
    if HEARTBEAT.type == mavutil.mavlink.MAV_TYPE_GCS:
        self = mavutil.mavfile_global
        if self.motors_armed():
            return 1
        return 0
    if HEARTBEAT.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return 1
    return 0

def rotation_df(ATT):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(radians(ATT.Roll), radians(ATT.Pitch), radians(ATT.Yaw))
    return r

def rotation2(AHRS2):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(AHRS2.roll, AHRS2.pitch, AHRS2.yaw)
    return r

def earth_accel2(RAW_IMU,ATTITUDE):
    '''return earth frame acceleration vector from AHRS2'''
    r = rotation2(ATTITUDE)
    accel = Vector3(RAW_IMU.xacc, RAW_IMU.yacc, RAW_IMU.zacc) * 9.81 * 0.001
    return r * accel

def earth_accel_df(IMU,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    return r * accel

def earth_accel2_df(IMU,IMU2,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel1 = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    accel2 = Vector3(IMU2.AccX, IMU2.AccY, IMU2.AccZ)
    accel = 0.5 * (accel1 + accel2)
    return r * accel

def gps_velocity_df(GPS):
    '''return GPS velocity vector'''
    vx = GPS.Spd * cos(radians(GPS.GCrs))
    vy = GPS.Spd * sin(radians(GPS.GCrs))
    return Vector3(vx, vy, GPS.VZ)

def distance_gps2(GPS, GPS2):
    '''distance between two points'''
    if GPS.TimeMS != GPS2.TimeMS:
        # reject messages not time aligned
        return None
    return distance_two(GPS, GPS2)


radius_of_earth = 6378100.0 # in meters

def wrap_valid_longitude(lon):
  ''' wrap a longitude value around to always have a value in the range
      [-180, +180) i.e 0 => 0, 1 => 1, -1 => -1, 181 => -179, -181 => 179
  '''
  return (((lon + 180.0) % 360.0) - 180.0)

def gps_newpos(lat, lon, bearing, distance):
  '''extrapolate latitude/longitude given a heading and distance
  thanks to http://www.movable-type.co.uk/scripts/latlong.html
  '''
  import math
  lat1 = math.radians(lat)
  lon1 = math.radians(lon)
  brng = math.radians(bearing)
  dr = distance/radius_of_earth

  lat2 = math.asin(math.sin(lat1)*math.cos(dr) +
                   math.cos(lat1)*math.sin(dr)*math.cos(brng))
  lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(dr)*math.cos(lat1),
                           math.cos(dr)-math.sin(lat1)*math.sin(lat2))
  return (math.degrees(lat2), wrap_valid_longitude(math.degrees(lon2)))

def gps_offset(lat, lon, east, north):
  '''return new lat/lon after moving east/north
  by the given number of meters'''
  import math
  bearing = math.degrees(math.atan2(east, north))
  distance = math.sqrt(east**2 + north**2)
  return gps_newpos(lat, lon, bearing, distance)

ekf_home = None

def ekf1_pos(EKF1):
  '''calculate EKF position when EKF disabled'''
  global ekf_home
  from . import mavutil
  self = mavutil.mavfile_global
  if ekf_home is None:
      if not 'GPS' in self.messages or self.messages['GPS'].Status != 3:
          return None
      ekf_home = self.messages['GPS']
      (ekf_home.Lat, ekf_home.Lng) = gps_offset(ekf_home.Lat, ekf_home.Lng, -EKF1.PE, -EKF1.PN)
  (lat,lon) = gps_offset(ekf_home.Lat, ekf_home.Lng, EKF1.PE, EKF1.PN)
  return (lat, lon)

def quat_to_euler(q):
  '''
  Get Euler angles from a quaternion
  :param q: quaternion [w, x, y , z]
  :returns: euler angles [roll, pitch, yaw]
  '''
  quat = Quaternion(q)
  return quat.euler

def euler_to_quat(e):
  '''
  Get quaternion from euler angles
  :param e: euler angles [roll, pitch, yaw]
  :returns: quaternion [w, x, y , z]
  '''
  quat = Quaternion(e)
  return quat.q

def rotate_quat(attitude, roll, pitch, yaw):
  '''
  Returns rotated quaternion
  :param attitude: quaternion [w, x, y , z]
  :param roll: rotation in rad
  :param pitch: rotation in rad
  :param yaw: rotation in rad
  :returns: quaternion [w, x, y , z]
  '''
  quat = Quaternion(attitude)
  rotation = Quaternion([roll, pitch, yaw])
  res = rotation * quat

  return res.q

def qroll(MSG):
    '''return quaternion roll in degrees'''
    q = Quaternion([MSG.Q1,MSG.Q2,MSG.Q3,MSG.Q4])
    return degrees(q.euler[0])

    
def qpitch(MSG):
    '''return quaternion pitch in degrees'''
    q = Quaternion([MSG.Q1,MSG.Q2,MSG.Q3,MSG.Q4])
    return degrees(q.euler[1])

    
def qyaw(MSG):
    '''return quaternion yaw in degrees'''
    q = Quaternion([MSG.Q1,MSG.Q2,MSG.Q3,MSG.Q4])
    return degrees(q.euler[2])

def euler_rotated(MSG, roll, pitch, yaw):
    '''return eulers in radians from quaternion for view at given attitude in euler radians'''
    rot_view = Matrix3()
    rot_view.from_euler(roll, pitch, yaw)
    q = Quaternion([MSG.Q1,MSG.Q2,MSG.Q3,MSG.Q4])
    dcm = (rot_view * q.dcm.transposed()).transposed()
    return dcm.to_euler()

def euler_p90(MSG):
    '''return eulers in radians from quaternion for view at pitch 90'''
    return euler_rotated(MSG, 0, radians(90), 0);

def qroll_p90(MSG):
    '''return quaternion roll in degrees for view at pitch 90'''
    return degrees(euler_p90(MSG)[0])

def qpitch_p90(MSG):
    '''return quaternion roll in degrees for view at pitch 90'''
    return degrees(euler_p90(MSG)[1])

def qyaw_p90(MSG):
    '''return quaternion roll in degrees for view at pitch 90'''
    return degrees(euler_p90(MSG)[2])


def rotation_df(ATT):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(radians(ATT.Roll), radians(ATT.Pitch), radians(ATT.Yaw))
    return r

def rotation2(AHRS2):
    '''return the current DCM rotation matrix'''
    r = Matrix3()
    r.from_euler(AHRS2.roll, AHRS2.pitch, AHRS2.yaw)
    return r

def earth_accel2(RAW_IMU,ATTITUDE):
    '''return earth frame acceleration vector from AHRS2'''
    r = rotation2(ATTITUDE)
    accel = Vector3(RAW_IMU.xacc, RAW_IMU.yacc, RAW_IMU.zacc) * 9.81 * 0.001
    return r * accel

def earth_accel_df(IMU,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    return r * accel

def earth_accel2_df(IMU,IMU2,ATT):
    '''return earth frame acceleration vector from df log'''
    r = rotation_df(ATT)
    accel1 = Vector3(IMU.AccX, IMU.AccY, IMU.AccZ)
    accel2 = Vector3(IMU2.AccX, IMU2.AccY, IMU2.AccZ)
    accel = 0.5 * (accel1 + accel2)
    return r * accel

def gps_velocity_df(GPS):
    '''return GPS velocity vector'''
    vx = GPS.Spd * cos(radians(GPS.GCrs))
    vy = GPS.Spd * sin(radians(GPS.GCrs))
    return Vector3(vx, vy, GPS.VZ)

def armed(HEARTBEAT):
    '''return 1 if armed, 0 if not'''
    from pymavlink import mavutil
    if HEARTBEAT.type == mavutil.mavlink.MAV_TYPE_GCS:
        self = mavutil.mavfile_global
        if self.motors_armed():
            return 1
        return 0
    if HEARTBEAT.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return 1
    return 0

def mode(HEARTBEAT):
    '''return flight mode number'''
    from pymavlink import mavutil
    if HEARTBEAT.type == mavutil.mavlink.MAV_TYPE_GCS:
        return None
    return HEARTBEAT.custom_mode


'''
    magnetic field tables for estimating earths mag field
'''

# set to the sampling in degrees for the table below
SAMPLING_RES = 10.0
SAMPLING_MIN_LAT = -90.0
SAMPLING_MAX_LAT = 90.0
SAMPLING_MIN_LON = -180.0
SAMPLING_MAX_LON = 180.0

declination_table = [
    [149.42982,139.42982,129.42982,119.42982,109.42982,99.42982,89.42982,79.42982,69.42982,59.42982,49.42982,39.42982,29.42982,19.42982,9.42982,-0.57018,-10.57018,-20.57018,-30.57018,-40.57018,-50.57018,-60.57018,-70.57018,-80.57018,-90.57018,-100.57018,-110.57018,-120.57018,-130.57018,-140.57018,-150.57018,-160.57018,-170.57018,179.42982,169.42982,159.42982,149.42982],
    [129.72350,117.44264,106.27675,96.07611,86.65367,77.82604,69.43354,61.34790,53.47334,45.74345,38.11493,30.55915,23.05266,15.56845,8.07026,0.51157,-7.16035,-14.99671,-23.03882,-31.31253,-39.82756,-48.58229,-57.57257,-66.80224,-76.29309,-86.09269,-96.27941,-106.96368,-118.28316,-130.38657,-143.39712,-157.34798,-172.09985,172.71041,157.61759,143.16328,129.72350],
    [85.55217,77.63610,71.28198,65.85129,60.92943,56.19924,51.40394,46.35152,40.93254,35.13240,29.02800,22.76471,16.51219,10.40361,4.47591,-1.36061,-7.29972,-13.56717,-20.33231,-27.64625,-35.43223,-43.52666,-51.74560,-59.94560,-68.06018,-76.11386,-84.22919,-92.65008,-101.81721,-112.57190,-126.68545,-148.05164,177.80280,138.85306,112.24808,96.22652,85.55217],
    [47.46676,46.20752,44.77315,43.38132,42.07405,40.72123,39.02862,36.61723,33.16561,28.53811,22.85208,16.48376,9.99764,3.97129,-1.24934,-5.73392,-9.95537,-14.56642,-20.09169,-26.69103,-34.10667,-41.82261,-49.31316,-56.20179,-62.27798,-67.43190,-71.55604,-74.40916,-75.36987,-72.73151,-60.81180,-21.51651,25.67221,42.27293,47.13896,48.09081,47.46676],
    [30.85230,31.07383,30.82762,30.44253,30.14685,30.03972,29.94303,29.32537,27.46672,23.77665,18.08319,10.82311,3.04859,-3.93382,-9.21525,-12.73761,-15.21249,-17.71784,-21.33214,-26.67316,-33.41515,-40.52179,-46.98754,-52.17899,-55.71832,-57.28691,-56.42962,-52.33208,-43.85402,-30.45586,-13.95751,1.69556,13.74137,21.89083,26.92848,29.67895,30.85230],
    [22.27872,22.80208,22.89144,22.73115,22.48110,22.36292,22.48106,22.51808,21.62358,18.70580,13.04246,4.91899,-4.12159,-11.97999,-17.35339,-20.33078,-21.74028,-22.30429,-22.99797,-25.48758,-30.37748,-36.24796,-41.36613,-44.68747,-45.62236,-43.81812,-39.04370,-31.23361,-21.30619,-11.43220,-3.04586,3.96581,9.92051,14.80522,18.51928,20.97339,22.27872],
    [16.77535,17.26078,17.48701,17.50586,17.28046,16.91869,16.66947,16.54681,15.86669,13.28797,7.65685,-0.80738,-10.09007,-17.67695,-22.39570,-24.71604,-25.51571,-24.75384,-22.31254,-20.27272,-21.40935,-25.31718,-29.47979,-31.85235,-31.48877,-28.41892,-23.12231,-16.18014,-9.02321,-3.45191,0.46489,3.92873,7.47714,10.84926,13.70257,15.70340,16.77535],
    [13.11722,13.37999,13.53697,13.64213,13.51862,13.08139,12.58669,12.22195,11.40281,8.74064,3.03248,-5.27164,-13.84131,-20.33623,-23.89768,-24.94427,-24.16696,-21.44804,-16.65468,-11.57062,-9.17726,-10.67932,-14.35074,-17.28133,-17.77932,-15.94890,-12.44438,-7.73479,-3.08253,-0.10308,1.47778,3.23739,5.69958,8.33884,10.69634,12.34763,13.11722],
    [10.86027,10.84311,10.78802,10.86614,10.82660,10.44376,9.97329,9.56517,8.51030,5.51344,-0.27795,-8.04143,-15.48088,-20.66310,-22.77532,-21.94614,-18.98400,-14.68117,-9.75013,-5.14534,-2.07246,-1.81242,-4.23526,-7.17511,-8.61911,-8.27316,-6.53525,-3.65636,-0.68530,0.85477,1.25432,2.22687,4.25977,6.61864,8.78455,10.29691,10.86027],
    [9.65412,9.47055,9.21866,9.26610,9.31617,9.03106,8.63298,8.13549,6.68936,3.23076,-2.55638,-9.56906,-15.79936,-19.60429,-20.16306,-17.72398,-13.51036,-8.94319,-4.99959,-1.80533,0.70520,1.63736,0.32148,-2.02391,-3.63410,-4.02175,-3.40313,-1.86837,-0.15286,0.47666,0.26260,0.84732,2.70977,5.05917,7.34111,9.03687,9.65412],
    [8.97014,9.00488,8.80979,8.96359,9.19654,9.04844,8.56530,7.64578,5.51969,1.46185,-4.34083,-10.58923,-15.55806,-17.91473,-17.18297,-14.00887,-9.67704,-5.46497,-2.28363,-0.02111,1.89784,2.95296,2.25306,0.43899,-1.01576,-1.61934,-1.61055,-1.06870,-0.42999,-0.57588,-1.26445,-1.05555,0.57935,2.99639,5.63172,7.86731,8.97014],
    [8.05364,8.90038,9.27928,9.81741,10.36068,10.39013,9.68557,8.05395,4.96137,0.07375,-5.95698,-11.57288,-15.26102,-16.24039,-14.65177,-11.37263,-7.38273,-3.56765,-0.72868,1.13805,2.65859,3.63912,3.29650,1.92360,0.69270,0.06311,-0.26069,-0.47485,-0.85717,-1.88327,-3.18863,-3.51318,-2.26377,0.14436,3.15228,6.07110,8.05364],
    [6.50676,8.60131,10.09409,11.34973,12.28177,12.46194,11.52588,9.17003,5.01214,-0.92961,-7.49086,-12.75176,-15.40894,-15.33506,-13.21707,-9.96509,-6.25217,-2.66248,0.13977,1.99766,3.36674,4.31107,4.34970,3.54042,2.61979,1.95032,1.27943,0.31325,-1.17765,-3.29530,-5.41927,-6.36449,-5.53191,-3.19867,0.05852,3.54451,6.50676],
    [4.72297,8.05666,10.83494,13.02130,14.42516,14.72066,13.56011,10.55536,5.32509,-1.84890,-9.22855,-14.49666,-16.58473,-15.89094,-13.41064,-10.03642,-6.31692,-2.68976,0.34194,2.55093,4.17653,5.41501,6.09964,6.11745,5.65956,4.85558,3.55177,1.51138,-1.37406,-4.81860,-7.87000,-9.36323,-8.80100,-6.47499,-3.01085,0.91836,4.72297],
    [3.37834,7.62747,11.43831,14.53532,16.55914,17.10928,15.75231,11.96543,5.32788,-3.51576,-12.05022,-17.59555,-19.44526,-18.41349,-15.62077,-11.92636,-7.86545,-3.83213,-0.19787,2.82486,5.30947,7.42114,9.13275,10.21765,10.41233,9.48469,7.23089,3.55521,-1.29658,-6.44422,-10.49765,-12.34822,-11.76790,-9.22856,-5.44532,-1.08105,3.37834],
    [2.74348,7.62543,12.14084,15.97571,18.69139,19.70118,18.21773,13.23975,4.12042,-7.61873,-17.81916,-23.55512,-24.98204,-23.43924,-20.11619,-15.80905,-11.02265,-6.11388,-1.36465,3.04381,7.06375,10.70545,13.87807,16.28311,17.42655,16.70088,13.51831,7.64077,-0.16718,-7.83650,-13.15188,-15.19775,-14.31554,-11.33845,-7.10164,-2.26053,2.74348],
    [2.39274,7.82484,12.91652,17.34244,20.60856,21.88978,19.76035,12.03027,-2.57373,-19.28852,-30.38750,-34.61212,-34.22997,-31.16364,-26.55257,-21.04868,-15.04630,-8.80967,-2.53355,3.63374,9.57427,15.16558,20.22250,24.42837,27.25922,27.88349,25.07001,17.49446,5.52718,-6.65781,-14.50735,-17.27006,-16.30170,-13.00872,-8.39222,-3.11386,2.39274],
    [1.41048,6.79922,11.76290,15.71616,17.64723,15.50964,5.20845,-16.74575,-39.26184,-50.10475,-52.24001,-49.91343,-45.26415,-39.31139,-32.58025,-25.36820,-17.86089,-10.18671,-2.44480,5.27857,12.89790,20.31585,27.40587,33.98419,39.75848,44.22469,46.44725,44.61461,35.58987,17.59449,-1.23274,-11.49558,-14.19386,-12.63756,-8.86147,-3.96333,1.41048],
    [179.28905,-170.71095,-160.71095,-150.71095,-140.71095,-130.71095,-120.71095,-110.71095,-100.71095,-90.71095,-80.71095,-70.71095,-60.71095,-50.71095,-40.71095,-30.71095,-20.71095,-10.71095,-0.71095,9.28905,19.28905,29.28905,39.28905,49.28905,59.28905,69.28905,79.28905,89.28905,99.28905,109.28905,119.28905,129.28905,139.28905,149.28905,159.28905,169.28905,179.28905]
]

inclination_table = [
    [-72.15514,-72.15514,-72.15514,-72.15514,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15513,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514,-72.15514],
    [-78.41938,-77.65720,-76.73750,-75.70236,-74.58798,-73.42713,-72.25182,-71.09474,-69.98907,-68.96648,-68.05411,-67.27158,-66.62917,-66.12826,-65.76418,-65.53067,-65.42438,-65.44784,-65.60989,-65.92339,-66.40121,-67.05164,-67.87473,-68.86017,-69.98700,-71.22453,-72.53382,-73.86899,-75.17816,-76.40410,-77.48547,-78.36007,-78.97179,-79.28132,-79.27661,-78.97577,-78.41938],
    [-81.01394,-79.19354,-77.36330,-75.50423,-73.59085,-71.60725,-69.56243,-67.50284,-65.51562,-63.71750,-62.22814,-61.13417,-60.45695,-60.14119,-60.07436,-60.13141,-60.22358,-60.32959,-60.49886,-60.83015,-61.43611,-62.40625,-63.78154,-65.54891,-67.65384,-70.02042,-72.56767,-75.21637,-77.88752,-80.49381,-82.91772,-84.94371,-86.09650,-85.83265,-84.51758,-82.81598,-81.01394],
    [-77.57314,-75.57627,-73.65643,-71.75236,-69.77902,-67.63834,-65.25509,-62.63653,-59.92397,-57.39957,-55.42857,-54.33546,-54.24860,-55.00475,-56.20273,-57.38532,-58.22377,-58.60633,-58.63690,-58.59002,-58.82471,-59.65214,-61.21772,-63.47750,-66.26844,-69.40327,-72.72828,-76.12771,-79.49774,-82.71529,-85.55816,-87.23537,-86.33673,-84.19492,-81.90400,-79.68142,-77.57314],
    [-71.61543,-69.68071,-67.81295,-65.98732,-64.14688,-62.17325,-59.89199,-57.17213,-54.08269,-51.00980,-48.64526,-47.75928,-48.75785,-51.33161,-54.60713,-57.67200,-59.93603,-61.13032,-61.22418,-60.51276,-59.68523,-59.55885,-60.61820,-62.82724,-65.81275,-69.14491,-72.49609,-75.61467,-78.21163,-79.94564,-80.58474,-80.18788,-79.04223,-77.44162,-75.58435,-73.60578,-71.61543],
    [-64.37232,-62.41896,-60.47622,-58.52696,-56.58944,-54.65086,-52.56912,-50.08837,-47.05221,-43.75107,-41.15259,-40.61471,-42.92315,-47.48780,-52.84135,-57.78264,-61.78868,-64.59420,-65.79355,-65.17341,-63.32395,-61.60337,-61.26328,-62.62104,-65.13165,-67.99298,-70.59799,-72.54035,-73.51467,-73.54501,-73.01716,-72.21990,-71.17485,-69.83733,-68.20192,-66.33391,-64.37232],
    [-54.95221,-52.86419,-50.76591,-48.58110,-46.34373,-44.17713,-42.09147,-39.78009,-36.80878,-33.28666,-30.49763,-30.50812,-34.40344,-41.04847,-48.30470,-54.79768,-60.23560,-64.58921,-67.32150,-67.73041,-65.88657,-62.97458,-60.81524,-60.54948,-61.84643,-63.65901,-65.18925,-65.95250,-65.66438,-64.64265,-63.60342,-62.78558,-61.89618,-60.67851,-59.04993,-57.06784,-54.95221],
    [-42.11386,-39.71540,-37.42483,-35.05675,-32.54684,-30.08394,-27.80378,-25.32349,-22.02374,-18.05872,-15.18383,-16.05640,-21.78336,-30.70367,-40.14151,-48.30425,-54.72071,-59.52309,-62.50036,-63.16678,-61.44170,-58.08476,-54.78220,-53.13352,-53.22163,-54.08263,-54.91880,-55.11744,-54.20645,-52.62937,-51.43900,-50.82759,-50.12042,-48.88316,-47.03524,-44.65772,-42.11386],
    [-25.13413,-22.26490,-19.81259,-17.42498,-14.84507,-12.27503,-9.86112,-7.06257,-3.32091,0.81170,3.26607,1.47837,-5.45280,-16.03006,-27.48020,-37.20828,-44.04254,-48.15139,-50.04572,-50.01060,-48.00052,-44.32786,-40.52423,-38.33249,-37.93574,-38.43154,-39.10891,-39.27530,-38.24513,-36.52908,-35.53036,-35.38136,-34.94364,-33.61633,-31.38812,-28.37515,-25.13413],
    [-4.98724,-1.67113,0.80624,2.98017,5.33226,7.69596,9.97866,12.77254,16.36048,19.83531,21.34544,19.04912,12.24385,1.74844,-10.05341,-20.06978,-26.53723,-29.54293,-30.15639,-29.36839,-27.14298,-23.31481,-19.29463,-16.98941,-16.55873,-17.01490,-17.72494,-18.08276,-17.33194,-15.95689,-15.51123,-16.06549,-16.12696,-14.92427,-12.47974,-8.91893,-4.98724],
    [14.90282,18.28085,20.60765,22.44210,24.40105,26.45274,28.51977,30.96055,33.77883,36.09590,36.59075,34.17570,28.42190,19.80101,10.09244,1.82326,-3.38282,-5.35317,-5.09095,-3.87579,-1.76843,1.66543,5.30533,7.38022,7.73795,7.35305,6.77420,6.35804,6.66144,7.31673,7.03597,5.77710,4.97055,5.51060,7.51077,10.90541,14.90282],
    [31.18963,34.07820,36.15493,37.77321,39.49122,41.42181,43.45993,45.61640,47.69248,49.01037,48.72897,46.33216,41.81270,35.71328,29.23542,23.81452,20.40725,19.30431,19.92626,21.21332,22.94355,25.46085,28.10581,29.64612,29.92208,29.67696,29.36743,29.13603,29.17665,29.18105,28.33513,26.63499,25.13382,24.70085,25.65298,28.02186,31.18963],
    [43.43755,45.49190,47.26481,48.85551,50.58716,52.57872,54.71110,56.79027,58.49411,59.27330,58.58690,56.31729,52.84224,48.81000,44.95503,41.89331,40.02366,39.54270,40.19143,41.32256,42.64128,44.25362,45.87796,46.89272,47.16813,47.12227,47.08047,47.09378,47.09015,46.74024,45.58111,43.68407,41.77157,40.56696,40.45969,41.53519,43.43755],
    [53.14782,54.39692,55.85421,57.48060,59.33861,61.41997,63.58932,65.61909,67.16297,67.73632,66.96414,64.95612,62.26740,59.52539,57.18039,55.46617,54.49933,54.33769,54.84891,55.69458,56.62748,57.61136,58.56598,59.29096,59.71825,59.99071,60.24551,60.46371,60.46654,59.94194,58.64378,56.71646,54.68166,53.08630,52.27188,52.34019,53.14782],
    [61.94434,62.65988,63.82448,65.37386,67.23521,69.29012,71.36784,73.24154,74.58711,74.99756,74.23745,72.52856,70.40872,68.38080,66.74355,65.60955,64.99647,64.87946,65.16376,65.68049,66.27974,66.90598,67.55683,68.22109,68.88444,69.53923,70.14291,70.56735,70.59156,69.97960,68.65713,66.83530,64.91593,63.28781,62.20398,61.76373,61.94434],
    [70.63808,71.09315,71.98608,73.26619,74.84990,76.61252,78.38590,79.94294,80.96893,81.12757,80.33123,78.87452,77.18440,75.59163,74.28473,73.34040,72.76654,72.53101,72.57154,72.80778,73.17108,73.63386,74.21058,74.92988,75.79757,76.76288,77.69093,78.35319,78.47680,77.89101,76.66981,75.09384,73.49103,72.12315,71.15227,70.65240,70.63808],
    [78.93654,79.23546,79.83613,80.70281,81.78075,82.98979,84.21001,85.24697,85.79655,85.59093,84.72327,83.52137,82.24759,81.05849,80.04256,79.24469,78.67956,78.34092,78.20960,78.26251,78.48242,78.86522,79.41957,80.15662,81.07098,82.11600,83.17407,84.02661,84.37742,84.04550,83.16374,82.03536,80.91903,79.98035,79.31253,78.96013,78.93654],
    [86.10236,86.22193,86.48572,86.87541,87.36129,87.89241,88.36587,88.57214,88.32223,87.73845,87.01883,86.26928,85.54489,84.88026,84.29972,83.82070,83.45516,83.21059,83.09120,83.09908,83.23514,83.49941,83.89025,84.40254,85.02486,85.73566,86.49754,87.24606,87.86481,88.16518,88.02685,87.59994,87.10131,86.65607,86.32486,86.13654,86.10236],
    [88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643,88.07643]
]

intensity_table = [
    [0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779,0.54779],
    [0.60837,0.60220,0.59450,0.58547,0.57534,0.56430,0.55260,0.54051,0.52830,0.51629,0.50481,0.49417,0.48467,0.47656,0.47008,0.46544,0.46285,0.46247,0.46446,0.46890,0.47579,0.48502,0.49634,0.50939,0.52367,0.53860,0.55356,0.56792,0.58108,0.59253,0.60189,0.60890,0.61344,0.61552,0.61525,0.61280,0.60837],
    [0.63258,0.61970,0.60507,0.58890,0.57125,0.55217,0.53178,0.51039,0.48854,0.46699,0.44657,0.42804,0.41193,0.39851,0.38787,0.38006,0.37526,0.37389,0.37653,0.38383,0.39624,0.41387,0.43636,0.46284,0.49210,0.52268,0.55299,0.58146,0.60660,0.62725,0.64260,0.65237,0.65672,0.65618,0.65146,0.64336,0.63258],
    [0.62095,0.60243,0.58290,0.56243,0.54073,0.51730,0.49170,0.46391,0.43471,0.40564,0.37864,0.35546,0.33702,0.32321,0.31311,0.30575,0.30078,0.29875,0.30106,0.30949,0.32559,0.34998,0.38207,0.42017,0.46201,0.50518,0.54726,0.58585,0.61871,0.64402,0.66081,0.66905,0.66955,0.66358,0.65259,0.63798,0.62095],
    [0.58620,0.56373,0.54114,0.51857,0.49563,0.47134,0.44448,0.41429,0.38135,0.34789,0.31726,0.29272,0.27609,0.26667,0.26176,0.25848,0.25542,0.25307,0.25351,0.26004,0.27617,0.30384,0.34230,0.38856,0.43871,0.48915,0.53690,0.57926,0.61368,0.63832,0.65262,0.65718,0.65336,0.64282,0.62713,0.60777,0.58620],
    [0.54054,0.51627,0.49228,0.46880,0.44572,0.42234,0.39721,0.36894,0.33742,0.30457,0.27434,0.25147,0.23907,0.23614,0.23823,0.24087,0.24203,0.24166,0.24097,0.24339,0.25474,0.27997,0.31964,0.36973,0.42394,0.47661,0.52408,0.56374,0.59341,0.61236,0.62157,0.62234,0.61581,0.60319,0.58561,0.56419,0.54054],
    [0.48867,0.46503,0.44165,0.41863,0.39623,0.37454,0.35283,0.32976,0.30421,0.27678,0.25090,0.23196,0.22388,0.22576,0.23267,0.24037,0.24755,0.25361,0.25697,0.25833,0.26348,0.28050,0.31390,0.36106,0.41386,0.46412,0.50708,0.53992,0.56069,0.57069,0.57365,0.57144,0.56381,0.55092,0.53338,0.51203,0.48867],
    [0.43253,0.41175,0.39135,0.37126,0.35187,0.33373,0.31703,0.30085,0.28340,0.26400,0.24494,0.23085,0.22546,0.22878,0.23747,0.24859,0.26158,0.27535,0.28592,0.29056,0.29243,0.29975,0.32084,0.35718,0.40141,0.44430,0.48014,0.50509,0.51651,0.51743,0.51457,0.51016,0.50215,0.48970,0.47325,0.45356,0.43253],
    [0.37920,0.36357,0.34861,0.33428,0.32094,0.30906,0.29901,0.29027,0.28111,0.27011,0.25800,0.24747,0.24170,0.24271,0.25038,0.26289,0.27848,0.29521,0.30916,0.31675,0.31844,0.31999,0.33017,0.35323,0.38456,0.41649,0.44359,0.46117,0.46580,0.46094,0.45426,0.44791,0.43910,0.42690,0.41213,0.39570,0.37920],
    [0.34153,0.33271,0.32464,0.31758,0.31215,0.30839,0.30613,0.30492,0.30318,0.29885,0.29113,0.28131,0.27228,0.26771,0.27053,0.28020,0.29350,0.30751,0.31975,0.32797,0.33146,0.33325,0.33952,0.35374,0.37352,0.39442,0.41263,0.42404,0.42554,0.41934,0.41078,0.40157,0.39035,0.37734,0.36408,0.35186,0.34153],
    [0.32870,0.32607,0.32446,0.32437,0.32687,0.33172,0.33779,0.34389,0.34801,0.34747,0.34080,0.32911,0.31570,0.30498,0.30097,0.30427,0.31213,0.32189,0.33188,0.34049,0.34689,0.35278,0.36094,0.37200,0.38471,0.39787,0.40970,0.41726,0.41829,0.41286,0.40244,0.38841,0.37237,0.35653,0.34312,0.33381,0.32870],
    [0.34033,0.34108,0.34427,0.35011,0.35952,0.37200,0.38564,0.39812,0.40667,0.40818,0.40093,0.38646,0.36911,0.35402,0.34504,0.34287,0.34584,0.35257,0.36183,0.37142,0.38023,0.38954,0.40019,0.41097,0.42118,0.43143,0.44120,0.44828,0.45031,0.44540,0.43237,0.41263,0.39030,0.36959,0.35349,0.34378,0.34033],
    [0.37286,0.37425,0.38045,0.39099,0.40563,0.42319,0.44141,0.45760,0.46878,0.47164,0.46425,0.44822,0.42852,0.41089,0.39901,0.39341,0.39308,0.39740,0.40545,0.41491,0.42425,0.43421,0.44546,0.45716,0.46876,0.48057,0.49201,0.50099,0.50465,0.49993,0.48499,0.46152,0.43456,0.40947,0.38986,0.37762,0.37286],
    [0.42301,0.42396,0.43139,0.44439,0.46152,0.48059,0.49920,0.51504,0.52566,0.52833,0.52129,0.50580,0.48620,0.46770,0.45379,0.44539,0.44205,0.44336,0.44854,0.45590,0.46409,0.47337,0.48463,0.49801,0.51297,0.52857,0.54321,0.55444,0.55930,0.55497,0.54021,0.51688,0.48967,0.46376,0.44295,0.42921,0.42301],
    [0.48379,0.48443,0.49110,0.50289,0.51807,0.53427,0.54920,0.56099,0.56797,0.56854,0.56177,0.54851,0.53155,0.51445,0.50002,0.48959,0.48345,0.48151,0.48332,0.48795,0.49463,0.50341,0.51494,0.52962,0.54684,0.56497,0.58158,0.59393,0.59937,0.59603,0.58366,0.56421,0.54142,0.51947,0.50159,0.48950,0.48379],
    [0.53971,0.53998,0.54408,0.55129,0.56040,0.56987,0.57817,0.58402,0.58645,0.58476,0.57870,0.56880,0.55636,0.54317,0.53091,0.52082,0.51365,0.50978,0.50925,0.51182,0.51725,0.52552,0.53678,0.55096,0.56731,0.58420,0.59944,0.61067,0.61601,0.61448,0.60643,0.59353,0.57838,0.56374,0.55175,0.54361,0.53971],
    [0.57270,0.57191,0.57270,0.57465,0.57722,0.57979,0.58173,0.58248,0.58164,0.57897,0.57445,0.56828,0.56094,0.55308,0.54545,0.53875,0.53362,0.53052,0.52975,0.53146,0.53567,0.54232,0.55121,0.56189,0.57362,0.58531,0.59567,0.60348,0.60783,0.60838,0.60542,0.59985,0.59290,0.58587,0.57979,0.57531,0.57270],
    [0.57801,0.57670,0.57563,0.57472,0.57387,0.57296,0.57187,0.57049,0.56875,0.56662,0.56412,0.56133,0.55839,0.55547,0.55278,0.55055,0.54900,0.54830,0.54859,0.54996,0.55239,0.55581,0.56003,0.56482,0.56984,0.57472,0.57910,0.58266,0.58517,0.58654,0.58683,0.58619,0.58487,0.58317,0.58133,0.57957,0.57801],
    [0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621,0.56621]
]


'''
    calculate magnetic field intensity and orientation

    returns array [declination_deg, inclination_deg, intensity] or None
'''
def get_mag_field_ef(latitude_deg, longitude_deg):
    # round down to nearest sampling resolution
    min_lat = int(floor(latitude_deg / SAMPLING_RES) * SAMPLING_RES)
    min_lon = int(floor(longitude_deg / SAMPLING_RES) * SAMPLING_RES)

    # limit to table bounds
    if latitude_deg <= SAMPLING_MIN_LAT:
        return None
    if latitude_deg >= SAMPLING_MAX_LAT:
        return None
    if longitude_deg <= SAMPLING_MIN_LON:
        return None
    if longitude_deg >= SAMPLING_MAX_LON:
        return None

    # find index of nearest low sampling point
    min_lat_index = int(floor(-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES)
    min_lon_index = int(floor(-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES)

    # calculate intensity
    data_sw = intensity_table[min_lat_index][min_lon_index]
    data_se = intensity_table[min_lat_index][min_lon_index + 1]
    data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1]
    data_nw = intensity_table[min_lat_index + 1][min_lon_index]

    # perform bilinear interpolation on the four grid corners
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw

    intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min

    # calculate declination
    data_sw = declination_table[min_lat_index][min_lon_index]
    data_se = declination_table[min_lat_index][min_lon_index + 1]
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1]
    data_nw = declination_table[min_lat_index + 1][min_lon_index]

    # perform bilinear interpolation on the four grid corners
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw

    declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min

    # calculate inclination
    data_sw = inclination_table[min_lat_index][min_lon_index]
    data_se = inclination_table[min_lat_index][min_lon_index + 1]
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1]
    data_nw = inclination_table[min_lat_index + 1][min_lon_index]

    # perform bilinear interpolation on the four grid corners
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw

    inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min

    return [declination_deg, inclination_deg, intensity_gauss]


earth_field = None

def expected_earth_field(GPS):
    '''return expected magnetic field for a location'''
    global earth_field
    if earth_field is not None:
        return earth_field

    if hasattr(GPS,'fix_type'):
        gps_status = GPS.fix_type
        lat = GPS.lat*1.0e-7
        lon = GPS.lon*1.0e-7
    else:
        gps_status = GPS.Status
        lat = GPS.Lat
        lon = GPS.Lng

    if gps_status < 3:
        return Vector3(0,0,0)
    field_var = get_mag_field_ef(lat, lon)
    mag_ef = Vector3(field_var[2]*1000.0, 0.0, 0.0)
    R = Matrix3()
    R.from_euler(0.0, -radians(field_var[1]), radians(field_var[0]))
    mag_ef = R * mag_ef
    earth_field = mag_ef
    return earth_field

def mag_yaw(GPS,ATT,MAG):
    '''calculate heading from raw magnetometer'''
    expected_earth_field(GPS)
    mag_x = MAG.MagX
    mag_y = MAG.MagY
    mag_z = MAG.MagZ

    # go via a DCM matrix to match the APM calculation
    dcm_matrix = rotation_df(ATT)
    cos_pitch_sq = 1.0-(dcm_matrix.c.x*dcm_matrix.c.x)
    headY = mag_y * dcm_matrix.c.z - mag_z * dcm_matrix.c.y
    headX = mag_x * cos_pitch_sq - dcm_matrix.c.x * (mag_y * dcm_matrix.c.y + mag_z * dcm_matrix.c.z)

    heading = degrees(atan2(-headY,headX)) + earth_declination
    if heading < 0:
        heading += 360
    return heading

def expected_mag(GPS,ATT,roll_adjust=0,pitch_adjust=0,yaw_adjust=0):
    '''return expected magnetic field for a location and attitude'''
    global earth_field

    expected_earth_field(GPS)
    if earth_field is None:
        return Vector3(0,0,0)

    if hasattr(ATT,'roll'):
        roll = degrees(ATT.roll)+roll_adjust
        pitch = degrees(ATT.pitch)+pitch_adjust
        yaw = degrees(ATT.yaw)+yaw_adjust
    else:
        roll = ATT.Roll+roll_adjust
        pitch = ATT.Pitch+pitch_adjust
        yaw = ATT.Yaw+yaw_adjust

    rot = Matrix3()
    rot.from_euler(radians(roll), radians(pitch), radians(yaw))

    field = rot.transposed() * earth_field

    return field

def expected_mag_yaw(GPS,ATT,MAG,roll_adjust=0,pitch_adjust=0,yaw_adjust=0):
    '''return expected magnetic field for a location and attitude'''
    global earth_field

    expected_earth_field(GPS)
    if earth_field is None:
        return Vector3(0,0,0)

    if hasattr(ATT,'roll'):
        roll = degrees(ATT.roll)+roll_adjust
        pitch = degrees(ATT.pitch)+pitch_adjust
    else:
        roll = ATT.Roll+roll_adjust
        pitch = ATT.Pitch+pitch_adjust
    yaw = mag_yaw(GPS,ATT,MAG)

    rot = Matrix3()
    rot.from_euler(radians(roll), radians(pitch), radians(yaw))

    field = rot.transposed() * earth_field

    return field

def earth_field_error(GPS,NKF2):
    '''return vector error in earth field estimate'''
    global earth_field
    expected_earth_field(GPS)
    if earth_field is None:
        return Vector3(0,0,0)
    ef = Vector3(NKF2.MN,NKF2.ME,NKF2.MD)
    ret = ef - earth_field
    return ret


def distance_home_df(GPS,ORGN):
    '''distance from home origin'''
    return distance_two(GPS_RAW, first_fix)

def airspeed_estimate(GLOBAL_POSITION_INT,WIND):
    '''estimate airspeed'''
    wind = WIND
    gpi = GLOBAL_POSITION_INT
    from pymavlink.rotmat import Vector3
    import math
    wind3d = Vector3(wind.speed*math.cos(math.radians(wind.direction)),
                     wind.speed*math.sin(math.radians(wind.direction)), 0)
    ground = Vector3(gpi.vx*0.01, gpi.vy*0.01, 0)
    airspeed = (ground + wind3d).length()
    return airspeed


def distance_from(GPS_RAW1, lat, lon):
    '''distance from a given location'''
    if hasattr(GPS_RAW1, 'Lat'):
        lat1 = radians(GPS_RAW1.Lat)
        lon1 = radians(GPS_RAW1.Lng)
    elif hasattr(GPS_RAW1, 'cog'):
        lat1 = radians(GPS_RAW1.lat)*1.0e-7
        lon1 = radians(GPS_RAW1.lon)*1.0e-7
    else:
        lat1 = radians(GPS_RAW1.lat)
        lon1 = radians(GPS_RAW1.lon)

    lat2 = radians(lat)
    lon2 = radians(lon)

    dLat = lat2 - lat1
    dLon = lon2 - lon1

    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def distance_lat_lon(lat1, lon1, lat2, lon2):
    '''distance between two points'''
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)

    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(a), sqrt(1.0-a))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def constrain(v, minv, maxv):
    if v < minv:
        v = minv
    if v > maxv:
        v = maxv
    return v

def sim_body_rates(SIM):
    '''return body frame rates from simulator attitudes'''
    rollRate = delta(SIM.Roll,'sbr',SIM.TimeUS)
    pitchRate = delta(SIM.Pitch,'sbp',SIM.TimeUS)
    yawRate = delta(SIM.Yaw,'sby',SIM.TimeUS)
    phi = radians(SIM.Roll)
    theta = radians(SIM.Pitch)
    phiDot = radians(rollRate)
    thetaDot = radians(pitchRate)
    psiDot = radians(yawRate)

    p = phiDot - psiDot*sin(theta)
    q = cos(phi)*thetaDot + sin(phi)*psiDot*cos(theta)
    r = cos(phi)*psiDot*cos(theta) - sin(phi)*thetaDot
    return Vector3(p, q, r)
