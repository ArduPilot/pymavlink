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
SAMPLING_MIN_LAT = -60.0
SAMPLING_MAX_LAT = 60.0
SAMPLING_MIN_LON = -180.0
SAMPLING_MAX_LON = 180.0

# table data containing magnetic declination angle in degrees
declination_table = [
    [47.225,46.016,44.6,43.214,41.929,40.625,38.996,36.648,33.246,28.649,22.973,16.599,10.096,4.0412,-1.2113,-5.7129,-9.9239,-14.505,-19.992,-26.538,-33.881,-41.531,-48.995,-55.908,-62.043,-67.26,-71.433,-74.32,-75.32,-72.759,-61.089,-22.66,24.673,41.679,46.715,47.77,47.225],
    [30.663,30.935,30.729,30.365,30.083,29.989,29.903,29.306,27.493,23.864,18.229,11.012,3.2503,-3.7771,-9.1638,-12.793,-15.299,-17.753,-21.275,-26.491,-33.098,-40.129,-46.619,-51.911,-55.576,-57.256,-56.482,-52.464,-44.092,-30.765,-14.236,1.4682,13.543,21.705,26.727,29.465,30.663],
    [22.141,22.67,22.786,22.665,22.457,22.365,22.478,22.512,21.659,18.814,13.22,5.1441,-3.8724,-11.749,-17.208,-20.303,-21.777,-22.354,-23.04,-25.422,-30.143,-35.947,-41.114,-44.536,-45.603,-43.943,-39.28,-31.541,-21.622,-11.631,-3.1023,3.9413,9.8774,14.757,18.445,20.858,22.141],
    [16.701,17.175,17.415,17.473,17.283,16.943,16.689,16.566,15.935,13.436,7.8847,-0.52637,-9.7982,-17.399,-22.182,-24.63,-25.544,-24.89,-22.583,-20.489,-21.371,-25.128,-29.294,-31.76,-31.569,-28.686,-23.466,-16.487,-9.2421,-3.5196,0.52944,4.0043,7.5112,10.847,13.654,15.633,16.701],
    [13.062,13.307,13.488,13.647,13.552,13.138,12.644,12.281,11.511,8.9083,3.288,-4.9363,-13.534,-20.108,-23.761,-24.943,-24.267,-21.633,-16.975,-11.895,-9.3052,-10.608,-14.22,-17.235,-17.896,-16.21,-12.734,-7.944,-3.1917,-0.09448,1.5925,3.3685,5.7977,8.374,10.643,12.277,13.062],
    [10.819,10.774,10.744,10.873,10.857,10.513,10.051,9.6504,8.6623,5.7203,-0.00103,-7.7113,-15.239,-20.543,-22.747,-22.021,-19.117,-14.884,-10.06,-5.4668,-2.2918,-1.8711,-4.2049,-7.1881,-8.7089,-8.4387,-6.7419,-3.8048,-0.7327,0.90936,1.399,2.3853,4.3796,6.6682,8.7425,10.241,10.819],
        [9.6261,9.4267,9.2035,9.2862,9.3524,9.1115,8.7204,8.235,6.8708,3.4733,-2.2704,-9.2885,-15.646,-19.575,-20.227,-17.864,-13.663,-9.1469,-5.2757,-2.071,0.49345,1.5251,0.29098,-2.0644,-3.6913,-4.1179,-3.5596,-1.9862,-0.1684,0.55548,0.42539,1.019,2.8249,5.1073,7.319,8.9998,9.6261],
        [8.9369,8.9745,8.8157,9.0087,9.2615,9.1408,8.6708,7.7842,5.7199,1.6936,-4.0873,-10.373,-15.479,-17.961,-17.333,-14.207,-9.8358,-5.6345,-2.4989,-0.22009,1.7216,2.8156,2.1698,0.36849,-1.0742,-1.6949,-1.7318,-1.1504,-0.42435,-0.50061,-1.1161,-0.88856,0.69851,3.0638,5.6417,7.8392,8.9369],
        [8.0661,8.9128,9.3097,9.8875,10.445,10.466,9.7864,8.2288,5.1818,0.28672,-5.7524,-11.438,-15.264,-16.348,-14.841,-11.596,-7.5597,-3.7288,-0.91243,0.96946,2.51,3.5079,3.1926,1.8339,0.62026,-0.01417,-0.36046,-0.53631,-0.84781,-1.8165,-3.0616,-3.3677,-2.1514,0.22612,3.1996,6.0876,8.0661],
        [6.6045,8.7101,10.209,11.491,12.418,12.553,11.622,9.3409,5.2254,-0.73322,-7.3408,-12.717,-15.505,-15.494,-13.422,-10.206,-6.4719,-2.8539,-0.05159,1.8185,3.2182,4.1923,4.2452,3.4363,2.5284,1.8671,1.1902,0.25353,-1.1684,-3.2141,-5.2908,-6.2436,-5.4483,-3.1334,0.11904,3.6145,6.6045],
        [4.8686,8.2538,11.062,13.264,14.651,14.898,13.714,10.725,5.4958,-1.7139,-9.1709,-14.568,-16.765,-16.116,-13.666,-10.317,-6.5773,-2.906,0.14042,2.3482,3.9956,5.2703,5.9678,5.9805,5.5419,4.7751,3.4951,1.4836,-1.3442,-4.7282,-7.762,-9.283,-8.7476,-6.4234,-2.9491,1.0086,4.8686],
        [3.5965,7.9367,11.797,14.893,16.888,17.409,16.035,12.22,5.5033,-3.468,-12.144,-17.817,-19.749,-18.751,-15.97,-12.268,-8.1724,-4.1017,-0.46224,2.551,5.0589,7.2268,8.9817,10.082,10.298,9.416,7.21,3.5705,-1.2522,-6.3856,-10.448,-12.313,-11.73,-9.1743,-5.367,-0.9513,3.5965],
        [3.0633,8.0696,12.667,16.527,19.239,20.252,18.783,13.765,4.4393,-7.6723,-18.2,-24.093,-25.554,-23.984,-20.61,-16.25,-11.425,-6.5023,-1.759,2.6571,6.7294,10.458,13.708,16.148,17.3,16.587,13.441,7.6143,-0.14961,-7.7957,-13.103,-15.143,-14.253,-11.262,-6.9868,-2.0631,3.0633]
    ]

# table data containing magnetic inclination angle in degrees
inclination_table = [
        [-77.62,-75.621,-73.703,-71.805,-69.84,-67.703,-65.317,-62.69,-59.968,-57.429,-55.437,-54.313,-54.191,-54.921,-56.118,-57.329,-58.214,-58.643,-58.712,-58.692,-58.934,-59.74,-61.257,-63.461,-66.211,-69.329,-72.657,-76.065,-79.441,-82.661,-85.506,-87.208,-86.36,-84.237,-81.951,-79.73,-77.62],
        [-71.644,-69.722,-67.862,-66.038,-64.191,-62.202,-59.91,-57.194,-54.113,-51.035,-48.639,-47.697,-48.625,-51.137,-54.39,-57.488,-59.821,-61.085,-61.237,-60.574,-59.768,-59.618,-60.616,-62.761,-65.711,-69.044,-72.416,-75.556,-78.175,-79.936,-80.593,-80.201,-79.054,-77.45,-75.591,-73.62,-71.644],
        [-64.379,-62.435,-60.505,-58.564,-56.621,-54.665,-52.574,-50.107,-47.083,-43.756,-41.093,-40.479,-42.721,-47.229,-52.537,-57.471,-61.513,-64.379,-65.669,-65.164,-63.395,-61.672,-61.269,-62.55,-65.012,-67.881,-70.533,-72.526,-73.547,-73.61,-73.075,-72.251,-71.191,-69.844,-68.201,-66.334,-64.379],
        [-54.946,-52.868,-50.792,-48.616,-46.372,-44.193,-42.1,-39.8,-36.829,-33.264,-30.385,-30.266,-34.046,-40.628,-47.854,-54.358,-59.856,-64.283,-67.119,-67.68,-65.968,-63.1,-60.894,-60.535,-61.761,-63.584,-65.189,-66.021,-65.775,-64.769,-63.698,-62.826,-61.898,-60.654,-59.023,-57.056,-54.946],
        [-42.121,-39.754,-37.516,-35.153,-32.626,-30.14,-27.828,-25.343,-22.028,-18.017,-15.062,-15.744,-21.265,-30.112,-39.56,-47.788,-54.306,-59.187,-62.247,-63.04,-61.444,-58.163,-54.854,-53.154,-53.207,-54.092,-55.004,-55.267,-54.381,-52.801,-51.562,-50.874,-50.1,-48.81,-46.97,-44.643,-42.121],
        [-25.138,-22.301,-19.912,-17.531,-14.951,-12.37,-9.9146,-7.1192,-3.3525,0.86465,3.4285,1.8632,-4.8485,-15.374,-26.858,-36.666,-43.599,-47.78,-49.783,-49.893,-48.024,-44.445,-40.638,-38.414,-38.009,-38.508,-39.247,-39.498,-38.5,-36.768,-35.692,-35.432,-34.911,-33.532,-31.323,-28.369,-25.138],
        [-4.9962,-1.7126,0.71401,2.8847,5.219,7.5873,9.91,12.689,16.306,19.897,21.553,19.47,12.842,2.3904,-9.4406,-19.56,-26.119,-29.178,-29.919,-29.286,-27.194,-23.471,-19.459,-17.133,-16.698,-17.132,-17.899,-18.371,-17.66,-16.255,-15.708,-16.114,-16.082,-14.846,-12.422,-8.9145,-4.9962],
        [14.869,18.185,20.462,22.293,24.256,26.334,28.442,30.896,33.762,36.172,36.784,34.538,28.924,20.356,10.616,2.2154,-3.0793,-5.0649,-4.8917,-3.7875,-1.7639,1.5757,5.1722,7.2324,7.5774,7.2059,6.5718,6.0595,6.3467,7.0334,6.8289,5.7043,4.9959,5.5796,7.5845,10.93,14.869],
        [31.164,34.006,36.041,37.647,39.383,41.332,43.382,45.569,47.716,49.104,48.9,46.613,42.181,36.116,29.611,24.084,20.599,19.485,20.056,21.274,22.956,25.416,28.017,29.529,29.786,29.536,29.184,28.895,28.929,28.95,28.145,26.531,25.104,24.717,25.695,28.042,31.164],
        [43.422,45.479,47.235,48.815,50.558,52.55,54.665,56.761,58.528,59.374,58.753,56.548,53.102,49.074,45.208,42.092,40.16,39.643,40.248,41.324,42.606,44.197,45.811,46.815,47.078,47.021,46.948,46.916,46.893,46.555,45.437,43.596,41.72,40.533,40.438,41.516,43.422],
        [53.102,54.356,55.814,57.449,59.331,61.427,63.597,65.639,67.216,67.832,67.107,65.133,62.449,59.699,57.346,55.602,54.591,54.397,54.884,55.703,56.613,57.588,58.539,59.25,59.652,59.904,60.137,60.33,60.32,59.812,58.553,56.66,54.639,53.044,52.229,52.294,53.102],
        [61.877,62.601,63.785,65.363,67.255,69.332,71.425,73.313,74.675,75.099,74.352,72.651,70.529,68.494,66.843,65.687,65.049,64.919,65.198,65.705,66.286,66.893,67.525,68.165,68.802,69.44,70.041,70.469,70.5,69.901,68.591,66.775,64.856,63.228,62.144,61.699,61.877],
        [70.565,71.043,71.969,73.282,74.89,76.669,78.458,80.035,81.082,81.248,80.439,78.963,77.256,75.651,74.334,73.378,72.792,72.544,72.57,72.785,73.123,73.565,74.13,74.845,75.713,76.683,77.618,78.289,78.419,77.836,76.61,75.025,73.414,72.04,71.067,70.568,70.565]
    ]

# table data containing magnetic intensity in Gauss
intensity_table = [
        [0.62195,0.60352,0.58407,0.56378,0.54235,0.5192,0.49375,0.46596,0.43661,0.40734,0.38017,0.35689,0.33841,0.32458,0.31447,0.30707,0.302,0.29983,0.30197,0.31023,0.32606,0.35005,0.38167,0.41941,0.4611,0.50431,0.54648,0.58515,0.61805,0.64341,0.66031,0.66875,0.6695,0.66384,0.65315,0.6388,0.62195],
        [0.58699,0.56471,0.54222,0.51977,0.49699,0.47285,0.44606,0.4159,0.38299,0.3495,0.31875,0.29401,0.2771,0.26745,0.26251,0.25941,0.25657,0.25431,0.25469,0.26104,0.27676,0.30379,0.34157,0.38741,0.43751,0.48812,0.53602,0.57843,0.61292,0.63771,0.65218,0.65693,0.65331,0.64296,0.62744,0.6083,0.58699],
        [0.54115,0.51716,0.49337,0.47003,0.44701,0.42359,0.39839,0.37018,0.33882,0.30605,0.27573,0.25265,0.23995,0.23673,0.23869,0.24146,0.24284,0.24258,0.24188,0.24416,0.25504,0.27952,0.31847,0.36815,0.42238,0.47534,0.52306,0.56286,0.59276,0.61199,0.62133,0.62218,0.61584,0.60338,0.58585,0.56455,0.54115],
        [0.48901,0.46546,0.44226,0.41941,0.39712,0.37544,0.35375,0.33083,0.30548,0.27808,0.25206,0.23294,0.22466,0.22623,0.23284,0.24043,0.24771,0.25392,0.25756,0.2592,0.26416,0.28046,0.31301,0.35947,0.41201,0.4626,0.5061,0.53933,0.56053,0.57088,0.5738,0.57145,0.56393,0.55115,0.53361,0.51231,0.48901],
        [0.43275,0.41204,0.39186,0.37191,0.35253,0.33441,0.31775,0.3017,0.28442,0.26511,0.24603,0.23176,0.22605,0.22892,0.2372,0.24818,0.2613,0.27524,0.28613,0.2912,0.29315,0.29999,0.32028,0.35581,0.39969,0.44294,0.47952,0.50496,0.51672,0.51792,0.51496,0.51028,0.50215,0.48957,0.47314,0.45367,0.43275],
        [0.37936,0.36387,0.34916,0.33493,0.32156,0.30968,0.29963,0.29094,0.28194,0.27119,0.25923,0.24862,0.24246,0.2429,0.25003,0.26227,0.27786,0.29465,0.30882,0.3168,0.31873,0.3201,0.32969,0.35219,0.38333,0.41554,0.44325,0.46129,0.46614,0.4614,0.45461,0.44801,0.43895,0.4265,0.41178,0.39569,0.37936],
        [0.34151,0.33271,0.3248,0.31786,0.31251,0.30889,0.30674,0.30563,0.30411,0.30005,0.29252,0.28275,0.27347,0.26836,0.27051,0.27963,0.29265,0.30666,0.31919,0.32782,0.33164,0.33341,0.33926,0.35308,0.37271,0.39367,0.41222,0.42402,0.42574,0.41956,0.41085,0.40143,0.39009,0.37701,0.36387,0.35184,0.34151],
        [0.32861,0.32598,0.32444,0.32451,0.32723,0.33225,0.33844,0.34476,0.34925,0.34892,0.34232,0.33071,0.31718,0.30603,0.30142,0.30408,0.3115,0.32118,0.33137,0.34024,0.34686,0.35275,0.36068,0.37146,0.38401,0.39715,0.40907,0.41682,0.418,0.41256,0.40201,0.38791,0.37189,0.35616,0.34293,0.33373,0.32861],
        [0.34027,0.34127,0.34464,0.3507,0.36038,0.37294,0.3865,0.39917,0.40815,0.4099,0.40274,0.38831,0.37072,0.3552,0.34582,0.34316,0.34566,0.35218,0.36135,0.37089,0.37971,0.38903,0.39965,0.41036,0.42049,0.43072,0.44044,0.44743,0.44945,0.44455,0.43158,0.41195,0.38966,0.369,0.35302,0.34348,0.34027],
        [0.37253,0.3743,0.38082,0.39172,0.4068,0.42456,0.44274,0.45904,0.47046,0.47349,0.4662,0.45016,0.43012,0.41204,0.39986,0.39392,0.39314,0.39717,0.40505,0.41434,0.42357,0.43356,0.44488,0.45654,0.46802,0.47977,0.49115,0.5,0.50357,0.49896,0.48426,0.46097,0.43398,0.40879,0.38917,0.37703,0.37253],
        [0.42232,0.42361,0.43153,0.44511,0.46277,0.48222,0.50102,0.51697,0.52763,0.53021,0.5231,0.50749,0.48759,0.46872,0.4545,0.44574,0.44202,0.44314,0.44829,0.45556,0.46359,0.47278,0.48398,0.49721,0.51197,0.52753,0.54226,0.5536,0.55855,0.55434,0.53968,0.51634,0.489,0.46299,0.44215,0.4284,0.42232],
        [0.48297,0.48409,0.49144,0.50386,0.51947,0.53592,0.55104,0.56298,0.57002,0.57048,0.5635,0.54997,0.53268,0.51524,0.50048,0.48973,0.48333,0.48125,0.48294,0.48735,0.49374,0.50234,0.51385,0.52856,0.54583,0.56407,0.58089,0.59345,0.59903,0.5957,0.58321,0.56356,0.54058,0.51853,0.50058,0.4885,0.48297],
        [0.53888,0.53954,0.54416,0.55185,0.5613,0.57099,0.5795,0.58558,0.58818,0.58651,0.58029,0.57006,0.55723,0.54365,0.53109,0.52078,0.51346,0.50946,0.50872,0.51103,0.51624,0.52445,0.53585,0.55027,0.56682,0.58384,0.59914,0.61041,0.61575,0.61419,0.60607,0.59307,0.5778,0.563,0.55086,0.54264,0.53888]
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
