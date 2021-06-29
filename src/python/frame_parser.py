'''
frame_parser.py

A not so real implementation of frame_parser.h which is located
in the src/uccm-sensors folder. This version of frame parser takes
the sent data to the CAN bus (which is sent through the
CAN_BBB_NUC_sp_test.py file) and pre-parses those values so that
they can compared with the frame values that are received on the CAN
Bus Listener code and at the NUC Eth Listener code. The actual
pre-parsing calls to the functions in here are done in dummy_data.py.
The functions are implemented as callbacks in the CAN_to_URI_to_ROS.py
file's giant dictionary so make sure to check that out as well.

You'll see some of the functions in reverse order to how they
are in frame_parser.h since it does the parsing before being received
on the CAN bus (eg. GET_WIND_ANGLE and GET_WIND_SPEED)

'''

import math
import struct

UCCM1_CMD_FRAME_ID = 0x00000000
UCCM2_CMD_FRAME_ID = 0x00000001
UCCM3_CMD_FRAME_ID = 0x00000002
UCCM4_CMD_FRAME_ID = 0x00000003
UCCM5_CMD_FRAME_ID = 0x00000004
UCCM6_CMD_FRAME_ID = 0x00000005
UCCM7_CMD_FRAME_ID = 0x00000006
UCCM8_CMD_FRAME_ID = 0x00000007
UCCM9_CMD_FRAME_ID = 0x00000008
UCCM10_CMD_FRAME_ID = 0x00000009
UCCM11_CMD_FRAME_ID = 0x0000000A
UCCM12_CMD_FRAME_ID = 0x0000000B
UCCM13_CMD_FRAME_ID = 0x0000000C
UCCM14_CMD_FRAME_ID = 0x0000000D
UCCM15_CMD_FRAME_ID = 0x0000000E
UCCM16_CMD_FRAME_ID = 0x0000000F
UCCM17_CMD_FRAME_ID = 0x00000010
UCCM18_CMD_FRAME_ID = 0x00000011
UCCM19_CMD_FRAME_ID = 0x00000012

UCCM1_DATA_FRAME_ID = 0x00000013
UCCM2_DATA_FRAME_ID = 0x00000014
UCCM3_DATA_FRAME_ID = 0x00000015
UCCM4_DATA_FRAME_ID = 0x00000016
UCCM5_DATA_FRAME_ID = 0x00000017
UCCM6_DATA_FRAME_ID = 0x00000018
UCCM7_DATA_FRAME_ID = 0x00000019
UCCM8_DATA_FRAME_ID = 0x0000001A
UCCM9_DATA_FRAME_ID = 0x0000001B
UCCM10_DATA_FRAME_ID = 0x0000001C
UCCM11_DATA_FRAME_ID = 0x0000001D
UCCM12_DATA_FRAME_ID = 0x0000001E
UCCM13_DATA_FRAME_ID = 0x0000001F
UCCM14_DATA_FRAME_ID = 0x00000020
UCCM15_DATA_FRAME_ID = 0x00000021
UCCM16_DATA_FRAME_ID = 0x00000022
UCCM17_DATA_FRAME_ID = 0x00000023
UCCM18_DATA_FRAME_ID = 0x00000024
UCCM19_DATA_FRAME_ID = 0x00000025

BMS_CMD_FRAME_ID = 0x00000030
BMS1_FRAME1_ID = 0x00000031
BMS2_FRAME1_ID = 0x00000032
BMS3_FRAME1_ID = 0x00000033
BMS4_FRAME1_ID = 0x00000034
BMS5_FRAME1_ID = 0x00000035
BMS6_FRAME1_ID = 0x00000036

BMS1_FRAME2_ID = 0x00000037
BMS2_FRAME2_ID = 0x00000038
BMS3_FRAME2_ID = 0x00000039
BMS4_FRAME2_ID = 0x0000003A
BMS5_FRAME2_ID = 0x0000003B
BMS6_FRAME2_ID = 0x0000003C

PDB_PORT_FRAME_ID = 0x00000040
PDB_STBD_FRAME_ID = 0x00000041

WINCH_MAIN_CMD_FRAME_ID = 0x00000050
WINCH_JIB_CMD_FRAME_ID = 0x00000051

SAILENCODER_FRAME_ID = 0x00000052

WINCH_MAIN_ANGLE_FRAME_ID = 0x00000053
WINCH_JIB_ANGLE_FRAME_ID = 0x00000054

RUDDER_PORT_CMD_FRAME_ID = 0x00000060
RUDDER_STBD_CMD_FRAME_ID = 0x00000061
RUDDER_PORT_SNS_FRAME_ID = 0x00000062
RUDDER_STBD_SNS_FRAME_ID = 0x00000063

GPS_LAT_FRAME_ID = 0x00000070
GPS_LONG_FRAME_ID = 0x00000071
GPS_OTHER_FRAME_ID = 0x00000072
GPS_DATE_FRAME_ID = 0x00000073

WIND_SENS1_FRAME_ID = 0x00000080
WIND_SENS2_FRAME_ID = 0x00000081
WIND_SENS3_FRAME_ID = 0x00000082

ACCEL_FRAME_ID = 0x00000090
GYRO_FRAME_ID = 0x00000091

NAV_LIGHT_FRAME_ID = 0x00000100


# returns the byte
def get_byte(val, byte):
    return (val & (0xFF << (8 * byte))) >> (8 * byte)


# THIS IS NOT REALLY FRAME_PARSER.H
def GET_GPS_LONG(data):
    upper = twos_complement((get_byte(data, 7) << 24) +
                            (get_byte(data, 6) << 16) +
                            (get_byte(data, 5) << 8) +
                            (get_byte(data, 4) << 0), 32)

    lower = twos_complement((get_byte(data, 3) << 24) +
                            (get_byte(data, 2) << 16) +
                            (get_byte(data, 1) << 8) +
                            (get_byte(data, 0) << 0), 32)/10000000.0
    # performs a unit conversion from decimal degree minutes to degrees
    lon_ddm = upper + lower
    lon_dd = lon_ddm / 100
    lon_m = lon_ddm - (lon_dd*100)
    lon_deg = lon_dd + (lon_m / 60)
    return lon_deg


def GET_GPS_LAT(data):
    upper = twos_complement((get_byte(data, 7) << 24) +
                            (get_byte(data, 6) << 16) +
                            (get_byte(data, 5) << 8) +
                            (get_byte(data, 4) << 0), 32)

    lower = twos_complement((get_byte(data, 3) << 24) +
                            (get_byte(data, 2) << 16) +
                            (get_byte(data, 1) << 8) +
                            (get_byte(data, 0) << 0), 32)/10000000.0
    # performs a unit conversion from decimal degree minutes to degrees
    lat_ddm = upper + lower
    lat_dd = lat_ddm / 100
    lat_m = lat_ddm - (lat_dd*100)
    lat_deg = lat_dd + (lat_m / 60)
    return lat_deg


def GET_GPS_GND_SPEED(data):
    return ((get_byte(data, 7) +
            (get_byte(data, 6) << 8))/100.0)


def GET_GPS_MAG_VAR(data):
    return (get_byte(data, 5) +
            (get_byte(data, 4) << 8))/10.0


def GET_GPS_TMG(data):
    return (get_byte(data, 3) +
            (get_byte(data, 2) << 8))/100.0


def GET_GPS_TRUE_HEADING(data):
    return (get_byte(data, 1) +
            (get_byte(data, 0) << 8))/100.0


def GET_HOUR(data):
    return get_byte(data, 7)


def GET_MINUTE(data):
    return get_byte(data, 6)


def GET_SECOND(data):
    return math.floor(((get_byte(data, 5)) +
                       (get_byte(data, 4) << 8))/100)


def GET_DAY(data):
    return get_byte(data, 3)


def GET_MONTH(data):
    return get_byte(data, 2)


def GET_YEAR(data):
    return get_byte(data, 1)


def GET_STATUS(data):
    return get_byte(data, 0) & 0b0001


def GET_VAR_WEST(data):
    return get_byte(data, 0) & 0b0010


def GET_VAR_NORTH(data):
    return get_byte(data, 0) & 0b0100


def GET_LONG_WEST(data):
    return get_byte(data, 0) & 0b1000


def twos_complement(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val


# SAIL ENCODER
# def GET_SAILENCODER_ANGLE(data):
#     return get_byte(data, 0)


# WIND SENSORS
# ENDIANNESS IS SWAPPED HERE
def GET_WIND_SPEED(data=None):
    new_data = ((get_byte(data, 0) << 24) +
                (get_byte(data, 1) << 16) +
                (get_byte(data, 2) << 8) +
                (get_byte(data, 3) << 0))
    new_data = twos_complement(new_data, 32)/10
    return math.ceil(new_data) if new_data < 0 else math.floor(new_data)


def GET_WIND_ANGLE(data=None):
    new_data = ((get_byte(data, 4) << 24) +
                (get_byte(data, 5) << 16) +
                (get_byte(data, 6) << 8) +
                (get_byte(data, 7) << 0))
    return twos_complement(new_data, 32)


# This function takes in a string of hex and swaps its endianness then converts it to float
def GET_RUDDER_PORT_ANGLE(can_msg=None):
    can_msg = "".join(can_msg.strip().split()[-8:])
    can_msg = can_msg[:len(can_msg)//2]
    can_msg = bytearray.fromhex(can_msg)
    can_msg.reverse()
    new_msg = ''.join(format(x, '02x') for x in can_msg)
    can_msg = struct.unpack('!f', bytes.fromhex(new_msg.upper()))[0]
    return can_msg


# TODO: Process the rest of these functions so we can convert the
# sent data into parsed data for that frame

def GET_ACCEL_X_DATA(data):
    up = get_byte(data, 7)
    low = get_byte(data, 6) << 8
    return twos_complement(up + low, 16)


def GET_ACCEL_Y_DATA(data):
    up = get_byte(data, 5)
    low = get_byte(data, 4) << 8
    return twos_complement(up + low, 16)


def GET_ACCEL_Z_DATA(data):
    up = get_byte(data, 3)
    low = get_byte(data, 2) << 8
    return twos_complement(up + low, 16)


def GET_GYRO_X_DATA(data):
    up = get_byte(data, 7)
    low = get_byte(data, 6) << 8
    return twos_complement(up + low, 32)/1000.0


def GET_GYRO_Y_DATA(data):
    up = get_byte(data, 5)
    low = get_byte(data, 4) << 8
    return twos_complement(up + low, 32)/1000.0


def GET_GYRO_Z_DATA(data):
    up = get_byte(data, 3)
    low = get_byte(data, 2) << 8
    return twos_complement(up + low, 32)/1000.0


# BMS
# TODO(anyone): this is assuming voltage,
# current, etc cannot be negative.
# is that true?
# def GET_BMS_VOLT_DATA(data):
#     return int(data[0] + (data[1] << 8))
#
#
# def GET_BMS_CURR_DATA(data):
#     return (data[2] + (data[3] << 8) / 100.0)

# #define GET_BMS_MAXCELL_DATA(data) {\
#    static_cast<uint16_t>(data[4] + (data[5] << 8)) \
# }
#
# #define GET_BMS_MINCELL_DATA(data) {\
#    static_cast<uint16_t>(data[6] + (data[7] << 8)) \
# }
#
