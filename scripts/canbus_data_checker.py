#!/usr/bin/env python3

# Check to see if wind sensor data
# is coming out of the canbus network.
# If it is, check to see if motor outputs
# are coming out of the canbus network.
# If they are, return success code,
# otherwise return error code.

import argparse
import can
import time
import sys

# TODO: Get rid of this
# its duplicated in idk how many places
SID = {'bms': [0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C],
       'gps': [0x70, 0x71, 0x72, 0x73],
       'sail': [0x52],
       'motor': [0x60, 0x61, 0x62, 0x63],
       'wind': [0x80, 0x81, 0x82],
       'acc': [0x90]}


def wind_sensor_check(input_channel, timeout):
    start = time.time()
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel,
                            bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['wind']:
            print("wind data received")
            print(msg)
            return True
        else:
            elapsed_time = (time.time() - start)
            if elapsed_time > timeout:
                return False


def acc_sensor_check(input_channel, timeout):
    start = time.time()
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel,
                            bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['acc']:
            print("accelerator data received")
            print(msg)
            return True
        else:
            elapsed_time = (time.time() - start)
            if elapsed_time > timeout:
                return False


def gps_sensor_check(input_channel, timeout):
    start = time.time()
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel,
                            bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['gps']:
            print("accelerator data received")
            print(msg)
            return True
        else:
            elapsed_time = (time.time() - start)
            if elapsed_time > timeout:
                return False


def rudder_output_check(input_channel, timeout):
    start = time.time()
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel,
                            bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['motor']:
            print("rudder data received")
            print(msg)
            return True
        else:
            elapsed_time = (time.time() - start)
            if elapsed_time > timeout:
                return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CANbus data checker")
    parser.add_argument(
        "-c", "--channel", help="Enter the canbus channel you want to record", default=None)
    args = parser.parse_args()

    # TODO: add a flag once all the sensors are fully integrated with can
    if args.channel is None:
        print("Error: channel not specified")
        print("Example usage: 'python3 canbus_data_checker.py -c vcan0'")
        sys.exit(-1)

    if acc_sensor_check(args.channel, 1):
        print("Detected accelerometer. Passed!")
    else:
        print("Failed to detect accelerometer.")

    if wind_sensor_check(args.channel, 1):
        print("Detected wind sensor. Passed!")
    else:
        print("Failed to detect wind sensor.")

    if gps_sensor_check(args.channel, 1):
        print("Detected gps sensor. Passed!")
    else:
        print("Failed to detect gps sensor.")

    if rudder_output_check(args.channel, 1):
        print("Detected rudder output. Passed!")
    else:
        print("Failed to detect rudder output.")

    sys.exit(0)
