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


def wind_sensor_check(input_channel):
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel, bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['wind']:
            print("got wind data")
            print(msg)
            return motor_output_check(bus, 15)


def motor_output_check(bus, timeout):
    start = time.time()
    while True:
        msg = bus.recv()
        # This is technically checking the rudder output
        if msg.arbitration_id in SID['motor']:
            print("motor output data received")
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

    if args.channel is None:
        print("Error: channel not specified")
        print("Example usage: 'python3 canbus_data_checker.py -c vcan0'")
        sys.exit(-1)

    if wind_sensor_check(args.channel):
        print("Detected motor outputs. Passed!")
        sys.exit(0)
    else:
        print("Failed to detect motor outputs. Failed.")
        sys.exit(-1)
