#!/usr/bin/env python3
import can
import time
import sys

SID = {'bms': [0x08],
       'gps': [0x11, 0x100, 0x101, 0x110],
       'sail': [0x0F],
       'motor': [0xAB],
       'wind': [0x0, 0x10, 0x111],
       'acc': [0xAC],
       'all': [0x08, 0x11, 0x100, 0x101,
               0x110, 0x0F, 0x0, 0x10,
               0x111, 0xAC]}


def wind_sensor_check(input_channel):
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel, bitrate=250000)
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['wind']:
            print("got wind data")
            print(msg)
            return motor_output_check(bus, 5)


def motor_output_check(bus, timeout):
    start = time.time()
    while True:
        msg = bus.recv()
        if msg.arbitration_id in SID['motor']:
            print("motor output data received")
            print(msg)
            return True
        else:
            elapsed_time = (time.time() - start)
            if elapsed_time > timeout:
                return False


if __name__ == "__main__":
    if wind_sensor_check("can0"):
        print("Detected motor outputs. Passed!")
        sys.exit(0)
    else:
        print("Failed to detect motor outputs. Failed.")
        sys.exit(-1)
