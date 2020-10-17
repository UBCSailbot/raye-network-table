#!/usr/bin/env python3

# Script to mock UCCM sensor data on a virtual CAN bus on vcan0
# Sends garbage data but you specify the SID
# --> needs virtual can set up first -> ./setup_vcan.sh
# needs server ->  in build/bin -> ./network_table_server &
# You should run this as a background job -> python3 mock_sensors.py -c vcan0 &
# -------------------------------------------------------------------------------------
# Script also can playback sensor data recorded from a log file to a virtual CAN bus
# **ONLY takes in a csv file created from the record_sensors script**
# --> Needs virtual can set up first -> ./setup_vcan.sh
# --> To run script -> python3 mock_sensors.py -c vcan0 -f [file_name]
# (OPTIONAL) Continuously loop through the data -> python3 mock_sensors.py -c vcan0 -f [file_name] --loop

import time
import can
import sys
import argparse
import csv
import os


def playback_sensor_data(input_file, channel, loop):
    bus = can.interface.Bus(bustype='socketcan',
                            channel=channel, bitrate=250000)
    print("Sending CAN messages on {}".format(bus.channel))
    logfile = open(input_file, "r")
    # Prevents the first line in the csv file that outlines column names from being scanned
    next(logfile)
    last_line = None
    while True:
        reader = csv.reader(logfile)
        for line in reader:
            msg = can.Message(arbitration_id=int(
                line[1]), dlc=None, data=bytes.fromhex(line[2]))
            # Skip sleeping when reading first line: no previous timestamp to reference off from
            if last_line is not None:
                # Calculate the amount of time to sleep for by
                # getting the different between the current and last
                # timestamp
                time.sleep((int(line[0])-int(last_line[0]))/1000)
            try:
                bus.send(msg)
            except can.CanError:
                print("Error Sending on CAN bus")
                pass
            last_line = line
        if loop:
            logfile.seek(0)
            next(logfile)
            last_line = None
            time.sleep(1)
        else:
            break


def send_sensor_data(device, SID, channel):
    bus = can.interface.Bus(bustype='socketcan',
                            channel=channel, bitrate=250000)
    print("Sending CAN messages on {}".format(bus.channel))
    print("device = {} \nRTR: 0b0\nDLC: 8\n".format(device))
    if device == 'bms':
        while True:
            msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                              20, 20, 20, 20, 20, 20, 20, 20])
            try:
                bus.send(msg)
            except can.CanError:
                print("Error Sending on CAN bus")
                pass
            time.sleep(3)
    elif device == 'gps':
        while True:

            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                                  50, 50, 50, 50, 50, 50, 50, 50])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass

    elif device == 'sail':
        while True:
            # cycles through all of the CAN ID frames for
            # the gparser.add_argument("-d", "--device_id",
            # help="device to simulate. Options: gps, bms, wind, acc", default='all')ps
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                                  40, 40, 40, 40, 40, 40, 40, 40])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass

    elif device == 'wind':
        while True:
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                                  30, 30, 30, 30, 30, 30, 30, 30])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass
    elif device == 'acc':
        while True:
            msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                              10, 10, 10, 10, 10, 10, 10, 10])
            try:
                bus.send(msg)
            except can.CanError:
                print("Error Sending on CAN bus")
                pass
    else:  # if device == all
        while True:
            for i in range(len(SID[device])):
                msg = can.Message(arbitration_id=SID[device][i], dlc=0x8, data=[
                                  1, 1, 1, 1, 1, 1, 1, 1])
                try:
                    bus.send(msg)
                except can.CanError:
                    print("Error Sending on CAN bus")
                    pass


def main():
    # SID is taken from frame_parser.h data relating to the CAN ID of each device
    # TODO: this is duplicated a bunch of times and should only exist in a single
    # file somewhere
    SID = {'bms': [0x08],
           'gps': [0x11, 0x100, 0x101, 0x110],
           'sail': [0x0F],
           'wind': [0x0, 0x10, 0x111],
           'acc': [0xAC],
           'all': [0x08, 0x11, 0x100, 0x101,
                   0x110, 0x0F, 0x0, 0x10,
                   0x111, 0xAC]}

    parser = argparse.ArgumentParser(description="Sensor mocking")
    parser.add_argument(
        "-d", "--device_id", help="device to simulate. Options: gps, bms, wind, acc", default='all')
    parser.add_argument("-c", "--channel",
                        help="Input the channel you want to use", default=None)
    parser.add_argument(
        "-f", "--file_name", help="Input the csv sensor data file you wish to mock", default=None)
    parser.add_argument(
        "--loop", help="Continuously playback the sensor data in a loop", action='store_true')
    args = parser.parse_args()
    channel = args.channel
    logfile = args.file_name
    loop = args.loop

    script_location = os.path.dirname(os.path.realpath(__file__))

    if channel is None:
        print("Error: channel not specified")
        print("Example usage: 'python3 mock_sensors.py -c vcan0'")
        return

    if logfile:
        if loop:
            print("Loop = ON")
        else:
            print("Loop = OFF")
        logfile_fullpath = os.path.join(script_location, logfile)
        playback_sensor_data(logfile_fullpath, channel, loop)
    else:
        send_sensor_data(args.device_id, SID, channel)

    return


if __name__ == "__main__":
    main()
