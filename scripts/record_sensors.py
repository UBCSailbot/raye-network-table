#!/usr/bin/env python3
# Record sensor data and write it on a .csv file
# You specify the channel
# To run the script---> python3 record_sensors.py -i [channel]
# (OPTIONAL) Print the data to terminal---> python3 record_sensors.py -i [channel] -v True

import argparse
import sys
import csv
import can
import time
import datetime


def record_sensor_data(input_channel, verbose=False):
    bus = can.interface.Bus(bustype='socketcan',
                            channel=input_channel, bitrate=250000)
    print("Receiving CAN messages on {}".format(bus.channel))
    timestamp = time.strftime('%H:%M:%S', time.localtime())
    start = time.time()
    with open('canbus_{}.csv'.format(timestamp), mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp(ms)", "ID", "Data"])
        while True:
            msg = bus.recv()
            if verbose:
                print(msg)
            try:
                time_elapsed = (time.time() - start)*1000
                writer.writerow(["{:.0f}".format(time_elapsed), str(
                    msg.arbitration_id), bytes(msg.data).hex()])
            except can.CanError:
                print("Error Recording data to csv")
                pass


def main():
    parser = argparse.ArgumentParser(description="Sensor recording")
    parser.add_argument(
        "-i", "--input", help="Enter the canbus channel you want to record", default=None)
    parser.add_argument("-v", "--verbose",
                        help="Print out what you are recording", default=False)
    args = parser.parse_args()

    input_channel = args.input
    verbose = args.verbose

    if input_channel is None:
        print("Error: input channel is None")
        return

    record_sensor_data(input_channel, verbose)

    return


if __name__ == "__main__":
    main()
