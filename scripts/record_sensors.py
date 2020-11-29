#!/usr/bin/env python3

# Record sensor data and write it on a .csv file
# .csv file is timestamped so that they dont overwrite each other.
# You specify the channel
# To run the script---> python3 record_sensors.py -i [channel]
# (OPTIONAL) Print the data to terminal---> python3 record_sensors.py -i [channel] -v True
# ***NOTE: The data recieved from the canbus will be written in little endian notation.
#          For example, if the bytearray received from the canbus is [0x1a, 0x2b, 0x3c],
#          then it will be written into the csv file as "3c2b1a".

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
                elapsed_time = (time.time() - start)*1000
                writer.writerow(["{:.0f}".format(elapsed_time), str(
                    msg.arbitration_id), bytes(msg.data).hex()[::-1]])
            except can.CanError:
                print("Error Recording data to csv")
                pass


def main():
    parser = argparse.ArgumentParser(description="Sensor recording")
    parser.add_argument(
        "-c", "--channel", help="Enter the canbus channel you want to record", default=None)
    parser.add_argument("-v", "--verbose",
                        help="Print out what you are recording", default=False)
    args = parser.parse_args()

    input_channel = args.channel
    verbose = args.verbose

    if input_channel is None:
        print("Error: input channel is None")
        return

    record_sensor_data(input_channel, verbose)

    return


if __name__ == "__main__":
    main()
