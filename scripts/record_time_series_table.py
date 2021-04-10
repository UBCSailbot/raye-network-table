#!/usr/bin/env python3

# A script that will convert a logfile into a time-series datatable.
# NOTE: **This script WILL ONLY work on csv files recorded by the record_sensors script**
# example usage: python3 record_time_series_table.py -f [directory/FILE_NAME or FILE_NAME]

import argparse
import csv
import time
import json
import os
import math


def record_time_series_table(input_file):
    start = time.time()
    with open('time_series_table_{}.csv'.format(start), mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp(s)", "ID", "Wind Speed (knots)", "Wind Angle (deg)", "Longitude (DDMM)",
                         "Latitude (DDMM)", "Ground Speed (knots)", "Magnetic Variation (deg)",
                         "Track Made Good (deg)",
                         "Sail-Encoder Angle (deg)", "X force (milli-g)", "Y force (milli-g)",
                         "Z force (milli-g)"])
        # A list to contain the data received: the index "i" in data_array[i] corresponds to "i" in writerow[i]
        data_array = ["N/A"] * 11
        # Open json file to recieve the ID's from frame_ids.json
        frame_ids = json.load(open("../src/frame_ids.json"))
        # Open the csv data file you input
        logfile = open(input_file, "r")
        next(logfile)
        reader = csv.reader(logfile)
        for line in reader:
            data = bytes.fromhex(line[2])
            parse_data(hex(int(line[1])), data, data_array, frame_ids)
            writer.writerow([(int(line[0]) / 1000),
                             # ID
                             line[1],
                             # Wind speed
                             data_array[0],
                             # Wind Angle
                             data_array[1],
                             # Longitude
                             data_array[2],
                             # Latitude
                             data_array[3],
                             # GND speed
                             data_array[4],
                             # Magnetic Variation
                             data_array[5],
                             # TMG
                             data_array[6],
                             # Sail Encoder Angle
                             data_array[7],
                             # Acceleration x
                             data_array[8],
                             # Acceleration y
                             data_array[9],
                             # Acceleration z
                             data_array[10]])


def parse_data(ID, data, data_array, frame_ids):
    if ID == frame_ids["WIND_SENS0_FRAME_ID" or "WIND_SENS1_FRAME_ID" or "WIND_SENS2_FRAME_ID"]:
        data_array[0] = ((data[4] +
                         (data[5] << 8) +
                         (data[6] << 16) +
                         (data[7] << 24)) / 10)
        data_array[1] = (data[0] +
                         (data[1] << 8) +
                         (data[2] << 16) +
                         (data[3] << 24))
    if ID == frame_ids["GPS_LONG_FRAME_ID"]:
        data_array[2] = (((data[0] << 0) +
                         (data[1] << 8) +
                         (data[2] << 16) +
                         (data[3] << 24)) +
                         (((data[4] << 0) +
                          (data[5] << 8) +
                          (data[6] << 16) +
                          (data[7] << 24))/10000000.0))
    if ID == frame_ids["GPS_LAT_FRAME_ID"]:
        data_array[3] = (((data[0] << 0) +
                         (data[1] << 8) +
                         (data[2] << 16) +
                         (data[3] << 24)) +
                         (((data[4] << 0) +
                          (data[5] << 8) +
                          (data[6] << 16) +
                          (data[7] << 24))/10000000.0))
    if ID == frame_ids["GPS_OTHER_FRAME_ID"]:
        data_array[4] = float((((data[0]) +
                              (data[1] << 8))/100.0))
        data_array[5] = float((((data[2]) +
                              (data[3] << 8))/10.0))
        data_array[6] = float((((data[4]) +
                              (data[5] << 8))/100.0))
    if ID == frame_ids["SAILENCODER_FRAME_ID"]:
        data_array[7] = data[0]
    if ID == frame_ids["ACCEL_FRAME_ID"]:
        data_array[8] = int((data[0] +
                            (data[1] << 8)))
        data_array[9] = int(((data[2] +
                            (data[3] << 8))))
        data_array[10] = int(((data[4] +
                             (data[5] << 8))))


def main():
    parser = argparse.ArgumentParser(description="csv data file")
    parser.add_argument(
        "-f", "--file", help="Input the .csv file you want to convert to a Time Series Data Table", default=None)
    args = parser.parse_args()

    input_file = args.file

    if input_file is None:
        print("Error: You have not provided an input file")
        return

    script_location = os.path.dirname(os.path.realpath(__file__))
    input_file_fullpath = os.path.join(script_location, input_file)
    record_time_series_table(input_file_fullpath)
    return


if __name__ == "__main__":
    main()
