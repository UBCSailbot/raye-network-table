#!/usr/bin/env python3

"""mock_waypoint_data.py: Populates the network table with mock waypoints to mimic a global path."""

__author__ = "Brielle Law (briellelaw)"
__copyright__ = "Copyright 2020 UBC Sailbot"

from nt_connection.Connection import Connection
from nt_connection.uri import *
import generated_python.Value_pb2 as Value_pb2
import generated_python.Satellite_pb2 as Satellite_pb2
import generated_python.Node_pb2 as Node_pb2
from http.server import HTTPServer, BaseHTTPRequestHandler
import http.server
import requests
import threading
import time
import sys
import argparse

TEST_LAT1 = 49.275202
TEST_LON1 = -123.160124
TEST_LAT2 = 49.212129
TEST_LON2 = -123.307560
TEST_LAT3 = 49.059004
TEST_LON3 = -123.494778

NUM_WAYPOINTS = 5
FREQUENCY = 10


def main():
    parser = argparse.ArgumentParser(description="Runs on the land server"
                                     " to populate network table with a mock global path")

    parser.add_argument('-l',
                        '--launch',
                        action='store_true',
                        help='Enabled for Raye launch test',
                        required=False)

    parser.add_argument('-t',
                        '--test',
                        action='store_true',
                        help='Enabled to continously set waypoints',
                        required=False)

    args = parser.parse_args()

    is_launch_test = args.launch
    is_continuous_test = args.test

    if not is_launch_test and not is_continuous_test:
        parser.print_help()
        return

    print("Connecting to network table")
    nt_connection = Connection()
    nt_connection.Connect()

    if is_launch_test is True:
        print("Setting Global Path for Raye Launch Test")
        value = Value_pb2.Value()
        value.type = Value_pb2.Value.Type.WAYPOINTS

        # Coordinate near shore
        gpsCoord = value.waypoints.add()
        gpsCoord.latitude = TEST_LAT1
        gpsCoord.longitude = TEST_LON1

        # Coordinate near Iona
        gpsCoord = value.waypoints.add()
        gpsCoord.latitude = TEST_LAT2
        gpsCoord.longitude = TEST_LON2

        # Coordinate near Island
        gpsCoord = value.waypoints.add()
        gpsCoord.latitude = TEST_LAT3
        gpsCoord.longitude = TEST_LON3

        uri = WAYPOINTS_GP
        print("Setting mock global path:")
        print(value)

        values = {uri: value}
        nt_connection.setValues(values)

    else:
        for x in range(1, 100):
            value = Value_pb2.Value()
            value.type = Value_pb2.Value.Type.WAYPOINTS
            for i in range(1, NUM_WAYPOINTS):
                gpsCoord = value.waypoints.add()
                gpsCoord.latitude = x
                gpsCoord.longitude = x

            uri = WAYPOINTS_GP
            print("Setting mock global path:")
            print(value)
            values = {uri: value}
            nt_connection.setValues(values)
            time.sleep(FREQUENCY)


if __name__ == "__main__":
    main()
