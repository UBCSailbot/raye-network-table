from nt_connection.Connection import Connection
import generated_python.Value_pb2 as Value_pb2
import generated_python.Satellite_pb2 as Satellite_pb2
import generated_python.Node_pb2 as Node_pb2
from http.server import HTTPServer, BaseHTTPRequestHandler
import http.server
import requests
import threading
import time
import sys


def main():
    print("Connecting to network table")
    nt_connection = Connection()
    nt_connection.Connect()

    for x in range(1, 100):
        value = Value_pb2.Value()
        value.type = Value_pb2.Value.Type.WAYPOINTS
        gpsCoord = value.waypoints.add()
        gpsCoord.latitude = x
        gpsCoord.longitude = x
        gpsCoord = value.waypoints.add()
        gpsCoord.latitude = x
        gpsCoord.longitude = x

        uri = "waypoints"
        print(value)
        values = {uri: value}
        nt_connection.setValues(values)
        time.sleep(5)


if __name__ == "__main__":
    main()
