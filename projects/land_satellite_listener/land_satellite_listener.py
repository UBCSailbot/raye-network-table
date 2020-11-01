#!/usr/bin/env python3

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
import argparse


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        print("Handling post request")
        content_len = int(self.headers['Content-Length'])
        body = self.rfile.read(content_len)
        sat = Satellite_pb2.Satellite()

        try:
            sat.ParseFromString(body)
            if sat.type == Satellite_pb2.Satellite.Type.SENSORS:
                print("Receiving Sensor Data")
                print(sat.sensors)
            elif sat.type == Satellite_pb2.Satellite.Type.UCCMS:
                print("Receiving UCCM Data")
                print(sat.uccms)
            elif sat.type == Satellite_pb2.Satellite.Type.VALUE:
                print("Receiving Waypoint Data")
                print(sat.value)
            else:
                print("Did Not receive Sensor or UCCM data")
                print(body)

            self.send_response(200)
            self.end_headers()

        except IOError:
            print("Error Decoding incoming data")


class runServer(threading.Thread):
    def __init__(self, port):
        """ Server class that receives satellite data from the boat

            port - http server port number
        """
        threading.Thread.__init__(self)
        self.port = port

    def run(self):
        """ Initiates the HTTP Server"""
        httpd = HTTPServer(("", self.port), HTTPRequestHandler)
        httpd.serve_forever()


class runClient(threading.Thread):
    def __init__(self, nt_connection, poll_freq, ENDPOINT):
        """ Client class that sends waypoints to the boat
            via satellite

            nt_connection - network table connection instance
                poll_freq - frequency at which the network table is
                            polled (seconds)
                 ENDPOINT - endpoint for POSTing waypoint data
        """
        threading.Thread.__init__(self)
        self.nt_connection = nt_connection
        self.poll_freq = poll_freq
        self.ENDPOINT = ENDPOINT

    def init_waypoints(self):
        """ Initializes a satellite object that can be sent to the boat"""
        satellite = Satellite_pb2.Satellite()
        satellite.type = Satellite_pb2.Satellite.Type.VALUE
        satellite.value.type = Value_pb2.Value.Type.WAYPOINTS
        return satellite

    def run(self):
        """ Polls the network table for changes in global
            pathfinding waypoints and sends changes to
            the boat via satellite
        """
        prev_sat = self.init_waypoints()
        cur_sat = self.init_waypoints()

        while (True):
            try:
                # Check connection to network table
                assert self.nt_connection.connected
                uris = ["waypoints"]
                node_container = self.nt_connection.getNodes(uris)

                node = Node_pb2.Node()
                node.CopyFrom(node_container["waypoints"])

                if (node.value.type == Value_pb2.Value.Type.WAYPOINTS
                        and node.value != prev_sat.value):
                    cur_sat.value.CopyFrom(node.value)
                    requests.post(
                        self.ENDPOINT, data=cur_sat.SerializeToString())
                    prev_sat.value.CopyFrom(cur_sat.value)

                time.sleep(self.poll_freq)

            except ConnectionError:
                print("Error getting waypoints from network table")
                pass


def check_pos(value):
    """Checks that arguments are valid positive integers """
    int_value = int(value)
    if int_value <= 0:
        raise argparse.ArgumentTypeError(
            "%s is an invalid positive int value" % value)
    return int_value


def main():
    parser = argparse.ArgumentParser(description="Runs on the land server"
                                     "to poll and write to the network table")

    parser.add_argument('-p',
                        '--port',
                        metavar='SERVER_PORT',
                        type=check_pos,
                        help='port number for http server',
                        required=True)

    parser.add_argument('-e',
                        '--endpoint',
                        metavar='ENDPOINT',
                        type=str,
                        help='endpoint for client-side http requests',
                        required=True)

    parser.add_argument('-f',
                        '--freq',
                        metavar='POLL_FREQUENCY',
                        type=check_pos,
                        help='frequency to poll network table',
                        required=True)

    parser.add_argument('-u',
                        '--unit',
                        type=str,
                        help='time unit for polling frequency',
                        choices=["SEC", "MIN", "HR"],
                        required=True)

    args = parser.parse_args()

    # Get the polling frequency in seconds
    poll_freq = args.freq
    freq_unit = args.unit

    if (freq_unit == "MIN"):
        poll_freq *= 60
    elif(freq_unit == "HR"):
        poll_freq *= 3600

    print("serving at port", args.port)
    print("Connecting to network table")
    nt_connection = Connection()
    nt_connection.Connect()

    server = runServer(args.port)
    client = runClient(nt_connection, poll_freq, args.endpoint)
    server.start()
    client.start()


if __name__ == "__main__":
    main()
