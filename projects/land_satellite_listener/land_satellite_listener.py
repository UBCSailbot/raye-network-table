#!/usr/bin/env python3

from nt_connection.Connection import Connection
from nt_connection.Help import Help
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
import keyring
from requests.auth import HTTPBasicAuth
from functools import partial


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, nt_connection, accepted_ip_addresses, *args, **kwargs):
        self.nt_connection = nt_connection
        self.accepted_ip_addresses = accepted_ip_addresses
        super().__init__(*args, **kwargs)

    def do_POST(self):
        if self.client_address[0] in self.accepted_ip_addresses:
            print("Handling post request")
            content_len = int(self.headers['Content-Length'])
            body = self.rfile.read(content_len)
            sat = Satellite_pb2.Satellite()
            helper = Help()

            try:
                sat.ParseFromString(body)
                if sat.type == Satellite_pb2.Satellite.Type.SENSORS:
                    print("Receiving Sensor Data")
                    values = helper.sensors_to_root(sat.sensors)
                    self.nt_connection.setValues(values)

                elif sat.type == Satellite_pb2.Satellite.Type.UCCMS:
                    print("Receiving UCCM Data")
                    values = helper.uccms_to_root(sat.uccms)
                    self.nt_connection.setValues(values)

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
                pass

            except ConnectionError:
                print("Error setting network table value")
                pass
        else:
            print("**ERROR: Client's IP Address is not valid, cancelling post request...")


class runServer(threading.Thread):
    def __init__(self, port, target_address, nt_connection, accepted_ip_addresses):
        """ Server class that receives satellite data from the boat

                     port - http server port number
            nt_connection - network table connection instance
        """
        threading.Thread.__init__(self)
        self.port = port
        self.target_address = target_address
        self.nt_connection = nt_connection
        self.accepted_ip_addresses = accepted_ip_addresses

    def run(self):
        """ Initiates the HTTP Server"""
        handler = partial(HTTPRequestHandler, self.nt_connection, self.accepted_ip_addresses)
        httpd = HTTPServer((self.target_address, self.port), handler)
        httpd.serve_forever()


class runClient(threading.Thread):
    def __init__(self, nt_connection, poll_freq, ENDPOINT, username, password):
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
        self.username = username
        self.password = password

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

                if self.password is None or self.username is None:
                    security = None
                else:
                    security = {'username': self.username, 'password': self.password}

                if (node.value.type == Value_pb2.Value.Type.WAYPOINTS
                        and node.value != prev_sat.value):
                    cur_sat.value.CopyFrom(node.value)
                    requests.post(self.ENDPOINT, params=security, data=cur_sat.SerializeToString())
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

    parser.add_argument(
        '-b',
        '--bind',
        type=str,
        help='Specifies a target address to bind to when sending requests',
        default="",
        required=False)

    parser.add_argument('-n',
                        '--username',
                        type=str,
                        help='Username for RockBlock')

    parser.add_argument('-w',
                        '--password',
                        type=str,
                        help='Password for RockBlock')

    parser.add_argument('-i',
                        '--ip_addresses',
                        nargs='+',
                        type=str,
                        help='Accepted ip addresses for RockBlock')

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

    server = runServer(args.port, args.bind, nt_connection, args.ip_addresses)
    client = runClient(nt_connection, poll_freq, args.endpoint, args.username, args.password)
    server.start()
    client.start()


if __name__ == "__main__":
    main()
