#!/usr/bin/env python3

# This script will:
# 1 Run the network table server.
# 2 Run multiple clients which will
#   communicate with the network table.
# 3 Determine how many errors occured
#   based on the return values of the clients.
#   The clients return 0 if no
#   errors occured, otherwise return the
#   number of errors.

import argparse
import os
import signal
import subprocess
import sys
from time import sleep
from threading import Thread
from nt_connection.Connection import *
from generated_python import Value_pb2

continue_server = True
error_occured = False


def run_server():
    """Runs the network table server"""
    server = subprocess.Popen(['./bin/network_table_server'],
                              preexec_fn=os.setsid)
    while continue_server:
        sleep(5)
        if not continue_server:
            os.killpg(os.getpgid(server.pid), signal.SIGTERM)
            return


def run_server_and_fake_crashes():
    """Runs the network table server, but closes the server and restarts it every few seconds.
    This is to simulate the server crashing and restarting."""
    while continue_server:
        server = subprocess.Popen(['./bin/network_table_server'],
                                  preexec_fn=os.setsid)
        for i in range(1, 5):
            if not continue_server:
                os.killpg(os.getpgid(server.pid), signal.SIGTERM)
                return

            sleep(1)
        # TODO: There might be a signal other than SIGTERM
        # which more closely simulates a crash.
        os.killpg(os.getpgid(server.pid), signal.SIGTERM)
        sleep(.01)

    os.killpg(os.getpgid(server.pid), signal.SIGTERM)


def run_python_client():
    nt_connection = Connection()
    nt_connection.Connect()

    num_queries = 10
    for i in range(num_queries):
        set_value_a = Value_pb2.Value()
        set_value_a.type = Value_pb2.Value.Type.INT
        set_value_a.int_data = 11
        set_value_b = Value_pb2.Value()
        set_value_b.type = Value_pb2.Value.Type.STRING
        set_value_b.string_data = "HELLO"
        try:
            nt_connection.setValues(
                    {'/python_client_dummy_data/a': set_value_a,
                     '/python_client_dummy_data/b': set_value_b})
        except ConnectionError as e:
            print(str(e))
            error_occured = True
            return

        try:
            get_node = nt_connection.getNodes(
                    ['/python_client_dummy_data/a',
                     '/python_client_dummy_data/b'])
            get_int_data = get_node['/python_client_dummy_data/a'].value.int_data
            if not get_int_data == set_value_a.int_data:
                # Use old style python string formatting, the BBB is still on python 3.5
                print("expected {}, got back {}".format(set_value_a.int_data, get_int_data))
                error_occured = True
                return

            get_string_data = get_node['/python_client_dummy_data/b'].value.string_data
            if not get_string_data == set_value_b.string_data:
                print("expected {}, got back {}".format(set_value_b.string_data, get_string_data))
                error_occured = True
                return

        except ConnectionError as e:
            print(str(e))
            error_occured = True
            return

    nt_connection.Disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Network Table Stress Test")
    parser.add_argument(
        "-n", "--num-clients",
        help="Enter number of clients to connect to network table",
        type=int,
        default=20)
    parser.add_argument(
        "--compile", help="Compile the network table first",
        action='store_true')
    parser.add_argument(
        "--disable-fake-crashes", help="Stop the network table\
                from crashing/restarting",
        action='store_true')
    parser.add_argument(
        "--clients-only", help="Only run the clients, no server",
        action='store_true')
    parser.add_argument(
        "--python-client", help="Runs the python client\
                instead of the C++ client",
        action='store_true')
    args = parser.parse_args()

    num_clients = args.num_clients
    disable_fake_crashes = args.disable_fake_crashes
    clients_only = args.clients_only
    python_client = args.python_client
    build = args.compile  # variable name compile is not allowed

    # Go to the build directory
    script_location = os.path.dirname(os.path.realpath(__file__))
    build_location = os.path.abspath(script_location + '/../build')
    os.chdir(build_location)

    if build:
        try:
            make = subprocess.check_call('make')
        except subprocess.CalledProcessError:
            print("could not compile project")
            sys.exit(-1)

    # Start the server
    if not clients_only:
        print("This test may take a few seconds.")
        if disable_fake_crashes:
            server_thread = Thread(target=run_server)
        else:
            server_thread = Thread(target=run_server_and_fake_crashes)
        server_thread.start()

    if python_client:
        clients = [Thread(target=run_python_client)
                   for i in range(num_clients)]

        for client in clients:
            client.start()

        for client in clients:
            client.join()

    else:
        # This is an array of client processes which will communicate with the server.
        # They will all run at the same time, then the return value of each
        # one will be checked.
        clients = [subprocess.Popen(['./bin/client'],
                                    preexec_fn=os.setsid)
                   for i in range(num_clients)]

        for client in clients:
            client.wait()
            if not client.returncode == 0:
                error_occured = True

        # Terminal any remaining processes
        for client in clients:
            try:
                os.killpg(os.getpgid(client.pid), signal.SIGTERM)
            except OSError:
                # The client should have exited normally.
                pass

    if not clients_only:
        continue_server = False
        server_thread.join()

    if error_occured:
        print("One or more clients failed.")
        sys.exit(-1)
    else:
        print("No problems detected.")
        sys.exit(0)
