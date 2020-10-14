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

continue_server = True


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


if __name__ == "__main__":
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
        "--disable-fake-crashes", help="Stop the network table from crashing/restarting",
        action='store_true')
    args = parser.parse_args()

    num_clients = args.num_clients
    disable_fake_crashes = args.disable_fake_crashes
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
    print("This test may take a few seconds.")
    if disable_fake_crashes:
        server_thread = Thread(target=run_server)
    else:
        server_thread = Thread(target=run_server_and_fake_crashes)
    server_thread.start()

    # This is an array of client processes which will communicate with the server.
    # They will all run at the same time, then the return value of each
    # one will be checked.
    clients = [subprocess.Popen(['./bin/client'],
                                preexec_fn=os.setsid)
               for i in range(num_clients)]

    errors_occured = 0
    for client in clients:
        client.wait()
        errors_occured += client.returncode

    if errors_occured == 0:
        print("No problems detected.")
    else:
        print("One or more clients failed.")

    # Terminal any remaining processes
    continue_server = False
    server_thread.join()
    for client in clients:
        try:
            os.killpg(os.getpgid(client.pid), signal.SIGTERM)
        except OSError:
            # The client should have exited normally.
            pass
    sys.exit(errors_occured)
