# This script will: 
# 1 Build the network table server
#   and an example client program. 
# 2 Run the network table server.
# 3 Run multiple clients which will
#   communicate with the network table.
# 4 Determine how many errors occured
#   based on the return values of the clients.
#   The clients should return 0 if no
#   errors occured, otherwise return the
#   number of errors.

import os
import signal
import subprocess
import sys
from time import sleep
from threading import Thread

continue_server = True

def run_server_and_fake_crashes():
    """Runs the network table server, but closes the server and restarts it every few seconds.
    This is to simulate the server crashing and restarting."""
    while continue_server:
        server = subprocess.Popen(['./bin/server'],
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

# Get the number of clients which will
# be querying the network table.
if len(sys.argv) != 2:
    num_clients = 100
else:
    num_clients = int(sys.argv[1])

# Go to the build directory, and build
# the latest version of the server and client
script_location = os.path.dirname(os.path.realpath(__file__))
build_location = os.path.abspath(script_location + '/../build')
os.chdir(build_location)

try:
    make = subprocess.check_call('make')
except subprocess.CalledProcessError:
    print "could not compile project"
    exit()

# Start the server
print "This test may take a few seconds."
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
    print "No problems detected."
else:
    print "One or more clients failed."

# Terminal any remaining processes
continue_server = False
server_thread.join()
for client in clients:
    try:
        os.killpg(os.getpgid(client.pid), signal.SIGTERM)
    except:
        # The client should have exited normally.
        pass
