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

# Get the number of clients which will
# be querying the network table.
if len(sys.argv) != 2:
    numClients = 100
else:
    numClients = int(sys.argv[1])

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
server = subprocess.Popen(['./bin/server'],
                          preexec_fn=os.setsid)

# This is an array of client processes which will communicate with the server.
# They will all run at the same time, then the return value of each
# one will be checked.
clients = [subprocess.Popen(['./bin/client'],
                            preexec_fn=os.setsid)
                   for i in range(numClients)]

numErrors = 0
for client in clients:
    client.wait()
    numErrors += client.returncode

if numErrors == 0:
    print "No problems detected."
else:
    print "A total of " + str(numErrors) + " errors occured."

# Terminal any remaining processes
os.killpg(os.getpgid(server.pid), signal.SIGTERM)
for client in clients:
    try:
        os.killpg(os.getpgid(client.pid), signal.SIGTERM)
    except:
        # The client should have exited normally.
        pass
