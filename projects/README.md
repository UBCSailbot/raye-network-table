# Projects
> Build targets and target-specific code.

## Server
Runs the network table server

## Client
An example client of the Network Table.
This is also used to test the functionality of
the NetworkTable.

## Viewtree
Prints out all the values in the network table

## Light Client
Simply updates a single value in the network table forever

## Read Wind Sensor Canbus
When the beaglebone is connected to the canbus, this uses 
[socketCAN](https://en.wikipedia.org/wiki/SocketCAN) to read
the wind sensor data off the can network, and put it in the network
table.

## Dummy Satellite
Synchronize two network tables
on either end of an ethernet cable.

One side must act as a server. When running on this side,
find the ip address of your ethernet port (ie with 'ifconfig'),
and run   
'''./dummy-satellite server <ip_address>'''

On the other side, you must connect to the _same_ ip address,
but as a client. Use the following command:  
'''./dummy-satellite client <ip_address>'''

Note that the code is 99% the same on the client and server side.
It's just the zmq has to call bind on the server side,
and connect on the client side in order to work.
