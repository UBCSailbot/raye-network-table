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

## Sensors Canbus
Reads data about various sensors on the canbus network
and places it into the network table.

## NUC to BBB
Sends simulator outputs to the BBB.
To build this, make sure the cmake variable ENABLE_ROS
is set to ON, in the top level CMakeLists.txt

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

## Rockblock
WIP. Sends/receives data over satellite via a serial connection to Rockblock+.
