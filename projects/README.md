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

## NUC Eth Listener
Sends simulator outputs to the BBB.
Receives sensor data from BBB and publishes.
It uses the ROS custom message types defined in the
sailbot-msg repo.
To build this, make sure the cmake variable ENABLE_ROS
is set to ON, in the top level CMakeLists.txt

## BBB Eth Listener
Receives changes to actuation angle from simulator running on NUC,
and places them in network table.
Sends changes to sensor data in the network table to the NUC.

## Rockblock
WIP. Sends/receives data over satellite via a serial connection to Rockblock+.
