## NUC Eth Listener
This runs on the NUC, and connects the simulator/controller to the ethernet port.
The ethernet port is connected to the BBB, which is then connected to the sensors/ais, etc.

To build this, make sure the cmake variable ENABLE_ROS
is set to ON, in the top level CMakeLists.txt

# Running
Pass the IP address of the beaglebone to this program.
See [bbb_eth_listener](../bbb_eth_listener/README.md) for more info.
