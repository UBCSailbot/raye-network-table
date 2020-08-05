# BBB Canbus Listener
Runs on the BBB and connects the CANbus network
to the network table. Reads sensor data from the CANbus,
and sends outputs from the controller (actuation angles)
onto the CANbus network.

## Setting up CANbus
After you have physically set up the BBB (attached the canbus shield and connected
it to the CANbus network), run the script (located in the top level scripts folder)  
```config_canbus_pins.sh```  

After running this, you should see a network interface called "can0"

## Running
This program requires the name of the network to interface to connect to.
When using the real CANbus, it should be called "can0". However, during
testing, a virtual canbus is also used called "vcan0".
Here is an example of how to run the program when using the real CANbus:  
```./bbb_canbus_listener can0```
