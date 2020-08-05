## BBB Rockblock Listener
This sends sensor data over the satellite via a serial connection
to a [Rockblock+](https://www.rock7.com/products/rockblock-iridium-9602-satellite-modem)

For more info on using the rockblock, see [here](https://docs.rockblock.rock7.com/docs)
Information about our iridium satellite usage, credits, etc [here](https://rockblock.rock7.com/Operations)

This program will periodically send sensor data as well as uccm diagnostic data.
These two data types can be sent at different rates (uccm data probably is not as important,
and can be sent at a relatively lower frequency).

## Running
The program arguments are how often to send sensor data (in seconds),
how often to send uccm data (in seconds), and the path to the serial port
connected to the Rockblock.  
Here is an example where sensors are sent every 30 seconds, and uccm every
360 seconds, and serial port is at /dev/ttyUSB0
```./bbb_rockblock_listener 60 360 /dev/ttyUSB0```
