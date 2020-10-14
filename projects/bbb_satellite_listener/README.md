## BBB Rockblock Listener
This sends sensor data over the satellite via a serial connection
to a [Rockblock+](https://www.rock7.com/products/rockblock-iridium-9602-satellite-modem)

For more info on using the rockblock, see [here](https://docs.rockblock.rock7.com/docs)
Information about our iridium satellite usage, credits, etc [here](https://rockblock.rock7.com/Operations)

This program will periodically send sensor data well as uccm diagnostic data.
These two data types can be sent at different rates (uccm data probably is not as important,
and can be sent at a relatively lower frequency).

This program will also periodically receive global pathfinding waypoints.
The frequency at which this data is received can be set as an argument. 

## Running
The program arguments are how often to send sensor data (in seconds),
how often to send uccm data (in seconds), how often to receive waypoints,
and the path to the serial port connected to the Rockblock.  
Here is an example where sensors are sent every 30 seconds, and uccm every
360 seconds, and waypoints are received every 600 seconds, 
and serial port is at /dev/ttyUSB0
```./bbb_rockblock_listener 60 360 600 /dev/ttyUSB0```

## Full Scale Testing with Viritual Iridium
This will test Sending data from BBB to Viritual Iridium to Land Server
as well as Sending data from Land Server to Virtual Iridium to BBB.

The bbb_rockblock_listener will send dummy sensor and uccm data to the virtual rockblock via serial port.
The virtual rockblock will relay the dummy sensor and uccm data to the land server via HTTP POST.

The land_satellite_listener will send dummy waypoint data to the virtual rockblock via HTTP POSt.
The virtual rockblock will relay the dummy waypoint data to bbb_rockblock_lisener via serial port.

### Start the virtual iridium
1. In a terminal, create a virtual serial port pair:
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

For example, the following serial devices will be created:
```
/dev/pts/3
/dev/pts/4
```

2. In a new terminal start the virtual iridium using HTTP_POST mode:
```
python2 Iridium9602.py <LAND_SERVER_ENDPOINT> <SERVER_PORT_NUMBER> -d <SERIAL_DEVICE> -m HTTP_POST
```

<LAND_SERVER_ENDPOINT> is the ip/location of the land_satellite_listner server (ie. http://localhost:8000)

<SERVER_PORT_NUMBER> is the port the iridium http handler will be running on (ie. 8080)

<SERIAL_DEVICE> is one of the two socat pairs created in the previous step (ie. /dev/pts/3)

### Start the land_satellite_listener
1. In a terminal start the land_satellite_listener:
```
land_satellite_listener.py <SERVER_PORT_NUMBER> <VIRTUAL_IRIDIUM_ENDPOINT>
```

<SERVER_PORT_NUMBER> is the port the land http handler server will be running on (ie 8000)

<VIRTUAL_IRIDIUM_ENDPOINT> is the ip/location of the server running in the virtual iridium (ie. http://localhost:8080)

Note: This will cause dummy waypoint data to be sent and queued up in the virtual iridium.


### Start the bbb_rockblock_listener
1. Start the network table server
```
./server
```

2. Run the client
```
./client
```

3. Run the light client
```
./light-client
```

4. In a terminal start bbb_rockblock_listner:
```
./bbb_rockblock_listener <SENSOR_FREQ> <UCCM_FREQ> <REC_FREQ> <SERIAL_PORT>
```

<SENSOR_FREQ> is the frequency at which sensor data is sent

<UCCM_FREQ> is the frequency at which uccm data is sent

<REC_FREQ> is the frequency at which waypoint data is received

<SERIAL_PORT> is the second socat pair device from setp 1 of starting the virtual iridium 

For example:
```
./bbb_rockblock_listener 5 10 15 /dev/pts/4
```

This will send sensor data every 5 seconds, send uccm data every 10 seconds and receive waypoints every 10 seconds

## Common Test Errors
### Segfaults After Restarting bbb_rockblock_listener
This might occur if you terminate the bbb_rockblock_listener code before all the contents of the serial port are read/removed.
When you rerun bbb_rockblock_listener, it will be reading old serial messages/commands.

To fix this:

1. Terminate the virtual iridium and land_satellite_listener 

In there respective terminals:
```
Ctr-C
```

2. Restart your socat pair
```
socat -d -d pty,raw,echo=0 pyt,raw,echo=0
```

3. Rerun virtual iridium and land_satellite_listener

4. Rerun bbb_rockblock_listener

### Terminating the Virtual Iridium:
In cases where the order of running/terminating these programs is incorrect, you may need to run the following command in a separate terminal:
```
pkill python
```

