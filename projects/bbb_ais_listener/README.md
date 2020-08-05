## BBB AIS Listener
This program will listen for AIS data, and store that data into the network table.
It does this by connecting to the [aisd program](https://github.com/UBCSailbot/ais/) via a ZeroMQ socket.
BBB AIS Listener will periodically ask for a list of current boats in the area, and will overwrite
the network table with the latest list of boats. This period is set to a default of 10 seconds,
but can also be passed in via command line arguments.

## Running
**important**
Make [aisd](https://github.com/UBCSailbot/ais/) is running, and the aisd antenna is connected.

Here is an example of running the program, and overwriting the default period to 5 seconds.  
```./bbb_ais_listener 5```
