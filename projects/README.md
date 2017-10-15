# Projects
> Build targets and target-specific code.

## Production targets
These targets are for production code.

## Server

A communication hub that runs on central controller on Ada 2.0.  
Receives updates on sensor data (from GPS, wind sensors, etc), allows other modules to connect
to the network table using pub/sub or request/reply.

See [here](server/README.md) for more info.

## Client

An example client of the Network Table.
This is also used to test the functionality of
the NetworkTable.

See [here](client/README.md) for more info.
