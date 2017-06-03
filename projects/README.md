# Projects
> Build targets and target-specific code.

## Production targets
These targets are for production code.

## Network Table

A communication hub that runs on central controller on Ada 2.0.  
Receives updates on sensor data (from GPS, wind sensors, etc), allows other modules to connect
to the network table using pub/sub or request/reply via the rabbitMQ message broker.

See [here](network-table/README.md) for more infot.
