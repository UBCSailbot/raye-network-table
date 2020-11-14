## Land Satellite Listener
A simple python HTTP server that will run on the land 
server hosted on Microsoft Azure. 

Handles HTTP POST requests containing google protobuf
serialized sensor or uccm data sent from the Iridium 
satellite network. Decodes and prints the received 
data to the console. In the future, this script will 
be modified to update the land-side database. 

This script also sends global pathfinding waypoints
to the Iridium satellite network via HTTP POST request. 

Install protobuf for python3:  
```pip3 install protobuf```

To run the script:  
```python3 land_satellite_listener.py <SERVER_PORT_NUMBER> <HTTP_POST_ENDPOINT> <POLL_FREQUENCY> <UNIT> <BINDING_ADDRESS>```

For example:
```
python3 land_satellite_listener.py -p 8000 -e https://rockblock.rock7.com/rockblock/MT -f 30 -u SEC -b 70.36.55.243 
```

## Full-Scale Testing 
Refer to bbb_rockblock_listener README for full-scale testing with the virtual rockblock.

## Small-Scale Testing
`sensorBinaryData.txt` contains serialized dummy sensor
data from the network table that can be sent as test 
data to the http server via post request. 

`uccmBinaryData.txt` contains serialized dummy uccm 
data from the network table that can be sent as test
data to the http server via post request.

### Testing using localhost
Run the script to start the HTTP server:  
```python3 land_satellite_listener.py 8080 ```

Send sensor data:  
```curl -X POST --data-binary @sensorBinaryData.txt http://localhost:8080```

Send uccm data:  
```curl -X POST --data-binary @uccmBinaryData.txt http://localhost:8080```

Should see sensor/uccm data printed in the console of the running server

### Testing on Microsoft Azure VM 
(yet to test with actual server setup)
*Assuming security group contains inbound rule for server port (ie. 8080)*

ssh into the VM

Open the firewall port:
```bash
sudo ufw allow 8080
sudo ufw status
```

Run the script to start the HTTP server:  
```python3 land_satellite_listener.py 8080 ```

Send sensor data:  
```curl -X POST --data-binary @sensorBinaryData.txt http://<VM_PUBLIC_IP>:8080```

Send uccm data:  
```curl -X POST --data-binary @uccmBinaryData.txt http://<VM_PUBLIC_IP>:8080```

Should see sensor/uccm data printed in the console of the running server
