## BBB Eth Listener
This runs on the BBB, and connects the network table to the ethernet port.
The ethernet port is connected to the NUC, which is running the controller/simulation
code. This is used to transfer data to and from the controller/simulator.

# Setting up 
Of course, make sure there is a connection between the BBB and the Intel NUC.
This is typically done via ethernet cable, although technically it is not
required (the devices could communicate over a home wifi network as well).
Before running, the ethernet port must appear as an active network interface.
You can view active interfaces on the BBB with  
```ifconfig```

# Running
Make sure to run the bbb_eth_listener first, before running the nuc_eth_listener
For both programs, you must pass it the IP address of the BBB's ethernet port.

For example, if the ip address of the ethernet port on the BBB is 192.168.1.25,
and you decide to use port 5555 then on the BBB run:  
```./bbb_eth_listener 192.168.1.25 5555```  

After that, on the intel NUC, run:
```./nuc_eth_listener 192.168.1.25 5555```
