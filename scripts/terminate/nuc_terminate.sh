#!/bin/bash

# Processes to terminate on NUC in no order:
# roscore
# nuc_eth_listener
# local pathfinding

printf "\nScript to terminate processes for NUC\n"
printf "\n================================\n"

# roscore
# Ask user if roscore should be shut down
#printf "Roscore\n"
#printf ".....................\n"

#read -p "Do you want to terminate roscore? " -r
#echo
#if [[ $REPLY =~ ^[Yy]$ ]]
#then
    #if pgrep -f "roscore" > /dev/null;
    #then
        #RC_PID=$(pgrep -f "roscore")
        #kill -9 $RC_PID
        #printf "Terminated roscore with PID: %d\n" $RC_PID
    #else
        #printf "PGREP couldn't find roscore running\n"
    #fi
#else
    #printf "NOT shutting down roscore\n"
#fi
#printf "\n================================\n"

# nuc_eth_listener
printf "NUC ETH Listener\n"
printf ".....................\n"

if pgrep -f "nuc_eth_listener" > /dev/null;
then
    NE_PID=$(pgrep -f "nuc_eth_listener")
    kill -9 $NE_PID
    printf "Terminated nuc_eth_listener with PID: %d\n" $NE_PID
else
    printf "PGREP couldn't find nuc_eth_listener running\n"
fi
printf "\n================================\n"

# local pathfinding
printf "Local pathfinding\n"
printf ".....................\n"

if pgrep -f "local_pathfinding" > /dev/null;
then
    LP_PID=$(pgrep -f "local_pathfinding")
    kill -9 $LP_PID
    printf "Terminated local_pathfinding with PID: %d\n" $LP_PID
else
    printf "PGREP couldn't find local_pathfinding running\n"
fi
if pgrep -f "local-pathfinding" > /dev/null;
then
    LP_PID=$(pgrep -f "local-pathfinding")
    kill -9 $LP_PID
    printf "Terminated local-pathfinding with PID: %d\n" $LP_PID
else
    printf "PGREP couldn't find local-pathfinding running\n"
fi

# mock_ais.py
printf "ROS Mock Scripts\n"
printf ".....................\n"

if pgrep -f "MOCK_AIS.py" > /dev/null;
then
    MA_PID=$(pgrep -f "MOCK_AIS.py")
    kill -9 $MA_PID
    printf "Terminated MOCK_AIS.py with PID: %d\n" $MA_PID
else
    printf "PGREP couldn't find MOCK_AIS.py running\n"
fi
# mock_sensors.py
if pgrep -f "MOCK_sensors.py" > /dev/null;
then
    MS_PID=$(pgrep -f "MOCK_sensors.py")
    kill -9 $MS_PID
    printf "Terminated MOCK_sensors.py with PID: %d\n" $MS_PID
else
    printf "PGREP couldn't find MOCK_sensors.py running\n"
fi
