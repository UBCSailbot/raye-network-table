#!/bin/bash

# Steps for NUC
# 1) Start roscore
# 2) Run nuc-eth-listener
# 3) Run local pathfinding
# 4) Run mocks scripts (MOCK_AIS ; MOCK_sensors)
# No need to start visualizer automatically

NT_ROOT="/home/raye/network-table"

# nuc-eth-listener arguments
NE_IP="192.168.1.60"
NE_PORT="5555"

NE_CMD="./build/bin/nuc_eth_listener $NE_IP $NE_PORT &"

HELPER_PATH="/home/raye/network-table/scripts/startup/helpers"
LP_PATH="$HELPER_PATH/local_pathfinding_startup.sh"
MOCK_AIS_PATH="$HELPER_PATH/mock_ais_startup.sh"
MOCK_SENSORS_PATH="$HELPER_PATH/mock_sensors_startup.sh"

# Assuming melodica is sourced properly

printf "\nStartup script for NUC\n"
printf "\n================================\n"

# Roscore
#printf "Roscore\n"
#printf ".....................\n"

#if pgrep -f "roscore" > /dev/null;
#then
    #ROS_PID=$(pgrep -f "roscore")
    #printf "Roscore already running with PID: %d\n"$ROS_PID
#else
    #printf "Roscore not started. Starting roscore\n"

    #roscore &
    #ROS_PID=$(pgrep -f "roscore")

    #printf "Started roscore with PID: %d\n" $ROS_PID
#fi
#printf "\n================================\n"

# nuc-eth-listener
printf "NUC ETH Listener\n"
printf ".....................\n"

if pgrep -f "nuc_eth_listener" > /dev/null;
then
    NE_PID=$(pgrep -f "nuc_eth_listener")
    printf "NUC ETH listener already running with PID: %d\n" $NE_PID
else
    printf "NUC ETH listener not started. Starting new process\n"

    cd $NT_ROOT
    eval "$NE_CMD"
    NE_PID=$(pgrep -f "nuc_eth_listener")

    printf "NUC ETH listener started with PID: %d\n" $NE_PID
fi
printf "\n================================\n"

# local pathfinding
printf "Local Pathfinding\n"
printf ".....................\n"

if pgrep -f "local_pathfinding" > /dev/null;
then
  LP_PID=$(pgrep -f "local_pathfinding")
  printf "Local pathfinding already running with PID: %d\n" $LP_PID
else
  printf "Local pathfinding not started. Starting new process\n"

  screen -dmS local_pathfinding $LP_PATH
  LP_PID=$(pgrep -f "local_pathfinding")

  printf "Local pathfinding started with PID: %d\n" $LP_PID
fi
printf "\n================================\n"

# Mock scripts
printf "Mock Scripts (AIS ; Sensors)\n"
printf ".....................\n"
# Ask user if both mock scripts should be run
read -p "Do you want to run MOCK_AIS.py? [y/n]" -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then 
    if pgrep -f "MOCK_AIS.py" > /dev/null;
    then
        MA_PID=$(pgrep -f "MOCK_AIS.py")
        printf "MOCK_AIS.py already running with PID: %d\n" $MA_PID
    else
        printf "MOCK_AIS.py not runnin. Starting process\n"

        screen -dmS mock_ais $MOCK_AIS_PATH
        MA_PID=$(pgrep -f "MOCK_AIS.py")

        printf "MOCK_AIS.py started with PID: %d\n" $MA_PID
    fi
else
    printf "Not running MOCK_AIS.py\n"
fi

read -p "Do you want to run MOCK_sensors.py? [y/n]" -r 
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then
    if pgrep -f "MOCK_sensors.py" > /dev/null;
    then
        MS_PID=$(pgrep -f "MOCK_sensors.py")
        printf "MOCK_sensors.py already running with PID: %d\n" $MS_PID
    else
        printf "MOCK_sensors.py not running. Starting process\n"

        screen -dmS mock_sensors $MOCK_SENSORS_PATH
        MS_PID=$(pgrep -f "MOCK_sensors.py")

        printf "MOCK_sensors.py started with PID: %d\n" $MS_PID
    fi
else
    printf "Not running MOCK_sensors.py\n"
fi

printf "\n================================\n"

printf "NUC Startup script complete\n"
