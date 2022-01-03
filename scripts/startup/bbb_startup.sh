#!/bin/bash

# Steps for BBB
# 1) Run network_table_server
# 2) Run bbb_canbus_listener
# 3) Run bbb_satellite_listener
# 4) Run bbb_ais_listener
# 5) Run light-client to populate with mock data

NT_ROOT="/home/debian/network-table"

# Command arguments
CAN_ARG1="can0"

SAT_ARG1="60"
SAT_ARG2="360"
SAT_ARG3="600"
SAT_TTY="/dev/ttyS2"

ETH_IP="192.168.1.60"
ETH_PORT="5555"

NT_CMD="./build/bin/network_table_server &"
CAN_CMD="./build/bin/bbb_canbus_listener $CAN_ARG1 &"
SAT_CMD="./build/bin/bbb_satellite_listener $SAT_ARG1 $SAT_ARG2 $SAT_ARG3 $SAT_TTY &"
AIS_CMD="./build/bin/bbb_ais_listener &"
ETH_CMD="./build/bin/bbb_eth_listener $ETH_IP $ETH_PORT &"
LC_CMD="./build/bin/light_client &"

printf "\nStartup script for BBB\n"
printf "\n================================\n"

########################
# network_table_server
########################
printf "Network Table Server\n"
printf ".....................\n"

if pgrep -f "network_table_server" > /dev/null;
then
    NT_PID=$(pgrep -f "network_table_server")
    printf "Network-table server already running with PID: %d\n" $NT_PID
else
    printf "Server not started. Starting new process\n"

    cd $NT_ROOT
    eval "$NT_CMD" > /dev/null
    NT_PID=$(pgrep -f "network_table_server")
    
    printf "Started nt_server with PID: %d\n" $NT_PID
fi
printf "\n================================\n"

########################
# bbb_canbus_listener
########################
printf "BBB CANbus Listener\n"
printf "\n================================\n"

if pgrep -f "bbb_canbus_listener" > /dev/null;
then
    BC_PID=$(pgrep -f "bbb_canbus_listener")
    printf "BBB Canbus listener already running with PID: %d\n" $BC_PID
else
    printf "BBB Canbus not started. Starting new process\n"

    cd $NT_ROOT
    eval "$CAN_CMD"
    BC_PID=$(pgrep -f "bbb_canbus_listener")

    printf "Started bbb_canbus_listener with PID: %d\n" $BC_PID
fi

########################
# bbb_satellite_listener
########################
printf "BBB Satellite Listener\n"
printf ".....................\n"

if pgrep -f "bbb_satellite_listener" > /dev/null;
then
    BS_PID=$(pgrep -f "bbb_satellite_listener")
    printf "BBB Satellite Listener already running with PID: %d\n" $BS_PID
else
    printf "BBB Satellite Listener not started. Starting new process\n"

    cd $NT_ROOT
    eval "$SAT_CMD"
    BS_PID=$(pgrep -f "bbb_satellite_listener")
    
    printf "Started bbb_satellite_listener with PID: %d\n" $BS_PID
fi
printf "\n================================\n"

########################
# bbb_ais_listener
########################
printf "BBB AIS Listener\n"
printf ".....................\n"

if pgrep -f "bbb_ais_listener" > /dev/null;
then
    BA_PID=$(pgrep -f "bbb_ais_listener")
    printf "BBB AIS Listener already running with PID: %d\n" $BA_PID
else
    printf "BBB AIS Listener not started. Starting new process\n"

    cd $NT_ROOT
    eval "$AIS_CMD"
    BA_PID=$(pgrep -f "bbb_ais_listener")
    
    printf "Started bbb_ais_listener with PID: %d\n" $BA_PID
fi
printf "\n================================\n"

########################
# bbb_eth_listener
########################
printf "BBB ETH Listener\n"
printf ".....................\n"

if pgrep -f "bbb_eth_listener" > /dev/null;
then
    BE_PID=$(pgrep -f "bbb_eth_listener")
    printf "BBB ETH Listener already running with PID: %d\n" $BE_PID
else
    printf "BBB ETH Listener not started. Starting new process\n"

    cd $NT_ROOT
    eval "$ETH_CMD" > /dev/null
    BE_PID=$(pgrep -f "bbb_eth_listener")
    
    printf "Started bbb_eth_listener with PID: %d\n" $BE_PID
fi
printf "\n================================\n"

########################
# light_client
########################
printf "Light client\n"
printf ".....................\n"

# Ask user if light client should be run
read -p "Do you want to run light client? [y/n]" -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]
then 
    if pgrep -f "light_client" > /dev/null;
    then
        LC_PID=$(pgrep -f "light_client")
        printf "Light client already running with PID: %d\n" $LC_PID
    else
        printf "Light client not started. Starting new process\n"
    
        cd $NT_ROOT
        eval "$LC_CMD" > /dev/null
        LC_PID=$(pgrep -f "light_client")
    
        printf "Started light_client with PID: %d\n" $LC_PID
    fi
else
    printf "Not running light client\n"
fi

printf "\n================================\n"
printf "Startup script for BBB complete\n"
printf "Network-table Server PID: %d\n" $NT_PID
printf "BBB Canbus Listener PID: %d\n" $BC_PID
printf "BBB Satellite Listener PID: %d\n" $BS_PID
printf "BBB AIS Listener PID: %d\n" $BA_PID
printf "BBB ETH Listener: %d\n" $BE_PID
printf "Light client: %d\n" $LC_PID
