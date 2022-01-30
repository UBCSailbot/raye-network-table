#!/bin/bash

# Processes to terminate for landserver in no order:
# land_satellite_listener
# pathfinder_cli
# network_table_server

printf "\nScript to terminate processes for land server\n"
printf "\n================================\n"


# land_satellite_listener
printf "Land Satellite Listener\n"
printf ".....................\n"

if pgrep -f "python3 land_satellite_listener.py" > /dev/null;
then
    LS_PID=$(pgrep -f "python3 land_satellite_listener.py")
    kill -9 $LS_PID
    printf "Terminated land_satellite_listener with PID: %d\n" $LS_PID
else
    printf "PGREP couldn't find running land_satellite_listener\n"
fi
printf "\n================================\n"

# pathfinder_cli
printf "Global Pathfinding\n"
printf ".....................\n"

if pgrep -f "./build/bin/pathfinder_cli" > /dev/null;
then
    PF_PID=$(pgrep -f "./build/bin/pathfinder_cli")
    kill -9 $PF_PID
    printf "Terminated global pathfinding with PID: %d\n" $PF_PID
else
    printf "PGREP couldn't find running Global pathfinding\n"
fi
printf "\n================================\n"

# network_table_server
printf "Network-table Server\n"
printf ".....................\n"

if pgrep -f "./build/bin/network_table_server" > /dev/null;
then
    NT_PID=$(pgrep -f "./build/bin/network_table_server")
    kill -9 $NT_PID
    printf "Terminated nt-server with PID: %d\n" $NT_PID
else
    printf "PGREP couldn't find running nt-server\n"
fi
printf "\n================================\n"

printf "Terminate script complete\n"