#!/bin/bash

# Steps for landserver:
# 1) Run network_table_server (./build/bin/network_table_server) 
# 2) Run land-satellite-listener (command too long to paste)
# 3) Run global pathfinding
#    (./build/bin/pathfinder_cli -p 8 --navigate 48 235 21 203)

NT_ROOT="/root/network-table"
# pgrep search commands (not full commands since pgrep can use regex)
NT_PGREP="network_table_server"
LS_PGREP="python3 land_satellite_listener.py"
GP_PGREP="pathfinder_cli"

#Arguments for land-satellite-listener
LS_PORT="-p 8000"
LS_URL="-e https://rockblock.rock7.com/rockblock/MT"
LS_FREQ_UNIT="-f 10 -u SEC"
LS_B="-b 70.36.55.243"
LS_USER="-n captain@ubcsailbot.org"
LS_PASS="-w raye2020"
LS_ID="-r 300234068129370"
LS_IPSRC="-i 212.71.235.32"

NT_CMD="./build/bin/network_table_server &"
LS_CMD="python3 land_satellite_listener.py $LS_PORT $LS_URL $LS_FREQ_UNIT $LS_UNIT $LS_B $LS_USER $LS_PASS $LS_ID $LS_IPSRC &"
GP_CMD="/root/network-table/scripts/startup/helpers/global_pathfinding_startup.sh"

printf "\nStartup script for land server\n"
printf "\n================================\n"

# network_table_server
printf "Network Table Server\n"
printf ".....................\n"

if pgrep -f "$NT_PGREP" > /dev/null;
then
    NT_PID=$(pgrep -f "$NT_PGREP")
    printf "Network-table server already running with PID: %d\n" $NT_PID
else
    printf "Server not started. Starting new process\n"

    cd $NT_ROOT
    eval "$NT_CMD"
    NT_PID=$(pgrep -f "$NT_PGREP")
    
    printf "Started nt_server with PID: %d\n" $NT_PID
fi
printf "\n================================\n"

# land_satellite_listener
printf "Land Satellite Listener\n"
printf ".....................\n"

if pgrep -f "land_satellite_listener" > /dev/null;
then
    LS_PID=$(pgrep -f "$LS_PGREP")
    printf "Satellite listener already running with PID: %d\n" $LS_PID
else
    printf "Listener not started. Starting new process\n"
    
    cd $NT_ROOT/projects/land_satellite_listener
    eval "$LS_CMD"
    LS_PID=$(pgrep -f "$LS_PGREP")

    printf "Started land satellite listener with PID: %d\n" $LS_PID
fi
printf "\n================================\n"

# global pathfinding
printf "Global Pathfinding\n"
printf ".....................\n"

if pgrep -f "$GP_PGREP" > /dev/null;
then
    GP_PID=$(pgrep -f "$GP_PGREP")
    printf "Global pathfinding already running with PID: %d\n" $GP_PID
else
    printf "Global pathfinding not started. Starting new process\n"

    cd $NT_ROOT
    screen -dmS global_pathfinding $GP_CMD
    GP_PID=$(pgrep -f "$GP_PGREP")

    printf "Started global pathfinding with PID: %d\n" $GP_PID
fi
printf "\n================================\n\n"

printf "Startup script complete\n"
printf "Network-table Server PID: %d\n" $NT_PID
printf "Land Satellite Listener PID: %d\n" $LS_PID
printf "Global Pathfinding PID: %d\n" $GP_PID
