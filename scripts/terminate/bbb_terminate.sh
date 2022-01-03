#!/bin/bash

# Processes to terminate on BBB in no order:
# bbb_canbus_listener
# bbb_satellite_listener
# bbb_ais_listener
# bbb_eth_listener
# network_table_server last (just in case)

printf "\nScript to terminate processes for BBB\n"
printf "\n================================\n"


# bbb_canbus_listener
printf "BBB Canbus Listener\n"
printf ".....................\n"

if pgrep -f "bbb_canbus_listener" > /dev/null;
then
    BC_PID=$(pgrep -f "bbb_canbus_listener")
    kill -9 $BC_PID
    printf "Terminated bbb_canbus_listener with PID: %d\n" $BC_PID
else
    printf "PGREP couldn't find running bbb_canbus_listener\n"
fi
printf "\n================================\n"

# bbb_satellite_listener
printf "BBB Satellite Listener\n"
printf ".....................\n"

if pgrep -f "bbb_satellite_listener" > /dev/null;
then
    BS_PID=$(pgrep -f "bbb_satellite_listener")
    kill -9 $BS_PID
    printf "Terminated bbb_satellite_listener with PID: %d\n" $BS_PID
else
    printf "PGREP couldn't find running bbb_satellite_listener\n"
fi
printf "\n================================\n"

# bbb_ais_listener
printf "BBB AIS Listener\n"
printf ".....................\n"

if pgrep -f "bbb_ais_listener" > /dev/null;
then
    BA_PID=$(pgrep -f "bbb_ais_listener")
    kill -9 $BA_PID
    printf "Terminated bbb_ais_listener with PID: %d\n" $BA_PID
else
    printf "PGREP couldn't find running bbb_ais_listener\n"
fi
printf "\n================================\n"

# bbb_eth_listener
printf "BBB ETH Listener\n"
printf ".....................\n"

if pgrep -f "bbb_eth_listener" > /dev/null;
then
    BE_PID=$(pgrep -f "bbb_eth_listener")
    kill -9 $BE_PID
    printf "Terminated bbb_eth_listener with PID: %d\n" $BE_PID
else
    printf "PGREP couldn't find running bbb_eth_listener\n"
fi
printf "\n================================\n"

# network_table_server
printf "Network Table Server"
printf ".....................\n"

if pgrep -f "network_table_server" > /dev/null;
then
    NT_PID=$(pgrep -f "network_table_server")
    kill -9 $NT_PID
    printf "Terminated network_table_server with PID: %d\n" $NT_PID
else
    printf "PGREP couldn't find running network_table_server\n"
fi
printf "\n================================\n\n"

printf "Terminate script completed\n"
