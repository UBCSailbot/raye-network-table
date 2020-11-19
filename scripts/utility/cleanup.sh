# bbb
pkill -f network_table_server
pkill -f bbb_eth_listener
pkill -f bbb_canbus_listener
pkill -f bbb_satellite_listener
pkill -f Iridium9602
pkill -f mock_sensors
pkill -f socat
rm -r /tmp/can_bbb_nuc_integration/

# nuc
pkill -f roscore
pkill -f nuc_eth_listener
pkill -f raye_communication_node
pkill -f main_loop.py
pkill -f network_table_server
pkill -f land_satellite_listener
