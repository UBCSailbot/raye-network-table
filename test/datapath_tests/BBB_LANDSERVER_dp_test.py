"""
--- LANDSERVER-ROCKBLOCK-BBB datapath test---

Tests if the waypoint data is sent correctly between the land_satellite_listener and the bbb_satellite_listener.

-----------------------------------------------------
Make sure you run the following jobs before running this script:

On the BBB:
~$ ./network_table_server &
~$ ./bbb_satellite_listener [val] [val] [val] /dev/ttyS2

On the SERVER:
~$ ./network_table_server &
~$ python3 land_satellite_listener -p [PORT] -e [ENDPOINT] -f [FREQUENCY] -u [TIME] -n [LOGIN USERNAME] -w [PASSWORD]

-------------------------------------------------------
**NOTE**:

For this script to work:

1.) Run this script remotely on your local machine
2.) Input new latitude and longitude data each time you run this script.

e.g. python3 BBB_LANDSERVER_dp_test -c [CONFIG FILE] -l [SAMPLE LATITUDE DATA] -g [SAMPLE LONGITUDE DATA]

"""

from nt_connection.Connection import Connection
from nt_connection.Connection import ConnectionError
import generated_python.Value_pb2 as Value_pb2
import generated_python.Satellite_pb2 as Satellite_pb2
import generated_python.Node_pb2 as Node_pb2
from nt_connection.SSH_Connection import SSH_Connection
from sys import argv
import time
import argparse


# This command is to write sample waypoint data to the network-table on the bbb or land-server
write_waypoint_nt_cmd = "python3 network-table/test/datapath_tests/mock_waypoint_data.py {} {}"

# This command is to poll for the network table waypoint table to verify it has been written correctly
poll_nt_cmd = "python3 network-table/test/datapath_tests/poll_network_table_data.py -u {}"


def write_waypoint_to_nt(land_ssh, longitude: float, latitude: float):
    """
    Writes mock waypoint data to the network-table on the ssh server.
    """

    test_command = write_waypoint_nt_cmd.format(longitude, latitude)
    print("Writing waypoint data to land server... TEST COMMAND: ", test_command)
    stdin, stdout, stderr = land_ssh.exec_command(test_command)
    while int(stdout.channel.recv_exit_status()) != 0:
        time.sleep(1)


def test_bbb_satellite_listener(bbb_ssh, longitude: float, latitude: float):
    """
    Verfies the waypoint data sent from the landserver has been correctly transferred to the bbb.
    """

    test_command = poll_nt_cmd.format("waypoints")
    print("Checking waypoint data on bbb... TEST COMMAND: ", test_command)
    stdin, stdout, stderr = bbb_ssh.exec_command(test_command)
    # reformat string to be a list again
    NT_data = stdout.readline()[1:-2].split(",")  # white space returned at end so need -2
    try:
        print("\nRecieved: ", NT_data)
        assert(latitude == float(NT_data[0]))
        assert(longitude == float(NT_data[1]))
        print("\n**PASSED** - BBB WAYPOINT LISTENER TEST\n")
    except AssertionError:
        print("\nFAILED - BBB WAYPOINT LISTENER TEST\n")
    return


def main():
    parser = argparse.ArgumentParser(description="SERVER<>BBB datapath test")
    parser.add_argument("-c", "--config",
                        help="Path to the config file which has \
                              all the credentials for the server, BBB, NUC",
                        default=None)
    parser.add_argument("-l", "--latitude",
                        help="Sample latitude data you want to test for",
                        default=None)
    parser.add_argument("-g", "--longitude",
                        help="Sample longitude data you want to test for",
                        default=None)
    args = parser.parse_args()

    if args.config is None or args.latitude is None or args.longitude is None:
        print("Missing parameters...")
        print("Example usage: python3 BBB_LANDSERVER_dp_test.py -c config.ini -l 23.54 -g 52.12")
        return

    bbb_client = None
    land_server_client = None

    config_file = args.config

    # Start SSH Connections to the nodes
    ssh_connection = SSH_Connection(config_file)

    ssh_connection.SSH_server()
    ssh_connection.SSH_BBB_tunnel()

    bbb_client = ssh_connection.bbb_connection
    server_client = ssh_connection.server_connection

    # Sample longitude and latitude
    latitude, longitude = float(args.latitude), float(args.longitude)

    print("\n---Beginning LANDSERVER-to-BBB DP test---\n")
    write_waypoint_to_nt(server_client, longitude, latitude)
    test_bbb_satellite_listener(bbb_client, longitude, latitude)
    print("\n---Finished LANDSERVER-to-BBB DP test---\n")
    ssh_connection.close_SSH()


if __name__ == "__main__":
    main()
