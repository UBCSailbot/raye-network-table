#!/usr/bin/env python3
"""
---- CAN-BBB-NUC datapath test ----

This script serves as a test that ensures correct propagation of
data between the CAN bus (runs on virtual CAN), the Beaglebone (BBB)
and the Intel NUC

You'll want to make a config.ini file in your home directory with all of
the credentials for the SSH logins

The following jobs are required to run this datapath test:
On the NUC:
~$ roscore &
~$ nuc_eth_listener 192.168.1.60 5555 &

On the BBB:
~$ network_table_server &
~$ bbb_canbus_listener vcan0 &
~$ bbb_eth_listener 192.168.1.60 5555 &

"""

# for now we will not use pytest until i can figure it out
# import pytest
import time
import argparse
import threading
from nt_connection.CAN_to_URI_to_ROStopic import \
    CAN_to_URI_to_ROStopic, CAN_bus_sensor
from nt_connection.SSH_Connection import SSH_Connection
from dummy_data import Test_Data, make_dummy_tests


# These are needed in order to not read an empty URI
CAN_producer = threading.Semaphore(0)
CAN_consumer = threading.Semaphore(1)

SETUP_TIME = 2
CAN_SEND_INTERVAL = 5

# These commands are used in the test_nuc_eth_listener thread exclusively
setup_bash_cmd = "source /opt/ros/melodic/setup.bash"
setup_sh_cmd = "source /home/raye/catkin_ws/devel/setup.sh"
get_sensor_cmd = "/opt/ros/melodic/bin/rostopic echo /sensors/{} -n 1"

# This command is to be used in the test_can_listener thread exclusively
command = "python3 workspace/datapath_tests_NAV-194/test/datapath_tests/connection_values.py -u \'{}\'"


def test_nuc_eth_listener(nuc, test_data_dict):
    '''
    NUC ethernet listener test thread (Runs on NUC)
    Runs a rostopic echo that tracks down the URI corresponding to
    the test input sent on the CAN Bus

    NOTE: On NUC, it needs to have roscore and the nuc_eth_listener
    job up and running (BEWARE OF JENKINS when testing locally)
    '''

    print("NUC ETH Listener test")
    for test in test_data_dict:
        published_sensor_cmd = setup_bash_cmd + ";" + setup_sh_cmd + ";" + \
            get_sensor_cmd.format(test_data_dict[test]['rostopic'])
        stdin, stdout, stderr = nuc.exec_command(published_sensor_cmd)
        NUC_data = stdout.readline()
        CAN_data = test_data_dict[test]['parsed_data']
        print("NUC LISTENER TEST - NUC ROS data:", NUC_data)
        print("NUC LISTENER TEST - CAN data:", CAN_data)
        try:
            assert(NUC_data == CAN_data)
            print("NUC LISTENER TEST - DATA PASSED")
        except AssertionError:
            print("NUC LISTENER TEST - NUC ETH Listener data not received correctly")
            print("NUC data:", NUC_data)
            print("CAN data:", CAN_data)
        print("=====")
    return


# TODO: actually use this to check if TCP data is transmitted correctly
# through ethernet
def test_bbb_eth_listener(bbb, data):
    assert True
    return


def test_bbb_can(bbb, test_data_dict):
    '''
    Canbus test thread (Runs on BBB)

    Runs CAN dump to see if data is transmitted correctly
    into the canbus listener
    NOTE: stdout.readline() will hang but we need
    it to until it receives data from main
    '''

    print("BBB Virtual CAN Test")
    for test in test_data_dict:
        CAN_consumer.acquire()
        stdin, stdout, stderr = bbb.exec_command('candump vcan0')
        can_msg = stdout.readline()
        # we extract the last 8 characters this way because
        # the format of candump is weird
        can_msg = "".join(can_msg.strip().split()[-8:])
        CAN_producer.release()
        print("CAN TEST - CAN Test message", can_msg)
        print("CAN TEST - Sent CAN message", test_data_dict[test]['data'])
        try:
            assert(can_msg == test_data_dict[test]['data'])
            print("CAN TEST - DATA PASSED")
        except AssertionError:
            print("CAN TEST - CAN Data not received correctly")
        print("*****")
    return


def test_bbb_can_listener(bbb, test_data_dict):
    '''
    Canbus listener test thread (Runs on BBB)

    Runs a command script which retrieves the values of the URI
    from the network-table to verify that canbus listener is
    publishing properly

    To test locally, it needs the network_table_server and
    bbb_canbus_listener on vcan0 running on the BBB.

    Currently test just the wind sensor transmission of data
    NOTE: BEWARE OF JENKINS WHEN TESTING LOCALLY since it might
    kill the jobs on the BBB required to run this
    '''

    print("BBB CAN Listener Test")
    for test in test_data_dict:
        test_command = command.format(test_data_dict[test]['uri'])
        CAN_producer.acquire()
        stdin, stdout, stderr = bbb.exec_command(test_command)
        NT_data = stdout.readline()
        CAN_consumer.release()
        NT_data = int(NT_data)
        CAN_data = test_data_dict[test]['parsed_data']
        print("CAN LISTENER TEST - CAN NT value:", NT_data)
        print("CAN LISTENER TEST - CAN Transmitted value:",
              test_data_dict[test]['parsed_data'])
        try:
            assert(NT_data == CAN_data)
            print("CAN LISTENER TEST - DATA PASSED")
        except AssertionError:
            print("CAN LISTENER TEST - BBB CAN Bus Listener data not received correctly through virtual CAN")
        print("-----")
    return


def main():
    parser = argparse.ArgumentParser(description="CAN<>BBB<>NUC datapath test")
    parser.add_argument("-r", "--remote",
                        help="If you are running this \
                              on your local machine then set this flag",
                        action='store_true')
    parser.add_argument("-c", "--config",
                        help="Path to the config file which has \
                              all the credentials for the server, BBB, NUC",
                        default='/home/bruno/config.ini')
    args = parser.parse_args()

    bbb_client = None
    nuc_client = None

    # Start SSH Connections to the nodes
    ssh_connection = SSH_Connection(args.config)

    if args.remote:
        ssh_connection.SSH_server()
        ssh_connection.SSH_NUC_tunnel()
        ssh_connection.SSH_BBB_tunnel()

        bbb_client = ssh_connection.bbb_connection
        nuc_client = ssh_connection.nuc_connection
    else:
        ssh_connection.SSH_BBB()
        ssh_connection.SSH_NUC()

        bbb_client = ssh_connection.bbb_connection
        nuc_client = ssh_connection.nuc_connection

    # This makes dummy tests to use for this datapath test
    # more info found in dummy_data.py
    my_data = make_dummy_tests(CAN_bus_sensor,
                               Test_Data,
                               CAN_to_URI_to_ROStopic)

    # Start all threads
    bbb_can_test_thread = threading.Thread(name='test_bbb_can',
                                           target=test_bbb_can,
                                           args=(bbb_client, my_data))
    bbb_canbus_listener_test_thread = \
        threading.Thread(name='test_bbb_can_listener',
                         target=test_bbb_can_listener,
                         args=(bbb_client, my_data))

    # TODO: implement the bbb_eth_listener thread to make it read the TCP
    # packets through the ethernet
    # bbb_eth_listener_test_thread = \
    #                       threading.Thread(target=bbb_eth_listener_test,
    #                                        args=(bbb_client, Test_Data))

    nuc_eth_listener_test_thread = \
        threading.Thread(name='test_nuc_eth_listener',
                         target=test_nuc_eth_listener,
                         args=(nuc_client, my_data))

    print("Beginning CAN-BBB-NUC DP test")

    bbb_can_test_thread.start()
    bbb_canbus_listener_test_thread.start()
    nuc_eth_listener_test_thread.start()

    # THIS SLEEP IS NEEDED SO THAT ALL THE THREADS CAN DO PROPER SETUP
    time.sleep(SETUP_TIME)

    # Virtual CAN bus emulation loop
    for test in my_data:
        time.sleep(CAN_SEND_INTERVAL)
        print("MAIN - Sending command:",
              "cansend vcan0 " +
              '{:03x}'.format(my_data[test]['id'], "x")
              + "#" + my_data[test]['data'])
        bbb_client.exec_command("cansend vcan0 " +
                                '{:03x}'.format(my_data[test]['id']) +
                                "#" + my_data[test]['data'])

    bbb_can_test_thread.join()
    bbb_canbus_listener_test_thread.join()
    nuc_eth_listener_test_thread.join()

    ssh_connection.close_SSH()


if __name__ == "__main__":
    main()
