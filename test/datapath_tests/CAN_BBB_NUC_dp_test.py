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
from nt_connection import uri
from nt_connection.frame_parser import *
import re


# These are needed in order to not read an empty URI from the NT
CAN_producer = threading.Semaphore(0)
CAN_consumer = threading.Semaphore(1)

# Due to the latency of reading from URIs, the NUC
# listener must wait for the CAN listener to be ready
CAN_Listener_ready = threading.Semaphore(0)

# Loopback semaphores to not read empty URIs from the NT
NUC_producer = threading.Semaphore(0)
NUC_consumer = threading.Semaphore(1)

CAN_listen_sem = threading.Semaphore(1)

# Time intervals to send CAN data
SETUP_TIME = 2
CAN_SEND_INTERVAL = 15

# actuation angle set 1,2 for testing (hard coded for now)
aa_set1 = 0.78
aa_set2 = 0.78

# These commands are used in the test_nuc_eth_listener thread exclusively
setup_bash_cmd = "source /opt/ros/melodic/setup.bash"
setup_sh_cmd = "source /home/raye/catkin_ws/devel/setup.sh"
get_sensor_cmd = "/opt/ros/melodic/bin/rostopic echo /sensors/{}"
stop_iter_1 = " -n 1"

# hard coded command to send actuation angles from the nuc
send_angle_cmd = \
    "/opt/ros/melodic/bin/rostopic pub /actuation_angle sailbot_msg/actuation_angle {} {} -1".format(aa_set1, aa_set2)

# This command is to be used in the test_bbb_can_listener thread exclusively
read_nt_cmd = "python3 workspace/datapath_tests/test/datapath_tests/connection_values.py -u {}"

# This command is to be used in the test_bbb_can thread exclusively
candump_cmd = 'candump vcan0'


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
        sensor_list = []
        published_sensor_cmd = setup_bash_cmd + ";" + setup_sh_cmd + ";"
        for topic in test_data_dict[test]['rostopic']:
            sensor_list.append(get_sensor_cmd.format(topic) + stop_iter_1)

        new_sensor_cmd = " & ".join(sensor_list)
        published_sensor_cmd += new_sensor_cmd
        print("NUC Listener - Published sensor command:", published_sensor_cmd)
        stdin, stdout, stderr = nuc.exec_command(published_sensor_cmd)
        NUC_data = stdout.read().decode('utf-8')
        NUC_data = list(filter(None, NUC_data.split('\n---\n')))
        CAN_data = list(map(str, test_data_dict[test]['parsed_data']))
        print("NUC LISTENER TEST - NUC ROS Sensor Data:", NUC_data)
        print("NUC LISTENER TEST - BBB CAN Sensor Data:", CAN_data)
        try:
            NUC_data = sorted(NUC_data)
            CAN_data = sorted(CAN_data)
            for i in range(len(NUC_data)):
                NUC_i = float(re.sub(r'[^-+\d.]', '', NUC_data[i]))
                CAN_i = float(re.sub(r'[^-+\d.]', '', CAN_data[i]))
                assert math.isclose(NUC_i, CAN_i, rel_tol=0.9)
            print("PASS - NUC LISTENER TEST - SENSOR DATA PASSED")
        except AssertionError:
            print("ERROR - NUC LISTENER TEST - NUC SENSOR DATA FAILED")

        # Waits for the CAN Listener thread to become ready after reading from the NT
        CAN_Listener_ready.acquire()
        print("---------\nNUC<>BBB<>CAN Loopback starting\n-----------")
        NUC_consumer.acquire()
        send_angle_sensor_cmd = setup_bash_cmd + ";" + setup_sh_cmd + ";" + send_angle_cmd
        print("NUC LISTENER TEST - NUC SEND ANGLE COMMAND:", send_angle_sensor_cmd)
        print("NUC LISTENER TEST - SENDING MOTOR ANGLE FROM NUC")
        stdin, stdout, stderr = nuc.exec_command(send_angle_sensor_cmd)
        NUC_producer.release()
        print("=====")

    return


# TODO: actually use this to check if TCP data is transmitted correctly
# through ethernet
def test_bbb_eth_listener(bbb, data):
    assert True
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
        # since the test data dictionary is a list, we need to convert to string
        SS_uri = ' '.join(test_data_dict[test]['uri'])
        test_command = read_nt_cmd.format(SS_uri)
        print("CAN LISTENER TEST - TEST COMMAND", test_command)
        CAN_producer.acquire()
        stdin, stdout, stderr = bbb.exec_command(test_command)
        # reformat string to be a list again
        NT_data = stdout.readline()[1:-2].split(", ")  # white space returned at end so need -2
        CAN_consumer.release()
        # converts the CAN data parsed data int list to a str list
        # this is needed to make the right string comparisons
        CAN_data = list(map(str, test_data_dict[test]['parsed_data']))
        print("CAN LISTENER TEST - BBB NT data:", NT_data)
        print("CAN LISTENER TEST - BBB CAN data:", CAN_data)
        try:
            # GPS DATE FRAME IS STORED AS A STRING
            NT_data = sorted(NT_data)
            CAN_data = sorted(CAN_data)
            for i in range(len(NT_data)):
                NT_i = float(re.sub(r'[^-+\d.]', '', NT_data[i]))
                CAN_i = float(re.sub(r'[^-+\d.]', '', CAN_data[i]))
                assert math.isclose(NT_i, CAN_i, rel_tol=1)
            print("CAN LISTENER TEST - DATA PASSED")
        except AssertionError:
            print("CAN LISTENER TEST - BBB CAN Bus Listener data not received correctly through virtual CAN")

        print("-----")
        # CAN Listener will have read the URIs in the first data propagation at this point
        CAN_Listener_ready.release()
        test_command = read_nt_cmd.format(uri.RUDDER_PORT_ANGLE)
        print("CAN LISTENER TEST - MOTOR CHECK COMMAND:", test_command)
        NUC_producer.acquire()
        stdin, stdout, stderr = bbb.exec_command(test_command)
        NT_data = stdout.readline()[1:-2].split(", ")
        NUC_consumer.release()
        motor_NT_data = float(NT_data[0])
        print("CAN LISTENER TEST - BBB NT Motor Data:", motor_NT_data)
        print("CAN LISTENER TEST - NUC ROS Motor Data:", aa_set1)
        CAN_listen_sem.release()
        try:
            assert math.isclose(motor_NT_data, aa_set1, rel_tol=0.1)
            print("PASS - CAN LISTENER TEST - MOTOR DATA PASSED")
        except AssertionError:
            print("ERROR - CAN LISTENER TEST - MOTOR DATA FAILED")
        print("-----")
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
        stdin, stdout, stderr = bbb.exec_command(candump_cmd)
        can_msg = stdout.readline()
        # we extract the last 8 characters this way because
        # the format of candump is weird
        VCAN_data = "".join(can_msg.strip().split()[-8:])
        CAN_producer.release()
        CAN_data = test_data_dict[test]['data']
        print("CAN TEST - BBB CAN Sensor Data", VCAN_data)
        print("CAN TEST - BBB VCAN Sensor Data:", CAN_data)
        try:
            assert(VCAN_data == CAN_data)
            print("CAN TEST - COMMAND SEND DATA PASSED")
        except AssertionError:
            print("ERROR - CAN TEST - CAN Data not received correctly")
        print("*****")
        stdin, stdout, stderr = bbb.exec_command(candump_cmd)
        can_motor_msg = stdout.readline()
        can_motor_msg = GET_RUDDER_PORT_ANGLE(can_motor_msg)
        print("CAN TEST - NUC ROS Motor Data:", aa_set1)
        print("CAN TEST - BBB CAN Motor Data:", can_motor_msg)
        try:
            assert math.isclose(can_motor_msg, aa_set1, rel_tol=0.1)
            print("PASS - CAN TEST - MOTOR DATA PASSED")
        except AssertionError:
            print("ERROR - CAN TEST - MOTOR DATA FAILED")
        print("*****")
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

    # for test in my_data:
    #     for topic in my_data[test]['rostopic']:
    #         print(topic)
    # print(my_data)
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

    print("=============================")
    print("Beginning CAN-BBB-NUC DP test")
    print("=============================")
    bbb_can_test_thread.start()
    bbb_canbus_listener_test_thread.start()
    nuc_eth_listener_test_thread.start()

    # # THIS SLEEP IS NEEDED SO THAT ALL THE THREADS CAN DO PROPER SETUP
    # time.sleep(SETUP_TIME)

    # Virtual CAN bus emulation loop
    for test in my_data:
        time.sleep(CAN_SEND_INTERVAL)
        CAN_listen_sem.acquire()
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
