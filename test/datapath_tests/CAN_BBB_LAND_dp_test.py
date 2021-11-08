"""
---- CAN-BBB-LAND datapath test ----

Based on CAN_BBB_NUC_dp_test.py

For the tests, run the following:

On the BBB:
~$ network_table_server &
~$ bbb_canbus_listener vcan0 &
~$ bbb_rockblock_listener ... (not sure, see bbb_satellite_listener)

On the Landserver:
~$ python3 land_satellite_listener -p [PORT] -e [ENDPOINT] -f [FREQUENCY] -u [TIME] -n [LOGIN USERNAME] -w [PASSWORD]
"""

import time
import argparse
import threading
from nt_connection.SSH_Connection import SSH_Connection
from nt_connection.CAN_to_URI_to_ROStopic import CAN_to_URI_to_ROStopic
from dummy_data import Test_Data, make_dummy_tests
from nt_connection import uri
from nt_connection.frame_parser import *
import re

CAN_producer = threading.Semaphore(0)
CAN_consumer = threading.Semaphore(1)
CAN_Listener_ready = threading.Semaphore(0)
CAN_listen_sem = threading.Semaphore(1)
read_nt_cmd = "python3 workspace/datapath_tests/test/datapath_tests/connection_values.py -u {}"

#Data sent from bbb_satallite_listener -> bb_can_bus_listener is sensor data (gps, sailencoder, wind, ...)

def test_bbb_can_listener(bbb, test_data_dict):
    print("BBB CAN Listener Test")
    for test in test_data_dict:
        ss_uri = ' '.join(test_data_dict[test]['uri']) #list to string
        test_command = read_nt_cmd.format(ss_uri)
        print("CAN LISTENER TEST - TEST COMMAND: ", test_command)
        CAN_producer.acquire() #Wait for main thread to fill NT (I think)
        _, stdout, __ = bbb.exec_command(test_command)
        NT_data = stdout.readline()[1:-2].split(", ") #string to list
        CAN_consumer.release()

        CAN_data = list(map(str, test_data_dict[test]["parsed_data"]))
        print("CAN LISTENER TEST - BBB NT data: ", NT_data)
        print("CAN LISTENER TEST - BBB CAN data: ", CAN_data)
        try:
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
    CAN_Listener_ready.release()
    # At this point, all sensor data should be sent to the network table

def test_land_satellite_listener(server, test_data_dict):
    print("Landserver Satellite Listener Test")
    # Use poll_network_table_data file to get data
    

    


def main():
    parser = argparse.ArgumentParser(description="CAN<>BBB<>LAND datapath test")
    parser.add_argument("-r", "--remote",
                        help="If you are running this \
                              on your local machine then set this flag",
                        action='store_true')
    parser.add_argument("-c", "--config",
                        help="Path to the config fill which has \
                              credentials for the server and BBB")
    args = parser.parse_args()

    

    ssh_connection = SSH_Connection(args.config)
    # ssh onto server even if already on server for simplicity
    server = ssh_connection.SSH_server() 
    bbb_client = None

    if args.remote:
        ssh_connection.SSH_BBB_tunnel()
        bbb_client = ssh_connection.bbb_connection
    else:
        ssh_connection.SSH_BBB()
        bbb_client = ssh_connection.bbb_connection

    # Just focus on GPS sensor data for now
    CAN_bus_filter = ["GPS_CAN_LAT_FRAME", "GPS_CAN_LON_FRAME", "GPS_CAN_OTHER_FRAME"] 
    test_data = make_dummy_tests(CAN_bus_filter, Test_Data, CAN_to_URI_to_ROStopic)

    bbb_can_listener_thread = threading.Thread( name="test_bbb_can_listener",
                                                target=test_bbb_can_listener,
                                                args=(bbb_client, test_data))
    land_satellite_listener_thread = threading.Thread(  name="test_land_satellite_listener",
                                                        target=test_land_satellite_listener,
                                                        args=(server, test_data))

if __name__ == "__main__":
    main()