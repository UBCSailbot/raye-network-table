"""
---- CAN-BBB-LAND datapath test ----

Based on CAN_BBB_NUC_dp_test.py

For the tests, run the following:

On the BBB:
~$ network_table_server &
~$ bbb_canbus_listener vcan0 &
~$ bbb_satellite_listener 5000 5000 5000 /dev/ttyS2 &

On the Landserver:
~$ python3 land_satellite_listener.py -p [PORT] -e [ENDPOINT] -f [FREQUENCY] -u [TIME] -n [LOGIN USERNAME] -w [PASSWORD]
ex.
~$ python3 land_satellite_listener.py -p 8000 -e http://rockblock.rock7.com/rockblock/MT -f 10 -u SEC -b 70.36.55.243
    -n [LOGIN USERNAME] -w [PASSWORD] -r 300234068129370 -i 212.71.235.32
"""

import time
import argparse
import threading
from nt_connection.SSH_Connection import SSH_Connection
from nt_connection.CAN_to_URI_to_ROStopic import CAN_to_URI_to_ROStopic
from dummy_data import Test_Data, make_dummy_tests
from nt_connection.uri import *
from nt_connection.frame_parser import *
import re

from python.CAN_to_URI_to_ROStopic import CAN_bus_sensor

CAN_producer = threading.Semaphore(0)
CAN_consumer = threading.Semaphore(1)
CAN_Listener_ready = threading.Semaphore(0)
CAN_listen_sem = threading.Semaphore(1)
CAN_SEND_INTERVAL = 15

read_nt_cmd = "python3 network-table/test/datapath_tests/connection_values.py -u {}"
candump_cmd = 'candump vcan0'

satellite_sensor_uris = [GPS_CAN_TIME, GPS_CAN_LAT, GPS_CAN_LON, GPS_CAN_GNDSPEED, GPS_CAN_TMG,
                         GPS_CAN_TRUE_HEADING, GPS_CAN_MAGVAR, GPS_CAN_VALID, GPS_CAN_VARWEST,
                         GPS_CAN_LATNORTH, GPS_CAN_LONWEST]
# Data sent from bbb_satallite_listener -> bb_can_bus_listener is sensor data (gps, sailencoder, wind, ...)


def test_bbb_can_listener(bbb, test_data_dict):
    print("BBB CAN Listener Test")
    for test in test_data_dict:
        ss_uri = ' '.join(test_data_dict[test]['uri'])  # list to string
        test_command = read_nt_cmd.format(ss_uri)
        print("CAN LISTENER TEST - TEST COMMAND: ", test_command)
        CAN_producer.acquire()  # Wait for main thread to fill NT (I think)
        _, stdout, __ = bbb.exec_command(test_command)
        NT_data = stdout.readline()[1:-2].split(", ")  # string to list
        CAN_consumer.release()
        CAN_data = list(map(str, test_data_dict[test]["parsed_data"]))
        print("CAN LISTENER TEST - BBB NT data: ", NT_data)
        print("CAN LISTENER TEST - BBB CAN data: ", CAN_data)
        try:
            NT_data = sorted(NT_data)
            CAN_data = sorted(CAN_data)
            for i in range(len(NT_data)):
                NT_i = float(re.sub(r'[^+\d.]', '', NT_data[i]))
                CAN_i = float(re.sub(r'[^+\d.]', '', CAN_data[i]))
                """
                Relative tolerance of 5% of the larger number.
                Absolute tolerance of 0.15 to help with smaller numbers.
                Realistically, rel_tol of 1% or lower would work for
                everything but latitude and longitude.
                """
                assert math.isclose(NT_i, CAN_i, rel_tol=0.05, abs_tol=0.15)
            print("CAN LISTENER TEST - DATA PASSED")
        except AssertionError:
            print("CAN LISTENER TEST - BBB CAN Bus Listener data not received correctly through virtual CAN")

        print("-----")
        CAN_listen_sem.release()


def test_bbb_can(bbb, test_data_dict):
    print("BBB Virtual CAN Test")
    for test in test_data_dict:
        CAN_consumer.acquire()
        _, stdout, __ = bbb.exec_command(candump_cmd)
        can_msg = stdout.readline()
        VCAN_data = "".join(can_msg.strip().split()[-8:])
        CAN_producer.release()
        CAN_data = test_data_dict[test]["data"]
        print("CAN TEST - BBB CAN Sensor Data: ", VCAN_data)
        print("CAN TEST - BBB VCAN Sensor Data: ", CAN_data)
        try:
            assert(VCAN_data == CAN_data)
            print("CAN TEST - COMMAND SEND DATA PASSED")
        except AssertionError:
            print("ERROR - CAN TEST - CAN Data not received correctly")
        print("*****")


def test_land_satellite_listener(server, test_data_dict):
    print("Landserver Satellite Listener Test")
    # Use poll_network_table_data file to get data
    for test in test_data_dict:
        ss_uri = " ".join(test_data_dict[test]["uri"])
        test_command = read_nt_cmd.format(ss_uri)
        print("LAND SATELLITE TEST - TEST COMMAND", test_command)
        _, stdout, __ = server.exec_command(test_command)
        LAND_data = stdout.readline()[1:-2].split(", ")
        CAN_data = list(map(str, test_data_dict[test]["parsed_data"]))
        print("LAND SATELLITE TEST - LAND Sensor Data: ", LAND_data)
        print("LAND SATELLITE TEST - BBB CAN Sensor Data: ", CAN_data)
        try:
            LAND_data = sorted(LAND_data)
            CAN_data = sorted(CAN_data)
            for i in range(len(LAND_data)):
                LAND_i = float(re.sub(r'[^-+\d.]', '', LAND_data[i]))
                CAN_i = float(re.sub(r'[^-+\d.]', '', CAN_data[i]))
                assert math.isclose(LAND_i, CAN_i, rel_tol=0.9)
            print("PASS - LAND SATELLITE TEST - SENSOR DATA PASSED")
        except AssertionError:
            print("ERROR - LAND SATELLITE TEST - LAND SENSOR DATA FAILED")


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

    # The commented line below is an example filter for just GPS data
    # CAN_bus_filter = ["GPS_CAN_DATE_FRAME", "GPS_CAN_LAT_FRAME", "GPS_CAN_LON_FRAME", "GPS_CAN_OTHER_FRAME"]
    CAN_bus_filter = CAN_bus_sensor
    test_data = make_dummy_tests(CAN_bus_filter, Test_Data, CAN_to_URI_to_ROStopic)

    bbb_can_test_thread = threading.Thread(name="test_bbb_can",
                                           target=test_bbb_can,
                                           args=(bbb_client, test_data))
    bbb_can_listener_thread = threading.Thread(name="test_bbb_can_listener",
                                               target=test_bbb_can_listener,
                                               args=(bbb_client, test_data))
    land_satellite_listener_thread = threading.Thread(name="test_land_satellite_listener",
                                                      target=test_land_satellite_listener,
                                                      args=(server, test_data))

    print("==============================")
    print("Beginning CAN-BBB-LAND DP Test")
    print("==============================")
    bbb_can_test_thread.start()
    bbb_can_listener_thread.start()
    # land_satellite_listener_thread.start()

    # Virtual CAN bus emulation loop
    for test in test_data:
        time.sleep(CAN_SEND_INTERVAL)
        CAN_listen_sem.acquire()
        print("MAIN - Sending command:",
              "cansend vcan0 " +
              '{:03x}'.format(test_data[test]['id'], "x")
              + "#" + test_data[test]['data'])
        bbb_client.exec_command("cansend vcan0 " +
                                '{:03x}'.format(test_data[test]['id']) +
                                "#" + test_data[test]['data'])

    bbb_can_test_thread.join()
    bbb_can_listener_thread.join()
    # land_satellite_listener_thread.join()
    ssh_connection.close_SSH()


if __name__ == "__main__":
    main()
