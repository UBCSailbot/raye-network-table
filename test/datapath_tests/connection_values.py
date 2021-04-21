#!/usr/bin/env python3
'''
connection_values.py

This script is ran on CAN_BBB_NUC_dp_test.py file to verify that the CAN
Bus Listener code is correctly writing to the URIs on the Network Table.
This script actually does take a few seconds to run and the output is not
instant. There is a bit of buffer time added in the CAN_BBB_NUC dp test
script so that it can track the output of the URI in the Network Table
before it updates again through another CAN send.

'''

from nt_connection.Connection import Connection
import generated_python.Node_pb2 as Node_pb2
import argparse


# NOTE: we might get an error here that says the value hasn't been
# populated in the network table
def main():
    parser = argparse.ArgumentParser(description="DP test helper script")
    parser.add_argument("-u", "--uri",
                        help="URI to check from the network-table",
                        default=None)
    args = parser.parse_args()
    nt_connection = Connection()
    nt_connection.Connect()

    uri = args.uri
    node_container = nt_connection.getNodes([uri])

    node = Node_pb2.Node()
    node.CopyFrom(node_container[uri])

    # capture output of the data so we can read it from the DP test script
    # NOTE: The data types to print out are gotten from Help.py and
    # Value_pb2.py in the src/generated_python folder
    if 'status' in uri or 'valid' in uri or 'utc_timestamp' in uri:
        print(node.value.string_data)
    elif 'latitude' in uri or 'longitude' in uri:
        print(node.value.float_data)
    else:
        print(node.value.int_data)

    # Note: not using the bool_data for magnetic_variation_sense and other
    # URIs since we got rid of them
    nt_connection.Disconnect()
    return


if __name__ == "__main__":
    main()
