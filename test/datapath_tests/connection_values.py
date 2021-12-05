#!/usr/bin/env python3
'''
connection_values.py

This script is ran on CAN_BBB_NUC_dp_test.py file to verify that the CAN
Bus Listener code is correctly writing to the items on the Network Table.
This script actually does take a few seconds to run and the output is not
instant. There is a bit of buffer time added in the CAN_BBB_NUC dp test
script so that it can track the output of the item in the Network Table
before it updates again through another CAN send.

'''

from nt_connection.Connection import Connection
import generated_python.Node_pb2 as Node_pb2
import argparse


# Function that filters the uri
def isFloatValue(uri_item):
    return ('latitude' in uri_item or 'longitude' in uri_item
            or 'rudder' in uri_item or 'winch' in uri_item
            or 'gyroscope' in uri_item or 'ground_speed' in uri_item
            or 'magnetic_variation' in uri_item
            or 'track_made_good' in uri_item
            or 'true_heading' in uri_item)


# NOTE: we might get an error here that says the value hasn't been
# populated in the network table
def main():
    parser = argparse.ArgumentParser(description="DP test helper script")
    parser.add_argument("-u", "--uri",
                        nargs="*",
                        type=str,
                        help="URI to check from the network-table",
                        default=[])
    args = parser.parse_args()
    nt_connection = Connection()
    nt_connection.Connect()

    uri_list = args.uri
    node_list = []
    # iterate through a list of uris and append each NT value into another list
    for item in uri_list:
        node_container = nt_connection.getNodes([item])
        node = Node_pb2.Node()
        node.CopyFrom(node_container[item])

        # capture output of the data so we can read it from the DP test script
        # NOTE: The data types to print out are gotten from Help.py and
        # Value_pb2.py in the src/generated_python folder
        if 'status' in item or 'valid' in item or 'utc_timestamp' in item:
            node_list.append(node.value.string_data)
        elif isFloatValue(item):
            node_list.append(node.value.float_data)
        else:
            node_list.append(node.value.int_data)

    # Note: not using the bool_data for magnetic_variation_sense and other
    # items since we got rid of them

    nt_connection.Disconnect()
    print(node_list)
    return node_list


if __name__ == "__main__":
    main()
